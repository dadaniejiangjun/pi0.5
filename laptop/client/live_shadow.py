"""Bounded, read-only PiPER camera+CAN to pi0.5 shadow inference.

Never constructs the PiPER SDK control interface and never sends CAN frames.
The policy output is recorded as a proxy prediction, not a robot command.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import threading
import time
from pathlib import Path

import numpy as np

from laptop.diagnostics.camera_identity import bind_camera_roles
from laptop.client.observation_schema import (
    PIPER_PROMPT,
    make_piper_proxy_warmup_observation,
    validate_piper_semantic_observation,
)
from laptop.control.contracts import PolicyPrediction, RobotStateSnapshot
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport

MAX_CAMERA_AGE_NS = 250_000_000
MAX_CAN_AGE_NS = 250_000_000
MAX_CAMERA_SKEW_NS = 20_000_000
MAX_SENSOR_SKEW_NS = 250_000_000
MAX_INITIAL_SENSOR_WAIT_NS = 2_000_000_000
REQUIRED_CAMERA_ROLES = ("base", "wrist")
REQUIRED_CAN_CHANNELS = ("status", "joint_12", "joint_34", "joint_56", "gripper")
# Compatibility exports preserve the old public imports.
from laptop.client.policy_client import (
    _SHA256_RE, _COMMIT_RE, ARTIFACT_IDENTITY_SCHEMA, ARTIFACT_LOCK_SCHEMA,
    DERIVED_MULTI_ARTIFACT_SCHEMA, DERIVED_MULTI_LOCK_SCHEMA, PROJECT_ROOT,
    DEFAULT_IDENTITY_LOCK, IDENTITY_HASH_FIELDS, validate_artifact_identity_lock,
    load_artifact_identity_lock, validate_policy_metadata, prewarm_policy,
)
from laptop.camera.worker import camera_loop as _camera_loop
from laptop.client.observation_schema import build_piper_observation
from laptop.piper.unit_conversion import joint_raw_to_nominal_rad, gripper_raw_to_nominal_m
from laptop.robot_client.feedback import sdk_feedback_snapshot









def build_policy_prediction(result: dict, metadata: dict, *, request_id: str,
                            request_sequence: int, observation_ns: int,
                            response_ns: int,
                            expected_identity_lock: dict | None = None) -> PolicyPrediction:
    """Validate server output and preserve its proxy-state identity."""
    validate_policy_metadata(metadata, expected_identity_lock)
    actions = np.asarray(result.get("actions"), dtype=np.float64)
    horizon = metadata.get("action_horizon")
    first_dt_s = metadata.get("first_target_dt_s")
    if actions.shape != (16, 7) or horizon != 16:
        raise ValueError("invalid proxy action shape or horizon metadata")
    if not np.isfinite(actions).all():
        raise ValueError("non-finite model prediction")
    if isinstance(first_dt_s, bool) or not isinstance(first_dt_s, (int, float)):
        raise ValueError("missing proxy target timing")
    if metadata.get("config_schema") == "pi05_derived_state_multi_episode_shadow_v1":
        checkpoint_hash = metadata["checkpoint_tree_sha256"]
        dataset_hash = metadata["dataset_sha256"]
        config_identity = (
            f"{metadata['config_schema']}#training-provenance-sha256:"
            f"{metadata['training_provenance_sha256']}#openpi:{metadata['openpi_commit']}"
        )
    else:
        checkpoint_hash = metadata["checkpoint_sha256"]
        dataset_hash = metadata["dataset_tree_sha256"]
        config_identity = (
            f"{metadata['config_schema']}#sha256:{metadata['policy_config_sha256']}"
            f"#pipeline-sha256:{metadata['policy_pipeline_code_sha256']}"
            f"#training-config-sha256:{metadata['training_config_code_sha256']}"
            f"#openpi:{metadata['openpi_commit']}"
        )
    return PolicyPrediction(
        request_id=request_id,
        request_sequence=request_sequence,
        observation_monotonic_ns=observation_ns,
        received_monotonic_ns=response_ns,
        actions=tuple(tuple(float(value) for value in row) for row in actions),
        target_dt_s=tuple(float(first_dt_s) * (index + 1)
                          for index in range(actions.shape[0])),
        checkpoint_identity=(
            f"{metadata['checkpoint']}#sha256:{checkpoint_hash}"),
        config_identity=config_identity,
        normalization_identity=(
            f"{metadata['normalization_identity']}#sha256:{metadata['normalization_sha256']}"),
        dataset_identity=(
            f"{metadata['dataset']}#tree-sha256:{dataset_hash}"),
    )


def segment_latency(result: dict, roundtrip_ms: float) -> dict:
    """Split OpenPI timings; the residual is not purely network latency."""
    model_ms = result.get("policy_timing", {}).get("infer_ms")
    server_ms = result.get("server_timing", {}).get("infer_ms")
    values = (model_ms, server_ms, roundtrip_ms)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("missing or invalid OpenPI timing")
    if model_ms > server_ms + 1 or server_ms > roundtrip_ms + 1:
        raise ValueError("inconsistent OpenPI timing hierarchy")
    return {
        "model_sampling_ms": model_ms,
        "server_non_model_ms": max(0.0, server_ms - model_ms),
        "client_server_residual_ms": max(0.0, roundtrip_ms - server_ms),
        "client_roundtrip_ms": roundtrip_ms,
        "residual_includes": "client serialization, localhost/tunnel/network, server serialization and scheduling",
    }


def initial_sensor_readiness(cameras: dict, feedback: dict) -> dict:
    """Describe whether one complete initial camera/CAN snapshot is available."""
    missing_camera_roles = []
    for role in REQUIRED_CAMERA_ROLES:
        sample = cameras.get(role) if isinstance(cameras, dict) else None
        if (not isinstance(sample, dict) or sample.get("frame") is None
                or type(sample.get("timestamp_ns")) is not int):
            missing_camera_roles.append(role)

    timestamps = feedback.get("timestamps_ns", {}) if isinstance(feedback, dict) else {}
    joints = feedback.get("joint_raw") if isinstance(feedback, dict) else None
    gripper = feedback.get("gripper_raw") if isinstance(feedback, dict) else None
    status = feedback.get("status") if isinstance(feedback, dict) else None
    missing_can_channels = []
    for channel in REQUIRED_CAN_CHANNELS:
        timestamp_ready = (isinstance(timestamps, dict)
                           and type(timestamps.get(channel)) is int)
        if channel == "status":
            payload_ready = isinstance(status, dict)
        elif channel == "joint_12":
            payload_ready = (isinstance(joints, (list, tuple)) and len(joints) >= 2
                             and all(type(value) is int for value in joints[:2]))
        elif channel == "joint_34":
            payload_ready = (isinstance(joints, (list, tuple)) and len(joints) >= 4
                             and all(type(value) is int for value in joints[2:4]))
        elif channel == "joint_56":
            payload_ready = (isinstance(joints, (list, tuple)) and len(joints) == 6
                             and all(type(value) is int for value in joints[4:6]))
        else:
            # Keep the raw feedback signed. A negative raw value is not evidence
            # that the CAN channel is missing, and its sign does not establish a
            # calibrated physical meaning or a control limit.
            payload_ready = type(gripper) is int
        if not timestamp_ready or not payload_ready:
            missing_can_channels.append(channel)

    return {
        "ready": not missing_camera_roles and not missing_can_channels,
        "missing_camera_roles": missing_camera_roles,
        "missing_can_channels": missing_can_channels,
    }


def new_can_diagnostics() -> dict:
    """Create counters that distinguish missing frames from decode failures."""
    return {
        "schema": "piper_can_receive_diagnostics_v1",
        "received_frames_total": 0,
        "received_frames_by_can_id": {},
        "error_frames": 0,
        "short_frames": 0,
        "short_usb_packets": 0,
        "feedback_frames_by_channel": {name: 0 for name in REQUIRED_CAN_CHANNELS},
        "decoded_feedback_frames_by_channel": {
            name: 0 for name in REQUIRED_CAN_CHANNELS
        },
        "decode_failures_by_channel": {name: 0 for name in REQUIRED_CAN_CHANNELS},
        "last_feedback_frame_monotonic_ns_by_channel": {},
        "last_decoded_feedback_monotonic_ns_by_channel": {},
    }


def record_can_frame_diagnostic(
    diagnostics: dict,
    *,
    arbitration_id: int,
    is_error_frame: bool,
    data_length: int,
    received_monotonic_ns: int,
    channel: str | None,
    decoded: bool | None = None,
) -> None:
    """Update read-only receive counters; caller owns synchronization."""
    if (type(arbitration_id) is not int or not 0 <= arbitration_id <= 0x1FFFFFFF
            or type(data_length) is not int or not 0 <= data_length <= 8):
        raise ValueError("CAN frame identity or length is invalid")
    if type(is_error_frame) is not bool:
        raise ValueError("is_error_frame must be a boolean")
    if type(received_monotonic_ns) is not int or received_monotonic_ns <= 0:
        raise ValueError("received_monotonic_ns must be a positive integer")
    if channel is not None and channel not in REQUIRED_CAN_CHANNELS:
        raise ValueError("unknown PiPER feedback channel")
    if decoded is not None and type(decoded) is not bool:
        raise ValueError("decoded must be a boolean or None")

    can_id = f"0x{arbitration_id:03X}"
    diagnostics["received_frames_total"] += 1
    id_counts = diagnostics["received_frames_by_can_id"]
    id_counts[can_id] = id_counts.get(can_id, 0) + 1
    if is_error_frame:
        diagnostics["error_frames"] += 1
    if data_length != 8:
        diagnostics["short_frames"] += 1
    if channel is None:
        return

    raw_counts = diagnostics["feedback_frames_by_channel"]
    raw_counts[channel] += 1
    diagnostics["last_feedback_frame_monotonic_ns_by_channel"][channel] = (
        received_monotonic_ns
    )
    if decoded is None:
        return
    if decoded:
        decoded_counts = diagnostics["decoded_feedback_frames_by_channel"]
        decoded_counts[channel] += 1
        diagnostics["last_decoded_feedback_monotonic_ns_by_channel"][channel] = (
            received_monotonic_ns
        )
    else:
        diagnostics["decode_failures_by_channel"][channel] += 1


def select_observation(cameras: dict, feedback: dict, now_ns: int) -> tuple[dict, dict]:
    """Reject incomplete/stale observations before policy inference."""
    required = ("status", "joint_12", "joint_34", "joint_56", "gripper")
    if any(key not in feedback.get("timestamps_ns", {}) for key in required):
        raise ValueError("missing CAN feedback channel")
    if any(role not in cameras for role in ("base", "wrist")):
        raise ValueError("missing camera frame")
    times = [cameras[role]["timestamp_ns"] for role in ("base", "wrist")]
    can_times = [feedback["timestamps_ns"][key] for key in required]
    if any(now_ns < timestamp or now_ns - timestamp > MAX_CAMERA_AGE_NS for timestamp in times):
        raise ValueError("stale camera frame")
    if any(now_ns < timestamp or now_ns - timestamp > MAX_CAN_AGE_NS for timestamp in can_times):
        raise ValueError("stale CAN feedback")
    if abs(times[0] - times[1]) > MAX_CAMERA_SKEW_NS:
        raise ValueError("Base/Wrist host-after-read timestamp skew exceeds 20 ms")
    if max(times + can_times) - min(times + can_times) > MAX_SENSOR_SKEW_NS:
        raise ValueError("camera/CAN skew exceeds bound")
    joints = feedback.get("joint_raw")
    gripper = feedback.get("gripper_raw")
    if (not isinstance(joints, (list, tuple)) or len(joints) != 6
            or any(type(value) is not int for value in joints)
            or type(gripper) is not int):
        raise ValueError("invalid joint/gripper feedback")
    import cv2
    observation = build_piper_observation(
        cv2.cvtColor(cameras["base"]["frame"], cv2.COLOR_BGR2RGB),
        cv2.cvtColor(cameras["wrist"]["frame"], cv2.COLOR_BGR2RGB),
        [joint_raw_to_nominal_rad(value) for value in joints]
        + [gripper_raw_to_nominal_m(gripper)], PIPER_PROMPT,
    )
    validate_piper_semantic_observation(observation)
    return observation, {
        "base_timestamp_ns": times[0], "wrist_timestamp_ns": times[1],
        "can_timestamps_ns": dict(feedback["timestamps_ns"]),
        "observation_built_ns": now_ns,
        "camera_skew_ms": abs(times[0] - times[1]) / 1e6,
        "oldest_sensor_age_ms": (now_ns - min(times + can_times)) / 1e6,
    }


def state_snapshot_from_live_observation(
    observation: dict,
    timing: dict,
    feedback: dict,
    *,
    sequence: int,
    connected: bool,
    units_verified: bool = False,
) -> RobotStateSnapshot:
    """Preserve feedback timestamps when binding an observation to robot state.

    The oldest CAN channel timestamp is used for conservative freshness checks;
    the latest is recorded as local receipt. Unit verification stays false by
    default and must come from a separate physical calibration record.
    """
    if not isinstance(observation, dict):
        raise TypeError("observation must be a dictionary")
    state = np.asarray(observation.get("state"))
    if state.shape != (7,) or state.dtype.kind not in "iuf":
        raise ValueError("observation state must contain seven numeric values")
    state = state.astype(np.float64)
    if not np.isfinite(state).all():
        raise ValueError("observation state must contain finite values")
    if not isinstance(timing, dict):
        raise TypeError("timing must be a dictionary")
    timestamps = timing.get("can_timestamps_ns")
    if not isinstance(timestamps, dict):
        raise ValueError("timing must preserve CAN channel timestamps")
    if any(key not in timestamps for key in REQUIRED_CAN_CHANNELS):
        raise ValueError("timing is missing a required CAN channel timestamp")
    can_times = tuple(timestamps[key] for key in REQUIRED_CAN_CHANNELS)
    if any(type(value) is not int or value <= 0 for value in can_times):
        raise ValueError("CAN channel timestamps must be positive monotonic integers")
    observation_ns = timing.get("observation_built_ns")
    if type(observation_ns) is not int or observation_ns < max(can_times):
        raise ValueError("observation time precedes a CAN feedback channel")
    if type(connected) is not bool or type(units_verified) is not bool:
        raise ValueError("connected and units_verified must be booleans")
    status = feedback.get("status") if isinstance(feedback, dict) else None
    status_ok = isinstance(status, dict) and status.get("err_code") in (0, "0")
    return RobotStateSnapshot(
        sequence=sequence,
        captured_monotonic_ns=min(can_times),
        received_monotonic_ns=max(can_times),
        joint_position_rad=tuple(float(value) for value in state[:6]),
        gripper_position_m=float(state[6]),
        status_ok=status_ok,
        connected=connected,
        units_verified=units_verified,
    )




def _can_loop(safety_bridge: SafetyBridge, shared: dict, lock: threading.Lock,
              stop: threading.Event, errors: list[str]) -> None:
    import can
    from piper_sdk.piper_msgs.msg_v2 import CanIDPiper, PiperMessage
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

    ids = {
        CanIDPiper.ARM_STATUS_FEEDBACK.value: "status",
        CanIDPiper.ARM_JOINT_FEEDBACK_12.value: "joint_12",
        CanIDPiper.ARM_JOINT_FEEDBACK_34.value: "joint_34",
        CanIDPiper.ARM_JOINT_FEEDBACK_56.value: "joint_56",
        CanIDPiper.ARM_GRIPPER_FEEDBACK.value: "gripper",
    }
    try:
        safety_bridge.start_read_only()
        parser, message = C_PiperParserV2(), PiperMessage()
        while not stop.is_set():
            frame = safety_bridge.receive_frame(timeout_ms=100)
            if frame is None:
                continue
            channel = ids.get(frame.arbitration_id)
            if frame.is_error_frame or len(frame.data) != 8:
                with lock:
                    record_can_frame_diagnostic(
                        shared["can_diagnostics"],
                        arbitration_id=frame.arbitration_id,
                        is_error_frame=frame.is_error_frame,
                        data_length=len(frame.data),
                        received_monotonic_ns=frame.received_monotonic_ns,
                        channel=channel,
                    )
                continue
            if channel is None:
                with lock:
                    record_can_frame_diagnostic(
                        shared["can_diagnostics"],
                        arbitration_id=frame.arbitration_id,
                        is_error_frame=frame.is_error_frame,
                        data_length=len(frame.data),
                        received_monotonic_ns=frame.received_monotonic_ns,
                        channel=None,
                    )
                continue
            stamp = frame.received_monotonic_ns
            parsed = can.Message(arbitration_id=frame.arbitration_id,
                                 data=frame.data,
                                 is_extended_id=frame.is_extended_id,
                                 timestamp=stamp / 1e9)
            decoded = bool(parser.DecodeMessage(parsed, message))
            with lock:
                record_can_frame_diagnostic(
                    shared["can_diagnostics"],
                    arbitration_id=frame.arbitration_id,
                    is_error_frame=frame.is_error_frame,
                    data_length=len(frame.data),
                    received_monotonic_ns=stamp,
                    channel=channel,
                    decoded=decoded,
                )
            if not decoded:
                continue
            with lock:
                shared.setdefault("timestamps_ns", {})[channel] = stamp
                decoded_fields = sdk_feedback_snapshot(message)
                shared["joint_raw"] = decoded_fields["joint_raw"]
                shared["gripper_raw"] = decoded_fields["gripper_raw"]["angle_or_stroke"]
                shared["status"] = decoded_fields["status"]
    except Exception as exc:
        errors.append(f"CAN: {type(exc).__name__}: {exc}")
        stop.set()


def _worker(duration_s: float, port: int, base_index: int, wrist_index: int,
            expected_identity_lock: dict, camera_identity: dict) -> dict:
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    # Connect and validate the remote policy before opening hardware.
    client = WebsocketClientPolicy(host="127.0.0.1", port=port)
    metadata = client.get_server_metadata()
    validate_policy_metadata(metadata, expected_identity_lock)
    warmup = prewarm_policy(client)
    transport = GsUsbPiperCanTransport()
    safety_bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
    )
    cameras: dict = {}
    feedback: dict = {"can_diagnostics": new_can_diagnostics()}
    camera_lock, can_lock = threading.Lock(), threading.Lock()
    stop = threading.Event()
    errors: list[str] = []
    threads = [
        threading.Thread(target=_camera_loop, args=(base_index, "base", cameras, camera_lock, stop, errors)),
        threading.Thread(target=_camera_loop, args=(wrist_index, "wrist", cameras, camera_lock, stop, errors)),
        threading.Thread(target=_can_loop,
                         args=(safety_bridge, feedback, can_lock, stop, errors)),
    ]
    start_ns = time.perf_counter_ns()
    deadline_ns = start_ns + int(duration_s * 1e9)
    # Stop hardware reads on schedule even if a policy request is still blocked.
    read_timer = threading.Timer(duration_s, stop.set)
    read_timer.daemon = True
    read_timer.start()
    for thread in threads:
        thread.start()
    predictions = []
    rejects = {}
    startup_started_ns = time.perf_counter_ns()
    startup_deadline_ns = min(deadline_ns, startup_started_ns + MAX_INITIAL_SENSOR_WAIT_NS)
    startup = {**initial_sensor_readiness({}, {}),
               "status": "WAITING", "wait_ms": 0.0}
    try:
        while time.perf_counter_ns() < startup_deadline_ns and not stop.is_set():
            with camera_lock:
                camera_snapshot = dict(cameras)
            with can_lock:
                feedback_snapshot = {**feedback,
                                     "timestamps_ns": dict(feedback.get("timestamps_ns", {}))}
            startup = initial_sensor_readiness(camera_snapshot, feedback_snapshot)
            startup["wait_ms"] = (time.perf_counter_ns() - startup_started_ns) / 1e6
            if startup["ready"]:
                startup["status"] = "READY"
                break
            time.sleep(0.01)
        else:
            startup["status"] = "SENSOR_ERROR" if errors else "TIMEOUT"
        if not startup["ready"]:
            # Startup emptiness is reported separately, not counted as runtime dropout.
            startup["wait_ms"] = (time.perf_counter_ns() - startup_started_ns) / 1e6

        while time.perf_counter_ns() < deadline_ns and not stop.is_set():
            if not startup["ready"]:
                break
            with camera_lock:
                camera_snapshot = dict(cameras)
            with can_lock:
                feedback_snapshot = {**feedback, "timestamps_ns": dict(feedback.get("timestamps_ns", {}))}
            now_ns = time.perf_counter_ns()
            try:
                observation, timing = select_observation(camera_snapshot, feedback_snapshot, now_ns)
            except ValueError as exc:
                reason = str(exc)
                rejects[reason] = rejects.get(reason, 0) + 1
                time.sleep(0.05)
                continue
            request_ns = time.perf_counter_ns()
            result = client.infer(observation)
            response_ns = time.perf_counter_ns()
            typed_prediction = build_policy_prediction(
                result, metadata,
                request_id=f"shadow-{len(predictions)}-{request_ns}",
                request_sequence=len(predictions),
                observation_ns=timing["observation_built_ns"],
                response_ns=response_ns,
                expected_identity_lock=expected_identity_lock,
            )
            state_snapshot = state_snapshot_from_live_observation(
                observation,
                timing,
                feedback_snapshot,
                sequence=typed_prediction.request_sequence,
                connected=safety_bridge.transport.connected,
            )
            # Prediction diagnostics must not latch the hardware receive owner.
            # Every inspection remains reject-only on an independent fake transport.
            from laptop.transport.can_transport import FakeCanTransport
            inspector = SafetyBridge(mode=RuntimeMode.POLICY_REJECT_ONLY,
                                     transport=FakeCanTransport(),
                                     limits=UNVERIFIED_ANALYSIS_LIMITS)
            inspector.record_heartbeat(max(1, response_ns - 1))
            safety_decision = inspector.inspect_prediction(
                snapshot=state_snapshot,
                prediction=typed_prediction,
                now_monotonic_ns=response_ns,
            )
            latency = segment_latency(result, (response_ns - request_ns) / 1e6)
            predictions.append({**timing, "request_ns": request_ns,
                                "response_ns": response_ns,
                                "request_id": typed_prediction.request_id,
                                "request_sequence": typed_prediction.request_sequence,
                                "target_dt_s": list(typed_prediction.target_dt_s),
                                "action_semantics": typed_prediction.action_semantics,
                                "checkpoint_identity": typed_prediction.checkpoint_identity,
                                "config_identity": typed_prediction.config_identity,
                                "normalization_identity": typed_prediction.normalization_identity,
                                "dataset_identity": typed_prediction.dataset_identity,
                                "safety_decision": {
                                    "mode": safety_decision.mode.value,
                                    "hardware_control_allowed": safety_decision.hardware_control_allowed,
                                    "transport_send_attempted": safety_decision.transport_send_attempted,
                                    "reasons": list(safety_decision.reasons),
                                    "audit_event": safety_decision.audit_event(),
                                },
                                "inference_latency_ms": (response_ns - request_ns) / 1e6,
                                "latency_segments": latency,
                                "robot_status": dict(feedback_snapshot["status"]),
                                "state": observation["state"].tolist(),
                                "first_predicted_proxy": list(typed_prediction.actions[0])})
            if len(predictions) >= 3:
                break
    finally:
        stop.set()
        read_timer.cancel()
        for thread in threads:
            thread.join(timeout=0.5)
        safety_bridge.close()
    can_diagnostics = json.loads(
        json.dumps(feedback.get("can_diagnostics", new_can_diagnostics()))
    )
    can_diagnostics["short_usb_packets"] = transport.short_usb_packets
    status = ("LIVE_SHADOW_CONTINUOUS_PASS" if len(predictions) >= 2 and not errors
              else "SINGLE_PREDICTION_ONLY" if len(predictions) == 1 and not errors
              else "PARTIAL")
    return {"status": status,
            "predictions": predictions, "rejected_snapshot_counts": rejects,
            "startup_readiness": startup,
            "can_diagnostics": can_diagnostics,
            "sensor_errors": errors, "duration_s": (time.perf_counter_ns() - start_ns) / 1e9,
            "policy_warmup": warmup,
            "checkpoint": metadata.get("checkpoint"),
            "artifact_identity_verified": True,
            "artifact_identity_schema": ARTIFACT_IDENTITY_SCHEMA,
            "camera_identity": camera_identity,
            "robot_command_sent": False, "can_data_tx_calls": 0,
            "safety_bridge_send_attempts": transport.send_attempts,
            "control_enabled": False, "units_hardware_verified": False}


def run(duration_s: float, port: int, base_index: int, wrist_index: int,
        *, approved: bool = False,
        expected_identity_lock: dict | None = None,
        base_device_id: str | None = None,
        wrist_device_id: str | None = None,
        roles_visually_confirmed: bool = False) -> dict:
    if not approved:
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "robot_command_sent": False}
    if expected_identity_lock is None:
        return {"status": "BLOCKED_ARTIFACT_IDENTITY_LOCK_REQUIRED",
                "robot_command_sent": False, "can_data_tx_calls": 0,
                "control_enabled": False}
    try:
        expected_identity_lock = validate_artifact_identity_lock(expected_identity_lock)
    except (ValueError, PermissionError) as exc:
        return {"status": "BLOCKED_ARTIFACT_IDENTITY_LOCK_INVALID",
                "error": str(exc), "robot_command_sent": False,
                "can_data_tx_calls": 0, "control_enabled": False}
    if not 0 < duration_s <= 20 or base_index == wrist_index or not 1 <= port <= 65535:
        raise ValueError("invalid bounded shadow configuration")
    if not base_device_id or not wrist_device_id or not roles_visually_confirmed:
        return {"status": "BLOCKED_CAMERA_IDENTITY_UNVERIFIED",
                "robot_command_sent": False, "can_data_tx_calls": 0,
                "camera_opened": False, "can_opened": False,
                "control_enabled": False}
    try:
        camera_identity = bind_camera_roles(
            base_index, wrist_index,
            base_device_id=base_device_id,
            wrist_device_id=wrist_device_id,
            roles_visually_confirmed=roles_visually_confirmed,
        )
    except Exception as exc:
        return {"status": "BLOCKED_CAMERA_IDENTITY_CHECK_FAILED",
                "error": f"{type(exc).__name__}: {exc}",
                "robot_command_sent": False, "can_data_tx_calls": 0,
                "camera_opened": False, "can_opened": False,
                "control_enabled": False}
    from laptop.diagnostics.bounded_process import run_bounded
    result = run_bounded(
        _worker,
        args=(duration_s, port, base_index, wrist_index,
              expected_identity_lock, camera_identity),
        timeout_s=duration_s + 10,
    )
    if result["status"] == "COMPLETE":
        return result["result"]
    return {"status": result["status"], "error": result["error"],
            "robot_command_sent": False, "can_data_tx_calls": 0,
            "control_enabled": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--base-index", type=int, required=True)
    parser.add_argument("--wrist-index", type=int, required=True)
    parser.add_argument("--base-device-id", required=True,
                        help="Windows camera InstanceId for the visually confirmed Base view")
    parser.add_argument("--wrist-device-id", required=True,
                        help="Windows camera InstanceId for the visually confirmed Wrist view")
    parser.add_argument("--roles-visually-confirmed", action="store_true",
                        help="confirm each selected camera's live image matches its declared role")
    parser.add_argument("--output", type=Path, help="exclusive-create local JSON report; no raw images")
    parser.add_argument("--read-only-shadow-approved", action="store_true")
    parser.add_argument("--artifact-identity-lock", type=Path,
                        default=DEFAULT_IDENTITY_LOCK,
                        help="local expected server artifact identity; required before device reads")
    args = parser.parse_args()
    identity_lock = None
    if args.read_only_shadow_approved:
        try:
            identity_lock = load_artifact_identity_lock(args.artifact_identity_lock)
        except (OSError, ValueError, PermissionError) as exc:
            result = {"status": "BLOCKED_ARTIFACT_IDENTITY_LOCK_INVALID",
                      "error": str(exc), "robot_command_sent": False,
                      "can_data_tx_calls": 0, "control_enabled": False}
        else:
            result = run(args.duration, args.port, args.base_index,
                         args.wrist_index, approved=True,
                         expected_identity_lock=identity_lock,
                         base_device_id=args.base_device_id,
                         wrist_device_id=args.wrist_device_id,
                         roles_visually_confirmed=args.roles_visually_confirmed)
    else:
        result = run(args.duration, args.port, args.base_index,
                     args.wrist_index, approved=False)
    if args.output is not None and args.read_only_shadow_approved:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
