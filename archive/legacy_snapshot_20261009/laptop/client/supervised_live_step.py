"""Run one batch or a bounded, explicitly approved short PiPER policy session.

The default invocation is inert. This entrypoint remains separate from
``live_shadow`` and requires an artifact lock, a complete site profile, camera
identity checks, and exact session confirmation before opening devices. The
short loop is limited to three batches and the existing one-step displacement
envelope. It is software plumbing only; a profile is still an operator declaration.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import threading
import time

from laptop.client.observation_schema import PIPER_PROMPT
from laptop.diagnostics.camera_identity import bind_camera_roles
from laptop.diagnostics.bounded_process import run_bounded
from laptop.robot_client.supervised_policy_session import (
    OneStepOperatorGrantTicket,
    ShortLoopOperatorGrant,
    ShortLoopOperatorGrantTicket,
    SHORT_LOOP_MAX_STEPS,
    request_short_loop_operator_grant,
    request_one_step_operator_grant,
)
from laptop.robot_client.supervised_policy_step import (
    prepare_supervised_policy_step,
    send_prepared_supervised_policy_step,
)
from laptop.robot_client.supervised_site_profile import (
    SupervisedSiteProfile,
    load_supervised_site_profile,
)
from laptop.safety.bridge import SafetyBridge
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


WORKER_TIMEOUT_S = 45
SENSOR_STARTUP_TIMEOUT_NS = 2_000_000_000
POST_SEND_FEEDBACK_TIMEOUT_NS = 750_000_000


def live_robot_status_ready(status: object) -> bool:
    """Require the controller to already be in normal CAN-control mode."""
    if not isinstance(status, dict):
        return False
    return (
        status.get("ctrl_mode") == "CAN_CTRL(0x1)"
        and status.get("arm_status") == "NORMAL(0x0)"
        and status.get("err_code") in (0, "0")
    )


def post_send_feedback_outcome(feedback_after_send: dict | None) -> dict[str, object]:
    """Report adapter acceptance separately from fresh robot feedback.

    A successful USB adapter write does not prove that PiPER received or
    executed the target. If no fresh feedback follows the batch, the caller
    must stop and inspect read-only feedback before considering another send.
    """
    if feedback_after_send is None:
        return {
            "status": "POSITION_BATCH_ACCEPTED_NO_FRESH_FEEDBACK_STOP_DO_NOT_RETRY",
            "post_send_feedback_observed": False,
            "retry_prohibited_until_read_only_feedback": True,
            "robot_motion_confirmed": False,
        }
    return {
        "status": "POSITION_BATCH_ACCEPTED_FRESH_FEEDBACK_OBSERVED",
        "post_send_feedback_observed": True,
        "retry_prohibited_until_read_only_feedback": False,
        "robot_motion_confirmed": False,
    }


def _metadata_identities(metadata: dict, lock: dict) -> dict[str, str]:
    """Use the same checked identity construction as live shadow predictions."""
    import numpy as np

    from laptop.client.live_shadow import build_policy_prediction

    now_ns = time.perf_counter_ns()
    preview_prediction = build_policy_prediction(
        {"actions": np.zeros((16, 7), dtype=np.float64)},
        metadata,
        request_id=f"supervised-consent-preview-{now_ns}",
        request_sequence=0,
        observation_ns=now_ns - 1,
        response_ns=now_ns,
        expected_identity_lock=lock,
    )
    return {
        "checkpoint_identity": preview_prediction.checkpoint_identity,
        "config_identity": preview_prediction.config_identity,
        "normalization_identity": preview_prediction.normalization_identity,
        "dataset_identity": preview_prediction.dataset_identity,
    }


def observation_follows_write(timing: object, write_monotonic_ns: int) -> bool:
    """Require both camera frames and all five CAN channels after a write."""
    if not isinstance(timing, dict) or type(write_monotonic_ns) is not int:
        return False
    can_times = timing.get("can_timestamps_ns")
    if not isinstance(can_times, dict):
        return False
    required = ("status", "joint_12", "joint_34", "joint_56", "gripper")
    times = [timing.get("base_timestamp_ns"), timing.get("wrist_timestamp_ns")]
    times.extend(can_times.get(name) for name in required)
    return len(times) == 7 and all(
        type(timestamp) is int and timestamp > write_monotonic_ns
        for timestamp in times
    )


def _worker(duration_s: float, port: int, base_index: int, wrist_index: int,
            expected_identity_lock: dict, camera_identity: dict,
            profile: SupervisedSiteProfile,
            grant_ticket: OneStepOperatorGrantTicket | ShortLoopOperatorGrantTicket) -> dict:
    """Run one batch or one explicitly approved, feedback-coupled short loop."""
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    from laptop.client.live_shadow import (
        # camera worker is imported from the public camera module below.
        _can_loop,
        build_policy_prediction,
        initial_sensor_readiness,
        new_can_diagnostics,
        select_observation,
        state_snapshot_from_live_observation,
        validate_policy_metadata,
    )
    from laptop.camera.worker import camera_loop
    grant = grant_ticket.materialize()
    short_loop = isinstance(grant, ShortLoopOperatorGrant)
    step_budget = grant.max_steps if short_loop else 1
    if grant.site_gates != profile.p12_evidence or grant.limits != profile.limits:
        return {"status": "BLOCKED_PROFILE_CHANGED_AFTER_APPROVAL",
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    if grant.gripper_effort_raw != profile.gripper_effort_raw:
        return {"status": "BLOCKED_GRIPPER_EFFORT_CHANGED_AFTER_APPROVAL",
                "position_send_attempts": 0, "robot_motion_confirmed": False}

    client = WebsocketClientPolicy(host="127.0.0.1", port=port)
    metadata = client.get_server_metadata()
    validate_policy_metadata(metadata, expected_identity_lock)
    identities = _metadata_identities(metadata, expected_identity_lock)
    if any(getattr(grant, field) != value for field, value in identities.items()):
        return {"status": "BLOCKED_POLICY_IDENTITY_CHANGED_AFTER_APPROVAL",
                "position_send_attempts": 0, "robot_motion_confirmed": False}

    transport = GsUsbPiperCanTransport()
    bridge = SafetyBridge(
        mode=RuntimeMode.SUPERVISED_POLICY_STEP,
        transport=transport,
        limits=profile.limits,
        supervised_position_tx_enabled=True,
    )
    cameras: dict = {}
    feedback: dict = {"can_diagnostics": new_can_diagnostics()}
    camera_lock, can_lock = threading.Lock(), threading.Lock()
    stop = threading.Event()
    errors: list[str] = []
    threads = [
        threading.Thread(target=camera_loop,
                         args=(base_index, "base", cameras, camera_lock, stop, errors)),
        threading.Thread(target=camera_loop,
                         args=(wrist_index, "wrist", cameras, camera_lock, stop, errors)),
        threading.Thread(target=_can_loop,
                         args=(bridge, feedback, can_lock, stop, errors)),
    ]
    started_ns = time.perf_counter_ns()
    if not 0 < duration_s <= 20:
        bridge.close()
        return {"status": "BLOCKED_INVALID_WORKER_DURATION",
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    deadline_ns = started_ns + int(duration_s * 1e9)
    timer = threading.Timer(duration_s, stop.set)
    timer.daemon = True
    current_receipt = None
    attempts_before_step = 0
    latest_write_ns = None
    step_results: list[dict] = []
    try:
        timer.start()
        for thread in threads:
            thread.start()

        startup_started_ns = time.perf_counter_ns()
        startup_deadline_ns = min(
            deadline_ns, startup_started_ns + SENSOR_STARTUP_TIMEOUT_NS,
        )
        readiness = {"ready": False, "missing_camera_roles": ["base", "wrist"],
                     "missing_can_channels": [], "status": "WAITING"}
        while time.perf_counter_ns() < startup_deadline_ns and not stop.is_set():
            with camera_lock:
                camera_snapshot = dict(cameras)
            with can_lock:
                feedback_snapshot = {
                    **feedback,
                    "timestamps_ns": dict(feedback.get("timestamps_ns", {})),
                }
            readiness = initial_sensor_readiness(camera_snapshot, feedback_snapshot)
            if readiness["ready"]:
                readiness["status"] = "READY"
                break
            time.sleep(0.01)
        if not readiness["ready"]:
            readiness["status"] = "SENSOR_ERROR" if errors else "TIMEOUT"
            return {"status": "BLOCKED_SENSOR_READINESS",
                    "startup_readiness": readiness, "sensor_errors": list(errors),
                    "position_send_attempts": 0, "robot_motion_confirmed": False}

        for sequence in range(step_budget):
            current_receipt = None
            if (time.perf_counter_ns() > grant.expires_at_monotonic_ns
                    or time.perf_counter_ns() >= deadline_ns):
                return {
                    "status": "BLOCKED_OPERATOR_GRANT_OR_WORKER_DEADLINE",
                    "steps": step_results,
                    "position_send_attempts": transport.position_send_attempts,
                    "position_frames_accepted": transport.position_frames_accepted,
                    "robot_motion_confirmed": False,
                }
            attempts_before_step = transport.position_send_attempts
            step_deadline_ns = min(deadline_ns, time.perf_counter_ns()
                                   + SENSOR_STARTUP_TIMEOUT_NS)
            observation = None
            timing = None
            feedback_snapshot = {}
            while time.perf_counter_ns() < step_deadline_ns and not stop.is_set():
                if time.perf_counter_ns() > grant.expires_at_monotonic_ns:
                    break
                with camera_lock:
                    camera_snapshot = dict(cameras)
                with can_lock:
                    feedback_snapshot = {
                        **feedback,
                        "timestamps_ns": dict(feedback.get("timestamps_ns", {})),
                    }
                try:
                    candidate_observation, candidate_timing = select_observation(
                        camera_snapshot, feedback_snapshot, time.perf_counter_ns(),
                    )
                    if (latest_write_ns is not None
                            and not observation_follows_write(candidate_timing, latest_write_ns)):
                        raise ValueError("next observation does not follow previous position batch")
                    observation, timing = candidate_observation, candidate_timing
                    break
                except ValueError:
                    time.sleep(0.01)
            if observation is None or timing is None:
                return {
                    "status": "BLOCKED_NO_FRESH_POST_WRITE_OBSERVATION" if latest_write_ns
                    else "BLOCKED_NO_FRESH_OBSERVATION",
                    "steps": step_results,
                    "sensor_errors": list(errors),
                    "position_send_attempts": transport.position_send_attempts,
                    "position_frames_accepted": transport.position_frames_accepted,
                    "robot_motion_confirmed": False,
                    "retry_prohibited_until_read_only_feedback": bool(latest_write_ns),
                }

            if not live_robot_status_ready(feedback_snapshot.get("status")):
                return {
                    "status": "BLOCKED_ROBOT_STATUS_NOT_READY",
                    "robot_status": feedback_snapshot.get("status"),
                    "steps": step_results,
                    "position_send_attempts": transport.position_send_attempts,
                    "position_frames_accepted": transport.position_frames_accepted,
                    "robot_motion_confirmed": False,
                }

            bridge.record_heartbeat(time.perf_counter_ns())
            inference_started_ns = time.perf_counter_ns()
            result = client.infer(observation)
            response_ns = time.perf_counter_ns()
            prediction = build_policy_prediction(
                result,
                metadata,
                request_id=f"supervised-{sequence}-{inference_started_ns}",
                request_sequence=sequence,
                observation_ns=timing["observation_built_ns"],
                response_ns=response_ns,
                expected_identity_lock=expected_identity_lock,
            )
            snapshot = state_snapshot_from_live_observation(
                observation,
                timing,
                feedback_snapshot,
                sequence=sequence,
                connected=bridge.transport.connected,
                units_verified=True,
            )
            prepared = prepare_supervised_policy_step(
                prediction,
                snapshot=snapshot,
                evidence=profile.p12_evidence,
                limits=profile.limits,
                gripper_effort_raw=profile.gripper_effort_raw,
                timing=profile.limits.timing_contract,
            )
            authorization = grant.authorize(
                prediction,
                prepared,
                active_task_description=PIPER_PROMPT,
                inference_started_monotonic_ns=inference_started_ns,
                limits=profile.limits,
            )
            bridge.record_heartbeat(time.perf_counter_ns())
            current_receipt = send_prepared_supervised_policy_step(
                prepared,
                bridge=bridge,
                authorization=authorization,
            )

            latest_write_ns = current_receipt.writes[-1].adapter_accepted_monotonic_ns
            feedback_after_send = None
            after_snapshot = None
            feedback_deadline_ns = min(
                deadline_ns, time.perf_counter_ns() + POST_SEND_FEEDBACK_TIMEOUT_NS,
            )
            while time.perf_counter_ns() < feedback_deadline_ns and not stop.is_set():
                with camera_lock:
                    after_cameras = dict(cameras)
                with can_lock:
                    after_feedback = {
                        **feedback,
                        "timestamps_ns": dict(feedback.get("timestamps_ns", {})),
                    }
                try:
                    after_observation, after_timing = select_observation(
                        after_cameras, after_feedback, time.perf_counter_ns(),
                    )
                    if not observation_follows_write(after_timing, latest_write_ns):
                        raise ValueError("post-send observation predates the position batch")
                    after_snapshot = state_snapshot_from_live_observation(
                        after_observation,
                        after_timing,
                        after_feedback,
                        sequence=sequence + 1,
                        connected=bridge.transport.connected,
                        units_verified=True,
                    )
                    feedback_after_send = asdict(after_snapshot)
                    break
                except ValueError:
                    pass
                time.sleep(0.01)

            step_outcome = post_send_feedback_outcome(feedback_after_send)
            step_result = {
                **step_outcome,
                "source_state_sequence": snapshot.sequence,
                "selected_action_index": prepared.preview.action_index,
                "target_monotonic_ns": prepared.preview.target_monotonic_ns,
                "target_nominal": list(prepared.proposal.target),
                "frame_writes": [
                    {"can_id": f"0x{write.arbitration_id:03X}",
                     "data_hex": write.data.hex(),
                     "adapter_accepted_monotonic_ns": write.adapter_accepted_monotonic_ns}
                    for write in current_receipt.writes
                ],
                "feedback_after_send": feedback_after_send,
            }
            step_results.append(step_result)

            if short_loop:
                if after_snapshot is None:
                    return {
                        "status": step_outcome["status"],
                        "task_description": PIPER_PROMPT,
                        "policy_identities": identities,
                        "steps": step_results,
                        "position_send_attempts": transport.position_send_attempts,
                        "position_frames_accepted": transport.position_frames_accepted,
                        "sensor_errors": list(errors),
                        "delivery_unknown": False,
                        "robot_motion_confirmed": False,
                    }
                grant.record_post_send_feedback(
                    prepared,
                    current_receipt,
                    after_snapshot,
                )
            else:
                return {
                    **step_result,
                    "task_description": PIPER_PROMPT,
                    "policy_identities": identities,
                    "gripper_effort_raw": profile.gripper_effort_raw,
                    "position_send_attempts": transport.position_send_attempts,
                    "position_frames_accepted": transport.position_frames_accepted,
                    "sensor_errors": list(errors),
                    "delivery_unknown": False,
                }

        if not short_loop or not grant.finished:
            raise RuntimeError("short-loop session ended without all feedback confirmations")
        return {
            "status": "SHORT_LOOP_COMPLETE_FRESH_FEEDBACK_OBSERVED",
            "task_description": PIPER_PROMPT,
            "policy_identities": identities,
            "steps": step_results,
            "gripper_effort_raw": profile.gripper_effort_raw,
            "position_send_attempts": transport.position_send_attempts,
            "position_frames_accepted": transport.position_frames_accepted,
            "sensor_errors": list(errors),
            "delivery_unknown": False,
            "robot_motion_confirmed": False,
        }
    except Exception as exc:
        attempts = transport.position_send_attempts
        unknown = attempts > attempts_before_step and current_receipt is None
        return {
            "status": ("POSITION_DELIVERY_UNKNOWN_DO_NOT_RETRY" if unknown
                       else "BLOCKED_BEFORE_POSITION_WRITE" if attempts == attempts_before_step
                       else "POSITION_BATCH_ACCEPTED_POSTCHECK_ERROR"),
            "error": f"{type(exc).__name__}: {exc}",
            "steps": step_results,
            "position_send_attempts": attempts,
            "position_frames_accepted": transport.position_frames_accepted,
            "delivery_unknown": unknown,
            "robot_motion_confirmed": False,
        }
    finally:
        stop.set()
        timer.cancel()
        for thread in threads:
            if thread.ident is not None:
                thread.join(timeout=0.5)
        bridge.close()


def run_supervised_one_step(
    *,
    approved: bool = False,
    expected_identity_lock: dict | None = None,
    site_profile_path: Path | None = None,
    port: int = 8002,
    base_index: int | None = None,
    wrist_index: int | None = None,
    base_device_id: str | None = None,
    wrist_device_id: str | None = None,
    roles_visually_confirmed: bool = False,
    _max_steps: int = 1,
    input_fn=input,
    output_fn=print,
) -> dict:
    """Require all gates before one batch or a bounded 3-step session."""
    if type(_max_steps) is not int or _max_steps not in (1, SHORT_LOOP_MAX_STEPS):
        return {"status": "BLOCKED_INVALID_SESSION_STEP_COUNT",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    if not approved:
        return {"status": "BLOCKED_EXPLICIT_SUPERVISED_STEP_FLAG_REQUIRED",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    if expected_identity_lock is None:
        return {"status": "BLOCKED_ARTIFACT_IDENTITY_LOCK_REQUIRED",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    if site_profile_path is None:
        return {"status": "BLOCKED_SITE_PROFILE_REQUIRED",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    if (type(base_index) is not int or type(wrist_index) is not int
            or base_index == wrist_index or not 1 <= port <= 65535):
        return {"status": "BLOCKED_CAMERA_OR_POLICY_CONFIGURATION_INVALID",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}
    if not base_device_id or not wrist_device_id or not roles_visually_confirmed:
        return {"status": "BLOCKED_CAMERA_IDENTITY_UNVERIFIED",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}

    try:
        from laptop.client.live_shadow import (
            prewarm_policy,
            validate_artifact_identity_lock,
            validate_policy_metadata,
        )
        from laptop.robot_client.supervised_policy_session import (
            MAX_ONE_STEP_GRANT_LIFETIME_NS,
        )

        profile = load_supervised_site_profile(Path(site_profile_path))
        lock = validate_artifact_identity_lock(expected_identity_lock)
        camera_identity = bind_camera_roles(
            base_index,
            wrist_index,
            base_device_id=base_device_id,
            wrist_device_id=wrist_device_id,
            roles_visually_confirmed=roles_visually_confirmed,
        )
        from openpi_client.websocket_client_policy import WebsocketClientPolicy

        client = WebsocketClientPolicy(host="127.0.0.1", port=port)
        metadata = client.get_server_metadata()
        validate_policy_metadata(metadata, lock)
        warmup = prewarm_policy(client)
        identities = _metadata_identities(metadata, lock)
        output_fn(
            f"Site profile: {profile.site_id}; robot {profile.robot_serial}; "
            f"firmware {profile.firmware_version}"
        )
        if _max_steps == SHORT_LOOP_MAX_STEPS:
            grant = request_short_loop_operator_grant(
                task_description=PIPER_PROMPT,
                **identities,
                gripper_effort_raw=profile.gripper_effort_raw,
                limits=profile.limits,
                site_gates=profile.p12_evidence,
                input_fn=input_fn,
                output_fn=output_fn,
            )
        else:
            grant = request_one_step_operator_grant(
                task_description=PIPER_PROMPT,
                **identities,
                gripper_effort_raw=profile.gripper_effort_raw,
                limits=profile.limits,
                site_gates=profile.p12_evidence,
                input_fn=input_fn,
                output_fn=output_fn,
                lifetime_ns=MAX_ONE_STEP_GRANT_LIFETIME_NS,
            )
        ticket = grant.to_worker_ticket()
    except Exception as exc:
        return {"status": "BLOCKED_SUPERVISED_PREFLIGHT",
                "error": f"{type(exc).__name__}: {exc}",
                "camera_opened": False, "can_opened": False,
                "position_send_attempts": 0, "robot_motion_confirmed": False}

    try:
        result = run_bounded(
            _worker,
            args=(20.0 if _max_steps == 1 else 10.0,
                  port, base_index, wrist_index, lock, camera_identity, profile, ticket),
            timeout_s=WORKER_TIMEOUT_S,
        )
    except Exception as exc:
        return {
            "status": "WORKER_START_OR_RESULT_UNKNOWN_DO_NOT_RETRY",
            "error": f"{type(exc).__name__}: {exc}",
            "camera_opened": "possibly",
            "can_opened": "possibly",
            "position_send_attempts": "unknown",
            "delivery_unknown": True,
            "retry_prohibited_until_adapter_enumeration_and_read_only_feedback": True,
            "robot_motion_confirmed": False,
            "policy_warmup": warmup,
            "camera_identity": camera_identity,
            "server_checkout_modified": False,
        }
    if result["status"] == "COMPLETE":
        return {**result["result"], "policy_warmup": warmup,
                "camera_identity": camera_identity,
                "server_checkout_modified": False}
    return {
        "status": "WORKER_TIMEOUT_OR_FAILURE_TREAT_DELIVERY_AS_UNKNOWN",
        "error": result.get("error"),
        "camera_opened": "possibly",
        "can_opened": "possibly",
        "position_send_attempts": "unknown",
        "delivery_unknown": True,
        "retry_prohibited_until_adapter_enumeration_and_read_only_feedback": True,
        "robot_motion_confirmed": False,
        "policy_warmup": warmup,
        "camera_identity": camera_identity,
        "server_checkout_modified": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    session_group = parser.add_mutually_exclusive_group()
    session_group.add_argument("--supervised-position-step", action="store_true",
                               help="explicitly opt into one live position batch")
    session_group.add_argument("--supervised-short-loop", action="store_true",
                               help="explicitly opt into a 3-step feedback-coupled session")
    parser.add_argument("--site-profile", type=Path)
    parser.add_argument("--artifact-identity-lock", type=Path)
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--base-index", type=int)
    parser.add_argument("--wrist-index", type=int)
    parser.add_argument("--base-device-id")
    parser.add_argument("--wrist-device-id")
    parser.add_argument("--roles-visually-confirmed", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if not (args.supervised_position_step or args.supervised_short_loop):
        result = {"status": "BLOCKED_EXPLICIT_SUPERVISED_STEP_FLAG_REQUIRED",
                  "camera_opened": False, "can_opened": False,
                  "position_send_attempts": 0, "robot_motion_confirmed": False}
    else:
        from laptop.client.live_shadow import (
            DEFAULT_IDENTITY_LOCK,
            load_artifact_identity_lock,
        )

        lock_path = args.artifact_identity_lock or DEFAULT_IDENTITY_LOCK
        try:
            lock = load_artifact_identity_lock(lock_path)
        except (OSError, ValueError, PermissionError) as exc:
            result = {"status": "BLOCKED_ARTIFACT_IDENTITY_LOCK_INVALID",
                      "error": f"{type(exc).__name__}: {exc}",
                      "camera_opened": False, "can_opened": False,
                      "position_send_attempts": 0, "robot_motion_confirmed": False}
        else:
            result = run_supervised_one_step(
                approved=True,
                expected_identity_lock=lock,
                site_profile_path=args.site_profile,
                port=args.port,
                base_index=args.base_index,
                wrist_index=args.wrist_index,
                base_device_id=args.base_device_id,
                wrist_device_id=args.wrist_device_id,
                roles_visually_confirmed=args.roles_visually_confirmed,
                _max_steps=(SHORT_LOOP_MAX_STEPS if args.supervised_short_loop else 1),
            )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
