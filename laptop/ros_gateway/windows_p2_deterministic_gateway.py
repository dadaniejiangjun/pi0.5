"""Run a local-authorized, model-free P2 ROS-to-CAN target test.

The CAN owner and SafetyBridge remain the existing Windows implementations.
The WSL ROS bridge is read-only until the local P2 plan is installed below.
This entry never resets, enables, homes, or changes the controller mode.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import uuid

from laptop.piper.control_profile import CONTINUOUS_GRIPPER_LIMITS
from laptop.piper.unit_conversion import rad_to_sdk_joint
from laptop.robot_client.piper_session import PiperSession
from laptop.ros_gateway.control_rpc_dispatcher import _validate_dispatch_feedback
from laptop.ros_gateway.p2_deterministic_stream import (
    P2_BOUNDED_TRIAL_REQUEST_BUDGET,
    P2DeterministicStreamDispatcher,
    STREAM_DURATION_NS,
    STREAM_EXPECTED_UPDATES,
    STREAM_RATE_HZ,
    derive_minimal_j1_target,
)
from laptop.ros_gateway.windows_read_only_gateway import WindowsReadOnlyGateway
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.ros_gateway.runtime_profiles import (
    load_runtime_profile,
    resolve_wsl_gateway_host,
)
from laptop.ros_gateway.runtime_owner_registry import (
    RuntimeOwnerRegistration,
    runtime_owner_directory,
)

def _raw_joints(state: dict) -> list[int]:
    return [int(rad_to_sdk_joint(value)) for value in state["joint_position_rad"]]


def _interval_summary(stamps: list[int]) -> dict:
    intervals = [b - a for a, b in zip(stamps, stamps[1:])]
    ordered = sorted(intervals)
    p95 = ordered[max(0, min(len(ordered) - 1, int(0.95 * len(ordered) + 0.999999) - 1))] if ordered else None
    return {
        "accepted_batches": len(stamps),
        "interval_count": len(intervals),
        "mean_hz": ((len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
                    if len(stamps) > 1 and stamps[-1] > stamps[0] else None),
        "interval_p95_ms": p95 / 1e6 if p95 is not None else None,
        "interval_max_ms": max(intervals) / 1e6 if intervals else None,
        "interval_min_ms": min(intervals) / 1e6 if intervals else None,
    }


def _write_json_exclusive(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _wait_for_fresh_same_session_preflight(gateway, session_id: str,
                                           *, timeout_s: float = 5.0):
    """Pair a recent ROS session snapshot with current local CAN feedback.

    The latest local CAN aggregate can advance just before the TCP writer
    publishes it. Accept a fresh same-session source at or before the latest
    local sequence, then validate both snapshots independently.
    """
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        source = gateway.recent_fresh_state(None, session_id)
        current = gateway.latest_state()
        now_ns = time.perf_counter_ns()
        if source is not None and current is not None:
            source_stamp = source.get("gateway_snapshot_monotonic_ns")
            current_stamp = current.get("gateway_snapshot_monotonic_ns")
            source_sequence = source.get("sequence")
            current_sequence = current.get("sequence")
            if (source.get("session_id") == session_id
                    and type(source_sequence) is int
                    and type(current_sequence) is int
                    and source_sequence <= current_sequence
                    and type(source_stamp) is int
                    and type(current_stamp) is int
                    and 0 <= now_ns - source_stamp <= 120_000_000
                    and 0 <= now_ns - current_stamp <= 120_000_000):
                _validate_dispatch_feedback(source)
                _validate_dispatch_feedback(current)
                return source, current
        time.sleep(0.01)
    raise TimeoutError("no fresh same-session feedback; no control authority installed")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="Windows WSL-facing interface IPv4; defaults to the current WSL route")
    parser.add_argument("--port", type=int, help="P2 control TCP port; defaults to the shared runtime profile")
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--can-log", type=Path, required=True)
    parser.add_argument("--confirm-device-owner", action="store_true", required=True)
    parser.add_argument("--physical-estop-ready", action="store_true", required=True)
    parser.add_argument("--low-speed-confirmed", action="store_true", required=True)
    parser.add_argument("--motion-envelope-cleared", action="store_true", required=True)
    parser.add_argument("--confirm-p2-deterministic-60s", action="store_true", required=True)
    args = parser.parse_args(argv)

    runtime_profile, profile_hash = load_runtime_profile("p2_control")
    if args.host is None:
        args.host = resolve_wsl_gateway_host()
    if args.port is None:
        args.port = runtime_profile["gateway"]["control_port"]

    if args.port < 1 or args.port > 65535:
        parser.error("port must be within 1..65535")
    for path in (args.plan, args.report, args.can_log):
        if path.exists():
            parser.error(f"refusing to overwrite existing evidence file: {path}")
    if args.plan.resolve() == args.report.resolve() or args.plan.resolve() == args.can_log.resolve():
        parser.error("plan, report, and CAN log paths must be different")

    report = {
        "schema": "piper_p2_deterministic_gateway_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": "STARTING_READ_ONLY_OWNER",
        "deployment_profile": "p2_control",
        "deployment_profile_sha256": profile_hash,
        "host": args.host,
        "port": args.port,
        "raw_can_rx_logging": False,
        "control_commands_transmitted": 0,
        "robot_motion_observed": "NOT_CLASSIFIED",
        "site_confirmations": {
            "windows_can_owner": True,
            "physical_estop_ready": True,
            "speed_percent": 5,
            "motion_envelope_cleared": True,
            "p2_deterministic_60s_authorized": True,
        },
        "paths": {"plan": str(args.plan), "report": str(args.report),
                  "can_log": str(args.can_log)},
        "post_stream_feedback": [],
    }
    _write_json_exclusive(args.report, report)

    def session_factory(**options):
        return PiperSession(
            bridge=SafetyBridge(
                mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
                transport=GsUsbPiperCanTransport(
                    bounded_trial_request_budget=P2_BOUNDED_TRIAL_REQUEST_BUDGET,
                ),
                limits=UNVERIFIED_ANALYSIS_LIMITS,
                supervised_j1_commissioning_tx_enabled=True,
            ),
            **options,
        )

    gateway = None
    owner_registration = None
    dispatcher = None
    session_id = None
    try:
        owner_registration = RuntimeOwnerRegistration(
            "windows_gateway",
            runtime_owner_directory(Path(__file__).resolve().parents[2]),
            details={
                "deployment_profile": "p2_control",
                "deployment_profile_sha256": profile_hash,
                "ros_domain_id": runtime_profile["ros_domain_id"],
                "host": args.host,
                "control_port": args.port,
                "base_image_port": None,
                "wrist_image_port": None,
                "can_access_enabled": True,
                "camera_access_enabled": False,
                "control_authority": "local_p2_deterministic_only",
                "report_path": str(args.report),
            },
        )
        gateway = WindowsReadOnlyGateway(
            host=args.host,
            control_port=args.port,
            base_image_port=None,
            wrist_image_port=None,
            log_path=args.can_log,
            raw_can_log=False,
            owner_registration=owner_registration,
            local_session_factory=session_factory,
        ).start()
        report["status"] = "WAITING_FOR_CURRENT_ROS_SESSION"
        deadline = time.perf_counter() + 60
        session_id = ""
        while time.perf_counter() < deadline:
            session_id = input("Current WSL ROS bridge session_id (blank cancels): ").strip()
            if not session_id:
                report["status"] = "CANCELLED_BEFORE_CONTROL_AUTHORIZATION"
                return 1
            if session_id.isascii() and len(session_id) <= 128:
                break
            print("Invalid bounded ASCII session ID; no target authority installed.", flush=True)
        else:
            raise TimeoutError("no current WSL ROS session ID supplied")

        ros_source_state, state = _wait_for_fresh_same_session_preflight(
            gateway, session_id, timeout_s=5.0,
        )

        target = derive_minimal_j1_target(state, delta_raw=1_000)
        gripper_raw = int(state["gripper_raw"])
        if not CONTINUOUS_GRIPPER_LIMITS.minimum_raw <= gripper_raw <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw:
            raise PermissionError("current gripper feedback is outside the shared continuous raw envelope")
        scope = "p2_deterministic_" + uuid.uuid4().hex
        now = time.perf_counter_ns()
        dispatcher = P2DeterministicStreamDispatcher(
            queue=gateway.task_request_queue,
            latest_state_provider=gateway.latest_state,
            source_state_provider=gateway.recent_control_source,
            session_id=session_id,
            task_scope_id=scope,
            joint_target_rad=target,
            gripper_target_raw=gripper_raw,
            valid_until_monotonic_ns=now + 120_000_000_000,
            operator_approved=True,
            physical_estop_ready=True,
            low_speed_confirmed=True,
            motion_envelope_cleared=True,
        )
        plan = {
            "schema": "piper_p2_deterministic_plan_v1",
            "status": "LOCAL_AUTHORIZED",
            "session_id": session_id,
            "task_scope_id": scope,
            "preflight_state": state,
            "same_session_ros_source_state": ros_source_state,
            "preflight_latest_local_state_sequence": state["sequence"],
            "joint_target_rad": list(target),
            "joint_target_raw_0p001deg": list(_raw_joints({"joint_position_rad": target})),
            "joint_step_deg": 1.0,
            "joint_single_step_hard_cap_deg": 20.0,
            "cumulative_joint_angle_limit_deg": None,
            "speed_percent": 5,
            "gripper_target_raw": gripper_raw,
            "gripper_effort_raw": CONTINUOUS_GRIPPER_LIMITS.effort_raw,
            "gripper_limits": asdict(CONTINUOUS_GRIPPER_LIMITS),
            "gripper_nominal_m_scale_verified": False,
            "stream_duration_ns": STREAM_DURATION_NS,
            "stream_expected_updates": STREAM_EXPECTED_UPDATES,
            "stream_rate_hz": STREAM_RATE_HZ,
            "stream_period_ns": 100_000_000,
            "bounded_trial_request_budget": P2_BOUNDED_TRIAL_REQUEST_BUDGET,
            "created_monotonic_ns": now,
            "local_authority_is_p2_only": True,
            "p12_or_p5_attestation_used": False,
            "automatic_mode_enable_home_zero_or_retry": False,
        }
        _write_json_exclusive(args.plan, plan)
        gateway.control_rpc_dispatcher = dispatcher
        gateway._local_session_revoker = dispatcher.revoke_session
        report.update({
            "status": "P2_PLAN_AUTHORIZED_WAITING_FOR_ROS_TARGETS",
            "plan": plan,
            "state_before_control": state,
            "same_session_ros_source_state": ros_source_state,
            "mode_switch_enable_home_zero": False,
        })
        print(json.dumps({
            "status": report["status"],
            "process_id": os.getpid(),
            "runtime_owner_path": str(owner_registration.path),
            "session_id": session_id,
            "task_scope_id": scope,
            "joint_target_raw_0p001deg": plan["joint_target_raw_0p001deg"],
            "gripper_target_raw_hold": gripper_raw,
            "continuous_stream": {"seconds": 60, "updates": 600, "hz": 10},
            "bounded_trial_request_budget": P2_BOUNDED_TRIAL_REQUEST_BUDGET,
            "speed_percent": 5,
            "step_cap_deg": 20,
            "actual_joint_step_deg": 1,
            "gripper_limits": plan["gripper_limits"],
            "plan_path": str(args.plan),
        }, sort_keys=True), flush=True)

        wait_until = time.perf_counter() + 125
        while time.perf_counter() < wait_until and not gateway.stop_event.wait(0.02):
            if dispatcher.failed_reason:
                report["status"] = "STOPPED_ON_FIRST_REJECTION_OR_FAILURE"
                report["stop_reason"] = dispatcher.failed_reason
                break
            if dispatcher.stream_complete:
                report["status"] = (
                    "STREAM_600_UPDATES_SENT" if len(dispatcher.stream_request_times_ns) == STREAM_EXPECTED_UPDATES
                    else "STREAM_TIME_WINDOW_ENDED_BEFORE_600_UPDATES"
                )
                break
            if time.perf_counter_ns() >= dispatcher.expiry_ns:
                report["status"] = "AUTHORIZATION_EXPIRED"
                break

        report["control_stop_monotonic_ns"] = time.perf_counter_ns()
        report["dispatcher_records"] = list(dispatcher.records)
        report["stream_request_times_monotonic_ns"] = list(dispatcher.stream_request_times_ns)
        report["stream_started_monotonic_ns"] = dispatcher.stream_started_ns
        report["stream_count"] = len(dispatcher.stream_request_times_ns)
        adapter_send_stamps = [
            record["adapter_accepted_monotonic_ns"][0]
            for record in report["dispatcher_records"]
            if record.get("phase") == "stream" and record.get("accepted") is True
            and record.get("adapter_accepted_monotonic_ns")
        ]
        report["actual_adapter_send_timing"] = _interval_summary(adapter_send_stamps)
        report["actual_adapter_send_first_frame_monotonic_ns"] = adapter_send_stamps
        gateway.control_rpc_dispatcher = None
        gateway._local_session_revoker = None
        pending_before_clear = gateway.task_request_queue._queue.qsize()
        gateway.task_request_queue.close()
        report["queue_clear"] = {
            "pending_before_clear": pending_before_clear,
            "pending_after_clear": gateway.task_request_queue._queue.qsize(),
            "queue_closed_to_future_targets": True,
        }
        report["tx_stopped_monotonic_ns"] = time.perf_counter_ns()

        read_end = time.perf_counter() + 5.0
        last_sequence = -1
        while time.perf_counter() < read_end:
            final = gateway.latest_state()
            if final is not None and final.get("sequence") != last_sequence:
                session_state = gateway.recent_fresh_state(final["sequence"], session_id)
                if session_state is None:
                    time.sleep(0.01)
                    continue
                report["post_stream_feedback"].append({
                    "host_monotonic_ns": session_state.get("gateway_snapshot_monotonic_ns"),
                    "session_id": session_id,
                    "sequence": final.get("sequence"),
                    "joint_position_rad": session_state.get("joint_position_rad"),
                    "joint_raw_0p001deg": _raw_joints(session_state),
                    "gripper_raw": session_state.get("gripper_raw"),
                    "ctrl_mode_raw": session_state.get("ctrl_mode_raw"),
                    "arm_status_raw": session_state.get("arm_status_raw"),
                    "error_code": session_state.get("error_code"),
                    "motor_enabled": session_state.get("motor_enabled"),
                })
                last_sequence = final["sequence"]
            time.sleep(0.01)
        report["post_stream_read_only_seconds"] = 5.0
        latest = gateway.latest_state()
        report["final_state"] = (
            gateway.recent_fresh_state(latest["sequence"], session_id)
            if latest is not None else None
        )
        report["control_commands_transmitted"] = sum(
            len(record.get("adapter_frame_ids", []))
            for record in report["dispatcher_records"]
        )
        report["physical_stop_verified"] = False
        return 0 if report["status"] == "STREAM_600_UPDATES_SENT" else 1
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED_NO_RETRY"
        report["stop_reason"] = "local gateway interrupted; no automatic retry"
        return 130
    except Exception as exc:
        report["status"] = "P2_PREFLIGHT_OR_STREAM_FAILED_NO_RETRY"
        report["stop_reason"] = f"{type(exc).__name__}: {exc}"
        if gateway is not None:
            now_ns = time.perf_counter_ns()
            latest = gateway.latest_state()
            with gateway.lock:
                session_cache = list(gateway._session_state_sources.get(session_id, ())) if session_id else []
                can_connected = bool(gateway._can_connected)
                can_error = gateway._can_error
            newest_cached = session_cache[-1] if session_cache else None
            report["preflight_diagnostics"] = {
                "session_id": session_id,
                "can_connected": can_connected,
                "can_error": can_error,
                "latest_local_sequence": latest.get("sequence") if latest else None,
                "latest_local_age_ms": (
                    (now_ns - latest.get("gateway_snapshot_monotonic_ns")) / 1e6
                    if latest and type(latest.get("gateway_snapshot_monotonic_ns")) is int else None
                ),
                "session_source_cache_count": len(session_cache),
                "session_source_sequences": [wire.get("sequence") for wire, _oldest in session_cache],
                "session_source_cache_sessions": list(gateway._session_state_sources),
                "newest_session_source_age_ms": (
                    (now_ns - newest_cached[0].get("gateway_snapshot_monotonic_ns")) / 1e6
                    if newest_cached and type(newest_cached[0].get("gateway_snapshot_monotonic_ns")) is int else None
                ),
                "tcp_session_metrics": (
                    gateway.control_session_diagnostics(session_id)
                    if session_id and callable(getattr(gateway, "control_session_diagnostics", None))
                    else {}
                ),
            }
        return 1
    finally:
        if dispatcher is not None and gateway is not None:
            dispatcher.revoke_session(dispatcher.session_id)
        if gateway is not None:
            gateway.control_rpc_dispatcher = None
            gateway._local_session_revoker = None
            try:
                gateway.task_request_queue.close()
            except Exception:
                pass
            gateway.close()
        if owner_registration is not None:
            owner_registration.close()
        report["gateway_closed_monotonic_ns"] = time.perf_counter_ns()
        try:
            args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except Exception as exc:
            print(json.dumps({"report_write_error": f"{type(exc).__name__}: {exc}",
                              "report_path": str(args.report)}), flush=True)
        print(json.dumps({"status": report["status"],
                          "stop_reason": report.get("stop_reason"),
                          "stream_count": report.get("stream_count", 0),
                          "control_commands_transmitted": report.get("control_commands_transmitted", 0),
                          "report": str(args.report)}, sort_keys=True), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
