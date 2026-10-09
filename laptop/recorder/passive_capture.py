"""Bounded dual-camera/PiPER feedback capture; never sends CAN data frames.

The default mode is a short P4 wiring smoke. ``operator_demo=True`` records a
bounded, operator-run task episode without trying to capture OEM command frames.
It does not enter teach mode, enable, move, or infer an action target.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import threading
import time
import uuid
from pathlib import Path

from laptop.recorder.episode import EpisodeWriter


CAN_CHANNEL = {
    "0x2A1": "status", "0x2A5": "joint_12", "0x2A6": "joint_34",
    "0x2A7": "joint_56", "0x2A8": "gripper",
}


def assemble(writer: EpisodeWriter, camera_report: dict, feedback_log: Path) -> None:
    """Copy timestamped raw streams into one immutable local episode."""
    for role in ("base", "wrist"):
        for frame_id, timestamp in enumerate(camera_report.get("frame_timestamps_ns", {}).get(role, [])):
            writer.add(role, timestamp, {"frame_id": frame_id,
                                         "timestamp_kind": "host_perf_counter_ns_after_read"})
    with Path(feedback_log).open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            channel = CAN_CHANNEL.get(row["can_id"])
            if channel is None:
                continue
            if channel == "status":
                payload = {"raw_payload_hex": row["raw_payload_hex"], **row["status"]}
            elif channel == "gripper":
                payload = {"raw_payload_hex": row["raw_payload_hex"], **row["gripper_raw"]}
            else:
                index = {"joint_12": 0, "joint_34": 2, "joint_56": 4}[channel]
                payload = {"raw_payload_hex": row["raw_payload_hex"],
                           "raw_joint_pair": row["joint_raw_0p001deg_nominal"][index:index + 2],
                           "unit_status": "nominal_0p001deg_not_hardware_verified"}
            writer.add(channel, row["monotonic_ns"], payload)


def capture(raw_root: Path, *, duration_s: float = 10, approved: bool = False,
            base_index: int = 0, wrist_index: int = 1,
            manual_teach: bool = False, lead_in_s: float = 0,
            operator_demo: bool = False,
            episode_id: str | None = None, camera_identity: dict | None = None,
            can_adapter_bus: int | None = None,
            can_adapter_address: int | None = None, task: str | None = None) -> dict:
    if manual_teach and operator_demo:
        raise ValueError("choose raw-command manual_teach or feedback-only operator_demo, not both")
    max_duration_s = 60 if operator_demo else 20 if manual_teach else 10
    if not 0 < duration_s <= max_duration_s:
        raise ValueError(f"capture duration must be in (0, {max_duration_s}] seconds")
    if not 0 <= lead_in_s <= 10 or (lead_in_s and not (manual_teach or operator_demo)):
        raise ValueError("lead-in must be 0-10 seconds and is only for a task demonstration")
    if base_index == wrist_index:
        raise ValueError("Base and Wrist camera indices must differ")
    if not approved:
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "can_started": False,
                "cameras_started": False, "can_data_tx_calls": 0}
    if manual_teach and (can_adapter_bus is None or can_adapter_address is None):
        raise ValueError(
            "manual command-target capture requires the exact USB bus/address of a separate passive adapter"
        )
    if (can_adapter_bus is None) != (can_adapter_address is None):
        raise ValueError("CAN adapter bus and address must be provided together")

    from laptop.diagnostics.probe_dual_cameras import probe as camera_probe
    from laptop.diagnostics.probe_piper_feedback_ack_only import probe as can_probe
    from laptop.transport.gs_usb_piper_can import CanReceiveMode

    source = ("manual_teach" if manual_teach else "operator_demonstration"
              if operator_demo else "passive_wiring_smoke")
    writer = EpisodeWriter(raw_root, episode_id=episode_id, source=source,
                           **({"task": task} if task is not None else {}))
    feedback_log = writer.path / "feedback_can_raw.jsonl"
    command_log = writer.path / "command_can_raw.jsonl" if manual_teach else None
    camera_report: dict = {}
    camera_error: list[str] = []
    camera_ready = threading.Event()
    start_gate = threading.Event()
    camera_opened: list[bool] = []

    def mark_camera_opened(opened: bool) -> None:
        camera_opened.append(opened)
        camera_ready.set()

    def run_cameras() -> None:
        try:
            camera_report.update(camera_probe(
                base_index, wrist_index, duration_s, writer.path,
                start_gate=start_gate, on_opened=mark_camera_opened,
                start_gate_timeout_s=lead_in_s + 10))
        except Exception as exc:
            camera_error.append(repr(exc))
            camera_ready.set()

    def start_camera_capture() -> None:
        if not camera_thread.is_alive():
            raise RuntimeError("camera capture stopped before CAN receiver was ready")
        writer.event("capture_start")
        start_gate.set()
        if manual_teach or operator_demo:
            print("CAPTURE_START: operator may begin the demonstration", flush=True)

    camera_thread = threading.Thread(target=run_cameras, name="piper-cameras")
    camera_thread.start()
    can_report: dict = {}
    can_error = None
    try:
        if not camera_ready.wait(timeout=10) or not camera_opened or not camera_opened[0]:
            raise RuntimeError("both cameras must open before CAN receive")
        if lead_in_s:
            print(f"CAPTURE_ARMED: starting in {lead_in_s:g} seconds", flush=True)
            time.sleep(lead_in_s)
        can_report = can_probe(duration_s, approved=True, feedback_log=feedback_log,
                               max_duration_s=max_duration_s, on_receiver_ready=start_camera_capture,
                               command_log=command_log,
                               adapter_bus=can_adapter_bus,
                               adapter_address=can_adapter_address,
                               receive_mode=(CanReceiveMode.LISTEN_ONLY if manual_teach
                                             else CanReceiveMode.NORMAL_ACKING))
    except Exception as exc:
        can_error = repr(exc)
    finally:
        camera_thread.join()
        writer.event("capture_end")

    assembly_error = None
    if feedback_log.is_file():
        try:
            assemble(writer, camera_report, feedback_log)
        except Exception as exc:
            assembly_error = repr(exc)
    if camera_identity is not None:
        camera_report["camera_identity"] = camera_identity
    (writer.path / "camera_report.json").write_text(
        json.dumps(camera_report, indent=2) + "\n", encoding="utf-8")
    (writer.path / "can_report.json").write_text(
        json.dumps(can_report, indent=2) + "\n", encoding="utf-8")
    if camera_identity is not None:
        (writer.path / "camera_identity.json").write_text(
            json.dumps(camera_identity, indent=2) + "\n", encoding="utf-8")
    writer.event("capture_result", camera_status=camera_report.get("status"),
                 can_status=can_report.get("status"), camera_error=camera_error,
                 can_error=can_error, assembly_error=assembly_error)
    command_target_decode = {"status": "NOT_CAPTURED", "target_count": 0}
    if command_log is not None and command_log.is_file():
        try:
            from laptop.recorder.piper_command_targets import (
                ACTION_TARGET_SOURCE,
                decode_command_timeline,
            )
            targets = decode_command_timeline(command_log)
            command_target_decode = {"status": "COMPLETE_RAW_COMMAND_TARGETS",
                                     "target_count": len(targets)}
            metadata_path = writer.path / "metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata.update({
                "action_target_present": True,
                "action_target_source": ACTION_TARGET_SOURCE,
                "action_target_provenance_verified": False,
                "command_target_log": "command_can_raw.jsonl",
                "command_target_log_sha256": hashlib.sha256(command_log.read_bytes()).hexdigest(),
                "command_direction_verified": False,
            })
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        except (OSError, ValueError, TypeError) as exc:
            command_target_decode = {"status": "RAW_COMMAND_TARGETS_INVALID",
                                     "target_count": 0, "error": repr(exc)}
    (writer.path / "command_target_capture_report.json").write_text(
        json.dumps(command_target_decode, indent=2) + "\n", encoding="utf-8")
    notes = ("Operator-led raw-command demonstration; outcome not yet validated"
             if manual_teach else "Feedback-only operator demonstration; target is derived offline"
             if operator_demo else "P4 passive wiring smoke; not a task demonstration")
    integrity = writer.close(success=None, notes=notes)
    recorded = (can_report.get("status") == "FEEDBACK_FIELDS_PRESENT"
                and camera_report.get("status") == "CONCURRENT_STREAM_PASS"
                and assembly_error is None and not camera_error
                and (not operator_demo or integrity.get("status") == "FILE_INTEGRITY_PASS"))
    recorded_status = ("MANUAL_CAPTURE_RECORDED" if manual_teach else
                       "OPERATOR_DEMO_CAPTURE_RECORDED" if operator_demo else
                       "PASSIVE_CAPTURE_RECORDED")
    return {"status": recorded_status if recorded else "PARTIAL",
            "path": str(writer.path), "camera_status": camera_report.get("status"),
            "can_status": can_report.get("status"), "can_error": can_error,
            "camera_error": camera_error, "assembly_error": assembly_error,
            "integrity": integrity, "can_data_tx_calls": 0,
            "command_log": str(command_log) if command_log is not None else None,
            "observed_command_id_counts": can_report.get("observed_command_id_counts", {}),
            "command_target_capture": command_target_decode,
            "command_direction_verified": False,
            "command_targets_decoded": False,
            "robot_motion_commanded": False, "training_eligible": False,
            "camera_identity_status": "OPERATOR_CONFIRMED" if camera_identity else "UNVERIFIED"}


def _capture_worker(raw_root: Path, kwargs: dict) -> dict:
    return capture(raw_root, **kwargs)


def capture_supervised(raw_root: Path, *, camera_identity: dict,
                       timeout_s: float | None = None, **capture_kwargs) -> dict:
    """Run a capture in a disposable process; write a separate completion marker."""
    from laptop.diagnostics.bounded_process import run_bounded

    if camera_identity.get("schema") != "piper_camera_role_identity_v1":
        raise ValueError("operator-confirmed camera identity is required")
    duration = capture_kwargs.get("duration_s", 10)
    lead_in = capture_kwargs.get("lead_in_s", 0)
    manual = capture_kwargs.get("manual_teach", False)
    operator_demo = capture_kwargs.get("operator_demo", False)
    if manual and operator_demo:
        raise ValueError("choose one demonstration capture mode")
    max_duration = 60 if operator_demo else 20 if manual else 10
    if not 0 < duration <= max_duration or not 0 <= lead_in <= 10:
        raise ValueError("invalid duration or lead-in")
    if not capture_kwargs.get("approved"):
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "can_started": False,
                "cameras_started": False, "can_data_tx_calls": 0}
    episode_id = uuid.uuid4().hex
    raw_root = Path(raw_root).resolve()
    kwargs = {**capture_kwargs, "episode_id": episode_id, "camera_identity": camera_identity}
    result = run_bounded(_capture_worker, args=(raw_root, kwargs),
                         timeout_s=timeout_s if timeout_s is not None else duration + lead_in + 15)
    episode = raw_root / episode_id
    manifest = episode / "manifest.json"
    marker = {
        "schema": "piper_capture_completion_v1",
        "episode_id": episode_id,
        "supervisor_status": result["status"],
        "capture_status": result["result"].get("status") if isinstance(result["result"], dict) else None,
        "manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest() if manifest.is_file() else None,
        "complete": result["status"] == "COMPLETE" and isinstance(result["result"], dict)
        and result["result"].get("status") in (
            "MANUAL_CAPTURE_RECORDED", "OPERATOR_DEMO_CAPTURE_RECORDED", "PASSIVE_CAPTURE_RECORDED")
        and manifest.is_file(),
        "complete_meaning": "capture process ended and raw manifest exists; not training acceptance",
        "training_eligible": False,
    }
    marker_dir = raw_root / "_capture_runs"
    marker_dir.mkdir(parents=True, exist_ok=True)
    marker_path = marker_dir / f"{episode_id}.json"
    marker_path.write_text(json.dumps(marker, indent=2) + "\n", encoding="utf-8")
    if result["status"] == "COMPLETE" and isinstance(result["result"], dict):
        return {**result["result"], "completion_marker": str(marker_path)}
    return {"status": result["status"], "path": str(episode), "completion_marker": str(marker_path),
            "error": result["error"], "training_eligible": False,
            "can_data_tx_calls": 0, "robot_motion_commanded": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, default=Path("datasets/raw"))
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--base-index", type=int, required=True)
    parser.add_argument("--wrist-index", type=int, required=True)
    parser.add_argument("--passive-capture-approved", action="store_true")
    parser.add_argument("--base-device-id", required=True)
    parser.add_argument("--wrist-device-id", required=True)
    parser.add_argument("--roles-visually-confirmed", action="store_true")
    args = parser.parse_args()
    if args.passive_capture_approved:
        from laptop.diagnostics.camera_identity import bind_camera_roles
        identity = bind_camera_roles(
            args.base_index, args.wrist_index,
            base_device_id=args.base_device_id, wrist_device_id=args.wrist_device_id,
            roles_visually_confirmed=args.roles_visually_confirmed,
        )
        result = capture_supervised(args.raw_root, duration_s=args.duration,
                                    approved=True, base_index=args.base_index,
                                    wrist_index=args.wrist_index, camera_identity=identity)
    else:
        result = capture(args.raw_root, duration_s=args.duration, approved=False,
                         base_index=args.base_index, wrist_index=args.wrist_index)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASSIVE_CAPTURE_RECORDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
