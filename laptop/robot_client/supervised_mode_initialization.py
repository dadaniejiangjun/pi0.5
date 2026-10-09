"""Operator-approved reset -> standby -> CAN; reset may cause the arm to drop."""

import argparse
import json
from pathlib import Path
import time

from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.mode_initialization import STAGES, ModeInitializationFrame


def _read_for(bridge, duration=0.6):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        bridge.receive_frame(timeout_ms=25)


def execute(*, initialize_can_mode=False, operator_authorized_reset=False,
            zero_pose_supported=False, physical_estop_ready=False,
            motion_envelope_cleared=False, transport=None, read_for=_read_for):
    attestations = dict(operator_authorized_reset=operator_authorized_reset,
                        zero_pose_supported=zero_pose_supported,
                        physical_estop_ready=physical_estop_ready,
                        motion_envelope_cleared=motion_envelope_cleared)
    base = {"camera_opened": False, "position_targets_sent": False,
            "motor_enable_command_sent": False, "automatic_retry": False,
            "reset_can_cause_loss_of_torque": True}
    if initialize_can_mode is not True or not all(v is True for v in attestations.values()):
        return {**base, "status": "BLOCKED_EXPLICIT_INITIALIZATION_AND_SITE_CONFIRMATION_REQUIRED",
                "can_opened": False, "can_data_tx_attempts": 0}
    transport = transport if transport is not None else GsUsbPiperCanTransport()
    bridge = SafetyBridge(mode=RuntimeMode.SUPERVISED_MODE_INITIALIZATION,
                          transport=transport, limits=UNVERIFIED_ANALYSIS_LIMITS,
                          supervised_mode_initialization_tx_enabled=True)
    writes = []
    opened = False
    initial = None
    try:
        bridge.start_read_only()
        opened = True
        read_for(bridge)
        initial = bridge.mode_initialization_snapshot()
        if initial["ctrl_mode_raw"] == 1:
            return {**base, "status": "ALREADY_CAN_CTRL_NO_WRITES", "can_opened": True,
                    "can_data_tx_attempts": 0, "final_feedback": initial}
        for stage in STAGES:
            writes.append(bridge.send_mode_initialization_stage(stage, **attestations))
            read_for(bridge)
            snapshot = bridge.mode_initialization_snapshot()
            if stage == "standby" and snapshot["ctrl_mode_raw"] != 0:
                raise PermissionError("standby mode was not confirmed; CAN setting withheld")
            if stage == "can_joint" and snapshot["ctrl_mode_raw"] != 1:
                raise PermissionError("CAN control mode was not confirmed")
        return {**base, "status": "CAN_CTRL_CONFIRMED", "can_opened": True,
                "can_data_tx_attempts": transport.mode_initialization_attempts,
                "writes": writes, "initial_feedback": initial, "final_feedback": snapshot,
                "all_six_enabled": all(snapshot["motor_enabled"])}
    except Exception as exc:
        return {**base, "status": "INITIALIZATION_STOPPED_NO_RETRY", "can_opened": opened,
                "can_data_tx_attempts": transport.mode_initialization_attempts,
                "writes": writes, "initial_feedback": initial,
                "error_type": type(exc).__name__, "reason": str(exc)}
    finally:
        bridge.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("initialize-can-mode", "operator-authorized-reset", "zero-pose-supported",
                 "physical-estop-ready", "motion-envelope-cleared"):
        parser.add_argument("--" + flag, action="store_true")
    parser.add_argument("--preview", action="store_true", help="Print fixed frames without opening CAN")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.preview:
        result = {"status": "OFFLINE_INITIALIZATION_PREVIEW", "can_opened": False,
                  "can_data_tx_attempts": 0, "reset_can_cause_loss_of_torque": True,
                  "frames": [{"stage": stage, "can_id": hex(ModeInitializationFrame(stage).arbitration_id),
                              "data_hex": ModeInitializationFrame(stage).data.hex()} for stage in STAGES]}
    else:
        values = vars(args).copy()
        values.pop("preview")
        values.pop("output")
        result = execute(**values)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["status"] in (
        "OFFLINE_INITIALIZATION_PREVIEW", "ALREADY_CAN_CTRL_NO_WRITES", "CAN_CTRL_CONFIRMED") else 1


if __name__ == "__main__":
    raise SystemExit(main())
