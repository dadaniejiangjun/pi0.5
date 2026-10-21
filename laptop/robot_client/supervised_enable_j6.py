"""One approved EnableArm(6), with fresh stable-pose and six-motor readback."""

import argparse
import json
from pathlib import Path

from laptop.robot_client.supervised_mode_initialization import _read_for
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


def execute(*, enable_j6_once=False, operator_authorized_enable=False,
            zero_pose_supported=False, physical_estop_ready=False,
            motion_envelope_cleared=False, transport=None, read_for=_read_for):
    attestations = dict(operator_authorized_enable=operator_authorized_enable,
                        zero_pose_supported=zero_pose_supported,
                        physical_estop_ready=physical_estop_ready,
                        motion_envelope_cleared=motion_envelope_cleared)
    base = {"automatic_retry": False, "camera_opened": False,
            "position_target_sent": False, "reset_or_mode_setting_sent": False}
    if enable_j6_once is not True or not all(v is True for v in attestations.values()):
        return {**base, "status": "BLOCKED_ENABLE_APPROVAL_AND_SITE_CONFIRMATION_REQUIRED",
                "can_opened": False, "can_data_tx_attempts": 0}
    transport = transport if transport is not None else GsUsbPiperCanTransport()
    bridge = SafetyBridge(mode=RuntimeMode.SUPERVISED_MODE_INITIALIZATION,
                          transport=transport, limits=UNVERIFIED_ANALYSIS_LIMITS,
                          supervised_mode_initialization_tx_enabled=True)
    opened = False
    initial = None
    write = None
    try:
        bridge.start_read_only()
        opened = True
        read_for(bridge)
        initial = bridge.mode_initialization_snapshot(require_stable=True)
        if initial["ctrl_mode_raw"] != 1:
            raise PermissionError("CAN_CTRL is required before J6 enable")
        if all(initial["motor_enabled"]):
            return {**base, "status": "ALREADY_ALL_ENABLED_NO_WRITES", "can_opened": True,
                    "can_data_tx_attempts": 0, "final_feedback": initial}
        write = bridge.send_j6_enable_once(**attestations)
        read_for(bridge)
        final = bridge.mode_initialization_snapshot()
        delta = [b - a for a, b in zip(initial["joint_raw_0p001deg"], final["joint_raw_0p001deg"])]
        passed = (final["ctrl_mode_raw"] == 1 and all(final["motor_enabled"])
                  and all(abs(value) <= 50 for value in delta))
        return {**base, "status": "ALL_SIX_ENABLED_CONFIRMED" if passed else "ENABLE_READBACK_MISMATCH_STOP",
                "can_opened": True, "can_data_tx_attempts": transport.j6_enable_attempts,
                "write": write, "initial_feedback": initial, "final_feedback": final,
                "joint_delta_raw_0p001deg": delta}
    except Exception as exc:
        return {**base, "status": "ENABLE_STOPPED_NO_RETRY", "can_opened": opened,
                "can_data_tx_attempts": transport.j6_enable_attempts, "write": write,
                "initial_feedback": initial, "error_type": type(exc).__name__, "reason": str(exc)}
    finally:
        bridge.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("enable-j6-once", "operator-authorized-enable", "zero-pose-supported",
                 "physical-estop-ready", "motion-envelope-cleared"):
        parser.add_argument("--" + flag, action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    values = vars(args).copy()
    values.pop("output")
    result = execute(**values)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["status"] in ("ALL_SIX_ENABLED_CONFIRMED", "ALREADY_ALL_ENABLED_NO_WRITES") else 1


if __name__ == "__main__":
    raise SystemExit(main())
