"""Operator-approved reset -> standby -> CAN; reset may cause the arm to drop."""

import argparse
import json
from pathlib import Path
import time

from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import (
    CanInitializationDeliveryUnknown,
    GsUsbPiperCanTransport,
)
from laptop.transport.mode_initialization import STAGES, ModeInitializationFrame


def _read_for(bridge, duration=0.6):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        bridge.receive_frame(timeout_ms=25)


def _execute_standby_to_can(*, operator_authorized_mode_switch,
                            zero_pose_supported, physical_estop_ready,
                            motion_envelope_cleared, transport=None,
                            read_for=_read_for):
    attestations = (operator_authorized_mode_switch, zero_pose_supported,
                    physical_estop_ready, motion_envelope_cleared)
    base = {"camera_opened": False, "reset_command_sent": False,
            "reset_can_cause_loss_of_torque": True,
            "position_targets_sent": False, "motor_enable_command_sent": False,
            "robot_motion_commanded": False, "can_data_tx_calls": 0,
            "automatic_retry": False, "path": "standby_to_can_without_reset"}
    if not all(value is True for value in attestations):
        return {**base, "status": "BLOCKED_EXPLICIT_MODE_SWITCH_AND_SITE_CONFIRMATION_REQUIRED",
                "can_opened": False, "can_data_tx_attempts": 0, "writes": []}

    can_transport = transport if transport is not None else GsUsbPiperCanTransport()
    bridge = SafetyBridge(mode=RuntimeMode.SUPERVISED_MODE_INITIALIZATION,
                          transport=can_transport, limits=UNVERIFIED_ANALYSIS_LIMITS,
                          supervised_mode_initialization_tx_enabled=True)
    opened = False
    writes = []
    initial = None
    try:
        bridge.start_read_only()
        opened = True
        read_for(bridge)
        initial = bridge.mode_initialization_snapshot(require_zero=True)
        if initial["ctrl_mode_raw"] == 1:
            ready = all(initial["motor_enabled"])
            return {**base,
                    "status": "ALREADY_CAN_CTRL_NO_WRITES" if ready
                    else "ALREADY_CAN_CTRL_MOTOR_STATE_NOT_READY",
                    "can_opened": True, "can_data_tx_attempts": 0,
                    "writes": [], "initial_feedback": initial,
                    "all_six_enabled": ready}
        if initial["ctrl_mode_raw"] != 0:
            raise PermissionError("direct mode switch only accepts fresh STANDBY feedback")

        write = bridge.send_standby_to_can_once(
            operator_authorized_mode_switch=operator_authorized_mode_switch,
            zero_pose_supported=zero_pose_supported,
            physical_estop_ready=physical_estop_ready,
            motion_envelope_cleared=motion_envelope_cleared,
        )
        writes.append(write)
        read_for(bridge)
        final = bridge.mode_initialization_snapshot(require_zero=True)
        if final["ctrl_mode_raw"] != 1:
            return {**base, "status": "MODE_SWITCH_NOT_CONFIRMED_NO_RETRY",
                    "can_opened": True,
                    "can_data_tx_attempts": can_transport.mode_initialization_attempts,
                    "can_data_tx_calls": can_transport.mode_initialization_attempts,
                    "writes": writes, "initial_feedback": initial,
                    "final_feedback": final,
                    "all_six_enabled": all(final["motor_enabled"])}
        if any(abs(after - before) > 50 for before, after in zip(
                initial["joint_raw_0p001deg"], final["joint_raw_0p001deg"])):
            return {**base,
                    "status": "UNEXPECTED_JOINT_CHANGE_AFTER_MODE_SWITCH_STOP_NO_RETRY",
                    "can_opened": True,
                    "can_data_tx_attempts": can_transport.mode_initialization_attempts,
                    "can_data_tx_calls": can_transport.mode_initialization_attempts,
                    "writes": writes, "initial_feedback": initial,
                    "final_feedback": final,
                    "all_six_enabled": all(final["motor_enabled"])}
        all_enabled = all(final["motor_enabled"])
        return {**base,
                "status": "CAN_CTRL_CONFIRMED" if all_enabled
                else "CAN_CTRL_CONFIRMED_MOTOR_STATE_NOT_READY",
                "can_opened": True,
                "can_data_tx_attempts": can_transport.mode_initialization_attempts,
                "can_data_tx_calls": can_transport.mode_initialization_attempts,
                "writes": writes, "initial_feedback": initial,
                "final_feedback": final, "all_six_enabled": all_enabled}
    except CanInitializationDeliveryUnknown as exc:
        return {**base, "status": "MODE_SWITCH_DELIVERY_UNKNOWN_NO_RETRY",
                "can_opened": opened,
                "can_data_tx_attempts": can_transport.mode_initialization_attempts,
                "can_data_tx_calls": can_transport.mode_initialization_attempts,
                "writes": writes, "initial_feedback": initial,
                "error_type": type(exc).__name__, "reason": str(exc)}
    except Exception as exc:
        return {**base, "status": "MODE_SWITCH_STOPPED_NO_RETRY",
                "can_opened": opened,
                "can_data_tx_attempts": can_transport.mode_initialization_attempts,
                "can_data_tx_calls": can_transport.mode_initialization_attempts,
                "writes": writes, "initial_feedback": initial,
                "error_type": type(exc).__name__, "reason": str(exc)}
    finally:
        bridge.close()


def execute(*, initialize_can_mode=False, operator_authorized_reset=False,
            standby_to_can_from_standby=False, operator_authorized_mode_switch=False,
            zero_pose_supported=False, physical_estop_ready=False,
            motion_envelope_cleared=False, transport=None, read_for=_read_for):
    if standby_to_can_from_standby is True:
        if initialize_can_mode is not True:
            return {"status": "BLOCKED_EXPLICIT_INITIALIZATION_REQUIRED",
                    "can_opened": False, "can_data_tx_attempts": 0,
                    "automatic_retry": False}
        return _execute_standby_to_can(
            operator_authorized_mode_switch=operator_authorized_mode_switch,
            zero_pose_supported=zero_pose_supported,
            physical_estop_ready=physical_estop_ready,
            motion_envelope_cleared=motion_envelope_cleared,
            transport=transport, read_for=read_for)
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
    for flag in ("initialize-can-mode", "operator-authorized-reset",
                 "standby-to-can-from-standby", "operator-authorized-mode-switch",
                 "zero-pose-supported", "physical-estop-ready", "motion-envelope-cleared"):
        parser.add_argument("--" + flag, action="store_true")
    parser.add_argument("--preview", action="store_true", help="Print fixed frames without opening CAN")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.preview and args.standby_to_can_from_standby:
        frame = ModeInitializationFrame("can_joint")
        result = {"status": "OFFLINE_STANDBY_TO_CAN_PREVIEW", "can_opened": False,
                  "can_data_tx_attempts": 0, "can_data_tx_calls": 0,
                  "robot_motion_commanded": False, "reset_command_sent": False,
                  "position_targets_sent": False, "motor_enable_command_sent": False,
                  "frames": [{"path": "standby_to_can_without_reset",
                              "can_id": hex(frame.arbitration_id),
                              "data_hex": frame.data.hex()}]}
    elif args.preview:
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
