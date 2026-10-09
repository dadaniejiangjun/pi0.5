"""Reusable, bounded PiPER operations with exclusive per-invocation reports."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from laptop.diagnostics.probe_piper_feedback_ack_only import probe as feedback_probe
from laptop.diagnostics.probe_motor_enable_ack_only import probe as motor_probe
from laptop.robot_client.supervised_mode_initialization import execute as initialize
from laptop.robot_client.supervised_enable_j6 import execute as enable_j6
from laptop.robot_client.supervised_axis_commissioning import execute as step_joint
from laptop.robot_client.supervised_gripper_commissioning import execute as step_gripper, PHRASE as GRIPPER_PHRASE
from laptop.robot_client.supervised_j1_commissioning import (
    execute as step_j1, CONFIRMATION_PHRASE, CONFIRMATION_PHRASE_ONE_DEGREE,
)


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="operation", required=True)
    for name in ("status", "motor-status", "mode-init", "enable-j6", "step-j1", "step-joint", "step-gripper"):
        command = commands.add_parser(name)
        command.add_argument("--output", type=Path, required=True,
                             help="New report path; existing files are refused before CAN opens")
        if name in ("status", "motor-status"):
            command.add_argument("--read-approved", action="store_true")
            command.add_argument("--duration", type=float, default=3.0)
        else:
            command.add_argument("--physical-estop-ready", action="store_true")
            command.add_argument("--motion-envelope-cleared", action="store_true")
        if name in ("mode-init", "enable-j6"):
            command.add_argument("--zero-pose-supported", action="store_true")
            flag = "operator-authorized-reset" if name == "mode-init" else "operator-authorized-enable"
            command.add_argument("--" + flag, action="store_true")
        if name == "step-j1":
            command.add_argument("--degrees", type=float, choices=(0.1, 1.0), required=True)
            command.add_argument("--low-speed-confirmed", action="store_true")
        if name == "step-joint":
            command.add_argument("--axis", type=int, choices=range(2, 7), required=True)
            command.add_argument("--low-speed-confirmed", action="store_true")
        if name == "step-gripper":
            command.add_argument("--opening-mm", type=int, choices=(1, 5), required=True)
            command.add_argument("--low-speed-confirmed", action="store_true")
    return root


def dispatch(args):
    if args.operation == "status":
        return feedback_probe(args.duration, args.read_approved)
    if args.operation == "motor-status":
        return motor_probe(args.duration, approved=args.read_approved)
    site = dict(physical_estop_ready=args.physical_estop_ready,
                motion_envelope_cleared=args.motion_envelope_cleared)
    if args.operation == "mode-init":
        return initialize(initialize_can_mode=True,
                          operator_authorized_reset=args.operator_authorized_reset,
                          zero_pose_supported=args.zero_pose_supported, **site)
    if args.operation == "enable-j6":
        return enable_j6(enable_j6_once=True,
                         operator_authorized_enable=args.operator_authorized_enable,
                         zero_pose_supported=args.zero_pose_supported, **site)
    if args.operation == "step-gripper":
        phrase = GRIPPER_PHRASE if args.opening_mm == 1 else "AUTHORIZE GRIPPER OPEN +5.000 MM AT 0.300 NM COMMISSIONING STEP"
        return step_gripper(opening_mm=args.opening_mm,
            log_path=args.output.with_name(args.output.name + ".can.jsonl"),
            low_speed_confirmed=args.low_speed_confirmed,
            confirm=lambda preview: input(f"Type exactly {phrase!r}: "), **site)
    if args.operation == "step-joint":
        phrase = f"AUTHORIZE J{args.axis} {(-1 if args.axis == 3 else 1):+.3f} DEGREE COMMISSIONING STEP"
        return step_joint(axis=args.axis, low_speed_confirmed=args.low_speed_confirmed,
                          confirm=lambda preview: input(f"Type exactly {phrase!r}: "), **site)
    phrase = CONFIRMATION_PHRASE_ONE_DEGREE if args.degrees == 1 else CONFIRMATION_PHRASE
    return step_j1(commissioning_j1_plus_0_1deg=args.degrees == 0.1,
                   commissioning_j1_plus_1deg=args.degrees == 1,
                   low_speed_confirmed=args.low_speed_confirmed,
                   confirm=lambda preview: input(f"Type exactly {phrase!r}: "), **site)


SUCCESS = {"FEEDBACK_FIELDS_PRESENT", "ALL_SIX_ENABLED", "CAN_CTRL_CONFIRMED",
           "ALREADY_CAN_CTRL_NO_WRITES", "ALL_SIX_ENABLED_CONFIRMED",
           "ALREADY_ALL_ENABLED_NO_WRITES", "SINGLE_STEP_FEEDBACK_MATCH", "GRIPPER_FEEDBACK_MATCH"}


def main(argv=None):
    args = parser().parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the report before any device access. No retries or hidden loops.
    with args.output.open("x", encoding="utf-8") as report:
        metadata = dict(operation=args.operation, started_utc=datetime.now(timezone.utc).isoformat())
        report.write(json.dumps({**metadata, "status": "STARTED_RESULT_PENDING"}) + "\n")
        report.flush()
        try:
            result = dispatch(args)
        except Exception as exc:
            result = dict(status="OPERATION_FAILED_NO_RETRY", error_type=type(exc).__name__,
                          reason=str(exc), automatic_retry=False)
        result = {**metadata, **result, "finished_utc": datetime.now(timezone.utc).isoformat()}
        report.seek(0)
        report.truncate()
        report.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] in SUCCESS else 1


if __name__ == "__main__":
    raise SystemExit(main())
