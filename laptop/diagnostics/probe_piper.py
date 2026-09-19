#!/usr/bin/env python3
"""Safe specification stub for a future read-only PiPER hardware audit.

This file intentionally performs no CAN/SDK operation in the plan-only phase.
It can emit the required report contract so later implementation has a stable
interface. Command-capable operations do not belong in this probe.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


SPEC = {
    "name": "probe_piper",
    "mode": "read_only_hardware_audit",
    "status": "NOT_RUN",
    "required_fields": [
        "firmware_version",
        "sdk_name",
        "sdk_version",
        "protocol_version",
        "can_device",
        "joint_feedback_hz",
        "gripper_type",
        "gripper_limits",
        "control_modes",
        "teach_mode_support",
        "real_joint_ranges",
        "joint_unit",
        "gripper_unit",
        "dh_convention",
        "safe_hold_behavior",
    ],
    "forbidden_operations": [
        "enable",
        "reset",
        "stop",
        "motion",
        "programmatic_teach_entry",
        "gripper_command",
    ],
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="emit NOT_RUN contract JSON")
    args = parser.parse_args()
    payload = {**SPEC, "implementation": "pending_laptop_hardware_audit"}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    else:
        print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
