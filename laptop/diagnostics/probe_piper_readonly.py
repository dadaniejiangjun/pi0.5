#!/usr/bin/env python3
"""Emit a future PiPER read-only audit contract without touching hardware.

The current P0/P1 invocation is intentionally ``NOT_RUN``. The module has no
SDK import, no CAN handle, and no state-changing call path. A future laptop
implementation may bind only the read methods listed below after explicit P2
approval and hardware audit.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


READ_ONLY_API_ALLOWLIST = (
    "GetCurrentSDKVersion",
    "GetCurrentProtocolVersion",
    "GetCanFps",
    "GetCanName",
    "GetArmStatus",
    "GetArmJointMsgs",
    "GetArmGripperMsgs",
    "GetArmEndPoseMsgs",
    "GetSDKJointLimitParam",
    "GetSDKGripperRangeParam",
)

REPORT = {
    "name": "probe_piper_readonly",
    "mode": "read_only_hardware_audit",
    "status": "NOT_RUN",
    "hardware_accessed": False,
    "can_accessed": False,
    "allowed_future_read_apis": list(READ_ONLY_API_ALLOWLIST),
    "fields": [
        "sdk_version",
        "protocol_version",
        "firmware_version",
        "can_fps",
        "arm_status",
        "ctrl_mode",
        "joint_state",
        "gripper_state",
    ],
    "reason": "P2 laptop hardware audit is explicitly out of scope for P0/P1.",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = dict(REPORT)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    else:
        print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
