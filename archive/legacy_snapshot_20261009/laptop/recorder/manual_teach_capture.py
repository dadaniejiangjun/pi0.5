"""Record one bounded operator-led raw demonstration; never command PiPER.

This preserves video and observed feedback, not a validated policy action
target. The operator alone decides whether and how to move the robot.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from laptop.recorder.passive_capture import capture, capture_supervised


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, default=Path("datasets/raw"))
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--lead-in", type=float, default=5)
    parser.add_argument("--base-index", type=int, required=True)
    parser.add_argument("--wrist-index", type=int, required=True)
    parser.add_argument("--can-adapter-bus", type=int, required=True,
                        help="USB bus for a separate candleLight passive listener")
    parser.add_argument("--can-adapter-address", type=int, required=True,
                        help="USB address for that passive listener on its current connection")
    parser.add_argument("--manual-capture-approved", action="store_true")
    parser.add_argument("--base-device-id", required=True)
    parser.add_argument("--wrist-device-id", required=True)
    parser.add_argument("--roles-visually-confirmed", action="store_true")
    args = parser.parse_args()
    if args.manual_capture_approved:
        from laptop.diagnostics.camera_identity import bind_camera_roles
        identity = bind_camera_roles(
            args.base_index, args.wrist_index,
            base_device_id=args.base_device_id, wrist_device_id=args.wrist_device_id,
            roles_visually_confirmed=args.roles_visually_confirmed,
        )
        result = capture_supervised(
            args.raw_root, duration_s=args.duration, approved=True,
            base_index=args.base_index, wrist_index=args.wrist_index,
            can_adapter_bus=args.can_adapter_bus,
            can_adapter_address=args.can_adapter_address,
            manual_teach=True, lead_in_s=args.lead_in, camera_identity=identity,
        )
    else:
        result = capture(args.raw_root, duration_s=args.duration,
                         approved=False, base_index=args.base_index,
                         wrist_index=args.wrist_index, manual_teach=True, lead_in_s=args.lead_in,
                         can_adapter_bus=args.can_adapter_bus,
                         can_adapter_address=args.can_adapter_address)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "MANUAL_CAPTURE_RECORDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
