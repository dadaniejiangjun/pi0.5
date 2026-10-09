#!/usr/bin/env python3
"""Five-second CAN receive probe that permits hardware ACK, never data TX.

Unlike listen-only, normal CAN receive acknowledges valid frames. This file
contains no CAN frame write or robot SDK call. The operator must explicitly
approve ACK-only mode at each invocation. Never use this as a motion probe.
"""

from __future__ import annotations

import argparse
import collections
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import PiperCanTransport
from laptop.transport.gs_usb_piper_can import (
    CanAdapterNotUniqueError,
    CanBitrateError,
    GsUsbPiperCanTransport,
)


def probe(duration_s: float, approved: bool = False,
          transport: PiperCanTransport | None = None) -> dict:
    if not 0 < duration_s <= 5:
        raise ValueError("duration must be in (0, 5] seconds")
    if not approved:
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "can_started": False, "can_data_tx_calls": 0}

    can_transport = transport if transport is not None else GsUsbPiperCanTransport()
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=can_transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
    )
    counts: collections.Counter[int] = collections.Counter()
    error_frames = 0
    start_ns = time.perf_counter_ns()
    try:
        try:
            bridge.start_read_only()
        except CanAdapterNotUniqueError as exc:
            return {"status": "EXPECTED_ADAPTER_NOT_UNIQUE",
                    "matching_count": exc.matching_count, "can_started": False}
        except CanBitrateError:
            return {"status": "BITRATE_UNSUPPORTED", "can_started": False}
        start_ns = time.perf_counter_ns()
        end_ns = start_ns + int(duration_s * 1_000_000_000)
        while time.perf_counter_ns() < end_ns:
            frame = bridge.receive_frame(timeout_ms=100)
            if frame is None:
                continue
            if frame.is_error_frame:
                error_frames += 1
            else:
                counts[frame.arbitration_id] += 1
    finally:
        if bridge.state is not BridgeState.CLOSED:
            bridge.close()

    return {
        "status": "ACK_ONLY_FRAMES_OBSERVED" if counts else "NO_DATA_FRAMES_OBSERVED",
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),
        "adapter_usb_id": "1d50:606f",
        "bitrate": 1_000_000,
        "mode": "normal_receive_ack_only",
        "safety_bridge_mode": bridge.mode.value,
        "can_owner": SafetyBridge.OWNER,
        "hardware_control_allowed": False,
        "command_tx_enabled": False,
        "can_data_tx_calls": 0,
        "robot_sdk_imported": False,
        "robot_motion_commanded": False,
        "data_frame_count": sum(counts.values()),
        "error_frame_count": error_frames,
        "arbitration_id_counts": {f"0x{identifier:03X}": count for identifier, count in sorted(counts.items())},
        "raw_payload_saved": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--ack-only-approved", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = probe(args.duration, args.ack_only_approved)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["status"] == "ACK_ONLY_FRAMES_OBSERVED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
