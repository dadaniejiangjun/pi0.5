#!/usr/bin/env python3
"""Bounded receive-only probe for one gs_usb CAN adapter.

This does not import a robot SDK or issue CAN frames. It changes only the USB
adapter's bit timing/mode, explicitly requiring hardware listen-only support.
Listen-only does not ACK CAN frames: with no other ACKing node on the bus, a
transmitter may retry continuously. The probe is blocked by default until an
independent ACKing peer is confirmed. Do not replace listen-only with normal
mode merely to bypass this guard.
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
    CanListenOnlyUnsupportedError,
    CanReceiveMode,
    GsUsbPiperCanTransport,
)


def probe(duration_s: float, bitrate: int, acknowledging_peer_confirmed: bool = False,
          *, transport: PiperCanTransport | None = None) -> dict:
    if not 0 < duration_s <= 60:
        raise ValueError("duration must be in (0, 60] seconds")
    if bitrate != 1_000_000:
        raise ValueError("this PiPER audit allows only the documented 1 Mbps CAN bitrate")
    if not acknowledging_peer_confirmed:
        return {
            "status": "BLOCKED_ACK_PEER_UNCONFIRMED",
            "reason": "listen-only will not ACK; a lone transmitter may retry continuously",
            "can_started": False,
            "robot_motion_commanded": False,
        }

    can_transport = (
        transport if transport is not None
        else GsUsbPiperCanTransport(receive_mode=CanReceiveMode.LISTEN_ONLY)
    )
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=can_transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
    )
    counts: collections.Counter[int] = collections.Counter()
    frame_count = 0
    first_frame_ns = None
    last_frame_ns = None
    start_ns = time.perf_counter_ns()
    try:
        try:
            bridge.start_read_only()
        except CanAdapterNotUniqueError as exc:
            return {"status": "EXPECTED_ADAPTER_NOT_UNIQUE",
                    "matching_count": exc.matching_count, "can_started": False}
        except CanBitrateError:
            return {"status": "BITRATE_UNSUPPORTED", "can_started": False}
        except CanListenOnlyUnsupportedError:
            return {"status": "LISTEN_ONLY_UNSUPPORTED", "can_started": False}
        start_ns = time.perf_counter_ns()
        end_ns = start_ns + int(duration_s * 1_000_000_000)
        while time.perf_counter_ns() < end_ns:
            frame = bridge.receive_frame(timeout_ms=100)
            if frame is None:
                continue
            now_ns = frame.received_monotonic_ns
            frame_count += 1
            counts[frame.arbitration_id] += 1
            first_frame_ns = now_ns if first_frame_ns is None else first_frame_ns
            last_frame_ns = now_ns
    finally:
        if bridge.state is not BridgeState.CLOSED:
            bridge.close()

    return {
        "status": "PASSIVE_FRAMES_OBSERVED" if frame_count else "NO_FRAMES_OBSERVED",
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),
        "adapter_usb_id": "1d50:606f",
        "bitrate": bitrate,
        "mode": "GS_CAN_MODE_LISTEN_ONLY",
        "safety_bridge_mode": bridge.mode.value,
        "can_owner": SafetyBridge.OWNER,
        "hardware_control_allowed": False,
        "can_tx_calls": 0,
        "robot_sdk_imported": False,
        "robot_motion_commanded": False,
        "frame_count": frame_count,
        "arbitration_id_counts": {f"0x{identifier:03X}": count for identifier, count in sorted(counts.items())},
        "first_frame_host_monotonic_ns": first_frame_ns,
        "last_frame_host_monotonic_ns": last_frame_ns,
        "raw_payload_saved": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--acknowledging-peer-confirmed", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = probe(args.duration, args.bitrate, args.acknowledging_peer_confirmed)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["status"] == "PASSIVE_FRAMES_OBSERVED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
