"""Bounded read-only PiPER six-motor enable status from 0x261..0x266.

The installed official SDK maps low-speed feedback byte 5 bit 6 to
driver_enable_status. This module sends no CAN data frames.
"""

from __future__ import annotations

import argparse
import json
import time

from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import CanFrame, PiperCanTransport
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


FEEDBACK_IDS = frozenset(range(0x261, 0x267))


def decode_motor_enable(can_id: int, data: bytes) -> tuple[int, bool, int]:
    if can_id not in FEEDBACK_IDS or len(data) != 8:
        raise ValueError("expected eight-byte low-speed motor feedback")
    status_code = data[5]
    return can_id - 0x260, bool(status_code & 0x40), status_code


def probe(duration_s: float = 3.0, *, approved: bool = False,
          transport: PiperCanTransport | None = None) -> dict:
    if not 0 < duration_s <= 5:
        raise ValueError("bounded duration required")
    if not approved:
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "can_started": False,
                "can_data_tx_calls": 0}
    can_transport = transport if transport is not None else GsUsbPiperCanTransport()
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=can_transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
    )
    seen: dict[int, list[bool]] = {motor: [] for motor in range(1, 7)}
    last_codes: dict[int, int] = {}
    start_ns = time.perf_counter_ns()
    try:
        bridge.start_read_only()
        start_ns = time.perf_counter_ns()
        deadline = time.perf_counter() + duration_s
        while time.perf_counter() < deadline:
            frame = bridge.receive_frame(timeout_ms=100)
            if frame is None or not isinstance(frame, CanFrame):
                continue
            if (not frame.is_error_frame and not frame.is_extended_id
                    and frame.arbitration_id in FEEDBACK_IDS and len(frame.data) == 8):
                motor, enabled, code = decode_motor_enable(
                    frame.arbitration_id, frame.data)
                seen[motor].append(enabled)
                last_codes[motor] = code
    finally:
        if bridge.state is not BridgeState.CLOSED:
            bridge.close()
    enough = all(len(values) >= 2 for values in seen.values())
    consistent = all(len(set(values)) == 1 for values in seen.values()) if enough else False
    return {
        "status": ("ALL_SIX_ENABLED" if enough and consistent
                   and all(values[-1] for values in seen.values()) else
                   "INCOMPLETE_OR_NOT_ALL_ENABLED"),
        "motor_feedback_counts": {str(m): len(seen[m]) for m in range(1, 7)},
        "motor_enabled": {str(m): seen[m][-1] if seen[m] else None for m in range(1, 7)},
        "last_foc_status_code": {str(m): last_codes.get(m) for m in range(1, 7)},
        "consistent_during_probe": consistent,
        "duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),
        "safety_bridge_mode": bridge.mode.value,
        "can_owner": SafetyBridge.OWNER,
        "hardware_control_allowed": False,
        "command_tx_enabled": False,
        "can_data_tx_calls": 0,
        "robot_motion_commanded": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor-feedback-read-approved", action="store_true")
    args = parser.parse_args()
    result = probe(approved=args.motor_feedback_read_approved)
    print(json.dumps(result, indent=2))
