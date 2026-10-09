"""Bounded non-motion PiPER joint-limit query (0x472 -> 0x473).

Sends only the official parameter *query* for J2/J3. It never constructs the
PiPER interface or sends mode, enable, joint-target, gripper, or reset frames.
"""

from __future__ import annotations

import argparse
import json
import struct
import time

from laptop.diagnostics.can_diagnostic_session import CanDiagnosticSession
from laptop.transport.can_transport import CanFrame
from laptop.transport.diagnostic_queries import joint_limit_query


def request_payload(motor_num: int) -> bytes:
    if type(motor_num) is not int or motor_num not in (2, 3):
        raise ValueError("only J2/J3 read-only queries are permitted")
    return bytes((motor_num, 0x01, 0, 0, 0, 0, 0, 0))


def decode_limit_payload(data: bytes, expected_motor: int) -> dict:
    if len(data) != 8 or data[0] != expected_motor:
        raise ValueError("wrong motor or malformed limit response")
    maximum, minimum, speed = struct.unpack(">hhH", data[1:7])
    if minimum >= maximum or speed == 0 or speed == 0x7FFF:
        raise ValueError("invalid or unavailable hardware limit response")
    return {"motor_num": expected_motor, "minimum_deg": minimum / 10,
            "maximum_deg": maximum / 10, "maximum_rad_s": speed / 1000}


def query(*, approved: bool, timeout_s: float = 2.0) -> dict:
    if not approved:
        raise PermissionError("explicit non-motion query flag required")
    if not 0 < timeout_s <= 3:
        raise ValueError("query timeout must be bounded")
    sent = 0
    received: dict[int, dict] = {}
    query_events = []
    with CanDiagnosticSession(
        approved=True,
        preflight_timeout_s=min(timeout_s, 3.0),
        max_queries=2,
    ) as session:
        for motor in (2, 3):
            query_events.append(session.send_query(joint_limit_query(motor)))
            sent += 1
            deadline = time.perf_counter() + timeout_s
            while time.perf_counter() < deadline:
                frame = session.receive_frame(timeout_ms=100)
                if (not isinstance(frame, CanFrame) or frame.is_error_frame
                        or frame.is_extended_id or frame.arbitration_id != 0x473
                        or len(frame.data) != 8):
                    continue
                if frame.data[0] == motor:
                    received[motor] = decode_limit_payload(frame.data, motor)
                    break
            if motor not in received:
                break
    return {"status": "LIMIT_FEEDBACK_PRESENT" if len(received) == 2 else "INCOMPLETE",
            "limits": received, "can_data_tx_calls": sent,
            "tx_ids": ["0x472"] * sent, "safety_bridge_query_events": query_events,
            "command_tx_enabled": False, "robot_motion_commanded": False,
            "mode_or_enable_command_sent": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--send-read-only-queries", action="store_true")
    args = parser.parse_args()
    report = query(approved=args.send_read_only_queries)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "LIMIT_FEEDBACK_PRESENT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
