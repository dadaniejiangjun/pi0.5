#!/usr/bin/env python3
"""Read PiPER's live joint limits and state using bounded CAN queries.

Only documented parameter-read requests (0x472) are transmitted. This module
does not instantiate the PiPER SDK interface, call ConnectPort/PiperInit, or
send mode, enable, motion, gripper, reset, or configuration frames.
"""

from __future__ import annotations

import collections
import importlib.metadata
import json
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

from laptop.diagnostics.can_diagnostic_session import CanDiagnosticSession
from laptop.transport.can_transport import CanFrame
from laptop.transport.diagnostic_queries import joint_limit_query


JOINTS = tuple(range(1, 7))
ANGLE_QUERY = 0x01
ACCELERATION_QUERY = 0x02
ANGLE_RESPONSE_ID = 0x473
ACCELERATION_RESPONSE_ID = 0x47C
FEEDBACK_IDS = {0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8}


def request_payload(motor_num: int, search_content: int) -> bytes:
    if type(motor_num) is not int or motor_num not in JOINTS:
        raise ValueError("joint number must be an integer in [1, 6]")
    if type(search_content) is not int or search_content not in (ANGLE_QUERY, ACCELERATION_QUERY):
        raise ValueError("only documented angle/speed or acceleration queries are allowed")
    return bytes((motor_num, search_content, 0, 0, 0, 0, 0, 0))


def decode_limit_payload(data: bytes, expected_motor: int) -> dict:
    if len(data) != 8 or data[0] != expected_motor:
        raise ValueError("wrong motor or malformed angle/speed response")
    maximum, minimum, speed = struct.unpack(">hhH", data[1:7])
    if minimum >= maximum or not 0 < speed <= 3000:
        raise ValueError("invalid angle/speed response or outside documented speed range")
    return {
        "motor_num": expected_motor,
        "minimum_raw_0p1deg": minimum,
        "maximum_raw_0p1deg": maximum,
        "minimum_deg": minimum / 10,
        "maximum_deg": maximum / 10,
        "maximum_speed_raw_0p001_rad_s": speed,
        "maximum_rad_s": speed / 1000,
        "raw_payload_hex": data.hex(),
    }


def decode_acceleration_payload(data: bytes, expected_motor: int) -> dict:
    if len(data) != 8 or data[0] != expected_motor:
        raise ValueError("wrong motor or malformed acceleration response")
    acceleration = struct.unpack(">H", data[1:3])[0]
    if not 0 < acceleration < 0x7FFF:
        raise ValueError("invalid or unavailable acceleration response")
    return {
        "motor_num": expected_motor,
        "maximum_acceleration_raw": acceleration,
        "maximum_rad_s2_sdk_documented": acceleration / 1000,
        "raw_payload_hex": data.hex(),
    }


def _feedback_snapshot(parser, message, frame: CanFrame, can, counts, latest):
    can_id = int(frame.arbitration_id)
    payload = frame.data
    packet = can.Message(
        arbitration_id=can_id,
        data=payload,
        is_extended_id=frame.is_extended_id,
        timestamp=frame.received_monotonic_ns / 1_000_000_000,
    )
    if parser.DecodeMessage(packet, message):
        counts[can_id] += 1
        latest[can_id] = {
            "monotonic_ns": frame.received_monotonic_ns,
            "raw_payload_hex": payload.hex(),
        }


def query(*, approved: bool, per_request_timeout_s: float = 1.0) -> dict:
    if not approved:
        raise PermissionError("explicit CAN parameter-read approval required")
    if not 0 < per_request_timeout_s <= 1.5:
        raise ValueError("each read request timeout must be bounded to 1.5 seconds")

    import can
    from piper_sdk.piper_msgs.msg_v2 import PiperMessage
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

    parser = C_PiperParserV2()
    message = PiperMessage()
    response_ids = {ANGLE_QUERY: ANGLE_RESPONSE_ID, ACCELERATION_QUERY: ACCELERATION_RESPONSE_ID}
    results: dict[str, dict] = {"limits": {}, "acceleration": {}}
    feedback_counts: collections.Counter[int] = collections.Counter()
    latest_feedback: dict[int, dict] = {}
    decode_errors = 0
    tx_payloads: list[str] = []
    query_events: list[dict] = []
    start_ns = time.perf_counter_ns()
    with CanDiagnosticSession(
        approved=True, preflight_timeout_s=3.0, max_queries=len(JOINTS) * 2
    ) as session:
        start_ns = time.perf_counter_ns()
        for frame in session.preflight_frames:
            try:
                _feedback_snapshot(
                    parser, message, frame, can, feedback_counts, latest_feedback
                )
            except (ValueError, TypeError, IndexError):
                decode_errors += 1

        for motor_num in JOINTS:
            for search_content in (ANGLE_QUERY, ACCELERATION_QUERY):
                query = joint_limit_query(
                    motor_num, acceleration=search_content == ACCELERATION_QUERY
                )
                query_events.append(session.send_query(query))
                tx_payloads.append(query.payload.hex())
                response_id = response_ids[search_content]
                deadline = time.perf_counter() + per_request_timeout_s
                got_reply = False
                while time.perf_counter() < deadline:
                    frame = session.receive_frame(timeout_ms=100)
                    if (not isinstance(frame, CanFrame) or frame.is_error_frame
                            or frame.is_extended_id or len(frame.data) != 8):
                        continue
                    can_id = int(frame.arbitration_id)
                    if can_id in FEEDBACK_IDS:
                        try:
                            _feedback_snapshot(
                                parser, message, frame, can, feedback_counts, latest_feedback
                            )
                        except (ValueError, TypeError, IndexError):
                            decode_errors += 1
                        continue
                    if can_id != response_id or frame.data[0] != motor_num:
                        continue
                    raw = frame.data
                    decoded = (
                        decode_limit_payload(raw, motor_num)
                        if search_content == ANGLE_QUERY
                        else decode_acceleration_payload(raw, motor_num)
                    )
                    decoded["response_can_id"] = f"0x{response_id:03X}"
                    key = "limits" if search_content == ANGLE_QUERY else "acceleration"
                    results[key][str(motor_num)] = decoded
                    got_reply = True
                    break
                if not got_reply:
                    break
            if (
                str(motor_num) not in results["limits"]
                or str(motor_num) not in results["acceleration"]
            ):
                break
    feedback_complete = all(feedback_counts[can_id] > 0 for can_id in FEEDBACK_IDS)
    joint_fields = [getattr(message.arm_joint_feedback, f"joint_{index}") for index in JOINTS]
    status = message.arm_status_msgs
    gripper = message.gripper_feedback
    complete = len(results["limits"]) == 6 and len(results["acceleration"]) == 6
    return {
        "schema": "piper_live_joint_parameters_v1",
        "status": "ALL_SIX_JOINTS_READ" if complete else "INCOMPLETE_READ",
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),
        "adapter_usb_id": "1d50:606f",
        "bitrate": 1_000_000,
        "sdk_package_version": importlib.metadata.version("piper-sdk"),
        "query_protocol": {
            "request_can_id": "0x472",
            "limit_response_can_id": "0x473",
            "acceleration_response_can_id": "0x47C",
            "request_frames_sent": len(tx_payloads),
            "request_payloads_hex": tx_payloads,
            "safety_bridge_query_events": query_events,
            "joint_numbers": list(JOINTS),
            "search_content_values": {"angle_and_speed": "0x01", "acceleration": "0x02"},
        },
        "limits": results["limits"],
        "acceleration": results["acceleration"],
        "live_feedback": {
            "status": "ALL_CHANNELS_SEEN" if feedback_complete else "PARTIAL_CHANNELS",
            "frame_counts": {
                f"0x{can_id:03X}": feedback_counts[can_id]
                for can_id in sorted(FEEDBACK_IDS)
            },
            "last_frame_monotonic_ns": {
                f"0x{can_id:03X}": latest_feedback.get(can_id, {}).get("monotonic_ns")
                for can_id in sorted(FEEDBACK_IDS)
            },
            "last_raw_payload_hex": {
                f"0x{can_id:03X}": latest_feedback.get(can_id, {}).get("raw_payload_hex")
                for can_id in sorted(FEEDBACK_IDS)
            },
            "ctrl_mode": str(status.ctrl_mode),
            "arm_status": str(status.arm_status),
            "teach_status": str(status.teach_status),
            "error_code": status.err_code,
            "joint_feedback_raw_0p001deg": (
                joint_fields
                if any(feedback_counts[cid] for cid in (0x2A5, 0x2A6, 0x2A7))
                else None
            ),
            "joint_feedback_nominal_deg": (
                [round(value / 1000, 3) for value in joint_fields]
                if any(feedback_counts[cid] for cid in (0x2A5, 0x2A6, 0x2A7))
                else None
            ),
            "gripper_feedback_raw": {
                "angle_or_stroke": gripper.grippers_angle,
                "effort": gripper.grippers_effort,
                "status_code": gripper.status_code,
            } if feedback_counts[0x2A8] else None,
            "decode_errors": decode_errors,
            "short_usb_packets": getattr(session.bridge.transport, "short_usb_packets", 0),
        },
        "safety": {
            "only_documented_0x472_read_requests_sent": True,
            "can_data_tx_calls": len(tx_payloads),
            "command_tx_enabled": False,
            "tx_ids": ["0x472"] * len(tx_payloads),
            "mode_enable_reset_joint_target_gripper_or_setter_frames_sent": False,
            "robot_motion_commanded": False,
            "robot_sdk_interface_constructed": False,
        },
        "unit_note": (
            "Limit angle: 0.1 deg; speed: 0.001 rad/s. The SDK query-response "
            "documentation reports acceleration as 0.001 rad/s^2; raw payloads "
            "are retained for independent review because setter documentation "
            "uses a different acceleration scale."
        ),
    }


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--send-documented-read-queries", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--per-request-timeout", type=float, default=1.0)
    args = parser.parse_args()
    report = query(
        approved=args.send_documented_read_queries,
        per_request_timeout_s=args.per_request_timeout,
    )
    serialized = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if report["status"] == "ALL_SIX_JOINTS_READ" else 1


if __name__ == "__main__":
    raise SystemExit(main())
