#!/usr/bin/env python3
"""Read PiPER gripper travel configuration after fresh feedback is present.

The command never constructs ``C_PiperInterface_V2`` or calls ``ConnectPort``.
It listens first and sends one documented 0x477/Byte-0=0x04 query only after
all five required feedback IDs have decoded recently. It never sends a setter,
mode, enable, reset, motion, or gripper-control frame.
"""

from __future__ import annotations

import argparse
import collections
import importlib.metadata
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping

from laptop.diagnostics.can_diagnostic_session import CanDiagnosticSession
from laptop.transport.can_transport import CanFrame
from laptop.transport.diagnostic_queries import gripper_parameters_query


QUERY_CAN_ID = 0x477
RESPONSE_CAN_ID = 0x47E
QUERY_PAYLOAD = bytes((0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))
REQUIRED_FEEDBACK_IDS = frozenset((0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8))
MAX_FEEDBACK_AGE_NS = 250_000_000


def decode_gripper_parameter_payload(payload: bytes) -> dict:
    """Decode the three documented 0x47E fields and retain the raw bytes."""

    if not isinstance(payload, bytes) or len(payload) != 8:
        raise ValueError("gripper parameter feedback must contain exactly 8 bytes")
    teaching_range_percent, max_range_config_mm, teaching_friction = payload[:3]
    return {
        "teaching_range_percent": teaching_range_percent,
        "teaching_range_percent_documented_valid": 100 <= teaching_range_percent <= 200,
        "max_range_config_mm": max_range_config_mm,
        "max_range_config_documented_valid": max_range_config_mm in (0, 70, 100),
        "teaching_friction": teaching_friction,
        "teaching_friction_documented_valid": teaching_friction == 0 or 1 <= teaching_friction <= 10,
        "reserved_payload_hex": payload[3:].hex(),
        "raw_payload_hex": payload.hex(),
    }


def evaluate_feedback_preflight(
    frame_counts: Mapping[int, int],
    last_frame_ns: Mapping[int, int],
    *,
    now_ns: int,
    max_age_ns: int = MAX_FEEDBACK_AGE_NS,
) -> dict:
    """Require each state channel to have a fresh, decoded frame before TX."""

    if (type(now_ns) is not int or now_ns <= 0
            or type(max_age_ns) is not int or not 0 < max_age_ns <= MAX_FEEDBACK_AGE_NS):
        raise ValueError("invalid feedback preflight clock or freshness bound")
    missing: list[int] = []
    stale: list[int] = []
    invalid: list[int] = []
    ages_ms: dict[str, float] = {}
    for can_id in sorted(REQUIRED_FEEDBACK_IDS):
        count = frame_counts.get(can_id, 0)
        timestamp = last_frame_ns.get(can_id)
        if type(count) is not int or count < 1 or type(timestamp) is not int or timestamp <= 0:
            missing.append(can_id)
            continue
        age_ns = now_ns - timestamp
        if age_ns < 0:
            invalid.append(can_id)
        else:
            ages_ms[f"0x{can_id:03X}"] = round(age_ns / 1_000_000, 3)
            if age_ns > max_age_ns:
                stale.append(can_id)
    return {
        "ready": not missing and not stale and not invalid,
        "missing_ids": [f"0x{can_id:03X}" for can_id in missing],
        "stale_ids": [f"0x{can_id:03X}" for can_id in stale],
        "future_or_invalid_ids": [f"0x{can_id:03X}" for can_id in invalid],
        "age_ms": ages_ms,
        "freshness_limit_ms": max_age_ns / 1_000_000,
    }


def dispatch_query_if_feedback_fresh(
    send_frame: Callable[[int, bytes], None],
    frame_counts: Mapping[int, int],
    last_frame_ns: Mapping[int, int],
    *,
    now_ns: int,
) -> dict:
    """Send exactly one whitelisted query only when the preflight passes."""

    preflight = evaluate_feedback_preflight(frame_counts, last_frame_ns, now_ns=now_ns)
    if not preflight["ready"]:
        return {"sent": False, "preflight": preflight}
    send_frame(QUERY_CAN_ID, QUERY_PAYLOAD)
    return {"sent": True, "preflight": preflight}


def query(*, approved: bool, preflight_timeout_s: float = 3.0,
          response_timeout_s: float = 1.5) -> dict:
    """Bounded one-shot query; never transmits if feedback preflight fails."""

    if not approved:
        raise PermissionError("explicit gripper-parameter query approval required")
    if not 0 < preflight_timeout_s <= 3.0:
        raise ValueError("feedback preflight timeout must be within (0, 3] seconds")
    if not 0 < response_timeout_s <= 2.0:
        raise ValueError("parameter response timeout must be within (0, 2] seconds")

    import can
    from piper_sdk.piper_msgs.msg_v2 import PiperMessage
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

    parser = C_PiperParserV2()
    message = PiperMessage()
    counts: collections.Counter[int] = collections.Counter()
    latest: dict[int, int] = {}
    decode_errors = 0
    tx_calls = 0
    response: bytes | None = None
    query_events: list[dict] = []
    start_ns = time.perf_counter_ns()
    preflight_result = evaluate_feedback_preflight(counts, latest, now_ns=start_ns)
    with CanDiagnosticSession(approved=True, preflight_timeout_s=preflight_timeout_s) as session:
        def account_feedback(frame: CanFrame) -> None:
            nonlocal decode_errors, preflight_result
            if (frame.is_error_frame or frame.is_extended_id or len(frame.data) != 8
                    or frame.arbitration_id not in REQUIRED_FEEDBACK_IDS):
                return
            arrival_ns = frame.received_monotonic_ns
            try:
                packet = can.Message(
                    arbitration_id=frame.arbitration_id,
                    data=frame.data,
                    is_extended_id=False,
                    timestamp=arrival_ns / 1_000_000_000,
                )
                if not parser.DecodeMessage(packet, message):
                    decode_errors += 1
                    return
            except (ValueError, TypeError, IndexError):
                decode_errors += 1
                return
            counts[frame.arbitration_id] += 1
            latest[frame.arbitration_id] = arrival_ns
            preflight_result = evaluate_feedback_preflight(
                counts, latest, now_ns=time.perf_counter_ns()
            )

        for frame in session.preflight_frames:
            account_feedback(frame)
        preflight_deadline = time.monotonic() + preflight_timeout_s
        while not preflight_result["ready"] and time.monotonic() < preflight_deadline:
            frame = session.receive_frame(timeout_ms=100)
            if isinstance(frame, CanFrame):
                account_feedback(frame)

        if preflight_result["ready"]:
            def send_checked_query(can_id: int, payload: bytes) -> None:
                if can_id != QUERY_CAN_ID or payload != QUERY_PAYLOAD:
                    raise ValueError("only the exact gripper parameter getter is allowed")
                query_events.append(session.send_query(gripper_parameters_query()))

            dispatch = dispatch_query_if_feedback_fresh(
                send_checked_query,
                counts,
                latest,
                now_ns=time.perf_counter_ns(),
            )
            preflight_result = dispatch["preflight"]
            if dispatch["sent"]:
                tx_calls = 1
                response_deadline = time.monotonic() + response_timeout_s
                while time.monotonic() < response_deadline:
                    frame = session.receive_frame(timeout_ms=100)
                    if (isinstance(frame, CanFrame) and not frame.is_error_frame
                            and not frame.is_extended_id and len(frame.data) == 8
                            and frame.arbitration_id == RESPONSE_CAN_ID):
                        response = frame.data
                        break

    elapsed_s = round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3)
    status = (
        "GRIPPER_PARAMETERS_READ" if response is not None
        else "PARAMETER_RESPONSE_TIMEOUT" if tx_calls == 1
        else "FEEDBACK_PREFLIGHT_FAILED"
    )
    return {
        "schema": "piper_gripper_parameter_query_v1",
        "status": status,
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": elapsed_s,
        "adapter_usb_id": "1d50:606f",
        "bitrate": 1_000_000,
        "sdk_package_version": importlib.metadata.version("piper-sdk"),
        "feedback_preflight": {
            **preflight_result,
            "frame_counts": {f"0x{can_id:03X}": counts[can_id]
                             for can_id in sorted(REQUIRED_FEEDBACK_IDS)},
            "decode_errors": decode_errors,
            "short_usb_packets": getattr(session.bridge.transport, "short_usb_packets", 0),
            "safety_bridge_query_events": query_events,
        },
        "query": {
            "request_can_id": f"0x{QUERY_CAN_ID:03X}",
            "request_payload_hex": QUERY_PAYLOAD.hex(),
            "response_can_id": f"0x{RESPONSE_CAN_ID:03X}",
            "response_payload_hex": response.hex() if response is not None else None,
            "parameters": decode_gripper_parameter_payload(response) if response is not None else None,
        },
        "safety": {
            "can_data_tx_calls": tx_calls,
            "command_tx_enabled": False,
            "query_frames_sent": tx_calls,
            "only_documented_0x477_read_query_sent": tx_calls <= 1,
            "setter_frames_sent": False,
            "mode_enable_reset_motion_or_gripper_frames_sent": False,
            "robot_sdk_interface_constructed": False,
            "robot_motion_commanded": False,
            "configured_travel_is_physical_range_proof": False,
            "gripper_units_hardware_verified": False,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--send-one-gripper-parameter-query",
        action="store_true",
        help="explicitly authorize at most one non-motion 0x477/0x04 query after live feedback preflight",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.send_one_gripper_parameter_query:
        parser.error("pass --send-one-gripper-parameter-query only when the operator authorizes it")
    report = query(approved=True)
    serialized = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as handle:
            handle.write(serialized)
    print(serialized, end="")
    return 0 if report["status"] == "GRIPPER_PARAMETERS_READ" else 1


if __name__ == "__main__":
    raise SystemExit(main())
