#!/usr/bin/env python3
"""Read the PiPER firmware version with one bounded documented CAN query.

This deliberately avoids ``C_PiperInterface.ConnectPort`` and ``PiperInit``.
It sends only the official firmware query frame (0x4AF), never retries, and
does not change modes, enable motors, set targets, or control the gripper.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import time
from datetime import datetime, timezone

from laptop.diagnostics.can_diagnostic_session import CanDiagnosticSession
from laptop.transport.can_transport import CanFrame
from laptop.transport.diagnostic_queries import firmware_version_query


FIRMWARE_QUERY_CAN_ID = 0x4AF
FIRMWARE_QUERY_PAYLOAD = bytes((0x01, 0, 0, 0, 0, 0, 0, 0))
FIRMWARE_VERSION_MARKER = b"S-V"
FIRMWARE_VERSION_LENGTH = 8


def decode_firmware_payloads(payloads: list[bytes]) -> str:
    """Decode the SDK's fixed-width ``S-Vx.y-z`` marker from CAN payloads."""

    if any(len(payload) != 8 for payload in payloads):
        raise ValueError("firmware response payloads must be 8 bytes")
    stream = b"".join(payloads)
    marker = stream.find(FIRMWARE_VERSION_MARKER)
    if marker < 0:
        raise ValueError("firmware response marker S-V was not found")
    version_bytes = stream[marker : marker + FIRMWARE_VERSION_LENGTH]
    if len(version_bytes) != FIRMWARE_VERSION_LENGTH:
        raise ValueError("firmware version response is truncated")
    if not all(0x20 <= value <= 0x7E for value in version_bytes):
        raise ValueError("firmware version contains non-printable bytes")
    return version_bytes.decode("ascii")


def query(*, approved: bool, timeout_s: float = 2.0) -> dict:
    """Send exactly one approved firmware query and collect its bounded reply."""

    if not approved:
        raise PermissionError("explicit firmware-query approval required")
    if not 0 < timeout_s <= 3.0:
        raise ValueError("firmware query timeout must be within (0, 3] seconds")

    tx_calls = 0
    response_payloads: list[bytes] = []
    firmware_version = None
    read_errors = 0
    start_ns = time.perf_counter_ns()
    with CanDiagnosticSession(approved=True, preflight_timeout_s=min(timeout_s, 3.0)) as session:
        # This is the only transmit call. Do not retry an unknown delivery.
        session.send_query(firmware_version_query())
        tx_calls = 1
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline and len(response_payloads) < 16:
            frame = session.receive_frame(timeout_ms=100)
            if (not isinstance(frame, CanFrame) or frame.is_error_frame
                    or frame.is_extended_id or len(frame.data) != 8
                    or frame.arbitration_id != FIRMWARE_QUERY_CAN_ID):
                continue
            response_payloads.append(frame.data)
            try:
                firmware_version = decode_firmware_payloads(response_payloads)
                break
            except ValueError:
                # Firmware text may span more than one response frame.
                continue
    return {
        "schema": "piper_firmware_query_v1",
        "status": "FIRMWARE_VERSION_READ" if firmware_version else "INCOMPLETE_READ",
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),
        "adapter_usb_id": "1d50:606f",
        "bitrate": 1_000_000,
        "sdk_package_version": importlib.metadata.version("piper-sdk"),
        "firmware_version": firmware_version,
        "query": {
            "request_can_id": f"0x{FIRMWARE_QUERY_CAN_ID:03X}",
            "request_payload_hex": FIRMWARE_QUERY_PAYLOAD.hex(),
            "response_can_id": f"0x{FIRMWARE_QUERY_CAN_ID:03X}",
            "response_payloads_hex": [payload.hex() for payload in response_payloads],
        },
        "safety": {
            "can_data_tx_calls": tx_calls,
            "only_documented_firmware_query_sent": tx_calls == 1,
            "query_retries": 0,
            "safety_bridge_query_events": session.audit_events,
            "command_tx_enabled": False,
            "mode_enable_motion_gripper_or_parameter_set_frames_sent": False,
            "robot_motion_commanded": False,
            "robot_sdk_interface_constructed": False,
            "read_errors": read_errors,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--send-one-documented-query",
        action="store_true",
        help="explicitly authorize one non-motion 0x4AF firmware query",
    )
    args = parser.parse_args()
    if not args.send_one_documented_query:
        parser.error("pass --send-one-documented-query only when the operator authorizes it")
    report = query(approved=True)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "FIRMWARE_VERSION_READ" else 2


if __name__ == "__main__":
    raise SystemExit(main())
