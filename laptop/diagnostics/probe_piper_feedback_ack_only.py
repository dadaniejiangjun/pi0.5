#!/usr/bin/env python3
"""Decode bounded PiPER feedback without transmitting CAN data frames.

The USB-CAN adapter is in normal receive mode, so hardware ACKs are emitted.
Only the official piper_sdk V2 parser is used; no SDK interface/ConnectPort or
robot command method is constructed. This is not firmware or unit validation.
"""

from __future__ import annotations

import argparse
import collections
import importlib.metadata
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from laptop.robot_client.feedback import sdk_feedback_snapshot
from laptop.piper.unit_conversion import joint_raw_to_nominal_rad
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import PiperCanTransport
from laptop.transport.gs_usb_piper_can import (
    CanAdapterNotUniqueError,
    CanBitrateError,
    CanReceiveMode,
    GsUsbPiperCanTransport,
    ensure_libusb_backend as _ensure_libusb_backend,
)


JOINT_COMMAND_IDS = frozenset((0x155, 0x156, 0x157))
GRIPPER_COMMAND_ID = 0x159
COMMAND_IDS = JOINT_COMMAND_IDS | {GRIPPER_COMMAND_ID}


def probe(duration_s: float, approved: bool = False, feedback_log: Path | None = None,
          *, max_duration_s: float = 5,
          on_receiver_ready: Callable[[], None] | None = None,
          transport: PiperCanTransport | None = None,
          command_log: Path | None = None,
          adapter_bus: int | None = None,
          adapter_address: int | None = None,
          receive_mode: CanReceiveMode = CanReceiveMode.NORMAL_ACKING) -> dict:
    # Standalone CLI remains capped at five seconds. Bounded capture callers
    # may explicitly request up to sixty seconds for a complete task episode.
    if not 0 < max_duration_s <= 60 or not 0 < duration_s <= max_duration_s:
        raise ValueError("duration exceeds the bounded ACK-only read limit")
    if not approved:
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "can_started": False, "can_data_tx_calls": 0}

    import can
    from piper_sdk.piper_msgs.msg_v2 import CanIDPiper, PiperMessage
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

    selected_ids = {
        CanIDPiper.ARM_STATUS_FEEDBACK.value,
        CanIDPiper.ARM_JOINT_FEEDBACK_12.value,
        CanIDPiper.ARM_JOINT_FEEDBACK_34.value,
        CanIDPiper.ARM_JOINT_FEEDBACK_56.value,
        CanIDPiper.ARM_GRIPPER_FEEDBACK.value,
    }
    parser = C_PiperParserV2()
    message = PiperMessage()
    counts: collections.Counter[int] = collections.Counter()
    command_counts: collections.Counter[int] = collections.Counter()
    decode_errors = 0
    short_usb_packets = 0
    start_ns = time.perf_counter_ns()
    log_handle = None
    command_log_handle = None
    if (adapter_bus is None) != (adapter_address is None):
        raise ValueError("adapter_bus and adapter_address must be provided together")
    if transport is not None and (adapter_bus is not None or adapter_address is not None):
        raise ValueError("USB adapter selector cannot be combined with an injected transport")
    if command_log is not None and receive_mode is not CanReceiveMode.LISTEN_ONLY:
        raise ValueError("raw command capture requires hardware listen-only mode")
    can_transport = transport if transport is not None else GsUsbPiperCanTransport(
        receive_mode=receive_mode,
        adapter_bus=adapter_bus,
        adapter_address=adapter_address,
    )
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=can_transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
    )
    try:
        if feedback_log is not None:
            feedback_log = Path(feedback_log)
            feedback_log.parent.mkdir(parents=True, exist_ok=True)
            log_handle = feedback_log.open("x", encoding="utf-8")
        if command_log is not None:
            command_log = Path(command_log)
            command_log.parent.mkdir(parents=True, exist_ok=True)
            command_log_handle = command_log.open("x", encoding="utf-8")
        try:
            bridge.start_read_only()
        except CanAdapterNotUniqueError as exc:
            return {
                "status": "EXPECTED_ADAPTER_NOT_UNIQUE",
                "matching_count": exc.matching_count,
                "can_started": False,
                "can_data_tx_calls": 0,
            }
        except CanBitrateError:
            return {
                "status": "BITRATE_UNSUPPORTED",
                "can_started": False,
                "can_data_tx_calls": 0,
            }
        start_ns = time.perf_counter_ns()
        if on_receiver_ready is not None:
            on_receiver_ready()
        end_ns = start_ns + int(duration_s * 1_000_000_000)
        while time.perf_counter_ns() < end_ns:
            frame = bridge.receive_frame(timeout_ms=100)
            if frame is None:
                continue
            can_id = frame.arbitration_id
            if frame.is_error_frame or len(frame.data) != 8:
                continue
            if (can_id in COMMAND_IDS and not frame.is_extended_id
                    and command_log_handle is not None):
                arrival_ns = frame.received_monotonic_ns
                command_counts[can_id] += 1
                command_log_handle.write(json.dumps({
                    "monotonic_ns": arrival_ns,
                    "can_id": f"0x{can_id:03X}",
                    "raw_payload_hex": frame.data.hex(),
                    "is_extended_id": frame.is_extended_id,
                    "is_error_frame": frame.is_error_frame,
                    "capture_source": "passive_usb_can_bus_receive",
                    "direction_verified": False,
                }, separators=(",", ":")) + "\n")
                continue
            if can_id not in selected_ids:
                continue
            try:
                arrival_ns = frame.received_monotonic_ns
                payload = frame.data
                parsed = can.Message(
                    arbitration_id=can_id,
                    data=payload,
                    is_extended_id=frame.is_extended_id,
                    timestamp=arrival_ns / 1_000_000_000,
                )
                if parser.DecodeMessage(parsed, message):
                    counts[can_id] += 1
                    if log_handle is not None:
                        decoded_fields = sdk_feedback_snapshot(message, gripper_details=True)
                        log_handle.write(json.dumps({
                            "monotonic_ns": arrival_ns, "can_id": f"0x{can_id:03X}",
                            "raw_payload_hex": payload.hex(),
                            "joint_raw_0p001deg_nominal": decoded_fields["joint_raw"],
                            "gripper_raw": decoded_fields["gripper_raw"],
                            "status": decoded_fields["status"],
                        }, separators=(",", ":")) + "\n")
            except (ValueError, TypeError, IndexError):
                decode_errors += 1
    finally:
        if log_handle is not None:
            log_handle.close()
        if command_log_handle is not None:
            command_log_handle.close()
        short_usb_packets = getattr(can_transport, "short_usb_packets", 0)
        if bridge.state is not BridgeState.CLOSED:
            bridge.close()

    required = selected_ids
    complete = all(counts[can_id] > 0 for can_id in required)
    joints_raw = [getattr(message.arm_joint_feedback, f"joint_{index}") for index in range(1, 7)]
    return {
        "status": "FEEDBACK_FIELDS_PRESENT" if complete and not decode_errors else "INCOMPLETE_FEEDBACK",
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),
        "timestamp_clock": "python_perf_counter_ns_QueryPerformanceCounter_on_Windows",
        "adapter_usb_id": "1d50:606f",
        "selected_adapter_usb_identity": (
            {"bus": adapter_bus, "address": adapter_address}
            if adapter_bus is not None else None
        ),
        "mode": ("listen_only_receive" if receive_mode is CanReceiveMode.LISTEN_ONLY
                 else "normal_receive_ack_only"),
        "safety_bridge_mode": bridge.mode.value,
        "can_owner": SafetyBridge.OWNER,
        "hardware_control_allowed": False,
        "command_tx_enabled": False,
        "can_data_tx_calls": 0,
        "robot_sdk_interface_constructed": False,
        "robot_motion_commanded": False,
        "parser": "piper_sdk.C_PiperParserV2.DecodeMessage",
        "parser_package_version": importlib.metadata.version("piper-sdk"),
        "firmware_queried": False,
        "firmware_hardware_verified": False,
        "units_hardware_verified": False,
        "selected_id_counts": {f"0x{can_id:03X}": counts[can_id] for can_id in sorted(required)},
        "observed_command_id_counts": {
            f"0x{can_id:03X}": command_counts[can_id] for can_id in sorted(COMMAND_IDS)
        },
        "command_log": str(command_log) if command_log is not None else None,
        "command_log_capture_source": "passive_usb_can_bus_receive" if command_log is not None else None,
        "command_direction_verified": False,
        "command_targets_decoded": False,
        "command_tx_enabled": False,
        "can_data_tx_calls": 0,
        "decode_errors": decode_errors,
        "short_usb_packets": short_usb_packets,
        "last_status": {
            "ctrl_mode": str(message.arm_status_msgs.ctrl_mode),
            "arm_status": str(message.arm_status_msgs.arm_status),
            "move_mode": str(message.arm_status_msgs.mode_feed),
            "teach_status": str(message.arm_status_msgs.teach_status),
            "err_code": message.arm_status_msgs.err_code,
        },
        "last_joint_raw_0p001deg": joints_raw,
        "last_joint_rad_nominal_only": [round(joint_raw_to_nominal_rad(value), 6) for value in joints_raw],
        "last_gripper_raw": {
            "angle_or_stroke": message.gripper_feedback.grippers_angle,
            "effort": message.gripper_feedback.grippers_effort,
            "status_code": message.gripper_feedback.status_code,
        },
        "raw_payload_saved": feedback_log is not None,
        "feedback_log": str(feedback_log) if feedback_log is not None else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--feedback-read-approved", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--feedback-log", type=Path, help="Exclusive-create JSONL with per-message host timestamps")
    parser.add_argument("--command-log", type=Path,
                        help="Exclusive-create raw JSONL of observed PiPER target CAN IDs; no TX")
    parser.add_argument("--adapter-bus", type=int,
                        help="Exact USB bus of the passive candleLight receiver")
    parser.add_argument("--adapter-address", type=int,
                        help="Exact USB address of the passive candleLight receiver")
    parser.add_argument("--listen-only", action="store_true",
                        help="Require the adapter's hardware listen-only mode")
    args = parser.parse_args()
    result = probe(args.duration, args.feedback_read_approved, args.feedback_log,
                   command_log=args.command_log, adapter_bus=args.adapter_bus,
                   adapter_address=args.adapter_address,
                   receive_mode=(CanReceiveMode.LISTEN_ONLY if args.listen_only
                                 else CanReceiveMode.NORMAL_ACKING))
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["status"] == "FEEDBACK_FIELDS_PRESENT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
