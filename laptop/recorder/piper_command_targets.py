"""Validate passive PiPER command-frame logs and derive timestamped targets.

This module parses only received raw CAN records. It never opens a transport,
emits a frame, or turns observed state feedback into an action target.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct


JOINT_COMMAND_IDS = (0x155, 0x156, 0x157)
GRIPPER_COMMAND_ID = 0x159
COMMAND_IDS = frozenset((*JOINT_COMMAND_IDS, GRIPPER_COMMAND_ID))
JOINT_DEGREES_PER_RAW = 0.001
GRIPPER_METERS_PER_RAW = 1e-6
MAX_JOINT_TRIPLET_SPAN_NS = 50_000_000
ACTION_TARGET_SOURCE = "observed_piper_can_command_frames"
ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_meter"


@dataclass(frozen=True)
class CommandTarget:
    """Last observed full absolute target, with the raw frames that supplied it."""

    timestamp_ns: int
    action_timestamp_ns: int
    target_raw: tuple[int, int, int, int, int, int, int]
    target: tuple[float, float, float, float, float, float, float]
    joint_command_refs: tuple[dict, dict, dict]
    gripper_command_ref: dict

    def as_action_row(self, frame_index: int) -> dict:
        return {
            "frame_index": frame_index,
            "action_target_timestamp_ns": self.action_timestamp_ns,
            "action": list(self.target),
            "action_target_raw": list(self.target_raw),
            "action_source_timestamp_ns": self.timestamp_ns,
            "action_source_refs": {
                "joint_command_frames": list(self.joint_command_refs),
                "gripper_command_frame": self.gripper_command_ref,
            },
        }


def _can_id(value: object) -> int:
    if not isinstance(value, str) or not value.startswith("0x"):
        raise ValueError("command log can_id must be a 0x-prefixed string")
    try:
        result = int(value, 16)
    except ValueError as exc:
        raise ValueError("invalid command log CAN ID") from exc
    if result not in COMMAND_IDS:
        raise ValueError(f"unexpected CAN ID in command-only log: {value}")
    return result


def _read_command_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    last_timestamp = 0
    with Path(path).open(encoding="utf-8") as handle:
        for line_index, line in enumerate(handle):
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError("command log rows must be JSON objects")
            timestamp = row.get("monotonic_ns")
            if type(timestamp) is not int or timestamp <= last_timestamp:
                raise ValueError("command log timestamps must increase strictly")
            if row.get("is_extended_id") is not False or row.get("is_error_frame") is not False:
                raise ValueError("only standard non-error CAN command frames are accepted")
            if (row.get("capture_source") != "passive_usb_can_bus_receive"
                    or row.get("direction_verified") is not False):
                raise ValueError("command provenance must identify passive CAN receive and unverified direction")
            can_id = _can_id(row.get("can_id"))
            payload_hex = row.get("raw_payload_hex")
            if not isinstance(payload_hex, str) or len(payload_hex) != 16:
                raise ValueError("command frame payload must contain exactly 8 bytes")
            try:
                payload = bytes.fromhex(payload_hex)
            except ValueError as exc:
                raise ValueError("command frame payload is not valid hex") from exc
            if len(payload) != 8:
                raise ValueError("command frame payload must contain exactly 8 bytes")
            rows.append({
                "line_index": line_index,
                "monotonic_ns": timestamp,
                "can_id": can_id,
                "raw_payload_hex": payload.hex(),
                "payload": payload,
            })
            last_timestamp = timestamp
    if not rows:
        raise ValueError("command log is empty")
    return rows


def decode_command_timeline(
    path: Path,
    *,
    max_joint_triplet_span_ns: int = MAX_JOINT_TRIPLET_SPAN_NS,
) -> list[CommandTarget]:
    """Decode complete joint triplets and enabled gripper targets.

    Joint packets are grouped only when all three protocol IDs arrive inside
    the configured span. A missing, duplicated, reordered-across-cycle, or
    partial triplet rejects the log; no state-feedback fallback is permitted.
    Targets use an explicit zero-order hold between absolute command updates.
    """
    if type(max_joint_triplet_span_ns) is not int or max_joint_triplet_span_ns <= 0:
        raise ValueError("max_joint_triplet_span_ns must be positive")
    rows = _read_command_rows(path)
    joint_parts: dict[int, dict] = {}
    joint_first_ns: int | None = None
    joint_state: tuple[int, ...] | None = None
    joint_refs: tuple[dict, dict, dict] | None = None
    gripper_state: int | None = None
    gripper_ref: dict | None = None
    timeline: list[CommandTarget] = []

    for row in rows:
        timestamp = row["monotonic_ns"]
        can_id = row["can_id"]
        payload = row["payload"]
        ref = {
            "log_line_index": row["line_index"],
            "monotonic_ns": timestamp,
            "can_id": f"0x{can_id:03X}",
            "raw_payload_hex": row["raw_payload_hex"],
        }
        if can_id in JOINT_COMMAND_IDS:
            if joint_first_ns is not None and timestamp - joint_first_ns > max_joint_triplet_span_ns:
                raise ValueError("joint command triplet exceeded its timestamp span")
            if can_id in joint_parts:
                raise ValueError("duplicate joint command ID before complete 0x155-0x157 triplet")
            joint_first_ns = timestamp if joint_first_ns is None else joint_first_ns
            first_raw, second_raw = struct.unpack(">ii", payload)
            joint_parts[can_id] = {
                "values": (first_raw, second_raw),
                "ref": ref,
            }
            if len(joint_parts) == 3:
                ordered = [joint_parts[identifier] for identifier in JOINT_COMMAND_IDS]
                joint_state = tuple(value for part in ordered for value in part["values"])
                joint_refs = tuple(part["ref"] for part in ordered)  # type: ignore[assignment]
                joint_parts.clear()
                joint_first_ns = None
        elif can_id == GRIPPER_COMMAND_ID:
            target_raw, effort_raw, enabled, zero_request = struct.unpack(">iHBB", payload)
            if enabled != 0x01 or zero_request != 0x00:
                raise ValueError("gripper frame is not an enabled position target")
            if target_raw < 0:
                raise ValueError("gripper target must be a nonnegative raw stroke")
            # Keep effort in the source ref/hash; action semantics train only
            # the position target and do not reinterpret effort as a force.
            ref["effort_raw"] = effort_raw
            gripper_state = target_raw
            gripper_ref = ref

        if joint_state is not None and gripper_state is not None:
            assert joint_refs is not None and gripper_ref is not None
            raw_target = (*joint_state, gripper_state)
            if any(type(value) is not int for value in raw_target):
                raise ValueError("decoded command target contains a noninteger")
            target = tuple(math.radians(value * JOINT_DEGREES_PER_RAW)
                           for value in joint_state) + (gripper_state * GRIPPER_METERS_PER_RAW,)
            if any(not math.isfinite(value) for value in target):
                raise ValueError("decoded command target is nonfinite")
            target_timestamp = max(
                max(reference["monotonic_ns"] for reference in joint_refs),
                gripper_ref["monotonic_ns"],
            )
            timeline.append(CommandTarget(
                timestamp_ns=timestamp,
                action_timestamp_ns=target_timestamp,
                target_raw=raw_target,
                target=target,
                joint_command_refs=joint_refs,
                gripper_command_ref=gripper_ref,
            ))
    if joint_parts:
        raise ValueError("command log ends with incomplete 0x155-0x157 joint triplet")
    if not timeline:
        raise ValueError("command log lacks a complete 7D joint-and-gripper target")
    return timeline


def target_at(timeline: list[CommandTarget], timestamp_ns: int) -> CommandTarget:
    """Return the last complete absolute command at or before timestamp_ns."""
    if type(timestamp_ns) is not int or timestamp_ns <= 0:
        raise ValueError("target timestamp must be a positive integer")
    stamps = [item.timestamp_ns for item in timeline]
    index = bisect_right(stamps, timestamp_ns) - 1
    if index < 0:
        raise ValueError("no complete command target exists at requested timestamp")
    selected = timeline[index]
    if (selected.action_timestamp_ns > timestamp_ns
            or any(reference["monotonic_ns"] > timestamp_ns
                   for reference in (*selected.joint_command_refs, selected.gripper_command_ref))):
        raise ValueError("command source timestamp is later than requested action timestamp")
    return selected
