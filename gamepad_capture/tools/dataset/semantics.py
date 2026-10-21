"""Interpret captured manual-gamepad send events as auditable action rows."""

from __future__ import annotations

from collections import defaultdict
import math
import struct
from typing import Any


JOINT_POSITION_IDS = (0x155, 0x156, 0x157)
GRIPPER_POSITION_ID = 0x159
POSITION_IDS = JOINT_POSITION_IDS + (GRIPPER_POSITION_ID,)
MODE_ID = 0x151
EXPECTED_MODE_PAYLOAD = bytes((1, 1, 5, 0, 0, 0, 0, 0))


def decode_mode_speed_percent(payload: bytes) -> int:
    """Decode the speed byte without assuming the historical 5 percent value."""
    if len(payload) != 8 or payload[0] != 1 or payload[1] != 1:
        raise ValueError("mode CAN payload has unsupported enable bytes")
    speed = int(payload[2])
    if not 1 <= speed <= 100:
        raise ValueError("mode CAN payload has an invalid speed percentage")
    return speed


def decode_position_payloads(
    accepted: list[dict[str, Any]], *, require_gripper: bool = False,
) -> dict[str, Any]:
    """Decode kernel-accepted joint frames and optional gripper evidence."""
    positions = [event for event in accepted if event.get("can_id") in POSITION_IDS]
    by_id: dict[int, dict[str, Any]] = {}
    for event in positions:
        can_id = int(event["can_id"])
        if can_id in by_id:
            raise ValueError("position CAN ID was accepted more than once")
        by_id[can_id] = event
    if set(JOINT_POSITION_IDS) - set(by_id):
        raise ValueError("accepted joint position frames are incomplete")
    if require_gripper and GRIPPER_POSITION_ID not in by_id:
        raise ValueError("accepted gripper position frame is missing")
    joints: list[int] = []
    for can_id in JOINT_POSITION_IDS:
        payload = bytes.fromhex(by_id[can_id]["data_hex"])
        if len(payload) != 8:
            raise ValueError("joint position CAN payload must be 8 bytes")
        joints.extend(struct.unpack(">ii", payload))
    result: dict[str, Any] = {
        "joint_raw_0p001deg": joints,
        "accepted_position_monotonic_ns": max(int(event["monotonic_ns"]) for event in positions),
        "accepted_position_first_monotonic_ns": min(int(event["monotonic_ns"]) for event in positions),
        "gripper_evidence_present": GRIPPER_POSITION_ID in by_id,
    }
    if GRIPPER_POSITION_ID in by_id:
        gripper_payload = bytes.fromhex(by_id[GRIPPER_POSITION_ID]["data_hex"])
        if len(gripper_payload) != 8:
            raise ValueError("gripper position CAN payload must be 8 bytes")
        gripper_raw, effort_raw, enabled, reserved = struct.unpack(">iHBB", gripper_payload)
        if enabled != 1 or reserved != 0:
            raise ValueError("gripper position CAN payload has unsupported enable/reserved flags")
        result.update({"gripper_raw": gripper_raw, "gripper_effort_raw": effort_raw})
    else:
        result.update({"gripper_raw": None, "gripper_effort_raw": None})
    return result


def group_command_events(events: list[dict[str, Any]], receipts: list[dict[str, Any]],
                         states_by_sequence: dict[int, dict[str, Any]], *,
                         expected_session_id: str,
                         max_action_age_ns: int = 250_000_000) -> dict[str, Any]:
    """Classify request groups, retaining incomplete delivery as a cutoff.

    A complete command audit requires a matching TARGET_INTENT, one mode
    frame, three joint position frames, write/accept pairs, an accepted
    service result, and a receipt. A gripper frame is optional evidence and is
    never required to validate a next-observed-state training label.
    """
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for event in events:
        request_id = event.get("request_id")
        if isinstance(request_id, str) and request_id:
            grouped[request_id].append(event)
    receipts_by_id: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for receipt in receipts:
        receipts_by_id[str(receipt.get("request_id", ""))].append(receipt)

    results: list[dict[str, Any]] = []
    cutoff_ns: int | None = None
    for request_id, records in grouped.items():
        records.sort(key=lambda item: int(item.get("monotonic_ns", 0)))
        intents = [item for item in records if item.get("event") == "TARGET_INTENT"]
        writes = [item for item in records if item.get("event") == "CAN_WRITE_INTENT"]
        accepted = [item for item in records if item.get("event") == "CAN_KERNEL_ACCEPTED"]
        unknown = [item for item in records if item.get("event") == "CAN_DELIVERY_UNKNOWN"]
        services = [item for item in records if item.get("event") == "SERVICE_RESULT"]
        reasons: list[str] = []
        intent = intents[0] if len(intents) == 1 else None
        if len(intents) != 1:
            reasons.append("target_intent_missing_or_duplicated")
        if intent is not None:
            if intent.get("source_session_id") != expected_session_id:
                reasons.append("session_mismatch")
            source_seq = intent.get("source_state_sequence")
            if type(source_seq) is not int or source_seq not in states_by_sequence:
                reasons.append("source_state_sequence_missing")
            if not isinstance(intent.get("operation"), str):
                reasons.append("operation_missing")
        if unknown:
            reasons.append("delivery_unknown")
        if not writes or len(writes) != len(accepted):
            reasons.append("write_and_kernel_accept_event_counts_differ")
        if len(writes) == len(accepted):
            # Every local send must have its corresponding matching kernel event.
            write_pairs = [(x.get("can_id"), x.get("data_hex")) for x in writes]
            accept_pairs = [(x.get("can_id"), x.get("data_hex")) for x in accepted]
            if sorted(write_pairs) != sorted(accept_pairs):
                reasons.append("write_accept_payload_mismatch")
        accepted_ids = [int(x.get("can_id", -1)) for x in accepted]
        speed_percent: int | None = None
        if accepted_ids.count(MODE_ID) != 1:
            reasons.append("mode_frame_missing_or_duplicated")
        else:
            try:
                speed_percent = decode_mode_speed_percent(bytes.fromhex(
                    next(x["data_hex"] for x in accepted if int(x["can_id"]) == MODE_ID)
                ))
            except (KeyError, TypeError, ValueError):
                reasons.append("mode_frame_payload_invalid")
        position_ids = [can_id for can_id in accepted_ids if can_id != MODE_ID]
        if any(can_id not in POSITION_IDS for can_id in position_ids):
            reasons.append("unexpected_position_can_id")
        if any(position_ids.count(can_id) != 1 for can_id in JOINT_POSITION_IDS):
            reasons.append("joint_position_frames_missing_or_duplicated")
        if position_ids.count(GRIPPER_POSITION_ID) > 1:
            reasons.append("gripper_position_frame_duplicated")
        if any(x.get("physical_execution_verified") is not False for x in accepted):
            reasons.append("physical_execution_status_missing_or_inconsistent")
        if (len(services) != 1 or services[0].get("accepted") is not True
                or services[0].get("can_tx_calls") != len(accepted)):
            reasons.append("service_result_missing_or_frame_accept_count_mismatch")
        matching_receipts = receipts_by_id.get(request_id, [])
        if (len(matching_receipts) != 1
                or matching_receipts[0].get("adapter_accepted") is not True
                or matching_receipts[0].get("can_tx_calls") != len(accepted)):
            reasons.append("command_receipt_missing_or_incomplete")

        row: dict[str, Any] = {
            "request_id": request_id,
            "complete": not reasons,
            "reasons": reasons,
            "first_event_monotonic_ns": int(records[0].get("monotonic_ns", 0)),
            "kernel_accepted_frame_count": len(accepted),
            "service_result_count": len(services),
            "receipt_count": len(matching_receipts),
            "mode_speed_percent": speed_percent,
            "joint_position_evidence_present": all(
                accepted_ids.count(can_id) == 1 for can_id in JOINT_POSITION_IDS
            ),
            "gripper_position_evidence_present": accepted_ids.count(GRIPPER_POSITION_ID) == 1,
        }
        if intent is not None:
            row.update({
                "source_session_id": intent.get("source_session_id"),
                "source_state_sequence": intent.get("source_state_sequence"),
                "operation": intent.get("operation"),
                "target_intent_joint_raw": intent.get("joint_raw"),
                "target_intent_gripper_raw": intent.get("gripper_raw"),
                "target_intent_monotonic_ns": intent.get("monotonic_ns"),
            })
        if not row["complete"]:
            if writes or accepted or unknown:
                candidate_cutoff = int(records[0].get("monotonic_ns", 0))
                cutoff_ns = candidate_cutoff if cutoff_ns is None else min(cutoff_ns, candidate_cutoff)
        else:
            target = decode_position_payloads(accepted)
            target["accepted_frame_payloads"] = [
                {"can_id": int(event["can_id"]), "data_hex": event["data_hex"],
                 "monotonic_ns": int(event["monotonic_ns"])}
                for event in sorted(accepted, key=lambda item: int(item.get("monotonic_ns", 0)))
            ]
            seq = int(intent["source_state_sequence"])
            state = states_by_sequence[seq]
            target["action_age_ns"] = int(target["accepted_position_monotonic_ns"]) - int(
                state["source_local_monotonic_ns"]
            )
            target["action_age_clock_id"] = state["source_local_clock_id"]
            target["source_state_monotonic_ns"] = int(state["source_local_monotonic_ns"])
            target["source_state_sequence"] = seq
            target["action_valid_by_age"] = 0 <= target["action_age_ns"] <= max_action_age_ns
            row.update(target)
            if not target["action_valid_by_age"]:
                row["reasons"].append("action_age_outside_configured_window")
        row["delivery_incomplete"] = not row["complete"] and bool(writes or accepted or unknown)
        results.append(row)

    results.sort(key=lambda item: item["first_event_monotonic_ns"])
    # Any command after a delivery-incomplete request is inadmissible. No prior
    # target is held across the event; timeline rows after it stay masked.
    for row in results:
        row["after_delivery_incomplete_cutoff"] = (
            cutoff_ns is not None and row["first_event_monotonic_ns"] >= cutoff_ns
        )
        row["action_valid"] = bool(
            row["complete"] and row.get("action_valid_by_age") is True
            and not row["after_delivery_incomplete_cutoff"]
        )
        row["command_evidence_valid"] = bool(row["complete"])
        if row["after_delivery_incomplete_cutoff"] and row["complete"]:
            row["reasons"].append("after_delivery_incomplete_cutoff")
    return {
        "requests": results,
        "delivery_incomplete_cutoff_monotonic_ns": cutoff_ns,
        "complete_request_count": sum(row["complete"] for row in results),
        "action_valid_request_count": sum(row["action_valid"] for row in results),
        "incomplete_request_count": sum(not row["complete"] for row in results),
    }


def raw_target_to_nominal_action(joints_raw: list[int], gripper_raw: int) -> list[float]:
    """Convert nominal SDK units; project hardware calibration remains open."""
    if len(joints_raw) != 6:
        raise ValueError("six joint target values are required")
    return [int(value) * math.pi / 180_000.0 for value in joints_raw] + [int(gripper_raw) * 1e-6]
