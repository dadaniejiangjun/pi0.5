"""Strict control-RPC envelope checks and reject-only gateway decisions.

This module only defines a bounded request envelope and a read-only decision.
It never creates authorization, opens CAN, or transmits a robot command.
"""
from __future__ import annotations

from typing import Mapping


CONTROL_OPERATIONS = frozenset({
    "set_joint_target", "set_gripper", "set_control_mode",
    "set_motor_enabled", "home_arm",
})
MAX_REQUEST_ID_LENGTH = 128
MAX_TASK_SCOPE_ID_LENGTH = 128
MAX_VALIDITY_MS = 5_000
MAX_SOURCE_STATE_AGE_NS = 120_000_000


def validate_control_request(payload: Mapping, *, session_id: str) -> tuple[str, str]:
    """Return ``(request_id, operation)`` for a valid session-bound request."""
    if not isinstance(payload, Mapping):
        raise ValueError("control request must be a map")
    expected = {
        "session_id", "request_id", "operation", "task_scope_id",
        "source_state_sequence", "validity_ms", "arguments",
    }
    if set(payload) != expected:
        raise ValueError("control request fields do not match the protocol")
    if payload.get("session_id") != session_id or not session_id:
        raise ValueError("control request session does not match active handshake")
    request_id = payload.get("request_id")
    if (not isinstance(request_id, str) or not request_id.isascii()
            or not request_id.strip() or len(request_id) > MAX_REQUEST_ID_LENGTH):
        raise ValueError("request_id must be bounded non-empty ASCII text")
    operation = payload.get("operation")
    if not isinstance(operation, str) or operation not in CONTROL_OPERATIONS:
        raise ValueError("unsupported control operation")
    source_sequence = payload.get("source_state_sequence")
    if type(source_sequence) is not int or not 0 <= source_sequence <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("source_state_sequence must be an unsigned 64-bit integer")
    if operation in {"set_joint_target", "set_gripper"}:
        if source_sequence == 0:
            raise ValueError("position target requires a nonzero source_state_sequence")
    elif source_sequence != 0:
        raise ValueError("management requests must use source_state_sequence=0")
    task_scope_id = payload.get("task_scope_id")
    if (not isinstance(task_scope_id, str)
            or len(task_scope_id) > MAX_TASK_SCOPE_ID_LENGTH):
        raise ValueError("task_scope_id must be bounded text")
    validity_ms = payload.get("validity_ms")
    if type(validity_ms) is not int or not 0 <= validity_ms <= MAX_VALIDITY_MS:
        raise ValueError("validity_ms must be an integer within 0..5000")
    arguments = payload.get("arguments")
    if not isinstance(arguments, dict) or len(arguments) > 16:
        raise ValueError("arguments must be a bounded map")
    if any(not isinstance(key, str) for key in arguments):
        raise ValueError("argument keys must be strings")
    return request_id, operation


def validate_control_source_binding(
    *, source_session_id: str, source_state_sequence: int,
    active_session_id: str, latest_state_sequence: int,
    latest_state_monotonic_ns: int, mapping_uncertainty_ns: int,
    source_state_monotonic_ns: int | None = None,
    source_mapping_uncertainty_ns: int | None = None,
    source_feedback_fresh: bool = True,
    feedback_fresh: bool, now_monotonic_ns: int,
) -> None:
    """Reject requests not bound to a recent same-session gateway observation.

    Source and current timestamps use the ROS-host monotonic clock. This check
    grants no command authority; it only rejects stale or cross-session targets.
    """
    if (not isinstance(source_session_id, str) or not source_session_id
            or source_session_id != active_session_id):
        raise ValueError("target source session does not match the active gateway session")
    if (type(source_state_sequence) is not int or source_state_sequence <= 0
            or type(latest_state_sequence) is not int or latest_state_sequence <= 0):
        raise ValueError("target source state sequence is missing or invalid")
    if source_state_sequence > latest_state_sequence:
        raise ValueError("target source state sequence refers to future feedback")
    if feedback_fresh is not True:
        raise ValueError("latest gateway feedback is not marked fresh")
    values = (latest_state_monotonic_ns, mapping_uncertainty_ns, now_monotonic_ns)
    if (any(type(value) is not int or value < 0 for value in values)
            or latest_state_monotonic_ns == 0):
        raise ValueError("target source timestamp or clock uncertainty is invalid")
    age_ns = now_monotonic_ns - latest_state_monotonic_ns
    if (mapping_uncertainty_ns > MAX_SOURCE_STATE_AGE_NS
            or age_ns < -mapping_uncertainty_ns
            or age_ns + mapping_uncertainty_ns > MAX_SOURCE_STATE_AGE_NS):
        raise ValueError("latest gateway state is stale or outside clock uncertainty")
    if source_feedback_fresh is not True:
        raise ValueError("requested gateway feedback is not marked fresh")
    if source_state_sequence != latest_state_sequence and source_state_monotonic_ns is None:
        raise ValueError("recent source timestamp is required when request sequence is not latest")
    source_stamp = latest_state_monotonic_ns if source_state_monotonic_ns is None else source_state_monotonic_ns
    source_uncertainty = (
        mapping_uncertainty_ns if source_mapping_uncertainty_ns is None
        else source_mapping_uncertainty_ns
    )
    if (type(source_stamp) is not int or type(source_uncertainty) is not int
            or source_stamp <= 0 or source_uncertainty < 0
            or source_uncertainty > MAX_SOURCE_STATE_AGE_NS):
        raise ValueError("requested source timestamp or clock uncertainty is invalid")
    source_age = now_monotonic_ns - source_stamp
    if (source_age < -source_uncertainty
            or source_age + source_uncertainty > MAX_SOURCE_STATE_AGE_NS):
        raise ValueError("requested source state is stale or outside clock uncertainty")


def read_only_control_result(
    payload: Mapping, *, session_id: str, used_request_ids: set[str],
) -> dict:
    """Produce a fail-closed result without dispatching to the CAN owner."""
    request_id = payload.get("request_id", "") if isinstance(payload, Mapping) else ""
    if not isinstance(request_id, str) or len(request_id) > MAX_REQUEST_ID_LENGTH:
        request_id = ""
    result = {
        "session_id": session_id,
        "request_id": request_id,
        "accepted": False,
        "decision": "REJECTED_READ_ONLY_PROFILE_NO_TX",
        "reason": "gateway control RPC is receipt-only; no local command dispatcher is enabled",
        "can_tx_calls": 0,
    }
    try:
        request_id, _operation = validate_control_request(payload, session_id=session_id)
    except (TypeError, ValueError) as exc:
        result["decision"] = "REJECTED_INVALID_CONTROL_REQUEST"
        result["reason"] = str(exc)[:192]
        return result
    result["request_id"] = request_id
    if request_id in used_request_ids:
        result["decision"] = "REJECTED_REPLAYED_CONTROL_REQUEST"
        result["reason"] = "request_id has already been used in this gateway session"
        return result
    if len(used_request_ids) >= 4096:
        # Keep the profile fail-closed instead of forgetting replay history.
        result["decision"] = "REJECTED_REQUEST_WINDOW_FULL"
        result["reason"] = "control request replay window is full; reconnect to start a new session"
        return result
    used_request_ids.add(request_id)
    return result
