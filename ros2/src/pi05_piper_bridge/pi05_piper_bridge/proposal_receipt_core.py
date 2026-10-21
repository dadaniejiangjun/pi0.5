"""Pure validation helpers for the fake-only ROS proposal receipt path."""
from __future__ import annotations

import math
import numbers
from typing import Any


EXPECTED_ACTION_SEMANTICS = "absolute_joint_position_command"
EXPECTED_UNITS = ("rad", "rad", "rad", "rad", "rad", "rad", "m")
MAX_PROPOSAL_LIFETIME_NS = 250_000_000


class ProposalRejected(ValueError):
    """A candidate did not satisfy the local proposal envelope."""


def _text(value: Any, name: str, *, maximum: int = 128) -> str:
    if (not isinstance(value, str) or not value.strip()
            or len(value) > maximum or not value.isascii()):
        raise ProposalRejected(f"{name} must be non-empty bounded ASCII text")
    return value


def _timestamp(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or int(value) <= 0:
        raise ProposalRejected(f"{name} must be a positive integer")
    return int(value)


def proposal_payload(message: Any, *, now_monotonic_ns: int) -> dict[str, Any]:
    """Validate a typed proposal before placing it on the fake-only TCP path.

    Timestamps must use the WSL process clock and remain valid for no more than
    250 ms. This helper is not a command mapper and is not a substitute for the
    Windows SafetyBridge or its live-state checks.
    """
    now = _timestamp(now_monotonic_ns, "now_monotonic_ns")
    request_id = _text(getattr(message, "request_id", None), "request_id")
    task_scope_id = _text(getattr(message, "task_scope_id", None), "task_scope_id")
    model_profile = _text(getattr(message, "model_profile", None), "model_profile")
    checkpoint_id = _text(getattr(message, "checkpoint_id", None), "checkpoint_id", maximum=256)
    source_request_id = _text(getattr(message, "source_request_id", None), "source_request_id")
    observation_id = _text(
        getattr(message, "source_observation_id", None), "source_observation_id", maximum=256
    )
    state_sequence = getattr(message, "source_state_sequence", None)
    if isinstance(state_sequence, bool) or not isinstance(state_sequence, numbers.Integral) or int(state_sequence) < 1:
        raise ProposalRejected("source_state_sequence must be a positive integer")
    created = _timestamp(getattr(message, "created_monotonic_ns", None), "created_monotonic_ns")
    valid_until = _timestamp(
        getattr(message, "valid_until_monotonic_ns", None), "valid_until_monotonic_ns"
    )
    if created > now:
        raise ProposalRejected("proposal creation time is in the future")
    if valid_until <= now:
        raise ProposalRejected("proposal has expired")
    if valid_until <= created or valid_until - created > MAX_PROPOSAL_LIFETIME_NS:
        raise ProposalRejected("proposal validity must be within 250 ms")
    clock_id = getattr(message, "source_clock_id", None)
    if clock_id != "wsl_perf_counter_ns":
        raise ProposalRejected("proposal timestamps must use the WSL monotonic clock")
    if getattr(message, "action_semantics", None) != EXPECTED_ACTION_SEMANTICS:
        raise ProposalRejected("proposal does not use absolute command semantics")
    raw_units = tuple(getattr(message, "units", ()))
    if raw_units != EXPECTED_UNITS:
        raise ProposalRejected("proposal units must be radian x6 plus meter")
    targets = tuple(getattr(message, "target", ()))
    if len(targets) != 7:
        raise ProposalRejected("proposal target must contain six joints and one gripper value")
    finite_targets: list[float] = []
    for value in targets:
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ProposalRejected("proposal targets must be finite numbers")
        converted = float(value)
        if not math.isfinite(converted):
            raise ProposalRejected("proposal targets must be finite numbers")
        finite_targets.append(converted)
    units_verified = getattr(message, "hardware_units_verified", None)
    mapping_verified = getattr(message, "mapping_verified", None)
    if type(units_verified) is not bool or type(mapping_verified) is not bool:
        raise ProposalRejected("verification fields must be booleans")
    return {
        "request_id": request_id,
        "task_scope_id": task_scope_id,
        "model_profile": model_profile,
        "checkpoint_id": checkpoint_id,
        "source_request_id": source_request_id,
        "observation_id": observation_id,
        "source_state_sequence": int(state_sequence),
        "created_monotonic_ns": created,
        "valid_until_monotonic_ns": valid_until,
        "source_clock_id": clock_id,
        "target": finite_targets,
        "action_semantics": EXPECTED_ACTION_SEMANTICS,
        "units": list(EXPECTED_UNITS),
        "hardware_units_verified": units_verified,
        "mapping_verified": mapping_verified,
    }
