"""Load an operator-maintained PiPER commissioning record.

This profile records site declarations; it cannot prove that they are true,
authorize robot motion, or change the read-only default runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any

from laptop.robot_client.replay_preflight import NOMINAL_JOINT_BOUNDS_DEG
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.safety.bridge import SafetyLimits
from laptop.transport.position_commands import P12_SITE_GATE_NAMES


SITE_PROFILE_SCHEMA = "piper_supervised_site_profile_v1"
READY_STATUS = "OPERATOR_DECLARED_READY_FOR_SINGLE_STEP"
# The current project config caps the SDK gripper command at 100 mm. This is a
# software ceiling, not evidence of the physical gripper travel.
MAX_CONFIGURED_GRIPPER_COMMAND_M = 0.100
FIRST_STEP_MAX_JOINT_DELTA_RAD = math.radians(1.0)
FIRST_STEP_MAX_GRIPPER_DELTA_M = 0.001
_TOP_LEVEL_FIELDS = {
    "schema", "status", "site_id", "robot_serial", "firmware_version",
    "commissioned_by", "commissioned_at_utc", "gripper_effort_raw", "limits",
    "p12_gate_evidence", "evidence_refs",
}
_LIMIT_FIELDS = {
    "joint_min_rad", "joint_max_rad", "max_joint_delta_rad",
    "gripper_min_m", "gripper_max_m", "max_gripper_delta_m",
    "max_state_age_ns", "max_prediction_response_latency_ns",
    "max_proposal_age_ns", "max_proposal_validity_ns", "max_heartbeat_age_ns",
}


@dataclass(frozen=True)
class SupervisedSiteProfile:
    """A complete site record, still requiring live runtime checks and consent."""

    site_id: str
    robot_serial: str
    firmware_version: str
    commissioned_by: str
    commissioned_at_utc: str
    gripper_effort_raw: int
    limits: SafetyLimits
    p12_evidence: P12GateEvidence
    evidence_refs: tuple[tuple[str, str], ...]


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_non_finite_number(value: str) -> None:
    raise ValueError(f"non-finite JSON number: {value}")


def _nonempty_text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value.strip()


def _check_sdk_nominal_joint_subset(limits: SafetyLimits) -> None:
    for index, (low, high, (sdk_low_deg, sdk_high_deg)) in enumerate(
        zip(limits.joint_min_rad, limits.joint_max_rad, NOMINAL_JOINT_BOUNDS_DEG),
        start=1,
    ):
        sdk_low, sdk_high = math.radians(sdk_low_deg), math.radians(sdk_high_deg)
        if low < sdk_low or high > sdk_high:
            raise ValueError(
                f"joint_{index} site range exceeds the official SDK nominal range"
            )


def load_supervised_site_profile(path: Path) -> SupervisedSiteProfile:
    """Load a completed JSON profile and reject any missing site declaration.

    Evidence references are required for every P12 item, but are not opened or
    authenticated here. This prevents an incomplete profile from reaching a
    future supervised entrypoint; it does not make the profile trusted.
    """
    try:
        data = json.loads(
            Path(path).read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_non_finite_number,
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read site profile: {exc}") from exc

    if not isinstance(data, dict) or set(data) != _TOP_LEVEL_FIELDS:
        raise ValueError("site profile has missing or unexpected top-level fields")
    if data["schema"] != SITE_PROFILE_SCHEMA:
        raise ValueError("unsupported supervised site profile schema")
    if data["status"] != READY_STATUS:
        raise PermissionError("site profile is not declared ready for one supervised step")

    for name in ("site_id", "robot_serial", "firmware_version", "commissioned_by"):
        _nonempty_text(data[name], name)
    timestamp = _nonempty_text(data["commissioned_at_utc"], "commissioned_at_utc")
    try:
        parsed = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("commissioned_at_utc must be an ISO-8601 timestamp") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("commissioned_at_utc must include a timezone")

    gripper_effort_raw = data["gripper_effort_raw"]
    if type(gripper_effort_raw) is not int or not 0 <= gripper_effort_raw <= 5000:
        raise ValueError("gripper_effort_raw must be an explicit integer in [0, 5000]")

    raw_limits = data["limits"]
    if not isinstance(raw_limits, dict) or set(raw_limits) != _LIMIT_FIELDS:
        raise ValueError("site limits have missing or unexpected fields")
    try:
        limits = SafetyLimits(**raw_limits)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid site SafetyLimits: {exc}") from exc
    _check_sdk_nominal_joint_subset(limits)
    if limits.gripper_max_m > MAX_CONFIGURED_GRIPPER_COMMAND_M:
        raise ValueError("site gripper limit exceeds the current 100 mm SDK command cap")
    if any(value > FIRST_STEP_MAX_JOINT_DELTA_RAD
           for value in limits.max_joint_delta_rad):
        raise ValueError("first supervised step is limited to 1 degree per joint")
    if limits.max_gripper_delta_m > FIRST_STEP_MAX_GRIPPER_DELTA_M:
        raise ValueError("first supervised step is limited to 1 mm gripper travel")

    raw_gates = data["p12_gate_evidence"]
    if not isinstance(raw_gates, dict) or set(raw_gates) != set(P12_SITE_GATE_NAMES):
        raise ValueError("P12 evidence has missing or unexpected gate names")
    try:
        gates = P12GateEvidence(**raw_gates)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid P12 evidence: {exc}") from exc
    if gates.missing:
        raise PermissionError("site checks remain incomplete: " + ", ".join(gates.missing))

    refs = data["evidence_refs"]
    if not isinstance(refs, dict) or set(refs) != set(P12_SITE_GATE_NAMES):
        raise ValueError("evidence_refs must name every P12 gate exactly once")
    clean_refs = tuple(
        (name, _nonempty_text(refs[name], f"evidence_refs.{name}"))
        for name in P12_SITE_GATE_NAMES
    )
    return SupervisedSiteProfile(
        site_id=data["site_id"].strip(),
        robot_serial=data["robot_serial"].strip(),
        firmware_version=data["firmware_version"].strip(),
        commissioned_by=data["commissioned_by"].strip(),
        commissioned_at_utc=timestamp,
        gripper_effort_raw=gripper_effort_raw,
        limits=limits,
        p12_evidence=gates,
        evidence_refs=clean_refs,
    )
