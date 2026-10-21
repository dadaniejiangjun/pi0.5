"""Immutable messages crossing the policy-to-robot safety boundary.

The proxy model predicts a *next observed state*.  It is deliberately a
different type from :class:`CommandProposal`; equal array shapes do not make
the two meanings interchangeable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Iterable, Tuple


NEXT_OBSERVED_STATE_PROXY = "next_observed_state_proxy_nominal_units"
ABSOLUTE_JOINT_POSITION_COMMAND = "absolute_joint_position_rad_gripper_meter"
JOINT_GRIPPER_UNITS = (
    "radian",
    "radian",
    "radian",
    "radian",
    "radian",
    "radian",
    "meter",
)
LAPTOP_MONOTONIC_CLOCK = "laptop_perf_counter_ns"


def _vector7(values: Iterable[float], field_name: str) -> Tuple[float, ...]:
    try:
        raw_values = tuple(values)
    except TypeError as exc:
        raise ValueError(f"{field_name} must be an iterable of real numbers") from exc
    result = tuple(_finite_real(value, field_name) for value in raw_values)
    if len(result) != 7:
        raise ValueError(f"{field_name} must contain exactly 7 values")
    return result


def _finite_real(value: object, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field_name} values must be real numbers, not booleans")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field_name} must contain only finite values")
    return result


def _canonical_int(value: object, field_name: str, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError(f"{field_name} must be an integer >= {minimum}")
    return int(value)


@dataclass(frozen=True)
class TimingContract:
    """Versioned timing rules shared by observations, predictions, and commands.

    All timestamps are on one laptop-local monotonic clock. ``target_dt_s`` is
    relative to the observation timestamp; command validity is a short,
    independent lease and cannot be extended by a delayed policy response.
    """

    schema: str = "piper_timing_contract_v1"
    clock_domain: str = LAPTOP_MONOTONIC_CLOCK
    max_state_age_ns: int = 250_000_000
    max_prediction_response_latency_ns: int = 250_000_000
    max_proposal_age_ns: int = 250_000_000
    max_proposal_validity_ns: int = 250_000_000
    max_heartbeat_age_ns: int = 250_000_000
    first_target_must_be_future_at_evaluation: bool = True

    def __post_init__(self) -> None:
        if self.schema != "piper_timing_contract_v1":
            raise ValueError("unsupported timing contract schema")
        if self.clock_domain != LAPTOP_MONOTONIC_CLOCK:
            raise ValueError("timing contract must use the laptop monotonic clock")
        for name in (
            "max_state_age_ns",
            "max_prediction_response_latency_ns",
            "max_proposal_age_ns",
            "max_proposal_validity_ns",
            "max_heartbeat_age_ns",
        ):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if type(self.first_target_must_be_future_at_evaluation) is not bool:
            raise ValueError("first_target_must_be_future_at_evaluation must be boolean")


@dataclass(frozen=True)
class RobotStateSnapshot:
    """A timestamped observation; it grants no authority to command hardware."""

    sequence: int
    captured_monotonic_ns: int
    received_monotonic_ns: int
    joint_position_rad: Tuple[float, ...]
    gripper_position_m: float
    status_ok: bool
    connected: bool
    units_verified: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", _canonical_int(self.sequence, "sequence", minimum=0))
        object.__setattr__(self, "captured_monotonic_ns", _canonical_int(
            self.captured_monotonic_ns, "captured_monotonic_ns", minimum=1))
        object.__setattr__(self, "received_monotonic_ns", _canonical_int(
            self.received_monotonic_ns, "received_monotonic_ns", minimum=1))
        if self.received_monotonic_ns < self.captured_monotonic_ns:
            raise ValueError("received timestamp precedes captured timestamp")
        try:
            raw_joints = tuple(self.joint_position_rad)
        except TypeError as exc:
            raise ValueError("joint_position_rad must contain 6 finite values") from exc
        joints = tuple(_finite_real(value, "joint_position_rad") for value in raw_joints)
        if len(joints) != 6:
            raise ValueError("joint_position_rad must contain 6 finite values")
        for field_name in ("status_ok", "connected", "units_verified"):
            if type(getattr(self, field_name)) is not bool:
                raise ValueError(f"{field_name} must be a boolean")
        object.__setattr__(self, "joint_position_rad", joints)
        object.__setattr__(self, "gripper_position_m", _finite_real(
            self.gripper_position_m, "gripper_position_m"))

    @property
    def state7(self) -> Tuple[float, ...]:
        return self.joint_position_rad + (self.gripper_position_m,)


@dataclass(frozen=True)
class PolicyPrediction:
    """Prediction-only output from the proxy policy server.

    Each ``target_dt_s`` entry is an offset from
    ``observation_monotonic_ns``, not from response receipt or execution time.
    """

    request_id: str
    request_sequence: int
    observation_monotonic_ns: int
    received_monotonic_ns: int
    actions: Tuple[Tuple[float, ...], ...]
    target_dt_s: Tuple[float, ...]
    checkpoint_identity: str
    config_identity: str
    normalization_identity: str
    dataset_identity: str
    action_semantics: str = NEXT_OBSERVED_STATE_PROXY
    units: Tuple[str, ...] = JOINT_GRIPPER_UNITS
    prediction_only: bool = True
    hardware_units_verified: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.request_id, str) or not self.request_id.strip():
            raise ValueError("request_id is required")
        object.__setattr__(self, "request_sequence", _canonical_int(
            self.request_sequence, "request_sequence", minimum=0))
        object.__setattr__(self, "observation_monotonic_ns", _canonical_int(
            self.observation_monotonic_ns, "observation_monotonic_ns", minimum=1))
        object.__setattr__(self, "received_monotonic_ns", _canonical_int(
            self.received_monotonic_ns, "received_monotonic_ns", minimum=1))
        if self.received_monotonic_ns < self.observation_monotonic_ns:
            raise ValueError("prediction received before its observation")
        actions = tuple(_vector7(row, "actions row") for row in self.actions)
        if not actions:
            raise ValueError("actions must contain at least one row")
        try:
            target_dt_s = tuple(_finite_real(value, "target_dt_s")
                                for value in self.target_dt_s)
        except TypeError as exc:
            raise ValueError("target_dt_s must be a sequence") from exc
        if len(target_dt_s) != len(actions):
            raise ValueError("target_dt_s length must equal the action horizon")
        if not all(value > 0 for value in target_dt_s):
            raise ValueError("target_dt_s must contain positive finite values")
        if any(current <= previous for previous, current in zip(target_dt_s, target_dt_s[1:])):
            raise ValueError("target_dt_s must be strictly increasing")
        offsets_ns = []
        for value in target_dt_s:
            offset = value * 1_000_000_000
            if not math.isfinite(offset) or offset < 1:
                raise ValueError("target_dt_s must resolve to a finite positive nanosecond offset")
            offsets_ns.append(round(offset))
        if any(current <= previous for previous, current in zip(offsets_ns, offsets_ns[1:])):
            raise ValueError("target_dt_s values must resolve to strictly increasing nanosecond offsets")
        if self.action_semantics != NEXT_OBSERVED_STATE_PROXY:
            raise ValueError("PolicyPrediction only accepts proxy-state semantics")
        if tuple(self.units) != JOINT_GRIPPER_UNITS:
            raise ValueError("proxy units must be radian x6 plus meter")
        if type(self.prediction_only) is not bool or not self.prediction_only:
            raise ValueError("proxy predictions must remain prediction-only")
        if type(self.hardware_units_verified) is not bool or self.hardware_units_verified:
            raise ValueError("proxy nominal units cannot assert hardware verification")
        for identity_name in (
            "checkpoint_identity",
            "config_identity",
            "normalization_identity",
            "dataset_identity",
        ):
            value = getattr(self, identity_name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{identity_name} is required")
        object.__setattr__(self, "actions", actions)
        object.__setattr__(self, "target_dt_s", target_dt_s)
        object.__setattr__(self, "units", tuple(self.units))

    @property
    def target_timestamps_monotonic_ns(self) -> Tuple[int, ...]:
        """Absolute target timestamps derived only from the laptop observation clock."""
        return tuple(
            self.observation_monotonic_ns + round(offset_s * 1_000_000_000)
            for offset_s in self.target_dt_s
        )

    @property
    def response_latency_ns(self) -> int:
        return self.received_monotonic_ns - self.observation_monotonic_ns


@dataclass(frozen=True)
class CommandProposal:
    """A candidate command, still requiring SafetyBridge approval.

    Creation and expiry timestamps use the same local monotonic clock as the
    source state snapshot. The bridge rejects proposals created before that
    state was received or evaluated after the proposal expires.
    """

    proposal_id: str
    source_request_id: str
    source_state_sequence: int
    created_monotonic_ns: int
    valid_until_monotonic_ns: int
    target: Tuple[float, ...]
    action_semantics: str
    units: Tuple[str, ...]
    hardware_units_verified: bool
    mapping_verified: bool
    task_scope_id: str = ""
    model_profile: str = ""
    checkpoint_id: str = ""

    def __post_init__(self) -> None:
        if (not isinstance(self.proposal_id, str) or not self.proposal_id.strip()
                or not isinstance(self.source_request_id, str)
                or not self.source_request_id.strip()):
            raise ValueError("proposal_id and source_request_id are required")
        object.__setattr__(self, "source_state_sequence", _canonical_int(
            self.source_state_sequence, "source_state_sequence", minimum=0))
        object.__setattr__(self, "created_monotonic_ns", _canonical_int(
            self.created_monotonic_ns, "created_monotonic_ns", minimum=1))
        object.__setattr__(self, "valid_until_monotonic_ns", _canonical_int(
            self.valid_until_monotonic_ns, "valid_until_monotonic_ns", minimum=1))
        if self.valid_until_monotonic_ns <= self.created_monotonic_ns:
            raise ValueError("valid_until must be after created")
        if self.action_semantics != ABSOLUTE_JOINT_POSITION_COMMAND:
            raise ValueError("CommandProposal requires absolute command semantics")
        if tuple(self.units) != JOINT_GRIPPER_UNITS:
            raise ValueError("command units must be radian x6 plus meter")
        if type(self.hardware_units_verified) is not bool or type(self.mapping_verified) is not bool:
            raise ValueError("verification fields must be booleans")
        scope_fields = (self.task_scope_id, self.model_profile, self.checkpoint_id)
        if any(not isinstance(value, str) for value in scope_fields):
            raise ValueError("task scope and model identity fields must be strings")
        if any(scope_fields) and any(not value.strip() or len(value) > 256 for value in scope_fields):
            raise ValueError("task-scoped proposals require bounded task/model/checkpoint identities")
        object.__setattr__(self, "target", _vector7(self.target, "target"))
        object.__setattr__(self, "units", tuple(self.units))


def command_from_proxy(*_: object, **__: object) -> CommandProposal:
    """Make the forbidden implicit conversion explicit and fail closed."""

    raise PermissionError(
        "a next-observed-state proxy is not a PiPER command; "
        "a separately verified controller/mapping is required"
    )
