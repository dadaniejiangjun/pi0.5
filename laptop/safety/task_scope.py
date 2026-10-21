"""Task-scoped authorization for bounded, supervised PiPER position updates.

One task permit derives short-lived, one-use approvals for exact targets.
SafetyBridge and the typed transport still check every batch. This module
does not convert policy proxies into commands or prove site attestations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from numbers import Real
import threading
import uuid

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    RobotStateSnapshot,
)
from laptop.transport.position_commands import (
    MAX_POSITION_APPROVAL_WINDOW_NS,
    P12_SITE_GATE_NAMES,
    SupervisedPositionAuthorization,
)
from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_LIMITS,
    GRIPPER_NOMINAL_METERS_PER_RAW,
    validate_continuous_gripper_target,
)


MAX_TASK_DURATION_NS = 120_000_000_000
MAX_TASK_JOINT_STEP_RAD = math.radians(20.0)
TASK_SPEED_PERCENT = 5
TASK_TARGET_PERIOD_NS = 100_000_000
TASK_FEEDBACK_MAX_AGE_NS = 120_000_000
TASK_JOINT_FEEDBACK_MAX_SKEW_NS = 20_000_000
TASK_REQUIRED_FEEDBACK_IDS = (
    0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8,
    0x261, 0x262, 0x263, 0x264, 0x265, 0x266,
)


@dataclass(frozen=True)
class TaskControlSnapshot:
    """Complete, fresh feedback required before task-scoped position TX."""

    robot_state: RobotStateSnapshot
    ctrl_mode_raw: int
    arm_status_raw: int
    move_mode_raw: int
    teach_status_raw: int
    error_code: int
    gripper_status_raw: int
    motor_enabled: tuple[bool, ...]
    motor_status_codes: tuple[int, ...]
    feedback_timestamps_ns: tuple[tuple[int, int], ...]

    def __post_init__(self) -> None:
        if type(self.robot_state) is not RobotStateSnapshot:
            raise TypeError("robot_state must be a RobotStateSnapshot")
        for name in (
            "ctrl_mode_raw", "arm_status_raw", "move_mode_raw",
            "teach_status_raw", "error_code", "gripper_status_raw",
        ):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value <= 0xFFFF:
                raise ValueError(f"{name} must be a bounded non-negative integer")
        enabled = tuple(self.motor_enabled)
        if len(enabled) != 6 or any(type(value) is not bool for value in enabled):
            raise ValueError("motor_enabled must contain six booleans")
        codes = tuple(self.motor_status_codes)
        if len(codes) != 6 or any(type(value) is not int or not 0 <= value <= 255 for value in codes):
            raise ValueError("motor_status_codes must contain six raw status bytes")
        try:
            stamps = tuple(self.feedback_timestamps_ns)
        except TypeError as exc:
            raise ValueError("feedback_timestamps_ns must list every feedback channel") from exc
        if (len(stamps) != len(TASK_REQUIRED_FEEDBACK_IDS)
                or any(type(item) is not tuple or len(item) != 2 for item in stamps)
                or tuple(item[0] for item in stamps) != TASK_REQUIRED_FEEDBACK_IDS
                or any(type(item[1]) is not int or item[1] <= 0 for item in stamps)):
            raise ValueError("feedback timestamps must list every required channel in order")
        times = tuple(stamp for _, stamp in stamps)
        if (self.robot_state.captured_monotonic_ns != min(times)
                or self.robot_state.received_monotonic_ns != max(times)):
            raise ValueError("robot-state timestamps must span the listed feedback window")
        object.__setattr__(self, "motor_enabled", enabled)
        object.__setattr__(self, "motor_status_codes", codes)
        object.__setattr__(self, "feedback_timestamps_ns", stamps)

    def validate_for_position_control(self, *, now_monotonic_ns: int) -> None:
        if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
            raise ValueError("now_monotonic_ns must be a positive integer")
        state = self.robot_state
        if not state.connected or not state.status_ok:
            raise PermissionError("task robot feedback is disconnected or unhealthy")
        if not state.units_verified:
            raise PermissionError("task robot-state units are not verified")
        if self.ctrl_mode_raw != 1:
            raise PermissionError("task position control requires CAN_CTRL mode")
        if self.arm_status_raw != 0:
            raise PermissionError("task position control requires normal arm status")
        if self.move_mode_raw != 1:
            raise PermissionError("task position control requires MOVE_J mode")
        # STOP_RECORDING (2) is also idle; match the mode-init bridge check.
        if self.teach_status_raw not in (0, 2):
            raise PermissionError("task position control requires teaching mode to be disabled")
        if self.error_code != 0:
            raise PermissionError("task position control rejects nonzero robot error codes")
        if self.gripper_status_raw & 0xBF:
            raise PermissionError("task position control rejects gripper fault/status flags")
        if not all(self.motor_enabled):
            raise PermissionError("task position control requires all six motors enabled")
        if any(code != 0x40 for code in self.motor_status_codes):
            raise PermissionError("task motor status bytes must be clean and enabled")
        if (now_monotonic_ns < state.received_monotonic_ns
                or now_monotonic_ns - state.received_monotonic_ns > TASK_FEEDBACK_MAX_AGE_NS):
            raise PermissionError("task robot feedback snapshot is stale or from the future")
        timestamps = dict(self.feedback_timestamps_ns)
        for can_id in TASK_REQUIRED_FEEDBACK_IDS:
            stamp = timestamps[can_id]
            if stamp > now_monotonic_ns or now_monotonic_ns - stamp > TASK_FEEDBACK_MAX_AGE_NS:
                raise PermissionError(f"task feedback channel 0x{can_id:03X} is stale or from the future")
        joint_ids = (0x2A5, 0x2A6, 0x2A7)
        joint_times = tuple(timestamps[can_id] for can_id in joint_ids)
        if max(joint_times) - min(joint_times) > TASK_JOINT_FEEDBACK_MAX_SKEW_NS:
            raise PermissionError("joint feedback channels exceed the 20 ms skew limit")


@dataclass(frozen=True)
class TaskScopedPositionAuthorization:
    """One operator authorization for one task/model, bounded to at most 120 s.

    Every target still requires fresh state and yields an exact, one-use
    position permit. Joint travel across the task is deliberately not capped.
    """

    authorization_id: str
    task_scope_id: str
    task: str
    model_profile: str
    checkpoint_id: str
    operator_attestation_id: str
    issued_monotonic_ns: int
    valid_until_monotonic_ns: int
    max_duration_s: int
    record_full_evidence: bool
    site_gate_attestations: tuple[tuple[str, bool], ...]
    speed_percent: int = TASK_SPEED_PERCENT
    max_joint_step_deg: float = 20.0
    gripper_min_raw: int = CONTINUOUS_GRIPPER_LIMITS.minimum_raw
    gripper_max_raw: int = CONTINUOUS_GRIPPER_LIMITS.maximum_raw
    gripper_max_step_raw: int = CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw
    gripper_effort_raw: int = CONTINUOUS_GRIPPER_LIMITS.effort_raw
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False, compare=False)
    _used_proposal_ids: set[str] = field(default_factory=set, init=False, repr=False, compare=False)
    _last_state_sequence: int = field(default=-1, init=False, repr=False, compare=False)
    _last_joint_feedback_timestamps: tuple[int, ...] = field(
        default=(), init=False, repr=False, compare=False,
    )
    _last_authorization_ns: int = field(default=0, init=False, repr=False, compare=False)
    _revoked: bool = field(default=False, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        names = (
            "authorization_id", "task_scope_id", "task", "model_profile",
            "checkpoint_id", "operator_attestation_id",
        )
        for name in names:
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip() or len(value) > 256:
                raise ValueError(f"{name} must be non-empty bounded text")
        if (type(self.issued_monotonic_ns) is not int or self.issued_monotonic_ns <= 0
                or type(self.valid_until_monotonic_ns) is not int
                or self.valid_until_monotonic_ns <= self.issued_monotonic_ns):
            raise ValueError("task authorization needs an ordered monotonic validity window")
        if type(self.max_duration_s) is not int or not 1 <= self.max_duration_s <= 120:
            raise ValueError("task authorization duration must be between 1 and 120 seconds")
        if self.valid_until_monotonic_ns - self.issued_monotonic_ns > self.max_duration_s * 1_000_000_000:
            raise ValueError("task authorization outlives its authorized duration")
        if type(self.record_full_evidence) is not bool:
            raise ValueError("record_full_evidence must be boolean")
        if type(self.speed_percent) is not int or self.speed_percent != TASK_SPEED_PERCENT:
            raise ValueError("ROS task position control requires the selected 5% speed")
        if (isinstance(self.max_joint_step_deg, bool)
                or not isinstance(self.max_joint_step_deg, Real)
                or not math.isfinite(float(self.max_joint_step_deg))
                or float(self.max_joint_step_deg) != 20.0):
            raise ValueError("ROS task position control requires a 20 degree single-step ceiling")
        effective_gripper = (
            self.gripper_min_raw, self.gripper_max_raw,
            self.gripper_max_step_raw, self.gripper_effort_raw,
        )
        configured_gripper = (
            CONTINUOUS_GRIPPER_LIMITS.minimum_raw,
            CONTINUOUS_GRIPPER_LIMITS.maximum_raw,
            CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
            CONTINUOUS_GRIPPER_LIMITS.effort_raw,
        )
        if any(type(value) is not int for value in effective_gripper):
            raise ValueError("continuous task gripper limits must be integer raw values")
        if effective_gripper != configured_gripper:
            raise ValueError("task authorization gripper limits must match the shared continuous profile")
        try:
            gate_items = tuple(self.site_gate_attestations)
        except TypeError as exc:
            raise ValueError("site_gate_attestations must include every P12 gate") from exc
        if (len(gate_items) != len(P12_SITE_GATE_NAMES)
                or any(type(item) is not tuple or len(item) != 2 for item in gate_items)):
            raise ValueError("site_gate_attestations must include every P12 gate exactly once")
        gates = dict(gate_items)
        if (len(gates) != len(P12_SITE_GATE_NAMES)
                or set(gates) != set(P12_SITE_GATE_NAMES)
                or any(type(value) is not bool for value in gates.values())):
            raise ValueError("site_gate_attestations contain missing, duplicate, or invalid gates")
        if any(not gates[name] for name in P12_SITE_GATE_NAMES):
            raise PermissionError("all P12 site gates must be attested before a task permit is issued")
        object.__setattr__(self, "max_joint_step_deg", float(self.max_joint_step_deg))
        object.__setattr__(self, "site_gate_attestations", tuple(
            (name, gates[name]) for name in P12_SITE_GATE_NAMES
        ))

    @property
    def revoked(self) -> bool:
        with self._lock:
            return self._revoked

    def validate_active(self, *, now_monotonic_ns: int) -> None:
        if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
            raise ValueError("now_monotonic_ns must be a positive integer")
        with self._lock:
            if self._revoked:
                raise PermissionError("task authorization has been revoked")
        if not self.issued_monotonic_ns <= now_monotonic_ns < self.valid_until_monotonic_ns:
            raise PermissionError("task authorization is not currently active")

    def authorize_target(
        self, proposal: CommandProposal, snapshot: TaskControlSnapshot, *,
        now_monotonic_ns: int,
    ) -> SupervisedPositionAuthorization:
        """Derive one one-use per-target permit after task and state checks."""
        if type(proposal) is not CommandProposal:
            raise TypeError("proposal must be a CommandProposal")
        if type(snapshot) is not TaskControlSnapshot:
            raise TypeError("snapshot must be a TaskControlSnapshot")
        self.validate_active(now_monotonic_ns=now_monotonic_ns)
        snapshot.validate_for_position_control(now_monotonic_ns=now_monotonic_ns)
        if proposal.task_scope_id != self.task_scope_id:
            raise PermissionError("proposal task scope does not match the active task")
        if proposal.model_profile != self.model_profile:
            raise PermissionError("proposal model profile does not match the authorized task")
        if proposal.checkpoint_id != self.checkpoint_id:
            raise PermissionError("proposal checkpoint does not match the authorized task")
        if proposal.source_state_sequence != snapshot.robot_state.sequence:
            raise PermissionError("proposal was not created from the current task feedback snapshot")
        if proposal.action_semantics != ABSOLUTE_JOINT_POSITION_COMMAND:
            raise PermissionError("task position control requires absolute position command semantics")
        if tuple(proposal.units) != JOINT_GRIPPER_UNITS:
            raise PermissionError("task position control requires radian/meter command units")
        if not proposal.hardware_units_verified or not proposal.mapping_verified:
            raise PermissionError("task position control requires verified units and command mapping")
        if proposal.created_monotonic_ns < snapshot.robot_state.received_monotonic_ns:
            raise PermissionError("proposal predates its source feedback snapshot")
        if not proposal.created_monotonic_ns <= now_monotonic_ns < proposal.valid_until_monotonic_ns:
            raise PermissionError("proposal is from the future or expired")
        if proposal.valid_until_monotonic_ns - proposal.created_monotonic_ns > MAX_POSITION_APPROVAL_WINDOW_NS:
            raise PermissionError("proposal validity exceeds the 250 ms command window")
        timestamps = dict(snapshot.feedback_timestamps_ns)
        joint_times = tuple(timestamps[can_id] for can_id in (0x2A5, 0x2A6, 0x2A7))
        for index, (current, target) in enumerate(zip(
            snapshot.robot_state.joint_position_rad, proposal.target[:6],
        ), start=1):
            if abs(target - current) > MAX_TASK_JOINT_STEP_RAD:
                raise PermissionError(f"joint_{index} target exceeds the 20 degree task step ceiling")
        current_gripper_raw = round(
            snapshot.robot_state.gripper_position_m / GRIPPER_NOMINAL_METERS_PER_RAW
        )
        target_gripper_raw = round(
            proposal.target[6] / GRIPPER_NOMINAL_METERS_PER_RAW
        )
        try:
            validate_continuous_gripper_target(
                current_gripper_raw, target_gripper_raw, self.gripper_effort_raw,
            )
        except ValueError as exc:
            raise PermissionError(str(exc)) from exc
        with self._lock:
            if self._revoked:
                raise PermissionError("task authorization has been revoked")
            if now_monotonic_ns >= self.valid_until_monotonic_ns:
                raise PermissionError("task authorization expired before target approval")
            if proposal.proposal_id in self._used_proposal_ids:
                raise PermissionError("task proposal id has already been consumed")
            if snapshot.robot_state.sequence <= self._last_state_sequence:
                raise PermissionError("task feedback sequence is not strictly increasing")
            if (self._last_joint_feedback_timestamps
                    and any(new <= old for new, old in zip(
                        joint_times, self._last_joint_feedback_timestamps,
                    ))):
                raise PermissionError("task target requires new bridge-received joint feedback")
            if (self._last_authorization_ns
                    and now_monotonic_ns - self._last_authorization_ns < TASK_TARGET_PERIOD_NS):
                raise PermissionError("task target updates must not exceed 10 Hz")
            self._used_proposal_ids.add(proposal.proposal_id)
            object.__setattr__(self, "_last_state_sequence", snapshot.robot_state.sequence)
            object.__setattr__(self, "_last_joint_feedback_timestamps", joint_times)
            object.__setattr__(self, "_last_authorization_ns", now_monotonic_ns)
        expiry_ns = min(
            proposal.valid_until_monotonic_ns,
            now_monotonic_ns + MAX_POSITION_APPROVAL_WINDOW_NS,
            self.valid_until_monotonic_ns,
        )
        return SupervisedPositionAuthorization(
            approval_id=f"{self.authorization_id}:{uuid.uuid4().hex}",
            proposal_id=proposal.proposal_id,
            source_request_id=proposal.source_request_id,
            source_state_sequence=proposal.source_state_sequence,
            target=proposal.target,
            approved_at_monotonic_ns=now_monotonic_ns,
            valid_until_monotonic_ns=expiry_ns,
            operator_confirmed=True,
            site_gate_attestations=self.site_gate_attestations,
        )

    def revoke(self) -> None:
        """Prevent new permits; this is not a physical hardware stop."""
        with self._lock:
            object.__setattr__(self, "_revoked", True)


def localize_ros_task_authorization(
    goal: object,
    authorization: object,
    *,
    ros_now_monotonic_ns: int,
    windows_now_monotonic_ns: int,
    site_gate_attestations: tuple[tuple[str, bool], ...],
) -> TaskScopedPositionAuthorization:
    """Validate a ROS task permit and rebase its deadline to Windows time.

    WSL and Windows monotonic clocks have different epochs. Never compare or
    reuse the ROS timestamps on Windows; verify the ROS permit against the
    ROS clock at receipt, then preserve only its remaining lifetime locally.
    The caller must pass site attestations from the local commissioning record.
    This is a data-contract adapter, not authentication or proof of approval.
    """
    if type(ros_now_monotonic_ns) is not int or ros_now_monotonic_ns <= 0:
        raise ValueError("ros_now_monotonic_ns must be a positive integer")
    if type(windows_now_monotonic_ns) is not int or windows_now_monotonic_ns <= 0:
        raise ValueError("windows_now_monotonic_ns must be a positive integer")

    goal_fields = (
        "task", "model_profile", "checkpoint_id", "record_full_evidence",
        "max_duration_s", "authorization_id",
    )
    authorization_fields = (
        "authorization_id", "task_scope_id", "task", "model_profile",
        "checkpoint_id", "record_full_evidence", "max_duration_s",
        "issued_monotonic_ns", "valid_until_monotonic_ns",
        "operator_attestation_id",
    )
    try:
        goal_values = {name: getattr(goal, name) for name in goal_fields}
        auth_values = {name: getattr(authorization, name) for name in authorization_fields}
    except AttributeError as exc:
        raise ValueError("ROS task goal or authorization is missing required fields") from exc

    for name in ("task", "model_profile", "checkpoint_id", "authorization_id"):
        left, right = goal_values[name], auth_values[name]
        if (not isinstance(left, str) or not left.strip()
                or not isinstance(right, str) or not right.strip() or left != right):
            raise PermissionError(f"ROS task goal and authorization disagree on {name}")
    if (not isinstance(auth_values["task_scope_id"], str)
            or not auth_values["task_scope_id"].strip()):
        raise ValueError("ROS task authorization has no task_scope_id")
    if (type(goal_values["record_full_evidence"]) is not bool
            or type(auth_values["record_full_evidence"]) is not bool
            or goal_values["record_full_evidence"] != auth_values["record_full_evidence"]):
        raise PermissionError("ROS task goal and authorization disagree on evidence recording")
    duration_s = goal_values["max_duration_s"]
    if (type(duration_s) is not int
            or not 1 <= duration_s <= MAX_TASK_DURATION_NS // 1_000_000_000
            or type(auth_values["max_duration_s"]) is not int
            or auth_values["max_duration_s"] != duration_s):
        raise PermissionError("ROS task goal and authorization duration is invalid or mismatched")
    issued_ns = auth_values["issued_monotonic_ns"]
    expires_ns = auth_values["valid_until_monotonic_ns"]
    if (type(issued_ns) is not int or type(expires_ns) is not int
            or issued_ns <= 0 or expires_ns <= issued_ns
            or expires_ns - issued_ns > duration_s * 1_000_000_000
            or not issued_ns <= ros_now_monotonic_ns < expires_ns):
        raise PermissionError("ROS task authorization is expired, future-dated, or overlong")
    operator_id = auth_values["operator_attestation_id"]
    if not isinstance(operator_id, str) or not operator_id.strip():
        raise PermissionError("ROS task authorization has no operator attestation ID")
    remaining_ns = min(expires_ns - ros_now_monotonic_ns, duration_s * 1_000_000_000)
    local_expires_ns = windows_now_monotonic_ns + remaining_ns
    return TaskScopedPositionAuthorization(
        authorization_id=auth_values["authorization_id"],
        task_scope_id=auth_values["task_scope_id"],
        task=auth_values["task"],
        model_profile=auth_values["model_profile"],
        checkpoint_id=auth_values["checkpoint_id"],
        operator_attestation_id=operator_id,
        issued_monotonic_ns=windows_now_monotonic_ns,
        valid_until_monotonic_ns=local_expires_ns,
        max_duration_s=duration_s,
        record_full_evidence=auth_values["record_full_evidence"],
        site_gate_attestations=site_gate_attestations,
    )
