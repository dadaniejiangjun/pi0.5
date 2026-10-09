"""Typed, single-use PiPER position command batches for supervised control.

This module validates command semantics and the exact V2 joint/gripper frame
encoding. It does not itself open CAN or grant operator/site authorization.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import struct

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
)
from laptop.piper.unit_conversion import (
    PIPER_NOMINAL_JOINT_BOUNDS_DEG,
    m_to_sdk_gripper,
    rad_to_sdk_joint,
)


POSITION_FRAME_IDS = (0x155, 0x156, 0x157, 0x159)
COMMISSIONING_J1_FRAME_IDS = (0x155, 0x156, 0x157)
MAX_POSITION_APPROVAL_WINDOW_NS = 250_000_000
MAX_COMMISSIONING_FEEDBACK_AGE_NS = 120_000_000
MAX_COMMISSIONING_JOINT_SKEW_NS = 20_000_000
MAX_COMMISSIONING_APPROVAL_WINDOW_NS = 250_000_000
COMMISSIONING_J1_STEP_RAW = 100  # fixed +0.100 degree at 0.001 degree/raw
COMMISSIONING_J1_STABLE_WINDOW_NS = 300_000_000
COMMISSIONING_JOINT_STABILITY_TOLERANCE_RAW = 50  # fixed +/-0.050 degree drift
COMMISSIONING_JOINT_BOUNDS_RAW = tuple(
    (round(low * 1000), round(high * 1000))
    for low, high in PIPER_NOMINAL_JOINT_BOUNDS_DEG
)
COMMISSIONING_FEEDBACK_IDS = (
    0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x261, 0x262, 0x263, 0x264, 0x265, 0x266,
)
P12_SITE_GATE_NAMES = (
    "firmware_protocol_verified",
    "joint_state_units_verified",
    "gripper_units_verified",
    "controller_limits_verified",
    "mechanical_limits_verified",
    "speed_acceleration_units_verified",
    "low_speed_setting_verified",
    "operator_and_estop_ready",
    "watchdog_verified",
    "disconnect_safe_hold_verified",
    "camera_state_sync_verified",
    "command_target_source_verified",
    "policy_to_command_mapping_verified",
)


@dataclass(frozen=True)
class CanTxFrame:
    """One standard-ID, eight-byte CAN command frame."""

    arbitration_id: int
    data: bytes

    def __post_init__(self) -> None:
        if type(self.arbitration_id) is not int or not 0 <= self.arbitration_id <= 0x7FF:
            raise ValueError("position command frame must use a standard CAN identifier")
        if not isinstance(self.data, bytes) or len(self.data) != 8:
            raise ValueError("position command frame data must contain exactly 8 bytes")


@dataclass(frozen=True)
class PositionFrameWrite:
    """One CAN frame accepted by the local USB adapter, with a monotonic time."""

    arbitration_id: int
    data: bytes
    adapter_accepted_monotonic_ns: int

    def __post_init__(self) -> None:
        CanTxFrame(self.arbitration_id, self.data)
        if (type(self.adapter_accepted_monotonic_ns) is not int
                or self.adapter_accepted_monotonic_ns <= 0):
            raise ValueError("adapter_accepted_monotonic_ns must be positive")


@dataclass(frozen=True)
class J1CommissioningSnapshot:
    """Raw, timestamped feedback and site attestations for one fixed J1 test."""

    sequence: int
    joint_raw_0p001deg: tuple[int, ...]
    status_payload: bytes
    feedback_timestamps_ns: tuple[tuple[int, int], ...]
    motor_enabled: tuple[bool, ...]
    stable_joint_samples: tuple[tuple[int, tuple[int, ...]], ...]
    competing_command_seen: bool
    physical_estop_ready: bool
    low_speed_confirmed: bool
    motion_envelope_cleared: bool

    def __post_init__(self) -> None:
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("commissioning sequence must be non-negative")
        joints = tuple(self.joint_raw_0p001deg)
        if len(joints) != 6 or any(type(value) is not int for value in joints):
            raise ValueError("J1 commissioning feedback must contain six integer joints")
        if not isinstance(self.status_payload, bytes) or len(self.status_payload) != 8:
            raise ValueError("commissioning status feedback must contain eight bytes")
        timestamps = tuple(self.feedback_timestamps_ns)
        if (len(timestamps) != len(COMMISSIONING_FEEDBACK_IDS)
                or any(type(item) is not tuple or len(item) != 2 for item in timestamps)
                or tuple(item[0] for item in timestamps) != COMMISSIONING_FEEDBACK_IDS
                or any(type(item[1]) is not int or item[1] <= 0 for item in timestamps)):
            raise ValueError("commissioning feedback timestamps must list every required channel in order")
        motors = tuple(self.motor_enabled)
        if len(motors) != 6 or any(type(value) is not bool for value in motors):
            raise ValueError("commissioning feedback must report all six motor-enable states")
        try:
            history = tuple(self.stable_joint_samples)
            normalized_history = tuple(
                (item[0], tuple(item[1]))
                for item in history
                if type(item) is tuple and len(item) == 2
            )
        except TypeError as exc:
            raise ValueError("commissioning joint history must contain timestamped joint tuples") from exc
        if (len(history) < 2 or len(normalized_history) != len(history) or any(
            type(stamp) is not int or stamp <= 0
            or len(joints) != 6
            or any(type(value) is not int for value in joints)
            for stamp, joints in normalized_history
        )):
            raise ValueError("commissioning needs at least two timestamped six-joint samples")
        history = normalized_history
        if any(left[0] > right[0] for left, right in zip(history, history[1:])):
            raise ValueError("commissioning joint-history timestamps must be monotonic")
        for name in (
            "competing_command_seen", "physical_estop_ready",
            "low_speed_confirmed", "motion_envelope_cleared",
        ):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be a boolean")
        object.__setattr__(self, "joint_raw_0p001deg", joints)
        object.__setattr__(self, "feedback_timestamps_ns", timestamps)
        object.__setattr__(self, "motor_enabled", motors)
        object.__setattr__(self, "stable_joint_samples", history)


@dataclass(frozen=True)
class J1CommissioningBatch:
    """A selected fixed joint step; J3 uses -1 degree, other axes use +1.

    The J1 default preserves the previous +0.1/+1-degree interface.
    """

    source_state_sequence: int
    source_received_monotonic_ns: int
    current_joint_raw_0p001deg: tuple[int, ...]
    target_joint_raw_0p001deg: tuple[int, ...]
    frames: tuple[CanTxFrame, ...]
    step_raw: int = COMMISSIONING_J1_STEP_RAW
    configure_control_mode: bool = False
    selected_axis: int = 1
    near_zero_home: bool = False

    def __post_init__(self) -> None:
        if type(self.source_state_sequence) is not int or self.source_state_sequence < 0:
            raise ValueError("source_state_sequence must be non-negative")
        if type(self.source_received_monotonic_ns) is not int or self.source_received_monotonic_ns <= 0:
            raise ValueError("source_received_monotonic_ns must be positive")
        current = tuple(self.current_joint_raw_0p001deg)
        target = tuple(self.target_joint_raw_0p001deg)
        if (len(current) != 6 or len(target) != 6
                or any(type(value) is not int for value in current + target)):
            raise ValueError("J1 commissioning current and target must contain six integers")
        if any(not low <= value <= high
               for value, (low, high) in zip(current, COMMISSIONING_JOINT_BOUNDS_RAW)):
            raise ValueError("J1 commissioning source is outside conservative SDK joint bounds")
        if type(self.selected_axis) is not int or self.selected_axis not in range(1, 7):
            raise ValueError("commissioning axis must be J1 through J6")
        allowed_steps = (100, 1000) if self.selected_axis == 1 else ((-1000,) if self.selected_axis == 3 else (1000,))
        if type(self.near_zero_home) is not bool:
            raise ValueError("near_zero_home must be boolean")
        if self.near_zero_home and (any(abs(v) > 1500 for v in current) or not self.configure_control_mode):
            raise ValueError("home requires source within 1.5 degrees of zero and 5% mode")
        if not self.near_zero_home and (type(self.step_raw) is not int or self.step_raw not in allowed_steps):
            raise ValueError("only explicitly selected +0.100 or +1.000 degree J1 steps are permitted")
        if type(self.configure_control_mode) is not bool:
            raise ValueError("configure_control_mode must be boolean")
        if abs(self.step_raw) == 1000 and not self.configure_control_mode:
            raise ValueError("the 1-degree test must explicitly configure CAN/MOVE_J/5% first")
        expected_target = tuple(value + (self.step_raw if index == self.selected_axis - 1 else 0)
                                for index, value in enumerate(current))
        if self.near_zero_home:
            expected_target = (0,) * 6
        if target != expected_target:
            raise ValueError("commissioning target must equal the selected fixed step on exactly one axis")
        if any(not low <= value <= high
               for value, (low, high) in zip(target, COMMISSIONING_JOINT_BOUNDS_RAW)):
            raise ValueError("J1 commissioning target is outside conservative SDK joint bounds")
        frames = tuple(self.frames)
        ids = ((0x151,) if self.configure_control_mode else ()) + COMMISSIONING_J1_FRAME_IDS
        if (len(frames) != len(ids) or any(type(frame) is not CanTxFrame for frame in frames)
                or tuple(frame.arbitration_id for frame in frames) != ids):
            raise ValueError("J1 commissioning requires its exact ordered mode/joint frames")
        if self.configure_control_mode and frames[0].data != bytes((1, 1, 5, 0, 0, 0, 0, 0)):
            raise ValueError("only CAN/MOVE_J/5%/position-speed mode is permitted")
        joint_frames = frames[1:] if self.configure_control_mode else frames
        encoded = tuple(value for frame in joint_frames for value in struct.unpack(">ii", frame.data))
        if encoded != target:
            raise ValueError("commissioning frame bytes do not encode the fixed J1 target")
        object.__setattr__(self, "current_joint_raw_0p001deg", current)
        object.__setattr__(self, "target_joint_raw_0p001deg", target)
        object.__setattr__(self, "frames", frames)

    @classmethod
    def for_fixed_j1_step(cls, snapshot: J1CommissioningSnapshot, *,
                          step_raw: int = 100, configure_control_mode: bool = False,
                          selected_axis: int = 1) -> "J1CommissioningBatch":
        if type(snapshot) is not J1CommissioningSnapshot:
            raise TypeError("snapshot must be a J1CommissioningSnapshot")
        if type(selected_axis) is not int or selected_axis not in range(1, 7):
            raise ValueError("commissioning axis must be J1 through J6")
        allowed_steps = (100, 1000) if selected_axis == 1 else ((-1000,) if selected_axis == 3 else (1000,))
        if type(step_raw) is not int or step_raw not in allowed_steps:
            raise ValueError("only integer +0.100 or +1.000 degree steps are permitted")
        target = tuple(value + (step_raw if index == selected_axis - 1 else 0)
                       for index, value in enumerate(snapshot.joint_raw_0p001deg))
        frames = tuple(
            CanTxFrame(
                0x155 + pair,
                struct.pack(">ii", *target[2 * pair:2 * pair + 2]),
            )
            for pair in range(3)
        )
        if configure_control_mode:
            frames = (CanTxFrame(0x151, bytes((1, 1, 5, 0, 0, 0, 0, 0))), *frames)
        return cls(
            source_state_sequence=snapshot.sequence,
            source_received_monotonic_ns=max(stamp for _, stamp in snapshot.feedback_timestamps_ns),
            current_joint_raw_0p001deg=snapshot.joint_raw_0p001deg,
            target_joint_raw_0p001deg=target,
            frames=frames,
            step_raw=step_raw,
            configure_control_mode=configure_control_mode,
            selected_axis=selected_axis,
        )


@dataclass(frozen=True)
class J1CommissioningAuthorization:
    """Single-use consent bound to the fixed J1 batch and onsite checks."""

    approval_id: str
    source_state_sequence: int
    target_joint_raw_0p001deg: tuple[int, ...]
    approved_at_monotonic_ns: int
    valid_until_monotonic_ns: int
    operator_confirmed: bool
    physical_estop_ready: bool
    low_speed_confirmed: bool
    motion_envelope_cleared: bool

    def __post_init__(self) -> None:
        if not isinstance(self.approval_id, str) or not self.approval_id.strip():
            raise ValueError("approval_id is required")
        if type(self.source_state_sequence) is not int or self.source_state_sequence < 0:
            raise ValueError("source_state_sequence must be non-negative")
        target = tuple(self.target_joint_raw_0p001deg)
        if len(target) != 6 or any(type(value) is not int for value in target):
            raise ValueError("commissioning approval must bind six integer joint targets")
        if (type(self.approved_at_monotonic_ns) is not int
                or type(self.valid_until_monotonic_ns) is not int
                or self.approved_at_monotonic_ns <= 0
                or self.valid_until_monotonic_ns <= self.approved_at_monotonic_ns
                or self.valid_until_monotonic_ns - self.approved_at_monotonic_ns
                > MAX_COMMISSIONING_APPROVAL_WINDOW_NS):
            raise ValueError("commissioning approval needs a maximum 250 ms validity window")
        for name in (
            "operator_confirmed", "physical_estop_ready", "low_speed_confirmed",
            "motion_envelope_cleared",
        ):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be a boolean")
        object.__setattr__(self, "target_joint_raw_0p001deg", target)

    def validate_for(
        self,
        snapshot: J1CommissioningSnapshot,
        batch: J1CommissioningBatch,
        *,
        now_monotonic_ns: int,
    ) -> None:
        if type(snapshot) is not J1CommissioningSnapshot:
            raise TypeError("snapshot must be a J1CommissioningSnapshot")
        if type(batch) is not J1CommissioningBatch:
            raise TypeError("batch must be a J1CommissioningBatch")
        if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
            raise ValueError("current monotonic time must be positive")
        if (self.source_state_sequence != snapshot.sequence
                or batch.source_state_sequence != snapshot.sequence
                or self.target_joint_raw_0p001deg != batch.target_joint_raw_0p001deg):
            raise ValueError("commissioning approval does not match the current J1 batch")
        if not self.operator_confirmed:
            raise PermissionError("fresh operator confirmation is required for the J1 step")
        if (not self.physical_estop_ready or not self.low_speed_confirmed
                or not self.motion_envelope_cleared):
            raise PermissionError("E-stop, low speed, and motion-envelope checks are required")
        if (not snapshot.physical_estop_ready or not snapshot.low_speed_confirmed
                or not snapshot.motion_envelope_cleared):
            raise PermissionError("snapshot is missing onsite commissioning attestations")
        if (now_monotonic_ns < self.approved_at_monotonic_ns
                or now_monotonic_ns > self.valid_until_monotonic_ns):
            raise PermissionError("J1 commissioning approval is not current")


@dataclass(frozen=True)
class J1CommissioningWriteReceipt:
    """Adapter-accepted J1 calibration frames; this does not prove arm motion."""

    approval_id: str
    source_state_sequence: int
    target_joint_raw_0p001deg: tuple[int, ...]
    writes: tuple[PositionFrameWrite, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.approval_id, str) or not self.approval_id.strip():
            raise ValueError("approval_id is required")
        if type(self.source_state_sequence) is not int or self.source_state_sequence < 0:
            raise ValueError("source_state_sequence must be non-negative")
        target = tuple(self.target_joint_raw_0p001deg)
        if len(target) != 6 or any(type(value) is not int for value in target):
            raise ValueError("receipt target must contain six integer joints")
        writes = tuple(self.writes)
        ids = tuple(write.arbitration_id for write in writes)
        if (any(type(write) is not PositionFrameWrite for write in writes)
                or ids not in (COMMISSIONING_J1_FRAME_IDS, (0x151, *COMMISSIONING_J1_FRAME_IDS))):
            raise ValueError("receipt must contain the ordered J1 commissioning frame writes")
        if len(writes) == 4 and writes[0].data != bytes((1, 1, 5, 0, 0, 0, 0, 0)):
            raise ValueError("receipt mode frame must encode CAN/MOVE_J/5%")
        times = tuple(write.adapter_accepted_monotonic_ns for write in writes)
        if any(left > right for left, right in zip(times, times[1:])):
            raise ValueError("commissioning frame timestamps must be monotonic")
        object.__setattr__(self, "target_joint_raw_0p001deg", target)
        object.__setattr__(self, "writes", writes)


def validate_j1_commissioning_snapshot(
    snapshot: J1CommissioningSnapshot,
    *,
    now_monotonic_ns: int,
) -> None:
    """Recheck raw feedback, mode, motors, and site checks before a single step."""
    if type(snapshot) is not J1CommissioningSnapshot:
        raise TypeError("snapshot must be a J1CommissioningSnapshot")
    if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
        raise ValueError("current monotonic time must be positive")
    stamps = tuple(stamp for _, stamp in snapshot.feedback_timestamps_ns)
    if any(stamp > now_monotonic_ns or now_monotonic_ns - stamp > MAX_COMMISSIONING_FEEDBACK_AGE_NS
           for stamp in stamps):
        raise PermissionError("J1 commissioning feedback is missing, stale, or in the future")
    joint_stamps = tuple(
        stamp for can_id, stamp in snapshot.feedback_timestamps_ns
        if can_id in (0x2A5, 0x2A6, 0x2A7)
    )
    if max(joint_stamps) - min(joint_stamps) > MAX_COMMISSIONING_JOINT_SKEW_NS:
        raise PermissionError("six-axis feedback channels are not time-aligned")
    if snapshot.status_payload[0] != 0x01:
        raise PermissionError("PiPER must already be in CAN_CTRL mode")
    if (snapshot.status_payload[1] != 0 or snapshot.status_payload[2] != 0x01
            or snapshot.status_payload[6:8] != b"\x00\x00"):
        raise PermissionError("PiPER arm status must be healthy and normal")
    if not all(snapshot.motor_enabled):
        raise PermissionError("all six motors must already be enabled")
    if snapshot.competing_command_seen:
        raise PermissionError("another controller sent CAN control frames during preflight")
    if not (snapshot.physical_estop_ready and snapshot.low_speed_confirmed
            and snapshot.motion_envelope_cleared):
        raise PermissionError("onsite E-stop, low-speed, and envelope checks are required")
    if any(not low <= value <= high for value, (low, high) in
           zip(snapshot.joint_raw_0p001deg, COMMISSIONING_JOINT_BOUNDS_RAW)):
        raise PermissionError("current six-joint pose is outside conservative SDK position bounds")
    stable_history = tuple(
        (stamp, joints) for stamp, joints in snapshot.stable_joint_samples
        if stamp <= now_monotonic_ns
        and now_monotonic_ns - stamp <= COMMISSIONING_J1_STABLE_WINDOW_NS + MAX_COMMISSIONING_FEEDBACK_AGE_NS
    )
    drift_exceeded = bool(stable_history) and any(
        max(sample[index] for _, sample in stable_history)
        - min(sample[index] for _, sample in stable_history)
        > COMMISSIONING_JOINT_STABILITY_TOLERANCE_RAW
        for index in range(6)
    )
    if (len(stable_history) < 2
            or stable_history[-1][0] - stable_history[0][0]
            < COMMISSIONING_J1_STABLE_WINDOW_NS
            or any(
                right[0] - left[0] > MAX_COMMISSIONING_FEEDBACK_AGE_NS
                for left, right in zip(stable_history, stable_history[1:])
            )
            or now_monotonic_ns - stable_history[-1][0] > MAX_COMMISSIONING_FEEDBACK_AGE_NS
            or any(any(not low <= value <= high
                       for value, (low, high) in zip(joints, COMMISSIONING_JOINT_BOUNDS_RAW))
                   for _, joints in stable_history)
            or drift_exceeded
            or stable_history[-1][1] != snapshot.joint_raw_0p001deg):
        raise PermissionError(
            "joint feedback must stay within SDK bounds and drift at most 0.050 degree "
            "across a 300 ms window, matching the latest sample"
        )


def validate_j1_commissioning_execution(
    snapshot: J1CommissioningSnapshot,
    batch: J1CommissioningBatch,
    authorization: J1CommissioningAuthorization,
    *,
    now_monotonic_ns: int,
) -> None:
    validate_j1_commissioning_snapshot(snapshot, now_monotonic_ns=now_monotonic_ns)
    if type(batch) is not J1CommissioningBatch:
        raise TypeError("batch must be a J1CommissioningBatch")
    if (batch.source_state_sequence != snapshot.sequence
            or batch.source_received_monotonic_ns
            != max(stamp for _, stamp in snapshot.feedback_timestamps_ns)
            or batch.current_joint_raw_0p001deg != snapshot.joint_raw_0p001deg):
        raise ValueError("J1 commissioning batch does not match its fresh source feedback")
    if type(authorization) is not J1CommissioningAuthorization:
        raise TypeError("authorization must be a J1CommissioningAuthorization")
    authorization.validate_for(snapshot, batch, now_monotonic_ns=now_monotonic_ns)


@dataclass(frozen=True)
class PositionBatchWriteReceipt:
    """Timestamped frame writes bound to one approved absolute position target.

    Adapter acceptance timestamps do not prove robot execution or feedback.
    """

    approval_id: str
    proposal_id: str
    source_request_id: str
    source_state_sequence: int
    target: tuple[float, ...]
    writes: tuple[PositionFrameWrite, ...]

    def __post_init__(self) -> None:
        for name in ("approval_id", "proposal_id", "source_request_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} is required")
        if type(self.source_state_sequence) is not int or self.source_state_sequence < 0:
            raise ValueError("source_state_sequence must be non-negative")
        raw_target = tuple(self.target)
        if (len(raw_target) != 7 or any(
                isinstance(value, bool) or not isinstance(value, Real)
                or not math.isfinite(float(value)) for value in raw_target)):
            raise ValueError("receipt target must contain seven finite values")
        target = tuple(float(value) for value in raw_target)
        writes = tuple(self.writes)
        if (len(writes) != 4 or any(type(write) is not PositionFrameWrite for write in writes)
                or tuple(write.arbitration_id for write in writes) != POSITION_FRAME_IDS):
            raise ValueError("receipt must contain all four position frame writes in order")
        times = tuple(write.adapter_accepted_monotonic_ns for write in writes)
        if any(left > right for left, right in zip(times, times[1:])):
            raise ValueError("frame write timestamps must be monotonic")
        object.__setattr__(self, "target", target)
        object.__setattr__(self, "writes", writes)


@dataclass(frozen=True)
class PositionCommandBatch:
    """Four SDK-encoded frames bound to one absolute-position proposal."""

    proposal: CommandProposal
    frames: tuple[CanTxFrame, ...]

    def __post_init__(self) -> None:
        if type(self.proposal) is not CommandProposal:
            raise TypeError("position batch requires a CommandProposal")
        if (self.proposal.action_semantics != ABSOLUTE_JOINT_POSITION_COMMAND
                or tuple(self.proposal.units) != JOINT_GRIPPER_UNITS):
            raise ValueError("position batch requires absolute radian/meter command semantics")
        if not self.proposal.hardware_units_verified or not self.proposal.mapping_verified:
            raise PermissionError("position proposal lacks verified units or command mapping")
        try:
            frames = tuple(self.frames)
        except TypeError as exc:
            raise ValueError("frames must contain the three joint frames and gripper frame") from exc
        if len(frames) != 4 or any(type(frame) is not CanTxFrame for frame in frames):
            raise ValueError("position batch requires exactly four CanTxFrame values")
        if tuple(frame.arbitration_id for frame in frames) != POSITION_FRAME_IDS:
            raise ValueError("position batch must use CAN IDs 0x155, 0x156, 0x157, 0x159 in order")

        joint_raw = tuple(value for frame in frames[:3]
                          for value in struct.unpack(">ii", frame.data))
        expected_joint_raw = tuple(
            int(rad_to_sdk_joint(value, 0.001))
            for value in self.proposal.target[:6]
        )
        if joint_raw != expected_joint_raw:
            raise ValueError("joint frame bytes do not encode the bound command proposal")

        gripper_raw, effort_raw, enable, reserved = struct.unpack(">iHBB", frames[3].data)
        expected_gripper_raw = int(m_to_sdk_gripper(self.proposal.target[6], 0.001))
        if gripper_raw != expected_gripper_raw or gripper_raw < 0:
            raise ValueError("gripper frame does not encode a non-negative proposal position")
        if not 0 <= effort_raw <= 5000 or enable != 0x01 or reserved != 0x00:
            raise ValueError("gripper frame has invalid effort or control flags")
        object.__setattr__(self, "frames", frames)


@dataclass(frozen=True)
class SupervisedPositionAuthorization:
    """One-use operator approval for one exact proposal and site-gate set."""

    approval_id: str
    proposal_id: str
    source_request_id: str
    source_state_sequence: int
    target: tuple[float, ...]
    approved_at_monotonic_ns: int
    valid_until_monotonic_ns: int
    operator_confirmed: bool
    site_gate_attestations: tuple[tuple[str, bool], ...]

    def __post_init__(self) -> None:
        for name in ("approval_id", "proposal_id", "source_request_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} is required")
        if type(self.source_state_sequence) is not int or self.source_state_sequence < 0:
            raise ValueError("source_state_sequence must be a non-negative integer")
        try:
            raw_target = tuple(self.target)
        except (TypeError, ValueError) as exc:
            raise ValueError("authorization target must contain seven finite values") from exc
        if (len(raw_target) != 7 or any(
                isinstance(value, bool) or not isinstance(value, Real)
                or not math.isfinite(float(value)) for value in raw_target)):
            raise ValueError("authorization target must contain seven finite values")
        target = tuple(float(value) for value in raw_target)
        if (type(self.approved_at_monotonic_ns) is not int
                or type(self.valid_until_monotonic_ns) is not int
                or self.approved_at_monotonic_ns <= 0
                or self.valid_until_monotonic_ns <= self.approved_at_monotonic_ns):
            raise ValueError("authorization needs an ordered positive monotonic time window")
        if (self.valid_until_monotonic_ns - self.approved_at_monotonic_ns
                > MAX_POSITION_APPROVAL_WINDOW_NS):
            raise ValueError("position approval window exceeds 250 ms")
        if type(self.operator_confirmed) is not bool:
            raise ValueError("operator_confirmed must be a boolean")
        try:
            gate_items = tuple(self.site_gate_attestations)
        except TypeError as exc:
            raise ValueError("site_gate_attestations must list every P12 gate") from exc
        if (len(gate_items) != len(P12_SITE_GATE_NAMES)
                or any(type(item) is not tuple or len(item) != 2 for item in gate_items)):
            raise ValueError("site_gate_attestations must list every P12 gate exactly once")
        gates = dict(gate_items)
        if (len(gates) != len(P12_SITE_GATE_NAMES)
                or set(gates) != set(P12_SITE_GATE_NAMES)
                or any(type(value) is not bool for value in gates.values())):
            raise ValueError("site_gate_attestations contain missing, duplicate, or invalid gates")
        object.__setattr__(self, "target", target)
        object.__setattr__(self, "site_gate_attestations", gate_items)

    def validate_for(self, proposal: CommandProposal, *, now_monotonic_ns: int) -> None:
        """Reject stale, mismatched, unapproved, or incomplete one-use permits."""
        if type(proposal) is not CommandProposal:
            raise TypeError("proposal must be a CommandProposal")
        if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
            raise ValueError("now_monotonic_ns must be a positive integer")
        if not self.operator_confirmed:
            raise PermissionError("explicit operator approval is required for this position target")
        if any(not value for _, value in self.site_gate_attestations):
            raise PermissionError("all P12 site gates must be attested before position TX")
        if (self.proposal_id != proposal.proposal_id
                or self.source_request_id != proposal.source_request_id
                or self.source_state_sequence != proposal.source_state_sequence
                or self.target != proposal.target):
            raise ValueError("operator approval does not match the exact proposal")
        if (self.approved_at_monotonic_ns < proposal.created_monotonic_ns
                or now_monotonic_ns < self.approved_at_monotonic_ns
                or now_monotonic_ns > self.valid_until_monotonic_ns
                or now_monotonic_ns > proposal.valid_until_monotonic_ns):
            raise ValueError("operator approval or position proposal is stale or expired")
        if self.valid_until_monotonic_ns > proposal.valid_until_monotonic_ns:
            raise ValueError("operator approval outlives the position proposal")


def validate_position_execution_context(
    proposal: CommandProposal,
    current_state: object,
    limits: object,
    *,
    now_monotonic_ns: int,
) -> None:
    """Recheck feedback freshness, physical envelope, and one-step deltas.

    Imports are local because SafetyBridge also depends on this transport
    package. The supplied snapshot and limits must still come from the live
    caller; this software cannot verify the truth of site measurements.
    """
    from laptop.control.contracts import RobotStateSnapshot
    from laptop.safety.bridge import SafetyLimits

    if type(proposal) is not CommandProposal:
        raise TypeError("proposal must be a CommandProposal")
    if type(current_state) is not RobotStateSnapshot:
        raise TypeError("current_state must be a RobotStateSnapshot")
    if type(limits) is not SafetyLimits:
        raise TypeError("limits must be explicit SafetyLimits")
    if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
        raise ValueError("now_monotonic_ns must be a positive integer")
    if not current_state.connected or not current_state.status_ok:
        raise PermissionError("current robot feedback is disconnected or unhealthy")
    if not current_state.units_verified:
        raise PermissionError("current robot-state units are not verified")
    if current_state.sequence != proposal.source_state_sequence:
        raise ValueError("position proposal was not created from this feedback sequence")
    if (current_state.captured_monotonic_ns > current_state.received_monotonic_ns
            or current_state.received_monotonic_ns > now_monotonic_ns
            or proposal.created_monotonic_ns < current_state.received_monotonic_ns):
        raise ValueError("feedback/proposal timestamps are inconsistent")
    if now_monotonic_ns - current_state.received_monotonic_ns > limits.max_state_age_ns:
        raise ValueError("current robot feedback is stale")
    if (now_monotonic_ns - proposal.created_monotonic_ns > limits.max_proposal_age_ns
            or proposal.valid_until_monotonic_ns - now_monotonic_ns
            > limits.max_proposal_validity_ns):
        raise ValueError("position proposal is stale or exceeds the configured lifetime")
    if now_monotonic_ns > proposal.valid_until_monotonic_ns:
        raise ValueError("position proposal has expired")

    for index, (current, target, low, high, delta_limit) in enumerate(zip(
        current_state.joint_position_rad,
        proposal.target[:6],
        limits.joint_min_rad,
        limits.joint_max_rad,
        limits.max_joint_delta_rad,
    )):
        if not low <= current <= high:
            raise ValueError(f"joint_{index + 1} feedback is outside configured position limits")
        if not low <= target <= high:
            raise ValueError(f"joint_{index + 1} target is outside configured position limits")
        if abs(target - current) > delta_limit:
            raise ValueError(f"joint_{index + 1} target exceeds the single-step limit")

    current_gripper = current_state.gripper_position_m
    target_gripper = proposal.target[6]
    if not limits.gripper_min_m <= current_gripper <= limits.gripper_max_m:
        raise ValueError("gripper feedback is outside configured position limits")
    if not limits.gripper_min_m <= target_gripper <= limits.gripper_max_m:
        raise ValueError("gripper target is outside configured position limits")
    if abs(target_gripper - current_gripper) > limits.max_gripper_delta_m:
        raise ValueError("gripper target exceeds the single-step limit")
