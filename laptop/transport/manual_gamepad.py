"""Typed feedback and command batch for locally supervised Xbox teleoperation."""
from __future__ import annotations

from dataclasses import dataclass
import struct
import time

from laptop.piper.control_profile import MANUAL_GAMEPAD_GRIPPER_LIMITS as CONTINUOUS_GRIPPER_LIMITS
from laptop.transport.position_commands import (
    MAX_COMMISSIONING_FEEDBACK_AGE_NS,
    MAX_COMMISSIONING_JOINT_SKEW_NS,
    TASK_JOINT_BOUNDS_RAW,
    CanTxFrame,
)


MANUAL_GAMEPAD_FEEDBACK_IDS = (
    0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8,
    0x261, 0x262, 0x263, 0x264, 0x265, 0x266,
)

# User-approved feedback-only allowance: 5000 SDK units = 5 degrees.
# Transmitted targets remain inside the original strict joint bounds.
MANUAL_JOINT_FEEDBACK_TOLERANCE_RAW = 5000


def manual_joint_feedback_in_bounds(values):
    return all(low - MANUAL_JOINT_FEEDBACK_TOLERANCE_RAW <= value
               <= high + MANUAL_JOINT_FEEDBACK_TOLERANCE_RAW
               for value, (low, high) in zip(values, TASK_JOINT_BOUNDS_RAW))


def bounded_hold_joints(values):
    """Use the nearest legal hold target without changing recorded feedback."""
    if len(values) != 6 or not manual_joint_feedback_in_bounds(values):
        raise ValueError('hold feedback is outside manual joint tolerance')
    return tuple(max(low, min(high, value))
                 for value, (low, high) in zip(values, TASK_JOINT_BOUNDS_RAW))


def recoverable_manual_rejection(exc, *, can_tx_calls, bridge_ready):
    """Only known transient pre-write failures may wait for a NEW observation."""
    return (can_tx_calls == 0 and bridge_ready is True
        and isinstance(exc, (ValueError, PermissionError)) and str(exc) in {
            'gamepad target exceeds the 5-degree per-update limit',
            'gamepad gripper target exceeds its per-update limit',
            'manual target expired or deadline too long',
            'manual gamepad target expired or validity exceeds 250 ms',
            'manual gamepad feedback is stale or from the future',
            'manual gamepad joint feedback is not time-aligned',
        })


@dataclass(frozen=True)
class ManualGamepadSnapshot:
    """One fresh, gateway-owned state; unlike commissioning it need not be still."""

    sequence: int
    joint_raw_0p001deg: tuple[int, ...]
    gripper_raw: int
    ctrl_mode_raw: int
    arm_status_raw: int
    move_mode_raw: int
    teach_status_raw: int
    error_code: int
    gripper_status_raw: int
    motor_status_codes: tuple[int, ...]
    feedback_timestamps_ns: tuple[tuple[int, int], ...]
    competing_command_seen: bool

    def __post_init__(self):
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("manual gamepad snapshot sequence must be non-negative")
        if (type(self.joint_raw_0p001deg) is not tuple
                or len(self.joint_raw_0p001deg) != 6
                or any(type(value) is not int for value in self.joint_raw_0p001deg)):
            raise ValueError("manual gamepad snapshot requires six raw joints")
        if type(self.gripper_raw) is not int:
            raise ValueError("manual gamepad snapshot requires raw gripper feedback")
        for name in ("ctrl_mode_raw", "arm_status_raw", "move_mode_raw",
                     "teach_status_raw", "error_code", "gripper_status_raw"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if (type(self.motor_status_codes) is not tuple
                or len(self.motor_status_codes) != 6
                or any(type(value) is not int or not 0 <= value <= 255
                       for value in self.motor_status_codes)):
            raise ValueError("six raw motor status codes are required")
        if (type(self.feedback_timestamps_ns) is not tuple
                or any(type(item) is not tuple or len(item) != 2
                       or type(item[0]) is not int or type(item[1]) is not int
                       or item[1] <= 0 for item in self.feedback_timestamps_ns)
                or tuple(item[0] for item in self.feedback_timestamps_ns)
                != MANUAL_GAMEPAD_FEEDBACK_IDS):
            raise ValueError("complete ordered feedback timestamps are required")
        if type(self.competing_command_seen) is not bool:
            raise ValueError("competing command flag must be boolean")


@dataclass(frozen=True)
class ManualGamepadBatch:
    """One target update authorized by the local, time-limited operator scope."""

    request_id: str
    source_sequence: int
    current_joints: tuple[int, ...]
    target_joints: tuple[int, ...]
    current_gripper: int
    target_gripper: int
    expires_ns: int
    joint_step_limit_raw: int = 5_000
    gripper_step_limit_raw: int = CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw
    gripper_effort_raw: int = CONTINUOUS_GRIPPER_LIMITS.effort_raw
    joints_only: bool = False
    controller_speed_percent: int = 5
    operator_approved: bool = False
    physical_estop_ready: bool = False
    low_speed_confirmed: bool = False
    motion_envelope_cleared: bool = False
    zero_return: bool = False

    def __post_init__(self):
        if type(self.controller_speed_percent) is not int or self.controller_speed_percent not in (5,10,15,30,50):
            raise ValueError("manual controller speed must be one of 5/10/15/30/50 percent")
        if (not isinstance(self.request_id, str) or not self.request_id.isascii()
                or not 1 <= len(self.request_id) <= 128):
            raise ValueError("bounded ASCII request ID required")
        if type(self.source_sequence) is not int or self.source_sequence < 0:
            raise ValueError("source sequence must be non-negative")
        for name in ("current_joints", "target_joints"):
            values = getattr(self, name)
            if (type(values) is not tuple or len(values) != 6
                    or any(type(value) is not int for value in values)):
                raise ValueError(f"{name} must contain six integer SDK joint targets")
        if type(self.current_gripper) is not int or type(self.target_gripper) is not int:
            raise ValueError("gripper targets must be integer SDK units")
        if type(self.expires_ns) is not int or self.expires_ns <= 0:
            raise ValueError("monotonic request deadline required")
        if type(self.zero_return) is not bool:
            raise ValueError('zero_return must be boolean')
        if self.zero_return and (self.target_joints != (0,) * 6
                or self.joints_only is not True
                or self.current_gripper != self.target_gripper
                or self.controller_speed_percent not in (5, 10)):
            raise ValueError('zero return requires joint-only zero target, held gripper and at most 10% speed')
        step_cap = 180_000 if self.zero_return else 5_000
        if (type(self.joint_step_limit_raw) is not int
                or not 1 <= self.joint_step_limit_raw <= step_cap):
            raise ValueError("gamepad joint step must be within 1..5 degrees")
        if (type(self.gripper_step_limit_raw) is not int
                or not 1 <= self.gripper_step_limit_raw
                <= CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw):
            raise ValueError("gamepad gripper step exceeds the tested per-update limit")
        if (type(self.gripper_effort_raw) is not int
                or self.gripper_effort_raw != CONTINUOUS_GRIPPER_LIMITS.effort_raw):
            raise ValueError("gamepad gripper effort differs from the shared P2 profile")
        if any(getattr(self, name) is not True for name in (
                "operator_approved", "physical_estop_ready", "low_speed_confirmed",
                "motion_envelope_cleared")):
            raise PermissionError("current local operator and site confirmations are required")
        if not manual_joint_feedback_in_bounds(self.current_joints):
            raise ValueError("current feedback is outside the shared task joint envelope")
        if any(not low <= value <= high
               for value, (low, high) in zip(self.target_joints, TASK_JOINT_BOUNDS_RAW)):
            raise ValueError("gamepad target is outside the shared task joint envelope")
        if any(abs(target - current) > self.joint_step_limit_raw
               for target, current in zip(self.target_joints, self.current_joints)):
            raise ValueError("gamepad target exceeds the 5-degree per-update limit")
        if type(self.joints_only) is not bool:
            raise ValueError("joints_only must be boolean")
        if self.joints_only and self.current_gripper != self.target_gripper:
            raise ValueError("joint-only batch cannot change gripper")
        if not self.joints_only and any(not CONTINUOUS_GRIPPER_LIMITS.minimum_raw <= value
               <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw
               for value in (self.target_gripper,)):
            raise ValueError("gripper target is outside the shared tested envelope")
        if abs(self.target_gripper - self.current_gripper) > self.gripper_step_limit_raw:
            raise ValueError("gamepad gripper target exceeds its per-update limit")

    @property
    def frames(self):
        joints = tuple(
            CanTxFrame(0x155 + index,
                       struct.pack(">ii", *self.target_joints[2*index:2*index+2]))
            for index in range(3)
        )
        gripper = CanTxFrame(
            0x159,
            struct.pack(">iHBB", self.target_gripper, self.gripper_effort_raw, 1, 0),
        )
        return joints if self.joints_only else joints + (gripper,)

    def validate_time(self, now_ns):
        if type(now_ns) is not int or not 0 < self.expires_ns - now_ns <= 250_000_000:
            raise PermissionError("manual gamepad target expired or validity exceeds 250 ms")


def validate_manual_gamepad_snapshot(snapshot, *, now_ns, joints_only=False):
    """Check fresh CAN_CTRL/MOVE_J feedback without requiring the arm to be still."""
    if type(snapshot) is not ManualGamepadSnapshot:
        raise TypeError("typed manual gamepad feedback snapshot required")
    if type(now_ns) is not int or now_ns <= 0:
        raise ValueError("current monotonic time required")
    stamps = dict(snapshot.feedback_timestamps_ns)
    if any(stamp > now_ns or now_ns-stamp > MAX_COMMISSIONING_FEEDBACK_AGE_NS
           for stamp in stamps.values()):
        raise PermissionError("manual gamepad feedback is stale or from the future")
    joint_stamps = tuple(stamps[key] for key in (0x2A5, 0x2A6, 0x2A7))
    if max(joint_stamps)-min(joint_stamps) > MAX_COMMISSIONING_JOINT_SKEW_NS:
        raise PermissionError("manual gamepad joint feedback is not time-aligned")
    if (snapshot.ctrl_mode_raw != 1 or snapshot.arm_status_raw != 0
            or snapshot.move_mode_raw != 1 or snapshot.error_code != 0
            or snapshot.teach_status_raw not in (0, 2)):
        raise PermissionError("manual gamepad requires healthy CAN_CTRL/MOVE_J feedback")
    if any(not code & 0x40 for code in snapshot.motor_status_codes):
        raise PermissionError("all six PiPER motors must be enabled")
    if (not 0 <= snapshot.gripper_status_raw <= 0xFF
            or snapshot.gripper_status_raw & 0xBF):
        raise PermissionError("gripper reports a fault")
    if snapshot.competing_command_seen:
        raise PermissionError("another controller sent a control frame during this session")
    if not manual_joint_feedback_in_bounds(snapshot.joint_raw_0p001deg):
        raise PermissionError("manual gamepad feedback is outside the shared task envelope")
    if type(joints_only) is not bool:
        raise ValueError("joints_only must be boolean")
    # Raw feedback is preserved, including end-stop offsets. Only transmitted
    # gripper targets are bounded; fault/status/freshness checks above remain.
    return snapshot
