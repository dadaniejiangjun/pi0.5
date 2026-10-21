"""Raw-unit contract for one explicitly supervised PiPER position update."""
from dataclasses import dataclass
import struct

from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_LIMITS,
    SINGLE_STEP_GRIPPER_LIMITS,
)
from laptop.transport.position_commands import CanTxFrame, TASK_JOINT_BOUNDS_RAW


@dataclass(frozen=True)
class BoundedTrialBatch:
    source_sequence: int
    current_joints: tuple
    target_joints: tuple
    current_gripper: int
    target_gripper: int
    prediction_request_id: str
    expires_ns: int
    joint_step_limit_raw: int = 1000
    gripper_step_limit_raw: int = CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw
    gripper_profile: str = "continuous_task"
    gripper_effort_raw: int = CONTINUOUS_GRIPPER_LIMITS.effort_raw
    controller_speed_percent: int = 5

    def __post_init__(self):
        if type(self.source_sequence) is not int or self.source_sequence < 0:
            raise ValueError("sequence required")
        if not self.prediction_request_id:
            raise ValueError("prediction request required")
        if type(self.expires_ns) is not int or self.expires_ns <= 0:
            raise ValueError("expiry required")
        if type(self.current_joints) is not tuple or type(self.target_joints) is not tuple:
            raise TypeError("immutable joint tuples required")
        if len(self.current_joints) != 6 or len(self.target_joints) != 6:
            raise ValueError("six joints required")
        if any(type(v) is not int for v in self.current_joints + self.target_joints
               + (self.current_gripper, self.target_gripper)):
            raise ValueError("integer SDK units required")
        if type(self.joint_step_limit_raw) is not int or not 0 < self.joint_step_limit_raw <= 20_000:
            raise ValueError("joint step limit must be in (0,20000]")
        if any(abs(a - b) > self.joint_step_limit_raw
               for a, b in zip(self.current_joints, self.target_joints)):
            raise ValueError("trial exceeds configured joint step")
        if any(not low <= value <= high
               for value, (low, high) in zip(self.current_joints, TASK_JOINT_BOUNDS_RAW)):
            raise ValueError("current joint feedback is outside configured task range")
        if any(not low <= value <= high
               for value, (low, high) in zip(self.target_joints, TASK_JOINT_BOUNDS_RAW)):
            raise ValueError("joint target is outside configured task range")
        if self.gripper_profile == "continuous_task":
            gripper_limits = CONTINUOUS_GRIPPER_LIMITS
        elif self.gripper_profile == "single_step_commissioning":
            gripper_limits = SINGLE_STEP_GRIPPER_LIMITS
        else:
            raise ValueError("gripper profile must be continuous_task or single_step_commissioning")
        if (not gripper_limits.minimum_raw <= self.current_gripper <= gripper_limits.maximum_raw
                or not gripper_limits.minimum_raw <= self.target_gripper <= gripper_limits.maximum_raw):
            raise ValueError("gripper exceeds the selected tested command envelope")
        if (type(self.gripper_step_limit_raw) is not int
                or not 0 < self.gripper_step_limit_raw <= gripper_limits.maximum_step_raw):
            raise ValueError("gripper step limit exceeds the selected profile")
        if abs(self.target_gripper - self.current_gripper) > self.gripper_step_limit_raw:
            raise ValueError("gripper exceeds configured step limit")
        if (type(self.gripper_effort_raw) is not int
                or self.gripper_effort_raw != gripper_limits.effort_raw):
            raise ValueError("gripper effort differs from the selected profile")
        if self.controller_speed_percent not in (5, 10, 15, 30, 50):
            raise ValueError("trial controller speed must be one of 5/10/15/30/50 percent")

    @property
    def frames(self):
        joints = tuple(
            CanTxFrame(0x155 + index, struct.pack(">ii", *self.target_joints[2 * index:2 * index + 2]))
            for index in range(3)
        )
        gripper = CanTxFrame(
            0x159,
            struct.pack(">iHBB", self.target_gripper, self.gripper_effort_raw, 1, 0),
        )
        return joints + (gripper,)

    def validate_time(self, now):
        if not 0 < self.expires_ns - now <= 250_000_000:
            raise PermissionError("trial expired or approval window too long")
