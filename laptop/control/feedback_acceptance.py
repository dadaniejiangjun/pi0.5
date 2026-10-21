"""Named tolerances for feedback motion and final-position acceptance.

All raw joint values use the PiPER 0.001 degree feedback unit. Gripper raw
values are preserved as nominal protocol units; their mechanical scale is not
verified. These limits classify evidence and do not authorize commands.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FeedbackAcceptanceLimits:
    target_joint_tolerance_raw: int = 50
    held_joint_drift_tolerance_raw: int = 50
    target_gripper_tolerance_raw: int = 50
    held_gripper_drift_tolerance_raw: int = 50
    motion_noise_joint_raw: int = 50
    motion_noise_gripper_raw: int = 50
    stable_duration_ns: int = 200_000_000
    stable_max_gap_ns: int = 200_000_000

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if type(value) is not int or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.stable_duration_ns <= 0 or self.stable_max_gap_ns <= 0:
            raise ValueError("stable duration and sample gap must be positive")

    @property
    def target_joint_tolerance_rad(self) -> float:
        return math.radians(self.target_joint_tolerance_raw / 1000.0)


MANUAL_FEEDBACK_LIMITS = FeedbackAcceptanceLimits()

# Separate, named limits used by existing zero/home/model-trial routines.
ZERO_MODE_TARGET_TOLERANCE_RAW = 1000       # nominal 1 degree
ZERO_MODE_STABLE_DURATION_NS = 300_000_000
ZERO_MODE_HELD_DRIFT_RAW = 50               # nominal 0.050 degree
ZERO_RETURN_FINAL_TOLERANCE_RAW = 100        # nominal 0.100 degree
ZERO_RETURN_SEGMENT_TOLERANCE_RAW = 500      # nominal 0.500 degree
MODEL_TRIAL_HELD_JOINT_DRIFT_RAW = 50
MODEL_TRIAL_HELD_GRIPPER_DRIFT_RAW = 1000
MODEL_TRIAL_GRIPPER_TARGET_TOLERANCE_RAW = 1000
