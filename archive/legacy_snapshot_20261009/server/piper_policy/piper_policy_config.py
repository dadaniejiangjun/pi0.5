"""PiPER policy initialization metadata; training is intentionally disabled."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PiperPolicyConfig:
    name: str = "pi05_piper_v1"
    base_checkpoint: str = "pi05_base"
    training_status: str = "NOT_STARTED"
    piper_dataset: str = "NOT_AVAILABLE"
    normalization: str = "NOT_AVAILABLE"
    action_semantics: str = "absolute_joint_position"
    state_semantics: str = "joint_position_plus_gripper"
    formal_fine_tuning: str = "NOT_STARTED"

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "base_checkpoint": self.base_checkpoint,
            "training_status": self.training_status,
            "piper_dataset": self.piper_dataset,
            "normalization": self.normalization,
            "action_semantics": self.action_semantics,
            "state_semantics": self.state_semantics,
            "formal_fine_tuning": self.formal_fine_tuning,
        }
