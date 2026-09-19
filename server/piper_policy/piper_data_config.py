"""DataConfig placeholder for a future PiPER-specific OpenPI training run."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class PiperDataConfig:
    """Contract metadata; no dataset or normalization statistics exist yet."""

    name: str = "piper_v1"
    state_dim: int = 7
    action_dim: int = 7
    action_semantics: str = "absolute_joint_position"
    state_semantics: str = "joint_position_plus_gripper"
    image_keys: tuple[str, ...] = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")
    dataset_status: str = "NOT_AVAILABLE"
    normalization_status: str = "NOT_AVAILABLE"
    action_sequence_keys: tuple[str, ...] = field(default_factory=lambda: ("action",))

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "action_semantics": self.action_semantics,
            "state_semantics": self.state_semantics,
            "image_keys": list(self.image_keys),
            "dataset_status": self.dataset_status,
            "normalization_status": self.normalization_status,
            "action_sequence_keys": list(self.action_sequence_keys),
        }
