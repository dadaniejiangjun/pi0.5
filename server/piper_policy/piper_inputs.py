"""Semantic PiPER input contract and official OpenPI-compatible preprocessing.

The custom semantic observation is deliberately small and explicit:
``base_rgb``, ``wrist_rgb``, ``state`` (7 canonical values), and ``prompt``.
The model-side image names follow OpenPI's current ModelTransform convention.
The second physical camera is mapped to ``left_wrist_0_rgb`` and the missing
third view is represented by a zero image plus a false mask. This is a shape
contract only; it is not a claim that the pretrained ALOHA policy is a PiPER
policy.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from openpi_client.image_tools import convert_to_uint8, resize_with_pad


PIPER_STATE_DIM = 7
DEFAULT_IMAGE_SIZE = (224, 224)
CANONICAL_PROMPT = "Pick up the block and place it in the tray."


def _canonical_image(image: np.ndarray, *, name: str) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(f"{name} must have HWC RGB shape, got {image.shape}")
    if np.issubdtype(image.dtype, np.floating):
        image = convert_to_uint8(image)
    if image.dtype != np.uint8:
        raise TypeError(f"{name} must be uint8 or floating point, got {image.dtype}")
    return np.ascontiguousarray(image)


@dataclass(frozen=True)
class PiperInputs:
    """Validated canonical PiPER observation, without any robot-side effects."""

    base_rgb: np.ndarray
    wrist_rgb: np.ndarray
    state: np.ndarray
    prompt: str

    def __post_init__(self) -> None:
        base = _canonical_image(self.base_rgb, name="base_rgb")
        wrist = _canonical_image(self.wrist_rgb, name="wrist_rgb")
        state = np.asarray(self.state)
        if state.shape != (PIPER_STATE_DIM,):
            raise ValueError(f"state must have shape ({PIPER_STATE_DIM},), got {state.shape}")
        if not np.issubdtype(state.dtype, np.number):
            raise TypeError(f"state must be numeric, got {state.dtype}")
        state = state.astype(np.float32, copy=False)
        if not np.isfinite(state).all():
            raise ValueError("state contains NaN or Inf")
        if not isinstance(self.prompt, str) or not self.prompt.strip():
            raise ValueError("prompt must be a non-empty string")
        object.__setattr__(self, "base_rgb", base)
        object.__setattr__(self, "wrist_rgb", wrist)
        object.__setattr__(self, "state", np.ascontiguousarray(state))

    @classmethod
    def from_observation(cls, observation: dict) -> "PiperInputs":
        required = {"base_rgb", "wrist_rgb", "state", "prompt"}
        missing = required.difference(observation)
        if missing:
            raise KeyError(f"missing PiPER observation fields: {sorted(missing)}")
        return cls(
            base_rgb=observation["base_rgb"],
            wrist_rgb=observation["wrist_rgb"],
            state=observation["state"],
            prompt=observation["prompt"],
        )

    def to_openpi_observation(self, *, image_size: tuple[int, int] = DEFAULT_IMAGE_SIZE) -> dict:
        """Return the pre-ModelTransform OpenPI semantic dictionary.

        OpenPI's official image helper performs aspect-ratio-preserving
        resize-with-pad. State remains canonical 7D here; a future PiPER
        DataConfig must define its normalization and model padding explicitly.
        """

        height, width = image_size
        base = resize_with_pad(self.base_rgb, height, width)
        wrist = resize_with_pad(self.wrist_rgb, height, width)
        missing_view = np.zeros_like(wrist)
        return {
            "image": {
                "base_0_rgb": np.ascontiguousarray(base),
                "left_wrist_0_rgb": np.ascontiguousarray(wrist),
                "right_wrist_0_rgb": missing_view,
            },
            "image_mask": {
                "base_0_rgb": np.bool_(True),
                "left_wrist_0_rgb": np.bool_(True),
                "right_wrist_0_rgb": np.bool_(False),
            },
            "state": self.state.copy(),
            "prompt": self.prompt,
        }
