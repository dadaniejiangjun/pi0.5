"""Dependency-light observation builders for server protocol tests."""

from __future__ import annotations

import numpy as np


PIPER_PROMPT = "Pick up the block and place it in the tray."


def make_official_aloha_dummy_observation(seed: int = 0) -> dict:
    """Create the official ALOHA semantic keys expected by pi05_aloha.

    This is deliberately not a PiPER observation and is used only to exercise
    the official base policy/server transport.
    """

    rng = np.random.default_rng(seed)
    return {
        "state": np.ones((14,), dtype=np.float32),
        "images": {
            "cam_high": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
            "cam_low": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
            "cam_left_wrist": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
            "cam_right_wrist": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
        },
        "prompt": "do something",
    }


def validate_piper_semantic_observation(observation: dict) -> None:
    required = {"base_rgb", "wrist_rgb", "state", "prompt"}
    missing = required.difference(observation)
    if missing:
        raise ValueError(f"missing PiPER fields: {sorted(missing)}")
    for key in ("base_rgb", "wrist_rgb"):
        image = np.asarray(observation[key])
        if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
            raise ValueError(f"{key} must be uint8 HWC RGB, got {image.shape} {image.dtype}")
    state = np.asarray(observation["state"])
    if state.shape != (7,) or state.dtype != np.float32:
        raise ValueError(f"state must be float32[7], got {state.shape} {state.dtype}")
    if not isinstance(observation["prompt"], str):
        raise ValueError("prompt must be str")
