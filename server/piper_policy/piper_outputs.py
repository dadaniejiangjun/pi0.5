"""PiPER action padding and canonical output contract.

The functions here only transform arrays. They do not call an SDK and do not
authorize or issue an action to a real robot. A future SafetyBridge must sit
between a model output and any hardware adapter.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


PIPER_ACTION_DIM = 7
DEFAULT_MODEL_ACTION_DIM = 32


def _validate_action_array(action: np.ndarray, *, name: str) -> np.ndarray:
    action = np.asarray(action)
    if action.ndim < 1 or action.shape[-1] != PIPER_ACTION_DIM:
        raise ValueError(f"{name} must have last dimension {PIPER_ACTION_DIM}, got {action.shape}")
    if not np.issubdtype(action.dtype, np.number):
        raise TypeError(f"{name} must be numeric, got {action.dtype}")
    action = action.astype(np.float32, copy=False)
    if not np.isfinite(action).all():
        raise ValueError(f"{name} contains NaN or Inf")
    return action


def action_pad_mask(model_action_dim: int = DEFAULT_MODEL_ACTION_DIM) -> np.ndarray:
    if model_action_dim < PIPER_ACTION_DIM:
        raise ValueError(f"model_action_dim must be >= {PIPER_ACTION_DIM}")
    return np.concatenate(
        [np.ones(PIPER_ACTION_DIM, dtype=bool), np.zeros(model_action_dim - PIPER_ACTION_DIM, dtype=bool)]
    )


def pad_action_7_to_model(
    action: np.ndarray,
    *,
    model_action_dim: int = DEFAULT_MODEL_ACTION_DIM,
    padding_value: float = 0.0,
) -> np.ndarray:
    action = _validate_action_array(action, name="action")
    if model_action_dim < PIPER_ACTION_DIM:
        raise ValueError(f"model_action_dim must be >= {PIPER_ACTION_DIM}")
    pad_shape = (*action.shape[:-1], model_action_dim - PIPER_ACTION_DIM)
    padding = np.full(pad_shape, padding_value, dtype=np.float32)
    return np.concatenate([action, padding], axis=-1)


def unpad_model_action_to_7(model_action: np.ndarray, *, model_action_dim: int | None = None) -> np.ndarray:
    model_action = np.asarray(model_action)
    if model_action.ndim < 1:
        raise ValueError("model_action must have at least one dimension")
    if model_action_dim is not None and model_action.shape[-1] != model_action_dim:
        raise ValueError(f"expected model action dim {model_action_dim}, got {model_action.shape[-1]}")
    if model_action.shape[-1] < PIPER_ACTION_DIM:
        raise ValueError(f"model action last dimension must be >= {PIPER_ACTION_DIM}")
    if not np.issubdtype(model_action.dtype, np.number):
        raise TypeError(f"model_action must be numeric, got {model_action.dtype}")
    result = model_action[..., :PIPER_ACTION_DIM].astype(np.float32, copy=False)
    if not np.isfinite(result).all():
        raise ValueError("model_action contains NaN or Inf in the canonical slice")
    return np.ascontiguousarray(result)


@dataclass(frozen=True)
class PiperOutputs:
    """Canonical absolute joint-position action in rad/meters."""

    action: np.ndarray

    def __post_init__(self) -> None:
        action = _validate_action_array(self.action, name="canonical action")
        if action.ndim != 1:
            raise ValueError(f"canonical action must be 1D, got {action.shape}")
        object.__setattr__(self, "action", np.ascontiguousarray(action))

    @classmethod
    def from_model_action(cls, model_action: np.ndarray, *, model_action_dim: int | None = None) -> "PiperOutputs":
        return cls(unpad_model_action_to_7(model_action, model_action_dim=model_action_dim))
