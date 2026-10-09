"""Unregistered PiPER/OpenPI data transforms for offline contract tests only.

This module never starts training, loads weights, or sends robot commands.
Registration, real dataset acceptance, and PiPER norm stats remain separate gates.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
from openpi import transforms
from openpi.models import model as model_api
from openpi.models import pi0_config
from openpi.training import config as training_config
from openpi.training import weight_loaders
from openpi_client.image_tools import convert_to_uint8


STATE_DIM = ACTION_DIM = 7


def _image(value: np.ndarray, name: str) -> np.ndarray:
    image = np.asarray(value)
    if image.ndim != 3:
        raise ValueError(f"{name} must be a 3D RGB image")
    if image.shape[-1] != 3 and image.shape[0] == 3:
        image = np.moveaxis(image, 0, -1)
    if image.shape[-1] != 3:
        raise ValueError(f"{name} must have three RGB channels")
    if np.issubdtype(image.dtype, np.floating):
        image = convert_to_uint8(image)
    if image.dtype != np.uint8:
        raise TypeError(f"{name} must be uint8 or floating point")
    return np.ascontiguousarray(image)


def _vector(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value)
    if array.shape != (STATE_DIM,) or not np.issubdtype(array.dtype, np.number):
        raise ValueError(f"{name} must be a numeric 7D vector")
    result = array.astype(np.float32, copy=False)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains non-finite values")
    return np.ascontiguousarray(result)


@dataclasses.dataclass(frozen=True)
class PiperOpenPIInputs(transforms.DataTransformFn):
    """Map two physical cameras and canonical 7D state/action to OpenPI."""

    def __call__(self, data: dict) -> dict:
        base = _image(data["base_rgb"], "base_rgb")
        wrist = _image(data["wrist_rgb"], "wrist_rgb")
        result = {
            "state": _vector(data["state"], "state"),
            "image": {
                "base_0_rgb": base,
                "left_wrist_0_rgb": wrist,
                "right_wrist_0_rgb": np.zeros_like(base),
            },
            "image_mask": {
                "base_0_rgb": np.bool_(True),
                "left_wrist_0_rgb": np.bool_(True),
                "right_wrist_0_rgb": np.bool_(False),
            },
        }
        if "actions" in data:
            actions = np.asarray(data["actions"])
            if actions.ndim != 2 or actions.shape[-1] != ACTION_DIM or not np.issubdtype(actions.dtype, np.number):
                raise ValueError("actions must have shape (horizon, 7)")
            actions = actions.astype(np.float32, copy=False)
            if not np.isfinite(actions).all():
                raise ValueError("actions contain non-finite values")
            result["actions"] = np.ascontiguousarray(actions)
        if "prompt" in data:
            result["prompt"] = data["prompt"]
        return result


@dataclasses.dataclass(frozen=True)
class PiperOpenPIOutputs(transforms.DataTransformFn):
    """Return only the canonical seven dimensions; not a hardware command."""

    def __call__(self, data: dict) -> dict:
        actions = np.asarray(data["actions"])
        if actions.ndim < 1 or actions.shape[-1] < ACTION_DIM or not np.isfinite(actions[..., :ACTION_DIM]).all():
            raise ValueError("model actions lack a finite 7D canonical slice")
        return {"actions": np.ascontiguousarray(actions[..., :ACTION_DIM], dtype=np.float32)}


@dataclasses.dataclass(frozen=True)
class PiperOpenPIDataConfig(training_config.DataConfigFactory):
    """Concrete transform factory; it does not assert that data or stats exist."""

    def create(self, assets_dirs: Path, model_config: model_api.BaseModelConfig) -> training_config.DataConfig:
        if model_config.model_type != model_api.ModelType.PI05:
            raise ValueError("PiPER offline config expects pi0.5")
        repack = transforms.Group(inputs=[transforms.RepackTransform({
            "base_rgb": "observation.images.image",
            "wrist_rgb": "observation.images.wrist_image",
            "state": "observation.state",
            "actions": "action",
            "prompt": "prompt",
        })])
        robot_transforms = transforms.Group(
            inputs=[PiperOpenPIInputs()], outputs=[PiperOpenPIOutputs()]
        )
        return dataclasses.replace(
            self.create_base_config(assets_dirs, model_config),
            repack_transforms=repack,
            data_transforms=robot_transforms,
            model_transforms=training_config.ModelTransformFactory()(model_config),
            action_sequence_keys=("action",),
            prompt_from_task=True,
        )


def make_unregistered_config(*, repo_id: str, base_params_path: str, assets_root: str) -> training_config.TrainConfig:
    """Build a config for inspection only; official training registry is untouched."""

    if not repo_id or not base_params_path or not assets_root:
        raise ValueError("repo_id, base_params_path and assets_root are required")
    return training_config.TrainConfig(
        name="pi05_piper_v1_unregistered",
        model=pi0_config.Pi0Config(pi05=True, action_dim=32, action_horizon=16),
        data=PiperOpenPIDataConfig(
            repo_id=repo_id,
            assets=training_config.AssetsConfig(assets_dir=assets_root, asset_id="piper"),
        ),
        weight_loader=weight_loaders.CheckpointWeightLoader(base_params_path),
        wandb_enabled=False,
    )
