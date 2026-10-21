"""PiPER PI05 contracts and fail-closed local pretrained loading.

This module deliberately does not use PI05Policy.from_pretrained: LeRobot
0.6.1 catches weight-loading errors and may return an uninitialized policy.
No network, hardware, or model allocation occurs during manifest checks.
"""

from __future__ import annotations

import hashlib
import json
from contextlib import contextmanager
from pathlib import Path
from typing import Any

HORIZON = 16
ACTION_DIM = 7
INTERNAL_DIM = 32
CAMERAS = ("observation.images.base", "observation.images.wrist")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def asset_manifest(root: str | Path, *, hash_weights: bool = False) -> dict[str, Any]:
    """Require a local monolithic safetensors model; never silently download.

    Sharded weights need an explicit future loader, rather than a misleading
    success that falls back to random parameters. Processor files may be built
    from the project's dataset statistics; they are reported separately.
    """
    root = Path(root).resolve(strict=True)
    config_path = root / "config.json"
    weights = root / "model.safetensors"
    if not config_path.is_file() or not weights.is_file():
        raise ValueError("PI05 assets require config.json and model.safetensors")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("type") != "pi05":
        raise ValueError("Asset config must identify type=pi05")
    # Parse the safetensors header without importing Torch or allocating a model.
    with weights.open("rb") as stream:
        header_size = int.from_bytes(stream.read(8), "little")
        if not 2 <= header_size <= min(100_000_000, weights.stat().st_size - 8):
            raise ValueError("Invalid safetensors header length")
        header = json.loads(stream.read(header_size))
    tensors = {key: value for key, value in header.items() if key != "__metadata__"}
    if not tensors:
        raise ValueError("Checkpoint contains no tensors")
    payload_size = weights.stat().st_size - 8 - header_size
    for key, tensor in tensors.items():
        offsets = tensor.get("data_offsets", [])
        if len(offsets) != 2 or not 0 <= offsets[0] <= offsets[1] <= payload_size:
            raise ValueError(f"Invalid tensor offsets: {key}")
    return {
        "root": str(root), "config_sha256": _sha256(config_path),
        "weights_bytes": weights.stat().st_size,
        "weights_sha256": _sha256(weights) if hash_weights else None,
        "tensor_count": len(tensors),
        "processor_files_present": all((root / name).is_file() for name in
            ("policy_preprocessor.json", "policy_postprocessor.json")),
        "source_chunk_size": config.get("chunk_size"),
        "source_max_state_dim": config.get("max_state_dim"),
        "source_max_action_dim": config.get("max_action_dim"),
        "loaded": False,
    }


def config_overrides(device: str = "cpu") -> dict[str, Any]:
    """Project settings, separate from checkpoint architecture and statistics."""
    return {
        "chunk_size": HORIZON, "n_action_steps": HORIZON,
        "max_state_dim": INTERNAL_DIM, "max_action_dim": INTERNAL_DIM,
        "empty_cameras": 0, "use_relative_actions": False,
        "device": device, "dtype": "bfloat16" if device.startswith("cuda") else "float32",
        "gradient_checkpointing": True, "compile_model": False,
        "freeze_vision_encoder": True, "train_expert_only": True,
    }


def make_pi05_config(device: str = "cpu", image_size: int = 224):
    """Build the genuine installed LeRobot config with the PiPER contract."""
    from lerobot.configs import FeatureType, PolicyFeature
    from lerobot.policies.pi05.configuration_pi05 import PI05Config

    return PI05Config(
        **config_overrides(device),
        input_features={
            **{name: PolicyFeature(type=FeatureType.VISUAL, shape=(3, image_size, image_size))
               for name in CAMERAS},
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(ACTION_DIM,)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(ACTION_DIM,))},
    )


def load_pi05_strict(root: str | Path, config=None):
    """Load all model tensors strictly, raising for every missing/incompatible key.

    Use upstream's explicit OpenPI name mapping, but do not inherit its exception
    swallowing. A conversion requiring missing trainable tensors is not accepted
    as a real-base load; such conversions need their own documented procedure.
    Caller owns CUDA placement and processor construction from dataset stats.
    """
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy
    from safetensors.torch import load_file

    evidence = asset_manifest(root)
    if config is None:
        config = PI05Config.from_pretrained(str(Path(root).resolve()), local_files_only=True)
    policy = PI05Policy(config)
    source = load_file(str(Path(root) / "model.safetensors"), device="cpu")
    fixed = policy._fix_pytorch_state_dict_keys(source, config)
    remapped = {key if key.startswith("model.") else "model." + key: tensor
                for key, tensor in fixed.items()}
    if len(remapped) != len(fixed):
        raise ValueError("Checkpoint key mapping collision")
    result = policy.load_state_dict(remapped, strict=True)
    if result.missing_keys or result.unexpected_keys:
        raise RuntimeError("Strict checkpoint load left unmatched tensors")
    # Compare source values in destination dtype, including any upstream remapping.
    state = policy.state_dict()
    import torch
    for key, value in remapped.items():
        if not torch.equal(state[key].cpu(), value.to(dtype=state[key].dtype)):
            raise RuntimeError(f"Post-load tensor mismatch: {key}")
    evidence.update(loaded=True, strict=True, matched_tensors=len(remapped),
                    missing_keys=[], unexpected_keys=[])
    return policy, evidence


@contextmanager
def strict_pi05_loading():
    """Run official trainer with strict local PI05 loading, restoring on exit.

    This is a process-local adapter, never a mutation of installed upstream
    source. It must wrap the trainer in the same process, not a subprocess.
    Each successful load appends actual tensor evidence to the yielded list.
    """
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    original = PI05Policy.__dict__["from_pretrained"]
    evidence = []

    def strict(cls, pretrained_name_or_path, *, config=None, **kwargs):
        if kwargs.get("strict", True) is not True:
            raise ValueError("Project PI05 loader does not allow strict=False")
        policy, record = load_pi05_strict(pretrained_name_or_path, config=config)
        evidence.append(record)
        return policy

    PI05Policy.from_pretrained = classmethod(strict)
    try:
        yield evidence
    finally:
        PI05Policy.from_pretrained = original
