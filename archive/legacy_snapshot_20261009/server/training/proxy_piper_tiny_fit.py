"""Bounded pi0.5 tiny-fit of one proxy episode; prediction only, never robot control."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = ROOT / "datasets" / "incoming"
os.environ["HF_LEROBOT_HOME"] = str(DATA_ROOT)

from openpi.models import pi0_config
from openpi.training import config as training_config
from openpi.training import weight_loaders

from server.training.piper_openpi_config import PiperOpenPIDataConfig
from server.training.proxy_dataset_contract import (
    PROXY_DATASETS,
    inspect_proxy_dataset,
)

DATASET = "local/pi05_proxy_cef166e2_2fps"
CONFIG_NAME = "pi05_piper_next_state_proxy_tiny_fit"
EXP_NAME = "cef166e2_two_step"
DEFAULT_VARIANT = "2fps"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _inspect_2fps_dataset() -> dict:
    root = DATA_ROOT / DATASET
    provenance = json.loads((root / "provenance/proxy_conversion.json").read_text(encoding="utf-8"))
    info = json.loads((root / "meta/info.json").read_text(encoding="utf-8"))
    if (provenance.get("schema") != "piper_proxy_lerobot_v1"
            or provenance.get("repo_id") != DATASET
            or provenance.get("action_semantics") != "next_observed_state_proxy_nominal_units"
            or provenance.get("formal_training_eligible") is not False
            or provenance.get("hardware_control_allowed") is not False
            or provenance.get("split") != "train_only"
            or provenance.get("episodes") != 1 or provenance.get("frames") != 39
            or info.get("total_episodes") != 1 or info.get("total_frames") != 39
            or info.get("fps") != 2):
        raise ValueError("proxy dataset identity or offline-only gate mismatch")
    source_proxy = DATA_ROOT / "pi05_proxy_cef166e2/cef166e2_proxy2fps"
    if (_sha256(source_proxy / "proxy_manifest.json") != provenance["proxy_manifest_sha256"]
            or _sha256(source_proxy / "proxy_targets.jsonl") != provenance["proxy_targets_sha256"]):
        raise ValueError("proxy source hash mismatch")
    return {"dataset": DATASET, "frames": 39, "episodes": 1,
            "action_semantics": provenance["action_semantics"],
            "train_only": True, "hardware_control_allowed": False}


def _spec(variant: str):
    try:
        return PROXY_DATASETS[variant]
    except KeyError as exc:
        raise ValueError(f"unknown proxy dataset variant: {variant}") from exc


def inspect_dataset(
    variant: str = DEFAULT_VARIANT,
    *,
    expected_dataset_tree_sha256: str | None = None,
    expected_readback_audit_sha256: str | None = None,
) -> dict:
    spec = _spec(variant)
    if variant == "2fps":
        if expected_dataset_tree_sha256 or expected_readback_audit_sha256:
            raise ValueError("external hash pins apply only to the 10fps candidate")
        return _inspect_2fps_dataset()
    return inspect_proxy_dataset(
        DATA_ROOT / spec.repo_id,
        variant=variant,
        expected_dataset_tree_sha256=expected_dataset_tree_sha256,
        expected_readback_audit_sha256=expected_readback_audit_sha256,
    )


def make_config(
    *,
    steps: int | None = None,
    exp_name: str | None = None,
    variant: str = DEFAULT_VARIANT,
) -> training_config.TrainConfig:
    spec = _spec(variant)
    if exp_name is None:
        exp_name = spec.exp_name
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,63}", exp_name):
        raise ValueError("experiment name must be a bounded simple directory name")
    if steps is None:
        steps = spec.default_steps
    model = pi0_config.Pi0Config(pi05=True, action_dim=32, action_horizon=16,
                                 paligemma_variant="gemma_2b_lora",
                                 action_expert_variant="gemma_300m_lora")
    return training_config.TrainConfig(
        name=spec.config_name,
        exp_name=exp_name,
        model=model,
        data=PiperOpenPIDataConfig(repo_id=spec.repo_id,
            assets=training_config.AssetsConfig(asset_id=spec.repo_id)),
        weight_loader=weight_loaders.CheckpointWeightLoader(
            str(ROOT / "cache/openpi/openpi-assets/checkpoints/pi05_base/params")),
        freeze_filter=model.get_freeze_filter(), ema_decay=None,
        assets_base_dir=str(ROOT / "assets"), checkpoint_base_dir=str(ROOT / "checkpoints"),
        num_train_steps=steps, batch_size=1, num_workers=0, log_interval=1,
        save_interval=steps, wandb_enabled=False,
    )


def _official(name: str):
    script = ROOT / "third_party/openpi/scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"official_{name}", script)
    if spec is None or spec.loader is None:
        raise FileNotFoundError(script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("inspect", "stats", "train"))
    parser.add_argument("--variant", choices=tuple(PROXY_DATASETS), default=DEFAULT_VARIANT)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--exp-name", default=None)
    parser.add_argument("--expected-dataset-tree-sha256", default=None)
    parser.add_argument("--expected-readback-audit-sha256", default=None)
    parser.add_argument("--allow-proxy-tiny-fit", action="store_true")
    args = parser.parse_args()
    spec = _spec(args.variant)
    source = inspect_dataset(
        args.variant,
        expected_dataset_tree_sha256=args.expected_dataset_tree_sha256,
        expected_readback_audit_sha256=args.expected_readback_audit_sha256,
    )
    print(json.dumps(source, indent=2), flush=True)
    if args.phase == "inspect":
        return
    steps = spec.default_steps if args.steps is None else args.steps
    if args.variant == "10fps" and not (
            args.expected_dataset_tree_sha256 and args.expected_readback_audit_sha256):
        raise PermissionError("10fps stats/train require laptop-pinned dataset and audit hashes")
    if not args.allow_proxy_tiny_fit or not 1 <= steps <= 10:
        raise PermissionError("offline proxy stats/train require explicit flag and 1..10 steps")
    import jax
    if jax.local_device_count() != 1:
        raise RuntimeError("proxy stats/train must expose exactly one GPU; set CUDA_VISIBLE_DEVICES=0")
    config = make_config(steps=steps, exp_name=args.exp_name, variant=args.variant)
    training_config._CONFIGS_DICT[config.name] = config
    stats = config.assets_dirs / spec.repo_id / "norm_stats.json"
    if args.phase == "stats":
        if stats.exists():
            raise FileExistsError(f"proxy stats already exist; do not overwrite: {stats}")
        _official("compute_norm_stats").main(config.name, max_frames=spec.frames)
    else:
        if not stats.is_file():
            raise FileNotFoundError(f"compute proxy stats first: {stats}")
        if config.checkpoint_dir.exists():
            raise FileExistsError(f"checkpoint exists; do not overwrite: {config.checkpoint_dir}")
        _official("train").main(config)


if __name__ == "__main__":
    main()
