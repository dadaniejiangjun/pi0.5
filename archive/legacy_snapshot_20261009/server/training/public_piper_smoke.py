"""Bounded public-data pi0.5 smoke; never imports a robot or sends CAN.

This deliberately does not unlock the formal real-PiPER training entry. The
public dataset uses degrees and a binary gripper, not the local robot contract.
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = ROOT / "datasets" / "incoming"
# LeRobot resolves this at import time, so set it before importing OpenPI.
os.environ["HF_LEROBOT_HOME"] = str(DATA_ROOT)

from openpi import transforms
from openpi.models import pi0_config
from openpi.training import config as training_config
from openpi.training import weight_loaders

from server.training.piper_openpi_config import PiperOpenPIDataConfig


DATASET = "mrfood1"
SOURCE_REVISION = "33d26e9f44bccc6412339119130901a7b2924769"
SOURCE_SHA256 = "fce2210c592935fba6d0e161b55f1a0cd96f657aa7bc9457ac43a627813253db"
CONFIG_NAME = "pi05_piper_public_smoke"


@dataclasses.dataclass(frozen=True)
class PublicPiperDataConfig(PiperOpenPIDataConfig):
    def create(self, assets_dirs: Path, model_config):
        base = super().create(assets_dirs, model_config)
        return dataclasses.replace(
            base,
            repack_transforms=transforms.Group(inputs=[transforms.RepackTransform({
                "base_rgb": "observation.images.top",
                "wrist_rgb": "observation.images.right-arm",
                "state": "observation.state",
                "actions": "action",
                "prompt": "prompt",
            })]),
        )


def make_config(*, steps: int = 2, batch_size: int = 2) -> training_config.TrainConfig:
    model = pi0_config.Pi0Config(
        pi05=True,
        action_dim=32,
        action_horizon=16,
        paligemma_variant="gemma_2b_lora",
        action_expert_variant="gemma_300m_lora",
    )
    return training_config.TrainConfig(
        name=CONFIG_NAME,
        exp_name="mrfood1_two_step",
        model=model,
        data=PublicPiperDataConfig(
            repo_id=DATASET,
            assets=training_config.AssetsConfig(asset_id=DATASET),
        ),
        weight_loader=weight_loaders.CheckpointWeightLoader(
            str(ROOT / "cache/openpi/openpi-assets/checkpoints/pi05_base/params")
        ),
        freeze_filter=model.get_freeze_filter(),
        ema_decay=None,
        assets_base_dir=str(ROOT / "assets"),
        checkpoint_base_dir=str(ROOT / "checkpoints"),
        num_train_steps=steps,
        batch_size=batch_size,
        num_workers=0,
        log_interval=1,
        save_interval=steps,
        wandb_enabled=False,
    )


def inspect_dataset() -> dict:
    path = DATA_ROOT / DATASET
    info = json.loads((path / "meta/info.json").read_text(encoding="utf-8"))
    features = info["features"]
    expected = ("observation.state", "action", "observation.images.top", "observation.images.right-arm")
    if info["codebase_version"] != "v2.1" or info["robot_type"] != "agilex_piper":
        raise ValueError("unexpected public dataset version or robot type")
    if any(key not in features for key in expected):
        raise ValueError("public dataset lacks a required 7D/two-camera channel")
    if features["observation.state"]["shape"] != [7] or features["action"]["shape"] != [7]:
        raise ValueError("public dataset state/action are not 7D")
    if not (path / "data/chunk-000/episode_000000.parquet").is_file():
        raise ValueError("public dataset has no local episode")
    return {
        "source": "angkul07/piper-single-arm-cotrain/mrfood1.tar",
        "source_revision": SOURCE_REVISION,
        "archive_sha256": SOURCE_SHA256,
        "episodes": info["total_episodes"],
        "frames": info["total_frames"],
        "fps": info["fps"],
        "joint_units": "degree (dataset card)",
        "gripper_units": "binary 0=closed 1=open (dataset card)",
        "robot_control_ready": False,
    }


def _load_official_script(name: str):
    script = ROOT / "third_party" / "openpi" / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"official_{name}", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load official OpenPI script: {script}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("inspect", "stats", "train"))
    parser.add_argument("--max-frames", type=int, default=512)
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument("--allow-public-smoke", action="store_true")
    args = parser.parse_args()
    print(json.dumps(inspect_dataset(), indent=2), flush=True)
    if args.phase == "inspect":
        return
    if not args.allow_public_smoke:
        raise PermissionError("stats/train requires --allow-public-smoke")
    if args.max_frames < 32 or not (1 <= args.steps <= 10):
        raise ValueError("smoke bounds: max_frames>=32 and 1<=steps<=10")
    os.environ["HF_LEROBOT_HOME"] = str(DATA_ROOT)
    config = make_config(steps=args.steps)
    # Register only in this process. The pinned official OpenPI tree stays intact.
    training_config._CONFIGS_DICT[config.name] = config
    if args.phase == "stats":
        _load_official_script("compute_norm_stats").main(config.name, max_frames=args.max_frames)
    else:
        stats_file = config.assets_dirs / DATASET / "norm_stats.json"
        if not stats_file.is_file():
            raise FileNotFoundError(f"compute norm stats first: {stats_file}")
        if config.checkpoint_dir.exists():
            raise FileExistsError(f"smoke checkpoint already exists; never overwrite: {config.checkpoint_dir}")
        _load_official_script("train").main(config)


if __name__ == "__main__":
    main()
