"""Tiny synthetic LeRobot write/read test in a temporary directory only.

No raw PiPER data is read, no model is trained, and no robot is contacted.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import numpy as np
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from openpi import transforms

from server.training.piper_openpi_config import make_unregistered_config


REPO_ID = "offline/piper_synthetic_smoke"


def run(root: Path) -> dict:
    root = Path(root)
    if root.exists():
        raise FileExistsError(root)
    features = {
        "observation.images.image": {"dtype": "video", "shape": (64, 64, 3), "names": ["height", "width", "channel"]},
        "observation.images.wrist_image": {"dtype": "video", "shape": (64, 64, 3), "names": ["height", "width", "channel"]},
        "observation.state": {"dtype": "float32", "shape": (7,), "names": ["state"]},
        "action": {"dtype": "float32", "shape": (7,), "names": ["action"]},
    }
    dataset = LeRobotDataset.create(repo_id=REPO_ID, fps=10, root=root, robot_type="piper", features=features)
    for index in range(3):
        base = np.full((64, 64, 3), index * 20, dtype=np.uint8)
        wrist = np.full((64, 64, 3), 100 + index * 20, dtype=np.uint8)
        dataset.add_frame({
            "observation.images.image": base,
            "observation.images.wrist_image": wrist,
            "observation.state": np.full(7, index / 100, dtype=np.float32),
            "action": np.full(7, (index + 1) / 100, dtype=np.float32),
            "task": "SYNTHETIC CONTRACT TEST ONLY",
        })
    dataset.save_episode()
    reopened = LeRobotDataset(REPO_ID, root=root, delta_timestamps={"action": [0.0, 0.1]})
    sample = reopened[0]
    required = set(features)
    if len(reopened) != 3 or not required.issubset(sample):
        raise RuntimeError("LeRobot write/read contract failed")
    config = make_unregistered_config(
        repo_id=REPO_ID, base_params_path="/nonexistent/synthetic-smoke-params", assets_root=str(root.parent)
    )
    data_config = config.data.create(root.parent, config.model)
    prompted = transforms.PromptFromLeRobotTask(reopened.meta.tasks)(sample)
    repacked = data_config.repack_transforms.inputs[0](prompted)
    model_input = data_config.data_transforms.inputs[0](repacked)
    if model_input["actions"].shape != (2, 7) or model_input["state"].shape != (7,):
        raise RuntimeError("LeRobot to PiPER OpenPI transform failed")
    return {
        "status": "SYNTHETIC_LEROBOT_TO_OPENPI_PASS",
        "episodes": reopened.meta.total_episodes,
        "frames": len(reopened),
        "training_eligible": False,
        "dataset_path": str(root),
    }


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="pi05-lerobot-smoke-") as temp:
        report = run(Path(temp) / "dataset")
        report["dataset_path"] = "temporary directory removed after test"
        print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
