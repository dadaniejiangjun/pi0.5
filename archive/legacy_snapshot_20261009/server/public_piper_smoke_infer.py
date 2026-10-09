"""One prediction from a public LeRobot sample; no robot interfaces are imported."""

from __future__ import annotations

import json
import os

import numpy as np
from openpi.policies import policy_config

from server.training.public_piper_smoke import DATA_ROOT, DATASET, make_config


def main() -> None:
    os.environ["HF_LEROBOT_HOME"] = str(DATA_ROOT)
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    config = make_config()
    checkpoint = config.checkpoint_dir / "1"
    if not checkpoint.is_dir():
        raise FileNotFoundError(checkpoint)
    dataset = LeRobotDataset(DATASET)
    sample = dataset[10]
    policy = policy_config.create_trained_policy(config, checkpoint)
    result = policy.infer({
        "base_rgb": sample["observation.images.top"].numpy(),
        "wrist_rgb": sample["observation.images.right-arm"].numpy(),
        "state": sample["observation.state"].numpy(),
        "prompt": sample["task"],
    })
    actions = np.asarray(result["actions"])
    if actions.shape != (16, 7) or not np.isfinite(actions).all():
        raise ValueError(f"unexpected output shape or non-finite predictions: {actions.shape}")
    print(json.dumps({
        "status": "PREDICTION_ONLY_PASS",
        "checkpoint": str(checkpoint),
        "action_shape": list(actions.shape),
        "first_action": actions[0].tolist(),
        "source_state": sample["observation.state"].tolist(),
        "robot_control_ready": False,
    }, indent=2))


if __name__ == "__main__":
    main()
