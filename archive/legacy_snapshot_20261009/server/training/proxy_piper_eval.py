"""Offline same-episode proxy comparison; never sends predictions to PiPER."""

from __future__ import annotations

import argparse
import json
import os

import numpy as np

from server.training.proxy_dataset_contract import PROXY_DATASETS
from server.training.proxy_piper_tiny_fit import DATA_ROOT, inspect_dataset, make_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=tuple(PROXY_DATASETS), default="2fps")
    parser.add_argument("--exp-name", default=None)
    parser.add_argument("--step", type=int, default=None)
    parser.add_argument("--expected-dataset-tree-sha256", default=None)
    parser.add_argument("--expected-readback-audit-sha256", default=None)
    args = parser.parse_args()
    spec = PROXY_DATASETS[args.variant]
    step = spec.eval_step if args.step is None else args.step
    if step < 1 or step > 10:
        raise ValueError("bounded proxy checkpoint step required")
    if args.variant == "10fps" and not (
            args.expected_dataset_tree_sha256 and args.expected_readback_audit_sha256):
        raise PermissionError("10fps evaluation requires laptop-pinned dataset and audit hashes")
    source = inspect_dataset(
        args.variant,
        expected_dataset_tree_sha256=args.expected_dataset_tree_sha256,
        expected_readback_audit_sha256=args.expected_readback_audit_sha256,
    )
    os.environ["HF_LEROBOT_HOME"] = str(DATA_ROOT)
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from openpi.policies import policy_config

    config = make_config(exp_name=args.exp_name, variant=args.variant)
    checkpoint = config.checkpoint_dir / str(step)
    if not checkpoint.is_dir():
        raise FileNotFoundError(checkpoint)
    dataset = LeRobotDataset(spec.repo_id)
    policy = policy_config.create_trained_policy(config, checkpoint)
    samples = []
    indices = (0, 10, 20, 30, 38) if args.variant == "2fps" else (0, 50, 100, 150, 198)
    for index in indices:
        row = dataset[index]
        observation = {
            "base_rgb": row["observation.images.image"].numpy(),
            "wrist_rgb": row["observation.images.wrist_image"].numpy(),
            "state": row["observation.state"].numpy(),
            "prompt": row["task"],
        }
        predicted = np.asarray(policy.infer(observation)["actions"])
        target = np.asarray(row["action"], dtype=np.float32)
        if predicted.shape != (16, 7) or not np.isfinite(predicted).all():
            raise ValueError("policy output is not finite 16x7")
        error = predicted[0] - target
        samples.append({
            "frame_index": index,
            "target": target.tolist(),
            "predicted_first": predicted[0].tolist(),
            "joint_mae_rad": float(np.mean(np.abs(error[:6]))),
            "gripper_abs_error_m_nominal": float(abs(error[6])),
        })
    print(json.dumps({
        "status": "SAME_EPISODE_PREDICTION_ONLY",
        "source": source,
        "checkpoint": str(checkpoint),
        "dataset_variant": args.variant,
        "evaluated_frames": [item["frame_index"] for item in samples],
        "mean_joint_mae_rad": float(np.mean([item["joint_mae_rad"] for item in samples])),
        "mean_gripper_abs_error_m_nominal": float(np.mean([item["gripper_abs_error_m_nominal"] for item in samples])),
        "samples": samples,
        "generalization_test": False,
        "hardware_control_allowed": False,
    }, indent=2))


if __name__ == "__main__":
    main()
