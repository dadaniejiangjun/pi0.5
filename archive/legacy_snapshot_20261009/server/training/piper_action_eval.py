"""Offline PiPER command-target evaluation; predictions are never sent to CAN."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from server.policy_server.artifact_identity import tree_sha256


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _dataset_tree_sha256(root: Path) -> str:
    root = Path(root).resolve()
    files = {}
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError("dataset symlinks are not accepted")
        if path.is_file():
            files[path.relative_to(root).as_posix()] = _sha256(path)
    if not files:
        raise ValueError("dataset is empty")
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _check_checkpoint_provenance(checkpoint: Path, training: dict) -> None:
    checkpoint = Path(checkpoint).resolve()
    if (Path(training.get("checkpoint_path", "")).resolve() != checkpoint
            or training.get("checkpoint_identity_scope") != "selected_numbered_openpi_checkpoint_tree"
            or tree_sha256(checkpoint) != training.get("checkpoint_sha256")):
        raise ValueError("checkpoint bytes do not match the training provenance")
    checkpoint_stats = checkpoint / "assets" / "piper" / "norm_stats.json"
    if (not checkpoint_stats.is_file()
            or _sha256(checkpoint_stats) != training.get("norm_stats_sha256")):
        raise ValueError("checkpoint normalization differs from training provenance")


def _validation_rows(dataset: Path, audit_manifest: Path | None) -> tuple[list[int], str]:
    if audit_manifest is None:
        raise ValueError("pass the train/validation audit manifest explicitly")
    audit = json.loads(Path(audit_manifest).read_text(encoding="utf-8"))
    expected_hash = _dataset_tree_sha256(dataset)
    if audit.get("dataset_sha256") != expected_hash:
        raise ValueError("split audit is not bound to this exact dataset")
    conversion = json.loads((dataset / "provenance/conversion_manifest.json").read_text(encoding="utf-8"))
    source_path = dataset / "provenance/source_map.jsonl"
    if _sha256(source_path) != conversion.get("source_map_sha256"):
        raise ValueError("dataset source map hash mismatch")
    valid_ids = set(audit.get("validation_episode_ids", []))
    rows = [json.loads(line) for line in source_path.read_text(encoding="utf-8").splitlines()]
    selected = [index for index, row in enumerate(rows)
                if row.get("raw_episode_id") in valid_ids]
    if selected:
        return selected, "HELD_OUT_EPISODE_EVALUATION"
    if audit.get("status") != "TRAIN_ONLY_NO_VALIDATION":
        raise ValueError("validation split is empty but audit does not declare train-only")
    return list(range(len(rows))), "TRAIN_ONLY_SAME_EPISODE_REPLAY_NO_VALIDATION"


def evaluate(dataset_path: Path, checkpoint: Path, training_provenance_path: Path,
             audit_manifest: Path, *, max_samples: int = 20) -> dict:
    """Evaluate first predicted absolute targets against source-verified commands."""
    if type(max_samples) is not int or not 1 <= max_samples <= 100:
        raise ValueError("max_samples must be in [1, 100]")
    dataset_path = Path(dataset_path).resolve()
    checkpoint = Path(checkpoint).resolve()
    training_provenance_path = Path(training_provenance_path).resolve()
    training = json.loads(training_provenance_path.read_text(encoding="utf-8"))
    if (training.get("schema") != "piper_offline_training_provenance_v1"
            or training.get("action_semantics") != "absolute_joint_position_rad_gripper_meter"
            or training.get("prediction_only") is not True
            or training.get("hardware_control_allowed") is not False
            or not checkpoint.is_dir()):
        raise ValueError("checkpoint/provenance is not an offline PiPER action model")
    _check_checkpoint_provenance(checkpoint, training)
    if training.get("dataset_sha256") != _dataset_tree_sha256(dataset_path):
        raise ValueError("checkpoint provenance names a different dataset tree")
    stats_path = Path(training.get("norm_stats_path", "")).resolve()
    if (not stats_path.is_file()
            or _sha256(stats_path) != training.get("norm_stats_sha256")):
        raise ValueError("checkpoint provenance names missing or changed PiPER statistics")
    repo_id = training.get("repo_id")
    if not isinstance(repo_id, str) or len(Path(repo_id).parts) != 2:
        raise ValueError("invalid LeRobot repo ID in training provenance")
    dataset_home = dataset_path
    for _ in Path(repo_id).parts:
        dataset_home = dataset_home.parent
    if (dataset_home / repo_id).resolve() != dataset_path:
        raise ValueError("dataset path does not match repo ID")
    indices, evaluation_scope = _validation_rows(dataset_path, audit_manifest)
    if not indices:
        raise ValueError("evaluation split has no frames")

    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from openpi.policies import policy_config
    from server.training.piper_train_entry import build_config

    config = build_config(
        Path(__file__).resolve().parents[2], repo_id=repo_id,
        base_params_path=training["base_params_path"],
        mode=training["mode"], steps=training["steps"],
    )
    expected_config_root = config.checkpoint_dir.resolve()
    if not checkpoint.is_relative_to(expected_config_root):
        raise ValueError("checkpoint path is outside the recorded experiment")
    dataset = LeRobotDataset(repo_id, root=dataset_home)
    if len(dataset) <= max(indices):
        raise ValueError("dataset length differs from its source map")
    policy = policy_config.create_trained_policy(config, checkpoint)
    sample_indices = sorted(set(np.linspace(
        0, len(indices) - 1, min(max_samples, len(indices)), dtype=int
    ).tolist()))
    samples = []
    for position in sample_indices:
        row_index = indices[position]
        row = dataset[row_index]
        observation = {
            "base_rgb": row["observation.images.image"].numpy(),
            "wrist_rgb": row["observation.images.wrist_image"].numpy(),
            "state": row["observation.state"].numpy(),
            "prompt": row["task"],
        }
        predicted = np.asarray(policy.infer(observation)["actions"], dtype=np.float64)
        target = np.asarray(row["action"], dtype=np.float64)
        if (predicted.ndim != 2 or predicted.shape[1] != 7 or predicted.shape[0] < 1
                or target.shape != (7,) or not np.isfinite(predicted).all()
                or not np.isfinite(target).all()):
            raise ValueError("model output and command target must be finite 7D action vectors")
        error = predicted[0] - target
        source_map = json.loads((dataset_path / "provenance/source_map.jsonl").read_text(
            encoding="utf-8").splitlines()[row_index])
        samples.append({
            "frame_index": row_index,
            "raw_episode_id": source_map["raw_episode_id"],
            "target": target.tolist(),
            "predicted_first_action": predicted[0].tolist(),
            "joint_mae_rad": float(np.mean(np.abs(error[:6]))),
            "gripper_abs_error_m_nominal": float(abs(error[6])),
        })
    return {
        "schema": "piper_action_offline_evaluation_v1",
        "status": evaluation_scope,
        "evaluation_scope": evaluation_scope,
        "dataset_sha256": _dataset_tree_sha256(dataset_path),
        "checkpoint": str(checkpoint),
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "evaluated_frames": [sample["frame_index"] for sample in samples],
        "mean_joint_mae_rad": float(np.mean([sample["joint_mae_rad"] for sample in samples])),
        "mean_gripper_abs_error_m_nominal": float(np.mean(
            [sample["gripper_abs_error_m_nominal"] for sample in samples])),
        "samples": samples,
        "generalization_test": evaluation_scope == "HELD_OUT_EPISODE_EVALUATION",
        "prediction_only": True,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--training-provenance", required=True, type=Path)
    parser.add_argument("--audit-manifest", required=True, type=Path)
    parser.add_argument("--max-samples", type=int, default=20)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.dataset, args.checkpoint, args.training_provenance,
                              args.audit_manifest, max_samples=args.max_samples), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
