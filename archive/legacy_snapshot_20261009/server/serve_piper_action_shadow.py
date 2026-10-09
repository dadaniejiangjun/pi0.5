"""Loopback-only server for a fine-tuned PiPER action checkpoint.

The server returns action-shaped predictions for offline shadow inspection.
It imports no robot/CAN transport and advertises control as disabled.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from openpi.policies import policy_config
from openpi.serving.websocket_policy_server import WebsocketPolicyServer

from server.policy_server.artifact_identity import (
    files_sha256,
    sha256_file,
    tree_sha256,
)
from server.training.piper_train_entry import build_config


ROOT = Path(__file__).resolve().parents[1]
LOCAL_HOST = "127.0.0.1"
ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_meter"
POLICY_PIPELINE_FILES = (
    "server/serve_piper_action_shadow.py",
    "server/policy_server/artifact_identity.py",
    "server/training/piper_train_entry.py",
    "server/training/piper_openpi_config.py",
)


def build_policy(dataset_path: Path, checkpoint_path: Path,
                 training_provenance_path: Path, *, port: int = 8004,
                 sampling_steps: int = 10):
    if not 1 <= port <= 65535 or not 1 <= sampling_steps <= 10:
        raise ValueError("port or sampling steps outside bounded shadow settings")
    dataset_path = Path(dataset_path).resolve()
    checkpoint_path = Path(checkpoint_path).resolve()
    provenance_path = Path(training_provenance_path).resolve()
    training = json.loads(provenance_path.read_text(encoding="utf-8"))
    if (training.get("schema") != "piper_offline_training_provenance_v1"
            or training.get("action_semantics") != ACTION_SEMANTICS
            or training.get("prediction_only") is not True
            or training.get("hardware_control_allowed") is not False):
        raise PermissionError("checkpoint is not pinned as a prediction-only PiPER action model")
    if training.get("dataset_sha256") != tree_sha256(dataset_path):
        raise ValueError("dataset bytes differ from training provenance")
    stats_path = Path(training.get("norm_stats_path", "")).resolve()
    if not stats_path.is_file() or sha256_file(stats_path) != training.get("norm_stats_sha256"):
        raise ValueError("normalization stats differ from training provenance")
    repo_id = training.get("repo_id")
    if not isinstance(repo_id, str) or len(Path(repo_id).parts) != 2:
        raise ValueError("training provenance has invalid LeRobot repo ID")
    data_home = dataset_path
    for _ in Path(repo_id).parts:
        data_home = data_home.parent
    if (data_home / repo_id).resolve() != dataset_path:
        raise ValueError("dataset path does not match repo ID")
    config = build_config(
        ROOT,
        repo_id=repo_id,
        base_params_path=training["base_params_path"],
        mode=training["mode"],
        steps=training["steps"],
    )
    if provenance_path != (config.checkpoint_dir / "training_provenance.json").resolve():
        raise ValueError("training provenance must be the sidecar for this experiment")
    if not checkpoint_path.is_relative_to(config.checkpoint_dir.resolve()):
        raise ValueError("checkpoint is outside the recorded experiment directory")
    if not checkpoint_path.is_dir():
        raise FileNotFoundError(checkpoint_path)
    if (Path(training.get("checkpoint_path", "")).resolve() != checkpoint_path
            or training.get("checkpoint_identity_scope") != "selected_numbered_openpi_checkpoint_tree"
            or tree_sha256(checkpoint_path) != training.get("checkpoint_sha256")):
        raise ValueError("checkpoint bytes do not match the training provenance")
    normalization = config.assets_dirs / "piper" / "norm_stats.json"
    if normalization.resolve() != stats_path:
        raise ValueError("normalization path differs from the OpenPI training config")
    checkpoint_normalization = checkpoint_path / "assets" / "piper" / "norm_stats.json"
    if (not checkpoint_normalization.is_file()
            or sha256_file(checkpoint_normalization) != training.get("norm_stats_sha256")):
        raise ValueError("checkpoint normalization does not match training provenance")

    identities = {
        "artifact_identity_schema": "piper_policy_artifact_identity_v1",
        "config_schema": "pi05_piper_action_shadow_v1",
        "checkpoint_sha256": training["checkpoint_sha256"],
        "policy_config_sha256": sha256_file(ROOT / "server/training/piper_openpi_config.py"),
        "policy_pipeline_code_sha256": files_sha256(ROOT, POLICY_PIPELINE_FILES),
        "training_config_code_sha256": sha256_file(ROOT / "server/training/piper_train_entry.py"),
        "normalization_sha256": sha256_file(checkpoint_normalization),
        "dataset_tree_sha256": tree_sha256(dataset_path),
        "openpi_commit": json.loads(
            (ROOT / "provenance/openpi_git.json").read_text(encoding="utf-8")
        )["head_sha"],
    }
    policy = policy_config.create_trained_policy(
        config, checkpoint_path, sample_kwargs={"num_steps": sampling_steps})
    metadata = {
        "project": "07_pi0.5",
        "config": config.name,
        "config_schema": identities["config_schema"],
        "checkpoint": str(checkpoint_path.relative_to(ROOT).as_posix()),
        "checkpoint_identity_scope": training["checkpoint_identity_scope"],
        "dataset": repo_id,
        "normalization_identity": "piper_action_train_split_stats",
        **identities,
        "action_semantics": ACTION_SEMANTICS,
        "action_horizon": config.model.action_horizon,
        "action_dimension": 7,
        "sampling_steps": sampling_steps,
        "units_verified": False,
        "prediction_only": True,
        "hardware_control_allowed": False,
        "robot_control_ready": False,
        "control_enabled": False,
        "bind_address": LOCAL_HOST,
    }
    return policy, metadata


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--training-provenance", required=True, type=Path)
    parser.add_argument("--port", type=int, default=8004)
    parser.add_argument("--sampling-steps", type=int, default=10)
    args = parser.parse_args()
    policy, metadata = build_policy(
        args.dataset, args.checkpoint, args.training_provenance,
        port=args.port, sampling_steps=args.sampling_steps)
    server = WebsocketPolicyServer(
        policy, host=LOCAL_HOST, port=args.port, metadata=metadata)
    print(json.dumps({"status": "ACTION_SHADOW_PREDICTION_ONLY_READY",
                      "host": LOCAL_HOST, "port": args.port,
                      "metadata": metadata,
                      "hardware_control_allowed": False,
                      "can_data_tx_calls": 0}, indent=2), flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
