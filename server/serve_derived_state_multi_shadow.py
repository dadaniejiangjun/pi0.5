"""Serve a multi-episode derived-state Pi0.5 checkpoint for prediction only.

This entrypoint accepts only a hash-bound, train-only LeRobot dataset and its
matching pilot checkpoint. It binds WebSocket to loopback and exposes no CAN
or robot transport.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Callable

from server.training.proxy_multi_episode_contract import (
    ACTION_SEMANTICS,
    ACTION_TARGET_KIND,
)
from server.training.proxy_multi_episode_pilot import prepare, build_config


ROOT = Path(__file__).resolve().parents[1]
TRAINING_SCHEMA = "piper_derived_state_multi_episode_training_v1"
SERVER_SCHEMA = "pi05_derived_state_multi_episode_shadow_v1"
ARTIFACT_IDENTITY_SCHEMA = "piper_derived_state_multi_shadow_identity_v1"
ACTION_HORIZON = 16
ACTION_DIMENSION = 7
FIRST_TARGET_DT_S = 0.1


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_runtime(*, host: str = "127.0.0.1", port: int = 8020,
                     sampling_steps: int = 10) -> None:
    if host not in {"127.0.0.1", "localhost"}:
        raise ValueError("derived-state shadow may bind to loopback only")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port must be an integer in [1, 65535]")
    if type(sampling_steps) is not int or not 1 <= sampling_steps <= 10:
        raise ValueError("sampling_steps must be an integer in [1, 10]")


def validate_training_record(training: dict[str, Any], *, dataset_path: Path,
                             checkpoint: Path, dataset_sha256: str,
                             checkpoint_sha256: str, stats_sha256: str,
                             checkpoint_stats_sha256: str,
                             current_openpi_commit: str) -> None:
    """Reject any checkpoint not bound to this exact derived-state dataset."""
    if (training.get("schema") != TRAINING_SCHEMA
            or training.get("action_semantics") != ACTION_SEMANTICS
            or training.get("action_target_kind") != ACTION_TARGET_KIND
            or training.get("prediction_only") is not True
            or training.get("hardware_control_allowed") is not False
            or training.get("training_scope") != "all_successful_episodes_train_only_no_holdout"
            or training.get("same_data_evaluation_only") is not True):
        raise ValueError("training provenance is not an isolated prediction-only derived-state pilot")
    if (Path(training.get("dataset_path", "")).resolve() != dataset_path.resolve()
            or training.get("dataset_sha256") != dataset_sha256):
        raise ValueError("training provenance does not match the exact LeRobot dataset")
    if (Path(training.get("checkpoint_path", "")).resolve() != checkpoint.resolve()
            or training.get("checkpoint_tree_sha256") != checkpoint_sha256):
        raise ValueError("training provenance does not match the exact checkpoint")
    if (training.get("norm_stats_sha256") != stats_sha256
            or stats_sha256 != checkpoint_stats_sha256):
        raise ValueError("training and checkpoint normalization identities differ")
    if training.get("openpi_commit") != current_openpi_commit:
        raise ValueError("checkpoint provenance OpenPI commit differs from this checkout")
    episodes = training.get("dataset_episodes")
    if type(episodes) is not int or episodes < 1:
        raise ValueError("derived-state shadow requires at least one successful source episode")


def inspect_artifacts(dataset_path: Path, checkpoint: Path,
                      training_provenance_path: Path) -> dict[str, Any]:
    dataset_path = Path(dataset_path).resolve(strict=True)
    checkpoint = Path(checkpoint).resolve(strict=True)
    provenance_path = Path(training_provenance_path).resolve(strict=True)
    training = json.loads(provenance_path.read_text(encoding="utf-8"))
    expected_dataset = training.get("dataset_sha256")
    if not isinstance(expected_dataset, str) or len(expected_dataset) != 64:
        raise ValueError("training provenance lacks a dataset SHA-256")

    episode_count = training.get("dataset_episodes")
    if type(episode_count) is not int or episode_count < 1:
        raise ValueError("training provenance must name at least one source episode")
    audit = prepare(dataset_path, min_episodes=episode_count,
                    expected_dataset_sha256=expected_dataset)
    if audit.get("episodes") != episode_count:
        raise ValueError("training provenance episode count differs from the audited dataset")
    from server.policy_server.artifact_identity import tree_sha256

    checkpoint_sha = tree_sha256(checkpoint)
    stats_path = Path(training.get("stats_path", "")).resolve(strict=True)
    checkpoint_stats_path = checkpoint / "assets" / "piper" / "norm_stats.json"
    if not checkpoint_stats_path.is_file():
        raise FileNotFoundError("checkpoint is missing its bound PiPER normalization stats")
    current_openpi_commit = json.loads(
        (ROOT / "provenance" / "openpi_git.json").read_text(encoding="utf-8")
    )["head_sha"]
    stats_sha = _sha256(stats_path)
    checkpoint_stats_sha = _sha256(checkpoint_stats_path)
    validate_training_record(
        training, dataset_path=dataset_path, checkpoint=checkpoint,
        dataset_sha256=audit["dataset_sha256"], checkpoint_sha256=checkpoint_sha,
        stats_sha256=stats_sha, checkpoint_stats_sha256=checkpoint_stats_sha,
        current_openpi_commit=current_openpi_commit,
    )
    if training.get("repo_id") != audit.get("repo_id"):
        raise ValueError("training provenance repo_id differs from the dataset")
    return {
        "audit": audit,
        "training": training,
        "training_provenance_path": provenance_path,
        "dataset_path": dataset_path,
        "checkpoint": checkpoint,
        "base_params_path": Path(training["base_params_path"]).resolve(strict=True),
        "stats_path": stats_path,
        "checkpoint_sha256": checkpoint_sha,
        "normalization_sha256": stats_sha,
        "openpi_commit": current_openpi_commit,
    }


def build_server_metadata(artifacts: dict[str, Any], *, config_sha256: str,
                          sampling_steps: int) -> dict[str, Any]:
    training = artifacts["training"]
    return {
        "project": "07_pi0.5",
        "artifact_identity_schema": ARTIFACT_IDENTITY_SCHEMA,
        "config_schema": SERVER_SCHEMA,
        "dataset": training["repo_id"],
        "dataset_sha256": training["dataset_sha256"],
        "dataset_identity": training["repo_id"],
        "action_semantics": ACTION_SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "action_horizon": ACTION_HORIZON,
        "action_dimension": ACTION_DIMENSION,
        "first_target_dt_s": FIRST_TARGET_DT_S,
        "raw_command_supervision": False,
        "checkpoint": str(artifacts["checkpoint"]),
        "checkpoint_tree_sha256": artifacts["checkpoint_sha256"],
        "normalization_identity": f"train_only_stats:{training['repo_id']}",
        "normalization_sha256": artifacts["normalization_sha256"],
        "openpi_commit": artifacts["openpi_commit"],
        "training_provenance_sha256": config_sha256,
        "sampling_steps": sampling_steps,
        "prediction_only": True,
        "hardware_control_allowed": False,
        "robot_control_ready": False,
        "control_enabled": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }


def create_server(artifacts: dict[str, Any], *, host: str, port: int,
                  sampling_steps: int,
                  policy_factory: Callable[..., Any] | None = None,
                  server_factory: Callable[..., Any] | None = None) -> Any:
    validate_runtime(host=host, port=port, sampling_steps=sampling_steps)
    audit = artifacts["audit"]
    training = artifacts["training"]
    os.environ["HF_LEROBOT_HOME"] = audit["hf_lerobot_home"]
    config = build_config(
        repo_id=audit["repo_id"], base_params_path=artifacts["base_params_path"],
        steps=training["steps"], exp_name=training["experiment_name"],
    )
    if policy_factory is None:
        from openpi.policies import policy_config
        policy_factory = policy_config.create_trained_policy
    if server_factory is None:
        from openpi.serving.websocket_policy_server import WebsocketPolicyServer
        server_factory = WebsocketPolicyServer
    policy = policy_factory(config, artifacts["checkpoint"],
                            sample_kwargs={"num_steps": sampling_steps})
    metadata = build_server_metadata(
        artifacts,
        config_sha256=_sha256(Path(artifacts["training_provenance_path"])),
        sampling_steps=sampling_steps,
    )
    return server_factory(policy, host=host, port=port, metadata=metadata)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-path", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--training-provenance", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8020)
    parser.add_argument("--sampling-steps", type=int, default=10)
    args = parser.parse_args()
    validate_runtime(host=args.host, port=args.port, sampling_steps=args.sampling_steps)
    artifacts = inspect_artifacts(args.dataset_path, args.checkpoint,
                                  args.training_provenance)
    server = create_server(artifacts, host=args.host, port=args.port,
                           sampling_steps=args.sampling_steps)
    print(f"PREDICTION_ONLY_LISTEN={args.host}:{args.port}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
