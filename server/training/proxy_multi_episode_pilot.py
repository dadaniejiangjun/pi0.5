"""Train/evaluate a bounded multi-episode derived-state Pi0.5 pilot offline.

This path is intentionally isolated from command-supervised training and all
robot interfaces. Predictions remain prediction-only; CAN TX is not available.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
from datetime import datetime, timezone

from server.training.proxy_multi_episode_contract import (
    ACTION_SEMANTICS,
    inspect_proxy_multi_episode,
)


ROOT = Path(__file__).resolve().parents[2]
DATA_HOME = ROOT / "datasets" / "incoming"
CONFIG_PREFIX = "pi05_derived_state_"
DEFAULT_BASE_PARAMS = ROOT / "cache/openpi/openpi-assets/checkpoints/pi05_base/params"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _tree_sha256(root: Path) -> str:
    root = Path(root).resolve(strict=True)
    digest = hashlib.sha256()
    files = []
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"checkpoint contains a symlink: {path}")
        if path.is_file():
            files.append((path.relative_to(root).as_posix(), path))
    if not files:
        raise ValueError("checkpoint is empty")
    for relative, path in sorted(files):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _resolve_dataset(dataset_path: Path, repo_id: str | None) -> tuple[Path, str, Path]:
    dataset_path = Path(dataset_path).resolve(strict=True)
    conversion = json.loads((dataset_path / "provenance/proxy_conversion.json").read_text(encoding="utf-8"))
    resolved_repo_id = repo_id or conversion.get("repo_id")
    if not isinstance(resolved_repo_id, str) or len(resolved_repo_id.split("/")) != 2:
        raise ValueError("repo_id must have namespace/name form")
    home = dataset_path
    for _ in resolved_repo_id.split("/"):
        home = home.parent
    if (home / resolved_repo_id).resolve() != dataset_path:
        raise ValueError("dataset_path must equal HF_LEROBOT_HOME/repo_id")
    return dataset_path, resolved_repo_id, home


def _config_identity(repo_id: str) -> tuple[str, str]:
    suffix = re.sub(r"[^A-Za-z0-9_-]", "_", repo_id.split("/")[-1])[:36]
    return f"{CONFIG_PREFIX}{suffix}", f"{suffix}_pilot"


def norm_stats_path(config, repo_id: str) -> Path:
    """Return the named stats asset consumed by the PiPER config and checkpoint."""
    if not isinstance(repo_id, str) or re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_-]+", repo_id) is None:
        raise ValueError("repo_id must contain a safe namespace/name pair")
    return Path(config.assets_dirs) / "piper" / "norm_stats.json"


def openpi_norm_stats_source_path(config, repo_id: str) -> Path:
    """Return the repo-id-scoped path written by OpenPI's stats script."""
    if not isinstance(repo_id, str) or re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_-]+", repo_id) is None:
        raise ValueError("repo_id must contain a safe namespace/name pair")
    return Path(config.assets_dirs).joinpath(*repo_id.split("/"), "norm_stats.json")


def materialize_norm_stats(config, repo_id: str) -> Path:
    """Copy OpenPI's repo-scoped stats into the PiPER asset id without overwrite."""
    source = openpi_norm_stats_source_path(config, repo_id)
    target = norm_stats_path(config, repo_id)
    if not source.is_file():
        raise FileNotFoundError(f"OpenPI did not write expected statistics: {source}")
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as handle:
        handle.write(source.read_bytes())
    return target


def local_lerobot_dataset_root(audit: dict) -> Path:
    """Resolve a prepared local dataset and refuse an implicit Hub fallback."""
    dataset_root = Path(audit["dataset_path"]).resolve(strict=True)
    if not (dataset_root / "meta" / "info.json").is_file():
        raise FileNotFoundError(
            f"prepared local LeRobot dataset metadata is missing: {dataset_root / 'meta' / 'info.json'}"
        )
    return dataset_root


def build_config(*, repo_id: str, base_params_path: Path, steps: int,
                 exp_name: str, save_interval: int | None = None,
                 keep_period: int | None = None):
    from openpi import training as _training  # noqa: F401 - import verifies the runtime lazily
    from openpi.training import config as training_config
    from server.training.piper_openpi_config import make_unregistered_config

    config_name, default_exp = _config_identity(repo_id)
    if not exp_name:
        exp_name = default_exp
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,63}", exp_name) is None:
        raise ValueError("experiment name must be a bounded simple directory name")
    candidate = make_unregistered_config(
        repo_id=repo_id,
        base_params_path=str(Path(base_params_path).resolve()),
        assets_root=str(ROOT / "assets" / config_name),
    )
    return dataclasses.replace(
        candidate,
        name=config_name,
        exp_name=exp_name,
        data=dataclasses.replace(
            candidate.data,
            assets=dataclasses.replace(candidate.data.assets, assets_dir=None),
        ),
        assets_base_dir=str(ROOT / "assets"),
        checkpoint_base_dir=str(ROOT / "checkpoints"),
        num_train_steps=steps,
        batch_size=1,
        num_workers=0,
        log_interval=1,
        save_interval=steps if save_interval is None else save_interval,
        keep_period=keep_period if keep_period is not None else candidate.keep_period,
        wandb_enabled=False,
    )


def _official(name: str):
    script = ROOT / "third_party/openpi/scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"derived_proxy_{name}", script)
    if spec is None or spec.loader is None:
        raise FileNotFoundError(script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare(dataset_path: Path, *, repo_id: str | None = None,
            min_episodes: int = 5, expected_dataset_sha256: str | None = None) -> dict:
    dataset_path, repo_id, home = _resolve_dataset(dataset_path, repo_id)
    audit = inspect_proxy_multi_episode(
        dataset_path, min_episodes=min_episodes,
        expected_dataset_sha256=expected_dataset_sha256,
    )
    audit.update({"repo_id": repo_id, "hf_lerobot_home": str(home),
                  "training_started": False, "hardware_control_allowed": False})
    return audit


def execute(dataset_path: Path, *, phase: str, repo_id: str | None = None,
            base_params_path: Path = DEFAULT_BASE_PARAMS, min_episodes: int = 5,
            steps: int = 20, max_frames: int = 100_000, exp_name: str = "",
            expected_dataset_sha256: str | None = None,
            allow_derived_state_pilot: bool = False,
            allow_long_training: bool = False,
            save_interval: int | None = None, keep_period: int | None = None) -> dict:
    if not allow_derived_state_pilot:
        raise PermissionError("offline pilot locked; pass --allow-derived-state-pilot explicitly")
    if phase not in ("stats", "train", "eval"):
        raise ValueError("phase must be stats, train, or eval")
    max_steps = 50_000 if allow_long_training is True else 100
    if type(steps) is not int or not 1 <= steps <= max_steps or type(max_frames) is not int or max_frames < 1:
        raise ValueError(f"training limits must be 1..{max_steps} optimizer steps and positive max_frames")
    audit = prepare(dataset_path, repo_id=repo_id, min_episodes=min_episodes,
                    expected_dataset_sha256=expected_dataset_sha256)
    repo_id = audit["repo_id"]
    dataset_path = Path(audit["dataset_path"])
    dataset_sha = audit["dataset_sha256"]
    if not Path(base_params_path).is_dir():
        raise FileNotFoundError(f"pi05_base params directory not found: {base_params_path}")
    os.environ["HF_LEROBOT_HOME"] = audit["hf_lerobot_home"]
    config = build_config(repo_id=repo_id, base_params_path=base_params_path,
                          steps=steps, exp_name=exp_name,
                          save_interval=save_interval, keep_period=keep_period)
    from openpi.training import config as training_config
    training_config._CONFIGS_DICT[config.name] = config
    stats_path = norm_stats_path(config, repo_id)
    stats_provenance_path = stats_path.with_name("norm_stats.provenance.json")
    openpi_stats_source_path = openpi_norm_stats_source_path(config, repo_id)
    checkpoint_root = config.checkpoint_dir
    if phase == "stats":
        if stats_path.exists() or stats_provenance_path.exists() or openpi_stats_source_path.exists():
            raise FileExistsError(f"refusing to overwrite existing pilot stats: {stats_path}")
        _official("compute_norm_stats").main(config.name, max_frames=max_frames)
        materialize_norm_stats(config, repo_id)
        stats_provenance = {
            "schema": "piper_derived_state_pilot_norm_stats_v1",
            "repo_id": repo_id,
            "dataset_path": str(dataset_path),
            "dataset_sha256": dataset_sha,
            "norm_stats_sha256": _sha256(stats_path),
            "openpi_norm_stats_path": str(openpi_stats_source_path),
            "openpi_norm_stats_sha256": _sha256(openpi_stats_source_path),
            "action_semantics": ACTION_SEMANTICS,
            "episodes": audit["episodes"],
            "frames": audit["frames"],
            "stats_source_split": "all_episodes_train_only",
            "hardware_control_allowed": False,
        }
        stats_provenance_path.parent.mkdir(parents=True, exist_ok=True)
        stats_provenance_path.write_text(
            json.dumps(stats_provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return {"phase": phase, "repo_id": repo_id, "dataset_sha256": dataset_sha,
                "norm_stats_sha256": stats_provenance["norm_stats_sha256"],
                "stats_provenance": str(stats_provenance_path),
                "training_started": False, "hardware_control_allowed": False}

    if phase == "train":
        if checkpoint_root.exists():
            raise FileExistsError(f"refusing to overwrite pilot checkpoint directory: {checkpoint_root}")
        if not stats_path.is_file() or not stats_provenance_path.is_file():
            raise FileNotFoundError("compute and provenance-bind train-only pilot stats first")
        stats_provenance = json.loads(stats_provenance_path.read_text(encoding="utf-8"))
        if (stats_provenance.get("schema") != "piper_derived_state_pilot_norm_stats_v1"
                or stats_provenance.get("repo_id") != repo_id
                or stats_provenance.get("dataset_sha256") != dataset_sha
                or stats_provenance.get("norm_stats_sha256") != _sha256(stats_path)
                or stats_provenance.get("action_semantics") != ACTION_SEMANTICS):
            raise ValueError("normalization is not bound to this exact derived-state dataset")
        import jax
        devices = jax.local_devices()
        if len(devices) != 1 or jax.default_backend() != "gpu":
            raise RuntimeError("pilot training must use exactly one visible GPU (no CPU fallback)")
        _official("train").main(config)
        candidates = [path for path in checkpoint_root.iterdir()
                      if path.is_dir() and path.name.isdecimal()] if checkpoint_root.is_dir() else []
        if not candidates:
            raise FileNotFoundError("OpenPI training produced no numbered checkpoint")
        checkpoint = max(candidates, key=lambda path: int(path.name)).resolve()
        checkpoint_stats = checkpoint / "assets" / "piper" / "norm_stats.json"
        if not checkpoint_stats.is_file() or _sha256(checkpoint_stats) != _sha256(stats_path):
            raise ValueError("saved checkpoint normalization differs from the audited training stats")
        from server.policy_server.artifact_identity import tree_sha256
        checkpoint_sha = tree_sha256(checkpoint)
        training_provenance = {
            "schema": "piper_derived_state_multi_episode_training_v1",
            "repo_id": repo_id,
            "dataset_path": str(dataset_path),
            "dataset_sha256": dataset_sha,
            "dataset_episodes": audit["episodes"],
            "dataset_frames": audit["frames"],
            "stats_path": str(stats_path),
            "norm_stats_sha256": _sha256(stats_path),
            "action_semantics": ACTION_SEMANTICS,
            "action_target_kind": audit["action_target_kind"],
            "config_name": config.name,
            "experiment_name": config.exp_name,
            "checkpoint_path": str(checkpoint),
            "checkpoint_tree_sha256": checkpoint_sha,
            "steps": steps,
            "save_interval": config.save_interval,
            "keep_period": config.keep_period,
            "base_params_path": str(Path(base_params_path).resolve()),
            "openpi_commit": json.loads((ROOT / "provenance/openpi_git.json").read_text(
                encoding="utf-8"))["head_sha"],
            "training_scope": "all_successful_episodes_train_only_no_holdout",
            "same_data_evaluation_only": True,
            "prediction_only": True,
            "hardware_control_allowed": False,
            "created_utc": datetime.now(timezone.utc).isoformat(),
        }
        provenance_path = checkpoint_root / "training_provenance.json"
        with provenance_path.open("x", encoding="utf-8") as handle:
            json.dump(training_provenance, handle, indent=2, sort_keys=True)
            handle.write("\n")
        return {"phase": phase, "repo_id": repo_id, "dataset_sha256": dataset_sha,
                "checkpoint": str(checkpoint), "checkpoint_tree_sha256": checkpoint_sha,
                "norm_stats_sha256": training_provenance["norm_stats_sha256"],
                "training_provenance": str(provenance_path),
                "training_scope": training_provenance["training_scope"],
                "hardware_control_allowed": False}

    raise NotImplementedError("evaluation requires an existing training-provenance-bound checkpoint")


def evaluate(dataset_path: Path, checkpoint: Path, training_provenance_path: Path, *,
             repo_id: str | None = None, min_episodes: int = 5,
             max_samples_per_episode: int = 5,
             expected_dataset_sha256: str | None = None) -> dict:
    import numpy as np
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from openpi.policies import policy_config
    from server.policy_server.artifact_identity import tree_sha256

    if type(max_samples_per_episode) is not int or not 1 <= max_samples_per_episode <= 20:
        raise ValueError("max_samples_per_episode must be in [1, 20]")
    audit = prepare(dataset_path, repo_id=repo_id, min_episodes=min_episodes,
                    expected_dataset_sha256=expected_dataset_sha256)
    checkpoint = Path(checkpoint).resolve(strict=True)
    training = json.loads(Path(training_provenance_path).read_text(encoding="utf-8"))
    if (training.get("schema") != "piper_derived_state_multi_episode_training_v1"
            or training.get("repo_id") != audit["repo_id"]
            or training.get("dataset_sha256") != audit["dataset_sha256"]
            or training.get("action_semantics") != ACTION_SEMANTICS
            or training.get("prediction_only") is not True
            or training.get("hardware_control_allowed") is not False
            or Path(training.get("checkpoint_path", "")).resolve() != checkpoint
            or tree_sha256(checkpoint) != training.get("checkpoint_tree_sha256")):
        raise ValueError("checkpoint/training provenance does not match this dataset and model")
    stats_path = Path(training.get("stats_path", "")).resolve()
    if (not stats_path.is_file() or _sha256(stats_path) != training.get("norm_stats_sha256")
            or _sha256(checkpoint / "assets/piper/norm_stats.json") != training.get("norm_stats_sha256")):
        raise ValueError("checkpoint normalization is missing or changed")
    os.environ["HF_LEROBOT_HOME"] = audit["hf_lerobot_home"]
    config = build_config(
        repo_id=audit["repo_id"], base_params_path=Path(training["base_params_path"]),
        steps=training["steps"], exp_name=training["experiment_name"],
        save_interval=training.get("save_interval"),
        keep_period=training.get("keep_period"),
    )
    if config.name != training.get("config_name"):
        raise ValueError("training config identity mismatch")
    dataset_root = local_lerobot_dataset_root(audit)
    dataset = LeRobotDataset(audit["repo_id"], root=dataset_root)
    policy = policy_config.create_trained_policy(config, checkpoint)
    source_rows = [json.loads(line) for line in
                   (Path(audit["dataset_path"]) / "provenance/source_map.jsonl").read_text(
                       encoding="utf-8").splitlines()]
    predictions = []
    episode_counts: dict[int, list[int]] = {}
    for index, source in enumerate(source_rows):
        episode_counts.setdefault(source["episode_index"], []).append(index)
    selected_indices = []
    for episode_index, indices in episode_counts.items():
        positions = np.linspace(0, len(indices) - 1,
                                min(max_samples_per_episode, len(indices)), dtype=int)
        selected_indices.extend(indices[int(position)] for position in positions)
    for index in sorted(set(selected_indices)):
        source = source_rows[index]
        row = dataset[index]
        observation = {
            "base_rgb": row["observation.images.image"].numpy(),
            "wrist_rgb": row["observation.images.wrist_image"].numpy(),
            "state": row["observation.state"].numpy(),
            "prompt": row["task"],
        }
        target = np.asarray(row["action"], dtype=np.float32)
        predicted = np.asarray(policy.infer(observation)["actions"], dtype=np.float32)
        if (target.shape != (7,) or predicted.ndim != 2 or predicted.shape[0] < 1
                or predicted.shape[1] < 7 or not np.isfinite(target).all()
                or not np.isfinite(predicted[:, :7]).all()):
            raise ValueError("policy output or derived-state target is not finite 7D")
        error = predicted[0, :7] - target
        predictions.append({
            "dataset_row_index": index,
            "episode_index": source["episode_index"],
            "raw_episode_id": source["raw_episode_id"],
            "predicted_first_action": predicted[0, :7].tolist(),
            "target": target.tolist(),
            "joint_mae_rad_nominal": float(np.mean(np.abs(error[:6]))),
            "gripper_abs_error_m_nominal": float(abs(error[6])),
        })
    return {
        "schema": "piper_derived_state_multi_episode_prediction_v1",
        "status": "SAME_EPISODE_PREDICTION_ONLY",
        "dataset_sha256": audit["dataset_sha256"],
        "checkpoint_tree_sha256": training["checkpoint_tree_sha256"],
        "action_semantics": ACTION_SEMANTICS,
        "evaluated_rows": [row["dataset_row_index"] for row in predictions],
        "episodes": audit["episodes"],
        "mean_joint_mae_rad_nominal": float(np.mean([row["joint_mae_rad_nominal"] for row in predictions])),
        "mean_gripper_abs_error_m_nominal": float(np.mean(
            [row["gripper_abs_error_m_nominal"] for row in predictions])),
        "samples": predictions,
        "held_out_evaluation": False,
        "generalization_claim": False,
        "hardware_success_claim": False,
        "prediction_only": True,
        "can_opened": False,
        "can_data_tx_calls": 0,
        "hardware_control_allowed": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("inspect", "stats", "train", "eval"))
    parser.add_argument("--dataset-path", type=Path, required=True)
    parser.add_argument("--repo-id")
    parser.add_argument("--min-episodes", type=int, default=5)
    parser.add_argument("--expected-dataset-sha256")
    parser.add_argument("--base-params-path", type=Path, default=DEFAULT_BASE_PARAMS)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--max-frames", type=int, default=100_000)
    parser.add_argument("--exp-name", default="")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--training-provenance", type=Path)
    parser.add_argument("--max-samples-per-episode", type=int, default=5)
    parser.add_argument("--allow-derived-state-pilot", action="store_true")
    args = parser.parse_args()
    if args.phase == "inspect":
        report = prepare(args.dataset_path, repo_id=args.repo_id,
                         min_episodes=args.min_episodes,
                         expected_dataset_sha256=args.expected_dataset_sha256)
    elif args.phase == "eval":
        if args.checkpoint is None or args.training_provenance is None:
            parser.error("eval requires --checkpoint and --training-provenance")
        report = evaluate(args.dataset_path, args.checkpoint, args.training_provenance,
                          repo_id=args.repo_id, min_episodes=args.min_episodes,
                          max_samples_per_episode=args.max_samples_per_episode,
                          expected_dataset_sha256=args.expected_dataset_sha256)
    else:
        report = execute(args.dataset_path, phase=args.phase, repo_id=args.repo_id,
                         base_params_path=args.base_params_path,
                         min_episodes=args.min_episodes, steps=args.steps,
                         max_frames=args.max_frames, exp_name=args.exp_name,
                         expected_dataset_sha256=args.expected_dataset_sha256,
                         allow_derived_state_pilot=args.allow_derived_state_pilot)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PermissionError, FileNotFoundError, FileExistsError, ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
