"""Mode-gated offline OpenPI training; never connects to a robot."""

from __future__ import annotations

import argparse
import ast
import dataclasses
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys

from server.training.check_piper_readiness import MODES, inspect
from server.training.piper_openpi_config import PiperOpenPIDataConfig, make_unregistered_config
from server.policy_server.artifact_identity import tree_sha256


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _dataset_tree_identity(path: Path) -> tuple[dict[str, str], str]:
    root = Path(path).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"missing LeRobot dataset: {root}")
    files: dict[str, str] = {}
    for item in sorted(root.rglob("*")):
        if item.is_symlink():
            raise ValueError(f"dataset symlinks are not accepted: {item}")
        if item.is_file():
            files[item.relative_to(root).as_posix()] = _sha256_file(item)
    if not files:
        raise ValueError("LeRobot dataset is empty")
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return files, hashlib.sha256(encoded).hexdigest()


def build_config(root: Path, *, repo_id: str, base_params_path: str,
                 mode: str, steps: int):
    """Rebuild the exact bounded PiPER training config for train/eval/serve."""
    root = Path(root).resolve()
    candidate = make_unregistered_config(
        repo_id=repo_id,
        base_params_path=base_params_path,
        assets_root=str(root / "assets/pi05_piper_v1"),
    )
    return dataclasses.replace(
        candidate,
        name=f"pi05_piper_{mode.replace('-', '_')}",
        exp_name=f"{mode.replace('-', '_')}_{steps}_steps",
        # Use TrainConfig.assets_dirs as the single source of truth for stats.
        # An explicit per-factory assets_dir would otherwise keep pointing at
        # the placeholder pi05_piper_v1 directory while stats are written under
        # the mode-specific config name.
        data=dataclasses.replace(
            candidate.data,
            assets=dataclasses.replace(candidate.data.assets, assets_dir=None),
        ),
        assets_base_dir=str(root / "assets"),
        checkpoint_base_dir=str(root / "checkpoints"),
        num_train_steps=steps,
        batch_size=1,
        num_workers=0,
        save_interval=steps,
        log_interval=1,
        wandb_enabled=False,
    )


def prepare(root: Path, *, repo_id: str, base_params_path: str,
            mode: str = "formal", dataset_path: Path | None = None,
            acceptance_path: Path | None = None) -> dict:
    root = Path(root).resolve()
    candidate = make_unregistered_config(
        repo_id=repo_id, base_params_path=base_params_path,
        assets_root=str(root / "assets/pi05_piper_v1"),
    )
    official_script = root / "third_party/openpi/scripts/train.py"
    official_main_present = False
    if official_script.is_file():
        syntax = ast.parse(official_script.read_text(encoding="utf-8"))
        official_main_present = any(isinstance(node, ast.FunctionDef) and node.name == "main" for node in syntax.body)
    readiness = inspect(root, mode, dataset_path=dataset_path, acceptance_path=acceptance_path)
    return {
        "status": "READY_FOR_OFFLINE_EXECUTION" if readiness["status"] == "READY_FOR_SEPARATE_REVIEW"
        and official_main_present else "TRAINING_LOCKED",
        "mode": mode,
        "readiness": readiness,
        "candidate_config": {
            "name": candidate.name,
            "repo_id": candidate.data.repo_id,
            "pi05": candidate.model.pi05,
            "model_action_dim": candidate.model.action_dim,
            "action_horizon": candidate.model.action_horizon,
            "piper_data_factory": isinstance(candidate.data, PiperOpenPIDataConfig),
        },
        "official_train_main_present": official_main_present,
        "training_started": False,
        "hardware_control_allowed": False,
    }


def _official_script(root: Path, name: str):
    script = root / "third_party/openpi/scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"piper_official_{name}", script)
    if spec is None or spec.loader is None:
        raise FileNotFoundError(script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def execute_training(root: Path | None = None, *, mode: str = "formal", phase: str = "train",
                     repo_id: str = "local/piper_v1", base_params_path: str = "UNSET",
                     dataset_path: Path | None = None, acceptance_path: Path | None = None,
                     steps: int = 2, max_frames: int = 512,
                     allow_offline_training: bool = False) -> dict:
    if root is None or not allow_offline_training:
        raise PermissionError("offline training locked: explicit root and --allow-offline-training required")
    if mode not in MODES or phase not in ("stats", "train"):
        raise ValueError("invalid mode or phase")
    if mode == "public-data":
        raise PermissionError(
            "public-data uses server.training.public_piper_smoke only; "
            "the local PiPER training config must not consume degree/binary data"
        )
    if not 1 <= steps <= (10 if mode != "formal" else 100_000) or max_frames < 1:
        raise ValueError("invalid training bounds")
    root = Path(root).resolve()
    report = prepare(root, repo_id=repo_id, base_params_path=base_params_path,
                     mode=mode, dataset_path=dataset_path, acceptance_path=acceptance_path)
    if report["status"] != "READY_FOR_OFFLINE_EXECUTION":
        raise PermissionError("offline training locked: mode-specific data/readiness checks failed")
    dataset = Path(report["readiness"]["dataset_path"]).resolve()
    # OpenPI resolves HF_LEROBOT_HOME/repo_id. Refuse any ambiguous dataset binding.
    data_home = dataset
    for _ in Path(repo_id).parts:
        data_home = data_home.parent
    if (data_home / repo_id).resolve() != dataset:
        raise ValueError("dataset_path must equal HF_LEROBOT_HOME/repo_id")
    if base_params_path == "UNSET" or not Path(base_params_path).is_dir():
        raise FileNotFoundError("pi05_base params directory is required")
    os.environ["HF_LEROBOT_HOME"] = str(data_home)
    from openpi.training import config as training_config
    config = build_config(root, repo_id=repo_id, base_params_path=base_params_path,
                          mode=mode, steps=steps)
    if config.checkpoint_dir.exists():
        raise FileExistsError(f"checkpoint already exists: {config.checkpoint_dir}")
    training_config._CONFIGS_DICT[config.name] = config
    stats_file = config.assets_dirs / "piper" / "norm_stats.json"
    stats_provenance_file = config.assets_dirs / "piper" / "norm_stats.provenance.json"
    dataset_files, dataset_sha256 = _dataset_tree_identity(dataset)
    if phase == "stats":
        if stats_file.exists() or stats_provenance_file.exists():
            raise FileExistsError("PiPER stats already exist; refusing to overwrite or reuse stale stats")
        stats_file.parent.mkdir(parents=True, exist_ok=True)
        _official_script(root, "compute_norm_stats").main(config.name, max_frames=max_frames)
        if not stats_file.is_file():
            raise FileNotFoundError(f"OpenPI did not write expected statistics: {stats_file}")
        stats_provenance = {
            "schema": "piper_offline_norm_stats_provenance_v1",
            "repo_id": repo_id,
            "dataset_path": str(dataset),
            "dataset_sha256": dataset_sha256,
            "dataset_file_count": len(dataset_files),
            "norm_stats_path": str(stats_file),
            "norm_stats_sha256": _sha256_file(stats_file),
            "action_semantics": "absolute_joint_position_rad_gripper_meter",
            "source_split": "all_episodes_are_train_only_for_single_episode_tiny_fit"
            if mode == "local-tiny-fit" else "training_data_configured_split",
            "hardware_control_allowed": False,
        }
        stats_provenance_file.write_text(
            json.dumps(stats_provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return {"mode": mode, "phase": phase, "dataset": str(dataset),
                "dataset_sha256": dataset_sha256,
                "norm_stats_sha256": stats_provenance["norm_stats_sha256"],
                "training_started": False, "hardware_control_allowed": False}

    stats_file = config.assets_dirs / "piper" / "norm_stats.json"
    if not stats_file.is_file() or not stats_provenance_file.is_file():
        raise FileNotFoundError(f"compute and provenance-bind PiPER stats first: {stats_file}")
    stats_provenance = json.loads(stats_provenance_file.read_text(encoding="utf-8"))
    if (stats_provenance.get("schema") != "piper_offline_norm_stats_provenance_v1"
            or stats_provenance.get("repo_id") != repo_id
            or stats_provenance.get("dataset_sha256") != dataset_sha256
            or stats_provenance.get("norm_stats_sha256") != _sha256_file(stats_file)
            or stats_provenance.get("action_semantics") != "absolute_joint_position_rad_gripper_meter"):
        raise ValueError("PiPER norm stats are not bound to this exact dataset/action contract")
    if base_params_path == "UNSET" or not Path(base_params_path).is_dir():
        raise FileNotFoundError("pi05_base params directory is required")
    _official_script(root, "train").main(config)
    if not config.checkpoint_dir.is_dir():
        raise FileNotFoundError(f"OpenPI training returned without its checkpoint directory: {config.checkpoint_dir}")
    checkpoints = [path for path in config.checkpoint_dir.iterdir()
                   if path.is_dir() and path.name.isdecimal()]
    if not checkpoints:
        raise FileNotFoundError(f"OpenPI produced no numbered checkpoint under {config.checkpoint_dir}")
    checkpoint_path = max(checkpoints, key=lambda path: int(path.name)).resolve()
    checkpoint_sha256 = tree_sha256(checkpoint_path)
    checkpoint_stats = checkpoint_path / "assets" / "piper" / "norm_stats.json"
    if (not checkpoint_stats.is_file()
            or _sha256_file(checkpoint_stats) != _sha256_file(stats_file)):
        raise ValueError("saved checkpoint did not preserve the exact PiPER normalization stats")
    training_provenance_path = config.checkpoint_dir / "training_provenance.json"
    if training_provenance_path.exists():
        raise FileExistsError(f"training provenance already exists: {training_provenance_path}")
    openpi_revision = json.loads((root / "provenance/openpi_git.json").read_text(encoding="utf-8"))
    training_provenance = {
        "schema": "piper_offline_training_provenance_v1",
        "mode": mode,
        "phase": "train",
        "repo_id": repo_id,
        "dataset_path": str(dataset),
        "dataset_sha256": dataset_sha256,
        "dataset_file_count": len(dataset_files),
        "norm_stats_path": str(stats_file),
        "norm_stats_sha256": _sha256_file(stats_file),
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "config_name": config.name,
        "experiment_name": config.exp_name,
        "checkpoint_path": str(checkpoint_path),
        "checkpoint_sha256": checkpoint_sha256,
        "checkpoint_identity_scope": "selected_numbered_openpi_checkpoint_tree",
        "steps": steps,
        "base_params_path": str(Path(base_params_path).resolve()),
        "openpi_commit": openpi_revision["head_sha"],
        "python_version": platform.python_version(),
        "single_episode_validation": "not_available_train_only"
        if mode == "local-tiny-fit" else "mode_specific",
        "prediction_only": True,
        "hardware_control_allowed": False,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    training_provenance_path.write_text(
        json.dumps(training_provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"mode": mode, "phase": phase, "dataset": str(dataset),
            "dataset_sha256": dataset_sha256,
            "norm_stats_sha256": training_provenance["norm_stats_sha256"],
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": checkpoint_sha256,
            "training_provenance": str(training_provenance_path),
            "hardware_control_allowed": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=MODES, default="formal")
    parser.add_argument("--phase", choices=("stats", "train"), default="train")
    parser.add_argument("--repo-id", default="local/piper_v1")
    parser.add_argument("--base-params-path", default="UNSET")
    parser.add_argument("--dataset-path", type=Path)
    parser.add_argument("--acceptance-path", type=Path)
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument("--max-frames", type=int, default=512)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--allow-offline-training", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    report = prepare(root, repo_id=args.repo_id, base_params_path=args.base_params_path,
                     mode=args.mode, dataset_path=args.dataset_path, acceptance_path=args.acceptance_path)
    print(json.dumps(report, indent=2))
    if not args.execute:
        return 0 if report["status"] == "READY_FOR_OFFLINE_EXECUTION" else 1
    try:
        result = execute_training(root, mode=args.mode, phase=args.phase, repo_id=args.repo_id,
                                  base_params_path=args.base_params_path, dataset_path=args.dataset_path,
                                  acceptance_path=args.acceptance_path, steps=args.steps,
                                  max_frames=args.max_frames,
                                  allow_offline_training=args.allow_offline_training)
    except (PermissionError, FileNotFoundError, FileExistsError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
