"""Run the offline PiPER demo-to-Pi0.5 pilot without touching robot hardware.

The default ``run`` phase selects operator-labeled successful raw episodes,
verifies their source files, aligns both cameras and feedback at 10 fps,
creates explicitly derived next-state targets, converts and reads the batch
back through the pinned LeRobot runtime, computes train-only statistics,
trains a bounded single-GPU pilot, and evaluates predictions on the same
episodes. It never opens a camera or CAN adapter and never authorizes control.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Sequence

from laptop.recorder.align_raw import align
from laptop.recorder.label_episode import verify_label
from laptop.recorder.next_state_proxy import SEMANTICS, build as build_proxy
from server.dataset_tools.convert_next_state_proxy import convert_many
from server.training import proxy_multi_episode_pilot as pilot
from server.training.proxy_multi_episode_contract import (
    ACTION_TARGET_KIND,
    inspect_proxy_multi_episode,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RAW_ROOT = ROOT / "datasets" / "raw"
DEFAULT_OUTCOME_ROOT = ROOT / "datasets" / "manifests" / "outcomes"
DEFAULT_DERIVED_ROOT = ROOT / "datasets" / "manifests" / "derived_state_pilot"
DEFAULT_DATASET_HOME = ROOT / "datasets" / "incoming"
# Match the existing local-tiny-fit contract: one reviewed success is enough to
# exercise the full offline data-to-prediction path. More episodes improve the
# experiment, but are not a prerequisite for the first fast end-to-end run.
MIN_SUCCESSFUL_EPISODES = 1
MAX_TRAIN_STEPS = 100
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{2,47}$")
REPO_ID_RE = re.compile(r"^[A-Za-z0-9_-]+/[A-Za-z0-9_-]+$")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _utc_run_id() -> str:
    return "piper_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _repo_id(run_id: str, requested: str | None) -> str:
    repo_id = requested or f"local/{run_id}"
    if REPO_ID_RE.fullmatch(repo_id) is None:
        raise ValueError("repo_id must contain a safe namespace/name pair")
    return repo_id


def select_successful_episodes(raw_root: Path, outcome_root: Path,
                               *, min_episodes: int = MIN_SUCCESSFUL_EPISODES,
                               episode_ids: Sequence[str] | None = None) -> dict[str, Any]:
    """Select only checksum-bound, explicitly successful operator episodes."""
    if type(min_episodes) is not int or min_episodes < MIN_SUCCESSFUL_EPISODES:
        raise ValueError("the fast Pi0.5 tiny-fit requires at least one successful episode")
    requested_ids = None
    if episode_ids is not None:
        if (not isinstance(episode_ids, (list, tuple)) or not episode_ids
                or any(not isinstance(item, str) or not item or Path(item).name != item
                       for item in episode_ids)):
            raise ValueError("episode_ids must be a nonempty list of directory names")
        if len(set(episode_ids)) != len(episode_ids):
            raise ValueError("episode_ids must not contain duplicates")
        requested_ids = set(episode_ids)
    raw_root = Path(raw_root).resolve(strict=True)
    outcome_root = Path(outcome_root).resolve()
    raw_ids = sorted(
        path.name for path in raw_root.iterdir()
        if path.is_dir() and (path / "manifest.json").is_file()
    )
    if requested_ids is not None:
        missing_ids = sorted(requested_ids - set(raw_ids))
        if missing_ids:
            raise ValueError(f"requested raw episode IDs are missing: {missing_ids}")
        candidate_raw_ids = requested_ids
    else:
        candidate_raw_ids = set(raw_ids)
    labels = {}
    failed_ids = []
    for label_path in sorted(outcome_root.glob("*.json")):
        episode_id = label_path.stem
        if episode_id not in candidate_raw_ids:
            continue
        raw_episode = raw_root / episode_id
        if not raw_episode.is_dir():
            raise ValueError(f"outcome label has no matching raw episode: {episode_id}")
        verified = verify_label(raw_episode, label_path)
        if verified["success"]:
            labels[episode_id] = {"raw": raw_episode, "outcome": label_path}
        else:
            failed_ids.append(episode_id)
    unlabeled_ids = sorted(candidate_raw_ids - (set(labels) | set(failed_ids)))
    if len(labels) < min_episodes:
        raise ValueError(
            f"need at least {min_episodes} checksum-bound successful episodes; "
            f"found {len(labels)} (failed={len(failed_ids)}, unlabeled={len(unlabeled_ids)})"
        )
    return {
        "successful": [{"episode_id": episode_id, **labels[episode_id]}
                       for episode_id in sorted(labels)],
        "failed_episode_ids": failed_ids,
        "unlabeled_episode_ids": unlabeled_ids,
    }


def prepare_dataset(*, raw_root: Path = DEFAULT_RAW_ROOT,
                    outcome_root: Path = DEFAULT_OUTCOME_ROOT,
                    derived_root: Path = DEFAULT_DERIVED_ROOT,
                    dataset_home: Path = DEFAULT_DATASET_HOME,
                    run_id: str | None = None, repo_id: str | None = None,
                    min_episodes: int = MIN_SUCCESSFUL_EPISODES,
                    max_skew_ms: float = 50.0,
                    episode_ids: Sequence[str] | None = None) -> dict[str, Any]:
    """Create one new hash-audited LeRobot dataset from labeled successes."""
    run_id = run_id or _utc_run_id()
    if RUN_ID_RE.fullmatch(run_id) is None:
        raise ValueError("run_id must be a safe 3-48 character directory name")
    repo_id = _repo_id(run_id, repo_id)
    raw_root = Path(raw_root).resolve(strict=True)
    outcome_root = Path(outcome_root).resolve(strict=True)
    derived_root = Path(derived_root).resolve()
    dataset_home = Path(dataset_home).resolve()
    if (derived_root.is_relative_to(raw_root) or raw_root.is_relative_to(derived_root)
            or derived_root.is_relative_to(outcome_root)
            or outcome_root.is_relative_to(derived_root)):
        raise ValueError("derived artifacts must remain separate from immutable raw/outcome data")

    selection = select_successful_episodes(
        raw_root, outcome_root, min_episodes=min_episodes,
        episode_ids=episode_ids,
    )
    run_root = derived_root / run_id
    if run_root.exists():
        raise FileExistsError(f"refusing to overwrite derived run: {run_root}")
    namespace, name = repo_id.split("/")
    dataset_path = dataset_home / namespace / name
    if dataset_path.exists():
        raise FileExistsError(f"refusing to overwrite LeRobot dataset: {dataset_path}")

    run_root.mkdir(parents=True)
    pairs = []
    for item in selection["successful"]:
        episode_id = item["episode_id"]
        raw_episode = item["raw"]
        preview = run_root / "previews" / episode_id
        proxy = run_root / "targets" / episode_id
        align(raw_episode, preview, fps=10, max_skew_ms=max_skew_ms)
        build_proxy(preview, raw_episode, item["outcome"], proxy)
        pairs.append((proxy, preview))

    os.environ["HF_LEROBOT_HOME"] = str(dataset_home)
    conversion = convert_many(pairs, dataset_path, repo_id)
    audit = inspect_proxy_multi_episode(
        dataset_path, min_episodes=min_episodes,
    )
    report = {
        "schema": "piper_post_capture_dataset_pipeline_v1",
        "status": "DATASET_READY_FOR_OFFLINE_DERIVED_STATE_PILOT",
        "run_id": run_id,
        "dataset_path": str(dataset_path),
        "dataset_home": str(dataset_home),
        "repo_id": repo_id,
        "dataset_sha256": audit["dataset_sha256"],
        "conversion": conversion,
        "audit": audit,
        "successful_episode_ids": [item["episode_id"] for item in selection["successful"]],
        "failed_episode_ids": selection["failed_episode_ids"],
        "unlabeled_episode_ids": selection["unlabeled_episode_ids"],
        "action_semantics": SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "action_target_description": (
            "observation at t paired with the next measured 6-joint plus gripper state at t+0.1s; "
            "derived offline position target, not an OEM/CAN command"
        ),
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }
    batch_manifest = run_root / "batch_manifest.json"
    with batch_manifest.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    report["batch_manifest"] = str(batch_manifest)
    return report


def train_evaluate_dataset(*, dataset_path: Path, base_params_path: Path = pilot.DEFAULT_BASE_PARAMS,
                           min_episodes: int = MIN_SUCCESSFUL_EPISODES,
                           steps: int = 20, max_frames: int = 100_000,
                           gpu_index: str = "0", exp_name: str = "",
                           expected_dataset_sha256: str | None = None) -> dict[str, Any]:
    """Run train-only stats, bounded one-GPU fitting, and same-data prediction eval."""
    if type(steps) is not int or not 1 <= steps <= MAX_TRAIN_STEPS:
        raise ValueError(f"pilot steps must be in [1, {MAX_TRAIN_STEPS}]")
    if type(max_frames) is not int or max_frames < 1:
        raise ValueError("max_frames must be a positive integer")
    if re.fullmatch(r"\d+", gpu_index) is None:
        raise ValueError("gpu_index must identify exactly one numeric server GPU")

    dataset_path = Path(dataset_path).resolve(strict=True)
    audit = pilot.prepare(
        dataset_path, min_episodes=min_episodes,
        expected_dataset_sha256=expected_dataset_sha256,
    )
    dataset_home = Path(audit["hf_lerobot_home"])
    os.environ["HF_LEROBOT_HOME"] = str(dataset_home)
    # JAX is imported lazily by the pilot; expose one selected accelerator only.
    os.environ["CUDA_VISIBLE_DEVICES"] = gpu_index
    base_params_path = Path(base_params_path).resolve(strict=True)
    config = pilot.build_config(
        repo_id=audit["repo_id"], base_params_path=base_params_path,
        steps=steps, exp_name=exp_name,
    )
    stats_path = pilot.norm_stats_path(config, audit["repo_id"])
    stats_provenance_path = stats_path.with_name("norm_stats.provenance.json")
    if stats_path.exists() != stats_provenance_path.exists():
        raise FileExistsError("partial normalization artifacts exist; preserve and audit them before retrying")
    if stats_path.is_file():
        stats_provenance = json.loads(stats_provenance_path.read_text(encoding="utf-8"))
        if (stats_provenance.get("schema") != "piper_derived_state_pilot_norm_stats_v1"
                or stats_provenance.get("repo_id") != audit["repo_id"]
                or stats_provenance.get("dataset_sha256") != audit["dataset_sha256"]
                or stats_provenance.get("norm_stats_sha256") != _sha256(stats_path)
                or stats_provenance.get("action_semantics") != SEMANTICS):
            raise ValueError("existing normalization is not bound to this exact dataset")
        stats_result = {"phase": "stats", "reused": True,
                        "norm_stats_sha256": _sha256(stats_path)}
    else:
        stats_result = pilot.execute(
            dataset_path, phase="stats", repo_id=audit["repo_id"],
            base_params_path=base_params_path, min_episodes=min_episodes,
            steps=steps, max_frames=max_frames, exp_name=exp_name,
            expected_dataset_sha256=audit["dataset_sha256"],
            allow_derived_state_pilot=True,
        )

    train_result = pilot.execute(
        dataset_path, phase="train", repo_id=audit["repo_id"],
        base_params_path=base_params_path, min_episodes=min_episodes,
        steps=steps, max_frames=max_frames, exp_name=exp_name,
        expected_dataset_sha256=audit["dataset_sha256"],
        allow_derived_state_pilot=True,
    )
    checkpoint = Path(train_result["checkpoint"]).resolve(strict=True)
    training_provenance = Path(train_result["training_provenance"]).resolve(strict=True)
    training_record = json.loads(training_provenance.read_text(encoding="utf-8"))
    evaluation = pilot.evaluate(
        dataset_path, checkpoint, training_provenance,
        repo_id=audit["repo_id"], min_episodes=min_episodes,
        expected_dataset_sha256=audit["dataset_sha256"],
    )
    identity_lock = {
        "schema": "piper_derived_state_multi_shadow_identity_lock_v1",
        "artifact_identity_schema": "piper_derived_state_multi_shadow_identity_v1",
        "config_schema": "pi05_derived_state_multi_episode_shadow_v1",
        "checkpoint": str(checkpoint),
        "checkpoint_tree_sha256": train_result["checkpoint_tree_sha256"],
        "dataset": audit["repo_id"],
        "dataset_sha256": audit["dataset_sha256"],
        "normalization_identity": f"train_only_stats:{audit['repo_id']}",
        "normalization_sha256": train_result["norm_stats_sha256"],
        "action_semantics": SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "action_horizon": 16,
        "action_dimension": 7,
        "first_target_dt_s": 0.1,
        "hardware_control_allowed": False,
        "prediction_only": True,
        "openpi_commit": training_record["openpi_commit"],
        "training_provenance_sha256": _sha256(training_provenance),
    }
    identity_lock_path = ROOT / "validation" / (
        f"derived_state_shadow_identity_{audit['repo_id'].split('/')[-1]}.json"
    )
    identity_lock_path.parent.mkdir(parents=True, exist_ok=True)
    with identity_lock_path.open("x", encoding="utf-8") as stream:
        json.dump(identity_lock, stream, indent=2, sort_keys=True)
        stream.write("\n")
    serve_args = [
        sys.executable, "-m", "server.serve_derived_state_multi_shadow",
        "--dataset-path", str(dataset_path), "--checkpoint", str(checkpoint),
        "--training-provenance", str(training_provenance),
        "--host", "127.0.0.1", "--port", "8020",
    ]
    return {
        "schema": "piper_post_capture_training_pipeline_v1",
        "status": "TRAINED_AND_EVALUATED_PREDICTION_ONLY",
        "dataset_path": str(dataset_path),
        "dataset_sha256": audit["dataset_sha256"],
        "episodes": audit["episodes"],
        "frames": audit["frames"],
        "steps": steps,
        "single_gpu_visible_selector": gpu_index,
        "stats": stats_result,
        "training": train_result,
        "evaluation": evaluation,
        "shadow_service_command": serve_args,
        "client_identity_lock_path": str(identity_lock_path),
        "client_identity_lock": identity_lock,
        "deployment_mode": "prediction_only_loopback_shadow",
        "action_semantics": SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }


def run_pipeline(*, raw_root: Path = DEFAULT_RAW_ROOT,
                 outcome_root: Path = DEFAULT_OUTCOME_ROOT,
                 derived_root: Path = DEFAULT_DERIVED_ROOT,
                 dataset_home: Path = DEFAULT_DATASET_HOME,
                 run_id: str | None = None, repo_id: str | None = None,
                 min_episodes: int = MIN_SUCCESSFUL_EPISODES,
                 max_skew_ms: float = 50.0,
                 base_params_path: Path = pilot.DEFAULT_BASE_PARAMS,
                 steps: int = 20, max_frames: int = 100_000,
                 gpu_index: str = "0") -> dict[str, Any]:
    run_id = run_id or _utc_run_id()
    prepared = prepare_dataset(
        raw_root=raw_root, outcome_root=outcome_root,
        derived_root=derived_root, dataset_home=dataset_home,
        run_id=run_id, repo_id=repo_id,
        min_episodes=min_episodes, max_skew_ms=max_skew_ms,
    )
    trained = train_evaluate_dataset(
        dataset_path=Path(prepared["dataset_path"]),
        base_params_path=base_params_path, min_episodes=min_episodes,
        steps=steps, max_frames=max_frames, gpu_index=gpu_index,
        expected_dataset_sha256=prepared["dataset_sha256"],
    )
    return {
        "schema": "piper_post_capture_end_to_end_pipeline_v1",
        "status": "TRAINED_AND_EVALUATED_PREDICTION_ONLY",
        "run_id": run_id,
        "dataset": prepared,
        "training": trained,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("prepare", "train", "run"), nargs="?", default="run")
    parser.add_argument("--raw-root", type=Path, default=DEFAULT_RAW_ROOT)
    parser.add_argument("--outcome-root", type=Path, default=DEFAULT_OUTCOME_ROOT)
    parser.add_argument("--derived-root", type=Path, default=DEFAULT_DERIVED_ROOT)
    parser.add_argument("--dataset-home", type=Path, default=DEFAULT_DATASET_HOME)
    parser.add_argument("--dataset-path", type=Path)
    parser.add_argument("--repo-id")
    parser.add_argument("--run-id")
    parser.add_argument("--min-episodes", type=int, default=MIN_SUCCESSFUL_EPISODES)
    parser.add_argument("--max-skew-ms", type=float, default=50.0)
    parser.add_argument("--base-params-path", type=Path, default=pilot.DEFAULT_BASE_PARAMS)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--max-frames", type=int, default=100_000)
    parser.add_argument("--gpu-index", default="0")
    parser.add_argument("--expected-dataset-sha256")
    parser.add_argument("--report-path", type=Path)
    args = parser.parse_args()

    if args.phase == "prepare":
        result = prepare_dataset(
            raw_root=args.raw_root, outcome_root=args.outcome_root,
            derived_root=args.derived_root, dataset_home=args.dataset_home,
            run_id=args.run_id, repo_id=args.repo_id,
            min_episodes=args.min_episodes, max_skew_ms=args.max_skew_ms,
        )
    elif args.phase == "train":
        if args.dataset_path is None:
            parser.error("train requires --dataset-path")
        result = train_evaluate_dataset(
            dataset_path=args.dataset_path, base_params_path=args.base_params_path,
            min_episodes=args.min_episodes, steps=args.steps,
            max_frames=args.max_frames, gpu_index=args.gpu_index,
            expected_dataset_sha256=args.expected_dataset_sha256,
        )
    else:
        result = run_pipeline(
            raw_root=args.raw_root, outcome_root=args.outcome_root,
            derived_root=args.derived_root, dataset_home=args.dataset_home,
            run_id=args.run_id, repo_id=args.repo_id,
            min_episodes=args.min_episodes, max_skew_ms=args.max_skew_ms,
            base_params_path=args.base_params_path, steps=args.steps,
            max_frames=args.max_frames, gpu_index=args.gpu_index,
        )
    report_path = args.report_path
    if report_path is None and args.phase in ("train", "run"):
        identity = result.get("run_id")
        if not identity and args.dataset_path is not None:
            safe_dataset_name = re.sub(r"[^A-Za-z0-9_-]", "_", args.dataset_path.name)
            identity = f"{safe_dataset_name}_{_utc_run_id()}"
        report_path = ROOT / "validation" / f"post_capture_{identity}.json"
    if report_path is not None:
        path = Path(report_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
        result["report_path"] = str(path.resolve())
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, PermissionError, ValueError, RuntimeError) as exc:
        print(f"post-capture pipeline stopped: {exc}", file=sys.stderr)
        raise SystemExit(2)
