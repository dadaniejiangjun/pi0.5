"""Audit an immutable LeRobot dataset, split by episode, and hash-bind stats.

Read-only for the dataset and checkpoint. Writes an unused audit directory only.
This does not promote data to training eligibility or launch training.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _files(root: Path) -> dict[str, str]:
    root = root.resolve()
    if not root.is_dir():
        raise ValueError(f"missing directory: {root}")
    result = {}
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"symlink is not accepted in audited files: {path}")
        if path.is_file():
            result[path.relative_to(root).as_posix()] = _sha256(path)
    if not result:
        raise ValueError(f"empty directory: {root}")
    return result


def _digest(files: dict[str, str]) -> str:
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def _stats(samples: list[np.ndarray]) -> dict[str, list[float]]:
    if not samples:
        raise ValueError("no training samples")
    values = np.stack(samples).astype(np.float64)
    if values.ndim != 2 or values.shape[1] != 7 or not np.isfinite(values).all():
        raise ValueError("state/actions must be finite 7D vectors")
    return {
        "mean": values.mean(axis=0).tolist(),
        "std": values.std(axis=0).tolist(),
        "q01": np.quantile(values, 0.01, axis=0).tolist(),
        "q99": np.quantile(values, 0.99, axis=0).tolist(),
    }


def build_audit(dataset_path: Path, output: Path, *, seed: int = 0,
                allow_fixture: bool = False) -> dict:
    dataset_path = Path(dataset_path).resolve()
    output = Path(output).resolve()
    if output.exists() or output.is_relative_to(dataset_path) or dataset_path.is_relative_to(output):
        raise ValueError("audit output must be unused and outside dataset")
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    files = _files(dataset_path)
    conversion = json.loads((dataset_path / "provenance/conversion_manifest.json").read_text(encoding="utf-8"))
    source_map_path = dataset_path / "provenance/source_map.jsonl"
    if conversion.get("schema") != "piper_lerobot_conversion_v1" or (
        _sha256(source_map_path) != conversion.get("source_map_sha256")
    ):
        raise ValueError("conversion manifest/source map mismatch")
    if not allow_fixture and (
        conversion.get("synthetic_fixture") is not False
        or conversion.get("source_frame_extraction_verified") is not True
        or conversion.get("command_target_provenance_verified") is not True
        or conversion.get("action_target_source") != "observed_piper_can_command_frames"
    ):
        raise ValueError("real dataset lacks sample-level image and command-target source verification")
    rows = [json.loads(line) for line in source_map_path.read_text(encoding="utf-8").splitlines()]
    sources = conversion.get("source_episodes")
    if not isinstance(sources, list) or not sources or len(rows) != conversion.get("frame_count"):
        raise ValueError("invalid conversion episode/frame count")
    groups: dict[int, list[int]] = {index: [] for index in range(len(sources))}
    current_episode = 0
    for global_index, row in enumerate(rows):
        episode = row.get("derived_episode_index")
        if type(episode) is not int or episode not in groups:
            raise ValueError("invalid episode index in source map")
        if episode != current_episode:
            if episode != current_episode + 1 or not groups[current_episode]:
                raise ValueError("source map episodes must be contiguous and ordered")
            current_episode = episode
        if row.get("derived_frame_index") != len(groups[episode]) or (
            row.get("raw_episode_id") != sources[episode].get("raw_episode_id")
        ):
            raise ValueError("source map order/episode identity mismatch")
        if not allow_fixture and (
            row.get("action_target_source") != "observed_piper_can_command_frames"
            or type(row.get("action_source_timestamp_ns")) is not int
            or not isinstance(row.get("action_source_refs"), dict)
        ):
            raise ValueError("source map lacks per-frame raw command target provenance")
        groups[episode].append(global_index)
    if any(not indices for indices in groups.values()):
        raise ValueError("empty episode in source map")
    identity = [str(item.get("raw_episode_id")) for item in sources]
    if len(set(identity)) != len(identity):
        raise ValueError("duplicate raw episode identity")
    ranked = sorted(groups, key=lambda index: hashlib.sha256(
        f"{seed}:{identity[index]}".encode()).hexdigest())
    val_count = max(1, round(len(ranked) * 0.2)) if len(ranked) > 1 else 0
    val_ids = set(ranked[:val_count])
    train_ids = set(groups) - val_ids
    train_indices = [index for episode in sorted(train_ids) for index in groups[episode]]
    dataset = LeRobotDataset(conversion["repo_id"], root=dataset_path)
    if len(dataset) != len(rows) or dataset.meta.total_episodes != len(sources):
        raise ValueError("LeRobot rows/episodes differ from source map")
    values: dict[str, list[np.ndarray]] = {"state": [], "actions": []}
    for index in train_indices:
        sample = dataset[index]
        for key, feature in (("state", "observation.state"), ("actions", "action")):
            values[key].append(np.asarray(sample[feature], dtype=np.float64))
    dataset_hash = _digest(files)
    stats = {
        "schema": "piper_train_split_norm_v1",
        "dataset_sha256": dataset_hash,
        "train_episode_ids": [identity[index] for index in sorted(train_ids)],
        "norm_stats": {key: _stats(samples) for key, samples in values.items()},
    }
    output.mkdir(parents=True)
    stats_path = output / "norm_stats.json"
    stats_path.write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema": "piper_dataset_training_audit_v1",
        "status": "TRAIN_ONLY_NO_VALIDATION" if not val_ids else "SPLIT_AND_STATS_VERIFIED",
        "dataset_path": str(dataset_path),
        "dataset_files": files,
        "dataset_sha256": dataset_hash,
        "conversion_manifest_sha256": files["provenance/conversion_manifest.json"],
        "seed": seed,
        "train_episode_ids": stats["train_episode_ids"],
        "validation_episode_ids": [identity[index] for index in sorted(val_ids)],
        "train_frame_count": len(train_indices),
        "validation_frame_count": sum(len(groups[index]) for index in val_ids),
        "norm_stats_sha256": _sha256(stats_path),
        "checkpoint": None,
        "training_eligible": False,
    }
    (output / "audit_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                                                 encoding="utf-8")
    return manifest


def bind_checkpoint(output: Path, checkpoint: Path) -> dict:
    """Bind a checkpoint only when its own provenance names this data/stats."""
    output = Path(output).resolve()
    checkpoint = Path(checkpoint).resolve()
    verify_audit(output)
    binding_path = output / "checkpoint_binding.json"
    if binding_path.exists():
        raise FileExistsError("checkpoint binding already exists")
    manifest = json.loads((output / "audit_manifest.json").read_text(encoding="utf-8"))
    record = json.loads((checkpoint / "training_provenance.json").read_text(encoding="utf-8"))
    if (record.get("dataset_sha256") != manifest["dataset_sha256"]
            or record.get("norm_stats_sha256") != manifest["norm_stats_sha256"]):
        raise ValueError("checkpoint provenance names a different dataset or statistics file")
    files = _files(checkpoint)
    binding = {
        "schema": "piper_checkpoint_binding_v1",
        "path": str(checkpoint),
        "files": files,
        "sha256": _digest(files),
        "dataset_sha256": manifest["dataset_sha256"],
        "norm_stats_sha256": manifest["norm_stats_sha256"],
    }
    binding_path.write_text(json.dumps(binding, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return binding


def verify_audit(output: Path) -> dict:
    output = Path(output).resolve()
    manifest = json.loads((output / "audit_manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != "piper_dataset_training_audit_v1":
        raise ValueError("invalid audit schema")
    current = _files(Path(manifest["dataset_path"]))
    if current != manifest.get("dataset_files") or _digest(current) != manifest.get("dataset_sha256"):
        raise ValueError("dataset changed since audit")
    stats_path = output / "norm_stats.json"
    if _sha256(stats_path) != manifest.get("norm_stats_sha256"):
        raise ValueError("normalization statistics changed since audit")
    stats = json.loads(stats_path.read_text(encoding="utf-8"))
    if stats.get("dataset_sha256") != manifest["dataset_sha256"] or (
        stats.get("train_episode_ids") != manifest.get("train_episode_ids")
    ):
        raise ValueError("statistics are bound to a different dataset/split")
    binding_path = output / "checkpoint_binding.json"
    binding = json.loads(binding_path.read_text(encoding="utf-8")) if binding_path.is_file() else None
    if binding is not None:
        current_checkpoint = _files(Path(binding["path"]))
        if (binding.get("schema") != "piper_checkpoint_binding_v1"
                or current_checkpoint != binding.get("files")
                or _digest(current_checkpoint) != binding.get("sha256")
                or binding.get("dataset_sha256") != manifest["dataset_sha256"]
                or binding.get("norm_stats_sha256") != manifest["norm_stats_sha256"]):
            raise ValueError("checkpoint changed since audit or binding does not match")
        record = json.loads((Path(binding["path"]) / "training_provenance.json").read_text(encoding="utf-8"))
        if (record.get("dataset_sha256") != manifest["dataset_sha256"]
                or record.get("norm_stats_sha256") != manifest["norm_stats_sha256"]
        ):
            raise ValueError("checkpoint provenance does not match audit")
    return {"status": "HASH_BINDING_PASS", "dataset_sha256": manifest["dataset_sha256"],
            "norm_stats_sha256": manifest["norm_stats_sha256"],
            "checkpoint_sha256": binding["sha256"] if binding else None,
            "training_eligible": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("build", "verify", "bind"))
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--checkpoint", type=Path)
    args = parser.parse_args()
    if args.phase == "build":
        if args.dataset is None:
            parser.error("--dataset is required for build")
        report = build_audit(args.dataset, args.output, seed=args.seed)
    elif args.phase == "bind":
        if args.checkpoint is None:
            parser.error("--checkpoint is required for bind")
        report = bind_checkpoint(args.output, args.checkpoint)
    else:
        report = verify_audit(args.output)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
