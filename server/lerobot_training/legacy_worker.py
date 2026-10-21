"""Run the pinned legacy LeRobot converter and inspector in its old runtime.

This subprocess performs data preparation only. It never invokes the old
training phase. The emitted handoff binds the actual strict inspect result to
the converter output tree and the normalized source-adapter manifest.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_new(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _runtime() -> dict:
    return {"python_version": sys.version.split()[0],
            "lerobot_version": importlib.metadata.version("lerobot"),
            "datasets_version": importlib.metadata.version("datasets")}


def _audit(dataset_path: Path, expected_sha: str | None = None) -> dict:
    from server.training.proxy_multi_episode_contract import (
        dataset_tree_sha256, inspect_proxy_multi_episode,
    )
    fresh = inspect_proxy_multi_episode(dataset_path, min_episodes=5)
    tree_sha = dataset_tree_sha256(dataset_path)
    if expected_sha is not None and expected_sha != tree_sha:
        raise ValueError("legacy dataset tree hash differs from handoff report")
    return {"strict_inspect": fresh, "dataset_tree_sha256": tree_sha}


def _validate_batch_wrapper(batch: dict, dataset: Path, strict: dict, *, run_id: str,
                            repo_id: str, expected_episode_ids: list[str]) -> None:
    if (batch.get("schema") != "piper_post_capture_dataset_pipeline_v1"
            or batch.get("status") != "DATASET_READY_FOR_OFFLINE_DERIVED_STATE_PILOT"
            or Path(batch.get("dataset_path", "")).resolve() != dataset
            or batch.get("run_id") != run_id or batch.get("repo_id") != repo_id
            or batch.get("audit") != strict
            or batch.get("dataset_sha256") != strict.get("dataset_sha256")):
        raise ValueError("post-capture wrapper does not bind the fresh strict inspect result")
    successful = batch.get("successful_episode_ids")
    if (not isinstance(successful, list) or len(successful) != len(expected_episode_ids)
            or len(set(successful)) != len(successful) or successful != expected_episode_ids):
        raise ValueError("post-capture wrapper accepted episodes differ from converted source origins")


def _validate_source_sidecars(binding: dict) -> None:
    source = Path(binding.get("source_path", "")).resolve(strict=True)
    hashes = binding.get("source_sidecar_sha256", {})
    if set(hashes) != {"source_map.jsonl", "run_sources.jsonl", "run_labels.jsonl"}:
        raise ValueError("source adapter does not bind the required source sidecars")
    for name, expected in hashes.items():
        path = source / "provenance" / name
        if not path.is_file() or _sha(path) != expected:
            raise ValueError(f"source sidecar changed since feedback normalization: {name}")


def prepare(args) -> dict:
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("legacy route must run under Python 3.10")
    runtime = _runtime()
    if runtime["lerobot_version"] != "0.1.0" or runtime["datasets_version"] != "3.6.0":
        raise RuntimeError(f"unsupported legacy runtime: {runtime}")
    adapter_manifest = Path(args.source_adapter_manifest).resolve(strict=True)
    adapter_sha = _sha(adapter_manifest)
    from server.training.post_capture_pipeline import prepare_dataset
    result = prepare_dataset(
        raw_root=Path(args.raw_root), outcome_root=Path(args.outcome_root),
        derived_root=Path(args.derived_root), dataset_home=Path(args.dataset_home),
        run_id=args.run_id, repo_id=args.repo_id, min_episodes=5,
        max_skew_ms=50.0,
    )
    dataset = Path(result["dataset_path"]).resolve(strict=True)
    batch_path = Path(result["batch_manifest"]).resolve(strict=True)
    batch = json.loads(batch_path.read_text(encoding="utf-8"))
    audited = _audit(dataset)
    conversion = json.loads((dataset / "provenance/proxy_conversion.json").read_text(encoding="utf-8"))
    expected_ids = [item["raw_episode_id"] for item in conversion.get("source_episodes", [])]
    _validate_batch_wrapper(batch, dataset, audited["strict_inspect"], run_id=args.run_id,
                            repo_id=args.repo_id, expected_episode_ids=expected_ids)
    if (audited["strict_inspect"].get("status") != "READY_FOR_OFFLINE_DERIVED_STATE_PILOT"
            or audited["strict_inspect"].get("episodes", 0) < 5
            or audited["strict_inspect"].get("formal_training_eligible") is not False
            or audited["strict_inspect"].get("offline_training_allowed") is not True):
        raise ValueError("legacy strict offline gate did not pass at five episodes")
    binding = json.loads(adapter_manifest.read_text(encoding="utf-8"))
    _validate_source_sidecars(binding)
    expected = {item["legacy_raw_episode_id"]: item["source_manifest_sha256"]
                for item in binding.get("episodes", [])}
    converted = {item["raw_episode_id"]: item["raw_manifest_sha256"]
                 for item in json.loads((dataset / "provenance/proxy_conversion.json").read_text(
                     encoding="utf-8")).get("source_episodes", [])}
    if len(expected) < 5 or len(converted) != len(expected) or set(expected) != set(converted):
        raise ValueError("legacy converter source episode set differs from source adapter")
    for item in binding["episodes"]:
        if converted.get(item["legacy_raw_episode_id"]) != item["legacy_raw_manifest_sha256"]:
            raise ValueError("legacy converter raw manifest hash differs from adapter")
    report = {
        "schema": "piper_feedback_legacy_handoff_v1",
        "status": "LEGACY_OFFLINE_PROXY_GATE_PASS",
        "runtime": runtime,
        "source_adapter_manifest_path": str(adapter_manifest),
        "source_adapter_manifest_sha256": adapter_sha,
        "post_capture_batch_manifest_path": str(batch_path),
        "post_capture_batch_manifest_sha256": _sha(batch_path),
        "run_id": args.run_id,
        "dataset_path": str(dataset),
        "dataset_tree_sha256": audited["dataset_tree_sha256"],
        "legacy_converter_sha256": _sha(dataset / "provenance/proxy_conversion.json"),
        "legacy_source_map_sha256": _sha(dataset / "provenance/source_map.jsonl"),
        "legacy_readback_audit_sha256": _sha(dataset / "provenance/readback_audit.json"),
        "legacy_readback_rows_sha256": _sha(dataset / "provenance/readback_frames.jsonl"),
        "strict_inspect": audited["strict_inspect"],
        "source_episode_bindings": binding["episodes"],
        "training_started": False,
        "hardware_control_allowed": False,
        "formal_training_eligible": False,
    }
    _write_new(Path(args.report), report)
    return {"status": report["status"], "dataset_path": str(dataset),
            "report_path": str(Path(args.report).resolve()),
            "dataset_tree_sha256": report["dataset_tree_sha256"],
            "episodes": report["strict_inspect"]["episodes"],
            "frames": report["strict_inspect"]["frames"],
            "training_started": False}


def verify(args) -> dict:
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("legacy route verification must run under Python 3.10")
    runtime = _runtime()
    if runtime["lerobot_version"] != "0.1.0" or runtime["datasets_version"] != "3.6.0":
        raise RuntimeError(f"unsupported legacy verification runtime: {runtime}")
    report_path = Path(args.report).resolve(strict=True)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("schema") != "piper_feedback_legacy_handoff_v1":
        raise ValueError("unknown legacy handoff report schema")
    if report.get("status") != "LEGACY_OFFLINE_PROXY_GATE_PASS":
        raise ValueError("legacy handoff did not record a passed offline proxy gate")
    dataset = Path(report["dataset_path"]).resolve(strict=True)
    if args.dataset and Path(args.dataset).resolve(strict=True) != dataset:
        raise ValueError("legacy report dataset path differs from requested dataset")
    if args.dataset_tree_sha256 and args.dataset_tree_sha256 != report.get("dataset_tree_sha256"):
        raise ValueError("legacy report dataset hash differs from v3 evidence")
    if _sha(Path(report["source_adapter_manifest_path"]).resolve(strict=True)) != report.get(
            "source_adapter_manifest_sha256"):
        raise ValueError("source adapter manifest hash changed")
    if _sha(Path(report["post_capture_batch_manifest_path"]).resolve(strict=True)) != report.get(
            "post_capture_batch_manifest_sha256"):
        raise ValueError("post-capture batch manifest hash changed")
    batch = json.loads(Path(report["post_capture_batch_manifest_path"]).read_text(encoding="utf-8"))
    audited = _audit(dataset, report.get("dataset_tree_sha256"))
    binding = json.loads(Path(report["source_adapter_manifest_path"]).read_text(encoding="utf-8"))
    _validate_source_sidecars(binding)
    if report.get("strict_inspect") != audited["strict_inspect"]:
        raise ValueError("fresh strict inspect differs from source handoff evidence")
    expected = {item["legacy_raw_episode_id"]: item["legacy_raw_manifest_sha256"]
                for item in report.get("source_episode_bindings", [])}
    converted = {item["raw_episode_id"]: item["raw_manifest_sha256"]
                 for item in json.loads((dataset / "provenance/proxy_conversion.json").read_text(
                     encoding="utf-8")).get("source_episodes", [])}
    if len(expected) < 5 or len(converted) != len(expected) or expected != converted:
        raise ValueError("fresh converter source identities differ from report")
    _validate_batch_wrapper(batch, dataset, audited["strict_inspect"],
                            run_id=report.get("run_id", ""),
                            repo_id=audited["strict_inspect"].get("repo_id"),
                            expected_episode_ids=list(expected))
    component_hashes = {
        "legacy_converter_sha256": "provenance/proxy_conversion.json",
        "legacy_source_map_sha256": "provenance/source_map.jsonl",
        "legacy_readback_audit_sha256": "provenance/readback_audit.json",
        "legacy_readback_rows_sha256": "provenance/readback_frames.jsonl",
    }
    for field, relative in component_hashes.items():
        if _sha(dataset / relative) != report.get(field):
            raise ValueError(f"legacy artifact hash changed: {relative}")
    if (audited["strict_inspect"].get("status") != "READY_FOR_OFFLINE_DERIVED_STATE_PILOT"
            or audited["strict_inspect"].get("formal_training_eligible") is not False
            or audited["strict_inspect"].get("offline_training_allowed") is not True):
        raise ValueError("legacy strict offline gate is no longer valid")
    return {"status": "FRESH_LEGACY_INSPECT_PASS", "dataset_path": str(dataset),
            "dataset_tree_sha256": audited["dataset_tree_sha256"],
            "strict_inspect": audited["strict_inspect"]}


def main() -> None:
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("prepare")
    for name in ("raw-root", "outcome-root", "derived-root", "dataset-home",
                 "source-adapter-manifest", "report", "run-id", "repo-id"):
        p.add_argument("--" + name, required=True)
    v = commands.add_parser("verify")
    v.add_argument("--report", required=True)
    v.add_argument("--dataset")
    v.add_argument("--dataset-tree-sha256")
    args = parser.parse_args()
    result = prepare(args) if args.command == "prepare" else verify(args)
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
