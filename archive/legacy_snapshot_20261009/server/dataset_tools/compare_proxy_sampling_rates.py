"""Verify paired 2 fps and higher-rate proxy variants from one raw episode.

This is a data/provenance experiment only. It does not train a policy, alter
the 2 fps policy config, or authorize robot control.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from laptop.recorder.label_episode import verify_label
from laptop.recorder.verify_source import verify_source
from server.dataset_tools.convert_next_state_proxy import inspect


BASELINE_FPS = 2
PROXY_SEMANTICS = "next_observed_state_proxy_nominal_units"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _variant(
    *,
    name: str,
    fps: int,
    raw: Path,
    outcome: Path,
    preview: Path,
    proxy: Path,
) -> dict:
    aligned_path = preview / "aligned_episode.json"
    frames_path = preview / "frames.jsonl"
    proxy_manifest_path = proxy / "proxy_manifest.json"
    proxy_targets_path = proxy / "proxy_targets.jsonl"
    aligned = _json(aligned_path)
    proxy_manifest = _json(proxy_manifest_path)
    outcome_info = verify_label(raw, outcome)
    if outcome_info["success"] is not True:
        raise ValueError("proxy sampling comparison requires an explicitly successful episode")
    if (aligned.get("schema") != "piper_aligned_preview_v1"
            or aligned.get("fps") != fps
            or aligned.get("raw_episode_id") != raw.name
            or aligned.get("training_eligible") is not False):
        raise ValueError(f"{name} preview identity, rate, or eligibility mismatch")
    if (proxy_manifest.get("semantics") != PROXY_SEMANTICS
            or proxy_manifest.get("fps") != fps
            or proxy_manifest.get("preview_episode_id") != preview.name
            or proxy_manifest.get("raw_episode_id") != raw.name
            or proxy_manifest.get("formal_training_eligible") is not False
            or proxy_manifest.get("hardware_control_allowed") is not False
            or proxy_manifest.get("outcome_label_sha256") != _sha256(outcome)):
        raise ValueError(f"{name} proxy manifest identity or offline-only gate mismatch")

    source = verify_source(raw, preview)
    manifest, proxy_rows, shapes = inspect(proxy, preview)
    if (source["source_frame_extraction_verified"] is not True
            or source["raw_manifest_sha256"] != proxy_manifest.get("raw_manifest_sha256")
            or manifest != proxy_manifest):
        raise ValueError(f"{name} raw-source verification failed")
    if len(proxy_rows) != len(frames_path.read_text(encoding="utf-8").splitlines()) - 1:
        raise ValueError(f"{name} proxy transition count does not match aligned frames")

    raw_manifest = _json(raw / "manifest.json")
    return {
        "name": name,
        "fps": fps,
        "aligned_episode_id": preview.name,
        "proxy_directory": proxy.name,
        "aligned_frame_count": aligned["frame_count"],
        "proxy_transition_count": len(proxy_rows),
        "proxy_target_offset_ns": proxy_manifest["target_offset_ns"],
        "verified_video_frames": source["video_frames_checked"],
        "source_frame_extraction_verified": source["source_frame_extraction_verified"],
        "camera_shapes": shapes,
        "source_hashes": {
            "raw_manifest_sha256": source["raw_manifest_sha256"],
            "raw_files_sha256": {
                relative: record["sha256"]
                for relative, record in sorted(raw_manifest["files"].items())
            },
            "outcome_label_sha256": _sha256(outcome),
            "aligned_metadata_sha256": source["aligned_metadata_sha256"],
            "aligned_frames_sha256": source["aligned_frames_sha256"],
            "frame_map_sha256": source["frame_map_sha256"],
            "proxy_manifest_sha256": _sha256(proxy_manifest_path),
            "proxy_targets_sha256": _sha256(proxy_targets_path),
        },
    }


def compare_sampling_rates(
    *,
    raw: Path,
    outcome: Path,
    baseline_preview: Path,
    baseline_proxy: Path,
    candidate_preview: Path,
    candidate_proxy: Path,
) -> dict:
    raw, outcome, baseline_preview, baseline_proxy, candidate_preview, candidate_proxy = (
        Path(value).resolve() for value in
        (raw, outcome, baseline_preview, baseline_proxy, candidate_preview, candidate_proxy)
    )
    baseline_metadata = _json(baseline_preview / "aligned_episode.json")
    candidate_metadata = _json(candidate_preview / "aligned_episode.json")
    baseline_fps, candidate_fps = baseline_metadata.get("fps"), candidate_metadata.get("fps")
    if (type(baseline_fps) is not int or baseline_fps != BASELINE_FPS
            or type(candidate_fps) is not int or candidate_fps <= BASELINE_FPS):
        raise ValueError("comparison must preserve a 2 fps baseline and a higher-rate candidate")
    if baseline_preview == candidate_preview or baseline_proxy == candidate_proxy:
        raise ValueError("baseline and candidate outputs must be separate")

    baseline = _variant(
        name="baseline", fps=baseline_fps, raw=raw, outcome=outcome,
        preview=baseline_preview, proxy=baseline_proxy,
    )
    candidate = _variant(
        name="higher_frequency_candidate", fps=candidate_fps, raw=raw, outcome=outcome,
        preview=candidate_preview, proxy=candidate_proxy,
    )
    if baseline["source_hashes"]["raw_manifest_sha256"] != candidate["source_hashes"]["raw_manifest_sha256"]:
        raise ValueError("sampling variants do not derive from the same immutable raw episode")
    if baseline["source_hashes"]["outcome_label_sha256"] != candidate["source_hashes"]["outcome_label_sha256"]:
        raise ValueError("sampling variants do not share the same outcome label")

    return {
        "schema": "piper_proxy_sampling_rate_comparison_v1",
        "status": "PASS_DATA_VARIANTS_VERIFIED",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "experiment_type": "same_raw_episode_source_verified_temporal_sampling",
        "raw_episode_id": raw.name,
        "outcome_success": True,
        "action_semantics": PROXY_SEMANTICS,
        "variants": {"baseline": baseline, "candidate": candidate},
        "comparison": {
            "fps_ratio": candidate_fps / baseline_fps,
            "transition_count_ratio": candidate["proxy_transition_count"] / baseline["proxy_transition_count"],
            "same_raw_episode": True,
            "same_outcome_label": True,
            "same_source_video_frames_verified": True,
        },
        "scope": {
            "raw_capture_modified": False,
            "resampling_method": "timestamp-grid selection of existing raw frames and feedback; no interpolation",
            "model_training_run": False,
            "held_out_evaluation_run": False,
            "policy_config_changed": False,
            "hardware_control_allowed": False,
            "formal_training_eligible": False,
        },
        "limitations": [
            "The successful outcome is operator-reported and the hand is visible in the demonstration.",
            "Proxy targets are next-observed states, not verified PiPER commands.",
            "Joint and gripper units remain nominal and hardware-unverified.",
            "Host arrival timestamps do not prove camera exposure synchronization.",
            "This experiment compares source-verified sample density, not policy quality or task success.",
        ],
    }


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--outcome", type=Path, required=True)
    parser.add_argument("--baseline-preview", type=Path, required=True)
    parser.add_argument("--baseline-proxy", type=Path, required=True)
    parser.add_argument("--candidate-preview", type=Path, required=True)
    parser.add_argument("--candidate-proxy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare_sampling_rates(
        raw=args.raw, outcome=args.outcome,
        baseline_preview=args.baseline_preview, baseline_proxy=args.baseline_proxy,
        candidate_preview=args.candidate_preview, candidate_proxy=args.candidate_proxy,
    )
    output = args.output.resolve()
    protected = (args.raw.resolve(), args.baseline_preview.resolve(), args.baseline_proxy.resolve(),
                 args.candidate_preview.resolve(), args.candidate_proxy.resolve())
    if any(output.is_relative_to(path) for path in protected):
        raise ValueError("report output must not be inside raw or derived source data")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"status": report["status"], "output": str(output),
                      "baseline_samples": report["variants"]["baseline"]["proxy_transition_count"],
                      "candidate_samples": report["variants"]["candidate"]["proxy_transition_count"],
                      "hardware_control_allowed": False}, indent=2))


if __name__ == "__main__":
    main()
