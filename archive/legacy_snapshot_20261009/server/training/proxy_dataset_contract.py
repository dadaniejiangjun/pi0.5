"""Validate immutable, source-audited proxy LeRobot datasets without OpenPI."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ACTION_SEMANTICS = "next_observed_state_proxy_nominal_units"
ACTION_TARGET_KIND = "derived_future_observed_state_position_target"
ACTION_TIME_RELATION = "observation_state_at_t_to_observed_state_at_t_plus_delta"
EXPECTED_DATASETS_VERSION = "3.6.0"
EXPECTED_LEROBOT_VERSION = "0.1.0"
EXPECTED_LEROBOT_REVISION = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class ProxyDatasetSpec:
    variant: str
    repo_id: str
    fps: int
    frames: int
    config_name: str
    exp_name: str
    default_steps: int
    eval_step: int
    source_audit_variant: str
    action_interval_ns: int


PROXY_DATASETS = {
    "2fps": ProxyDatasetSpec(
        variant="2fps",
        repo_id="local/pi05_proxy_cef166e2_2fps",
        fps=2,
        frames=39,
        config_name="pi05_piper_next_state_proxy_tiny_fit",
        exp_name="cef166e2_two_step",
        default_steps=2,
        eval_step=1,
        source_audit_variant="baseline",
        action_interval_ns=500_000_000,
    ),
    "10fps": ProxyDatasetSpec(
        variant="10fps",
        repo_id="local/cef166e2_proxy10fps_ds36",
        fps=10,
        frames=199,
        config_name="pi05_piper_next_state_proxy_10fps_tiny_fit",
        exp_name="cef166e2_10fps_ten_step",
        default_steps=10,
        # OpenPI numbers the first optimization update as step 0, so a
        # 10-update run writes checkpoint 9.
        eval_step=9,
        source_audit_variant="candidate",
        action_interval_ns=100_000_000,
    ),
}

# This is a separately named, source-audited copy of the historical 2fps
# baseline. It deliberately stays out of PROXY_DATASETS, so training/evaluation
# entrypoints cannot silently switch away from the checkpoint's original ID.
AUDITED_2FPS_BASELINE = ProxyDatasetSpec(
    variant="2fps-baseline-audited",
    repo_id="local/cef166e2_proxy2fps_ds36",
    fps=2,
    frames=39,
    config_name="pi05_piper_next_state_proxy_tiny_fit",
    exp_name="cef166e2_two_step",
    default_steps=2,
    eval_step=1,
    source_audit_variant="baseline",
    action_interval_ns=500_000_000,
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dataset_tree_sha256(root: Path) -> str:
    """Hash sorted relative paths and file hashes; reject symlinks."""
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError("dataset root must be a directory")
    entries: list[tuple[str, Path]] = []
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"dataset tree contains a symlink: {path}")
        if path.is_file():
            relative = path.relative_to(root).as_posix()
            entries.append((relative, path))
    if not entries:
        raise ValueError("dataset tree is empty")
    manifest = hashlib.sha256()
    for relative, path in sorted(entries):
        manifest.update(relative.encode("utf-8"))
        manifest.update(b"\0")
        manifest.update(sha256_file(path).encode("ascii"))
        manifest.update(b"\n")
    return manifest.hexdigest()


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path.name}")
    return value


def _jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSONL at {path.name}:{line_number}") from exc
        if not isinstance(value, dict):
            raise ValueError(f"expected JSON object at {path.name}:{line_number}")
        rows.append(value)
    if not rows:
        raise ValueError(f"JSONL sidecar is empty: {path.name}")
    return rows


def _require_sha(value: Any, name: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return value


def _dataset_file(root: Path, relative: Any, name: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise ValueError(f"{name} path is missing")
    path = (root / relative).resolve(strict=False)
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"{name} path escapes the dataset root")
    if not path.is_file():
        raise ValueError(f"{name} path is missing or escapes the dataset")
    return path


def _verify_tree_identity(root: Path, spec: ProxyDatasetSpec) -> None:
    if root.name != spec.repo_id.rsplit("/", 1)[-1]:
        raise ValueError("dataset directory does not match the required repo ID")


def inspect_proxy_dataset(
    root: Path,
    *,
    variant: str = "10fps",
    expected_dataset_tree_sha256: str | None = None,
    expected_readback_audit_sha256: str | None = None,
) -> dict[str, Any]:
    """Verify dataset identity, provenance, row links, file hashes, and audit claims.

    Training callers must supply hashes independently obtained from the laptop.
    This function validates the uploaded bytes against those hashes; it does not
    turn the proxy action reference into a robot command or formal training data.
    """
    if variant == AUDITED_2FPS_BASELINE.variant:
        spec = AUDITED_2FPS_BASELINE
    else:
        try:
            spec = PROXY_DATASETS[variant]
        except KeyError as exc:
            raise ValueError(f"unknown proxy dataset variant: {variant}") from exc
    root = root.resolve(strict=True)
    _verify_tree_identity(root, spec)

    provenance = root / "provenance"
    info_path = root / "meta" / "info.json"
    conversion_path = provenance / "proxy_conversion.json"
    readback_path = provenance / "readback_audit.json"
    source_map_path = provenance / "source_map.jsonl"
    source_map_report_path = provenance / "source_map_report.json"
    readback_frames_path = provenance / "readback_frames.jsonl"
    for required in (info_path, conversion_path, readback_path, source_map_path,
                     source_map_report_path, readback_frames_path):
        if not required.is_file():
            raise ValueError(f"required provenance or dataset file is missing: {required}")

    info = _json(info_path)
    conversion = _json(conversion_path)
    audit = _json(readback_path)
    source_map_report = _json(source_map_report_path)
    audit_sha = sha256_file(readback_path)
    tree_sha = dataset_tree_sha256(root)
    if expected_readback_audit_sha256 is not None:
        expected = _require_sha(expected_readback_audit_sha256, "expected readback audit hash")
        if audit_sha != expected:
            raise ValueError("readback audit hash differs from the laptop-provided value")
    if expected_dataset_tree_sha256 is not None:
        expected = _require_sha(expected_dataset_tree_sha256, "expected dataset tree hash")
        if tree_sha != expected:
            raise ValueError("dataset tree hash differs from the laptop-provided value")

    if (info.get("robot_type") != "piper"
            or info.get("total_episodes") != 1
            or info.get("total_frames") != spec.frames
            or info.get("fps") != spec.fps
            or info.get("splits") != {"train": "0:1"}):
        raise ValueError("LeRobot info identity, frame count, fps, or split mismatch")
    try:
        info_parquet_path = info["data_path"].format(episode_chunk=0, episode_index=0)
        info_video_paths = {
            "base": info["video_path"].format(
                episode_chunk=0, video_key="observation.images.image", episode_index=0),
            "wrist": info["video_path"].format(
                episode_chunk=0, video_key="observation.images.wrist_image", episode_index=0),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("LeRobot data/video path templates are invalid") from exc
    if info_parquet_path != "data/chunk-000/episode_000000.parquet":
        raise ValueError("LeRobot Parquet template does not identify the audited episode")
    for feature in ("observation.images.image", "observation.images.wrist_image"):
        feature_info = info.get("features", {}).get(feature, {})
        if (feature_info.get("dtype") != "video"
                or feature_info.get("info", {}).get("video.fps") != spec.fps
                or feature_info.get("shape") != [480, 640, 3]):
            raise ValueError(f"LeRobot image feature metadata mismatch: {feature}")
    if (info.get("features", {}).get("observation.state", {}).get("shape") != [7]
            or info.get("features", {}).get("action", {}).get("shape") != [7]):
        raise ValueError("proxy state/action features must both have seven dimensions")

    runtime = conversion.get("conversion_runtime", {})
    if (conversion.get("schema") != "piper_proxy_lerobot_v1"
            or conversion.get("repo_id") != spec.repo_id
            or conversion.get("frames") != spec.frames
            or conversion.get("episodes") != 1
            or conversion.get("split") != "train_only"
            or conversion.get("action_semantics") != ACTION_SEMANTICS
            or conversion.get("formal_training_eligible") is not False
            or conversion.get("hardware_control_allowed") is not False
            or runtime.get("datasets_version") != EXPECTED_DATASETS_VERSION
            or runtime.get("lerobot_version") != EXPECTED_LEROBOT_VERSION
            or runtime.get("lerobot_revision") != EXPECTED_LEROBOT_REVISION):
        raise ValueError("conversion provenance or pinned runtime mismatch")

    if (audit.get("schema") != "piper_proxy_lerobot_readback_audit_v1"
            or audit.get("dataset_path") != root.name
            or audit.get("dataset_repo_id") != spec.repo_id
            or audit.get("fps") != spec.fps
            or audit.get("frames") != spec.frames
            or audit.get("episodes") != 1
            or audit.get("split") != "train_only"
            or audit.get("action_semantics") != ACTION_SEMANTICS
            or audit.get("source_frame_extraction_verified") is not True
            or audit.get("source_audit_variant") != spec.source_audit_variant
            or audit.get("source_map_path") != "provenance/source_map.jsonl"
            or audit.get("readback_frames_path") != "provenance/readback_frames.jsonl"
            or audit.get("formal_training_eligible") is not False
            or audit.get("hardware_control_allowed") is not False):
        raise ValueError("readback audit is not a verified offline-only 10fps candidate")
    if audit.get("dataset_info_sha256") != sha256_file(info_path):
        raise ValueError("LeRobot info hash does not match the readback audit")

    source_map = _jsonl(source_map_path)
    readback_rows = _jsonl(readback_frames_path)
    source_map_sha = sha256_file(source_map_path)
    readback_rows_sha = sha256_file(readback_frames_path)
    if (len(source_map) != spec.frames
            or len(readback_rows) != spec.frames
            or conversion.get("source_map_rows") != spec.frames
            or source_map_report.get("source_map_rows") != spec.frames
            or source_map_report.get("source_map_sha256") != source_map_sha
            or conversion.get("source_map_sha256") != source_map_sha
            or audit.get("source_map_sha256") != source_map_sha
            or audit.get("readback_frames_sha256") != readback_rows_sha):
        raise ValueError("source/readback sidecar hashes or row counts mismatch")
    if (source_map_report.get("schema") != "piper_proxy_lerobot_source_map_v1"
            or source_map_report.get("action_semantics") != ACTION_SEMANTICS
            or source_map_report.get("formal_training_eligible") is not False
            or source_map_report.get("hardware_control_allowed") is not False):
        raise ValueError("source-map report does not preserve the proxy-only contract")
    if (audit.get("raw_episode_id") != source_map_report.get("raw_episode_id")
            or source_map_report.get("raw_episode_id") != "cef166e2cc6240b397db241a916ece3a"):
        raise ValueError("raw episode identity does not match the audited candidate")

    source_hashes = audit.get("source_audit_hashes", {})
    for key in ("raw_manifest_sha256", "aligned_metadata_sha256", "aligned_frames_sha256",
                "frame_map_sha256", "proxy_manifest_sha256", "proxy_targets_sha256"):
        _require_sha(source_hashes.get(key), f"source audit {key}")
    _require_sha(source_hashes.get("outcome_label_sha256"),
                 "source audit outcome_label_sha256")
    raw_file_hashes = source_hashes.get("raw_files_sha256")
    if not isinstance(raw_file_hashes, dict) or not raw_file_hashes:
        raise ValueError("source audit raw-file hash map is missing")
    for filename, digest in raw_file_hashes.items():
        if not isinstance(filename, str) or Path(filename).name != filename:
            raise ValueError("source audit contains an unsafe raw-file identity")
        _require_sha(digest, f"source audit raw file {filename}")
    for key in ("proxy_manifest_sha256", "proxy_targets_sha256"):
        if conversion.get(key) != source_hashes.get(key):
            raise ValueError(f"conversion/source audit mismatch: {key}")
    if (source_map_report.get("raw_manifest_sha256") != source_hashes.get("raw_manifest_sha256")
            or source_map_report.get("aligned_metadata_sha256") != source_hashes.get("aligned_metadata_sha256")
            or source_map_report.get("aligned_frames_sha256") != source_hashes.get("aligned_frames_sha256")
            or source_map_report.get("frame_map_sha256") != source_hashes.get("frame_map_sha256")
            or source_map_report.get("proxy_manifest_sha256") != source_hashes.get("proxy_manifest_sha256")
            or source_map_report.get("proxy_targets_sha256") != source_hashes.get("proxy_targets_sha256")):
        raise ValueError("source-map report does not match readback source audit hashes")

    timestamp_keys = ("gripper_timestamp_ns", "joint_12_timestamp_ns", "joint_34_timestamp_ns",
                      "joint_56_timestamp_ns", "status_timestamp_ns")
    for index, (source_row, readback_row) in enumerate(zip(source_map, readback_rows)):
        if (source_row.get("dataset_row_index") != index
                or source_row.get("episode_index") != 0
                or source_row.get("action_semantics") != ACTION_SEMANTICS
                or readback_row.get("dataset_row_index") != index):
            raise ValueError(f"source map/readback order mismatch at row {index}")
        observation = source_row.get("observation", {})
        proxy_target = source_row.get("proxy_target", {})
        source_refs = observation.get("source_refs", {})
        images = observation.get("images", {})
        if (not isinstance(observation.get("timestamp_ns"), int)
                or not isinstance(proxy_target.get("timestamp_ns"), int)
                or proxy_target["timestamp_ns"] - observation["timestamp_ns"] != spec.action_interval_ns
                or not isinstance(proxy_target.get("action_float32_sha256"), str)):
            raise ValueError(f"proxy target timing or identity mismatch at row {index}")
        _require_sha(proxy_target["action_float32_sha256"],
                     f"proxy action hash at row {index}")
        for key in timestamp_keys:
            if not isinstance(source_refs.get(key), int) or source_refs[key] <= 0:
                raise ValueError(f"source reference timestamp missing at row {index}: {key}")
        if not isinstance(source_refs.get("base_frame_id"), int) or not isinstance(
                source_refs.get("wrist_frame_id"), int):
            raise ValueError(f"camera source frame reference missing at row {index}")
        for role in ("base", "wrist"):
            preview_relpath = images.get(role, {}).get("preview_relpath")
            if (not isinstance(preview_relpath, str)
                    or Path(preview_relpath).is_absolute()
                    or ".." in Path(preview_relpath).parts
                    or "\\" in preview_relpath):
                raise ValueError(f"unsafe preview source reference at row {index}: {role}")
            expected_rgb_sha = _require_sha(
                images.get(role, {}).get("rgb_pixels_sha256"),
                f"source {role} RGB hash at row {index}",
            )
            if readback_row.get(f"{role}_source_rgb_sha256") != expected_rgb_sha:
                raise ValueError(f"readback frame is not linked to source {role} at row {index}")
            _require_sha(readback_row.get(f"{role}_decoded_rgb_sha256"),
                         f"decoded {role} RGB hash at row {index}")

    parquet_path = _dataset_file(root, audit.get("parquet_path"), "Parquet")
    if audit.get("parquet_path") != info_parquet_path:
        raise ValueError("readback audit and LeRobot Parquet template disagree")
    if audit.get("parquet_sha256") != sha256_file(parquet_path):
        raise ValueError("Parquet file hash differs from readback audit")
    parquet_summary = audit.get("parquet", {})
    if (parquet_summary.get("rows") != spec.frames
            or parquet_summary.get("index_order_verified") is not True
            or parquet_summary.get("state_values_verified") is not True
            or parquet_summary.get("action_values_verified") is not True):
        raise ValueError("readback audit does not verify Parquet values and row order")

    video_tracks = audit.get("video_tracks", {})
    for role in ("base", "wrist"):
        track = video_tracks.get(role, {})
        video_path = _dataset_file(root, track.get("path"), f"{role} video")
        if (track.get("path") != info_video_paths[role]
                or track.get("sha256") != sha256_file(video_path)
                or track.get("decoded_frames") != spec.frames
                or track.get("fps") != spec.fps
                or track.get("pts_monotonic") is not True
                or track.get("decoded_to_source_order_by_dataset_index") is not True
                or track.get("width") != 640
                or track.get("height") != 480):
            raise ValueError(f"{role} video readback does not match its source audit")

    return {
        "variant": spec.variant,
        "repo_id": spec.repo_id,
        "fps": spec.fps,
        "frames": spec.frames,
        "episodes": 1,
        "split": "train_only",
        "action_semantics": ACTION_SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "action_time_relation": ACTION_TIME_RELATION,
        "action_source": "future_observed_joint_and_gripper_feedback",
        "raw_command_supervision": False,
        "offline_tiny_fit_allowed": True,
        "source_audit_variant": spec.source_audit_variant,
        "source_frame_extraction_verified": True,
        "dataset_tree_sha256": tree_sha,
        "readback_audit_sha256": audit_sha,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
        "prediction_only": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-path", required=True, type=Path)
    parser.add_argument(
        "--variant",
        choices=(*PROXY_DATASETS, AUDITED_2FPS_BASELINE.variant),
        default="10fps",
    )
    parser.add_argument("--expected-dataset-tree-sha256")
    parser.add_argument("--expected-readback-audit-sha256")
    args = parser.parse_args()
    print(json.dumps(inspect_proxy_dataset(
        args.dataset_path,
        variant=args.variant,
        expected_dataset_tree_sha256=args.expected_dataset_tree_sha256,
        expected_readback_audit_sha256=args.expected_readback_audit_sha256,
    ), indent=2))


if __name__ == "__main__":
    main()
