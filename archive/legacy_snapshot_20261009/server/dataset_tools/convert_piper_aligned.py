"""Validate reviewed PiPER aligned episodes, then derive a local LeRobot dataset.

This is NOT a raw-CAN-to-action estimator. Upstream alignment and human review
must supply a separately verified 7D action target. No robot or policy runs here.
The command defaults to read-only preflight; --write-derived is explicit.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
import re
import uuid

import numpy as np
from PIL import Image
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

from laptop.recorder.piper_command_targets import ACTION_TARGET_SOURCE


SCHEMA = "piper_aligned_episode_v1"
TASK_ID = "pick_place_black_rod_green_mat_v1"
TASK = "Pick up the black rod and place it on the green desk mat."
UNITS = ["radian"] * 6 + ["meter"]
QC_FLAGS = (
    "visual_review_passed", "camera_sync_verified", "resampling_verified",
    "joint_units_verified", "gripper_units_verified", "gripper_action_verified",
    "action_target_verified",
)
REF_KEYS = (
    "base_frame_id", "wrist_frame_id", "joint_12_timestamp_ns",
    "joint_34_timestamp_ns", "joint_56_timestamp_ns",
    "gripper_timestamp_ns", "status_timestamp_ns",
)
TIME_KEYS = ("policy_timestamp_ns", "base_timestamp_ns", "wrist_timestamp_ns",
             "state_timestamp_ns", "action_target_timestamp_ns")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _contained_file(directory: Path, relative: str) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError(f"invalid relative file path: {relative!r}")
    path = (directory / relative).resolve()
    if not path.is_relative_to(directory.resolve()) or not path.is_file():
        raise ValueError(f"file missing or outside episode: {relative!r}")
    return path


def _seven(value: object, label: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} is not numeric") from exc
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must be a finite 7D vector")
    return result


def _rgb(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        if image.mode != "RGB":
            raise ValueError(f"image must be RGB: {path}")
        result = np.asarray(image)
    if result.ndim != 3 or result.shape[-1] != 3 or result.dtype != np.uint8:
        raise ValueError(f"image has invalid shape/dtype: {path}")
    return result


@dataclass(frozen=True)
class CheckedEpisode:
    path: Path
    metadata: dict
    rows: list[dict]
    raw_manifest_sha256: str
    aligned_metadata_sha256: str
    aligned_frames_sha256: str
    image_shapes: dict[str, tuple[int, int, int]]
    source_verification: dict | None


def inspect_episode(path: Path, raw_root: Path, *, allow_fixture: bool = False) -> CheckedEpisode:
    """Check every file/row before output creation; a manifest claim alone is insufficient."""

    path = Path(path).resolve()
    raw_root = Path(raw_root).resolve()
    metadata_path = _contained_file(path, "aligned_episode.json")
    frames_path = _contained_file(path, "frames.jsonl")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("schema") != SCHEMA or metadata.get("task_id") != TASK_ID or metadata.get("task") != TASK:
        raise ValueError("wrong aligned schema or task")
    source = metadata.get("source")
    if source != "manual_teach" and not (allow_fixture and source == "synthetic_fixture"):
        raise ValueError("only reviewed manual-teach episodes can be converted")
    if metadata.get("success") is not True or any(metadata.get(flag) is not True for flag in QC_FLAGS):
        raise ValueError("episode success or required QC evidence missing")
    if metadata.get("action_semantics") != "absolute_joint_position":
        raise ValueError("unverified action semantics")
    if source == "manual_teach":
        if metadata.get("action_target_source") != ACTION_TARGET_SOURCE:
            raise ValueError("production PiPER conversion requires raw CAN command-frame targets")
        if (metadata.get("action_target_provenance_verified") is not True
                or metadata.get("action_target_source_log_relpath") != "command_can_raw.jsonl"
                or metadata.get("action_hold_semantics") != "zero_order_hold_latest_complete_absolute_target"):
            raise ValueError("raw command target provenance/hold contract is not verified")
    if metadata.get("state_units") != UNITS or metadata.get("action_units") != UNITS:
        raise ValueError("canonical state/action units not verified")
    fps = metadata.get("fps")
    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError("fps must be an integer in [1, 60]")
    offset = metadata.get("action_offset_ns")
    if type(offset) is not int or offset < 0:
        raise ValueError("action_offset_ns must be explicit and nonnegative")
    episode_id = metadata.get("episode_id")
    raw_id = metadata.get("raw_episode_id")
    if not isinstance(episode_id, str) or not episode_id.isalnum() or not isinstance(raw_id, str) or not raw_id.isalnum():
        raise ValueError("episode identifiers must be alphanumeric")
    raw_dir = (raw_root / raw_id).resolve()
    if not raw_dir.is_relative_to(raw_root):
        raise ValueError("raw episode path escapes root")
    raw_manifest_path = _contained_file(raw_dir, "manifest.json")
    raw_hash = _sha256(raw_manifest_path)
    if raw_hash != metadata.get("raw_manifest_sha256"):
        raise ValueError("raw source manifest checksum mismatch")
    raw_manifest = json.loads(raw_manifest_path.read_text(encoding="utf-8"))
    if raw_manifest.get("episode_id") != raw_id or raw_manifest.get("source") != source:
        raise ValueError("raw source episode identity mismatch")
    files = raw_manifest.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("raw source manifest has no files")
    if source == "manual_teach" and not {"base.mp4", "wrist.mp4"}.issubset(files):
        raise ValueError("raw source lacks both camera videos")
    for relative, record in files.items():
        source_file = _contained_file(raw_dir, relative)
        if not isinstance(record, dict) or _sha256(source_file) != record.get("sha256"):
            raise ValueError(f"raw source checksum mismatch: {relative}")

    rows: list[dict] = []
    shapes: dict[str, tuple[int, int, int]] = {}
    previous = {key: -1 for key in TIME_KEYS}
    with frames_path.open(encoding="utf-8") as handle:
        for frame_index, line in enumerate(handle):
            row = json.loads(line)
            if not isinstance(row, dict) or row.get("frame_index") != frame_index:
                raise ValueError("frame indices must be contiguous from zero")
            for key in TIME_KEYS:
                value = row.get(key)
                if type(value) is not int or value <= previous[key]:
                    raise ValueError(f"non-increasing {key} at frame {frame_index}")
                previous[key] = value
            policy_ts = row["policy_timestamp_ns"]
            state_ts = row["state_timestamp_ns"]
            if any(abs(row[key] - state_ts) > 50_000_000 for key in
                   ("base_timestamp_ns", "wrist_timestamp_ns", "policy_timestamp_ns")):
                raise ValueError(f"camera/state skew exceeds 50 ms at frame {frame_index}")
            if abs((row["action_target_timestamp_ns"] - policy_ts) - offset) > 1_000_000:
                raise ValueError(f"action offset mismatch at frame {frame_index}")
            if rows and abs((policy_ts - rows[-1]["policy_timestamp_ns"]) - 1e9 / fps) > 2_000_000:
                raise ValueError("derived policy timestamps are not on the declared fps grid")
            refs = row.get("source_refs")
            if not isinstance(refs, dict) or any(type(refs.get(key)) is not int or refs[key] < 0 for key in REF_KEYS):
                raise ValueError(f"missing per-frame raw source references at frame {frame_index}")
            if any(abs(refs[key] - state_ts) > 50_000_000 for key in REF_KEYS[2:]):
                raise ValueError(f"raw state-channel reference skew exceeds 50 ms at frame {frame_index}")
            counts = raw_manifest.get("counts", {})
            if source == "manual_teach" and (
                not isinstance(counts, dict)
                or any(type(counts.get(role)) is not int or refs[f"{role}_frame_id"] >= counts[role]
                       for role in ("base", "wrist"))
            ):
                raise ValueError(f"camera frame reference outside raw manifest counts at frame {frame_index}")
            _seven(row.get("state"), "state")
            _seven(row.get("action"), "action")
            for role in ("base", "wrist"):
                image_path = _contained_file(path, row.get(f"{role}_image"))
                shape = _rgb(image_path).shape
                if role in shapes and shapes[role] != shape:
                    raise ValueError(f"{role} image resolution changes within episode")
                shapes[role] = shape
            rows.append(row)
    if len(rows) < 2 or len(rows) != metadata.get("frame_count"):
        raise ValueError("episode frame count is missing or too short")
    source_verification = None
    if source == "manual_teach":
        from laptop.recorder.verify_source import verify_source
        source_verification = verify_source(raw_dir, path)
        if source_verification.get("command_target_provenance_verified") is not True:
            raise ValueError("raw CAN command targets did not pass sample-level verification")
    return CheckedEpisode(path, metadata, rows, raw_hash, _sha256(metadata_path),
                          _sha256(frames_path), shapes, source_verification)


def inspect_collection(paths: list[Path], raw_root: Path, *, allow_fixture: bool = False) -> list[CheckedEpisode]:
    if not paths:
        raise ValueError("at least one aligned episode is required")
    episodes = [inspect_episode(path, raw_root, allow_fixture=allow_fixture) for path in paths]
    if len({item.metadata["episode_id"] for item in episodes}) != len(episodes):
        raise ValueError("duplicate episode IDs")
    if len({item.metadata["raw_episode_id"] for item in episodes}) != len(episodes):
        raise ValueError("duplicate raw source episodes")
    if len({item.metadata["source"] for item in episodes}) != 1:
        raise ValueError("synthetic fixtures and real episodes cannot be mixed")
    if len({item.metadata["fps"] for item in episodes}) != 1:
        raise ValueError("mixed fps in one derived dataset")
    if any(item.image_shapes != episodes[0].image_shapes for item in episodes):
        raise ValueError("mixed camera resolutions in one derived dataset")
    return episodes


def convert(episodes: list[CheckedEpisode], output: Path, repo_id: str) -> dict:
    """Write immutable derived data and a source map; never mark it trainable."""

    output = Path(output).resolve()
    if not episodes or output.exists() or not re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_-]+", repo_id):
        raise ValueError("episodes, unused output path and namespace/repo_id are required")
    if any(output.is_relative_to(item.path) or item.path.is_relative_to(output) for item in episodes):
        raise ValueError("output may not overlap source episodes")
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_name(output.name + ".INCOMPLETE-" + uuid.uuid4().hex[:12])
    first = episodes[0]
    features = {
        "observation.images.image": {"dtype": "video", "shape": first.image_shapes["base"],
                                     "names": ["height", "width", "channel"]},
        "observation.images.wrist_image": {"dtype": "video", "shape": first.image_shapes["wrist"],
                                           "names": ["height", "width", "channel"]},
        "observation.state": {"dtype": "float32", "shape": (7,), "names": ["state"]},
        "action": {"dtype": "float32", "shape": (7,), "names": ["action"]},
    }
    dataset = LeRobotDataset.create(repo_id=repo_id, root=pending, robot_type="piper",
                                    fps=first.metadata["fps"], features=features)
    source_map = []
    for derived_episode_index, item in enumerate(episodes):
        for row in item.rows:
            base_path = _contained_file(item.path, row["base_image"])
            wrist_path = _contained_file(item.path, row["wrist_image"])
            dataset.add_frame({
                "observation.images.image": _rgb(base_path),
                "observation.images.wrist_image": _rgb(wrist_path),
                "observation.state": _seven(row["state"], "state"),
                "action": _seven(row["action"], "action"),
                "task": TASK,
            })
            source_row = {
                "derived_episode_index": derived_episode_index,
                "derived_frame_index": row["frame_index"],
                "aligned_episode_id": item.metadata["episode_id"],
                "raw_episode_id": item.metadata["raw_episode_id"],
                "timestamps_ns": {key: row[key] for key in TIME_KEYS},
                "source_refs": row["source_refs"],
                "base_image_relpath": row["base_image"],
                "wrist_image_relpath": row["wrist_image"],
                "base_image_sha256": _sha256(base_path),
                "wrist_image_sha256": _sha256(wrist_path),
                "action_target_source": item.metadata.get("action_target_source"),
            }
            if "action_source_timestamp_ns" in row:
                source_row["action_source_timestamp_ns"] = row["action_source_timestamp_ns"]
                source_row["action_source_refs"] = row["action_source_refs"]
            source_map.append(source_row)
        dataset.save_episode()
    provenance = pending / "provenance"
    provenance.mkdir()
    map_path = provenance / "source_map.jsonl"
    with map_path.open("x", encoding="utf-8") as handle:
        for row in source_map:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    synthetic_fixture = any(item.metadata["source"] == "synthetic_fixture" for item in episodes)
    acceptance_path = provenance / "acceptance_manifest.json"
    acceptance_sha256 = None
    if not synthetic_fixture:
        accepted_episodes = []
        for item in episodes:
            flags = item.metadata
            command_verified = bool(
                item.source_verification is not None
                and item.source_verification.get("command_target_provenance_verified") is True
            )
            accepted_episodes.append({
                "aligned_episode_id": flags["episode_id"],
                "raw_episode_id": flags["raw_episode_id"],
                "source": flags["source"],
                "success": flags["success"],
                "action_target_source": flags.get("action_target_source"),
                "action_target_verified": flags.get("action_target_verified") is True,
                "command_target_provenance_verified": command_verified,
                "camera_sync_verified": flags.get("camera_sync_verified") is True,
                "gripper_verified": (flags.get("gripper_units_verified") is True
                                     and flags.get("gripper_action_verified") is True),
                "units_verified": (flags.get("joint_units_verified") is True
                                   and flags.get("gripper_units_verified") is True),
                "raw_manifest_sha256": item.raw_manifest_sha256,
                "aligned_metadata_sha256": item.aligned_metadata_sha256,
                "aligned_frames_sha256": item.aligned_frames_sha256,
            })
        acceptance = {
            "schema": "piper_real_demo_acceptance_v1",
            "repo_id": repo_id,
            "action_semantics": "absolute_joint_position_rad_gripper_meter",
            "episodes": accepted_episodes,
            "training_eligible": False,
            "human_review_source": "checksum_bound_piper_alignment_review_v1",
        }
        acceptance_path.write_text(json.dumps(acceptance, indent=2, sort_keys=True) + "\n",
                                   encoding="utf-8")
        acceptance_sha256 = _sha256(acceptance_path)
    manifest = {
        "schema": "piper_lerobot_conversion_v1",
        "repo_id": repo_id,
        "source_episodes": [{
            "aligned_episode_id": item.metadata["episode_id"],
            "raw_episode_id": item.metadata["raw_episode_id"],
            "raw_manifest_sha256": item.raw_manifest_sha256,
            "aligned_metadata_sha256": item.aligned_metadata_sha256,
            "aligned_frames_sha256": item.aligned_frames_sha256,
            "source_verification": item.source_verification,
        } for item in episodes],
        "source_map_sha256": _sha256(map_path),
        "frame_count": len(source_map),
        "synthetic_fixture": synthetic_fixture,
        "training_eligible": False,
        "source_frame_extraction_verified": all(item.source_verification is not None for item in episodes),
        "command_target_provenance_verified": all(
            item.source_verification is not None
            and item.source_verification.get("command_target_provenance_verified") is True
            for item in episodes
        ),
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "action_target_source": ACTION_TARGET_SOURCE,
        "acceptance_manifest": ("provenance/acceptance_manifest.json" if acceptance_sha256 else None),
        "acceptance_manifest_sha256": acceptance_sha256,
        "reason": "raw command provenance is verified; one-episode tiny-fit remains an offline train-only experiment",
    }
    (provenance / "conversion_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    pending.rename(output)
    return {"status": "CONVERTED_NEEDS_AUDIT", "output": str(output), **manifest}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--raw-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-id", default="local/piper_v1")
    parser.add_argument("--write-derived", action="store_true")
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.raw_root.resolve()):
        print(json.dumps({"status": "INPUT_REJECTED", "error": "output must not be inside raw source root"}))
        return 2
    try:
        checked = inspect_collection(args.episodes, args.raw_root)
    except (OSError, ValueError, TypeError) as exc:
        print(json.dumps({"status": "INPUT_REJECTED", "error": str(exc)}))
        return 2
    if not args.write_derived:
        print(json.dumps({"status": "PREFLIGHT_PASS_NO_WRITE", "episodes": len(checked),
                          "frames": sum(len(item.rows) for item in checked),
                          "training_eligible": False}, indent=2))
        return 0
    print(json.dumps(convert(checked, args.output, args.repo_id), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
