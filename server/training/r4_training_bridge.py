"""Adapt the onsite r4 LeRobot v2.1 capture to existing offline training tools.

The source dataset is already aligned at 10 Hz, but its live-writer sidecar is
not the raw-episode schema consumed by training_bridge.prepare_candidate.
This adapter preserves that source package, builds the existing converter's
preview/proxy inputs, and reuses the pinned LeRobot converter and strict
derived-state contract. It never trains or opens a robot interface.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import inspect
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Sequence

from laptop.recorder.next_state_proxy import SEMANTICS, audit_transition_deltas
from server.dataset_tools import convert_next_state_proxy
from server.training import proxy_multi_episode_contract


ROOT = Path(__file__).resolve().parents[2]
BRIDGE_ROOT = ROOT / "cache" / "training_bridge"
DEFAULT_SOURCE = ROOT / "gamepad_capture" / "data" / "lerobot" / "onsite_gamepad_20261019_r4"
SOURCE_RUN_ID = "20261019T152215Z_1986d33d"
FPS = 10
PERIOD_NS = 100_000_000
ACTUAL_COMMAND_SEMANTICS = (
    "matched actual send/receipt evidence; missing evidence remains invalid"
)
STATE_NAMES = [
    "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad",
    "joint_5_rad", "joint_6_rad", "gripper_nominal_m",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _tree_sha256(root: Path) -> str:
    root = Path(root).resolve(strict=True)
    entries = []
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"source tree contains a symlink: {path}")
        if path.is_file():
            entries.append((path.relative_to(root).as_posix(), path))
    digest = hashlib.sha256()
    for relative, path in sorted(entries):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\n")
    if not entries:
        raise ValueError("source dataset tree is empty")
    return digest.hexdigest()


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return value


def _jsonl(path: Path) -> tuple[list[dict[str, Any]], list[bytes]]:
    rows: list[dict[str, Any]] = []
    raw_lines: list[bytes] = []
    with Path(path).open("rb") as stream:
        for line_number, raw in enumerate(stream, 1):
            if not raw.strip():
                continue
            row = json.loads(raw)
            if not isinstance(row, dict):
                raise ValueError(f"expected an object at {path}:{line_number}")
            rows.append(row)
            raw_lines.append(raw)
    if not rows:
        raise ValueError(f"empty JSONL: {path}")
    return rows, raw_lines


def _jsonl_bytes(rows: Sequence[dict[str, Any]]) -> bytes:
    return b"".join(
        (json.dumps(row, sort_keys=True, separators=(",", ":"),
                    allow_nan=False) + "\n").encode("utf-8")
        for row in rows
    )


def _seven(value: object, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise ValueError(f"{name} must be a 7D sequence")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


def audit_source_rows(data_rows: Sequence[dict[str, Any]],
                      source_rows: Sequence[dict[str, Any]],
                      *, info: dict[str, Any]) -> dict[str, Any]:
    """Check the v2.1 table and old live-writer map agree without relabeling."""
    if len(data_rows) != len(source_rows) or len(data_rows) < 2:
        raise ValueError("source parquet and live source map need the same 2+ rows")
    if info.get("codebase_version") != "v2.1" or info.get("fps") != FPS:
        raise ValueError("r4 adapter requires the existing 10 Hz LeRobot v2.1 dataset")
    if (info.get("robot_type") != "piper" or info.get("total_episodes") != 1
            or info.get("total_frames") != len(data_rows)):
        raise ValueError("r4 LeRobot metadata does not match its single episode")
    for key in ("observation.state", "action"):
        if info.get("features", {}).get(key, {}).get("names") != STATE_NAMES:
            raise ValueError(f"unexpected v2.1 feature names for {key}")

    action_valid_count = 0
    observation_valid_count = 0
    camera_valid_count = 0
    prior_timestamp = None
    for index, (data, source) in enumerate(zip(data_rows, source_rows, strict=True)):
        if (data.get("index") != index or data.get("episode_index") != 0
                or data.get("frame_index") != index
                or source.get("dataset_row_index") != index
                or source.get("episode_index") != 0
                or source.get("episode_frame_index") != index
                or source.get("run_id") != SOURCE_RUN_ID):
            raise ValueError(f"dataset/source row identity mismatch at {index}")
        state = _seven(data.get("observation.state"), f"state row {index}")
        action = _seven(data.get("action"), f"action row {index}")
        source_state = _seven(source.get("state"), f"source state row {index}")
        source_action = _seven(source.get("action"), f"source action row {index}")
        if state != source_state or action != source_action:
            raise ValueError(f"parquet values differ from original source map at row {index}")

        action_valid = data.get("action_valid")
        observation_valid = data.get("observation_valid")
        camera_valid = data.get("camera_valid")
        per_camera = source.get("camera_valid")
        if type(action_valid) is not bool or type(observation_valid) is not bool:
            raise ValueError(f"validity masks are not booleans at row {index}")
        if type(camera_valid) is not bool or not isinstance(per_camera, list) or len(per_camera) != 2:
            raise ValueError(f"camera validity masks are malformed at row {index}")
        if (source.get("action_valid") is not action_valid
                or source.get("observation_valid") is not observation_valid
                or source.get("dataset_camera_valid") is not camera_valid
                or per_camera != [camera_valid, camera_valid]):
            raise ValueError(f"parquet/source validity masks differ at row {index}")
        nested = source.get("source_map", {})
        nested_action = nested.get("action", {}) if isinstance(nested, dict) else {}
        if (source.get("action_semantics") != ACTUAL_COMMAND_SEMANTICS
                or nested_action.get("action_valid") is not action_valid):
            raise ValueError(f"nested actual-command mask differs at row {index}")
        if nested.get("camera_pair_valid") is not camera_valid:
            raise ValueError(f"source camera-pair mask differs at row {index}")

        timestamp = float(data.get("timestamp"))
        if not math.isfinite(timestamp):
            raise ValueError(f"non-finite dataset timestamp at row {index}")
        if prior_timestamp is not None and abs((timestamp - prior_timestamp) - 0.1) > 2e-5:
            raise ValueError(f"dataset timestamp is not a 10 Hz grid at row {index}")
        prior_timestamp = timestamp
        action_valid_count += action_valid
        observation_valid_count += observation_valid
        camera_valid_count += camera_valid

    return {
        "frames": len(data_rows),
        "fps": FPS,
        "action_valid_rows": action_valid_count,
        "action_masked_rows": len(data_rows) - action_valid_count,
        "observation_valid_rows": observation_valid_count,
        "camera_valid_rows": camera_valid_count,
        "camera_pair_valid_rows": sum(row.get("camera_valid") == [True, True]
                                      for row in source_rows),
        "actual_command_semantics": ACTUAL_COMMAND_SEMANTICS,
        "state_and_actual_action_feature_names": STATE_NAMES,
        "state_units": ["radian_nominal_unverified"] * 6 + ["meter_nominal_unverified"],
        "actual_command_units": ["radian_nominal_unverified"] * 6 + ["meter_nominal_unverified"],
    }


def build_server_training_plan(*, run_id: str, repo_id: str,
                              dataset_sha256: str,
                              local_dataset_path: Path) -> dict[str, str]:
    """Render exact server paths with POSIX separators without contacting it."""
    server_root = "/mnt/ssd/users/mingkai/07_pi0.5"
    server_run = f"{server_root}/cache/codex/{run_id}_server"
    server_home = f"{server_run}/datasets"
    server_dataset = f"{server_home}/{repo_id}"
    server_report = f"{server_run}/r4_tinyfit_training_report.json"
    ssh_alias = "rtx6000_Mingkai"
    stage_command = (
        f"ssh {ssh_alias} 'mkdir -p {server_home}/local' && "
        f"scp -r '{Path(local_dataset_path).resolve().as_posix()}' "
        f"{ssh_alias}:{server_home}/local/"
    )
    train_command = "\n".join((
        f"cd {server_root}",
        "CUDA_VISIBLE_DEVICES=1 \\",
        f"HF_LEROBOT_HOME={server_home} \\",
        ".venv/bin/python -B -m server.training.post_capture_pipeline train \\",
        f"  --dataset-path {server_dataset} \\",
        "  --min-episodes 1 \\",
        "  --steps 20 \\",
        "  --gpu-index 1 \\",
        f"  --base-params-path {server_root}/cache/openpi/openpi-assets/checkpoints/pi05_base/params \\",
        f"  --expected-dataset-sha256 {dataset_sha256} \\",
        f"  --report-path {server_report}",
    ))
    return {
        "server_root": server_root,
        "server_run_path": server_run,
        "server_dataset_staging_path": server_dataset,
        "stage_proxy_dataset_command": stage_command,
        "training_command": train_command,
    }


def make_frame_rows(data_rows: Sequence[dict[str, Any]],
                    source_rows: Sequence[dict[str, Any]],
                    source_line_bytes: Sequence[bytes], *,
                    source_dataset_sha256: str,
                    source_map_sha256: str) -> list[dict[str, Any]]:
    """Build existing aligned-preview rows and retain actual-command masks."""
    if not (len(data_rows) == len(source_rows) == len(source_line_bytes)):
        raise ValueError("frame/source-map inputs have different lengths")
    frames = []
    for index, (data, source, raw_line) in enumerate(
            zip(data_rows, source_rows, source_line_bytes, strict=True)):
        nested = source.get("source_map", {})
        state_source = nested.get("state", {}) if isinstance(nested, dict) else {}
        image_sources = nested.get("images", {}) if isinstance(nested, dict) else {}
        action_source = nested.get("action", {}) if isinstance(nested, dict) else {}
        command_values = _seven(data["action"], f"actual action row {index}")
        command_valid = data["action_valid"] is True
        refs = {
            "source_kind": "existing_lerobot_v2.1_row",
            "source_dataset_row_index": index,
            "source_dataset_timestamp_s": float(data["timestamp"]),
            "source_dataset_sha256": source_dataset_sha256,
            "source_map_path": "provenance/source_map.jsonl",
            "source_map_sha256": source_map_sha256,
            "source_map_row_index": index,
            "source_map_row_sha256": hashlib.sha256(raw_line).hexdigest(),
            "state_source": {
                "sequence": state_source.get("sequence"),
                "source_monotonic_ns": state_source.get("source_monotonic_ns"),
                "source_clock_id": state_source.get("source_clock_id"),
                "units": ["radian_nominal_unverified"] * 6 + ["meter_nominal_unverified"],
            },
            "images_source": {
                role: {
                    "sequence": image_sources.get(role, {}).get("sequence"),
                    "source_monotonic_ns": image_sources.get(role, {}).get("source_monotonic_ns"),
                    "source_clock_id": image_sources.get(role, {}).get("source_clock_id"),
                    "pre_encode_rgb_sha256": image_sources.get(role, {}).get("rgb_pixels_sha256"),
                    "valid": image_sources.get(role, {}).get("valid"),
                }
                for role in ("base", "wrist")
            },
            "actual_command": {
                "semantics": source.get("action_semantics"),
                "mask_valid": command_valid,
                "values": command_values if command_valid else None,
                "masked_storage_values": None if command_valid else command_values,
                "request_id": action_source.get("request_id"),
                "invalid_reason": action_source.get("invalid_reason"),
                "source_state_sequence": action_source.get("source_state_sequence"),
            },
        }
        frames.append({
            "frame_index": index,
            "policy_timestamp_ns": index * PERIOD_NS,
            "base_image": f"base_{index:06d}.png",
            "wrist_image": f"wrist_{index:06d}.png",
            "state": _seven(data["observation.state"], f"state row {index}"),
            "source_refs": refs,
        })
    return frames


def make_proxy_rows(frames: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    """Derive each proxy target only from the next observed feedback state."""
    if len(frames) < 3:
        raise ValueError("the existing proxy converter needs at least two transitions")
    rows = []
    for index, (current, following) in enumerate(zip(frames, frames[1:])):
        if (current.get("frame_index") != index
                or following.get("frame_index") != index + 1
                or following.get("policy_timestamp_ns", 0)
                - current.get("policy_timestamp_ns", 0) != PERIOD_NS):
            raise ValueError(f"proxy input is not a contiguous 10 Hz grid at row {index}")
        rows.append({
            "frame_index": index,
            "observation_timestamp_ns": current["policy_timestamp_ns"],
            "target_timestamp_ns": following["policy_timestamp_ns"],
            "base_image": current["base_image"],
            "wrist_image": current["wrist_image"],
            "state": list(current["state"]),
            "proxy_action": list(following["state"]),
            "target_source_frame_index": index + 1,
        })
    return rows


def _extract_video(video_path: Path, *, role: str, output: Path,
                   expected_frames: int, expected_width: int,
                   expected_height: int) -> dict[str, Any]:
    """Decode one source video sequentially; never opens a physical camera."""
    import av
    import numpy as np
    from PIL import Image

    output.mkdir(parents=True, exist_ok=True)
    container = av.open(str(video_path))
    try:
        if not container.streams.video:
            raise ValueError(f"{role} source has no video stream")
        stream = container.streams.video[0]
        fps = float(stream.average_rate) if stream.average_rate is not None else None
        if (stream.frames != expected_frames or fps is None
                or abs(fps - FPS) > 1e-6
                or stream.codec_context.width != expected_width
                or stream.codec_context.height != expected_height):
            raise ValueError(f"{role} video metadata differs from the v2.1 dataset")
        count = 0
        first_pts = None
        last_pts = None
        for frame in container.decode(stream):
            if count >= expected_frames:
                raise ValueError(f"{role} video decodes more frames than the dataset index")
            rgb = frame.to_ndarray(format="rgb24")
            if rgb.shape != (expected_height, expected_width, 3) or rgb.dtype != np.uint8:
                raise ValueError(f"{role} decoded frame shape/type differs at {count}")
            if first_pts is None:
                first_pts = frame.pts
            last_pts = frame.pts
            Image.fromarray(rgb, mode="RGB").save(output / f"{role}_{count:06d}.png")
            count += 1
        if count != expected_frames:
            raise ValueError(f"{role} decoded {count} frames; expected {expected_frames}")
        return {
            "video_path": str(video_path.resolve()),
            "video_sha256": _sha256(video_path),
            "decoded_frames": count,
            "fps": fps,
            "width": expected_width,
            "height": expected_height,
            "first_pts": first_pts,
            "last_pts": last_pts,
            "decoder": "PyAV sequential offline decode",
        }
    finally:
        container.close()


def _copy_actual_candidate(source: Path, target: Path, *,
                           source_dataset_sha256: str,
                           counts: dict[str, Any],
                           label_sha256: str,
                           acceptance_sha256: str,
                           source_map_sha256: str) -> dict[str, Any]:
    if target.exists() or target.is_relative_to(source) or source.is_relative_to(target):
        raise FileExistsError(f"actual-command review copy path is not unused/separate: {target}")
    shutil.copytree(source, target, copy_function=shutil.copy2)
    copied_sha = _tree_sha256(target)
    if copied_sha != source_dataset_sha256:
        raise ValueError("review copy does not match the source v2.1 dataset tree")
    descriptor = {
        "schema": "piper_r4_actual_command_review_candidate_v1",
        "candidate_kind": "immutable_v2.1_copy_for_actual_command_review",
        "source_run_id": SOURCE_RUN_ID,
        "source_dataset_tree_sha256": source_dataset_sha256,
        "source_map_sha256": source_map_sha256,
        "run_labels_sha256": label_sha256,
        "acceptance_sha256": acceptance_sha256,
        "action_semantics": ACTUAL_COMMAND_SEMANTICS,
        "action_valid_rows": counts["action_valid_rows"],
        "action_masked_rows": counts["action_masked_rows"],
        "action_values_unchanged": True,
        "action_masks_unchanged": True,
        "observation_and_camera_masks_unchanged": True,
        "state_units": counts["state_units"],
        "actual_command_units": counts["actual_command_units"],
        "training_eligible_as_recorded": False,
        "formal_training_eligible": False,
        "training_started": False,
        "hardware_control_allowed": False,
        "note": (
            "Review-only copy. Masked action storage values remain byte-for-byte "
            "unchanged and are not command labels. This artifact is not a training dataset."
        ),
    }
    manifest_path = target / "provenance" / "r4_actual_command_candidate.json"
    manifest_path.write_text(json.dumps(descriptor, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return {
        "path": str(target.resolve()),
        "source_dataset_tree_sha256": source_dataset_sha256,
        "copied_source_tree_sha256_before_candidate_manifest": copied_sha,
        "candidate_manifest": str(manifest_path.resolve()),
        "candidate_manifest_sha256": _sha256(manifest_path),
        "action_semantics": ACTUAL_COMMAND_SEMANTICS,
        "action_valid_rows": counts["action_valid_rows"],
        "action_masked_rows": counts["action_masked_rows"],
        "formal_training_eligible": False,
    }


def prepare_r4_candidate(*, run_id: str,
                         source_dataset: Path = DEFAULT_SOURCE) -> dict[str, Any]:
    import re
    import pyarrow.parquet as pq

    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,47}", run_id) is None:
        raise ValueError("run_id must be a safe 3-48 character directory name")
    source = Path(source_dataset).resolve(strict=True)
    if source != DEFAULT_SOURCE.resolve():
        raise ValueError("this adapter is pinned to the reviewed r4 source dataset")
    if not (source / "meta" / "info.json").is_file():
        raise FileNotFoundError(f"v2.1 dataset metadata is missing under {source}")
    output_root = BRIDGE_ROOT / "r4_candidates" / run_id
    if output_root.exists():
        raise FileExistsError(f"refusing to overwrite r4 training candidate: {output_root}")
    info = _json(source / "meta" / "info.json")
    identity = _json(source / "provenance" / "dataset_identity.json")
    source_rows, source_lines = _jsonl(source / "provenance" / "source_map.jsonl")
    labels, _ = _jsonl(source / "provenance" / "run_labels.jsonl")
    acceptances = list((source / "provenance").glob("*_acceptance.json"))
    if len(acceptances) != 1:
        raise ValueError("expected exactly one immutable r4 acceptance record")
    acceptance_path = acceptances[0]
    acceptance = _json(acceptance_path)
    label = next((row for row in labels
                  if row.get("run_id") == SOURCE_RUN_ID and row.get("episode_index") == 0), None)
    if label is None:
        raise ValueError("the source run has no matching episode outcome label")
    if (identity.get("format") != "LeRobot v2.1"
            or identity.get("repo_id") != f"piper/live_{SOURCE_RUN_ID}"
            or label.get("demonstration_label") != "SUCCESS"
            or label.get("outcome") != "success"
            or label.get("success_is_operator_assertion") is not True
            or label.get("training_eligible") is not False
            or label.get("physical_stop_verified") is not False
            or acceptance.get("training_eligible") is not False
            or acceptance.get("quality_pass") is not False
            or acceptance.get("supervision_complete") is not False):
        raise ValueError("r4 source label/acceptance differs from the reviewed evidence")

    parquet_files = sorted((source / "data").glob("chunk-*/episode_*.parquet"))
    if len(parquet_files) != 1:
        raise ValueError("r4 source must contain exactly one episode Parquet file")
    table = pq.read_table(parquet_files[0])
    data_rows = table.to_pylist()
    counts = audit_source_rows(data_rows, source_rows, info=info)
    if (counts["frames"] != 931 or counts["action_valid_rows"] != 603
            or counts["action_masked_rows"] != 328
            or counts["observation_valid_rows"] != 931
            or counts["camera_valid_rows"] != 931
            or counts["camera_pair_valid_rows"] != 931):
        raise ValueError("r4 source counts differ from the fixed capture acceptance record")
    if (acceptance.get("frames") != counts["frames"]
            or acceptance.get("action_valid_frames") != counts["action_valid_rows"]
            or acceptance.get("observation_valid_frames") != counts["observation_valid_rows"]
            or acceptance.get("camera_valid_frames") != counts["camera_valid_rows"]):
        raise ValueError("r4 acceptance counters differ from full row audit")

    source_dataset_sha = _tree_sha256(source)
    source_map_path = source / "provenance" / "source_map.jsonl"
    source_map_sha = _sha256(source_map_path)
    label_path = source / "provenance" / "run_labels.jsonl"
    label_sha = _sha256(label_path)
    acceptance_sha = _sha256(acceptance_path)
    output_root.mkdir(parents=True)
    actual_copy = _copy_actual_candidate(
        source, output_root / "actual_command_review_copy",
        source_dataset_sha256=source_dataset_sha, counts=counts,
        label_sha256=label_sha, acceptance_sha256=acceptance_sha,
        source_map_sha256=source_map_sha,
    )

    adapter_manifest_path = output_root / "r4_source_adapter_manifest.json"
    adapter_manifest = {
        "schema": "piper_r4_lerobot_v2_1_adapter_manifest_v1",
        "source_kind": "existing_real_lerobot_v2.1_capture_dataset",
        "source_run_id": SOURCE_RUN_ID,
        "source_dataset_path": str(source),
        "source_dataset_tree_sha256": source_dataset_sha,
        "dataset_identity_sha256": _sha256(source / "provenance" / "dataset_identity.json"),
        "source_map_sha256": source_map_sha,
        "run_labels_sha256": label_sha,
        "acceptance_sha256": acceptance_sha,
        "source_format": identity["format"],
        "source_frames": counts["frames"],
        "proxy_transitions": counts["frames"] - 1,
        "action_semantics_as_recorded": ACTUAL_COMMAND_SEMANTICS,
        "action_valid_rows": counts["action_valid_rows"],
        "action_masked_rows": counts["action_masked_rows"],
        "training_eligible_as_recorded": False,
        "formal_training_eligible": False,
        "compatibility_field_note": (
            "The existing proxy converter requires a field named raw_manifest_sha256. "
            "For this already-aligned LeRobot input, that field is bound to this explicit "
            "adapter-manifest hash; it does not claim that the source is a piper_raw_manifest_v1."
        ),
    }
    adapter_manifest_path.write_text(
        json.dumps(adapter_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    adapter_manifest_sha = _sha256(adapter_manifest_path)

    run_root = output_root / "derived"
    preview = run_root / "previews" / SOURCE_RUN_ID
    proxy = run_root / "targets" / SOURCE_RUN_ID
    if preview.exists() or proxy.exists():
        raise FileExistsError("r4 preview or target output already exists")
    preview.mkdir(parents=True)
    image_root = preview
    features = info.get("features", {})
    base_shape = features.get("observation.images.base", {}).get("shape")
    wrist_shape = features.get("observation.images.wrist", {}).get("shape")
    if (base_shape != [3, 480, 640] or wrist_shape != [3, 480, 640]):
        raise ValueError("r4 source camera features differ from 640x480 RGB")
    base_video = source / "videos" / "chunk-000" / "observation.images.base" / "episode_000000.mp4"
    wrist_video = source / "videos" / "chunk-000" / "observation.images.wrist" / "episode_000000.mp4"
    video_audit = {
        "base": _extract_video(base_video, role="base", output=image_root,
                               expected_frames=counts["frames"],
                               expected_width=640, expected_height=480),
        "wrist": _extract_video(wrist_video, role="wrist", output=image_root,
                                expected_frames=counts["frames"],
                                expected_width=640, expected_height=480),
    }
    frames = make_frame_rows(
        data_rows, source_rows, source_lines,
        source_dataset_sha256=source_dataset_sha,
        source_map_sha256=source_map_sha,
    )
    frames_path = preview / "frames.jsonl"
    frames_path.write_bytes(_jsonl_bytes(frames))
    frame_map_sha = _sha256(frames_path)
    preview_meta = {
        "schema": "piper_aligned_preview_v1",
        "episode_id": SOURCE_RUN_ID,
        "raw_episode_id": SOURCE_RUN_ID,
        "raw_manifest_sha256": adapter_manifest_sha,
        "source": "operator_demonstration",
        "success": True,
        "operator_success_is_assertion": True,
        "fps": FPS,
        "frame_count": counts["frames"],
        "frame_map_sha256": frame_map_sha,
        "task_id": "pick_place_black_rod_green_mat_v1",
        "task": "Pick up the black rod and place it on the green desk mat.",
        "timestamp_kind": "source LeRobot v2.1 10 Hz dataset grid; not capture monotonic time",
        "camera_pairing_semantics": "source host-after-read pair; not exposure synchronization",
        "camera_sync_verified": False,
        "joint_units_verified": False,
        "gripper_units_verified": False,
        "action_target_verified": False,
        "training_eligible": False,
        "action_semantics_as_recorded": ACTUAL_COMMAND_SEMANTICS,
        "source_dataset_sha256": source_dataset_sha,
        "source_map_sha256": source_map_sha,
        "run_labels_sha256": label_sha,
        "acceptance_sha256": acceptance_sha,
        "full_task_success_inferred_from_fragment": False,
        "frame_selection": "all 931 existing rows, no resampling or row filtering",
    }
    metadata_path = preview / "aligned_episode.json"
    metadata_path.write_text(json.dumps(preview_meta, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")

    proxy_rows = make_proxy_rows(frames)
    proxy.mkdir(parents=True)
    proxy_targets_path = proxy / "proxy_targets.jsonl"
    proxy_targets_path.write_bytes(_jsonl_bytes(proxy_rows))
    from laptop.recorder.next_state_proxy import GRIPPER_METERS_PER_RAW_NOMINAL
    proxy_manifest = {
        "schema": "piper_next_state_proxy_v1",
        "semantics": SEMANTICS,
        "raw_episode_id": SOURCE_RUN_ID,
        "preview_episode_id": SOURCE_RUN_ID,
        "raw_manifest_sha256": adapter_manifest_sha,
        "source_manifest_kind": "piper_r4_lerobot_v2_1_adapter_manifest_v1",
        "preview_metadata_sha256": _sha256(metadata_path),
        "preview_frames_sha256": _sha256(frames_path),
        "outcome_label_sha256": label_sha,
        "outcome_success": True,
        "outcome_is_operator_assertion": True,
        "proxy_targets_sha256": _sha256(proxy_targets_path),
        "samples": len(proxy_rows),
        "fps": FPS,
        "target_offset_ns": PERIOD_NS,
        "joint_unit": "radian_nominal_unverified",
        "gripper_unit": "meter_nominal_unverified",
        "gripper_meters_per_raw_nominal": GRIPPER_METERS_PER_RAW_NOMINAL,
        "source_dataset_sha256": source_dataset_sha,
        "source_map_sha256": source_map_sha,
        "source_action_valid_rows": counts["action_valid_rows"],
        "source_action_masked_rows": counts["action_masked_rows"],
        "source_action_masks_preserved_in_source_refs": True,
        "transition_delta_audit": audit_transition_deltas(proxy_rows),
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
        "note": (
            "Offline next-observed-state proxy derived from consecutive r4 feedback rows. "
            "It is not an actual-command label; nominal units remain unverified."
        ),
    }
    proxy_manifest_path = proxy / "proxy_manifest.json"
    proxy_manifest_path.write_text(json.dumps(proxy_manifest, indent=2, sort_keys=True) + "\n",
                                   encoding="utf-8")

    repo_id = f"local/{run_id}_proxy"
    dataset_home = output_root / "datasets"
    dataset_path = dataset_home / repo_id
    if dataset_path.exists():
        raise FileExistsError(f"refusing to overwrite converted dataset: {dataset_path}")
    conversion = convert_next_state_proxy.convert_many(
        [(proxy, preview)], dataset_path, repo_id)
    dataset_sha = proxy_multi_episode_contract.dataset_tree_sha256(dataset_path)
    strict = proxy_multi_episode_contract.inspect_proxy_multi_episode(
        dataset_path, min_episodes=1, expected_dataset_sha256=dataset_sha)
    if (strict.get("formal_training_eligible") is not False
            or strict.get("hardware_control_allowed") is not False
            or strict.get("action_semantics") != SEMANTICS):
        raise ValueError("strict derived-state contract returned an unexpected admission state")

    default_min = inspect.signature(
        proxy_multi_episode_contract.inspect_proxy_multi_episode
    ).parameters["min_episodes"].default
    local_base_params = (
        ROOT / "cache" / "openpi" / "openpi-assets" / "checkpoints"
        / "pi05_base" / "params"
    )
    local_training_blockers = []
    try:
        importlib.metadata.version("jax")
    except importlib.metadata.PackageNotFoundError:
        local_training_blockers.append("JAX is absent from the local LeRobot 0.1.0 environment")
    if not local_base_params.is_dir():
        local_training_blockers.append(f"local pi05_base params are absent: {local_base_params}")

    server_plan = build_server_training_plan(
        run_id=run_id,
        repo_id=repo_id,
        dataset_sha256=dataset_sha,
        local_dataset_path=dataset_path,
    )
    report = {
        "schema": "piper_r4_training_bridge_report_v1",
        "status": "PROXY_CONVERSION_AND_STRICT_SOFTWARE_AUDIT_PASS_DIAGNOSTIC_ONLY",
        "run_id": run_id,
        "source": {
            "run_id": SOURCE_RUN_ID,
            "dataset_path": str(source),
            "dataset_identity_sha256": _sha256(source / "provenance" / "dataset_identity.json"),
            "dataset_tree_sha256": source_dataset_sha,
            "format": identity["format"],
            "runtime_recorded_at_capture": _jsonl(source / "provenance" / "run_sources.jsonl")[0][0].get("runtime"),
            "source_map_sha256": source_map_sha,
            "run_labels_sha256": label_sha,
            "acceptance_sha256": acceptance_sha,
            "operator_outcome": "SUCCESS",
            "success_is_operator_assertion": True,
            "training_eligible_as_recorded": False,
            "formal_training_eligible": False,
            "counters": counts,
            "acceptance_blockers_as_recorded": acceptance.get("training_blockers", []),
            "original_dataset_modified": False,
        },
        "actual_command_review_candidate": actual_copy,
        "adapter": {
            "manifest_path": str(adapter_manifest_path.resolve()),
            "manifest_sha256": adapter_manifest_sha,
            "compatibility_field_mapping": (
                "existing converter raw_manifest_sha256 field -> explicit adapter manifest hash; "
                "not an original raw episode manifest"
            ),
            "source_frames_decoded": counts["frames"],
            "videos": video_audit,
        },
        "proxy_candidate": {
            "kind": "next_observed_state_proxy_nominal_units",
            "dataset_path": str(dataset_path.resolve()),
            "repo_id": repo_id,
            "dataset_sha256": dataset_sha,
            "episodes": 1,
            "frames": len(proxy_rows),
            "fps": FPS,
            "converter": conversion,
            "strict_contract": strict,
            "strict_contract_min_episodes_applied": 1,
            "strict_contract_default_min_episodes": default_min,
            "readback": {
                "status": "PASS_READBACK",
                "audit_path": str((dataset_path / "provenance" / "readback_audit.json").resolve()),
                "audit_sha256": _sha256(dataset_path / "provenance" / "readback_audit.json"),
                "frames": len(proxy_rows),
                "decoded_rgb_frames_verified": True,
            },
            "actual_command_masks_retained_in_proxy_source_refs": True,
            "actual_command_is_training_target": False,
            "formal_training_eligible": False,
            "generalization_eligible": False,
        },
        "training": {
            "entrypoint": "server.training.post_capture_pipeline train",
            "diagnostic_tinyfit_candidate": True,
            "formal_training_eligible": False,
            "training_started": False,
            "model_loaded": False,
            "hardware_control_allowed": False,
            "local_training_blockers": local_training_blockers,
            "server_root_from_current_runbook": server_plan["server_root"],
            "server_run_path": server_plan["server_run_path"],
            "server_dataset_staging_path": server_plan["server_dataset_staging_path"],
            "stage_proxy_dataset_command": server_plan["stage_proxy_dataset_command"],
            "command": server_plan["training_command"],
            "steps": 20,
            "min_episodes": 1,
            "gpu_index": 1,
            "expected_dataset_sha256": dataset_sha,
            "operational_note": (
                "Command is prepared only. Stage this local candidate on the server and "
                "check GPU 1 availability before an operator starts it."
            ),
        },
        "limitations": [
            "The source operator SUCCESS label is an assertion and does not verify robot execution or physical stop.",
            "The original source records supervision_complete=false, quality_pass=false, and training_eligible=false.",
            "The proxy target is the next observed feedback state; it is not an actual command.",
            "Joint radians and gripper meters remain nominal and unverified for hardware.",
            "The one-episode min_episodes=1 override applies only to this diagnostic software audit.",
            "Formal multi-episode, field data admission, held-out generalization, and robot success remain unestablished.",
            "No training, model load, device access, CAN, camera, ROS session, motion, network install, or upload was run.",
        ],
    }
    report_path = output_root / "r4_training_bridge_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--source-dataset", type=Path, default=DEFAULT_SOURCE)
    args = parser.parse_args(argv)
    report = prepare_r4_candidate(run_id=args.run_id, source_dataset=args.source_dataset)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, PermissionError, ValueError,
            RuntimeError) as exc:
        print(f"r4 training bridge stopped: {exc}", file=sys.stderr)
        raise SystemExit(2)
