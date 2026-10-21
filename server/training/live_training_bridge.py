"""Derive a next-observed-state training dataset from one live capture.

The source LeRobot episode is never modified. Every derived row uses the current
seven-dimensional feedback state as observation and the feedback state 100 ms
later as the offline training action. CAN delivery evidence is retained as a
separate audit and never participates in the training-label mask.
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Sequence

from laptop.recorder.next_state_proxy import SEMANTICS, audit_transition_deltas
from server.dataset_tools import convert_next_state_proxy
from server.training import proxy_multi_episode_contract
from server.training.r4_training_bridge import (
    _extract_video,
    _json,
    _jsonl,
    _jsonl_bytes,
    _seven,
    _sha256,
    _tree_sha256,
)


ROOT = Path(__file__).resolve().parents[2]
FPS = 10
PERIOD_NS = 100_000_000
STATE_NAMES = [
    "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad",
    "joint_5_rad", "joint_6_rad", "gripper_nominal_m",
]
COMMAND_SEMANTICS = (
    "matched actual send/receipt evidence; missing evidence remains invalid"
)


def _first_json(source: Path) -> tuple[dict[str, Any], bytes]:
    with source.open("rb") as stream:
        for raw in stream:
            if raw.strip():
                row = json.loads(raw)
                if not isinstance(row, dict):
                    raise ValueError(f"expected a JSON object at {source}:1")
                return row, raw
    raise ValueError(f"empty JSONL: {source}")


def _episode_file(source: Path) -> tuple[Path, int, int]:
    paths = sorted(source.glob("data/chunk-*/episode_*.parquet"))
    if len(paths) != 1:
        raise ValueError("live bridge currently requires exactly one source episode")
    path = paths[0]
    chunk_match = re.fullmatch(r"chunk-(\d+)", path.parent.name)
    episode_match = re.fullmatch(r"episode_(\d+)\.parquet", path.name)
    if chunk_match is None or episode_match is None:
        raise ValueError("source parquet path is not a LeRobot v2.1 episode path")
    return path, int(chunk_match.group(1)), int(episode_match.group(1))


def _video_path(source: Path, info: dict[str, Any], role: str, *,
                chunk: int, episode: int) -> Path:
    template = info.get("video_path")
    if not isinstance(template, str):
        raise ValueError("LeRobot info.json has no video_path template")
    relative = template.format(episode_chunk=chunk, episode_index=episode,
                               video_key=f"observation.images.{role}")
    path = (source / relative).resolve()
    if not path.is_relative_to(source.resolve()) or not path.is_file():
        raise FileNotFoundError(f"missing {role} source video: {relative}")
    return path


def _bool(value: object) -> bool:
    return value is True


def _command_row_valid(row: dict[str, Any]) -> bool | None:
    if type(row.get("command_evidence_valid")) is bool:
        return bool(row["command_evidence_valid"])
    evidence = row.get("command_evidence")
    if isinstance(evidence, dict) and type(evidence.get("valid")) is bool:
        return bool(evidence["valid"])
    nested = row.get("source_map")
    action = nested.get("action") if isinstance(nested, dict) else None
    if isinstance(action, dict) and type(action.get("action_valid")) is bool:
        return bool(action["action_valid"])
    semantics = row.get("action_semantics")
    if isinstance(semantics, str) and (
        "send/receipt" in semantics or "kernel_accepted" in semantics
        or "kernel accepted" in semantics
    ):
        value = row.get("action_valid")
        return value if type(value) is bool else None
    return None


def command_evidence_summary(rows: Sequence[dict[str, Any]],
                             label: dict[str, Any] | None = None,
                             metadata: dict[str, Any] | None = None) -> dict[str, Any]:
    """Summarize CAN evidence without reading or changing training actions."""
    supplied = None
    if isinstance(metadata, dict):
        supplied = metadata.get("command_evidence")
    if supplied is None and isinstance(label, dict):
        supplied = label.get("command_evidence")
    if isinstance(supplied, dict):
        valid = supplied.get("valid") is True or supplied.get("command_evidence_valid") is True
        status = supplied.get("status")
        if status not in {"COMPLETE", "PARTIAL", "MISSING", "UNKNOWN"}:
            status = "COMPLETE" if valid else "PARTIAL"
        blockers = supplied.get("blockers", supplied.get("reasons", []))
        normalized_blockers = (
            [str(item) for item in blockers] if isinstance(blockers, list) else []
        )
        if status != "COMPLETE" and not normalized_blockers:
            normalized_blockers = ["source_command_evidence_incomplete"]
        return {
            "status": status,
            "valid": status == "COMPLETE",
            "source": "explicit",
            "accepted_command_rows": supplied.get("accepted_command_rows"),
            "blockers": normalized_blockers,
        }

    evidence = [_command_row_valid(row) for row in rows]
    known = [value for value in evidence if value is not None]
    if not known:
        return {
            "status": "UNKNOWN", "valid": False, "source": "row_audit",
            "accepted_command_rows": 0,
            "blockers": ["no_explicit_command_evidence_in_source_rows"],
        }
    accepted = sum(known)
    if accepted == len(rows) and len(known) == len(rows):
        status = "COMPLETE"
    elif accepted:
        status = "PARTIAL"
    else:
        status = "MISSING"
    blockers = [] if status == "COMPLETE" else ["source_command_evidence_incomplete"]
    return {
        "status": status, "valid": status == "COMPLETE", "source": "row_audit",
        "accepted_command_rows": accepted,
        "known_command_rows": len(known),
        "blockers": blockers,
    }


def audit_source_rows(data_rows: Sequence[dict[str, Any]],
                      source_rows: Sequence[dict[str, Any]],
                      *, info: dict[str, Any], source_run_id: str,
                      task: str) -> dict[str, Any]:
    """Validate a 10 Hz single episode and keep command evidence independent."""
    if len(data_rows) != len(source_rows) or len(data_rows) < 3:
        raise ValueError("source parquet and source map need the same 3+ rows")
    if (info.get("codebase_version") != "v2.1" or info.get("fps") != FPS
            or info.get("robot_type") != "piper" or info.get("total_episodes") != 1
            or info.get("total_frames") != len(data_rows)):
        raise ValueError("source metadata is not a supported one-episode 10 Hz PiPER v2.1 dataset")
    for key in ("observation.state", "action"):
        if info.get("features", {}).get(key, {}).get("names") != STATE_NAMES:
            raise ValueError(f"unexpected source feature names for {key}")

    observation_valid_count = 0
    camera_valid_count = 0
    command_rows: list[dict[str, Any]] = []
    previous_timestamp: float | None = None
    for index, (data, source) in enumerate(zip(data_rows, source_rows, strict=True)):
        if (data.get("index") != index or data.get("episode_index") != 0
                or data.get("frame_index") != index
                or source.get("dataset_row_index") != index
                or source.get("episode_index") != 0
                or source.get("episode_frame_index") != index
                or source.get("run_id") != source_run_id
                or source.get("task") != task):
            raise ValueError(f"source row identity/task mismatch at {index}")
        state = _seven(data.get("observation.state"), f"state row {index}")
        action = _seven(data.get("action"), f"action row {index}")
        if state != _seven(source.get("state"), f"source state row {index}"):
            raise ValueError(f"source state differs from parquet at {index}")
        if action != _seven(source.get("action"), f"source action row {index}"):
            raise ValueError(f"source action differs from parquet at {index}")
        observation_valid = data.get("observation_valid")
        camera_valid = data.get("camera_valid")
        per_camera = source.get("camera_valid")
        if type(observation_valid) is not bool or type(camera_valid) is not bool:
            raise ValueError(f"validity masks are not booleans at row {index}")
        if (per_camera != [True, True] or source.get("dataset_camera_valid") is not True):
            raise ValueError(f"source camera pair is incomplete at row {index}")
        nested = source.get("source_map")
        if not isinstance(nested, dict):
            raise ValueError(f"source map payload is missing at row {index}")
        if nested.get("observation_valid") is not True or nested.get("camera_pair_valid") is not True:
            raise ValueError(f"nested observation/camera pair is invalid at row {index}")
        images = nested.get("images")
        if not isinstance(images, dict) or any(
                not isinstance(images.get(role), dict)
                or images[role].get("valid") is not True
                or images[role].get("width") != 640
                or images[role].get("height") != 480
                for role in ("base", "wrist")):
            raise ValueError(f"source image provenance is incomplete at row {index}")
        timestamp = float(data.get("timestamp"))
        if not math.isfinite(timestamp):
            raise ValueError(f"non-finite dataset timestamp at row {index}")
        if previous_timestamp is not None and abs((timestamp - previous_timestamp) - 0.1) > 2e-5:
            raise ValueError(f"source timestamps are not a contiguous 10 Hz grid at row {index}")
        previous_timestamp = timestamp
        observation_valid_count += int(observation_valid)
        camera_valid_count += int(camera_valid)
        command_rows.append(source)

    if observation_valid_count != len(data_rows) or camera_valid_count != len(data_rows):
        raise ValueError("all source observation and camera rows must be valid")
    return {
        "frames": len(data_rows),
        "fps": FPS,
        "observation_valid_rows": observation_valid_count,
        "camera_valid_rows": camera_valid_count,
        "source_timestamp_grid_exact_100ms": True,
        "source_command_evidence": command_evidence_summary(command_rows),
    }


def make_frame_rows(data_rows: Sequence[dict[str, Any]],
                    source_rows: Sequence[dict[str, Any]],
                    source_line_bytes: Sequence[bytes], *,
                    source_dataset_sha256: str,
                    source_map_sha256: str,
                    source_run_id: str) -> list[dict[str, Any]]:
    if not (len(data_rows) == len(source_rows) == len(source_line_bytes)):
        raise ValueError("frame/source-map inputs have different lengths")
    frames: list[dict[str, Any]] = []
    for index, (data, source, raw_line) in enumerate(
            zip(data_rows, source_rows, source_line_bytes, strict=True)):
        nested = source.get("source_map")
        if not isinstance(nested, dict):
            raise ValueError(f"missing nested source map at row {index}")
        state_source = nested.get("state", {})
        image_sources = nested.get("images", {})
        action_source = nested.get("action", {})
        command_valid = _command_row_valid(source)
        command_values = _seven(data.get("action"), f"source action row {index}")
        refs = {
            "source_kind": "live_lerobot_v2.1_row",
            "source_run_id": source_run_id,
            "source_dataset_row_index": index,
            "source_dataset_timestamp_s": float(data["timestamp"]),
            "source_dataset_sha256": source_dataset_sha256,
            "source_map_path": "provenance/source_map.jsonl",
            "source_map_sha256": source_map_sha256,
            "source_map_row_index": index,
            "source_map_row_sha256": hashlib.sha256(raw_line).hexdigest(),
            "base_frame_id": index,
            "wrist_frame_id": index,
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
                    "state_delta_ns": image_sources.get(role, {}).get("state_delta_ns"),
                    "valid": image_sources.get(role, {}).get("valid"),
                }
                for role in ("base", "wrist")
            },
            "actual_command": {
                "semantics": source.get("action_semantics"),
                "mask_valid": command_valid,
                "values": command_values if command_valid is True else None,
                "masked_storage_values": command_values if command_valid is not True else None,
                "request_id": action_source.get("request_id") if isinstance(action_source, dict) else None,
                "invalid_reason": action_source.get("invalid_reason") if isinstance(action_source, dict) else None,
                "source_state_sequence": (
                    action_source.get("source_state_sequence") if isinstance(action_source, dict) else None
                ),
            },
            "training_action_semantics": SEMANTICS,
        }
        frames.append({
            "frame_index": index,
            "policy_timestamp_ns": index * PERIOD_NS,
            "base_image": f"base_{index:06d}.png",
            "wrist_image": f"wrist_{index:06d}.png",
            "state": _seven(data.get("observation.state"), f"state row {index}"),
            "source_refs": refs,
        })
    return frames


def make_proxy_rows(frames: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    """Create N-1 rows; the final source frame is never copied into an action."""
    if len(frames) < 3:
        raise ValueError("next-state derivation needs at least three source rows")
    rows: list[dict[str, Any]] = []
    for index, (current, following) in enumerate(zip(frames, frames[1:], strict=False)):
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
    if len(rows) != len(frames) - 1:
        raise ValueError("proxy action count must equal source frames minus one")
    return rows


def _acceptance_path(source: Path) -> Path:
    paths = sorted((source / "provenance").glob("*_acceptance.json"))
    if len(paths) != 1:
        raise ValueError("expected exactly one source acceptance record")
    return paths[0]


def prepare_live_candidate(*, source_dataset: Path,
                           output_dataset: Path | None = None,
                           stage_root: Path | None = None,
                           dry_run: bool = False) -> dict[str, Any]:
    import pyarrow.parquet as pq

    source = Path(source_dataset).resolve(strict=True)
    info = _json(source / "meta" / "info.json")
    parquet_path, chunk, episode_index = _episode_file(source)
    if episode_index != 0:
        raise ValueError("the live bridge currently accepts episode index zero")
    source_identity, _ = _first_json(source / "provenance" / "run_sources.jsonl")
    labels, _ = _jsonl(source / "provenance" / "run_labels.jsonl")
    if len(labels) != 1:
        raise ValueError("expected exactly one operator outcome label")
    label = labels[0]
    acceptance_path = _acceptance_path(source)
    acceptance = _json(acceptance_path)
    run_id = source_identity.get("run_id")
    task = source_identity.get("task")
    if (not isinstance(run_id, str)
            or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,95}", run_id) is None):
        raise ValueError("source run ID is missing or unsafe")
    if not isinstance(task, str) or not task.strip() or len(task) > 512:
        raise ValueError("source task label is missing")
    metadata = source_identity.get("metadata")
    metadata = metadata if isinstance(metadata, dict) else {}
    if metadata.get("task") not in (None, task):
        raise ValueError("source task identities conflict")
    if (label.get("run_id") != run_id or label.get("episode_index") != 0
            or label.get("human_labeled") is not True
            or label.get("label_source") not in {"human", "operator"}
            or label.get("outcome") not in {"success", "successful"}
            or label.get("successful_complete_end") is not True
            or label.get("complete_closeout_label_present") is not True
            or label.get("retake_requested") is True):
        raise ValueError("source episode lacks an explicit successful operator label")
    if (not acceptance_path.name.startswith(run_id + "_")
            or acceptance.get("frames") != info.get("total_frames")
            or acceptance.get("task_success") is not True
            or acceptance.get("timeline_complete") is not True
            or acceptance.get("observation_complete") is not True
            or acceptance.get("closeout_complete") is not True):
        raise ValueError("source acceptance record does not match the episode")

    data_rows = pq.read_table(parquet_path).to_pylist()
    source_rows, source_lines = _jsonl(source / "provenance" / "source_map.jsonl")
    source_dataset_sha = _tree_sha256(source)
    source_map_sha = _sha256(source / "provenance" / "source_map.jsonl")
    label_sha = _sha256(source / "provenance" / "run_labels.jsonl")
    acceptance_sha = _sha256(acceptance_path)
    audit = audit_source_rows(data_rows, source_rows, info=info,
                              source_run_id=run_id, task=task)
    expected_actions = len(data_rows) - 1
    if audit["frames"] != 577 and audit["frames"] < 3:
        raise ValueError("source frame count is invalid")

    output = (Path(output_dataset).resolve() if output_dataset is not None
              else ROOT / "gamepad_capture" / "data" / "lerobot" / "derived"
              / f"{source.name}_proxy100ms")
    staging = (Path(stage_root).resolve() if stage_root is not None
               else ROOT / "cache" / "training_bridge" / "live_runs" / run_id)
    for path in (output, staging):
        if path == source or path.is_relative_to(source) or source.is_relative_to(path):
            raise ValueError("source, staging, and output paths must be separate")
    if dry_run:
        return {
            "status": "PREFLIGHT_PASS_NO_WRITE",
            "source_dataset": str(source),
            "run_id": run_id,
            "task": task,
            "source_frames": len(data_rows),
            "expected_training_rows": expected_actions,
            "last_source_frame_used_only_as_future_state": True,
            "training_label": {
                "status": "COMPLETE", "semantics": SEMANTICS,
                "valid_rows": expected_actions, "total_rows": expected_actions,
            },
            "command_evidence": audit["source_command_evidence"],
            "command_evidence_blocks_training_label": False,
            "hardware_control_allowed": False,
        }
    if output.exists():
        raise FileExistsError(f"refusing to overwrite derived dataset: {output}")
    if staging.exists():
        raise FileExistsError(f"refusing to overwrite staging output: {staging}")

    staging.mkdir(parents=True)
    features = info.get("features", {})
    base_shape = features.get("observation.images.base", {}).get("shape")
    wrist_shape = features.get("observation.images.wrist", {}).get("shape")
    if base_shape != [3, 480, 640] or wrist_shape != [3, 480, 640]:
        raise ValueError("source cameras must both be 640x480 RGB")
    preview_root = staging / "derived" / "previews" / run_id
    proxy_root = staging / "derived" / "targets" / run_id
    preview_root.mkdir(parents=True)
    proxy_root.mkdir(parents=True)

    video_audit = {}
    for role in ("base", "wrist"):
        video = _video_path(source, info, role, chunk=chunk, episode=episode_index)
        video_audit[role] = _extract_video(
            video, role=role, output=preview_root,
            expected_frames=len(data_rows), expected_width=640, expected_height=480,
        )

    frames = make_frame_rows(
        data_rows, source_rows, source_lines,
        source_dataset_sha256=source_dataset_sha,
        source_map_sha256=source_map_sha,
        source_run_id=run_id,
    )
    frames_path = preview_root / "frames.jsonl"
    frames_path.write_bytes(_jsonl_bytes(frames))
    adapter_manifest = {
        "schema": "piper_live_lerobot_next_state_bridge_manifest_v1",
        "source_run_id": run_id,
        "source_dataset": str(source),
        "source_dataset_tree_sha256": source_dataset_sha,
        "source_map_sha256": source_map_sha,
        "run_labels_sha256": label_sha,
        "acceptance_sha256": acceptance_sha,
        "task": task,
        "frames": len(data_rows),
        "fps": FPS,
        "last_source_frame_policy": "retained only as q(t+100ms); no copied final action",
        "source_command_evidence": audit["source_command_evidence"],
        "source_command_evidence_affects_training_label": False,
        "training_action_semantics": SEMANTICS,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    adapter_manifest_path = staging / "source_adapter_manifest.json"
    adapter_manifest_path.write_text(
        json.dumps(adapter_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    adapter_manifest_sha = _sha256(adapter_manifest_path)

    preview_meta = {
        "schema": "piper_aligned_preview_v1",
        "episode_id": run_id,
        "raw_episode_id": run_id,
        "raw_manifest_sha256": adapter_manifest_sha,
        "source": "operator_demonstration",
        "success": True,
        "operator_success_is_assertion": True,
        "fps": FPS,
        "frame_count": len(data_rows),
        "frame_map_sha256": _sha256(frames_path),
        "task": task,
        "timestamp_kind": "source LeRobot v2.1 10 Hz dataset grid; not capture monotonic time",
        "camera_pairing_semantics": "source host-after-read pair; not exposure synchronization",
        "camera_sync_verified": False,
        "joint_units_verified": False,
        "gripper_units_verified": False,
        "source_dataset_sha256": source_dataset_sha,
        "source_map_sha256": source_map_sha,
        "run_labels_sha256": label_sha,
        "acceptance_sha256": acceptance_sha,
        "source_command_evidence": audit["source_command_evidence"],
        "command_evidence_affects_training_label": False,
        "training_action_semantics": SEMANTICS,
        "frame_selection": (
            f"all {len(data_rows)} source rows; derive {expected_actions} q(t)->q(t+100ms) "
            "training labels and discard the final row as an action observation"
        ),
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    metadata_path = preview_root / "aligned_episode.json"
    metadata_path.write_text(
        json.dumps(preview_meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    proxy_rows = make_proxy_rows(frames)
    if len(proxy_rows) != expected_actions:
        raise ValueError("derived row count differs from source frames minus one")
    proxy_targets_path = proxy_root / "proxy_targets.jsonl"
    proxy_targets_path.write_bytes(_jsonl_bytes(proxy_rows))
    proxy_manifest = {
        "schema": "piper_next_state_proxy_v1",
        "semantics": SEMANTICS,
        "raw_episode_id": run_id,
        "preview_episode_id": run_id,
        "raw_manifest_sha256": adapter_manifest_sha,
        "source_manifest_kind": "piper_live_lerobot_next_state_bridge_manifest_v1",
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
        "source_dataset_sha256": source_dataset_sha,
        "source_map_sha256": source_map_sha,
        "source_command_evidence": audit["source_command_evidence"],
        "command_evidence_affects_training_label": False,
        "transition_delta_audit": audit_transition_deltas(proxy_rows),
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
        "note": (
            "Offline next-observed-state actions from consecutive feedback rows; "
            "actual CAN targets remain audit-only. Nominal units remain unverified."
        ),
    }
    (proxy_root / "proxy_manifest.json").write_text(
        json.dumps(proxy_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    repo_id = f"piper/{output.name}"
    conversion = convert_next_state_proxy.convert_many(
        [(proxy_root, preview_root)], output, repo_id, task=task
    )
    if conversion.get("frames") != expected_actions or conversion.get("episodes") != 1:
        raise ValueError("conversion frame count differs from the next-state contract")
    dataset_sha = proxy_multi_episode_contract.dataset_tree_sha256(output)
    strict = proxy_multi_episode_contract.inspect_proxy_multi_episode(
        output, min_episodes=1, expected_dataset_sha256=dataset_sha
    )
    if (strict.get("frames") != expected_actions
            or strict.get("action_semantics") != SEMANTICS
            or strict.get("formal_training_eligible") is not False
            or strict.get("hardware_control_allowed") is not False):
        raise ValueError("strict next-state contract returned an unexpected result")

    converted_rows = pq.read_table(next(output.glob("data/chunk-*/episode_*.parquet"))).to_pylist()
    if len(converted_rows) != expected_actions:
        raise ValueError("converted parquet row count differs from source frames minus one")
    for index, row in enumerate(converted_rows):
        if _seven(row.get("observation.state"), f"converted state {index}") != _seven(
                data_rows[index].get("observation.state"), f"source state {index}"):
            raise ValueError(f"converted observation differs at row {index}")
        if _seven(row.get("action"), f"converted action {index}") != _seven(
                data_rows[index + 1].get("observation.state"), f"future source state {index + 1}"):
            raise ValueError(f"converted action is not the next observed state at row {index}")
    if _tree_sha256(source) != source_dataset_sha or _sha256(source / "provenance" / "run_labels.jsonl") != label_sha:
        raise ValueError("source dataset or operator label changed during conversion")

    tasks_payload = json.loads((output / "meta" / "tasks.jsonl").read_text(encoding="utf-8").strip().splitlines()[0])
    if task not in json.dumps(tasks_payload, ensure_ascii=False):
        raise ValueError("source task text was not written to the derived LeRobot metadata")

    report = {
        "schema": "piper_live_next_state_bridge_report_v1",
        "status": "DERIVED_NEXT_STATE_DATASET_READY_OFFLINE",
        "source": {
            "dataset": str(source),
            "run_id": run_id,
            "task": task,
            "frames": len(data_rows),
            "fps": FPS,
            "source_tree_sha256_before": source_dataset_sha,
            "source_tree_sha256_after": _tree_sha256(source),
            "source_map_sha256": source_map_sha,
            "run_labels_sha256": label_sha,
            "acceptance_sha256": acceptance_sha,
            "operator_outcome": "SUCCESS",
            "operator_success_is_assertion": True,
            "videos": video_audit,
        },
        "training_label": {
            "status": "COMPLETE",
            "semantics": SEMANTICS,
            "time_relation": "q(t) to q(t+100ms)",
            "valid_rows": expected_actions,
            "total_rows": expected_actions,
            "source_rows": len(data_rows),
            "final_source_row_action_written": False,
            "final_source_row_role": "target-only future observation for the preceding row",
            "units": "nominal_unverified",
        },
        "command_evidence": {
            **audit["source_command_evidence"],
            "affects_training_label": False,
            "role": "send audit and traceability only",
        },
        "derived_dataset": {
            "path": str(output),
            "repo_id": repo_id,
            "sha256": dataset_sha,
            "conversion": conversion,
            "strict_contract": strict,
            "source_unchanged": True,
            "task_metadata_verified": True,
            "all_rows_action_equals_next_state": True,
        },
        "formal_training_eligible": False,
        "offline_training_allowed": True,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
        "note": (
            "One episode proves the offline conversion path only; it cannot establish "
            "model generalization or physical task success."
        ),
    }
    report_path = output.with_name(output.name + ".bridge_report.json")
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report["report_path"] = str(report_path)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dataset", type=Path, required=True)
    parser.add_argument("--output-dataset", type=Path)
    parser.add_argument("--stage-root", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    report = prepare_live_candidate(
        source_dataset=args.source_dataset,
        output_dataset=args.output_dataset,
        stage_root=args.stage_root,
        dry_run=args.dry_run,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, PermissionError, ValueError, RuntimeError) as exc:
        print(f"live training bridge stopped: {exc}", file=sys.stderr)
        raise SystemExit(2)
