"""Normalize accepted live-writer feedback episodes for the pinned old route."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from typing import Any

from laptop.piper.unit_conversion import nominal_m_to_gripper_raw, nominal_rad_to_joint_raw

TASK = "Pick up the black rod and place it on the green desk mat."
TASK_ID = "pick_place_black_rod_green_mat_v1"
CHANNELS = ("base", "wrist", "joint_12", "joint_34", "joint_56", "gripper", "status")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
                    encoding="utf-8")


def _raw_id(index: int, origin: dict) -> str:
    digest = hashlib.sha256(
        f"{origin['run_id']}:{origin['episode_index']}:{origin['manifest_sha256']}".encode()
    ).hexdigest()[:16]
    return f"fb{index:03d}{digest}"


def materialize_legacy_inputs(source_root: str | Path, source_info: dict,
                              accepted: list[dict], sidecar_paths: dict[str, Path],
                              workspace: str | Path) -> dict:
    """Create checksum-bound raw/outcome copies; never alter the source dataset."""
    source_root, workspace = Path(source_root).resolve(strict=True), Path(workspace).resolve()
    raw_root, outcome_root = workspace / "raw", workspace / "outcomes"
    raw_root.mkdir(parents=True, exist_ok=False)
    outcome_root.mkdir(parents=True, exist_ok=False)
    source_hashes = {name: sha256(path) for name, path in sidecar_paths.items()}
    episodes = []
    for position, item in enumerate(accepted):
        origin = item["source"]
        label = item["label"]
        rows = item["rows"]
        meta = item["meta"]
        raw_id = _raw_id(position, origin)
        raw = raw_root / raw_id
        raw.mkdir()
        episode_index = int(meta["episode_index"])
        episode_chunk = episode_index // int(source_info["chunks_size"])
        assets = {}
        for channel in ("base", "wrist"):
            video = source_root / source_info["video_path"].format(
                episode_index=episode_index, episode_chunk=episode_chunk,
                video_key=f"observation.images.{channel}")
            if not video.is_file():
                raise FileNotFoundError(f"source {channel} video is missing")
            shutil.copy2(video, raw / f"{channel}.mp4")
            assets[f"{channel}.mp4"] = {"bytes": video.stat().st_size, "sha256": sha256(video)}

        state_rows = []
        camera_rows = {"base": [], "wrist": []}
        frame_bindings = []
        for frame_index, row in enumerate(rows):
            state = row["state"]
            values = state["values_float32"]
            timestamp = state["source_local_monotonic_ns"]
            raw_joints = [nominal_rad_to_joint_raw(float(value)) for value in values[:6]]
            gripper_raw = nominal_m_to_gripper_raw(float(values[6]))
            for joint_index, channel in enumerate(("joint_12", "joint_34", "joint_56")):
                pair = raw_joints[joint_index * 2:joint_index * 2 + 2]
                state_rows.append((channel, {"monotonic_ns": timestamp, "raw_joint_pair": pair}))
            state_rows.append(("gripper", {"monotonic_ns": timestamp,
                                           "angle_or_stroke": gripper_raw,
                                           "source_feedback_timestamp_ns": timestamp}))
            state_rows.append(("status", {"monotonic_ns": timestamp,
                                          "ctrl_mode": None, "arm_status": None,
                                          "teach_status": None, "err_code": None}))
            for camera in ("base", "wrist"):
                image = row.get("images", {}).get(camera, {})
                image_ts = image.get("source_local_monotonic_ns")
                if type(image_ts) is not int:
                    raise ValueError(f"source row lacks raw {camera} monotonic timestamp")
                camera_rows[camera].append({"monotonic_ns": image_ts, "frame_id": frame_index,
                                            "source_image_sequence": image.get("source_sequence"),
                                            "source_rgb_sha256": image.get("rgb_pixels_sha256")})
            frame_bindings.append({
                "legacy_raw_frame_index": frame_index,
                "source_run_id": origin["run_id"],
                "source_episode_index": origin["episode_index"],
                "source_frame_index": row["episode_frame_index"],
                "source_manifest_sha256": origin["manifest_sha256"],
                "source_feedback_timestamp_ns": timestamp,
                "source_action_semantics": row.get("action_semantics"),
                "source_action_valid": row.get("action_valid"),
                "source_frame_timestamp_s": row.get("frame_timestamp_s"),
                "source_map_row_sha256": hashlib.sha256(json.dumps(
                    row, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
            })
        channel_records: dict[str, list[dict]] = {name: [] for name in CHANNELS}
        channel_records.update(camera_rows)
        for channel, record in state_rows:
            channel_records[channel].append(record)
        for channel, records in channel_records.items():
            if not records:
                raise ValueError(f"source episode lacks {channel} samples")
            path = raw / f"{channel}.jsonl"
            path.write_text("".join(json.dumps(record, sort_keys=True, separators=(",", ":"),
                                        allow_nan=False) + "\n" for record in records),
                            encoding="utf-8")
            assets[path.name] = {"bytes": path.stat().st_size, "sha256": sha256(path)}

        metadata = {
            "schema": "piper_raw_episode_v1", "episode_id": raw_id,
            "source": "operator_demonstration", "task_id": TASK_ID, "task": TASK,
            "timestamp_kind": "original_feedback_and_image_source_local_monotonic_ns",
            "state_encoding": "nominal feedback quantized to existing integer raw schema",
            "action_target_present": False, "training_eligible": False,
            "adapter_source_manifest_sha256": origin["manifest_sha256"],
        }
        write_json(raw / "metadata.json", metadata)
        (raw / "instruction.txt").write_text(TASK + "\n", encoding="utf-8")
        (raw / "events.jsonl").write_text(
            json.dumps({"name": "start", "source_adapter": True}) + "\n" +
            json.dumps({"name": "end", "success": True,
                        "source_label_sha256": source_hashes["run_labels.jsonl"]}) + "\n",
            encoding="utf-8")
        for name in ("metadata.json", "instruction.txt", "events.jsonl"):
            path = raw / name
            assets[name] = {"bytes": path.stat().st_size, "sha256": sha256(path)}
        manifest = {
            "schema": "piper_raw_manifest_v1", "episode_id": raw_id,
            "source": "operator_demonstration",
            "counts": {channel: len(channel_records[channel]) for channel in CHANNELS},
            "files": assets, "training_eligible": False,
            "reason": "normalized feedback source; no command supervision",
            "adapter_source_manifest_sha256": origin["manifest_sha256"],
        }
        write_json(raw / "manifest.json", manifest)
        raw_manifest_sha = sha256(raw / "manifest.json")
        normalized_label = {
            "schema": "piper_episode_outcome_v1", "episode_id": raw_id,
            "raw_manifest_sha256": raw_manifest_sha,
            "success": True,
            "notes": str(label.get("label_note") or label.get("notes") or "explicit complete success label"),
            "operator": str(label.get("operator") or origin.get("operator") or "source_label_operator"),
            "source_label": label,
            "source_label_sha256": source_hashes["run_labels.jsonl"],
            "source_manifest_sha256": origin["manifest_sha256"],
            "training_eligible": False,
        }
        label_path = outcome_root / f"{raw_id}.json"
        write_json(label_path, normalized_label)
        episodes.append({
            "legacy_raw_episode_id": raw_id,
            "legacy_raw_manifest_sha256": raw_manifest_sha,
            "source_run_id": origin["run_id"],
            "source_episode_index": origin["episode_index"],
            "source_manifest_sha256": origin["manifest_sha256"],
            "source_bag_db_sha256": origin.get("bag_db_sha256"),
            "source_label_sha256": source_hashes["run_labels.jsonl"],
            "source_frame_count": len(rows),
            "frame_bindings": frame_bindings,
        })
    binding = {
        "schema": "piper_feedback_source_adapter_v1",
        "source_path": str(source_root),
        "source_sidecar_sha256": source_hashes,
        "adapter": "v2.1 sidecars and feedback normalized to immutable legacy raw episode schema",
        "episodes": episodes,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    binding_path = workspace / "source_adapter_manifest.json"
    write_json(binding_path, binding)
    return {"raw_root": raw_root, "outcome_root": outcome_root,
            "source_adapter_manifest": binding_path, "binding": binding}
