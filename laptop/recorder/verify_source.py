"""Recompute every aligned source reference, numeric state, and extracted frame.

This is file-only verification. It never opens cameras, CAN, or a robot.
"""

from __future__ import annotations

from laptop.piper.unit_conversion import joint_raw_to_nominal_rad

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from laptop.recorder.align_raw import plan_alignment
from laptop.recorder.piper_command_targets import (
    ACTION_TARGET_SOURCE,
    decode_command_timeline,
    target_at,
)


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _video_frames(path: Path, wanted: set[int], expected_count: int) -> dict[int, np.ndarray]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f"cannot decode source video: {path}")
    selected = {}
    index = 0
    try:
        while True:
            ok, image = capture.read()
            if not ok:
                break
            if index in wanted:
                selected[index] = image
            index += 1
    finally:
        capture.release()
    if index != expected_count or len(selected) != len(wanted):
        raise ValueError(f"video frame count/source references mismatch: {path.name}")
    return selected


def verify_source(raw: Path, aligned: Path) -> dict:
    raw = Path(raw).resolve()
    aligned = Path(aligned).resolve()
    meta = json.loads((aligned / "aligned_episode.json").read_text(encoding="utf-8"))
    if meta.get("raw_episode_id") != raw.name:
        raise ValueError("raw episode identity mismatch")
    plan = plan_alignment(raw, fps=meta["fps"], max_skew_ms=meta["max_skew_ms"])
    if (meta.get("raw_manifest_sha256") != plan["raw_manifest_sha256"]
            or meta.get("frame_map_sha256") != plan["frame_map_sha256"]):
        raise ValueError("raw manifest or frame-map hash mismatch")
    rows = [json.loads(line) for line in (aligned / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(rows) != len(plan["frames"]) or meta.get("frame_count") != len(rows):
        raise ValueError("aligned row count differs from recomputed source map")
    decoded = {}
    for role in ("base", "wrist"):
        ids = {item["source_refs"][f"{role}_frame_id"] for item in plan["frames"]}
        decoded[role] = _video_frames(raw / f"{role}.mp4", ids, len(plan["channels"][role]))
    reviewed = meta.get("schema") == "piper_aligned_episode_v1"
    if not reviewed and meta.get("schema") != "piper_aligned_preview_v1":
        raise ValueError("unsupported aligned schema")
    actions = None
    review = None
    command_timeline = None
    if reviewed:
        if meta.get("review_relpath") != "review.json" or meta.get("actions_relpath") != "actions.jsonl":
            raise ValueError("reviewed episode lacks copied review/action evidence")
        review_path = aligned / "review.json"
        actions_path = aligned / "actions.jsonl"
        if (_hash(review_path) != meta.get("review_sha256")
                or _hash(actions_path) != meta.get("actions_sha256")):
            raise ValueError("review/action evidence hash mismatch")
        review = json.loads(review_path.read_text(encoding="utf-8"))
        action_source = meta.get("action_target_source")
        if (review.get("raw_manifest_sha256") != plan["raw_manifest_sha256"]
                or review.get("frame_map_sha256") != plan["frame_map_sha256"]
                or review.get("joint_degrees_per_raw") != meta.get("joint_degrees_per_raw")
                or review.get("gripper_meters_per_raw") != meta.get("gripper_meters_per_raw")
                or review.get("action_offset_ns") != meta.get("action_offset_ns")):
            raise ValueError("review metadata does not match source/aligned record")
        if action_source == ACTION_TARGET_SOURCE:
            command_log = raw / "command_can_raw.jsonl"
            raw_command_hash = plan.get("command_log_sha256")
            if (not command_log.is_file()
                    or meta.get("action_target_source_log_relpath") != "command_can_raw.jsonl"
                    or meta.get("action_target_source_log_sha256") != raw_command_hash
                    or review.get("command_can_raw_sha256") != raw_command_hash
                    or meta.get("action_hold_semantics") != "zero_order_hold_latest_complete_absolute_target"):
                raise ValueError("raw command log identity/checksum/hold contract mismatch")
            command_timeline = decode_command_timeline(command_log)
        elif action_source == "operator_verified_target":
            if review.get("actions_sha256") != meta["actions_sha256"]:
                raise ValueError("reviewed external action-file checksum mismatch")
        else:
            raise ValueError("unsupported reviewed action target source")
        actions = [json.loads(line) for line in actions_path.read_text(encoding="utf-8").splitlines()]
        if len(actions) != len(rows):
            raise ValueError("copied action count mismatch")
    for index, (actual, expected) in enumerate(zip(rows, plan["frames"])):
        for field in ("frame_index", "policy_timestamp_ns", "base_timestamp_ns",
                      "wrist_timestamp_ns", "state_timestamp_ns", "source_refs",
                      "gripper_raw", "status"):
            if actual.get(field) != expected[field]:
                raise ValueError(f"raw sample reference/value mismatch: frame {index} {field}")
        nominal = [joint_raw_to_nominal_rad(value) for value in expected["joint_raw"]]
        if not np.allclose(actual.get("joint_rad_nominal_only"), nominal, rtol=0, atol=1e-12):
            raise ValueError(f"nominal joint conversion mismatch: frame {index}")
        if reviewed:
            joint_scale = meta.get("joint_degrees_per_raw")
            gripper_scale = meta.get("gripper_meters_per_raw")
            if any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                   for value in (joint_scale, gripper_scale)):
                raise ValueError("invalid verified unit scales")
            state = [value * joint_scale * math.pi / 180 for value in expected["joint_raw"]]
            state.append(expected["gripper_raw"] * gripper_scale)
            if not np.allclose(actual.get("state"), state, rtol=0, atol=1e-9):
                raise ValueError(f"canonical state conversion mismatch: frame {index}")
            if action_source == ACTION_TARGET_SOURCE:
                requested_ts = expected["policy_timestamp_ns"] + meta["action_offset_ns"]
                target = target_at(command_timeline, requested_ts)
                expected_action = target.as_action_row(index) | {
                    "action_target_timestamp_ns": requested_ts,
                    "action_target_source": ACTION_TARGET_SOURCE,
                }
                if (actions[index] != expected_action
                        or actual.get("action_target_timestamp_ns") != requested_ts
                        or actual.get("action") != expected_action["action"]
                        or actual.get("action_target_raw") != expected_action["action_target_raw"]
                        or actual.get("action_source_timestamp_ns") != expected_action["action_source_timestamp_ns"]
                        or actual.get("action_source_refs") != expected_action["action_source_refs"]):
                    raise ValueError(f"action target cannot be reproduced from raw CAN command frames: {index}")
            elif (actions[index].get("frame_index") != index
                    or actions[index].get("action_target_timestamp_ns") != actual.get("action_target_timestamp_ns")
                    or actions[index].get("action") != actual.get("action")):
                raise ValueError(f"action target differs from reviewed file: frame {index}")
        for role in ("base", "wrist"):
            relative = actual.get(f"{role}_image")
            expected_name = f"{role}_{index:06d}.png"
            if relative != expected_name:
                raise ValueError(f"unexpected extracted image name: frame {index} {role}")
            path = (aligned / relative).resolve()
            if not path.is_relative_to(aligned) or not path.is_file():
                raise ValueError(f"missing or external extracted image: frame {index} {role}")
            image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            source_id = expected["source_refs"][f"{role}_frame_id"]
            if image is None or not np.array_equal(image, decoded[role][source_id]):
                raise ValueError(f"extracted RGB differs from source video frame: frame {index} {role}")
    return {
        "schema": "piper_sample_source_verification_v1",
        "raw_episode_id": raw.name,
        "aligned_episode_id": aligned.name,
        "raw_manifest_sha256": plan["raw_manifest_sha256"],
        "aligned_metadata_sha256": _hash(aligned / "aligned_episode.json"),
        "aligned_frames_sha256": _hash(aligned / "frames.jsonl"),
        "frame_map_sha256": plan["frame_map_sha256"],
        "samples_checked": len(rows),
        "video_frames_checked": 2 * len(rows),
        "source_frame_extraction_verified": True,
        "command_target_provenance_verified": bool(reviewed and meta.get("action_target_source") == ACTION_TARGET_SOURCE),
        "training_eligible": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("aligned", type=Path)
    args = parser.parse_args()
    print(json.dumps(verify_source(args.raw, args.aligned), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
