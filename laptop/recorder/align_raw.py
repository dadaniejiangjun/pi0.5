"""Derive timestamp-aligned PiPER frames from an immutable raw episode.

The default output is a review-only preview. A separate, checksum-bound human
review and canonical action-target file are required for the converter schema.
This module reads files only; it never opens CAN or controls a robot.
"""

from __future__ import annotations

from laptop.piper.unit_conversion import joint_raw_to_nominal_rad

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import shutil
import uuid

import cv2

from laptop.recorder.piper_command_targets import (
    ACTION_SEMANTICS as COMMAND_ACTION_SEMANTICS,
    ACTION_TARGET_SOURCE,
    GRIPPER_METERS_PER_RAW,
    JOINT_DEGREES_PER_RAW,
    decode_command_timeline,
    target_at,
)


TASK_ID = "pick_place_black_rod_green_mat_v1"
TASK = "Pick up the black rod and place it on the green desk mat."
CHANNELS = ("base", "wrist", "joint_12", "joint_34", "joint_56", "gripper", "status")
STATE_CHANNELS = CHANNELS[2:]
QC_FLAGS = (
    "visual_review_passed", "camera_sync_verified", "resampling_verified",
    "joint_units_verified", "gripper_units_verified", "gripper_action_verified",
    "action_target_verified",
)
UNITS = ["radian"] * 6 + ["meter"]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _inside(root: Path, relative: str) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError(f"invalid relative path: {relative!r}")
    target = (root / relative).resolve()
    if not target.is_relative_to(root.resolve()) or not target.is_file():
        raise ValueError(f"missing or external source file: {relative!r}")
    return target


def _lines(path: Path) -> list[dict]:
    rows = []
    last = -1
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            timestamp = row.get("monotonic_ns")
            if type(timestamp) is not int or timestamp <= last:
                raise ValueError(f"nonmonotonic source timestamps: {path.name}")
            rows.append(row)
            last = timestamp
    if not rows:
        raise ValueError(f"empty source channel: {path.name}")
    return rows


def _raw_episode(path: Path) -> tuple[dict, dict, dict[str, list[dict]], str, str | None]:
    path = Path(path).resolve()
    metadata = json.loads(_inside(path, "metadata.json").read_text(encoding="utf-8"))
    manifest_path = _inside(path, "manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if metadata.get("schema") != "piper_raw_episode_v1" or manifest.get("schema") != "piper_raw_manifest_v1":
        raise ValueError("unsupported raw episode schema")
    if metadata.get("episode_id") != path.name or manifest.get("episode_id") != path.name:
        raise ValueError("raw episode identity mismatch")
    if metadata.get("source") != manifest.get("source"):
        raise ValueError("raw source mismatch")
    if metadata.get("task_id") != TASK_ID or _inside(path, "instruction.txt").read_text(encoding="utf-8").strip() != TASK:
        raise ValueError("raw episode task mismatch")
    files = manifest.get("files")
    expected = {f"{channel}.jsonl" for channel in CHANNELS} | {"base.mp4", "wrist.mp4", "metadata.json", "instruction.txt"}
    if not isinstance(files, dict) or not expected.issubset(files):
        raise ValueError("raw manifest lacks required channels or videos")
    for relative, record in files.items():
        file = _inside(path, relative)
        if not isinstance(record, dict) or record.get("sha256") != _sha256(file) or record.get("bytes") != file.stat().st_size:
            raise ValueError(f"raw checksum/size mismatch: {relative}")
    command_log_hash = None
    if "command_can_raw.jsonl" in files:
        command_log_hash = files["command_can_raw.jsonl"].get("sha256")
        if not isinstance(command_log_hash, str) or len(command_log_hash) != 64:
            raise ValueError("invalid command log checksum record")
    channels = {channel: _lines(_inside(path, f"{channel}.jsonl")) for channel in CHANNELS}
    counts = manifest.get("counts", {})
    if any(counts.get(channel) != len(channels[channel]) for channel in CHANNELS):
        raise ValueError("raw manifest channel counts differ from files")
    for channel in ("base", "wrist"):
        if any(row.get("frame_id") != index for index, row in enumerate(channels[channel])):
            raise ValueError(f"noncontiguous {channel} frame IDs")
    return metadata, manifest, channels, _sha256(manifest_path), command_log_hash


def _nearest(rows: list[dict], stamps: list[int], timestamp: int, limit_ns: int, label: str) -> dict:
    index = bisect.bisect_left(stamps, timestamp)
    choices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(rows)]
    chosen = min(choices, key=lambda candidate: (abs(stamps[candidate] - timestamp), candidate))
    if abs(stamps[chosen] - timestamp) > limit_ns:
        raise ValueError(f"{label} has no sample inside alignment tolerance")
    return rows[chosen]


def _positive_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return float(value)


def plan_alignment(raw: Path, *, fps: int = 10, max_skew_ms: float = 50,
                   max_camera_skew_ms: float = 20) -> dict:
    """Validate a raw episode and create a deterministic, in-memory source map."""
    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError("fps must be an integer in [1, 60]")
    tolerance_ns = round(_positive_number(max_skew_ms, "max_skew_ms") * 1_000_000)
    if tolerance_ns > 50_000_000:
        raise ValueError("host-arrival tolerance may not exceed 50 ms")
    camera_tolerance_ns = round(
        _positive_number(max_camera_skew_ms, "max_camera_skew_ms") * 1_000_000
    )
    if camera_tolerance_ns > 50_000_000:
        raise ValueError("Base/Wrist host-timestamp pair tolerance may not exceed 50 ms")
    metadata, manifest, channels, raw_sha, command_log_hash = _raw_episode(raw)
    # Keep raw channel inventories intact for full video/hash verification.
    # Select images only inside the common complete feedback window.
    state_start = max(channels[name][0]["monotonic_ns"] for name in STATE_CHANNELS)
    state_end = min(channels[name][-1]["monotonic_ns"] for name in STATE_CHANNELS)
    selection_channels = dict(channels)
    for role in (("base", "wrist") if metadata.get("source") == "operator_demonstration" else ()):
        selection_channels[role] = [row for row in channels[role]
                                   if state_start <= row["monotonic_ns"] <= state_end]
        if len(selection_channels[role]) < 2:
            raise ValueError(f"fewer than two {role} frames in complete feedback window")
    start = max(rows[0]["monotonic_ns"] for rows in selection_channels.values())
    end = min(rows[-1]["monotonic_ns"] for rows in selection_channels.values())
    period = round(1_000_000_000 / fps)
    first = ((start + period - 1) // period) * period
    if first + period > end:
        raise ValueError("raw channels have fewer than two complete aligned samples")
    stamps = {name: [row["monotonic_ns"] for row in rows] for name, rows in selection_channels.items()}
    planned = []
    for policy_ts in range(first, end + 1, period):
        selected = {name: _nearest(rows, stamps[name], policy_ts, tolerance_ns, name)
                    for name, rows in selection_channels.items()}
        state_ts = selected["joint_12"]["monotonic_ns"]
        if any(abs(row["monotonic_ns"] - state_ts) > tolerance_ns for row in selected.values()):
            raise ValueError("selected camera/state samples exceed skew tolerance")
        camera_pair_skew_ns = abs(
            selected["base"]["monotonic_ns"] - selected["wrist"]["monotonic_ns"]
        )
        if (metadata.get("source") in ("operator_demonstration", "manual_teach")
                and camera_pair_skew_ns > camera_tolerance_ns):
            raise ValueError(
                "selected Base/Wrist host-after-read timestamps exceed "
                f"the {max_camera_skew_ms:g} ms software-pair tolerance; "
                "this is not an exposure-synchronization measurement"
            )
        joint_raw = []
        for name in ("joint_12", "joint_34", "joint_56"):
            pair = selected[name].get("raw_joint_pair")
            if not isinstance(pair, list) or len(pair) != 2 or any(type(value) is not int for value in pair):
                raise ValueError(f"invalid raw joint pair: {name}")
            joint_raw.extend(pair)
        gripper_raw = selected["gripper"].get("angle_or_stroke")
        if type(gripper_raw) is not int:
            raise ValueError("invalid raw gripper value")
        refs = {f"{name}_frame_id": selected[name]["frame_id"] for name in ("base", "wrist")}
        refs.update({f"{name}_timestamp_ns": selected[name]["monotonic_ns"] for name in STATE_CHANNELS})
        planned.append({
            "frame_index": len(planned), "policy_timestamp_ns": policy_ts,
            "base_timestamp_ns": selected["base"]["monotonic_ns"],
            "wrist_timestamp_ns": selected["wrist"]["monotonic_ns"],
            "base_wrist_host_timestamp_skew_ms": camera_pair_skew_ns / 1e6,
            "state_timestamp_ns": state_ts,
            "source_refs": refs, "joint_raw": joint_raw, "gripper_raw": gripper_raw,
            "status": {key: selected["status"].get(key) for key in ("ctrl_mode", "arm_status", "teach_status", "err_code")},
        })
    if len(planned) < 2:
        raise ValueError("alignment produced fewer than two frames")
    encoded = "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in planned)
    return {"metadata": metadata, "manifest": manifest, "channels": channels,
            "raw_path": raw, "command_log_sha256": command_log_hash,
            "raw_manifest_sha256": raw_sha, "frames": planned, "fps": fps,
            "max_camera_pair_skew_ms": max_camera_skew_ms,
            "max_skew_ms": max_skew_ms, "frame_map_sha256": hashlib.sha256(encoded.encode()).hexdigest()}


def _review(path: Path, plan: dict) -> tuple[dict, list[dict], str]:
    review = json.loads(Path(path).read_text(encoding="utf-8"))
    if (review.get("schema") != "piper_alignment_review_v1"
            or review.get("raw_manifest_sha256") != plan["raw_manifest_sha256"]
            or review.get("frame_map_sha256") != plan["frame_map_sha256"]):
        raise ValueError("review is not bound to this raw episode and frame map")
    if (plan["metadata"]["source"] not in ("manual_teach", "operator_demonstration")
            or review.get("success") is not True):
        raise ValueError("converter input requires a reviewed successful operator demonstration")
    if any(review.get(flag) is not True for flag in QC_FLAGS):
        raise ValueError("review lacks required QC evidence")
    action_source = review.get("action_target_source")
    if action_source not in ("operator_verified_target", ACTION_TARGET_SOURCE):
        raise ValueError("action targets require a supported, explicit source")
    joint_scale = _positive_number(review.get("joint_degrees_per_raw"), "joint_degrees_per_raw")
    gripper_scale = _positive_number(review.get("gripper_meters_per_raw"), "gripper_meters_per_raw")
    if action_source == ACTION_TARGET_SOURCE:
        if plan["metadata"]["source"] != "manual_teach":
            raise ValueError("command-derived targets require a manual-teach episode")
        if (joint_scale != JOINT_DEGREES_PER_RAW
                or gripper_scale != GRIPPER_METERS_PER_RAW
                or review.get("action_semantics") != COMMAND_ACTION_SEMANTICS
                or review.get("command_hold_semantics") != "zero_order_hold_latest_complete_absolute_target"):
            raise ValueError("command target units/semantics must match the nominal PiPER wire contract")
        if (plan.get("command_log_sha256") is None
                or review.get("command_can_raw_sha256") != plan["command_log_sha256"]):
            raise ValueError("review is not bound to the raw CAN command log")
        timeline = decode_command_timeline(plan["raw_path"] / "command_can_raw.jsonl")
        offset = review.get("action_offset_ns")
        if type(offset) is not int or offset < 0:
            raise ValueError("review action_offset_ns must be nonnegative integer")
        actions = []
        for index, frame in enumerate(plan["frames"]):
            action_ts = frame["policy_timestamp_ns"] + offset
            target = target_at(timeline, action_ts)
            actions.append(target.as_action_row(index) | {
                "action_target_timestamp_ns": action_ts,
                "action_target_source": ACTION_TARGET_SOURCE,
            })
        return review, actions, _sha256(Path(path))
    offset = review.get("action_offset_ns")
    if type(offset) is not int or offset < 0:
        raise ValueError("review action_offset_ns must be nonnegative integer")
    action_path = _inside(Path(path).resolve().parent, review.get("actions_file"))
    if _sha256(action_path) != review.get("actions_sha256"):
        raise ValueError("review action file checksum mismatch")
    actions = []
    with action_path.open(encoding="utf-8") as handle:
        for index, line in enumerate(handle):
            row = json.loads(line)
            if row.get("frame_index") != index or index >= len(plan["frames"]):
                raise ValueError("action target frame indices are not contiguous")
            if row.get("action_target_timestamp_ns") != plan["frames"][index]["policy_timestamp_ns"] + offset:
                raise ValueError("action target timestamp does not match policy grid")
            vector = row.get("action")
            if (not isinstance(vector, list) or len(vector) != 7
                    or any(isinstance(value, bool) or not isinstance(value, (int, float))
                           or not math.isfinite(value) for value in vector)):
                raise ValueError("action target must be a finite canonical 7D vector")
            actions.append(row)
    if len(actions) != len(plan["frames"]):
        raise ValueError("action target count does not match aligned frames")
    return review, actions, _sha256(Path(path))


def _extract_video(video: Path, output: Path, role: str, refs: list[int], expected_count: int) -> None:
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError(f"cannot decode {role} video")
    requested = {source_index: derived_index for derived_index, source_index in enumerate(refs)}
    if len(requested) != len(refs):
        raise ValueError(f"duplicate {role} source frame selected")
    actual = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if actual in requested:
                path = output / f"{role}_{requested[actual]:06d}.png"
                if not cv2.imwrite(str(path), frame):
                    raise OSError(f"cannot save extracted {role} frame")
            actual += 1
    finally:
        capture.release()
    if actual != expected_count or max(refs) >= actual:
        raise ValueError(f"{role} video frame count differs from timestamp log")


def align(raw: Path, output: Path, *, fps: int = 10, max_skew_ms: float = 50,
          max_camera_skew_ms: float = 20,
          review_path: Path | None = None) -> dict:
    """Write one immutable preview or a reviewed converter input."""
    raw = Path(raw).resolve()
    output = Path(output).resolve()
    if output.exists() or output.is_relative_to(raw) or raw.is_relative_to(output):
        raise ValueError("output must be unused and outside the raw episode")
    if review_path and not output.name.isalnum():
        raise ValueError("reviewed aligned episode ID must be alphanumeric")
    plan = plan_alignment(raw, fps=fps, max_skew_ms=max_skew_ms,
                          max_camera_skew_ms=max_camera_skew_ms)
    review, actions, review_sha = _review(review_path, plan) if review_path else (None, None, None)
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_name(output.name + ".INCOMPLETE-" + uuid.uuid4().hex[:12])
    pending.mkdir()
    if review:
        shutil.copyfile(review_path, pending / "review.json")
        if review.get("action_target_source") == ACTION_TARGET_SOURCE:
            with (pending / "actions.jsonl").open("x", encoding="utf-8") as handle:
                for row in actions:
                    handle.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
        else:
            shutil.copyfile(_inside(Path(review_path).resolve().parent, review["actions_file"]),
                            pending / "actions.jsonl")
    for role in ("base", "wrist"):
        refs = [row["source_refs"][f"{role}_frame_id"] for row in plan["frames"]]
        _extract_video(_inside(raw, f"{role}.mp4"), pending, role, refs, len(plan["channels"][role]))
    with (pending / "frames.jsonl").open("x", encoding="utf-8") as handle:
        for index, source in enumerate(plan["frames"]):
            row = dict(source)
            row["base_image"] = f"base_{index:06d}.png"
            row["wrist_image"] = f"wrist_{index:06d}.png"
            row["joint_rad_nominal_only"] = [joint_raw_to_nominal_rad(value) for value in row.pop("joint_raw")]
            if review:
                row["state"] = [value * review["joint_degrees_per_raw"] * math.pi / 180
                                for value in source["joint_raw"]] + [source["gripper_raw"] * review["gripper_meters_per_raw"]]
                row["action"] = actions[index]["action"]
                row["action_target_timestamp_ns"] = actions[index]["action_target_timestamp_ns"]
                if review["action_target_source"] == ACTION_TARGET_SOURCE:
                    row["action_target_raw"] = actions[index]["action_target_raw"]
                    row["action_source_timestamp_ns"] = actions[index]["action_source_timestamp_ns"]
                    row["action_source_refs"] = actions[index]["action_source_refs"]
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    metadata = {
        "schema": "piper_aligned_episode_v1" if review else "piper_aligned_preview_v1",
        "episode_id": output.name, "raw_episode_id": raw.name,
        "raw_manifest_sha256": plan["raw_manifest_sha256"],
        "source": plan["metadata"]["source"], "task_id": TASK_ID, "task": TASK,
        "fps": fps, "frame_count": len(plan["frames"]),
        "frame_map_sha256": plan["frame_map_sha256"],
        "timestamp_kind": "host_perf_counter_ns_after_read_not_camera_exposure",
        "max_skew_ms": max_skew_ms, "training_eligible": False,
        "max_camera_pair_skew_ms": max_camera_skew_ms,
        "camera_pairing_semantics": "host_after_read_timestamp_pairing_not_exposure_sync",
        "success": review["success"] if review else None,
        **{flag: review[flag] if review else False for flag in QC_FLAGS},
    }
    if review:
        actions_sha = _sha256(pending / "actions.jsonl")
        metadata.update({
            "action_semantics": "absolute_joint_position",
            "action_target_source": review["action_target_source"],
            "action_offset_ns": review["action_offset_ns"],
            "state_units": UNITS, "action_units": UNITS,
            "joint_degrees_per_raw": review["joint_degrees_per_raw"],
            "gripper_meters_per_raw": review["gripper_meters_per_raw"],
            "review_sha256": review_sha, "actions_sha256": actions_sha,
            "review_relpath": "review.json", "actions_relpath": "actions.jsonl",
        })
        if review["action_target_source"] == ACTION_TARGET_SOURCE:
            metadata.update({
                "action_target_source_log_relpath": "command_can_raw.jsonl",
                "action_target_source_log_sha256": plan["command_log_sha256"],
                "action_hold_semantics": "zero_order_hold_latest_complete_absolute_target",
                "action_target_provenance_verified": True,
            })
    (pending / "aligned_episode.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    pending.rename(output)
    return {"status": "ALIGNED_REVIEWED_NEEDS_CONVERTER_AUDIT" if review else "PREVIEW_ONLY",
            "output": str(output), "frame_count": len(plan["frames"]),
            "frame_map_sha256": plan["frame_map_sha256"], "training_eligible": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--max-skew-ms", type=float, default=50)
    parser.add_argument("--max-camera-skew-ms", type=float, default=20,
                        help="maximum Base/Wrist host-after-read timestamp skew for each operator-demo sample")
    parser.add_argument("--review", type=Path, help="separate checksum-bound review JSON; omit for preview")
    args = parser.parse_args()
    print(json.dumps(align(args.raw, args.output, fps=args.fps,
                           max_skew_ms=args.max_skew_ms,
                           max_camera_skew_ms=args.max_camera_skew_ms,
                           review_path=args.review), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
