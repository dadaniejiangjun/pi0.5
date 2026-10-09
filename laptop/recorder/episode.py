"""Write and validate local episodes; this module never accesses robot control."""

from __future__ import annotations

import hashlib
import json
import math
import bisect
import statistics
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path


PROMPT = "Pick up the black rod and place it on the green desk mat."
CHANNELS = ("base", "wrist", "joint_12", "joint_34", "joint_56", "gripper", "status")
DEMONSTRATION_SOURCES = frozenset(("manual_teach", "operator_demonstration"))


def _write_json_line(handle, value: dict) -> None:
    handle.write(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n")


class EpisodeWriter:
    """Record raw observations without inventing actions or gripper targets."""

    def __init__(self, raw_root: Path, *, episode_id: str | None = None, source: str = "synthetic",
                 task: str = PROMPT):
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task must be nonempty text")
        self.episode_id = episode_id or uuid.uuid4().hex
        if not self.episode_id.isalnum():
            raise ValueError("episode_id must be alphanumeric")
        if source not in ("synthetic", "manual_teach", "operator_demonstration", "passive_wiring_smoke"):
            raise ValueError("unsupported source")
        self.path = Path(raw_root) / self.episode_id
        self.path.mkdir(parents=True, exist_ok=False)
        self.source = source
        self.closed = False
        self.last_ts = {}
        self.counts = {channel: 0 for channel in CHANNELS}
        self.streams = {
            channel: (self.path / f"{channel}.jsonl").open("x", encoding="utf-8")
            for channel in CHANNELS
        }
        self.events = (self.path / "events.jsonl").open("x", encoding="utf-8")
        (self.path / "instruction.txt").write_text(task.strip() + "\n", encoding="utf-8")
        (self.path / "metadata.json").write_text(json.dumps({
            "schema": "piper_raw_episode_v1", "episode_id": self.episode_id,
            "source": source, "task_id": ("pick_place_black_rod_green_mat_v1" if task.strip() == PROMPT else "custom_task"),
            "task": task.strip(),
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "timestamp_kind": "host_perf_counter_ns_after_read_not_exposure",
            "joint_unit": "raw_sdk_0p001deg_nominal_unverified",
            "gripper_unit": "raw_sdk_ambiguous_unverified",
            "action_target_present": False, "training_eligible": False,
        }, indent=2) + "\n", encoding="utf-8")
        self.event("start")

    def add(self, channel: str, timestamp_ns: int, payload: dict) -> None:
        if self.closed or channel not in self.streams:
            raise ValueError("episode closed or unknown channel")
        if not isinstance(timestamp_ns, int) or timestamp_ns <= self.last_ts.get(channel, -1):
            raise ValueError("timestamps must increase per channel")
        if not isinstance(payload, dict):
            raise ValueError("payload must be a dict")
        if "monotonic_ns" in payload:
            raise ValueError("payload cannot override timestamp")
        self.last_ts[channel] = timestamp_ns
        _write_json_line(self.streams[channel], {"monotonic_ns": timestamp_ns, **payload})
        self.counts[channel] += 1

    def event(self, name: str, **details) -> None:
        if self.closed:
            raise ValueError("episode closed")
        _write_json_line(self.events, {"name": name, "monotonic_ns": time.perf_counter_ns(), **details})

    def close(self, *, success: bool | None = None, notes: str = "") -> dict:
        if self.closed:
            raise ValueError("episode already closed")
        self.event("end", success=success, notes=notes)
        for handle in (*self.streams.values(), self.events):
            handle.close()
        self.closed = True
        files = {}
        for path in sorted(self.path.iterdir()):
            if path.is_file():
                files[path.name] = {"bytes": path.stat().st_size,
                                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        manifest = {"schema": "piper_raw_manifest_v1", "episode_id": self.episode_id,
                    "source": self.source, "counts": self.counts,
                    "files": files, "training_eligible": False,
                    "reason": "raw feedback is not a validated action target"}
        (self.path / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        return validate_episode(self.path)


def validate_episode(path: Path) -> dict:
    path = Path(path)
    errors = []
    warnings = []
    try:
        manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
        metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"status": "INVALID", "errors": [str(exc)]}
    for name, record in manifest["files"].items():
        file_path = path / name
        if not file_path.is_file() or hashlib.sha256(file_path.read_bytes()).hexdigest() != record["sha256"]:
            errors.append(f"checksum mismatch: {name}")
    channel_times = {}
    for channel in CHANNELS:
        stream = path / f"{channel}.jsonl"
        if not stream.is_file():
            errors.append(f"missing channel: {channel}")
            continue
        last = -1
        rows = 0
        try:
            with stream.open(encoding="utf-8") as handle:
                for line in handle:
                    row = json.loads(line)
                    ts = row["monotonic_ns"]
                    if not isinstance(ts, int) or ts <= last:
                        errors.append(f"nonmonotonic: {channel}")
                    last = ts
                    rows += 1
                    for key, value in row.items():
                        if isinstance(value, float) and not math.isfinite(value):
                            errors.append(f"nonfinite: {channel}.{key}")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append(f"invalid channel {channel}: {exc}")
        if rows != manifest["counts"].get(channel):
            errors.append(f"count mismatch: {channel}")
        if rows == 0:
            errors.append(f"empty channel: {channel}")
        channel_times[channel] = []
        if stream.is_file():
            try:
                channel_times[channel] = [json.loads(line)["monotonic_ns"]
                                          for line in stream.read_text(encoding="utf-8").splitlines()]
            except (ValueError, KeyError, TypeError):
                pass
    for name in ("start", "end"):
        if not any(json.loads(line).get("name") == name for line in (path / "events.jsonl").read_text(encoding="utf-8").splitlines()):
            errors.append(f"missing event: {name}")
    for name in ("base.mp4", "wrist.mp4"):
        if not (path / name).is_file():
            errors.append(f"missing video: {name}")
    video_counts = {}
    try:
        import cv2
        for role in ("base", "wrist"):
            video = path / f"{role}.mp4"
            if not video.is_file():
                continue
            capture = cv2.VideoCapture(str(video))
            count = 0
            try:
                if not capture.isOpened():
                    errors.append(f"video open failed: {role}")
                else:
                    while True:
                        ok, _ = capture.read()
                        if not ok:
                            break
                        count += 1
            finally:
                capture.release()
            video_counts[role] = count
            if count != manifest["counts"].get(role):
                errors.append(f"video/frame timestamp count mismatch: {role}")
    except ImportError:
        errors.append("OpenCV unavailable: video decode unverified")
    skew_ms = []
    wrist = channel_times.get("wrist", [])
    for ts in channel_times.get("base", []):
        if not wrist:
            break
        position = bisect.bisect_left(wrist, ts)
        nearby = wrist[max(0, position - 1):position + 1]
        skew_ms.append(min(abs(ts - other) for other in nearby) / 1e6)
    state_channels = ("joint_12", "joint_34", "joint_56", "gripper", "status")
    state_window = None
    if all(channel_times.get(channel) for channel in state_channels):
        state_window = (max(channel_times[channel][0] for channel in state_channels),
                        min(channel_times[channel][-1] for channel in state_channels))
        if state_window[1] <= state_window[0]:
            errors.append("state channels have no common time window")
    camera_state = {}
    if state_window and state_window[1] > state_window[0]:
        reference = channel_times["joint_12"]
        for role in ("base", "wrist"):
            timestamps = channel_times.get(role, [])
            outside = sum(ts < state_window[0] or ts > state_window[1] for ts in timestamps)
            inside = [ts for ts in timestamps if state_window[0] <= ts <= state_window[1]]
            nearest_ms = []
            for ts in inside:
                position = bisect.bisect_left(reference, ts)
                nearby = reference[max(0, position - 1):min(len(reference), position + 1)]
                nearest_ms.append(min(abs(ts - other) for other in nearby) / 1e6)
            camera_state[role] = {
                "frames_outside_complete_state_window": outside,
                "frames_inside_complete_state_window": len(inside),
                "joint_12_nearest_host_arrival_median_ms": statistics.median(nearest_ms) if nearest_ms else None,
                "joint_12_nearest_host_arrival_p95_ms": (
                    sorted(nearest_ms)[int(0.95 * (len(nearest_ms) - 1))] if nearest_ms else None),
            }
            if outside:
                warnings.append(f"camera boundary frames excluded during alignment: {role}={outside}")
            if len(inside) < 2:
                errors.append(f"camera has fewer than two frames inside complete state window: {role}")
    source = metadata.get("source")
    if source not in DEMONSTRATION_SOURCES:
        errors.append("episode source is not a task demonstration")
    if not metadata.get("action_target_present") and source != "operator_demonstration":
        errors.append("validated action target absent")
    elif metadata.get("action_target_source") == "observed_piper_can_command_frames":
        command_path = path / "command_can_raw.jsonl"
        record = manifest.get("files", {}).get("command_can_raw.jsonl")
        try:
            if (not command_path.is_file() or not isinstance(record, dict)
                    or hashlib.sha256(command_path.read_bytes()).hexdigest() != record.get("sha256")
                    or record.get("sha256") != metadata.get("command_target_log_sha256")):
                raise ValueError("command log checksum does not match episode provenance")
            from laptop.recorder.piper_command_targets import decode_command_timeline
            decode_command_timeline(command_path)
        except (OSError, ValueError, TypeError) as exc:
            errors.append(f"invalid command target source: {exc}")
        if metadata.get("command_direction_verified") is not False:
            errors.append("command bus source direction must remain explicitly unverified")
    elif metadata.get("action_target_present"):
        errors.append("action target source is not a supported command-frame source")
    return {"status": "FILE_INTEGRITY_PASS" if not errors else "INCOMPLETE", "errors": errors,
            "warnings": warnings,
            "training_eligible": False, "hardware_control_allowed": False,
            "episode_id": manifest["episode_id"],
            "decoded_video_frames": video_counts,
            "base_wrist_host_arrival_skew_median_ms": statistics.median(skew_ms) if skew_ms else None,
            "complete_state_window_duration_s": ((state_window[1] - state_window[0]) / 1e9
                                                 if state_window else None),
            "camera_state_alignment": camera_state,
            "timestamp_note": "Host arrival is not camera exposure or camera-to-state synchronization."}
