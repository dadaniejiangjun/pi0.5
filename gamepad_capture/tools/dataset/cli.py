"""Offline conversion, resume, audit, and masked-filter export CLI."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import sys
from typing import Any, Iterable

from .bag import (
    DEFAULT_MANIFEST,
    RUN_ID,
    RosbagReader,
    match_camera_frames,
    resolve_manifest_bag_root,
    sha256_file,
    verify_source,
)
from .semantics import group_command_events, raw_target_to_nominal_action


FPS = 10
MAX_ACTION_AGE_MS = 250
CAMERA_MATCH_MAX_MS = 60
TASK = "Pick up the black rod and place it on the green desk mat."
DEFAULT_OUTPUT = Path("gamepad_capture/data/lerobot") / f"{RUN_ID}_kernel_accepted"
ACTION_NAMES = ["joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad",
                "joint_5_rad", "joint_6_rad", "gripper_nominal_m"]
STATE_NAMES = ACTION_NAMES.copy()
CAMERA_TOPICS = {
    "base": "/piper/cameras/base/image_raw",
    "wrist": "/piper/cameras/wrist/image_raw",
}


def _json_write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                         encoding="utf-8")
    os.replace(temporary, path)


def _jsonl_write(path: Path, rows: Iterable[dict[str, Any]], *, append: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = "a" if append else "w"
    with path.open(mode, encoding="utf-8", newline="\n") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False))
            handle.write("\n")


def _jsonl_upsert(path: Path, rows: Iterable[dict[str, Any]], key_fields: tuple[str, ...]) -> None:
    """Atomically merge JSONL sidecars by their stable identity fields."""
    existing = []
    if path.exists():
        existing = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    merged: dict[tuple[Any, ...], dict[str, Any]] = {}
    order: list[tuple[Any, ...]] = []
    for row in [*existing, *rows]:
        key = tuple(row.get(field) for field in key_fields)
        if any(value is None for value in key):
            raise ValueError(f"JSONL upsert row lacks identity fields {key_fields}: {key}")
        if key not in merged:
            order.append(key)
        merged[key] = row
    _jsonl_write(path, (merged[key] for key in order))


def _sha256_tree(root: Path, roots: tuple[str, ...]) -> tuple[str, dict[str, str]]:
    files = []
    for relative_root in roots:
        source = root / relative_root
        if source.is_file():
            files.append(source)
        elif source.is_dir():
            files.extend(path for path in source.rglob("*") if path.is_file())
    files = sorted(files, key=lambda path: path.relative_to(root).as_posix())
    listing = {}
    digest = hashlib.sha256()
    for path in files:
        relative = path.relative_to(root).as_posix()
        file_digest = sha256_file(path)
        listing[relative] = file_digest
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_digest.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest(), listing


def _check_runtime() -> dict[str, str]:
    from importlib.metadata import distribution, version

    actual = {
        "python_version": ".".join(map(str, sys.version_info[:3])),
        "lerobot_version": version("lerobot"),
        "datasets_version": version("datasets"),
        "rerun_sdk_version": version("rerun-sdk"),
    }
    expected = {"lerobot_version": "0.1.0", "datasets_version": "3.6.0",
                "rerun_sdk_version": "0.26.2"}
    for key, value in expected.items():
        if actual[key] != value:
            raise RuntimeError(f"training-compatible dataset environment mismatch: {key}={actual[key]} expected {value}")
    direct_url = distribution("lerobot").read_text("direct_url.json")
    revision = "unknown"
    if direct_url:
        try:
            revision = json.loads(direct_url).get("vcs_info", {}).get("commit_id", "unknown")
        except json.JSONDecodeError:
            pass
    actual["lerobot_revision"] = revision
    if revision != "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5":
        raise RuntimeError(f"LeRobot source revision mismatch: {revision}")
    return actual


def _configure_offline_cache(cache_root: Path) -> None:
    """Isolate Hugging Face caches and temp files for offline local loading.

    `datasets.load_dataset("parquet", ...)` otherwise serializes builders in
    the user's shared cache. This tool must not wait on or disturb that cache.
    Set these before importing `datasets`/LeRobot in the current process.
    """
    cache_root = Path(cache_root).resolve()
    cache_root.mkdir(parents=True, exist_ok=True)
    temp_root = cache_root / "tmp"
    temp_root.mkdir(parents=True, exist_ok=True)
    os.environ["HF_HOME"] = str(cache_root)
    os.environ["HF_DATASETS_CACHE"] = str(cache_root / "datasets")
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    os.environ["TEMP"] = str(temp_root)
    os.environ["TMP"] = str(temp_root)
    # Some launcher/test processes import `datasets` before this CLI gets
    # control. Keep its already-loaded constants aligned with the isolated
    # environment so a subsequent loader never falls back to the shared cache.
    datasets_config = sys.modules.get("datasets.config")
    if datasets_config is not None:
        datasets_config.HF_DATASETS_CACHE = str(cache_root / "datasets")
        datasets_config.DOWNLOADED_DATASETS_PATH = str(
            cache_root / "datasets" / datasets_config.DOWNLOADED_DATASETS_DIR)


def _feature_schema(height: int, width: int) -> dict[str, dict[str, Any]]:
    image_shape = (3, height, width)
    return {
        "observation.images.base": {"dtype": "video", "shape": image_shape,
                                    "names": ["channel", "height", "width"]},
        "observation.images.wrist": {"dtype": "video", "shape": image_shape,
                                     "names": ["channel", "height", "width"]},
        "observation.state": {"dtype": "float32", "shape": (7,), "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": (7,), "names": ACTION_NAMES},
        "action_valid": {"dtype": "bool", "shape": (1,), "names": ["valid"]},
        "observation_valid": {"dtype": "bool", "shape": (1,), "names": ["valid"]},
        "camera_valid": {"dtype": "bool", "shape": (1,), "names": ["valid"]},
    }


class SelectedImageStream:
    """Decode only selected messages while walking each camera topic once."""

    def __init__(self, reader: RosbagReader, role: str, selected_stamps: set[int]):
        self.reader = reader
        self.role = role
        self.selected = set(selected_stamps)
        self.topic = CAMERA_TOPICS[role]
        self.iterator = reader.iter_topic(self.topic)
        self.seen: set[int] = set()

    def read(self, wanted_stamp_ns: int):
        import cv2
        import numpy as np
        from .cdr import decode_image, decode_image_header

        while True:
            try:
                bag_timestamp_ns, data = next(self.iterator)
            except StopIteration as exc:
                raise ValueError(f"selected {self.role} image header stamp was not present: {wanted_stamp_ns}") from exc
            header = decode_image_header(data)
            if header.stamp_ns not in self.selected:
                continue
            if header.stamp_ns in self.seen:
                raise ValueError(f"duplicate selected {self.role} image header stamp {header.stamp_ns}")
            self.seen.add(header.stamp_ns)
            _, image = decode_image(data)
            if image["encoding"] != "bgr8" or image["is_bigendian"] not in (0, 1):
                raise ValueError(f"unsupported {self.role} image encoding {image['encoding']}")
            height, width, step = image["height"], image["width"], image["step"]
            if step < width * 3:
                raise ValueError(f"{self.role} image row stride is shorter than width*3")
            rows = np.frombuffer(image["data"], dtype=np.uint8).reshape(height, step)
            bgr = rows[:, :width * 3].reshape(height, width, 3)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return bag_timestamp_ns, header, np.ascontiguousarray(rgb)


def _read_all_records(reader: RosbagReader, *, session_id: str,
                      max_action_age_ms: int = MAX_ACTION_AGE_MS,
                      camera_match_max_ms: int = CAMERA_MATCH_MAX_MS) -> dict[str, Any]:
    records = reader.low_rate_records()
    states = records["states"]
    if not states:
        raise ValueError("rosbag contains no state snapshots")
    state_by_seq = {}
    for bag_ts, state in states:
        seq = int(state["sequence"])
        if seq in state_by_seq:
            raise ValueError(f"duplicate state sequence {seq}")
        if not all(math.isfinite(float(v)) for v in state["joint_position_rad"] + [state["gripper_nominal_m"]]):
            raise ValueError(f"non-finite state values at sequence {seq}")
        state_by_seq[seq] = state
    commands = group_command_events(
        records["send_events"], records["receipts"], state_by_seq,
        expected_session_id=session_id,
        max_action_age_ns=max_action_age_ms * 1_000_000,
    )
    camera_rows = match_camera_frames(
        states, records["metadata"], max_delta_ns=camera_match_max_ms * 1_000_000,
    )
    return {**records, "state_by_seq": state_by_seq, "commands": commands,
            "camera_rows": camera_rows}


def _action_by_state(records: dict[str, Any]) -> tuple[dict[int, dict[str, Any]], dict[int, str]]:
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for request in records["commands"]["requests"]:
        seq = request.get("source_state_sequence")
        if type(seq) is int:
            grouped[seq].append(request)
    chosen: dict[int, dict[str, Any]] = {}
    rejected: dict[int, str] = {}
    cutoff = records["commands"]["delivery_incomplete_cutoff_monotonic_ns"]
    for seq, requests in grouped.items():
        if len(requests) != 1:
            rejected[seq] = "multiple_requests_share_source_state_sequence"
            continue
        request = requests[0]
        if request.get("action_valid") is not True:
            rejected[seq] = ";".join(request.get("reasons", [])) or "request_not_complete"
            continue
        if cutoff is not None and int(request["accepted_position_monotonic_ns"]) >= cutoff:
            rejected[seq] = "after_delivery_incomplete_cutoff"
            continue
        chosen[seq] = request
    return chosen, rejected


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    at = (len(values) - 1) * percentile / 100.0
    low = math.floor(at)
    high = math.ceil(at)
    if low == high:
        return values[low]
    return values[low] * (high - at) + values[high] * (at - low)


def _consecutive_windows(rows: list[dict[str, Any]], key: str) -> list[dict[str, Any]]:
    eligible = [index for index, row in enumerate(rows) if row.get(key) is True]
    windows = []
    for index in eligible:
        previous = rows[index - 1] if index > 0 else None
        same_source_episode = (previous is not None
                               and previous.get("run_id") == rows[index].get("run_id")
                               and previous.get("episode_index") == rows[index].get("episode_index"))
        if (not windows or index != windows[-1]["end_frame_index"] + 1
                or not same_source_episode):
            windows.append({"start_frame_index": index, "end_frame_index": index,
                            "frames": 1,
                            "source_run_id": rows[index].get("run_id"),
                            "source_episode_index": rows[index].get("episode_index"),
                            "source_start_monotonic_ns": rows[index]["state_source_local_monotonic_ns"],
                            "source_end_monotonic_ns": rows[index]["state_source_local_monotonic_ns"]})
        else:
            windows[-1]["end_frame_index"] = index
            windows[-1]["frames"] += 1
            windows[-1]["source_end_monotonic_ns"] = rows[index]["state_source_local_monotonic_ns"]
    for window in windows:
        window["duration_s"] = (window["frames"] - 1) / FPS
    return windows


def _validate_camera_metadata(records: dict[str, Any], expected_session_id: str) -> None:
    if not records["metadata"]:
        raise ValueError("rosbag contains no camera metadata")
    roles = {role: [] for role in ("base", "wrist")}
    for _, item in records["metadata"]:
        role = item.get("role")
        if role in roles:
            roles[role].append(item)
    for role, items in roles.items():
        if not items:
            raise ValueError(f"camera role {role} is missing")
        for item in items:
            if (item["encoding"] != "bgr8" or item["width"] != 640 or item["height"] != 480
                    or item["source_clock_id"] != "linux_perf_counter_ns"
                    or item["source_local_clock_id"] != "wsl_perf_counter_ns"
                    or item["clock_mapping_uncertainty_ns"] != 0
                    or item["session_id"] != expected_session_id):
                raise ValueError(f"unexpected {role} camera contract at sequence {item['sequence']}")


def _source_row(run_id: str, episode_index: int, frame_index: int,
                bag_ts: int, state: dict[str, Any], camera: dict[str, Any],
                action: dict[str, Any] | None, invalid_reason: str | None,
                base_image: dict[str, Any], wrist_image: dict[str, Any],
                task: str = TASK) -> dict[str, Any]:
    state_values = [float(v) for v in state["joint_position_rad"]] + [float(state["gripper_nominal_m"])]
    row = {
        "run_id": run_id,
        "episode_index": episode_index,
        "episode_frame_index": frame_index,
        "dataset_row_index": None,
        "state_source_local_monotonic_ns": state["source_local_monotonic_ns"],
        "task": task,
        "state": {
            "rosbag_timestamp_ns": bag_ts,
            "header_stamp_ns": state["header_stamp_ns"],
            "sequence": state["sequence"],
            "source_local_monotonic_ns": state["source_local_monotonic_ns"],
            "source_local_clock_id": state["source_local_clock_id"],
            "source_clock_id": state["source_clock_id"],
            "feedback_fresh": state["feedback_fresh"],
            "fake": state["fake"],
            "values_float32": state_values,
        },
        "images": {},
        "action_semantics": "socketcan_kernel_accepted_position_target_nominal_unverified",
        "action_valid": action is not None,
        "action_invalid_reason": invalid_reason,
        "action_placeholder": ([0.0] * 7 if action is None else None),
    }
    for role, image, camera_ref in (("base", base_image, camera["base"]),
                                    ("wrist", wrist_image, camera["wrist"])):
        row["images"][role] = {
            "header_stamp_ns": image["header_stamp_ns"],
            "rosbag_timestamp_ns": image["rosbag_timestamp_ns"],
            "source_sequence": camera_ref["sequence"],
            "source_local_monotonic_ns": camera_ref["source_local_monotonic_ns"],
            "state_delta_ns": camera_ref["delta_ns"],
            "rgb_pixels_sha256": image["rgb_pixels_sha256"],
        }
    if action is not None:
        row["action"] = {
            "request_id": action["request_id"],
            "operation": action["operation"],
            "source_state_sequence": action["source_state_sequence"],
            "source_state_monotonic_ns": action["source_state_monotonic_ns"],
            "first_position_accept_monotonic_ns": action["accepted_position_first_monotonic_ns"],
            "last_position_accept_monotonic_ns": action["accepted_position_monotonic_ns"],
            "action_age_ns": action["action_age_ns"],
            "action_age_clock_id": action["action_age_clock_id"],
            "target_joint_raw_0p001deg": action["joint_raw_0p001deg"],
            "target_gripper_raw": action["gripper_raw"],
            "target_nominal": raw_target_to_nominal_action(
                action["joint_raw_0p001deg"], action["gripper_raw"]),
            "gripper_effort_raw": action["gripper_effort_raw"],
            "accepted_frame_payloads": [
                {"can_id": frame["can_id"], "data_hex": frame["data_hex"],
                 "monotonic_ns": frame["monotonic_ns"]}
                for frame in action["accepted_frame_payloads"]
            ],
        }
    return row


class RerunPreviewWriter:
    def __init__(self, root: Path, run_id: str, task: str, *, recovered: bool = False):
        import rerun as rr

        self.rr = rr
        suffix = "_recovered" if recovered else ""
        stem = "dual_view_state_action" if run_id == RUN_ID else f"dual_view_state_action_{run_id}"
        self.output = root / "preview" / f"{stem}{suffix}.rrd"
        self.output.parent.mkdir(parents=True, exist_ok=True)
        rr.init(f"pi05_dataset_{run_id}{suffix}", spawn=False)
        rr.save(str(self.output))
        rr.log("metadata/task", rr.TextDocument(task))
        rr.log("metadata/action_semantics", rr.TextDocument(
            "Position target decoded from four CAN_KERNEL_ACCEPTED payloads. "
            "Kernel acceptance does not verify robot execution; physical units are nominal and uncalibrated."))

    def log_frame(self, index: int, row: dict[str, Any], state: dict[str, Any],
                  base_rgb: Any, wrist_rgb: Any, action_request: dict[str, Any] | None) -> None:
        import cv2
        import numpy as np

        rr = self.rr
        rr.set_time("frame_index", sequence=index)
        rr.log("observation/joint_state", rr.Scalars(np.asarray(
            state["joint_position_rad"] + [state["gripper_nominal_m"]], dtype=np.float64)))
        rr.log("observation/gripper_nominal_m", rr.Scalars(float(state["gripper_nominal_m"])))
        rr.log("quality/observation_valid", rr.Scalars(float(row["observation_valid"])))
        rr.log("quality/camera_valid", rr.Scalars(float(row["camera_valid"])))
        rr.log("quality/action_valid", rr.Scalars(float(row["action_valid"])))
        # Five hertz and 320x240 keeps the review recording modest while the
        # LeRobot video retains all full-resolution 10 Hz source pairs.
        if index % 2 == 0:
            rr.log("observation/images/base", rr.Image(
                cv2.resize(base_rgb, (320, 240), interpolation=cv2.INTER_AREA)))
            rr.log("observation/images/wrist", rr.Image(
                cv2.resize(wrist_rgb, (320, 240), interpolation=cv2.INTER_AREA)))
        if action_request is not None:
            target = raw_target_to_nominal_action(action_request["joint_raw_0p001deg"],
                                                  action_request["gripper_raw"])
            rr.log("action/target_joint_rad", rr.Scalars(np.asarray(target[:6], dtype=np.float64)))
            rr.log("action/target_gripper_nominal_m", rr.Scalars(float(target[6])))
            rr.log("action/action_age_ms", rr.Scalars(float(action_request["action_age_ns"]) / 1e6))
        else:
            rr.log("action/status", rr.TextLog("masked: no unique fresh complete kernel-accepted target"))

    def finish(self) -> dict[str, Any]:
        self.rr.disconnect()
        return {"path": self.output.relative_to(self.output.parent.parent).as_posix(),
                "bytes": self.output.stat().st_size, "sha256": sha256_file(self.output),
                "preview_image_rate_hz": FPS / 2, "preview_resolution": [320, 240],
                "camera_note": "Bag timestamp/source clock pairing only; not exposure synchronization."}


def _save_masked_action_stats(root: Path, episode_index: int, valid_actions: list[list[float]]) -> None:
    """Replace LeRobot's placeholder-inclusive action stats with valid-only stats."""
    import numpy as np
    from lerobot.common.datasets.utils import serialize_dict

    stats_path = root / "meta" / "episodes_stats.jsonl"
    lines = stats_path.read_text(encoding="utf-8").splitlines()
    output = []
    found = False
    for line in lines:
        record = json.loads(line)
        if record.get("episode_index") == episode_index:
            episode_stats = record.setdefault("stats", {})
            if valid_actions:
                values = np.asarray(valid_actions, dtype=np.float32)
                stats = {
                    "min": values.min(axis=0),
                    "max": values.max(axis=0),
                    "mean": values.mean(axis=0),
                    "std": values.std(axis=0),
                    "count": np.asarray([len(values)], dtype=np.int64),
                }
                episode_stats["action"] = serialize_dict(stats)
            else:
                # An observation-only episode has no action normalization stats.
                episode_stats.pop("action", None)
            found = True
        output.append(json.dumps(record, separators=(",", ":"), allow_nan=False))
    if not found:
        raise ValueError(f"LeRobot episode stats for episode {episode_index} are missing")
    temporary = stats_path.with_suffix(".jsonl.tmp")
    temporary.write_text("\n".join(output) + "\n", encoding="utf-8")
    os.replace(temporary, stats_path)


def _record_command_sidecar(path: Path, requests: list[dict[str, Any]], send_events: list[dict[str, Any]]) -> None:
    by_request: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for event in send_events:
        if event.get("request_id"):
            by_request[str(event["request_id"])].append(event)
    rows = []
    for request in requests:
        row = dict(request)
        row["all_send_events"] = by_request.get(request["request_id"], [])
        row["accepted_frame_payloads"] = [
            {"can_id": event["can_id"], "data_hex": event["data_hex"],
             "monotonic_ns": event["monotonic_ns"]}
            for event in by_request.get(request["request_id"], [])
            if event.get("event") == "CAN_KERNEL_ACCEPTED"
        ]
        rows.append(row)
    _jsonl_upsert(path, rows, ("request_id",))


def _validate_existing_dataset(root: Path, repo_id: str, expected_features: dict[str, Any]):
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id=repo_id, root=root, video_backend="pyav")
    if dataset.meta.fps != FPS:
        raise ValueError("cannot append: existing LeRobot dataset has a different fps")
    for key, expected in expected_features.items():
        actual = dataset.meta.features.get(key)
        if actual is None or actual.get("dtype") != expected["dtype"] or tuple(actual.get("shape", ())) != tuple(expected["shape"]):
            raise ValueError(f"cannot append: existing feature schema differs at {key}")
    # LeRobot 0.1.0 exposes add_frame/save_episode on its dataset object but
    # its public create() method only accepts a missing root. Re-arm the loaded
    # object with the same episode buffer used by create(), retaining its
    # on-disk metadata and loaded Parquet rows for append.
    dataset.image_writer = None
    dataset.image_transforms = None
    dataset.delta_timestamps = None
    dataset.delta_indices = None
    dataset.episode_data_index = None
    dataset.episodes = None
    dataset.episode_buffer = dataset.create_episode_buffer()
    return dataset


def export_run(manifest_path: Path, bag_root: Path | None, output_root: Path, *,
               resume: bool = False, max_action_age_ms: int = MAX_ACTION_AGE_MS,
               camera_match_max_ms: int = CAMERA_MATCH_MAX_MS, create_preview: bool = True) -> dict[str, Any]:
    output_root = Path(output_root).resolve()
    _configure_offline_cache(output_root.parent / ".offline_hf_cache")
    import cv2
    import numpy as np
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    if type(max_action_age_ms) is not int or max_action_age_ms < 0:
        raise ValueError("max_action_age_ms must be a non-negative integer")
    if type(camera_match_max_ms) is not int or camera_match_max_ms <= 0:
        raise ValueError("camera_match_max_ms must be a positive integer")
    runtime = _check_runtime()
    resolved_bag_root = resolve_manifest_bag_root(manifest_path, bag_root)
    source = verify_source(manifest_path, resolved_bag_root, verify_hashes=True)
    run_id = str(source["manifest"]["run_id"])
    task = source["manifest"].get("task")
    session_id = source["manifest"].get("session_id")
    if not isinstance(task, str) or not task.strip():
        raise ValueError("capture manifest must contain a non-empty task")
    if not isinstance(session_id, str) or not session_id:
        raise ValueError("capture manifest must contain a non-empty session_id")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    if output_root.exists() and not resume:
        raise FileExistsError(f"output exists; use --resume to append/recover with LeRobotDataset API: {output_root}")
    if not output_root.exists():
        available = shutil.disk_usage(output_root.parent).free
        reserve = 2_000_000_000
        if available < reserve:
            raise OSError(f"need at least {reserve} free bytes to stage one local export; available={available}")
    reader = RosbagReader(source["db_path"])
    try:
        for required in ("/piper/state_snapshot", "/piper/cameras/frame_metadata",
                         "/piper/cameras/base/image_raw", "/piper/cameras/wrist/image_raw",
                         "/gamepad/command_send_events", "/piper/command_receipt",
                         "/gamepad/target_events", "/gamepad/execution_events"):
            if required not in reader.topic_ids:
                raise ValueError(f"required rosbag topic is missing: {required}")
        records = _read_all_records(reader, session_id=session_id,
                                    max_action_age_ms=max_action_age_ms,
                                    camera_match_max_ms=camera_match_max_ms)
        _validate_camera_metadata(records, expected_session_id=session_id)
        state_pairs = records["states"]
        camera_pairs = records["camera_rows"]
        if len(camera_pairs) != len(state_pairs):
            raise AssertionError("camera pairing row count differs from state count")
        if any(not row["base"] or not row["wrist"] for row in camera_pairs):
            raise ValueError("a state row lacks either camera image metadata; refusing fabricated imagery")

        state_by_seq = records["state_by_seq"]
        actions_by_state, invalid_actions_by_state = _action_by_state(records)
        selected = {role: {int(row[role]["header_stamp_ns"]) for row in camera_pairs}
                    for role in ("base", "wrist")}
        if any(len(selected[role]) != len(camera_pairs) for role in selected):
            raise ValueError("nearest-camera matching reused a frame; refusing to duplicate images")

        h, w = int(records["metadata"][0][1]["height"]), int(records["metadata"][0][1]["width"])
        features = _feature_schema(h, w)
        has_existing = output_root.exists()
        pending_path = output_root / "provenance" / "pending_run.json"
        provenance = output_root / "provenance"
        run_sources_path = provenance / "run_sources.jsonl"
        existing_sources = []
        if run_sources_path.exists():
            existing_sources = [json.loads(line) for line in run_sources_path.read_text(encoding="utf-8").splitlines() if line]
        pending = None
        recovery_saved_episode = False
        if has_existing:
            if not resume:
                raise FileExistsError("output exists; pass --resume to append or recover a matching pending run")
            provenance.mkdir(parents=True, exist_ok=True)
            identity_path = provenance / "dataset_identity.json"
            if identity_path.exists():
                repo_id = json.loads(identity_path.read_text(encoding="utf-8"))["repo_id"]
            else:
                previous_report = provenance / "quality_report.json"
                if previous_report.exists():
                    repo_id = json.loads(previous_report.read_text(encoding="utf-8"))["dataset_repo_id"]
                elif existing_sources:
                    repo_id = f"piper/{existing_sources[0]['run_id']}"
                else:
                    raise ValueError("existing output has no dataset_identity.json, quality report, or source run")
            dataset = _validate_existing_dataset(output_root, repo_id, features)
            if pending_path.exists():
                pending = json.loads(pending_path.read_text(encoding="utf-8"))
                if pending.get("run_id") != run_id or pending.get("source_db_sha256") != source["db_sha256"]:
                    raise ValueError("pending run marker does not match this source bag")
                if pending.get("stage") == "converting":
                    raise RuntimeError(
                        "pending stage is 'converting': the LeRobot episode was not committed. "
                        "The CLI leaves its partial media untouched. Re-run this manifest and bag to a new empty "
                        "--output path; after verifying that export, keep this pending folder for manual review."
                    )
                if pending.get("stage") != "episode_saved":
                    raise RuntimeError(f"unsupported pending stage {pending.get('stage')!r}; inspect the pending marker")
                episode_index = int(pending["episode_index"])
                saved_frames = int(pending.get("saved_episode_frames", -1))
                if (episode_index >= dataset.meta.total_episodes or saved_frames != len(state_pairs)
                        or episode_index < 0):
                    raise RuntimeError("pending episode_saved marker does not match committed LeRobot episode metadata")
                recovery_saved_episode = True
            elif any(row.get("run_id") == run_id for row in existing_sources):
                raise ValueError(f"source run {run_id} is already complete in this dataset; refusing duplicate episode")
            else:
                episode_index = dataset.meta.total_episodes
        else:
            repo_id = f"piper/{run_id}"
            dataset = LeRobotDataset.create(
                repo_id=repo_id, root=output_root, fps=FPS, robot_type="piper",
                features=features, use_videos=True,
            )
            provenance.mkdir(parents=True, exist_ok=True)
            _json_write(provenance / "dataset_identity.json", {
                "schema": "piper_lerobot_dataset_identity_v1", "repo_id": repo_id,
                "created_from_run_id": run_id,
            })

        if not recovery_saved_episode:
            episode_index = dataset.meta.total_episodes
            pending_record = {"schema": "piper_dataset_pending_episode_v1", "run_id": run_id,
                              "episode_index": episode_index, "source_db_sha256": source["db_sha256"],
                              "stage": "converting", "started_from_resume": has_existing}
            _json_write(pending_path, pending_record)
        else:
            pending_record = pending
        dataset_row_start = int(dataset.meta.total_frames)
        if recovery_saved_episode:
            dataset_row_start -= int(pending_record["saved_episode_frames"])

        valid_actions: list[list[float]] = []
        source_map_rows: list[dict[str, Any]] = []
        report_action_ages: list[float] = []
        image_state_deltas: dict[str, list[float]] = {"base": [], "wrist": []}
        camera_pair_deltas: list[float] = []
        preview_writer = RerunPreviewWriter(
            output_root, run_id, task, recovered=recovery_saved_episode) if create_preview else None
        frame_stats: dict[str, Any] = {
            "state_fresh_count": 0, "camera_valid_count": 0,
            "observation_valid_count": 0, "action_valid_count": 0,
            "action_placeholder_count": 0,
        }
        selected_streams = {role: SelectedImageStream(reader, role, selected[role])
                            for role in ("base", "wrist")}
        for frame_index, ((state_bag_ns, state), camera_row) in enumerate(zip(state_pairs, camera_pairs)):
            seq = int(state["sequence"])
            base_bag_ns, base_header, base_rgb = selected_streams["base"].read(
                int(camera_row["base"]["header_stamp_ns"]))
            wrist_bag_ns, wrist_header, wrist_rgb = selected_streams["wrist"].read(
                int(camera_row["wrist"]["header_stamp_ns"]))
            if base_rgb.shape != (h, w, 3) or wrist_rgb.shape != (h, w, 3):
                raise ValueError(f"camera dimensions changed at state sequence {seq}")
            base_hash = hashlib.sha256(base_rgb.tobytes()).hexdigest()
            wrist_hash = hashlib.sha256(wrist_rgb.tobytes()).hexdigest()
            camera_valid = bool(camera_row["camera_valid"])
            state_valid = state.get("feedback_fresh") is True and state.get("fake") is False
            action_request = actions_by_state.get(seq) if state_valid and camera_valid else None
            action_reason = None
            if action_request is None:
                action_reason = invalid_actions_by_state.get(seq)
                if not state_valid:
                    action_reason = "state_feedback_not_fresh_or_fake"
                elif not camera_valid:
                    action_reason = "camera_pair_or_state_match_outside_limit"
                if action_reason is None:
                    action_reason = "no_unique_complete_kernel_accepted_target_for_state"
                action_values = [0.0] * 7
            else:
                action_values = raw_target_to_nominal_action(
                    action_request["joint_raw_0p001deg"], action_request["gripper_raw"])
                valid_actions.append(action_values)
                report_action_ages.append(action_request["action_age_ns"] / 1e6)
            observation_valid = bool(state_valid and camera_valid)
            frame_stats["state_fresh_count"] += int(state_valid)
            frame_stats["camera_valid_count"] += int(camera_valid)
            frame_stats["observation_valid_count"] += int(observation_valid)
            frame_stats["action_valid_count"] += int(action_request is not None)
            frame_stats["action_placeholder_count"] += int(action_request is None)
            for role, ref in (("base", camera_row["base"]), ("wrist", camera_row["wrist"])):
                image_state_deltas[role].append(float(ref["delta_ns"]) / 1e6)
            if camera_row["camera_pair_delta_ns"] is not None:
                camera_pair_deltas.append(float(camera_row["camera_pair_delta_ns"]) / 1e6)
            timestamp = frame_index / FPS
            frame = {
                "observation.images.base": base_rgb,
                "observation.images.wrist": wrist_rgb,
                "observation.state": np.asarray(
                    state["joint_position_rad"] + [state["gripper_nominal_m"]], dtype=np.float32),
                "action": np.asarray(action_values, dtype=np.float32),
                "action_valid": np.asarray([action_request is not None], dtype=np.bool_),
                "observation_valid": np.asarray([observation_valid], dtype=np.bool_),
                "camera_valid": np.asarray([camera_valid], dtype=np.bool_),
                "task": task,
            }
            if not recovery_saved_episode:
                dataset.add_frame(frame)
            row = _source_row(
                run_id, episode_index, frame_index, state_bag_ns, state, camera_row,
                action_request, action_reason,
                {"header_stamp_ns": base_header.stamp_ns, "rosbag_timestamp_ns": base_bag_ns,
                 "rgb_pixels_sha256": base_hash},
                {"header_stamp_ns": wrist_header.stamp_ns, "rosbag_timestamp_ns": wrist_bag_ns,
                 "rgb_pixels_sha256": wrist_hash},
                task=task,
            )
            row["dataset_row_index"] = dataset_row_start + frame_index
            row["observation_valid"] = observation_valid
            row["camera_valid"] = camera_valid
            row["frame_timestamp_s"] = timestamp
            source_map_rows.append(row)
            if preview_writer is not None:
                preview_writer.log_frame(frame_index, row, state, base_rgb, wrist_rgb, action_request)

        if not valid_actions:
            raise ValueError("no action-valid rows remain; refusing an empty-supervision LeRobot export")
        if not recovery_saved_episode:
            dataset.save_episode()
            pending_record["stage"] = "episode_saved"
            pending_record["saved_episode_frames"] = len(source_map_rows)
            _json_write(pending_path, pending_record)

        # Stats for `action` exclude all zero placeholders by construction.
        _save_masked_action_stats(output_root, episode_index, valid_actions)

        run_labels = {
            "schema": "piper_demo_outcome_label_v1",
            "run_id": run_id,
            "episode_index": episode_index,
            "demonstration_label": "incomplete-demonstration",
            "outcome": "unknown",
            "successful_complete_end": False,
            "complete_closeout_label_present": False,
            "delivery_incomplete": records["commands"]["incomplete_request_count"] > 0,
            "training_eligible": False,
            "robot_execution_verified": False,
            "label_note": "No successful complete-end label was present. One final hold request has an incomplete CAN send event.",
        }
        run_source = {
            "schema": "piper_lerobot_run_source_v1",
            "run_id": run_id,
            "episode_index": episode_index,
            "task": task,
            "manifest_path": str(source["manifest_path"]),
            "manifest_sha256": source["manifest_sha256"],
            "bag_path": str(source["bag_root"]),
            "bag_db_sha256": source["db_sha256"],
            "bag_db_bytes": source["db_bytes"],
            "action_semantics": "position target decoded from 4 CAN_KERNEL_ACCEPTED frames; not physical execution",
            "action_units": "six radian_nominal_unverified values and gripper meter_nominal_unverified",
            "fps": FPS,
            "source_state_rows": len(source_map_rows),
            "action_valid_rows": len(valid_actions),
            "formal_training_eligible": False,
            "label": run_labels,
        }
        _jsonl_upsert(run_sources_path, [run_source], ("run_id", "episode_index"))
        source_map_path = provenance / "source_map.jsonl"
        _jsonl_upsert(source_map_path, source_map_rows,
                      ("run_id", "episode_index", "episode_frame_index"))
        _jsonl_upsert(provenance / "command_requests.jsonl", records["commands"]["requests"],
                      ("request_id",))
        labels_path = provenance / "run_labels.jsonl"
        _jsonl_upsert(labels_path, [run_labels], ("run_id", "episode_index"))
        _json_write(provenance / f"{run_id}_valid_action_windows.json",
                    {"schema": "piper_action_valid_windows_v1",
                     "run_id": run_id,
                     "observation_and_action_valid_windows": _consecutive_windows(source_map_rows, "action_valid"),
                     "training_eligible": False,
                     "note": "Review-only intervals; do not stitch across missing action rows."})
        _json_write(provenance / f"{run_id}_outcome_label.json", run_labels)
        _record_command_sidecar(provenance / "command_send_event_map.jsonl",
                                records["commands"]["requests"], records["send_events"])

        preview_info = preview_writer.finish() if preview_writer is not None else None

        age_values = report_action_ages
        source_state_times_ns = [int(state["source_local_monotonic_ns"])
                                 for _, state in state_pairs]
        source_state_intervals_ms = [
            (right - left) / 1e6
            for left, right in zip(source_state_times_ns, source_state_times_ns[1:])
        ]
        report = {
            "schema": "piper_lerobot_kernel_accepted_quality_report_v1",
            "run_id": run_id,
            "dataset_repo_id": repo_id,
            "dataset_root": str(output_root),
            "dataset_episode_index": episode_index,
            "status": "REVIEW_EXPORT_CREATED_INCOMPLETE_DEMONSTRATION",
            "formal_training_eligible": False,
            "hardware_control_allowed": False,
            "robot_execution_verified": False,
            "manifest_action_semantics_conflict": {
                "manifest_claim": source["manifest"].get("command_recording_semantics"),
                "recorded_command_events": len(records["send_events"]),
                "complete_kernel_accepted_requests": records["commands"]["complete_request_count"],
                "incomplete_requests": records["commands"]["incomplete_request_count"],
                "resolution": "Decode accepted CAN payloads as targets; record kernel acceptance only, not physical execution.",
            },
            "source": {
                "manifest_path": str(source["manifest_path"]),
                "manifest_sha256": source["manifest_sha256"],
                "bag_root": str(source["bag_root"]),
                "db_path": str(source["db_path"]),
                "metadata_path": str(source["metadata_path"]),
                "db_sha256": source["db_sha256"],
                "metadata_sha256": source["metadata_sha256"],
                "db_bytes": source["db_bytes"],
            },
            "source_manifest_status": source["manifest"].get("status"),
            "manifest_training_eligible": source["manifest"].get("training_eligible"),
            "manifest_robot_motion_commanded_claim": source["manifest"].get("robot_motion_commanded"),
            "actual_recorded_kernel_send_evidence": {
                "complete_accepted_batches": records["commands"]["complete_request_count"],
                "incomplete_batches": records["commands"]["incomplete_request_count"],
                "accepted_frame_total": sum(row["kernel_accepted_frame_count"] for row in records["commands"]["requests"]),
                "physical_execution_verified": False,
                "delivery_incomplete_cutoff_monotonic_ns": records["commands"]["delivery_incomplete_cutoff_monotonic_ns"],
            },
            "rows": {
                "source_state_rows": len(state_pairs),
                "exported_le_robot_rows": len(source_map_rows),
                **frame_stats,
                "action_valid_fraction": frame_stats["action_valid_count"] / len(source_map_rows),
            },
            "source_state_timing": {
                "clock_id": "wsl_perf_counter_ns",
                "source_interval_ms_min": min(source_state_intervals_ms),
                "source_interval_ms_p50": _percentile(source_state_intervals_ms, 50),
                "source_interval_ms_p95": _percentile(source_state_intervals_ms, 95),
                "source_interval_ms_max": max(source_state_intervals_ms),
                "intervals_outside_100ms_plus_minus_5ms": sum(
                    abs(value - 100.0) > 5.0 for value in source_state_intervals_ms),
                "lerobot_timestamp_basis": "row_index / 10 Hz nominal FPS; original per-row source monotonic timestamp is preserved in provenance/source_map.jsonl",
                "timestamp_interpretation": "LeRobot uses a nominal 10 Hz index grid; it does not encode the observed source sampling jitter.",
            },
            "action_join": {
                "primary_join": "exact source_state_sequence from TARGET_INTENT to StateSnapshot sequence",
                "age_clock": "wsl_perf_counter_ns; event time is CAN_KERNEL_ACCEPTED monotonic_ns",
                "age_definition": "last accepted position frame timestamp minus matched source state local monotonic timestamp",
                "max_action_age_ms": max_action_age_ms,
                "age_ms_min": min(age_values) if age_values else None,
                "age_ms_p50": _percentile(age_values, 50),
                "age_ms_p95": _percentile(age_values, 95),
                "age_ms_max": max(age_values) if age_values else None,
                "missing_or_invalid_rows": frame_stats["action_placeholder_count"],
                "invalid_value_policy": "LeRobot review field uses zero sentinel plus action_valid=false; action statistics exclude invalid rows; all consumers must apply action_valid.",
                "no_hold_forward": True,
            },
            "camera_pairing": {
                "max_camera_state_delta_ms": camera_match_max_ms,
                "base_state_delta_abs_ms_max": max(abs(v) for v in image_state_deltas["base"]),
                "wrist_state_delta_abs_ms_max": max(abs(v) for v in image_state_deltas["wrist"]),
                "camera_pair_delta_ms_p50": _percentile(camera_pair_deltas, 50),
                "camera_pair_delta_ms_p95": _percentile(camera_pair_deltas, 95),
                "camera_pair_delta_ms_max": max(camera_pair_deltas) if camera_pair_deltas else None,
                "timestamp_note": "Pairing uses rosbag/header/perf-counter times; camera frames are not exposure synchronized.",
                "valid_mask": "camera_valid and observation_valid LeRobot columns; source_map records per-camera deltas.",
            },
            "units": {
                "joint_state": "radian nominal, hardware calibration unverified",
                "gripper_state": "meter nominal, hardware calibration unverified",
                "joint_action": "radian nominal target decoded from accepted CAN bytes, hardware calibration unverified",
                "gripper_action": "meter nominal target decoded from accepted CAN bytes, hardware calibration unverified",
                "hardware_unit_calibration_verified": False,
            },
            "runtime": runtime,
            "lerobot_schema": {
                "fps": FPS,
                "camera_keys": ["observation.images.base", "observation.images.wrist"],
                "state_feature": {"key": "observation.state", "dtype": "float32", "shape": [7], "names": STATE_NAMES},
                "action_feature": {"key": "action", "dtype": "float32", "shape": [7], "names": ACTION_NAMES},
                "masks": ["action_valid", "observation_valid", "camera_valid"],
                "invalid_action_sentinel": 0.0,
                "action_stats_source_rows": len(valid_actions),
            },
            "preview": preview_info,
        }
        payload_sha, payload_files = _sha256_tree(output_root, ("meta", "data", "videos"))
        report["lerobot_payload_sha256"] = payload_sha
        report["lerobot_payload_files_sha256"] = payload_files
        _json_write(provenance / f"{run_id}_quality_report.json", report)
        _json_write(provenance / "quality_report.json", report)
        # The pinned LeRobot loader's Parquet builder can retain its cache
        # lock for the lifetime of the writer object. Release it before the
        # independent readback audit opens the same local dataset in-process.
        del dataset
        import gc
        gc.collect()
        readback = _readback_audit(output_root, repo_id, episode_index, source_map_rows, valid_actions)
        _json_write(provenance / f"{run_id}_readback_audit.json", readback)
        _json_write(provenance / "readback_audit.json", readback)
        pending_path.unlink(missing_ok=True)
        return report
    finally:
        reader.close()


def _readback_audit(root: Path, repo_id: str, episode_index: int,
                    source_rows: list[dict[str, Any]], valid_actions: list[list[float]]) -> dict[str, Any]:
    """Read Parquet and two frames through the pinned LeRobot loader."""
    _configure_offline_cache(Path(root).resolve().parent / ".offline_hf_cache")
    import numpy as np
    import torch
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id=repo_id, root=root, video_backend="pyav")
    if dataset.meta.total_episodes <= episode_index:
        raise ValueError("LeRobot loader did not expose the just-exported episode")
    index_valid = next((i for i, row in enumerate(source_rows) if row["action_valid"]), None)
    if index_valid is None:
        raise ValueError("no valid action row for readback")
    selected_rows = sorted({0, index_valid})
    decoded = []
    for local_index in selected_rows:
        dataset_index = int(source_rows[local_index]["dataset_row_index"])
        sample = dataset[dataset_index]
        for key in ("observation.images.base", "observation.images.wrist"):
            image = sample[key]
            if tuple(image.shape) != (3, 480, 640):
                raise ValueError(f"LeRobot loader returned unexpected image shape {tuple(image.shape)}")
            if not bool(torch.isfinite(image).all()):
                raise ValueError("LeRobot loader returned non-finite image values")
        mask = bool(sample["action_valid"].reshape(-1)[0].item())
        if mask != bool(source_rows[local_index]["action_valid"]):
            raise ValueError(f"LeRobot action_valid mask changed at frame {local_index}")
        state = sample["observation.state"].numpy().astype(np.float32)
        if not np.allclose(state, source_rows[local_index]["state"]["values_float32"], rtol=0.0, atol=1e-6):
            raise ValueError(f"LeRobot state readback mismatch at frame {local_index}")
        if mask:
            action = sample["action"].numpy().astype(np.float32)
            expected = np.asarray(source_rows[local_index]["action"]["target_nominal"], dtype=np.float32)
            if not np.allclose(action, expected, rtol=0.0, atol=1e-6):
                raise ValueError(f"LeRobot action readback mismatch at frame {local_index}")
        decoded.append({"frame_index": local_index, "dataset_row_index": dataset_index,
                        "action_valid": mask})
    # Reload the metadata after valid-only stats patching; the action count must
    # equal valid supervision rows, never the masked zero sentinel rows.
    episode_stats = dataset.meta.episodes_stats[episode_index]
    action_count = int(episode_stats["action"]["count"].reshape(-1)[0])
    action_mean = np.asarray(episode_stats["action"]["mean"])
    if action_count != len(valid_actions) or not np.isfinite(action_mean).all():
        raise ValueError("LeRobot metadata action statistics include masked placeholders or are non-finite")
    report = {
        "schema": "piper_lerobot_kernel_accepted_readback_v1",
        "status": "PASS_REVIEW_ONLY_LOADER_READBACK",
        "dataset_repo_id": repo_id,
        "episode_index": episode_index,
        "frames": len(source_rows),
        "readback_samples": decoded,
        "source_state_values_verified": True,
        "valid_action_values_verified": True,
        "action_mask_verified": True,
        "decoded_rgb_videos_verified": True,
        "action_stats_count_valid_rows_only": action_count,
        "action_stats_finite": True,
        "formal_training_eligible": False,
        "note": "Loader and valid-only metadata stats read back successfully; capture remains an incomplete review demonstration.",
    }
    return report


def audit_dataset(root: Path) -> dict[str, Any]:
    root = Path(os.path.abspath(root))
    if not root.is_dir():
        raise FileNotFoundError(root)
    _configure_offline_cache(root.parent / ".offline_hf_cache")
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    source_path = root / "provenance" / "run_sources.jsonl"
    if not source_path.exists():
        raise FileNotFoundError(source_path)
    sources = [json.loads(line) for line in source_path.read_text(encoding="utf-8").splitlines() if line]
    if not sources:
        raise ValueError("dataset has no source sidecars")
    repo_id = (root / "provenance" / "quality_report.json")
    if repo_id.exists():
        repo_id_value = json.loads(repo_id.read_text(encoding="utf-8")).get("dataset_repo_id")
    else:
        identity_path = root / "provenance" / "dataset_identity.json"
        if identity_path.exists():
            repo_id_value = json.loads(identity_path.read_text(encoding="utf-8"))["repo_id"]
        else:
            repo_id_value = f"piper/{sources[0]['run_id']}"
    dataset = LeRobotDataset(repo_id=repo_id_value, root=root, video_backend="pyav")
    if dataset.meta.total_episodes != len(sources):
        raise ValueError("LeRobot episode count differs from run_sources sidecar")
    report = {"dataset_root": str(root), "repo_id": repo_id_value,
              "episodes": dataset.meta.total_episodes, "frames": dataset.meta.total_frames,
              "fps": dataset.meta.fps, "features": list(dataset.meta.features),
              "run_ids": [row["run_id"] for row in sources],
              "training_eligible": False}
    return report


def filter_export(source_root: Path, output_root: Path, *, only_action_valid: bool = True) -> dict[str, Any]:
    """Create a new dataset of fully valid contiguous 10 Hz windows.

    Missing rows are represented as boundaries between episodes; no timestamp
    gaps are compressed into a continuous trajectory. Source data is read-only.
    """
    source_root = Path(os.path.abspath(source_root))
    if not source_root.is_dir():
        raise FileNotFoundError(source_root)
    _configure_offline_cache(source_root.parent / ".offline_hf_cache")
    import numpy as np
    import torch
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    output_root = Path(os.path.abspath(output_root))
    if output_root.exists():
        raise FileExistsError(f"filtered export path already exists: {output_root}")
    report_path = source_root / "provenance" / "quality_report.json"
    if not report_path.exists():
        raise FileNotFoundError(report_path)
    source_report = json.loads(report_path.read_text(encoding="utf-8"))
    source_dataset = LeRobotDataset(source_report["dataset_repo_id"], root=source_root, video_backend="pyav")
    source_rows = [json.loads(line) for line in (source_root / "provenance" / "source_map.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(source_rows) != len(source_dataset):
        raise ValueError("source map row count differs from LeRobot data")
    feature_defs = source_dataset.meta.features
    # Do not export while any source rows have masked actions or this episode is incomplete.
    # This command is for precisely filtered review/analysis copies only.
    eligible_rows = [dict(row, _filter_eligible=(
        bool(row.get("action_valid")) and bool(row.get("observation_valid"))
        if only_action_valid else bool(row.get("observation_valid")))) for row in source_rows]
    contiguous = _consecutive_windows(eligible_rows, "_filter_eligible")
    for window in contiguous:
        window.pop("source_start_monotonic_ns", None)
        window.pop("source_end_monotonic_ns", None)
        window["source_frame_start"] = window.pop("start_frame_index")
        window["source_frame_end"] = window.pop("end_frame_index")
        window["start_frame_index"] = window["source_frame_start"]
        window["end_frame_index"] = window["source_frame_end"]
    if not contiguous:
        raise ValueError("no fully eligible rows found for the selected filter")
    # Preserve the exact feature and mask contract in an independent copy.
    feature_keys = ("observation.images.base", "observation.images.wrist", "observation.state",
                    "action", "action_valid", "observation_valid", "camera_valid")
    features = {key: dict(feature_defs[key]) for key in feature_keys}
    target_repo_id = f"piper/{output_root.name}"
    target = LeRobotDataset.create(repo_id=target_repo_id, root=output_root,
                                   fps=FPS, robot_type="piper", features=features, use_videos=True)
    source_labels_path = source_root / "provenance" / "run_labels.jsonl"
    source_sources_path = source_root / "provenance" / "run_sources.jsonl"
    if not source_labels_path.exists() or not source_sources_path.exists():
        raise FileNotFoundError("source dataset needs run_labels.jsonl and run_sources.jsonl sidecars")
    source_labels = {
        (row["run_id"], int(row["episode_index"])): row
        for row in (json.loads(line) for line in source_labels_path.read_text(encoding="utf-8").splitlines() if line)
    }
    source_sources = {
        (row["run_id"], int(row["episode_index"])): row
        for row in (json.loads(line) for line in source_sources_path.read_text(encoding="utf-8").splitlines() if line)
    }
    copied_map = []
    copied_labels = []
    copied_sources = []
    copied_actions = []
    copied_rows = 0
    for window in contiguous:
        episode_index = target.meta.total_episodes
        start, stop = window["start_frame_index"], window["end_frame_index"]
        source_key = (window["source_run_id"], int(window["source_episode_index"]))
        if source_key not in source_labels or source_key not in source_sources:
            raise ValueError(f"source run label/provenance missing for {source_key}")
        source_label = source_labels[source_key]
        source_record = source_sources[source_key]
        copied_label = dict(source_label)
        copied_label.update({
            "episode_index": episode_index,
            "derived_from_run_id": source_key[0],
            "derived_from_episode_index": source_key[1],
            "derived_source_frame_start": start,
            "derived_source_frame_end": stop,
            "training_eligible": False,
        })
        copied_labels.append(copied_label)
        copied_source = dict(source_record)
        copied_source.update({
            "episode_index": episode_index,
            "derived_from_run_id": source_key[0],
            "derived_from_episode_index": source_key[1],
            "derived_source_frame_start": start,
            "derived_source_frame_end": stop,
            "source_dataset": str(source_root),
            "formal_training_eligible": False,
        })
        copied_sources.append(copied_source)
        for source_index in range(start, stop + 1):
            sample = source_dataset[source_index]
            frame = {}
            for key in features:
                if key in ("observation.images.base", "observation.images.wrist"):
                    tensor = sample[key]
                    frame[key] = tensor.detach().cpu().mul(255).round().to(torch.uint8).permute(1, 2, 0).numpy()
                elif key in ("observation.state", "action"):
                    frame[key] = sample[key].detach().cpu().numpy().astype(np.float32)
                elif key in ("action_valid", "observation_valid", "camera_valid"):
                    frame[key] = np.asarray(
                        sample[key].detach().cpu().numpy(), dtype=np.bool_).reshape(1)
            frame["task"] = str(source_rows[source_index].get("task") or source_record.get("task") or TASK)
            target.add_frame(frame)
            row = dict(source_rows[source_index])
            row["source_dataset_row_index"] = source_index
            row["dataset_row_index"] = copied_rows
            row["episode_index"] = episode_index
            row["episode_frame_index"] = source_index - start
            copied_map.append(row)
            if row["action_valid"]:
                copied_actions.append(row["action"]["target_nominal"])
            copied_rows += 1
        target.save_episode()
        _save_masked_action_stats(output_root, episode_index,
                                  [row["action"]["target_nominal"] for row in copied_map
                                   if row["episode_index"] == episode_index and row["action_valid"]])
    filtered_provenance = output_root / "provenance"
    filtered_provenance.mkdir(parents=True, exist_ok=True)
    _jsonl_write(filtered_provenance / "source_map.jsonl", copied_map)
    _jsonl_write(filtered_provenance / "run_labels.jsonl", copied_labels)
    _jsonl_write(filtered_provenance / "run_sources.jsonl", copied_sources)
    _json_write(filtered_provenance / "dataset_identity.json", {
        "schema": "piper_lerobot_dataset_identity_v1", "repo_id": target_repo_id,
        "derived_from_dataset": str(source_root),
    })
    _json_write(filtered_provenance / "filter_export.json", {
        "schema": "piper_lerobot_action_valid_filter_v1",
        "source_dataset": str(source_root),
        "source_repo_id": source_report["dataset_repo_id"],
        "output_repo_id": target_repo_id,
        "filter": "action_valid AND observation_valid" if only_action_valid else "observation_valid",
        "contiguous_valid_windows": contiguous,
        "episode_count": len(contiguous),
        "frame_count": copied_rows,
        "source_run_labels_preserved": bool(copied_labels),
        "run_labels_sidecar": "provenance/run_labels.jsonl",
        "run_sources_sidecar": "provenance/run_sources.jsonl",
        "training_eligible": False,
        "note": "Each maximal contiguous 10 Hz window is a separate episode. Gaps are not spliced into trajectories.",
    })
    filtered_report = {
        "schema": "piper_lerobot_filtered_review_dataset_v1",
        "dataset_repo_id": target_repo_id,
        "dataset_root": str(output_root),
        "derived_from_dataset": str(source_root),
        "episodes": len(contiguous),
        "frames": copied_rows,
        "formal_training_eligible": False,
        "robot_execution_verified": False,
        "source_labels_copied": len(copied_labels),
    }
    payload_sha, payload_files = _sha256_tree(output_root, ("meta", "data", "videos"))
    filtered_report["lerobot_payload_sha256"] = payload_sha
    filtered_report["lerobot_payload_files_sha256"] = payload_files
    _json_write(filtered_provenance / "quality_report.json", filtered_report)
    return {"output_root": str(output_root), "episodes": len(contiguous), "frames": copied_rows,
            "action_valid_rows": len(copied_actions), "training_eligible": False}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pi05-dataset", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    export = sub.add_parser("export", help="convert the scoped rosbag to an offline LeRobot dataset")
    export.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    export.add_argument("--bag", type=Path,
                        help="local archived bag folder; default resolves the archive path from the manifest")
    export.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    export.add_argument("--resume", action="store_true", help="open existing data through LeRobotDataset and append")
    export.add_argument("--max-action-age-ms", type=int, default=MAX_ACTION_AGE_MS)
    export.add_argument("--camera-match-max-ms", type=int, default=CAMERA_MATCH_MAX_MS)
    export.add_argument("--no-preview", action="store_true", help="skip Rerun preview generation")
    audit = sub.add_parser("audit", help="load a local LeRobot dataset and check source sidecars")
    audit.add_argument("--dataset", type=Path, default=DEFAULT_OUTPUT)
    filtered = sub.add_parser("filter-export", help="export fully valid contiguous windows to a new dataset")
    filtered.add_argument("--dataset", type=Path, default=DEFAULT_OUTPUT)
    filtered.add_argument("--output", type=Path, required=True)
    filtered.add_argument("--observation-only", action="store_true",
                          help="filter only observation_valid rows; action remains separately masked")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == "export":
        report = export_run(args.manifest, args.bag, args.output,
                            resume=args.resume,
                            max_action_age_ms=args.max_action_age_ms,
                            camera_match_max_ms=args.camera_match_max_ms,
                            create_preview=not args.no_preview)
    elif args.command == "audit":
        report = audit_dataset(args.dataset)
    else:
        report = filter_export(args.dataset, args.output,
                               only_action_valid=not args.observation_only)
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
