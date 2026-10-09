"""Fail-closed audit for the derived-state, multi-demonstration pilot dataset.

This contract accepts only offline next-observed-state targets. It deliberately
does not satisfy raw-command training acceptance or authorize robot control.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re


ACTION_SEMANTICS = "next_observed_state_proxy_nominal_units"
ACTION_TARGET_KIND = "derived_future_observed_state_position_target"
ACTION_TIME_RELATION = "observation_state_at_t_to_observed_state_at_t_plus_delta"
EXPECTED_FPS = 10
EXPECTED_INTERVAL_NS = 100_000_000
EXPECTED_DATASETS_VERSION = "3.6.0"
EXPECTED_LEROBOT_VERSION = "0.1.0"
EXPECTED_LEROBOT_REVISION = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dataset_tree_sha256(root: Path) -> str:
    root = Path(root).resolve(strict=True)
    if not root.is_dir():
        raise ValueError("dataset path must be a directory")
    entries = []
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"dataset tree contains a symlink: {path}")
        if path.is_file():
            entries.append((path.relative_to(root).as_posix(), path))
    if not entries:
        raise ValueError("dataset tree is empty")
    digest = hashlib.sha256()
    for relative, path in sorted(entries):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path.name}")
    return value


def _jsonl(path: Path) -> list[dict]:
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        row = json.loads(line)
        if not isinstance(row, dict):
            raise ValueError(f"expected an object at {path.name}:{line_number}")
        rows.append(row)
    if not rows:
        raise ValueError(f"empty JSONL: {path.name}")
    return rows


def _valid_sha(value: object) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def inspect_proxy_multi_episode(dataset_path: Path, *, min_episodes: int = 5,
                                expected_dataset_sha256: str | None = None) -> dict:
    """Validate successful 10fps derived-state episodes and full readback."""
    if type(min_episodes) is not int or min_episodes < 1:
        raise ValueError("derived-state offline fitting requires at least one episode")
    root = Path(dataset_path).resolve(strict=True)
    info_path = root / "meta" / "info.json"
    conversion_path = root / "provenance" / "proxy_conversion.json"
    source_map_path = root / "provenance" / "source_map.jsonl"
    source_report_path = root / "provenance" / "source_map_report.json"
    readback_path = root / "provenance" / "readback_audit.json"
    readback_rows_path = root / "provenance" / "readback_frames.jsonl"
    required = (info_path, conversion_path, source_map_path, source_report_path,
                readback_path, readback_rows_path)
    if any(not path.is_file() for path in required):
        raise ValueError("dataset is missing converter or row-level readback provenance")

    info = _json(info_path)
    conversion = _json(conversion_path)
    source_report = _json(source_report_path)
    audit = _json(readback_path)
    source_map = _jsonl(source_map_path)
    readback_rows = _jsonl(readback_rows_path)
    repo_id = conversion.get("repo_id")
    if not isinstance(repo_id, str) or len(repo_id.split("/")) != 2 or root.name != repo_id.split("/")[-1]:
        raise ValueError("dataset directory does not match its LeRobot repo ID")
    if (conversion.get("schema") != "piper_proxy_lerobot_multi_episode_v1"
            or conversion.get("action_semantics") != ACTION_SEMANTICS
            or conversion.get("action_target_kind") != ACTION_TARGET_KIND
            or conversion.get("action_time_relation") != ACTION_TIME_RELATION
            or conversion.get("raw_command_supervision") is not False
            or conversion.get("formal_training_eligible") is not False
            or conversion.get("hardware_control_allowed") is not False
            or conversion.get("prediction_only") is not True
            or conversion.get("split") != "train_only"
            or conversion.get("joint_units") != "radian_nominal_unverified"
            or conversion.get("gripper_units") != "meter_nominal_unverified"):
        raise ValueError("dataset is not an isolated derived-state train-only pilot")
    runtime = conversion.get("conversion_runtime", {})
    if (runtime.get("datasets_version") != EXPECTED_DATASETS_VERSION
            or runtime.get("lerobot_version") != EXPECTED_LEROBOT_VERSION
            or runtime.get("lerobot_revision") != EXPECTED_LEROBOT_REVISION):
        raise ValueError("dataset conversion was not produced by the pinned OpenPI-compatible runtime")

    episode_count = conversion.get("episodes")
    frame_count = conversion.get("frames")
    episodes = conversion.get("source_episodes")
    if (type(episode_count) is not int or episode_count < min_episodes
            or not isinstance(episodes, list) or len(episodes) != episode_count
            or type(frame_count) is not int or frame_count <= 0):
        raise ValueError("dataset must contain at least the required number of demonstrations")
    if (info.get("robot_type") != "piper" or info.get("fps") != EXPECTED_FPS
            or info.get("total_episodes") != episode_count
            or info.get("total_frames") != frame_count
            or info.get("features", {}).get("observation.state", {}).get("shape") != [7]
            or info.get("features", {}).get("action", {}).get("shape") != [7]):
        raise ValueError("LeRobot metadata does not match the expected 10fps PiPER 7D dataset")

    ids = []
    expected_episode_frames = {}
    for index, episode in enumerate(episodes):
        if (not isinstance(episode, dict) or episode.get("episode_index") != index
                or episode.get("outcome_success") is not True or episode.get("split") != "train"
                or type(episode.get("frames")) is not int or episode["frames"] < 2):
            raise ValueError(f"source episode {index} is not a successful train-only demonstration")
        episode_id = episode.get("raw_episode_id")
        if not isinstance(episode_id, str) or not episode_id or episode_id in ids:
            raise ValueError("raw source episode IDs must be present and unique")
        for field in ("raw_manifest_sha256", "aligned_metadata_sha256", "aligned_frames_sha256",
                      "frame_map_sha256", "outcome_label_sha256", "proxy_manifest_sha256",
                      "proxy_targets_sha256"):
            if not _valid_sha(episode.get(field)):
                raise ValueError(f"source episode {index} has invalid {field}")
        ids.append(episode_id)
        expected_episode_frames[index] = episode["frames"]
    if sum(expected_episode_frames.values()) != frame_count:
        raise ValueError("source episode frame counts do not sum to total frames")

    source_map_sha = _sha256(source_map_path)
    readback_sha = _sha256(readback_rows_path)
    if (source_report.get("schema") != "piper_proxy_lerobot_multi_source_map_v1"
            or source_report.get("source_map_rows") != frame_count
            or source_report.get("source_episodes") != episodes
            or source_report.get("source_map_sha256") != source_map_sha
            or source_report.get("action_semantics") != ACTION_SEMANTICS
            or source_report.get("formal_training_eligible") is not False
            or source_report.get("hardware_control_allowed") is not False
            or conversion.get("source_map_sha256") != source_map_sha
            or conversion.get("source_map_rows") != frame_count):
        raise ValueError("multi-episode source map identity or hash mismatch")
    if (audit.get("schema") != "piper_proxy_lerobot_multi_readback_audit_v1"
            or audit.get("status") != "PASS_READBACK"
            or audit.get("dataset_repo_id") != repo_id
            or audit.get("dataset_path") != root.name
            or audit.get("fps") != EXPECTED_FPS
            or audit.get("episodes") != episode_count
            or audit.get("frames") != frame_count
            or audit.get("split") != "train_only"
            or audit.get("action_semantics") != ACTION_SEMANTICS
            or audit.get("source_frame_extraction_verified") is not True
            or audit.get("state_values_verified") is not True
            or audit.get("action_values_verified") is not True
            or audit.get("episode_and_frame_order_verified") is not True
            or audit.get("decoded_rgb_frames_verified") is not True
            or audit.get("source_map_sha256") != source_map_sha
            or audit.get("readback_frames_sha256") != readback_sha
            or audit.get("dataset_info_sha256") != _sha256(info_path)
            or audit.get("formal_training_eligible") is not False
            or audit.get("hardware_control_allowed") is not False):
        raise ValueError("complete LeRobot readback audit is required before training")
    if len(source_map) != frame_count or len(readback_rows) != frame_count:
        raise ValueError("source or readback row count mismatch")

    episode_next_frame = {index: 0 for index in range(episode_count)}
    for row_index, (source, readback) in enumerate(zip(source_map, readback_rows)):
        episode_index = source.get("episode_index")
        if (source.get("dataset_row_index") != row_index
                or type(episode_index) is not int or episode_index not in episode_next_frame
                or source.get("episode_frame_index") != episode_next_frame[episode_index]
                or source.get("raw_episode_id") != ids[episode_index]
                or source.get("action_semantics") != ACTION_SEMANTICS):
            raise ValueError(f"source map episode/order mismatch at dataset row {row_index}")
        episode_next_frame[episode_index] += 1
        observation = source.get("observation", {})
        target = source.get("proxy_target", {})
        observation_ts, target_ts = observation.get("timestamp_ns"), target.get("timestamp_ns")
        if (type(observation_ts) is not int or type(target_ts) is not int
                or target_ts - observation_ts != EXPECTED_INTERVAL_NS):
            raise ValueError(f"source action target is not exactly next 10fps state at row {row_index}")
        for role in ("base", "wrist"):
            image_info = observation.get("images", {}).get(role, {})
            if not isinstance(image_info.get("preview_relpath"), str) or not _valid_sha(image_info.get("rgb_pixels_sha256")):
                raise ValueError(f"source image provenance is incomplete at row {row_index}")
        for key in ("state_float32_sha256",):
            if not _valid_sha(observation.get(key)):
                raise ValueError(f"source state provenance is incomplete at row {row_index}")
        if not _valid_sha(target.get("action_float32_sha256")):
            raise ValueError(f"source action provenance is incomplete at row {row_index}")
        if (readback.get("dataset_row_index") != row_index
                or readback.get("episode_index") != episode_index
                or readback.get("episode_frame_index") != source.get("episode_frame_index")
                or readback.get("raw_episode_id") != source.get("raw_episode_id")
                or readback.get("state_float32_sha256") != observation.get("state_float32_sha256")
                or readback.get("action_float32_sha256") != target.get("action_float32_sha256")):
            raise ValueError(f"LeRobot readback row is not bound to its source map at {row_index}")
        for role in ("base", "wrist"):
            for suffix in ("source", "decoded"):
                if not _valid_sha(readback.get(f"{role}_{suffix}_rgb_sha256")):
                    raise ValueError(f"LeRobot {role} readback hash is missing at row {row_index}")
            error = readback.get(f"{role}_mean_absolute_pixel_error")
            if isinstance(error, bool) or not isinstance(error, (float, int)) or not math.isfinite(error):
                raise ValueError(f"LeRobot {role} readback pixel error is invalid at row {row_index}")
    if episode_next_frame != expected_episode_frames:
        raise ValueError("source map frames per episode differ from the episode manifest")

    media = audit.get("media_and_parquet_sha256")
    if not isinstance(media, dict) or not media:
        raise ValueError("readback audit lacks media/Parquet file hashes")
    for relative, expected_hash in media.items():
        if not _valid_sha(expected_hash):
            raise ValueError("readback audit contains an invalid media hash")
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file() or _sha256(path) != expected_hash:
            raise ValueError(f"readback media hash mismatch: {relative}")

    dataset_sha = dataset_tree_sha256(root)
    if expected_dataset_sha256 is not None:
        if not _valid_sha(expected_dataset_sha256) or dataset_sha != expected_dataset_sha256:
            raise ValueError("dataset tree hash differs from the independently supplied identity")
    return {
        "status": "READY_FOR_OFFLINE_DERIVED_STATE_PILOT",
        "repo_id": repo_id,
        "dataset_path": str(root),
        "dataset_sha256": dataset_sha,
        "fps": EXPECTED_FPS,
        "episodes": episode_count,
        "frames": frame_count,
        "split": "train_only",
        "successful_episodes": episode_count,
        "action_semantics": ACTION_SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "raw_command_supervision": False,
        "units_verified_for_hardware": False,
        "formal_training_eligible": False,
        "offline_training_allowed": True,
        "prediction_only": True,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }
