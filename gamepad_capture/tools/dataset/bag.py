"""Streaming rosbag2 SQLite reader and source-identity checks."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
from typing import Any, Iterator

from .cdr import (
    decode_camera_metadata,
    decode_image,
    decode_image_header,
    decode_state_snapshot,
    decode_std_string,
)


RUN_ID = "20261017T113409Z_72321bfd"
DEFAULT_MANIFEST = Path("gamepad_capture/data/manifests") / f"{RUN_ID}.json"
DEFAULT_BAG = Path("gamepad_capture/data/rosbag") / f"piper_operator_demo_{RUN_ID}"


def sha256_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return value


def resolve_manifest_bag_root(manifest_path: Path, bag_root: Path | None = None) -> Path:
    """Resolve the archived bag declared by a capture manifest.

    An explicit CLI path always wins. Otherwise use the manifest's archive
    location, translating the common WSL ``/mnt/<drive>/...`` form on
    Windows, then fall back to the archive basename beside the project data
    folders. The run ID is never used as a hard-coded source selector.
    """
    if bag_root is not None:
        return Path(bag_root).resolve()
    manifest = load_json_object(manifest_path)
    archive_path = manifest.get("archive", {}).get("archive_path")
    candidates: list[Path] = []
    if isinstance(archive_path, str) and archive_path:
        if os.name == "nt":
            match = re.fullmatch(r"/mnt/([A-Za-z])/(.+)", archive_path)
            if match:
                candidates.append(Path(f"{match.group(1).upper()}:/{match.group(2)}"))
        candidates.append(Path(archive_path))
        basename = Path(archive_path.replace("\\", "/")).name
        if basename:
            candidates.append(Path(manifest_path).resolve().parent.parent / "rosbag" / basename)
    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(
        f"could not find the archived bag declared in {manifest_path}; "
        "pass its local archive folder with --bag"
    )


def verify_source(manifest_path: Path, bag_root: Path, *, verify_hashes: bool = True) -> dict[str, Any]:
    manifest = load_json_object(manifest_path)
    run_id = manifest.get("run_id")
    if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,96}", run_id):
        raise ValueError("manifest run_id must contain 1-96 ASCII letters, digits, '_' or '-'")
    if manifest.get("storage_id") != "sqlite3":
        raise ValueError("expected a sqlite3 rosbag2 archive")
    archive = manifest.get("archive", {})
    expected_files = archive.get("files", {})
    db_files = sorted(Path(bag_root).glob("*.db3"))
    if len(db_files) != 1:
        raise ValueError(f"expected one db3 file under {bag_root}, found {len(db_files)}")
    db_path = db_files[0]
    expected_db = expected_files.get(db_path.name)
    metadata_path = Path(bag_root) / "metadata.yaml"
    expected_metadata = expected_files.get("metadata.yaml")
    if not isinstance(expected_db, dict) or not isinstance(expected_metadata, dict):
        raise ValueError("source manifest does not declare hashes for the db3 and metadata.yaml")
    if db_path.stat().st_size != expected_db.get("bytes"):
        raise ValueError("db3 size differs from the capture manifest")
    if metadata_path.stat().st_size != expected_metadata.get("bytes"):
        raise ValueError("metadata.yaml size differs from the capture manifest")
    actual_db_sha = sha256_file(db_path) if verify_hashes else None
    actual_meta_sha = sha256_file(metadata_path) if verify_hashes else None
    if verify_hashes and (actual_db_sha != expected_db.get("sha256")
                          or actual_meta_sha != expected_metadata.get("sha256")):
        raise ValueError("source bag hash does not match the capture manifest")
    return {
        "manifest": manifest,
        "manifest_path": Path(manifest_path).resolve(),
        "manifest_sha256": sha256_file(manifest_path),
        "bag_root": Path(bag_root).resolve(),
        "db_path": db_path.resolve(),
        "metadata_path": metadata_path.resolve(),
        "db_sha256": actual_db_sha or expected_db["sha256"],
        "metadata_sha256": actual_meta_sha or expected_metadata["sha256"],
        "db_bytes": db_path.stat().st_size,
    }


class RosbagReader:
    """Read one message at a time from an immutable SQLite rosbag2 archive."""

    def __init__(self, db_path: Path):
        db_uri = Path(db_path).resolve().as_uri() + "?mode=ro&immutable=1"
        self.connection = sqlite3.connect(db_uri, uri=True)
        self.connection.row_factory = sqlite3.Row
        self.topics = {
            str(row["name"]): {"id": int(row["id"]), "type": str(row["type"])}
            for row in self.connection.execute("SELECT id, name, type FROM topics")
        }
        self.topic_ids = {name: int(info["id"]) for name, info in self.topics.items()}
        self.counts = {
            str(row[0]): int(row[1]) for row in self.connection.execute(
                "SELECT topics.name, COUNT(messages.id) "
                "FROM topics LEFT JOIN messages ON topics.id=messages.topic_id "
                "GROUP BY topics.name"
            )
        }

    def close(self) -> None:
        self.connection.close()

    def iter_topic(self, topic: str) -> Iterator[tuple[int, bytes]]:
        if topic not in self.topic_ids:
            raise ValueError(f"required rosbag topic is missing: {topic}")
        cursor = self.connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp, id",
            (self.topic_ids[topic],),
        )
        for timestamp_ns, data in cursor:
            yield int(timestamp_ns), data

    def iter_images(self, topic: str, needed_header_stamps: set[int]) -> Iterator[tuple[int, Any, dict[str, Any]]]:
        for bag_timestamp_ns, data in self.iter_topic(topic):
            header = decode_image_header(data)
            if header.stamp_ns not in needed_header_stamps:
                continue
            parsed_header, image = decode_image(data)
            if parsed_header != header:
                raise AssertionError("image header changed while decoding")
            yield bag_timestamp_ns, header, image

    def low_rate_records(self) -> dict[str, Any]:
        """Collect states, camera metadata, and event logs; never retain images."""
        states = [(timestamp, decode_state_snapshot(data)) for timestamp, data in
                  self.iter_topic("/piper/state_snapshot")]
        metadata = [(timestamp, decode_camera_metadata(data)) for timestamp, data in
                    self.iter_topic("/piper/cameras/frame_metadata")]
        send_events = [(timestamp, json.loads(decode_std_string(data))) for timestamp, data in
                       self.iter_topic("/gamepad/command_send_events")]
        receipts = [(timestamp, decode_command_receipt_bytes(data)) for timestamp, data in
                    self.iter_topic("/piper/command_receipt")]
        other_events: dict[str, list[dict[str, Any]]] = {}
        for topic in ("/gamepad/target_events", "/gamepad/execution_events"):
            other_events[topic] = [
                {"bag_timestamp_ns": timestamp, "payload": json.loads(decode_std_string(data))}
                for timestamp, data in self.iter_topic(topic)
            ]
        return {
            "states": states,
            "metadata": metadata,
            "send_events": [record for _, record in send_events],
            "receipts": [record for _, record in receipts],
            "other_events": other_events,
        }


def decode_command_receipt_bytes(data: bytes) -> dict[str, Any]:
    # Kept local to avoid a larger public CDR symbol list.
    from .cdr import decode_command_receipt

    return decode_command_receipt(data)


def match_camera_frames(states: list[tuple[int, dict[str, Any]]],
                        metadata: list[tuple[int, dict[str, Any]]], *,
                        max_delta_ns: int = 60_000_000) -> list[dict[str, Any]]:
    """Nearest source-clock image metadata per state row, one use per frame."""
    import bisect

    by_role: dict[str, list[dict[str, Any]]] = {"base": [], "wrist": []}
    for bag_ts, item in metadata:
        role = item["role"]
        if role in by_role:
            by_role[role].append({**item, "metadata_bag_timestamp_ns": bag_ts})
    for items in by_role.values():
        items.sort(key=lambda item: int(item["source_local_monotonic_ns"]))
    used: dict[str, set[int]] = {"base": set(), "wrist": set()}
    output = []
    for state_index, (_, state) in enumerate(states):
        source_ns = int(state["source_local_monotonic_ns"])
        selected: dict[str, dict[str, Any] | None] = {}
        for role, items in by_role.items():
            times = [int(item["source_local_monotonic_ns"]) for item in items]
            pos = bisect.bisect_left(times, source_ns)
            candidate_indices = sorted({i for i in (pos - 1, pos) if 0 <= i < len(items)},
                                       key=lambda i: abs(times[i] - source_ns))
            chosen = next((i for i in candidate_indices if i not in used[role]), None)
            if chosen is None:
                selected[role] = None
            else:
                selected[role] = items[chosen]
                used[role].add(chosen)
        row = {"state_index": state_index, "state_source_local_monotonic_ns": source_ns}
        for role in ("base", "wrist"):
            item = selected[role]
            row[role] = None if item is None else {
                **item,
                "delta_ns": int(item["source_local_monotonic_ns"]) - source_ns,
                "valid": abs(int(item["source_local_monotonic_ns"]) - source_ns) <= max_delta_ns,
            }
        pair_delta_ns = None
        if selected["base"] is not None and selected["wrist"] is not None:
            pair_delta_ns = abs(int(selected["base"]["source_local_monotonic_ns"])
                                - int(selected["wrist"]["source_local_monotonic_ns"]))
        row["camera_pair_delta_ns"] = pair_delta_ns
        row["camera_valid"] = bool(
            row["base"] and row["base"]["valid"] and row["wrist"] and row["wrist"]["valid"]
            and pair_delta_ns is not None and pair_delta_ns <= max_delta_ns
        )
        output.append(row)
    return output
