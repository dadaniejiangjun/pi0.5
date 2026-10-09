"""Attach an immutable outcome label outside an already closed raw episode."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from laptop.recorder.episode import DEMONSTRATION_SOURCES


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def label_episode(raw_episode: Path, output_dir: Path, *, success: bool,
                  notes: str, operator: str) -> dict:
    raw_episode = Path(raw_episode).resolve()
    output_dir = Path(output_dir).resolve()
    if output_dir.is_relative_to(raw_episode) or raw_episode.is_relative_to(output_dir):
        raise ValueError("outcome label must be outside the immutable raw episode")
    if type(success) is not bool or not operator.strip() or not notes.strip():
        raise ValueError("explicit outcome, operator, and notes are required")
    manifest_path = raw_episode / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    metadata = json.loads((raw_episode / "metadata.json").read_text(encoding="utf-8"))
    if (manifest.get("schema") != "piper_raw_manifest_v1"
            or manifest.get("episode_id") != raw_episode.name
            or manifest.get("source") not in DEMONSTRATION_SOURCES
            or metadata.get("episode_id") != raw_episode.name
            or metadata.get("source") not in DEMONSTRATION_SOURCES):
        raise ValueError("only a closed operator demonstration may receive a task label")
    files = manifest.get("files")
    if not isinstance(files, dict) or not files or "metadata.json" not in files:
        raise ValueError("raw manifest has no verifiable file inventory")
    for relative, record in files.items():
        path = (raw_episode / relative).resolve()
        if (not path.is_relative_to(raw_episode) or not path.is_file()
                or not isinstance(record, dict) or _sha256(path) != record.get("sha256")):
            raise ValueError(f"raw file integrity failed: {relative}")
    label = {
        "schema": "piper_episode_outcome_v1",
        "episode_id": raw_episode.name,
        "raw_manifest_sha256": _sha256(manifest_path),
        "success": success,
        "notes": notes.strip(),
        "operator": operator.strip(),
        "labeled_utc": datetime.now(timezone.utc).isoformat(),
        "action_target_verified": False,
        "training_eligible": False,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{raw_episode.name}.json"
    with path.open("x", encoding="utf-8") as handle:
        json.dump(label, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return {"path": str(path), **label}


def verify_label(raw_episode: Path, label_path: Path) -> dict:
    raw_episode = Path(raw_episode).resolve()
    label = json.loads(Path(label_path).read_text(encoding="utf-8"))
    if (label.get("schema") != "piper_episode_outcome_v1"
            or label.get("episode_id") != raw_episode.name
            or label.get("raw_manifest_sha256") != _sha256(raw_episode / "manifest.json")
            or type(label.get("success")) is not bool):
        raise ValueError("outcome label does not match this raw episode")
    return {"status": "OUTCOME_LABEL_BOUND", "success": label["success"],
            "training_eligible": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_episode", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("datasets/manifests/outcomes"))
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument("--success", action="store_true")
    choice.add_argument("--failure", action="store_true")
    parser.add_argument("--notes", required=True)
    parser.add_argument("--operator", required=True)
    args = parser.parse_args()
    report = label_episode(args.raw_episode, args.output_dir, success=args.success,
                           notes=args.notes, operator=args.operator)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
