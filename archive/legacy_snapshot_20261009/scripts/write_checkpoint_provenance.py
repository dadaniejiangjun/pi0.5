#!/usr/bin/env python3
"""Inventory and hash the project-local pi05_base checkpoint cache."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("cache/openpi/openpi-assets/checkpoints/pi05_base"))
    parser.add_argument("--output", type=Path, default=Path("provenance/pi05_base_checkpoint.json"))
    args = parser.parse_args()
    root = args.root.resolve()
    if not root.is_dir() or not (root / "params/commit_success.txt").exists():
        raise SystemExit(f"checkpoint is not complete: {root}")
    files = []
    total_size = 0
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        size = path.stat().st_size
        total_size += size
        files.append({"path": str(path.relative_to(root)), "size_bytes": size, "sha256": sha256_file(path)})
    report = {
        "status": "PASS",
        "checkpoint_name": "pi05_base",
        "source_uri": "gs://openpi-assets/checkpoints/pi05_base",
        "cache_root": str(root),
        "total_size_bytes": total_size,
        "total_size_gib": total_size / (1024**3),
        "file_count": len(files),
        "files": files,
        "metadata_files": [
            "params/_CHECKPOINT_METADATA",
            "params/_METADATA",
            "params/_sharding",
            "params/commit_success.txt",
        ],
        "load_report": "reports/pi05_base_inference_smoke.json",
        "hardware_accessed": False,
        "piper_policy": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("status", "total_size_bytes", "total_size_gib", "file_count")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
