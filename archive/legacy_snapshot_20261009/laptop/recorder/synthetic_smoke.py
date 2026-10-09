"""Generate a tiny synthetic episode to exercise files and rejection gates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from laptop.recorder.episode import EpisodeWriter


def run(raw_root: Path) -> dict:
    import cv2
    import numpy as np

    episode = EpisodeWriter(raw_root, source="synthetic")
    writers = {}
    for role in ("base", "wrist"):
        writer = cv2.VideoWriter(str(episode.path / f"{role}.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (160, 120))
        if not writer.isOpened():
            raise RuntimeError(f"cannot open {role} video writer")
        writers[role] = writer
    try:
        start_ns = 1_000_000_000
        for frame_id in range(10):
            base_ts = start_ns + frame_id * 100_000_000
            wrist_ts = base_ts + 5_000_000
            for role, ts in (("base", base_ts), ("wrist", wrist_ts)):
                frame = np.full((120, 160, 3), (frame_id * 15) % 256, dtype=np.uint8)
                cv2.putText(frame, f"SYNTHETIC {role} {frame_id}", (3, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 0), 1)
                writers[role].write(frame)
                episode.add(role, ts, {"frame_id": frame_id,
                                       "timestamp_kind": "synthetic_host_after_read"})
            episode.add("joint_12", base_ts + 1, {"raw": [frame_id, 0]})
            episode.add("joint_34", base_ts + 2, {"raw": [0, 0]})
            episode.add("joint_56", base_ts + 3, {"raw": [0, 0]})
            episode.add("gripper", base_ts + 4, {"angle_or_stroke_raw": frame_id,
                                                  "effort_raw": 0, "status_code": 0})
            episode.add("status", base_ts + 5, {"ctrl_mode": "SYNTHETIC",
                                                 "teach_status": "SYNTHETIC"})
    finally:
        for writer in writers.values():
            writer.release()
    report = episode.close(success=None, notes="Synthetic pipeline test only")
    report["path"] = str(episode.path)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, default=Path("datasets/raw"))
    args = parser.parse_args()
    print(json.dumps(run(args.raw_root), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
