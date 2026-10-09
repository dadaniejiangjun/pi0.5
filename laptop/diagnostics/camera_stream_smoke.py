"""Bounded dual-camera stream check without recording or assigning roles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from laptop.diagnostics.probe_dual_cameras import probe_supervised


def run(*, base_index: int = 0, wrist_index: int = 1,
        duration_s: float = 5.0, timeout_s: float = 30.0) -> dict:
    if duration_s <= 0 or timeout_s < duration_s + 1:
        raise ValueError("timeout must exceed the positive camera test duration")
    deadline = time.monotonic() + timeout_s
    attempts = []
    result = {"status": "OPEN_FAILED"}
    best_stream_result = None
    for backend_name in ("DSHOW", "MSMF", "ANY"):
        remaining = deadline - time.monotonic()
        if remaining < duration_s + 1:
            break
        result = probe_supervised(
            base_index,
            wrist_index,
            duration_s,
            video_dir=None,
            timeout_s=min(remaining, duration_s + 5),
            backend_name=backend_name,
        )
        attempts.append({"backend": backend_name, "result": result})
        if result.get("status") in {
            "CONCURRENT_STREAM_PASS", "CONCURRENT_STREAM_LOW_INFORMATION", "PARTIAL"
        }:
            best_stream_result = result
        if result.get("status") == "CONCURRENT_STREAM_PASS":
            break
    if result.get("status") == "OPEN_FAILED" and best_stream_result is not None:
        result = best_stream_result
    return {
        "schema": "piper_camera_stream_smoke_v1",
        "status": result.get("status", "INCOMPLETE"),
        "camera_indices": {"first": base_index, "second": wrist_index},
        "duration_requested_s": duration_s,
        "backend_attempts": attempts,
        "stream_result": result,
        "camera_roles_verified": False,
        "image_content_status": result.get("image_content_status", "NOT_MEASURED"),
        "camera_usable_for_observation": (
            result.get("status") == "CONCURRENT_STREAM_PASS"
            and result.get("image_content_status") == "SIGNAL_PRESENT"
        ),
        "video_saved": False,
        "can_opened": False,
        "robot_accessed": False,
        "robot_command_sent": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-index", type=int, default=0)
    parser.add_argument("--second-index", type=int, default=1)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = run(
        base_index=args.first_index,
        wrist_index=args.second_index,
        duration_s=args.duration_s,
        timeout_s=args.timeout_s,
    )
    encoded = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    return 0 if report["status"] == "CONCURRENT_STREAM_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
