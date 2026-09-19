#!/usr/bin/env python3
"""Read-only laptop camera enumeration and ten-frame image probe.

Prepared for P2 and intentionally not executed on the server in P0/P1. The
probe opens only camera capture devices after explicit user invocation; it has
no robot, CAN, or SDK dependency.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def _serial_candidates(index: int) -> list[dict]:
    by_id = Path("/dev/v4l/by-id")
    if not by_id.is_dir():
        return []
    target = f"video{index}"
    matches = []
    for link in sorted(by_id.iterdir()):
        try:
            resolved = link.resolve()
        except OSError:
            continue
        if resolved.name == target:
            matches.append({"name": link.name, "target": str(resolved)})
    return matches


def _realsense_intrinsics() -> dict:
    try:
        import pyrealsense2 as rs
    except ImportError:
        return {"available": False, "reason": "pyrealsense2_not_installed"}
    try:
        context = rs.context()
        devices = []
        for device in context.query_devices():
            devices.append({"serial": device.get_info(rs.camera_info.serial_number), "intrinsics": "profile_query_pending"})
        return {"available": bool(devices), "devices": devices}
    except Exception as exc:
        return {"available": False, "error": repr(exc)}


def probe(args: argparse.Namespace) -> dict:
    try:
        import cv2
    except ImportError as exc:
        return {"status": "BLOCKED_MISSING_OPENCV", "error": repr(exc), "hardware_accessed": False}

    args.output_dir.mkdir(parents=True, exist_ok=True)
    cameras = []
    for index in range(args.max_index):
        capture = cv2.VideoCapture(index)
        if not capture.isOpened():
            capture.release()
            continue
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        entry = {
            "index": index,
            "serial_candidates": _serial_candidates(index),
            "backend": capture.getBackendName(),
            "resolution": {"width": width, "height": height},
            "resolution_modes": [
                {"width": width, "height": height, "source": "OpenCV current backend-reported mode"}
            ],
            "fps_reported": fps,
            "frames": [],
        }
        for frame_index in range(args.frames):
            timestamp_ns = time.time_ns()
            monotonic_ns = time.monotonic_ns()
            ok, frame = capture.read()
            item = {
                "index": frame_index,
                "ok": bool(ok),
                "timestamp_ns": timestamp_ns,
                "monotonic_ns": monotonic_ns,
            }
            if ok:
                path = args.output_dir / f"camera_{index:02d}_frame_{frame_index:02d}.png"
                cv2.imwrite(str(path), frame)
                item.update({"shape": list(frame.shape), "path": str(path)})
            entry["frames"].append(item)
        capture.release()
        entry["observed_frame_count"] = sum(bool(frame["ok"]) for frame in entry["frames"])
        entry["observed_fps"] = None
        cameras.append(entry)
    return {
        "status": "PASS" if cameras else "NO_CAMERA_DETECTED",
        "hardware_accessed": bool(cameras),
        "camera_count": len(cameras),
        "cameras": cameras,
        "intrinsics": _realsense_intrinsics(),
        "model_input_contract": {"dtype": "uint8", "target_size": [224, 224], "aspect_ratio_policy": "official_resize_with_pad"},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-index", type=int, default=8)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--output-dir", type=Path, default=Path("camera_probe"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = probe(args)
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report["status"] in {"PASS", "NO_CAMERA_DETECTED"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
