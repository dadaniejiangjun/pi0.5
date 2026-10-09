#!/usr/bin/env python3
"""Probe two explicitly selected cameras without recording video or touching PiPER.

The reported timestamps are host monotonic times *after* ``read`` returns.
They describe frame arrival, not exposure timing or hardware synchronization.
Camera roles must be confirmed from the actual images by the operator.
"""

from __future__ import annotations

import argparse
import bisect
import json
import statistics
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100
    lower = int(position)
    fraction = position - lower
    if lower + 1 == len(ordered):
        return ordered[lower]
    return ordered[lower] + fraction * (ordered[lower + 1] - ordered[lower])


def _summary(index: int, timestamps: list[int], failures: int, shapes: set[tuple[int, ...]], elapsed_s: float) -> dict:
    intervals_ms = [(right - left) / 1_000_000 for left, right in zip(timestamps, timestamps[1:])]
    return {
        "opencv_index": index,
        "frames_ok": len(timestamps),
        "read_failures": failures,
        "effective_fps": round(len(timestamps) / elapsed_s, 2) if elapsed_s else 0.0,
        "shapes": [list(shape) for shape in sorted(shapes)],
        "interval_median_ms": round(statistics.median(intervals_ms), 2) if intervals_ms else None,
        "interval_p95_ms": round(_percentile(intervals_ms, 95), 2) if intervals_ms else None,
        "interval_max_ms": round(max(intervals_ms), 2) if intervals_ms else None,
        "interval_gt_100ms_count": sum(interval > 100 for interval in intervals_ms),
    }


def _nearest_skew_ms(base: list[int], wrist: list[int]) -> dict | None:
    if not base or not wrist:
        return None
    skews = []
    for timestamp in base:
        position = bisect.bisect_left(wrist, timestamp)
        neighbors = wrist[max(0, position - 1) : min(len(wrist), position + 1)]
        skews.append(min(abs(timestamp - other) for other in neighbors) / 1_000_000)
    return {
        "median": round(statistics.median(skews), 2),
        "p95": round(_percentile(skews, 95), 2),
        "max": round(max(skews), 2),
    }


def _backend(cv2, backend_name: str | None):
    from laptop.camera.source import backend_constant
    return backend_constant(cv2, backend_name)


def _capture_properties(capture, cv2) -> dict:
    properties = {
        "reported_fps": cv2.CAP_PROP_FPS,
        "reported_width": cv2.CAP_PROP_FRAME_WIDTH,
        "reported_height": cv2.CAP_PROP_FRAME_HEIGHT,
        "reported_fourcc": cv2.CAP_PROP_FOURCC,
    }
    result = {}
    for name, property_id in properties.items():
        try:
            value = float(capture.get(property_id))
            result[name] = value if value >= 0 and value < float("inf") else None
        except Exception:
            result[name] = None
    return result


def _frame_content_sample(frame) -> dict:
    """Measure a narrow near-black condition; this does not verify camera role."""
    import numpy as np

    pixels = np.asarray(frame)
    if pixels.size == 0 or pixels.ndim not in (2, 3):
        raise ValueError("camera frame must be a non-empty 2D or 3D image")
    intensity = pixels.astype(np.float64, copy=False)
    if intensity.ndim == 3:
        intensity = intensity.mean(axis=2)
    mean_intensity = float(intensity.mean())
    std_intensity = float(intensity.std())
    nonzero_fraction = float(np.count_nonzero(intensity) / intensity.size)
    near_black = mean_intensity <= 3.0 and nonzero_fraction <= 0.10
    return {
        "mean_intensity": mean_intensity,
        "std_intensity": std_intensity,
        "fraction_nonzero": nonzero_fraction,
        "near_black": near_black,
    }


def _image_content_summary(samples: list[dict]) -> dict:
    if not samples:
        return {"status": "NOT_MEASURED", "sample_count": 0}
    all_near_black = all(sample["near_black"] for sample in samples)
    return {
        "status": "LOW_INFORMATION_NEAR_BLACK" if all_near_black else "SIGNAL_PRESENT",
        "sample_count": len(samples),
        "near_black_sample_count": sum(sample["near_black"] for sample in samples),
        "mean_intensity_median": round(statistics.median(
            sample["mean_intensity"] for sample in samples), 3),
        "std_intensity_median": round(statistics.median(
            sample["std_intensity"] for sample in samples), 3),
        "fraction_nonzero_median": round(statistics.median(
            sample["fraction_nonzero"] for sample in samples), 6),
        "heuristic": "LOW_INFORMATION_NEAR_BLACK when every sampled frame has mean <= 3 and nonzero fraction <= 0.10",
        "role_verified": False,
    }


def probe(base_index: int, wrist_index: int, duration_s: float, video_dir: Path | None = None,
          *, start_gate: threading.Event | None = None,
          on_opened: Callable[[bool], None] | None = None,
          start_gate_timeout_s: float = 10,
          backend_name: str | None = None,
          on_frame: Callable | None = None) -> dict:
    if base_index == wrist_index:
        raise ValueError("base and wrist must use different OpenCV indices")
    if duration_s <= 0:
        raise ValueError("duration must be positive")

    import cv2

    backend, resolved_backend_name = _backend(cv2, backend_name)
    cameras = {}
    if video_dir is not None:
        video_dir = Path(video_dir)
        video_dir.mkdir(parents=True, exist_ok=True)
    for role, index in (("base", base_index), ("wrist", wrist_index)):
        from laptop.camera.source import open_camera
        capture = open_camera(index, backend_name=resolved_backend_name)
        cameras[role] = {
            "role": role,
            "index": index,
            "capture": capture,
            "opened": capture.isOpened(),
            "properties": _capture_properties(capture, cv2),
            "timestamps": [],
            "failures": 0,
            "shapes": set(),
            "image_quality_samples": [],
            "last_quality_sample_ns": None,
            "video_path": str(video_dir / f"{role}.mp4") if video_dir is not None else None,
            "writer": None,
            "written_frames": 0,
            "thread_error": None,
        }
    try:
        opened = all(camera["opened"] for camera in cameras.values())
        if on_opened is not None:
            on_opened(opened)
        if not opened:
            return {
                "status": "OPEN_FAILED",
                "opened": {role: camera["opened"] for role, camera in cameras.items()},
                "camera_properties": {role: camera["properties"] for role, camera in cameras.items()},
                "robot_accessed": False,
                "video_saved": False,
            }

        if start_gate_timeout_s <= 0:
            raise ValueError("start gate timeout must be positive")
        if start_gate is not None and not start_gate.wait(timeout=start_gate_timeout_s):
            return {"status": "START_GATE_TIMEOUT", "robot_accessed": False,
                    "video_saved": False, "frame_timestamps_ns": {"base": [], "wrist": []}}

        start_event = threading.Event()
        start_ns = time.perf_counter_ns()
        end_ns = start_ns + int(duration_s * 1_000_000_000)

        def capture_loop(camera: dict) -> None:
            try:
                start_event.wait()
                while time.perf_counter_ns() < end_ns:
                    ok, frame = camera["capture"].read()
                    timestamp = time.perf_counter_ns()
                    if ok and frame is not None:
                        camera["timestamps"].append(timestamp)
                        camera["shapes"].add(tuple(frame.shape))
                        last_quality_sample_ns = camera["last_quality_sample_ns"]
                        if (last_quality_sample_ns is None
                                or timestamp - last_quality_sample_ns >= 1_000_000_000):
                            camera["image_quality_samples"].append(_frame_content_sample(frame))
                            camera["last_quality_sample_ns"] = timestamp
                        if video_dir is not None:
                            if camera["writer"] is None:
                                height, width = frame.shape[:2]
                                nominal_fps = camera["capture"].get(cv2.CAP_PROP_FPS)
                                fps = nominal_fps if 1 <= nominal_fps <= 120 else 30.0
                                camera["writer"] = cv2.VideoWriter(
                                    camera["video_path"], cv2.VideoWriter_fourcc(*"mp4v"),
                                    fps, (width, height))
                                if not camera["writer"].isOpened():
                                    raise RuntimeError(f"video writer failed: {camera['video_path']}")
                            camera["writer"].write(frame)
                            camera["written_frames"] += 1
                        if on_frame is not None:
                            on_frame(camera["role"], frame, timestamp,
                                     len(camera["timestamps"]) - 1)
                    else:
                        camera["failures"] += 1
            except Exception as exc:
                camera["thread_error"] = str(exc)

        threads = [threading.Thread(target=capture_loop, args=(camera,)) for camera in cameras.values()]
        for thread in threads:
            thread.start()
        start_event.set()
        for thread in threads:
            thread.join()
        elapsed_s = (time.perf_counter_ns() - start_ns) / 1_000_000_000
        transport_ok = all(
            camera["timestamps"] and camera["failures"] == 0
            and camera["thread_error"] is None
            and (video_dir is None or camera["written_frames"] == len(camera["timestamps"]))
            for camera in cameras.values()
        )
        camera_summaries = {
            role: {
                **_summary(camera["index"], camera["timestamps"], camera["failures"],
                           camera["shapes"], elapsed_s),
                "camera_properties": camera["properties"],
                "image_content": _image_content_summary(camera["image_quality_samples"]),
                "video_path": camera["video_path"],
                "written_frames": camera["written_frames"],
                "thread_error": camera["thread_error"],
            }
            for role, camera in cameras.items()
        }
        content_low_information = any(
            summary["image_content"]["status"] == "LOW_INFORMATION_NEAR_BLACK"
            for summary in camera_summaries.values()
        )
        return {
            "status": (
                "CONCURRENT_STREAM_LOW_INFORMATION" if transport_ok and content_low_information
                else "CONCURRENT_STREAM_PASS" if transport_ok
                else "PARTIAL"
            ),
            "test_utc": datetime.now(timezone.utc).isoformat(),
            "duration_s": round(elapsed_s, 3),
            "timestamp_clock": "python_perf_counter_ns_QueryPerformanceCounter_on_Windows",
            "role_mapping": "operator_visual_confirmation_required",
            "image_content_status": (
                "LOW_INFORMATION_NEAR_BLACK" if content_low_information else "SIGNAL_PRESENT"
            ),
            "image_content_caveat": "Signal presence does not verify Base/Wrist role, framing, or task visibility.",
            "backend": resolved_backend_name,
            "robot_accessed": False,
            "video_saved": video_dir is not None and all(
                camera["written_frames"] > 0 and camera["thread_error"] is None
                for camera in cameras.values()),
            "timestamp_kind": "host_monotonic_after_read_not_hardware_exposure",
            "cameras": camera_summaries,
            "frame_timestamps_ns": {
                role: camera["timestamps"] for role, camera in cameras.items()
            } if video_dir is not None else None,
            "nearest_host_arrival_skew_ms": _nearest_skew_ms(
                cameras["base"]["timestamps"], cameras["wrist"]["timestamps"]
            ),
        }
    finally:
        for camera in cameras.values():
            if camera["writer"] is not None:
                camera["writer"].release()
            camera["capture"].release()


def _probe_worker(base_index: int, wrist_index: int, duration_s: float,
                  video_dir: Path | None, backend_name: str | None) -> dict:
    return probe(base_index, wrist_index, duration_s, video_dir,
                 backend_name=backend_name)


def probe_supervised(base_index: int, wrist_index: int, duration_s: float,
                     video_dir: Path | None = None, *, timeout_s: float | None = None,
                     backend_name: str | None = None) -> dict:
    from laptop.diagnostics.bounded_process import run_bounded

    budget = timeout_s if timeout_s is not None else duration_s + 15
    result = run_bounded(_probe_worker,
                         args=(base_index, wrist_index, duration_s, video_dir, backend_name),
                         timeout_s=budget)
    if result["status"] == "COMPLETE":
        return result["result"]
    return {"status": result["status"], "error": result["error"],
            "robot_accessed": False, "video_saved": False,
            "frame_timestamps_ns": {"base": [], "wrist": []},
            "completion_verified": False}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-index", type=int, required=True)
    parser.add_argument("--wrist-index", type=int, required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--video-dir", type=Path, help="Optional local video capture; use an ignored private directory")
    parser.add_argument("--base-device-id", required=True)
    parser.add_argument("--wrist-device-id", required=True)
    parser.add_argument("--roles-visually-confirmed", action="store_true")
    args = parser.parse_args()
    from laptop.diagnostics.camera_identity import bind_camera_roles
    identity = bind_camera_roles(
        args.base_index, args.wrist_index,
        base_device_id=args.base_device_id, wrist_device_id=args.wrist_device_id,
        roles_visually_confirmed=args.roles_visually_confirmed,
    )
    result = probe_supervised(args.base_index, args.wrist_index, args.duration, args.video_dir)
    result["camera_identity"] = identity
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    display = dict(result)
    if args.output:
        display.pop("frame_timestamps_ns", None)
    print(json.dumps(display, indent=2))
    return 0 if result["status"] == "CONCURRENT_STREAM_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
