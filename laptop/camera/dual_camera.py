"""Reusable Windows Base/Wrist image and video capture, without robot access."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import math
import multiprocessing as mp
from pathlib import Path
import subprocess
import threading
import time

from laptop.diagnostics.probe_dual_cameras import probe

BASE_NAME = "ACER HD User Facing"
WRIST_NAME = "Dabai DC1"


def inventory():
    script = Path(__file__).resolve().parents[1] / "diagnostics/list_directshow_cameras.ps1"
    result = subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", str(script)], capture_output=True, text=True, encoding="utf-8",
        errors="replace", timeout=10)
    if result.returncode:
        raise RuntimeError("DirectShow name enumeration failed: " + result.stderr[-1000:])
    devices = json.loads(result.stdout.lstrip("\ufeff"))
    return devices if isinstance(devices, list) else [devices]


def resolve_roles(devices, *, base_name=BASE_NAME, wrist_name=WRIST_NAME,
                  base_index=None, wrist_index=None):
    roles = {}
    for role, name, requested in (("base", base_name, base_index),
                                  ("wrist", wrist_name, wrist_index)):
        matches = [d for d in devices if d["Name"].casefold() == name.casefold()]
        if len(matches) != 1:
            raise ValueError(f"{role}: expected one device named {name!r}, got {len(matches)}")
        device = matches[0]
        if requested is not None and requested != device["Index"]:
            raise ValueError(f"{role}: index {requested} is not current {name!r} index {device['Index']}")
        roles[role] = dict(opencv_index=device["Index"], device_name=device["Name"],
                           device_path=device.get("DevicePath"), binding="current_DSHOW_name_enumeration")
    if roles["base"]["opencv_index"] == roles["wrist"]["opencv_index"]:
        raise ValueError("Base and Wrist must be different cameras")
    return roles


def _write_report(path, report):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def capture(output_dir, *, mode="check", duration_s=5, image_fps=10,
            base_name=BASE_NAME, wrist_name=WRIST_NAME, base_index=None, wrist_index=None,
            on_frame=None, progress=None, devices=None, probe_function=None):
    """Capture using two reader threads. on_frame(role, BGR_frame, ns, index).

    Callbacks run concurrently and must be thread-safe. Frames use uint8 BGR.
    Timestamps are perf_counter_ns AFTER read, never hardware exposure times.
    Each run exclusively creates a directory and preserves any partial outputs.
    """
    if mode not in ("check", "snapshot", "video", "frames", "all"):
        raise ValueError("unsupported capture mode")
    if not math.isfinite(duration_s) or not 0 < duration_s <= 3600:
        raise ValueError("duration must be within (0, 3600] seconds")
    if not math.isfinite(image_fps) or not 0 < image_fps <= 120:
        raise ValueError("image_fps must be within (0, 120]")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    report_path = output / "report.json"
    report = dict(schema="piper_dual_camera_capture_v1", status="STARTED",
        started_utc=datetime.now(timezone.utc).isoformat(), mode=mode,
        duration_requested_s=duration_s, backend="DSHOW", color_order="BGR",
        timestamp_clock="python_perf_counter_ns", timestamp_kind="host_after_read",
        execution_requirement="outside_restricted_Codex_sandbox",
        hardware_exposure_sync_verified=False, camera_can_sync_verified=False,
        robot_accessed=False, requested_image_fps=image_fps)
    _write_report(report_path, report)
    logs = {}
    latest = {}
    saved = {"base": 0, "wrist": 0}
    next_saved = {"base": None, "wrist": None}
    heartbeat_lock = threading.Lock()
    last_progress = [0]
    try:
        roles = resolve_roles(inventory() if devices is None else devices,
            base_name=base_name, wrist_name=wrist_name, base_index=base_index, wrist_index=wrist_index)
        report["roles"] = roles
        _write_report(report_path, report)
        import cv2
        for role in roles:
            logs[role] = (output / f"{role}_timestamps.jsonl").open("x", encoding="utf-8")
            if mode in ("frames", "all"):
                (output / role).mkdir()

        def receive(role, frame, stamp, index):
            path = None
            if mode in ("snapshot", "all"):
                latest[role] = frame.copy()
            if mode in ("frames", "all") and (next_saved[role] is None or stamp >= next_saved[role]):
                relative = f"{role}/{index:08d}.png"
                if not cv2.imwrite(str(output / relative), frame):
                    raise RuntimeError(f"image write failed: {relative}")
                path = relative
                saved[role] += 1
                period = 1_000_000_000/image_fps
                previous = stamp if next_saved[role] is None else next_saved[role]
                next_saved[role] = previous + (math.floor((stamp-previous)/period)+1)*period
            logs[role].write(json.dumps(dict(role=role, frame_index=index,
                received_monotonic_ns=stamp, shape=list(frame.shape), dtype=str(frame.dtype),
                image_path=path, video_path=f"{role}.mp4" if mode in ("video", "all") else None,
                video_frame_index=index if mode in ("video", "all") else None)) + "\n")
            logs[role].flush()
            if on_frame is not None:
                on_frame(role, frame, stamp, index)
            if progress is not None:
                with heartbeat_lock:
                    if stamp-last_progress[0] >= 1_000_000_000:
                        progress()
                        last_progress[0] = stamp

        if progress is not None:
            progress()
        result = (probe_function or probe)(roles["base"]["opencv_index"],
            roles["wrist"]["opencv_index"], duration_s,
            output if mode in ("video", "all") else None,
            backend_name="DSHOW", on_frame=receive)
        # Per-frame times are already durable JSONL; avoid duplicating them in report.
        result.pop("frame_timestamps_ns", None)
        report["stream_result"] = result
        for role, frame in latest.items():
            if not cv2.imwrite(str(output / f"{role}_snapshot.jpg"), frame):
                raise RuntimeError(f"snapshot write failed for {role}")
        report["saved_images"] = saved
        report["video_validation"] = {}
        if mode in ("video", "all"):
            for role in roles:
                video = cv2.VideoCapture(str(output / f"{role}.mp4"))
                decoded = 0
                try:
                    opened = video.isOpened()
                    while video.read()[0]:
                        decoded += 1
                        if progress is not None and decoded % 100 == 0:
                            progress()
                finally:
                    video.release()
                expected = result.get("cameras", {}).get(role, {}).get("written_frames", 0)
                report["video_validation"][role] = dict(opened=opened, decoded_frames=decoded,
                    expected_frames=expected, passed=opened and decoded == expected and decoded > 0,
                    playback_timing="fixed_encoded_fps; use_JSONL_for_actual_arrival_times")
        valid_video = all(item["passed"] for item in report["video_validation"].values())
        report["status"] = "CAPTURE_PASS" if result["status"] == "CONCURRENT_STREAM_PASS" and valid_video else "CAPTURE_PARTIAL"
    except Exception as exc:
        report.update(status="CAPTURE_FAILED", error_type=type(exc).__name__, reason=str(exc))
    finally:
        for stream in logs.values():
            stream.close()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        _write_report(report_path, report)
    return report


def _worker(sender, output, options):
    try:
        result = capture(output, progress=lambda: sender.send(("progress", None)), **options)
        sender.send(("result", result))
    except BaseException as exc:
        sender.send(("error", f"{type(exc).__name__}: {exc}"))
    finally:
        sender.close()


def supervised_capture(output_dir, **options):
    """CLI process supervisor: abort unresponsive drivers and preserve partial data."""
    output = Path(output_dir)
    if output.exists():
        raise FileExistsError(f"capture directory already exists: {output}")
    context = mp.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    process = context.Process(target=_worker, args=(sender, output, options))
    started = last_progress = time.monotonic()
    budget = options.get("duration_s", 5) + max(30, options.get("duration_s", 5))
    outcome = None
    try:
        process.start()
        sender.close()
        while time.monotonic()-started < budget:
            if receiver.poll(0.5):
                try:
                    kind, payload = receiver.recv()
                except EOFError:
                    outcome = dict(status="WORKER_ERROR", reason="worker exited without report")
                    break
                last_progress = time.monotonic()
                if kind == "result":
                    outcome = payload
                    break
                if kind == "error":
                    outcome = dict(status="WORKER_ERROR", reason=payload)
                    break
            if time.monotonic()-last_progress > 15:
                outcome = dict(status="TIMEOUT_STOPPED", reason="no capture progress for 15 seconds")
                break
        if outcome is None:
            outcome = dict(status="TIMEOUT_STOPPED", reason="capture deadline exceeded")
        process.join(timeout=1)
        if process.is_alive():
            process.terminate()
            process.join(timeout=1)
            if process.is_alive():
                process.kill()
                process.join(timeout=1)
            if outcome.get("status") == "CAPTURE_PASS":
                outcome = dict(status="WORKER_ERROR", reason="worker did not exit after completion")
        if outcome.get("status") in ("WORKER_ERROR", "TIMEOUT_STOPPED") and output.exists():
            report_path = output / "report.json"
            previous = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {}
            outcome = {**previous, **outcome, "partial_outputs_preserved": True}
            _write_report(report_path, outcome)
        return outcome
    except KeyboardInterrupt:
        if process.is_alive():
            process.terminate()
            process.join(timeout=1)
        report_path = output / "report.json"
        previous = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {}
        outcome = {**previous, "status": "INTERRUPTED", "partial_outputs_preserved": True}
        if output.exists():
            _write_report(report_path, outcome)
        return outcome
    finally:
        receiver.close()
        if process.is_alive():
            process.kill()
            process.join(timeout=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "snapshot", "video", "frames", "all"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--image-fps", type=float, default=10)
    parser.add_argument("--base-name", default=BASE_NAME)
    parser.add_argument("--wrist-name", default=WRIST_NAME)
    parser.add_argument("--base-index", type=int)
    parser.add_argument("--wrist-index", type=int)
    args = vars(parser.parse_args())
    output = args.pop("output_dir")
    args["duration_s"] = args.pop("duration")
    report = supervised_capture(output, **args)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "CAPTURE_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
