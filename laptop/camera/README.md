---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/RUNBOOK.md)。

# Reusable Base/Wrist capture

Use `laptop.camera.dual_camera` for image and video acquisition. It uses DSHOW
and resolves current indices by name: Base = `ACER HD User Facing`,
Wrist = `Dabai DC1`. On October 7 these are indices 0 and 1. Missing or
ambiguous names and mismatched explicit indices are refused.

## Execution environment

Run from `E:\01_NUS\02_5003\12_pi0.5` with
`E:\anaconda3\envs\pi05_laptop\python.exe`. In Codex, actual camera commands
must use `sandbox_permissions="require_escalated"` to run outside the
restricted sandbox. A normal user terminal does not thereby require Windows
administrator privileges. The script does not elevate itself. On October 7,
ACER returned `0x80070005 E_ACCESSDENIED` inside the sandbox and streamed
normally outside it.

## Commands

Each run exclusively creates a new output directory. Examples assume the
`pi05_laptop` environment is active.

```powershell
python -B -m laptop.camera.dual_camera check --duration 5 --output-dir captures/check_001
python -B -m laptop.camera.dual_camera snapshot --duration 2 --output-dir captures/snapshot_001
python -B -m laptop.camera.dual_camera video --duration 60 --output-dir captures/video_001
python -B -m laptop.camera.dual_camera frames --duration 60 --image-fps 10 --output-dir captures/frames_001
python -B -m laptop.camera.dual_camera all --duration 10 --image-fps 10 --output-dir captures/all_001
```

`check` measures timing and image signal; `snapshot` saves final images;
`video` records both MP4 files; `frames` saves sampled lossless PNGs; `all`
saves videos, PNGs, and final images together.

Override `--base-name`, `--wrist-name`, and optionally their indices when
hardware changes. Inspect names with `laptop/diagnostics/list_directshow_cameras.ps1`.
This binds only device properties by default. Avoid blind index scans:
opening the Broadcast virtual filter can restart Broadcast.

## Outputs and timing

Every mode writes `report.json`, `base_timestamps.jsonl`, and
`wrist_timestamps.jsonl`. Video modes add `base.mp4` and `wrist.mp4`;
frame modes add `base/*.png` and `wrist/*.png`; snapshot modes add
`base_snapshot.jpg` and `wrist_snapshot.jpg`.

JSONL rows contain source frame index, host `perf_counter_ns` after reading,
shape, dtype, optional image path, and optional video frame index. Image
sampling follows a fixed schedule without inventing timestamps; incoming
frames and disk speed determine the actual rate. Unselected PNG frames still
have a row with `image_path=null`.

OpenCV arrays are uint8 BGR. PNG/JPEG files have normal encoded colors;
PIL readers return RGB. Video playback uses fixed encoded FPS. Use JSONL
arrival times for alignment, rather than reconstructing times from playback
FPS. Completed videos are fully decoded to compare frame counts. This does
not certify hardware exposure synchronization, CAN alignment, task framing,
or training dataset readiness.

## Python integration

Call the same implementation from a collection/inference application:

```python
from laptop.camera.dual_camera import capture

def receive(role, frame_bgr, received_monotonic_ns, frame_index):
    # Copy/store using a lock when sharing across threads.
    # Convert BGR to RGB for consumers that expect RGB.
    pass

result = capture("captures/python_session_001", mode="video", duration_s=30,
                 on_frame=receive)
```

Callbacks run concurrently on two reader threads and must be quick and
thread-safe. Keep CAN ownership in its own component; this module never opens
CAN. For a disposable worker with driver-stall protection, use
`supervised_capture(path, mode=..., duration_s=...)` with an
`if __name__ == "__main__":` guard. Arbitrary callbacks use `capture()` and
require parent-application supervision. CLI durations support up to one hour.
The supervisor stops after 15 seconds without progress and preserves partial
outputs on failure or Ctrl+C. Interrupted videos may be incomplete; only
`CAPTURE_PASS` indicates completed checks.

The shared `source.open_camera()` is also used by the existing dual-camera
probe/recorder path and `live_shadow` loops. Historical episode schemas are
preserved.
