"""DepthAI v3 RGB-D source for OAK-D Pro (RVC2).

This source is independent of the project's DirectShow Base/Wrist camera map.
It returns BGR uint8 RGB frames and RGB-aligned uint16 depth in millimeters.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class OakRGBDFrame:
    rgb_bgr: object
    depth_mm: object
    device_timestamp_ns: int
    depth_timestamp_ns: int
    received_monotonic_ns: int
    sequence_num: int

    @property
    def rgb_depth_delta_ns(self) -> int:
        return abs(self.device_timestamp_ns - self.depth_timestamp_ns)


class OakRGBD:
    """Open one OAK-D Pro and yield synchronized, spatially aligned RGB-D frames."""

    def __init__(self, *, width: int = 1280, height: int = 960,
                 mono_width: int = 640, mono_height: int = 400,
                 fps: float = 25.0, sync_tolerance_ms: float = 20.0,
                 extended_disparity: bool = False) -> None:
        if min(width, height, mono_width, mono_height) <= 0:
            raise ValueError("image dimensions must be positive")
        if fps <= 0 or sync_tolerance_ms <= 0:
            raise ValueError("fps and sync tolerance must be positive")

        try:
            import depthai as dai
        except ImportError as exc:
            raise RuntimeError(
                "DepthAI is missing. Install laptop/camera/requirements-oak.txt "
                "into the selected project Python environment."
            ) from exc

        self._dai = dai
        self._pipeline = dai.Pipeline()
        rgb_socket = dai.CameraBoardSocket.CAM_A
        left_socket = dai.CameraBoardSocket.CAM_B
        right_socket = dai.CameraBoardSocket.CAM_C

        # OAK-D Pro uses the RVC2 stereo path. RGB is undistorted; the stereo
        # depth map is reprojected to that RGB output and then timestamp-synced.
        self._device = self._pipeline.getDefaultDevice()
        platform = self._device.getPlatform()
        if platform != dai.Platform.RVC2:
            raise RuntimeError(
                f"Expected OAK-D Pro RVC2, connected device reports {platform!s}."
            )

        rgb = self._pipeline.create(dai.node.Camera).build(rgb_socket)
        left = self._pipeline.create(dai.node.Camera).build(left_socket)
        right = self._pipeline.create(dai.node.Camera).build(right_socket)
        stereo = self._pipeline.create(dai.node.StereoDepth)
        sync = self._pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=sync_tolerance_ms))

        self._rgb_output = rgb.requestOutput(
            size=(width, height), fps=fps, enableUndistortion=True
        )
        left.requestOutput(size=(mono_width, mono_height), fps=fps).link(stereo.left)
        right.requestOutput(size=(mono_width, mono_height), fps=fps).link(stereo.right)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.setLeftRightCheck(True)
        if extended_disparity:
            stereo.setExtendedDisparity(True)

        # On RVC2, StereoDepth performs depth-to-RGB alignment using the RGB
        # frame metadata provided to inputAlignTo.
        self._rgb_output.link(stereo.inputAlignTo)
        self._rgb_output.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth_aligned"])
        self._queue = sync.out.createOutputQueue(maxSize=2, blocking=False)
        self._started = False

        info = self._device.getDeviceInfo()
        calibration = self._device.readCalibration()
        self.device_info = {
            "device_id": str(info.getDeviceId()),
            "platform": str(platform),
            "usb_speed": str(self._device.getUsbSpeed()),
            "connected_cameras": [str(socket) for socket in self._device.getConnectedCameras()],
            "camera_sensors": {
                str(socket): str(name)
                for socket, name in self._device.getCameraSensorNames().items()
            },
        }
        self.calibration_json = calibration.eepromToJson()

    def start(self) -> "OakRGBD":
        if not self._started:
            self._pipeline.start()
            self._started = True
        return self

    def close(self) -> None:
        if self._started:
            self._pipeline.stop()
            self._started = False

    def __enter__(self) -> "OakRGBD":
        return self.start()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def read(self, timeout_s: float = 2.0) -> Optional[OakRGBDFrame]:
        """Return one paired frame, or None if no synchronized group arrives in time."""
        if not self._started:
            raise RuntimeError("OakRGBD is not started; use it as a context manager")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")

        deadline = time.monotonic() + timeout_s
        group = self._queue.tryGet()
        while group is None and time.monotonic() < deadline:
            time.sleep(0.001)
            group = self._queue.tryGet()
        if group is None:
            return None

        rgb_message = group["rgb"]
        depth_message = group["depth_aligned"]
        rgb = rgb_message.getCvFrame()
        depth = depth_message.getFrame()
        if rgb.dtype.name != "uint8" or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise RuntimeError(f"Unexpected RGB format: {rgb.dtype} {rgb.shape}")
        if depth.dtype.name != "uint16" or depth.ndim != 2:
            raise RuntimeError(f"Unexpected depth format: {depth.dtype} {depth.shape}")
        if rgb.shape[:2] != depth.shape:
            raise RuntimeError(
                f"Depth is not aligned to RGB: rgb={rgb.shape}, depth={depth.shape}"
            )

        return OakRGBDFrame(
            rgb_bgr=rgb,
            depth_mm=depth,
            device_timestamp_ns=int(rgb_message.getTimestampDevice().total_seconds() * 1e9),
            depth_timestamp_ns=int(depth_message.getTimestampDevice().total_seconds() * 1e9),
            received_monotonic_ns=time.perf_counter_ns(),
            sequence_num=int(rgb_message.getSequenceNum()),
        )


def _colorize_depth(depth_mm):
    import cv2
    import numpy as np

    valid = depth_mm > 0
    view = np.zeros(depth_mm.shape, dtype=np.uint8)
    if np.any(valid):
        view[valid] = np.clip(depth_mm[valid].astype(np.float32) / 5000.0 * 255.0,
                              0, 255).astype(np.uint8)
    color = cv2.applyColorMap(view, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


def run_capture(output_dir: Path, *, duration_s: float = 10.0,
                preview: bool = False) -> dict:
    import cv2

    output_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "RUNNING",
        "source": "DepthAI v3 / OAK-D Pro / RVC2",
        "duration_requested_s": duration_s,
        "started_unix_time": time.time(),
    }
    frame_count = 0
    first_timestamp_ns = None
    last_timestamp_ns = None
    max_pair_delta_ns = 0
    max_host_gap_ns = 0
    previous_host_timestamp_ns = None
    last_frame = None
    pipeline_started = None
    capture_finished = None

    try:
        with OakRGBD() as camera:
            report.update(camera.device_info)
            pipeline_started = time.monotonic()
            report["pipeline_started_unix_time"] = time.time()
            while time.monotonic() - pipeline_started < duration_s:
                frame = camera.read(timeout_s=2.0)
                if frame is None:
                    raise TimeoutError("No synchronized RGB-D frame arrived within 2 seconds")
                frame_count += 1
                if first_timestamp_ns is None:
                    first_timestamp_ns = frame.device_timestamp_ns
                last_timestamp_ns = frame.device_timestamp_ns
                max_pair_delta_ns = max(max_pair_delta_ns, frame.rgb_depth_delta_ns)
                if previous_host_timestamp_ns is not None:
                    max_host_gap_ns = max(
                        max_host_gap_ns,
                        frame.received_monotonic_ns - previous_host_timestamp_ns,
                    )
                previous_host_timestamp_ns = frame.received_monotonic_ns
                last_frame = frame
                if preview:
                    cv2.imshow("OAK-D Pro RGB", frame.rgb_bgr)
                    cv2.imshow("OAK-D Pro depth (mm)", _colorize_depth(frame.depth_mm))
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            capture_finished = time.monotonic()
            calibration_path = output_dir / "calibration_eeprom.json"
            calibration_path.write_text(
                json.dumps(camera.calibration_json, indent=2), encoding="utf-8"
            )

        if last_frame is None:
            raise RuntimeError("Capture ended without an RGB-D frame")
        cv2.imwrite(str(output_dir / "rgb_bgr.png"), last_frame.rgb_bgr)
        cv2.imwrite(str(output_dir / "depth_mm.png"), last_frame.depth_mm)
        cv2.imwrite(str(output_dir / "depth_preview.png"),
                    _colorize_depth(last_frame.depth_mm))
        duration_actual = capture_finished - pipeline_started
        import numpy as np
        valid_depth = last_frame.depth_mm[last_frame.depth_mm > 0]
        min_frames = max(2, int(duration_s * 25.0 * 0.5))
        device_time_span_s = (last_timestamp_ns - first_timestamp_ns) / 1e9
        report.update({
            "status": "PASS",
            "frames": frame_count,
            "duration_actual_s": duration_actual,
            "mean_fps": (
                (frame_count - 1) / device_time_span_s
                if device_time_span_s > 0 else 0.0
            ),
            "max_host_frame_gap_ms": max_host_gap_ns / 1e6,
            "rgb_shape": list(last_frame.rgb_bgr.shape),
            "rgb_dtype": str(last_frame.rgb_bgr.dtype),
            "depth_shape": list(last_frame.depth_mm.shape),
            "depth_dtype": str(last_frame.depth_mm.dtype),
            "depth_unit": "millimeter",
            "valid_depth_pixel_percent": (
                100.0 * valid_depth.size / last_frame.depth_mm.size
            ),
            "valid_depth_min_mm": int(valid_depth.min()) if valid_depth.size else None,
            "valid_depth_median_mm": (
                float(np.median(valid_depth)) if valid_depth.size else None
            ),
            "valid_depth_max_mm": int(valid_depth.max()) if valid_depth.size else None,
            "max_rgb_depth_timestamp_delta_ms": max_pair_delta_ns / 1e6,
            "rgb_device_timestamp_start_ns": first_timestamp_ns,
            "rgb_device_timestamp_end_ns": last_timestamp_ns,
            "sample_rgb_file": "rgb_bgr.png",
            "sample_depth_file": "depth_mm.png",
            "sample_depth_preview_file": "depth_preview.png",
            "calibration_file": "calibration_eeprom.json",
        })
        if frame_count < min_frames:
            raise RuntimeError(
                f"Incomplete frame stream: got {frame_count} paired frames in "
                f"{duration_actual:.2f}s; expected at least {min_frames} "
                "(half of the configured 25 FPS rate)."
            )
    except Exception as exc:
        report.update({
            "status": "FAIL",
            "error": f"{type(exc).__name__}: {exc}",
            "frames": frame_count,
        })
        raise
    finally:
        if preview:
            cv2.destroyAllWindows()
        report["finished_unix_time"] = time.time()
        (output_dir / "report.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="capture duration in seconds (default: 10)")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="new, unused directory for the report and sample RGB-D pair")
    parser.add_argument("--preview", action="store_true",
                        help="show RGB and depth windows; press Q to stop")
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration must be positive")
    try:
        report = run_capture(args.output_dir, duration_s=args.duration,
                             preview=args.preview)
    except Exception as exc:
        print(f"OAK RGB-D capture failed: {type(exc).__name__}: {exc}")
        print(f"Report: {args.output_dir / 'report.json'}")
        return 2
    print(json.dumps(report, indent=2))
    print(f"Report: {args.output_dir / 'report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
