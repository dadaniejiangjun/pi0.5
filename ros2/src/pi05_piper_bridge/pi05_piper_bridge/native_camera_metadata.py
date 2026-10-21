"""Normalize native ROS color images and attach local arrival-time metadata.

No camera is opened here. Driver timestamps are retained in the audit log;
the policy-facing timestamp is the ROS callback arrival, not exposure time.
"""
from array import array
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from pi05_piper_interfaces.msg import CameraFrameMetadata, StateSnapshot
from .image_rate_limiter import ImageRateGate


def packed_bgr(image):
    if image.encoding not in ("rgb8", "bgr8", "yuv422_yuy2"):
        raise ValueError(f"unsupported native encoding: {image.encoding}")
    if image.width <= 0 or image.height <= 0:
        raise ValueError("invalid native image dimensions")
    bytes_per_pixel = 2 if image.encoding == "yuv422_yuy2" else 3
    if image.encoding == "yuv422_yuy2" and image.width % 2:
        raise ValueError("YUY2 image width must be even")
    if image.step < image.width * bytes_per_pixel:
        raise ValueError("native image step is shorter than one packed row")
    raw = np.frombuffer(image.data, dtype=np.uint8)
    if raw.size != image.height * image.step:
        raise ValueError("native image payload size mismatch")
    pixels = raw.reshape(image.height, image.step)[:, :image.width * bytes_per_pixel]
    if image.encoding == "yuv422_yuy2":
        yuy2 = pixels.reshape(image.height, image.width, 2)
        return cv2.cvtColor(yuy2, cv2.COLOR_YUV2BGR_YUY2)
    pixels = pixels.reshape(image.height, image.width, 3)
    return np.ascontiguousarray(pixels[:, :, ::-1] if image.encoding == "rgb8" else pixels)


def bgr_image_data(bgr):
    """Use the ROS uint8-array setter path without per-byte Python validation."""
    payload = array("B")
    payload.frombytes(bgr.tobytes())
    return payload


class NativeCameraMetadata(Node):
    def __init__(self):
        super().__init__("pi05_native_camera_metadata")
        self.declare_parameter("log_path", "/tmp/pi05_native_camera_timing.jsonl")
        self.declare_parameter("base_device", "")
        self.declare_parameter("wrist_device", "")
        self.declare_parameter("publish_hz", 10.0)
        if not all(str(self.get_parameter(f"{r}_device").value).strip() for r in ("base", "wrist")):
            raise ValueError("explicit verified base_device and wrist_device identities required")
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        if not 1.0 <= self.publish_hz <= 30.0:
            raise ValueError("publish_hz must be within 1..30")
        self.sequence = {"base": 0, "wrist": 0}
        self.image_rate_gates = {
            role: ImageRateGate(self.publish_hz) for role in self.sequence
        }
        self.image_callback_groups = {
            role: MutuallyExclusiveCallbackGroup() for role in self.sequence
        }
        self.state = None
        self.last_state_ns = 0
        self.received_count = {"base": 0, "wrist": 0}
        self.throttled_count = {"base": 0, "wrist": 0}
        self.stale_state_count = {"base": 0, "wrist": 0}
        self.decode_error_count = {"base": 0, "wrist": 0}
        self.published_count = {"base": 0, "wrist": 0}
        self.last_decode_error = {"base": "", "wrist": ""}
        self.last_source_arrival_ns = {"base": 0, "wrist": 0}
        self.conversion_durations_ns = {"base": [], "wrist": []}
        self.callback_durations_ns = {"base": [], "wrist": []}
        self.metric_lock = threading.Lock()
        path = Path(str(self.get_parameter("log_path").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        self.log = path.open("x", encoding="utf-8")
        self.metrics_window_start_ns = time.perf_counter_ns()
        self.metrics_previous = {
            role: {"received": 0, "throttled": 0, "stale_state": 0,
                   "decode_error": 0, "published": 0}
            for role in self.sequence
        }
        self.meta_pub = self.create_publisher(CameraFrameMetadata, "/piper/cameras/frame_metadata", qos_profile_sensor_data)
        self.image_pubs = {r: self.create_publisher(Image, f"/piper/cameras/{r}/image_raw", qos_profile_sensor_data) for r in self.sequence}
        self.state_sub = self.create_subscription(StateSnapshot, "/piper/state_snapshot", self.on_state, qos_profile_sensor_data)
        self.image_subs = [
            self.create_subscription(
                Image, f"/native/cameras/{role}/image_raw",
                lambda msg, role=role: self.on_image(role, msg),
                qos_profile_sensor_data,
                callback_group=self.image_callback_groups[role],
            )
            for role in self.sequence
        ]
        self.metrics_timer = self.create_timer(1.0, self.write_periodic_metrics)

    def on_state(self, msg):
        self.state = msg
        self.last_state_ns = time.perf_counter_ns()

    def on_image(self, role, msg):
        arrival = time.perf_counter_ns()
        with self.metric_lock:
            self.received_count[role] += 1
            self.last_source_arrival_ns[role] = arrival
        state = self.state
        state_arrival = self.last_state_ns
        if (state is None or not state.feedback_fresh
                or state.source_clock_id != "linux_perf_counter_ns"
                or arrival - state_arrival > 250_000_000):
            with self.metric_lock:
                self.stale_state_count[role] += 1
            return
        if not self.image_rate_gates[role].accept(arrival):
            with self.metric_lock:
                self.throttled_count[role] += 1
            return
        try:
            conversion_start = time.perf_counter_ns()
            bgr = packed_bgr(msg)
            conversion_duration = time.perf_counter_ns() - conversion_start
        except ValueError as exc:
            with self.metric_lock:
                self.decode_error_count[role] += 1
                self.last_decode_error[role] = str(exc)
            return
        output = Image()
        output.height = msg.height
        output.width = msg.width
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = role
        output.encoding = "bgr8"
        output.step = msg.width * 3
        output.is_bigendian = 0
        output.data = bgr_image_data(bgr)
        with self.metric_lock:
            self.sequence[role] += 1
            sequence = self.sequence[role]
        meta = CameraFrameMetadata(header=output.header)
        meta.role, meta.session_id = role, state.session_id
        meta.device_name = str(self.get_parameter(f"{role}_device").value)
        meta.sequence = sequence
        meta.source_monotonic_ns = arrival
        meta.source_clock_id = "linux_perf_counter_ns"
        meta.source_local_monotonic_ns = arrival
        meta.source_local_clock_id = "wsl_perf_counter_ns"
        meta.clock_mapping_uncertainty_ns = 0
        meta.encoding, meta.width, meta.height = "bgr8", msg.width, msg.height
        self.image_pubs[role].publish(output)
        self.meta_pub.publish(meta)
        with self.metric_lock:
            self.published_count[role] += 1
            self.conversion_durations_ns[role].append(conversion_duration)
            self.callback_durations_ns[role].append(time.perf_counter_ns() - arrival)

    def write_periodic_metrics(self):
        """Write one aggregate record per role per second; never log each frame."""
        now = time.perf_counter_ns()
        with self.metric_lock:
            elapsed_s = max((now - self.metrics_window_start_ns) / 1e9, 1e-9)
            for role in self.sequence:
                current = {
                    "received": self.received_count[role],
                    "throttled": self.throttled_count[role],
                    "stale_state": self.stale_state_count[role],
                    "decode_error": self.decode_error_count[role],
                    "published": self.published_count[role],
                }
                previous = self.metrics_previous[role]
                deltas = {key: current[key] - previous[key] for key in current}
                self.metrics_previous[role] = current
                row = {
                    "schema": "piper_native_camera_periodic_metrics_v1",
                    "role": role,
                    "window_end_monotonic_ns": now,
                    "elapsed_s": elapsed_s,
                    "source_callback_hz": deltas["received"] / elapsed_s,
                    "normalized_publish_hz": deltas["published"] / elapsed_s,
                    "delta": deltas,
                    "cumulative": current,
                    "last_source_arrival_monotonic_ns": self.last_source_arrival_ns[role],
                    "last_decode_error": self.last_decode_error[role],
                    "conversion_ms_samples": [n / 1e6 for n in self.conversion_durations_ns[role]],
                    "callback_ms_samples": [n / 1e6 for n in self.callback_durations_ns[role]],
                    "publish_hz_limit": self.publish_hz,
                    "semantics": "source ROS callback arrival; not exposure synchronization",
                }
                self.log.write(json.dumps(row) + "\n")
                self.conversion_durations_ns[role].clear()
                self.callback_durations_ns[role].clear()
            self.metrics_window_start_ns = now
            self.log.flush()


def main(args=None):
    rclpy.init(args=args)
    node = NativeCameraMetadata()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
