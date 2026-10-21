"""ROS-to-Windows preview API. The request sink is fake-only or reject-only."""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import socket
import threading
import time
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from pi05_piper_interfaces.msg import CameraFrameMetadata, StateSnapshot
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from .control_contract import MAX_STATE_AGE_NS, ProposalAuthority


HTTP_BODY_LIMIT = 64 * 1024
HTTP_STATE_TOPIC = "/piper/state_snapshot"
HTTP_BASE_IMAGE_TOPIC = "/native/cameras/base/image_raw"
HTTP_WRIST_IMAGE_TOPIC = "/native/cameras/wrist/image_raw"
HTTP_METADATA_TOPIC = "/piper/cameras/frame_metadata"
MAX_IMAGE_DIMENSION = 4096
MAX_IMAGE_BYTES = 64 * 1024 * 1024


class LatestRosData:
    """Thread-safe state and one latest encoded frame per camera role."""

    def __init__(self):
        self._condition = threading.Condition()
        self._state: dict | None = None
        self._joint_state: dict | None = None
        self._status: dict = {"level": "unknown", "message": "waiting for ROS status"}
        self._metadata: dict[str, dict] = {"base": {}, "wrist": {}}
        self._frames: dict[str, dict] = {
            role: {"sequence": 0, "jpeg": b"", "received_ns": 0, "error": "waiting for image"}
            for role in ("base", "wrist")
        }

    def update_state(self, message: StateSnapshot) -> None:
        now_ns = time.perf_counter_ns()
        source_local_ns = int(message.source_local_monotonic_ns)
        age_ns = now_ns - source_local_ns if source_local_ns else -1
        joint_positions = [_safe_float(value) for value in message.joint_position_rad]
        gripper_position = _safe_float(message.gripper_nominal_m)
        with self._condition:
            self._state = {
                "source_session_id": str(message.session_id),
                "sequence": int(message.sequence),
                "source_monotonic_ns": int(message.source_monotonic_ns),
                "source_clock_id": str(message.source_clock_id),
                "source_local_monotonic_ns": source_local_ns,
                "source_local_clock_id": str(message.source_local_clock_id),
                "clock_mapping_uncertainty_ns": int(message.source_clock_mapping_uncertainty_ns),
                "feedback_fresh": bool(message.feedback_fresh),
                "joint_skew_ns": int(message.joint_skew_ns),
                "status_monotonic_ns": int(message.status_monotonic_ns),
                "joint_pair_monotonic_ns": [int(value) for value in message.joint_pair_monotonic_ns],
                "gripper_monotonic_ns": int(message.gripper_monotonic_ns),
                "motor_monotonic_ns": [int(value) for value in message.motor_monotonic_ns],
                "joint_position_rad": joint_positions,
                "numeric_state_valid": all(value is not None for value in joint_positions)
                and gripper_position is not None,
                "gripper_raw": int(message.gripper_raw),
                "gripper_nominal_m": gripper_position,
                "ctrl_mode_raw": int(message.ctrl_mode_raw),
                "arm_status_raw": int(message.arm_status_raw),
                "error_code": int(message.error_code),
                "motor_enabled": [bool(value) for value in message.motor_enabled],
                "hardware_connected": bool(message.hardware_connected),
                "fake": bool(message.fake),
                "age_ns": age_ns,
            }
            self._condition.notify_all()

    def update_joint_state(self, message: JointState) -> None:
        with self._condition:
            self._joint_state = {
                "names": list(message.name),
                "position": [_safe_float(value) for value in message.position],
                "velocity": [_safe_float(value) for value in message.velocity],
                "effort": [_safe_float(value) for value in message.effort],
                "stamp_sec": int(message.header.stamp.sec),
                "stamp_nanosec": int(message.header.stamp.nanosec),
                "frame_id": str(message.header.frame_id),
            }
            self._condition.notify_all()

    def update_status(self, message: DiagnosticArray) -> None:
        summary = [
            {
                "name": str(item.name),
                "level": int.from_bytes(item.level, "little")
                if isinstance(item.level, (bytes, bytearray)) else int(item.level),
                "message": str(item.message),
            }
            for item in message.status
        ]
        with self._condition:
            self._status = {
                "stamp_sec": int(message.header.stamp.sec),
                "stamp_nanosec": int(message.header.stamp.nanosec),
                "items": summary,
            }
            self._condition.notify_all()

    def update_metadata(self, message: CameraFrameMetadata) -> None:
        role = str(message.role)
        if role not in ("base", "wrist"):
            return
        with self._condition:
            self._metadata[role] = {
                "role": role,
                "session_id": str(message.session_id),
                "device_name": str(message.device_name),
                "sequence": int(message.sequence),
                "source_monotonic_ns": int(message.source_monotonic_ns),
                "source_clock_id": str(message.source_clock_id),
                "source_local_monotonic_ns": int(message.source_local_monotonic_ns),
                "source_local_clock_id": str(message.source_local_clock_id),
                "clock_mapping_uncertainty_ns": int(message.clock_mapping_uncertainty_ns),
                "dropped_frames": int(message.dropped_frames),
                "encoding": str(message.encoding),
                "width": int(message.width),
                "height": int(message.height),
            }
            self._condition.notify_all()

    def update_image(self, role: str, message: Image) -> None:
        try:
            if (int(message.width) > MAX_IMAGE_DIMENSION
                    or int(message.height) > MAX_IMAGE_DIMENSION
                    or len(message.data) > MAX_IMAGE_BYTES):
                raise ValueError("image exceeds the 4096-pixel or 64 MiB intake bound")
            bgr = image_to_bgr(message)
            ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                raise ValueError("OpenCV could not encode the image")
            jpeg = encoded.tobytes()
            error = ""
        except (ValueError, TypeError, cv2.error) as exc:
            jpeg = b""
            error = str(exc)
        now_ns = time.perf_counter_ns()
        with self._condition:
            previous = self._frames[role]
            self._frames[role] = {
                "sequence": int(previous["sequence"]) + 1,
                "jpeg": jpeg,
                "received_ns": now_ns,
                "source_stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec),
                "frame_id": str(message.header.frame_id),
                "encoding": str(message.encoding),
                "width": int(message.width),
                "height": int(message.height),
                "error": error,
            }
            self._condition.notify_all()

    def snapshot(self) -> dict:
        now_ns = time.perf_counter_ns()
        with self._condition:
            state = dict(self._state or {})
            if state:
                age_ns = now_ns - int(state.get("source_local_monotonic_ns", 0))
                state["age_ns"] = age_ns
                state["age_ms"] = age_ns / 1e6
                state["available"] = True
                state["fresh"] = bool(
                    state.get("feedback_fresh") is True
                    and state.get("numeric_state_valid") is True
                    and 0 <= age_ns <= MAX_STATE_AGE_NS
                    and (state.get("hardware_connected") is True or state.get("fake") is True)
                )
            else:
                state = {"available": False, "fresh": False, "age_ns": -1, "age_ms": None}
            cameras = {}
            for role in ("base", "wrist"):
                frame = self._frames[role]
                cameras[role] = {
                    "available": bool(frame["jpeg"]),
                    "sequence": int(frame["sequence"]),
                    "age_ms": (now_ns - int(frame["received_ns"])) / 1e6 if frame["received_ns"] else None,
                    "error": str(frame["error"]),
                    "encoding": str(frame.get("encoding", "")),
                    "width": int(frame.get("width", 0)),
                    "height": int(frame.get("height", 0)),
                    "frame_id": str(frame.get("frame_id", "")),
                    "metadata": dict(self._metadata[role]),
                }
            return {
                "protocol_version": 1,
                "state": state,
                "joint_state": dict(self._joint_state or {}),
                "status": dict(self._status),
                "cameras": cameras,
                "control_mode": "disabled",
                "hardware_control_allowed": False,
            }

    def frame_after(self, role: str, after_sequence: int, timeout_s: float) -> dict | None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                frame = self._frames[role]
                if frame["jpeg"] and int(frame["sequence"]) > after_sequence:
                    return dict(frame)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)


def _safe_float(value):
    converted = float(value)
    return converted if math.isfinite(converted) else None


def image_to_bgr(message: Image) -> np.ndarray:
    """Convert the common color encodings used by the two RGB ROS topics."""
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError("image dimensions and row step must be positive")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("image data is shorter than height * step")
    rows = raw[:height * step].reshape(height, step)
    encoding = str(message.encoding).lower()
    if encoding in {"bgr8", "rgb8", "bgra8", "rgba8"}:
        channels = 4 if encoding in {"bgra8", "rgba8"} else 3
        if step < width * channels:
            raise ValueError(f"{encoding} row step is shorter than the pixel data")
        pixels = rows[:, :width * channels].reshape(height, width, channels)
        if encoding == "bgr8":
            return pixels
        if encoding == "rgb8":
            return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    if encoding == "mono8":
        if step < width:
            raise ValueError("mono8 row step is shorter than the pixel data")
        return cv2.cvtColor(rows[:, :width], cv2.COLOR_GRAY2BGR)
    if encoding in {"yuyv", "yuyv422", "yuv422_yuy2"}:
        if step < width * 2:
            raise ValueError("YUYV row step is shorter than the pixel data")
        pixels = rows[:, :width * 2].reshape(height, width, 2)
        return cv2.cvtColor(pixels, cv2.COLOR_YUV2BGR_YUY2)
    if encoding in {"uyvy", "uyvy422"}:
        if step < width * 2:
            raise ValueError("UYVY row step is shorter than the pixel data")
        pixels = rows[:, :width * 2].reshape(height, width, 2)
        return cv2.cvtColor(pixels, cv2.COLOR_YUV2BGR_UYVY)
    raise ValueError(f"unsupported ROS image encoding: {message.encoding}")


def discover_wsl_bind_ipv4() -> str:
    """Find the WSL NAT interface by routing toward the known Windows gateway."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("172.27.64.1", 9))
        address = probe.getsockname()[0]
    finally:
        probe.close()
    if not address or address.startswith("127."):
        raise RuntimeError("cannot resolve the WSL NAT IPv4 interface; pass http_host explicitly")
    return address


class _HttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, data: LatestRosData, authority: ProposalAuthority,
                 allowed_client_ip: str):
        self.data = data
        self.authority = authority
        self.allowed_client_ip = allowed_client_ip
        super().__init__(address, _RequestHandler)


class _RequestHandler(BaseHTTPRequestHandler):
    server: _HttpServer
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        self.server.node.get_logger().info("operator_api " + fmt % args)

    def do_OPTIONS(self):
        self.send_error(405)

    def do_GET(self):
        if not self._peer_allowed():
            return
        parsed = urlsplit(self.path)
        if parsed.path == "/api/v1/healthz":
            self._json(200, {"ok": True, "service": "pi05_windows_operator_gateway", "protocol_version": 1})
            return
        if parsed.path == "/api/v1/state":
            self._json(200, self.server.data.snapshot())
            return
        parts = parsed.path.strip("/").split("/")
        if len(parts) == 5 and parts[:3] == ["api", "v1", "camera"] and parts[4] == "frame":
            role = parts[3]
            if role not in {"base", "wrist"}:
                self._json(404, {"error": "camera role must be base or wrist"})
                return
            try:
                query = parse_qs(parsed.query)
                after = int(query.get("after", ["0"])[0])
                timeout_ms = int(query.get("timeout_ms", ["1500"])[0])
                if after < 0 or not 0 <= timeout_ms <= 2000:
                    raise ValueError
            except ValueError:
                self._json(400, {"error": "invalid after or timeout_ms query"})
                return
            frame = self.server.data.frame_after(role, after, timeout_ms / 1000.0)
            if frame is None:
                self.send_response(204)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            body = frame["jpeg"]
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("X-Frame-Sequence", str(frame["sequence"]))
            self.send_header("X-Frame-Source-Stamp-Ns", str(frame["source_stamp_ns"]))
            self.send_header("X-Frame-Id", str(frame["frame_id"]))
            self.end_headers()
            self.wfile.write(body)
            return
        self._json(404, {"error": "route not found"})

    def do_POST(self):
        if not self._peer_allowed():
            return
        path = urlsplit(self.path).path
        try:
            payload = self._read_json()
        except (ValueError, UnicodeError) as exc:
            self._json(400, {"accepted": False, "decision": "REJECTED_INVALID_JSON", "reason": str(exc)})
            return
        snapshot = self.server.data.snapshot()
        state = snapshot["state"]
        if path == "/api/v1/session":
            response = self.server.authority.open_session()
            response["source_state_session_id"] = str(state.get("source_session_id", ""))
            self._json(200, response)
            return
        if path == "/api/v1/authorize":
            response = self.server.authority.authorize(payload, state=state)
            self._json(200, response)
            return
        if path == "/api/v1/control/request":
            response = self.server.authority.propose(payload, state=state)
            self._json(200, response)
            return
        self._json(404, {"error": "route not found"})

    def _read_json(self) -> dict:
        raw_length = self.headers.get("Content-Length", "0")
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if not 0 <= length <= HTTP_BODY_LIMIT:
            raise ValueError("request body must be within 64 KiB")
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise ValueError("body must be UTF-8 JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def _peer_allowed(self) -> bool:
        if self.client_address[0] == self.server.allowed_client_ip:
            return True
        self._json(403, {"error": "request source is outside the configured Windows WSL NAT gateway"})
        return False

    def _json(self, status: int, value: dict):
        body = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


class WindowsOperatorGateway(Node):
    def __init__(self):
        super().__init__("windows_operator_gateway")
        self.declare_parameter("http_host", "auto")
        self.declare_parameter("http_port", 18771)
        self.declare_parameter("allowed_client_ip", "172.27.64.1")
        self.declare_parameter("control_mode", "disabled")
        self.declare_parameter("image_preview_hz", 15.0)
        self.declare_parameter("state_topic", HTTP_STATE_TOPIC)
        self.declare_parameter("joint_state_topic", "/piper/feedback/joint_states")
        self.declare_parameter("status_topic", "/piper/status")
        self.declare_parameter("base_image_topic", HTTP_BASE_IMAGE_TOPIC)
        self.declare_parameter("wrist_image_topic", HTTP_WRIST_IMAGE_TOPIC)
        self.declare_parameter("camera_metadata_topic", HTTP_METADATA_TOPIC)
        host = str(self.get_parameter("http_host").value)
        self.http_host = discover_wsl_bind_ipv4() if host == "auto" else host
        self.http_port = int(self.get_parameter("http_port").value)
        self.allowed_client_ip = str(self.get_parameter("allowed_client_ip").value)
        self.image_preview_hz = float(self.get_parameter("image_preview_hz").value)
        control_mode = str(self.get_parameter("control_mode").value)
        if not 1 <= self.http_port <= 65535:
            raise ValueError("http_port must be within 1..65535")
        if not math.isfinite(self.image_preview_hz) or not 1 <= self.image_preview_hz <= 30:
            raise ValueError("image_preview_hz must be within 1..30")
        self.authority = ProposalAuthority(control_mode=control_mode)
        self.data = LatestRosData()
        self._next_image_ns = {"base": 0, "wrist": 0}
        self._image_period_ns = int(1e9 / self.image_preview_hz)
        self._subscriptions = [
            self.create_subscription(
                StateSnapshot, str(self.get_parameter("state_topic").value),
                self.data.update_state, qos_profile_sensor_data,
            ),
            self.create_subscription(
                JointState, str(self.get_parameter("joint_state_topic").value),
                self.data.update_joint_state, qos_profile_sensor_data,
            ),
            self.create_subscription(
                DiagnosticArray, str(self.get_parameter("status_topic").value),
                self.data.update_status, 10,
            ),
            self.create_subscription(
                CameraFrameMetadata, str(self.get_parameter("camera_metadata_topic").value),
                self.data.update_metadata, qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, str(self.get_parameter("base_image_topic").value),
                lambda message: self._on_image("base", message), qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, str(self.get_parameter("wrist_image_topic").value),
                lambda message: self._on_image("wrist", message), qos_profile_sensor_data,
            ),
        ]
        self.http = _HttpServer(
            (self.http_host, self.http_port), self.data, self.authority,
            self.allowed_client_ip,
        )
        self.http.node = self
        self._http_thread = threading.Thread(
            target=self.http.serve_forever, name="windows-operator-http", daemon=True
        )
        self._http_thread.start()
        self.get_logger().info(
            f"HTTP preview listening on http://{self.http_host}:{self.http_port}; "
            f"control_mode={control_mode}; hardware_control_allowed=false"
        )

    def _on_image(self, role: str, message: Image) -> None:
        now_ns = time.perf_counter_ns()
        if now_ns < self._next_image_ns[role]:
            return
        self._next_image_ns[role] = now_ns + self._image_period_ns
        self.data.update_image(role, message)

    def close(self) -> None:
        self.http.shutdown()
        self.http.server_close()
        self._http_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WindowsOperatorGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
