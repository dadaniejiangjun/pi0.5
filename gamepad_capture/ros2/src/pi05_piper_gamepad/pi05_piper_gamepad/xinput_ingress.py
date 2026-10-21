"""Receive Windows XInput frames and publish read-only ROS Joy input."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import queue
import socket
import socketserver
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import String


_PROJECT_ROOT = Path(os.environ.get(
    "PI05_PROJECT_ROOT", "/mnt/e/01_NUS/02_5003/12_pi0.5"
)).resolve()
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from gamepad_capture.protocol import INPUT_SCHEMA, neutral_joy, validate_input_frame  # noqa: E402


MAX_FRAME_BYTES = 8192


class _InputServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, handler, node):
        self.node = node
        self.client_lock = threading.Lock()
        super().__init__(address, handler)


class _InputHandler(socketserver.BaseRequestHandler):
    def handle(self):
        server: _InputServer = self.server
        if not server.client_lock.acquire(blocking=False):
            server.node.enqueue_event("connection_rejected", reason="sender_already_connected")
            return
        buffer = bytearray()
        last_byte_ns = time.monotonic_ns()
        self.request.settimeout(0.2)
        try:
            while not server.node.stop_event.is_set():
                try:
                    chunk = self.request.recv(4096)
                except socket.timeout:
                    if time.monotonic_ns() - last_byte_ns > 2_000_000_000:
                        break
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                last_byte_ns = time.monotonic_ns()
                buffer.extend(chunk)
                if len(buffer) > MAX_FRAME_BYTES and b"\n" not in buffer:
                    server.node.enqueue_event("input_rejected", reason="frame_too_large")
                    break
                while b"\n" in buffer:
                    line, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    if len(line) > MAX_FRAME_BYTES:
                        server.node.enqueue_event("input_rejected", reason="frame_too_large")
                        return
                    try:
                        decoded = json.loads(line.decode("utf-8"))
                        frame = validate_input_frame(decoded)
                    except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                        server.node.enqueue_event("input_rejected", reason=str(exc))
                        return
                    server.node.enqueue_frame(frame, time.monotonic_ns())
        finally:
            server.client_lock.release()
            server.node.enqueue_event("sender_disconnected")


class XInputIngress(Node):
    def __init__(self):
        super().__init__("gamepad_xinput_ingress")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 18901)
        self.declare_parameter("timeout_ms", 250)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("status_topic", "/gamepad/input_status")
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.timeout_ns = int(self.get_parameter("timeout_ms").value) * 1_000_000
        if not 1 <= port <= 65535 or self.timeout_ns < 50_000_000:
            raise ValueError("port must be 1..65535 and timeout_ms at least 50")

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.joy_pub = self.create_publisher(
            Joy, str(self.get_parameter("joy_topic").value), qos
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 50
        )
        self.events: queue.Queue = queue.Queue(maxsize=512)
        self.stop_event = threading.Event()
        self.queue_lock = threading.Lock()
        self.dropped_frames = 0
        self.last_sequence = None
        self.last_session_id = None
        self.sequence_gaps = 0
        self.last_received_ns = None
        self.last_device_index = 0
        self.last_joy_neutral = True
        self.stale_reported = False

        self.server = _InputServer((host, port), _InputHandler, self)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            kwargs={"poll_interval": 0.2},
            name="gamepad-xinput-tcp",
            daemon=True,
        )
        self.server_thread.start()
        self.timer = self.create_timer(0.01, self.drain_events)
        self.get_logger().info(
            f"read-only XInput ingress listening on {host}:{port}; "
            f"publishing /joy and {self.get_parameter('status_topic').value}; no robot topics"
        )

    def enqueue_frame(self, frame: dict, received_ns: int) -> None:
        with self.queue_lock:
            try:
                self.events.put_nowait(("frame", frame, received_ns))
            except queue.Full:
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    pass
                self.dropped_frames += 1
                self.events.put_nowait(("frame", frame, received_ns))

    def enqueue_event(self, name: str, **fields) -> None:
        try:
            self.events.put_nowait(("event", {"event": name, **fields}, time.monotonic_ns()))
        except queue.Full:
            with self.queue_lock:
                self.dropped_frames += 1

    def publish_joy(self, axes, buttons, device_index: int) -> None:
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = f"xinput:{device_index}"
        message.axes = [float(value) for value in axes]
        message.buttons = [int(value) for value in buttons]
        self.joy_pub.publish(message)

    def publish_status(self, fields: dict) -> None:
        message = String()
        message.data = json.dumps(fields, separators=(",", ":"), allow_nan=False)
        self.status_pub.publish(message)

    def neutralize(self, reason: str) -> None:
        axes, buttons = neutral_joy()
        self.publish_joy(axes, buttons, self.last_device_index)
        self.last_joy_neutral = True
        self.publish_status({
            "schema": "piper_gamepad_input_status_v1",
            "event": reason,
            "received_monotonic_ns": time.monotonic_ns(),
            "sequence_gaps": self.sequence_gaps,
            "dropped_frames": self.dropped_frames,
            "connected": False,
        })

    def drain_events(self) -> None:
        processed = 0
        while processed < 128:
            try:
                kind, payload, received_ns = self.events.get_nowait()
            except queue.Empty:
                break
            processed += 1
            if kind == "event":
                self.publish_status({
                    "schema": "piper_gamepad_input_status_v1",
                    **payload,
                    "received_monotonic_ns": received_ns,
                })
                if payload.get("event") in ("input_rejected", "sender_disconnected"):
                    self.neutralize(payload["event"])
                continue

            frame = payload
            if frame["session_id"] != self.last_session_id:
                self.last_session_id = frame["session_id"]
                self.last_sequence = None
            sequence = frame["sequence"]
            if self.last_sequence is not None:
                if sequence <= self.last_sequence:
                    self.publish_status({
                        "schema": "piper_gamepad_input_status_v1",
                        "event": "input_rejected",
                        "reason": "non_increasing_sequence",
                        "session_id": frame["session_id"],
                        "sequence": sequence,
                        "received_monotonic_ns": received_ns,
                    })
                    self.neutralize("non_increasing_sequence")
                    continue
                self.sequence_gaps += max(0, sequence - self.last_sequence - 1)
            self.last_sequence = sequence
            self.last_received_ns = received_ns
            self.last_device_index = frame["device_index"]
            self.stale_reported = False
            joy = frame["joy"]
            axes = joy["axes"] if frame["connected"] else neutral_joy()[0]
            buttons = joy["buttons"] if frame["connected"] else neutral_joy()[1]
            self.publish_joy(axes, buttons, self.last_device_index)
            self.last_joy_neutral = not frame["connected"] or (
                not any(buttons) and not any(abs(float(value)) > 1e-9 for value in axes)
            )
            self.publish_status({
                "schema": "piper_gamepad_input_status_v1",
                "event": "input_frame",
                "session_id": frame["session_id"],
                "sequence": sequence,
                "sequence_gaps": self.sequence_gaps,
                "device_index": frame["device_index"],
                "connected": frame["connected"],
                "packet_number": frame["packet_number"],
                "source_clock": frame["source_clock"],
                "source_time_ns": frame["source_time_ns"],
                "received_monotonic_ns": received_ns,
                "raw": frame["raw"],
                "dropped_frames": self.dropped_frames,
            })

        if self.last_received_ns is not None and not self.stale_reported:
            age_ns = time.monotonic_ns() - self.last_received_ns
            if age_ns > self.timeout_ns:
                self.neutralize("input_timeout")
                self.stale_reported = True

    def destroy_node(self):
        self.stop_event.set()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    # Let the node clean up its socket server before the ROS context shuts down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = XInputIngress()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
