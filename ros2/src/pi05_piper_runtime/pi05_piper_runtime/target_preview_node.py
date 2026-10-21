"""Publish bounded ROS target previews without creating hardware commands."""
from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException


def _project_root() -> Path:
    default = Path(__file__).resolve().parents[4]
    return Path(os.environ.get("PI05_PROJECT_ROOT", str(default))).resolve()


class TargetPreviewNode(Node):
    """Select one fresh future row per policy chunk and publish it as preview."""

    def __init__(self) -> None:
        super().__init__("pi05_target_preview")
        self.project_root = _project_root()
        if str(self.project_root) not in sys.path:
            sys.path.insert(0, str(self.project_root))

        self.declare_parameter("allow_fake_observations", False)
        self.declare_parameter("max_joint_step_deg", 20.0)
        self.declare_parameter("speed_percent", 5)
        self.declare_parameter("update_period_ms", 100)
        self.declare_parameter("max_state_age_ms", 120)
        self.declare_parameter("max_chunk_age_ms", 250)
        self.declare_parameter("max_response_age_ms", 250)
        self.declare_parameter("max_observation_age_ms", 500)
        self.declare_parameter("max_inference_latency_ms", 500)
        self.declare_parameter("max_preview_window_s", 120.0)
        self.allow_fake = bool(self.get_parameter("allow_fake_observations").value)
        speed_percent = int(self.get_parameter("speed_percent").value)
        update_period_ms = int(self.get_parameter("update_period_ms").value)
        max_preview_window_s = float(self.get_parameter("max_preview_window_s").value)
        if not math.isfinite(max_preview_window_s) or max_preview_window_s <= 0:
            raise ValueError("max_preview_window_s must be finite and positive")

        from laptop.piper.unit_conversion import PIPER_NOMINAL_JOINT_BOUNDS_DEG
        from pi05_piper_interfaces.msg import (
            PolicyChunk,
            StateSnapshot,
            TargetPreview,
        )
        from .chunk_scheduler_core import (
            LOCAL_MONOTONIC_CLOCK_ID,
            SchedulerLimits,
            TargetRejected,
            preview_next_target,
        )

        self._TargetPreview = TargetPreview
        self._TargetRejected = TargetRejected
        self._preview_next_target = preview_next_target
        self._clock_id = LOCAL_MONOTONIC_CLOCK_ID
        self._limits = SchedulerLimits(
            joint_min_rad=tuple(math.radians(pair[0]) for pair in PIPER_NOMINAL_JOINT_BOUNDS_DEG),
            joint_max_rad=tuple(math.radians(pair[1]) for pair in PIPER_NOMINAL_JOINT_BOUNDS_DEG),
            max_joint_step_deg=float(self.get_parameter("max_joint_step_deg").value),
            speed_percent=speed_percent,
            update_period_ns=update_period_ms * 1_000_000,
            max_state_age_ns=int(self.get_parameter("max_state_age_ms").value) * 1_000_000,
            max_chunk_age_ns=int(self.get_parameter("max_chunk_age_ms").value) * 1_000_000,
            max_response_age_ns=int(self.get_parameter("max_response_age_ms").value) * 1_000_000,
            max_observation_age_ns=int(self.get_parameter("max_observation_age_ms").value) * 1_000_000,
            max_inference_latency_ns=int(self.get_parameter("max_inference_latency_ms").value) * 1_000_000,
            max_task_duration_ns=int(max_preview_window_s * 1_000_000_000),
        )
        if update_period_ms != 100:
            raise ValueError("the selected rolling preview profile fixes target updates at 10 Hz")

        self._state: Any | None = None
        self._state_received_ns: int | None = None
        self._chunk: Any | None = None
        self._chunk_received_ns: int | None = None
        self._preview_window_started_ns: int | None = None
        self._last_preview_ns: int | None = None
        self._last_observation_id: str | None = None
        self._last_status_signature = ""

        self.state_sub = self.create_subscription(
            StateSnapshot, "/piper/state_snapshot", self._on_state, qos_profile_sensor_data
        )
        self.chunk_sub = self.create_subscription(
            PolicyChunk, "/pi05/policy_chunk", self._on_chunk, 10
        )
        self.preview_pub = self.create_publisher(
            TargetPreview, "/pi05/target_preview", 10
        )
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/pi05/target_preview/status", 10
        )
        # Poll more often than the output cadence so timer jitter cannot make a
        # 99 ms tick permanently discard a valid 10 Hz policy update.
        self._timer = self.create_timer(0.02, self._tick)
        self.get_logger().info(
            "prediction-only scheduler ready; output=/pi05/target_preview; CAN access=none"
        )

    def _on_state(self, message: Any) -> None:
        self._state = message
        self._state_received_ns = time.monotonic_ns()

    def _on_chunk(self, message: Any) -> None:
        self._chunk = message
        self._chunk_received_ns = time.monotonic_ns()

    def _publish_status(self, *, level: int, message: str, now_ns: int) -> None:
        signature = f"{level}:{message}"
        if signature == self._last_status_signature and now_ns - getattr(
            self, "_last_status_ns", 0
        ) < 2_000_000_000:
            return
        self._last_status_signature = signature
        self._last_status_ns = now_ns
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "pi05_target_preview"
        status.level = level
        status.message = message
        status.hardware_id = "prediction-only"
        status.values = [
            KeyValue(key="local_clock_id", value=self._clock_id),
            KeyValue(key="hardware_control_allowed", value="false"),
            KeyValue(key="emitted_to_hardware", value="false"),
        ]
        array.status = [status]
        self.status_pub.publish(array)

    def _tick(self) -> None:
        now_ns = time.monotonic_ns()
        if self._state is None or self._chunk is None:
            missing = "state" if self._state is None else "policy chunk"
            self._publish_status(
                level=DiagnosticStatus.WARN,
                message=f"waiting for {missing}",
                now_ns=now_ns,
            )
            return
        if self._state_received_ns is None or self._chunk_received_ns is None:
            self._publish_status(
                level=DiagnosticStatus.ERROR,
                message="receive timestamp is missing",
                now_ns=now_ns,
            )
            return
        observation_id = str(self._chunk.observation_id)
        if observation_id == self._last_observation_id:
            return
        if self._preview_window_started_ns is None:
            self._preview_window_started_ns = now_ns
        try:
            preview = self._preview_next_target(
                chunk=self._chunk,
                state=self._state,
                now_monotonic_ns=now_ns,
                task_started_monotonic_ns=self._preview_window_started_ns,
                chunk_received_monotonic_ns=self._chunk_received_ns,
                state_received_monotonic_ns=self._state_received_ns,
                last_preview_monotonic_ns=self._last_preview_ns,
                last_observation_id=self._last_observation_id,
                limits=self._limits,
                allow_fake=self.allow_fake,
            )
        except self._TargetRejected as exc:
            if str(exc) == "10 Hz target update is not due":
                return
            self._publish_status(
                level=DiagnosticStatus.WARN,
                message=f"preview rejected: {exc}",
                now_ns=now_ns,
            )
            return

        message = self._TargetPreview()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = str(self._chunk.model_id)
        message.checkpoint_id = str(self._chunk.checkpoint_id)
        message.source_observation_id = preview.source_observation_id
        message.state_sequence = preview.state_sequence
        message.row_index = preview.row_index
        message.target_local_monotonic_ns = preview.target_time_monotonic_ns
        message.target_expiry_monotonic_ns = preview.target_expiry_monotonic_ns
        message.local_clock_id = self._clock_id
        message.action_semantics = preview.semantics
        message.joint_position_rad = list(preview.target_position[:6])
        message.gripper_nominal_m = preview.target_position[6]
        message.speed_percent = preview.speed_percent
        message.state_age_ns = preview.state_age_ns
        message.observation_age_ns = preview.observation_age_ns
        message.prediction_age_ns = preview.prediction_age_ns
        message.chunk_age_ns = preview.chunk_age_ns
        message.inference_latency_ns = preview.inference_latency_ns
        message.preview_window_elapsed_ns = preview.task_elapsed_ns
        message.prediction_only = True
        message.emitted_to_hardware = False
        message.hardware_control_allowed = False
        self.preview_pub.publish(message)
        self._last_preview_ns = now_ns
        self._last_observation_id = preview.source_observation_id
        self._publish_status(
            level=DiagnosticStatus.OK,
            message="fresh bounded target preview published; no hardware command was sent",
            now_ns=now_ns,
        )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = TargetPreviewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
