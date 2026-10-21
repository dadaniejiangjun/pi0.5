"""Concrete read-only subscribers for the existing PiPER ROS topics."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping
import uuid

import numpy as np

from gamepad_capture.live_dataset.contracts import image_header_stamp_ns
from gamepad_capture.live_dataset.pairing import BoundedObservationAssembler
from gamepad_capture.live_dataset.ros_adapter import (
    ros_metadata_to_dict,
    ros_state_to_dict,
)

_default_backend: "ExistingRosTopicBackend | None" = None
_default_users = 0


def acquire_default_backend() -> "ExistingRosTopicBackend":
    """Create/connect the standard subscriber-only backend on first use."""
    global _default_backend, _default_users
    if _default_backend is None:
        _default_backend = ExistingRosTopicBackend.from_environment()
    _default_backend.connect()
    _default_users += 1
    return _default_backend


def release_default_backend(backend: "ExistingRosTopicBackend") -> None:
    """Release one adapter reference and close the owned ROS node at zero."""
    global _default_backend, _default_users
    if backend is not _default_backend:
        return
    _default_users = max(0, _default_users - 1)
    if _default_users == 0:
        backend.disconnect()
        _default_backend = None


class ExistingRosTopicBackend:
    """Read existing gateway/camera and gamepad topics without publishing.

    Pass ``node`` and generated ``message_types`` to reuse a ROS runtime owned
    by the caller, or call ``from_environment`` to lazily create a subscriber
    node in an already-sourced ROS 2 environment. This class never opens a
    camera, creates a gateway, publishes lifecycle/control messages, or calls
    an actuator service.
    """

    def __init__(
        self,
        *,
        node: Any,
        message_types: Mapping[str, Any],
        sensor_qos: Any,
        event_qos: Any = 10,
        owns_node: bool = False,
        rclpy_module: Any | None = None,
        executor: Any | None = None,
        max_feedback_age_ms: int = 250,
        arrival_clock=time.perf_counter_ns,
    ) -> None:
        if max_feedback_age_ms <= 0:
            raise ValueError("max_feedback_age_ms must be positive")
        required = {"StateSnapshot", "CameraFrameMetadata", "Image", "String"}
        missing = required - set(message_types)
        if missing:
            raise ValueError(f"message_types missing {sorted(missing)}")
        self.node = node
        self.types = dict(message_types)
        self.sensor_qos = sensor_qos
        self.event_qos = event_qos
        self.owns_node = owns_node
        self.rclpy = rclpy_module
        self.executor = executor
        self.max_feedback_age_ns = max_feedback_age_ms * 1_000_000
        self.arrival_clock = arrival_clock
        self.assembler = BoundedObservationAssembler(
            max_action_age_ms=max_feedback_age_ms,
            pair_wait_ms=min(50, max_feedback_age_ms - 1),
            expected_session_id=None,
        )
        self.assembler.start_episode(
            "lerobot_observation_" + uuid.uuid4().hex,
            "read-only LeRobot observation",
        )
        self.subscriptions: list[Any] = []
        self._connected = False
        self._lock = threading.RLock()
        self._latest_observation: dict[str, Any] | None = None
        self._latest_target: dict[str, Any] | None = None
        self._latest_target_arrival_ns = 0
        self._states: dict[tuple[str, int], dict[str, Any]] = {}
        self._samples_emitted = 0

    @classmethod
    def from_environment(cls) -> "ExistingRosTopicBackend":
        """Build a subscriber-only ROS node from the active ROS environment."""
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image, JointState
            from std_msgs.msg import String
            from pi05_piper_interfaces.msg import CameraFrameMetadata, StateSnapshot
            from pi05_piper_interfaces.srv import SetGripper, SetJointTarget
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 and the current pi05_piper_interfaces overlay must be sourced"
            ) from exc
        initialized_here = not rclpy.ok()
        if initialized_here:
            rclpy.init(args=None)
        node = Node("pi05_lerobot_read_only_adapter")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        backend = cls(
            node=node,
            message_types={
                "StateSnapshot": StateSnapshot,
                "CameraFrameMetadata": CameraFrameMetadata,
                "Image": Image,
                "String": String,
            "JointState": JointState,
            "SetGripper": SetGripper,
            "SetJointTarget": SetJointTarget,
            },
            sensor_qos=qos_profile_sensor_data,
            owns_node=True,
            rclpy_module=rclpy,
            executor=executor,
        )
        backend._initialized_rclpy_here = initialized_here
        return backend

    def make_control_delegate(
        self, *, validity_ms: int = 200, service_timeout_s: float = 1.5,
    ) -> Any:
        """Bind the existing task-scoped ROS services; caller still supplies grant."""
        if not self.is_connected:
            raise RuntimeError("connect the existing read-only ROS topic backend first")
        required = {"JointState", "SetGripper", "SetJointTarget"}
        if not required.issubset(self.types):
            raise RuntimeError("current pi05_piper_interfaces service types are unavailable")
        from .ros_control_delegate import ExistingRosControlDelegate
        return ExistingRosControlDelegate(
            backend=self,
            service_types={
                "SetGripper": self.types["SetGripper"],
                "SetJointTarget": self.types["SetJointTarget"],
            },
            joint_state_type=self.types["JointState"],
            validity_ms=validity_ms,
            service_timeout_s=service_timeout_s,
        )

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return
        self.subscriptions = [
            self.node.create_subscription(
                self.types["StateSnapshot"], "/piper/state_snapshot",
                self._on_state, self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["CameraFrameMetadata"], "/piper/cameras/frame_metadata",
                self._on_metadata, self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["Image"], "/piper/cameras/base/image_raw",
                lambda message: self._on_image("base", message), self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["Image"], "/piper/cameras/wrist/image_raw",
                lambda message: self._on_image("wrist", message), self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["String"], "/gamepad/target_events",
                self._on_target_event, self.event_qos,
            ),
        ]
        self._connected = True
        if self.executor is not None:
            thread = threading.Thread(
                target=self.executor.spin,
                name="lerobot-existing-ros-subscriber",
                daemon=True,
            )
            self._spin_thread = thread
            thread.start()

    def _on_state(self, message: Any) -> None:
        now = self.arrival_clock()
        state = ros_state_to_dict(message)
        with self._lock:
            key = (state["session_id"], state["sequence"])
            self._states[key] = {**state, "arrival_monotonic_ns": now}
            if len(self._states) > 128:
                self._states.pop(next(iter(self._states)))
            self.assembler.add_state(state, now)
            self._collect_samples(now)

    def _on_metadata(self, message: Any) -> None:
        now = self.arrival_clock()
        with self._lock:
            self.assembler.add_metadata(ros_metadata_to_dict(message), now)
            self._collect_samples(now)

    def _on_image(self, role: str, message: Any) -> None:
        now = self.arrival_clock()
        with self._lock:
            self.assembler.add_image(role, message, image_header_stamp_ns(message), now)
            self._collect_samples(now)

    def _collect_samples(self, now_ns: int) -> None:
        for sample in self.assembler.poll(now_ns):
            self._latest_observation = sample
            self._samples_emitted += 1

    def _on_target_event(self, message: Any) -> None:
        now = self.arrival_clock()
        try:
            payload = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, Mapping) or payload.get("schema") != "piper_gamepad_target_event_v1":
            return
        if payload.get("status") != "TARGET_PREVIEW":
            return
        try:
            session_id = payload["source_session_id"]
            sequence = payload["feedback_sequence"]
            joints = payload["target_joint_rad"]
            gripper_m = payload["target_gripper_nominal_m"]
            if not isinstance(session_id, str) or type(sequence) is not int:
                return
            target = np.asarray([*joints, gripper_m], dtype=np.float32)
            if target.shape != (7,) or not np.isfinite(target).all():
                return
        except (KeyError, TypeError, ValueError):
            return
        with self._lock:
            state = self._states.get((session_id, sequence))
            if state is None:
                return
            if (
                state.get("feedback_fresh") is not True
                or state.get("hardware_connected") is not True
                or state.get("fake") is not False
                or now - state["arrival_monotonic_ns"] > self.max_feedback_age_ns
            ):
                return
            self._latest_target = {
                "operator_intent": {
                    "input_axes": payload.get("input_axes"),
                    "input_buttons": payload.get("input_buttons"),
                    "action_semantics": payload.get("action_semantics"),
                    "status": payload.get("status"),
                },
                "safe_target": target,
                # TARGET_PREVIEW is not a gateway acceptance receipt.
                "accepted": None,
                "feedback": {
                    "state": list(state["joint_position_rad"])
                    + [state["gripper_nominal_m"]],
                    "session_id": session_id,
                    "sequence": sequence,
                    "feedback_fresh": state["feedback_fresh"],
                    "hardware_connected": state["hardware_connected"],
                    "fake": state["fake"],
                },
                "session_id": session_id,
                "source_sequence": sequence,
                "arrival_monotonic_ns": now,
            }
            self._latest_target_arrival_ns = now

    def latest_observation(self) -> Mapping[str, Any]:
        now = self.arrival_clock()
        with self._lock:
            self._collect_samples(now)
            sample = self._latest_observation
            if sample is None:
                raise TimeoutError("no fresh state plus paired Base/Wrist RGB frames received yet")
            state_meta = sample["source_map"]["state"]
            if now - state_meta["arrival_monotonic_ns"] > self.max_feedback_age_ns:
                raise TimeoutError("latest paired observation exceeds the feedback freshness limit")
            return sample

    def latest_target(self) -> Mapping[str, Any]:
        now = self.arrival_clock()
        with self._lock:
            target = self._latest_target
            if target is None or now - self._latest_target_arrival_ns > self.max_feedback_age_ns:
                raise TimeoutError("no fresh existing post-IK gamepad target is available")
            return dict(target)

    def disconnect(self) -> None:
        self._connected = False
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1.0)
            self.executor = None
        for subscription in self.subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception:
                pass
        self.subscriptions.clear()
        if self.owns_node:
            self.node.destroy_node()
            self.node = None
            if getattr(self, "_initialized_rclpy_here", False) and self.rclpy.ok():
                self.rclpy.shutdown()
