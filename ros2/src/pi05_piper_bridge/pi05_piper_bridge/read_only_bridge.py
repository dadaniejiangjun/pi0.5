"""Read-only ROS 2 bridge for the Windows PiPER gateway.

This first bridge deliberately has no command subscription or CAN code. It
publishes only data received from the configured gateway.
"""
from __future__ import annotations

import math
import os
import queue
import select
import socket
import sys
import threading
import time
import uuid
from array import array
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header

from pi05_piper_interfaces.msg import CameraFrameMetadata, StateSnapshot
from pi05_piper_interfaces.srv import (
    HomeArm, SetControlMode, SetGripper, SetJointTarget, SetMotorEnabled,
)
from .image_rate_limiter import ImageRateGate
from .control_receipt_core import valid_control_receipt


UINT64_MAX = (1 << 64) - 1
SERVICE_WAIT_S = 1.5


@dataclass
class _PendingControlCall:
    session_id: str
    event: threading.Event
    result: dict | None = None
    queued_monotonic_ns: int = field(default_factory=time.perf_counter_ns)


class ReadOnlyBridge(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        for service in (SetJointTarget, SetGripper):
            missing = {"source_session_id", "source_state_sequence"} - set(
                service.Request.get_fields_and_field_types()
            )
            if missing:
                raise RuntimeError(
                    f"Stale ROS {service.__name__} interface: missing {sorted(missing)}. "
                    "Rebuild pi05_piper_interfaces and source the current P2 overlay last."
                )
        super().__init__(
            "pi05_read_only_bridge", parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("host", os.environ.get("PI05_ROS_GATEWAY_HOST", ""))
        self.declare_parameter(
            "control_port", int(os.environ.get("PI05_ROS_CONTROL_PORT", "18765"))
        )
        self.declare_parameter(
            "base_image_port", int(os.environ.get("PI05_ROS_BASE_IMAGE_PORT", "18766"))
        )
        self.declare_parameter(
            "wrist_image_port", int(os.environ.get("PI05_ROS_WRIST_IMAGE_PORT", "18767"))
        )
        self.declare_parameter(
            "image_publish_hz", float(os.environ.get("PI05_ROS_IMAGE_PUBLISH_HZ", "30"))
        )
        self.declare_parameter("connect_timeout_s", 2.0)
        self.declare_parameter("client_name", "pi05_ros_read_only_bridge")
        self.host = str(self.get_parameter("host").value)
        self.control_port = int(self.get_parameter("control_port").value)
        self.image_ports = {
            "base": int(self.get_parameter("base_image_port").value),
            "wrist": int(self.get_parameter("wrist_image_port").value),
        }
        image_publish_hz = float(self.get_parameter("image_publish_hz").value)
        self.image_rate_gates = {
            role: ImageRateGate(image_publish_hz)
            for role in ("base", "wrist") if self.image_ports[role]
        }
        self.connect_timeout_s = float(self.get_parameter("connect_timeout_s").value)
        self.client_name = str(self.get_parameter("client_name").value)
        if self.host in ("", "0.0.0.0", "::"):
            raise ValueError("gateway host must name an explicit destination interface")
        if not 0.1 <= self.connect_timeout_s <= 10.0:
            raise ValueError("connect_timeout_s must be in [0.1, 10]")
        if not 1 <= self.control_port <= 65535:
            raise ValueError("control_port must be within 1..65535")
        for role, port in self.image_ports.items():
            if port != 0 and not 1 <= port <= 65535:
                raise ValueError(f"{role}_image_port must be 0 or within 1..65535")

        self._owner_registration = None
        project_root = os.environ.get("PI05_PROJECT_ROOT")
        if project_root:
            if project_root not in sys.path:
                sys.path.insert(0, project_root)
            try:
                from laptop.ros_gateway.runtime_owner_registry import (
                    RuntimeOwnerRegistration,
                    runtime_owner_directory,
                )
                self._owner_registration = RuntimeOwnerRegistration(
                    "ros_bridge",
                    runtime_owner_directory(project_root),
                    details={
                        "profile": os.environ.get("PI05_ROS_PROFILE", "unknown"),
                        "profile_sha256": os.environ.get("PI05_ROS_PROFILE_SHA256", ""),
                        "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
                        "gateway_host": self.host,
                        "control_port": self.control_port,
                        "base_image_port": self.image_ports["base"],
                        "wrist_image_port": self.image_ports["wrist"],
                        "image_publish_hz": image_publish_hz,
                        "workspace": os.environ.get("PI05_ROS_WORKSPACE", ""),
                        "overlay": os.environ.get("PI05_ROS_OVERLAY", ""),
                        "client_name": self.client_name,
                    },
                )
            except (OSError, ValueError, TypeError) as exc:
                self.get_logger().warning(f"runtime owner registration unavailable: {exc}")

        self.state_pub = self.create_publisher(StateSnapshot, "/piper/state_snapshot", 10)
        self.joint_pub = self.create_publisher(JointState, "/piper/feedback/joint_states", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(DiagnosticArray, "/piper/status", 10)
        self.image_pubs = {
            role: self.create_publisher(
                Image, f"/piper/cameras/{role}/image_raw", qos_profile_sensor_data
            )
            for role, port in self.image_ports.items() if port
        }
        self.metadata_pub = self.create_publisher(
            CameraFrameMetadata, "/piper/cameras/frame_metadata", qos_profile_sensor_data
        )

        # These services cross the existing TCP connection only. The paired
        # Windows gateway currently answers every request with a read-only
        # rejection and never dispatches a CAN command.
        self._service_callback_group = ReentrantCallbackGroup()
        self._control_request_queue: queue.Queue = queue.Queue(maxsize=16)
        self._pending_control_lock = threading.Lock()
        self._pending_control_calls: dict[str, _PendingControlCall] = {}
        self._used_control_request_ids: set[str] = set()
        self._management_services = [
            self.create_service(
                SetJointTarget, "/piper/control/set_joint_target",
                self._on_set_joint_target, callback_group=self._service_callback_group,
            ),
            self.create_service(
                SetGripper, "/piper/control/set_gripper",
                self._on_set_gripper, callback_group=self._service_callback_group,
            ),
            self.create_service(
                SetControlMode, "/piper/control/set_control_mode",
                self._on_set_control_mode, callback_group=self._service_callback_group,
            ),
            self.create_service(
                SetMotorEnabled, "/piper/control/set_motor_enabled",
                self._on_set_motor_enabled, callback_group=self._service_callback_group,
            ),
            self.create_service(
                HomeArm, "/piper/control/home_arm",
                self._on_home_arm, callback_group=self._service_callback_group,
            ),
        ]

        self.stop_event = threading.Event()
        self.clock_lock = threading.Lock()
        self.sequence_lock = threading.Lock()
        self._session_id = ""
        self._connected = False
        self._hardware_connected = False
        self._feedback_fresh = False
        self._camera_ready = False
        self._fake_profile = False
        self.camera_identity: dict = {}
        self._last_status = "starting"
        self._remote_to_local_mono_ns: Optional[int] = None
        self._clock_uncertainty_ns = UINT64_MAX
        self._clock_map_updated_local_ns: Optional[int] = None
        self._last_heartbeat_send_ns: Optional[int] = None
        self._heartbeat_seq = 0
        self._heartbeat_send_times: dict[int, int] = {}
        self._clock_sync_estimates: dict[int, tuple[int, int, int]] = {}
        self._last_state_sequence = 0
        self._state_snapshot_lock = threading.Lock()
        self._last_state_source_local_ns = 0
        self._last_state_mapping_uncertainty_ns = UINT64_MAX
        self._last_state_feedback_fresh = False
        self._recent_state_sources = deque(maxlen=4)
        self._image_sequences: dict[str, int] = {}
        self._image_metric_lock = threading.Lock()
        self._image_publish_metrics = {
            role: {"count": 0, "first_ns": None, "last_ns": None,
                   "last_sequence": 0, "publish_ms": deque(maxlen=128)}
            for role in ("base", "wrist")
        }
        self._threads: list[threading.Thread] = []
        self._state_metric_lock = threading.Lock()
        self._state_metrics = {
            "received_count": 0, "published_count": 0, "rejected_count": 0,
            "wire_sequence_gaps": 0, "last_wire_sequence": 0,
            "first_received_ns": None, "last_received_ns": None,
            "first_published_ns": None, "last_published_ns": None,
            "partial_read_timeouts": 0, "idle_read_timeouts": 0,
            "last_rejection": "", "publish_ms": deque(maxlen=128),
        }
        self._gateway_timing = {}
        self._diag_timer = self.create_timer(0.5, self._publish_diagnostics)

    def start_workers(self) -> None:
        self._threads.append(threading.Thread(target=self._control_worker, name="gateway-control", daemon=True))
        self._threads[-1].start()
        for role, port in self.image_ports.items():
            if port:
                thread = threading.Thread(
                    target=self._image_worker, args=(role, port),
                    name=f"gateway-image-{role}", daemon=True,
                )
                self._threads.append(thread)
                thread.start()
        self._update_runtime_owner(status="running", session_id=None)

    def stop_workers(self) -> None:
        self.stop_event.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()

    def _update_runtime_owner(self, *, status: str, session_id: str | None,
                              **details) -> None:
        if self._owner_registration is None:
            return
        try:
            self._owner_registration.update(
                status=status, session_id=session_id, **details,
            )
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"runtime owner registration update failed: {exc}")

    def close_runtime_owner(self) -> None:
        if self._owner_registration is not None:
            self._owner_registration.close()
            self._owner_registration = None

    def _set_connection(self, connected: bool, text: str) -> None:
        self._connected = connected
        self._last_status = text

    @staticmethod
    def _service_result(response, *, request_id: str, accepted: bool,
                        decision: str, reason: str):
        response.accepted = accepted
        response.decision = decision
        response.reason = reason[:256]
        response.request_id = request_id[:128]
        return response

    @staticmethod
    def _valid_request_id(request_id: str) -> bool:
        return (
            isinstance(request_id, str) and request_id.isascii()
            and bool(request_id.strip()) and len(request_id) <= 128
        )

    def _control_rpc(self, *, request_id: str, operation: str,
                     task_scope_id: str, validity_ms: int,
                     arguments: dict,
                     source_state_sequence: int = 0) -> tuple[bool, str, str]:
        if not self._valid_request_id(request_id):
            return False, "REJECTED_INVALID_REQUEST_ID", "request_id must be bounded non-empty ASCII text"
        if not self._connected or not self._session_id:
            return False, "REJECTED_NO_GATEWAY_SESSION", "no active Windows gateway session"
        if isinstance(validity_ms, bool) or not isinstance(validity_ms, int) or not 0 <= validity_ms <= 5_000:
            return False, "REJECTED_INVALID_VALIDITY", "validity_ms must be within 0..5000"

        pending = _PendingControlCall(self._session_id, threading.Event())
        with self._pending_control_lock:
            if request_id in self._used_control_request_ids:
                return False, "REJECTED_REPLAYED_REQUEST_ID", "request_id was already used by this ROS bridge"
            if len(self._used_control_request_ids) >= 4096:
                return False, "REJECTED_REQUEST_WINDOW_FULL", "reconnect bridge to start a new request-id window"
            self._used_control_request_ids.add(request_id)
            self._pending_control_calls[request_id] = pending
            try:
                self._control_request_queue.put_nowait({
                    "session_id": pending.session_id,
                    "request_id": request_id,
                    "operation": operation,
                    "task_scope_id": task_scope_id,
                    "source_state_sequence": source_state_sequence,
                    "validity_ms": validity_ms,
                    "arguments": arguments,
                    "pending": pending,
                })
            except queue.Full:
                self._pending_control_calls.pop(request_id, None)
                return False, "REJECTED_CONTROL_QUEUE_FULL", "Windows gateway request queue is full"

        if not pending.event.wait(SERVICE_WAIT_S):
            with self._pending_control_lock:
                self._pending_control_calls.pop(request_id, None)
            pending.result = {
                "accepted": False,
                "decision": "REJECTED_CONTROL_REQUEST_TIMEOUT",
                "reason": "gateway did not return a result before the deadline; request was not retried",
            }
            pending.event.set()
            return False, "REJECTED_CONTROL_REQUEST_TIMEOUT", "gateway did not return a result before the deadline; request was not retried"
        result = pending.result or {}
        return (
            result.get("accepted") is True,
            str(result.get("decision", "REJECTED_MISSING_GATEWAY_RESULT")),
            str(result.get("reason", "gateway returned no reason")),
        )

    def _validate_target_source(self, request) -> tuple[bool, str, str]:
        from .control_source_core import validate_bridge_control_source

        with self._state_snapshot_lock:
            latest_sequence = self._last_state_sequence
            latest_source_local_ns = self._last_state_source_local_ns
            uncertainty_ns = self._last_state_mapping_uncertainty_ns
            latest_state_feedback_fresh = self._last_state_feedback_fresh
            recent_state_sources = tuple(self._recent_state_sources)
        try:
            validate_bridge_control_source(
                recent_state_sources=recent_state_sources,
                source_session_id=request.source_session_id,
                source_state_sequence=request.source_state_sequence,
                active_session_id=self._session_id,
                latest_state_sequence=latest_sequence,
                latest_state_monotonic_ns=latest_source_local_ns,
                latest_mapping_uncertainty_ns=uncertainty_ns,
                latest_state_feedback_fresh=latest_state_feedback_fresh,
                now_monotonic_ns=time.perf_counter_ns(),
            )
        except (TypeError, ValueError) as exc:
            return False, "REJECTED_STALE_OR_MISMATCHED_SOURCE_STATE", str(exc)[:192]
        return True, "", ""

    def _on_set_joint_target(self, request, response):
        target = request.target
        names = list(target.name)
        positions = list(target.position)
        canonical = [f"joint{index}" for index in range(1, 7)]
        if (names != canonical or len(positions) != 6
                or target.velocity or target.effort
                or not all(math.isfinite(value) for value in positions)
                or not isinstance(request.task_scope_id, str)
                or not request.task_scope_id.strip()
                or len(request.task_scope_id) > 128
                or type(request.validity_ms) is not int
                or not 1 <= request.validity_ms <= 250):
            return self._service_result(
                response, request_id=request.request_id, accepted=False,
                decision="REJECTED_INVALID_JOINT_TARGET",
                reason="target needs joint1..joint6 radians, task scope, and 1..250 ms validity",
            )
        source_ok, source_decision, source_reason = self._validate_target_source(request)
        if not source_ok:
            return self._service_result(
                response, request_id=request.request_id, accepted=False,
                decision=source_decision, reason=source_reason,
            )
        accepted, decision, reason = self._control_rpc(
            request_id=request.request_id, operation="set_joint_target",
            task_scope_id=request.task_scope_id,
            validity_ms=request.validity_ms,
            source_state_sequence=request.source_state_sequence,
            arguments={"joint_position_rad": positions},
        )
        return self._service_result(
            response, request_id=request.request_id, accepted=accepted,
            decision=decision, reason=reason,
        )

    def _on_set_gripper(self, request, response):
        from laptop.piper.control_profile import (
            CONTINUOUS_GRIPPER_LIMITS,
            GRIPPER_NOMINAL_METERS_PER_RAW,
        )

        values = (request.nominal_position_m, request.nominal_effort_nm)
        if (type(request.gripper_raw) is not int
                or not CONTINUOUS_GRIPPER_LIMITS.minimum_raw
                    <= request.gripper_raw <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw
                or not all(math.isfinite(value) for value in values)
                or abs(float(request.nominal_position_m)
                       - request.gripper_raw * GRIPPER_NOMINAL_METERS_PER_RAW)
                    > GRIPPER_NOMINAL_METERS_PER_RAW / 2
                or float(request.nominal_effort_nm) * 1000
                    != CONTINUOUS_GRIPPER_LIMITS.effort_raw
                or not isinstance(request.task_scope_id, str)
                or not request.task_scope_id.strip()
                or len(request.task_scope_id) > 128
                or type(request.validity_ms) is not int
                or not 1 <= request.validity_ms <= 250):
            return self._service_result(
                response, request_id=request.request_id, accepted=False,
                decision="REJECTED_INVALID_GRIPPER_TARGET",
                reason="gripper target must match the tested continuous range, raw conversion, zero effort, task scope, and 1..250 ms validity",
            )
        source_ok, source_decision, source_reason = self._validate_target_source(request)
        if not source_ok:
            return self._service_result(
                response, request_id=request.request_id, accepted=False,
                decision=source_decision, reason=source_reason,
            )
        accepted, decision, reason = self._control_rpc(
            request_id=request.request_id, operation="set_gripper",
            task_scope_id=request.task_scope_id,
            validity_ms=request.validity_ms,
            source_state_sequence=request.source_state_sequence,
            arguments={
                "gripper_raw": request.gripper_raw,
                "nominal_position_m": float(request.nominal_position_m),
                "nominal_effort_nm": float(request.nominal_effort_nm),
            },
        )
        return self._service_result(
            response, request_id=request.request_id, accepted=accepted,
            decision=decision, reason=reason,
        )

    def _on_set_control_mode(self, request, response):
        if type(request.requested_mode) is not int or not 0 <= request.requested_mode <= 0xFF:
            return self._service_result(
                response, request_id=request.request_id, accepted=False,
                decision="REJECTED_INVALID_CONTROL_MODE", reason="requested_mode must be an 8-bit value",
            )
        accepted, decision, reason = self._control_rpc(
            request_id=request.request_id, operation="set_control_mode",
            task_scope_id="", validity_ms=0,
            arguments={"requested_mode": request.requested_mode},
        )
        return self._service_result(
            response, request_id=request.request_id, accepted=accepted,
            decision=decision, reason=reason,
        )

    def _on_set_motor_enabled(self, request, response):
        if type(request.enable) is not bool:
            return self._service_result(
                response, request_id=request.request_id, accepted=False,
                decision="REJECTED_INVALID_MOTOR_REQUEST", reason="enable must be boolean",
            )
        accepted, decision, reason = self._control_rpc(
            request_id=request.request_id, operation="set_motor_enabled",
            task_scope_id="", validity_ms=0,
            arguments={"enable": request.enable},
        )
        return self._service_result(
            response, request_id=request.request_id, accepted=accepted,
            decision=decision, reason=reason,
        )

    def _on_home_arm(self, request, response):
        # A ROS-provided token is not local operator authorization. Do not put
        # it on the network or treat it as permission to reset/home the robot.
        return self._service_result(
            response, request_id=request.request_id, accepted=False,
            decision="REJECTED_LOCAL_OPERATOR_AUTHORITY_REQUIRED",
            reason="home/reset is not exposed through remote RPC in the read-only P2 profile",
        )

    def _fail_pending_control_calls(self, reason: str) -> None:
        with self._pending_control_lock:
            pending = list(self._pending_control_calls.values())
            self._pending_control_calls.clear()
        for call in pending:
            call.result = {
                "accepted": False,
                "decision": "REJECTED_GATEWAY_SESSION_LOST",
                "reason": reason,
            }
            call.event.set()

    def _control_worker(self) -> None:
        # Import from the canonical Windows-side project source tree. The path
        # can be overridden for another checkout or workspace mount.
        project_root = os.environ.get(
            "PI05_PROJECT_ROOT", "/mnt/e/01_NUS/02_5003/12_pi0.5"
        )
        if project_root not in sys.path:
            sys.path.insert(0, project_root)
        from laptop.ros_gateway.wire_protocol import (
            ControlMessageReader,
            estimate_monotonic_clock_offset,
            recv_control_message,
            send_control_message,
            validate_state_snapshot_envelope,
        )

        while not self.stop_event.is_set():
            try:
                with socket.create_connection(
                    (self.host, self.control_port), timeout=self.connect_timeout_s
                ) as sock:
                    sock.settimeout(0.1)
                    reader = ControlMessageReader()
                    send_control_message(sock, "hello", {
                        "client_name": self.client_name,
                        "client_role": "ros2_control_bridge_read_only_profile",
                        "hardware_commands_enabled": False,
                    })
                    self._set_connection(False, "waiting for gateway handshake")
                    self._session_id = ""
                    self._last_heartbeat_send_ns = None
                    self._heartbeat_send_times.clear()
                    self._clock_sync_estimates.clear()
                    with self._state_snapshot_lock:
                        self._last_state_sequence = 0
                        self._last_state_source_local_ns = 0
                        self._last_state_mapping_uncertainty_ns = UINT64_MAX
                        self._last_state_feedback_fresh = False
                        self._recent_state_sources.clear()
                    with self._state_metric_lock:
                        self._state_metrics["last_wire_sequence"] = 0
                    with self.clock_lock:
                        self._remote_to_local_mono_ns = None
                        self._clock_uncertainty_ns = UINT64_MAX
                        self._clock_map_updated_local_ns = None
                    with self.sequence_lock:
                        self._image_sequences.clear()
                    self._hardware_connected = False
                    self._feedback_fresh = False
                    self._camera_ready = False
                    while not self.stop_event.is_set():
                        now_mono = time.perf_counter_ns()
                        try:
                            control_request = self._control_request_queue.get_nowait()
                        except queue.Empty:
                            control_request = None
                        if control_request is not None:
                            pending = control_request["pending"]
                            if pending.event.is_set():
                                continue
                            if control_request["session_id"] != self._session_id:
                                pending.result = {
                                    "accepted": False,
                                    "decision": "REJECTED_GATEWAY_SESSION_CHANGED",
                                    "reason": "request was queued for a different gateway session",
                                }
                                pending.event.set()
                            else:
                                with self._state_metric_lock:
                                    delay = time.perf_counter_ns()-pending.queued_monotonic_ns
                                    self._state_metrics["control_sent_count"] = self._state_metrics.get("control_sent_count",0)+1
                                    self._state_metrics["control_queue_delay_max_ns"] = max(
                                        delay,self._state_metrics.get("control_queue_delay_max_ns",0))
                                send_control_message(sock, "control_request", {
                                    key: value for key, value in control_request.items()
                                    if key != "pending"
                                })
                        if self._session_id and (
                            self._last_heartbeat_send_ns is None
                            or now_mono - self._last_heartbeat_send_ns >= 500_000_000
                        ):
                            self._heartbeat_seq += 1
                            self._last_heartbeat_send_ns = now_mono
                            self._heartbeat_send_times[self._heartbeat_seq] = now_mono
                            for old_seq in list(self._heartbeat_send_times)[:-8]:
                                self._heartbeat_send_times.pop(old_seq, None)
                            send_control_message(sock, "heartbeat", {
                                "session_id": self._session_id,
                                "sequence": self._heartbeat_seq,
                                "client_monotonic_ns": now_mono,
                            })
                        try:
                            # Service requests share this I/O owner. Don't park
                            # them behind a 100 ms receive timeout between states.
                            if not reader.buffered_bytes and not select.select([sock],[],[],.01)[0]:
                                continue
                            kind, payload = reader.receive(sock)
                        except socket.timeout:
                            with self._state_metric_lock:
                                key = "partial_read_timeouts" if reader.buffered_bytes else "idle_read_timeouts"
                                self._state_metrics[key] += 1
                            continue
                        except TimeoutError:
                            continue
                        recv_mono = time.perf_counter_ns()
                        if kind == "hello_ack":
                            self._session_id = str(payload.get("session_id", ""))
                            if not self._session_id:
                                raise ValueError("gateway handshake omitted session_id")
                            self._update_runtime_owner(
                                status="connected", session_id=self._session_id,
                                gateway_profile=str(payload.get("profile", "unknown")),
                            )
                            self.get_logger().info(
                                f"gateway owner pid={os.getpid()} session_id={self._session_id} "
                                f"endpoint={self.host}:{self.control_port} "
                                f"ros_domain_id={os.environ.get('ROS_DOMAIN_ID', 'unknown')}"
                            )
                            self._fake_profile = str(payload.get("profile", "")) == "fake"
                            identity = payload.get("camera_identity", {})
                            self.camera_identity = identity if isinstance(identity, dict) else {}
                            self._set_connection(True, "connected; waiting for feedback")
                        elif kind == "state_snapshot":
                            self._publish_state(payload)
                        elif kind == "gateway_status":
                            with self._state_metric_lock:
                                self._gateway_timing = {
                                    key: value for key, value in payload.items()
                                    if key.startswith("tcp_state_")
                                }
                            self._connected = bool(payload.get("transport_connected", False))
                            self._hardware_connected = bool(payload.get("hardware_connected", False))
                            self._feedback_fresh = bool(payload.get("feedback_fresh", False))
                            self._camera_ready = bool(payload.get("camera_ready", False))
                            identity = payload.get("camera_identity")
                            if isinstance(identity, dict):
                                self.camera_identity = identity
                            if payload.get("fake", False):
                                self._fake_profile = True
                            self._last_status = str(payload.get("reason", "gateway status"))[:256]
                        elif kind == "heartbeat_ack":
                            ack_seq = payload.get("sequence")
                            sent_ns = self._heartbeat_send_times.pop(ack_seq, None) if type(ack_seq) is int else None
                            if (sent_ns is None
                                    or payload.get("session_id") != self._session_id
                                    or payload.get("client_monotonic_ns") != sent_ns):
                                self.get_logger().warning(
                                    "ignored unmatched clock-sync heartbeat acknowledgement"
                                )
                                continue
                            try:
                                estimate = estimate_monotonic_clock_offset(
                                    client_send_ns=sent_ns,
                                    server_receive_ns=payload.get("gateway_receive_monotonic_ns"),
                                    server_send_ns=payload.get("gateway_monotonic_ns"),
                                    client_receive_ns=recv_mono,
                                )
                            except (TypeError, ValueError) as exc:
                                self.get_logger().warning(f"rejected clock-sync sample: {exc}")
                                continue
                            self._clock_sync_estimates[ack_seq] = (
                                estimate.server_minus_client_offset_ns,
                                estimate.uncertainty_ns,
                                recv_mono,
                            )
                            for old_seq in sorted(self._clock_sync_estimates)[:-8]:
                                self._clock_sync_estimates.pop(old_seq, None)
                            send_control_message(sock, "clock_sync_complete", {
                                "session_id": self._session_id,
                                "sequence": ack_seq,
                                "client_monotonic_ns": sent_ns,
                                "gateway_receive_monotonic_ns": payload["gateway_receive_monotonic_ns"],
                                "gateway_send_monotonic_ns": payload["gateway_monotonic_ns"],
                                "client_receive_monotonic_ns": recv_mono,
                            })
                        elif kind == "clock_sync_ack":
                            ack_seq = payload.get("sequence")
                            candidate = (
                                self._clock_sync_estimates.pop(ack_seq, None)
                                if type(ack_seq) is int else None
                            )
                            if (candidate is None
                                    or payload.get("session_id") != self._session_id
                                    or payload.get("accepted") is not True):
                                self.get_logger().warning(
                                    "gateway rejected or could not match clock-sync sample: "
                                    f"{payload.get('reason', 'unknown reason')}"
                                )
                                continue
                            offset, uncertainty, _sample_received_ns = candidate
                            if (payload.get("server_minus_client_offset_ns") != offset
                                    or payload.get("uncertainty_ns") != uncertainty):
                                self.get_logger().warning(
                                    "gateway and ROS clock-sync estimates disagree"
                                )
                                continue
                            with self.clock_lock:
                                self._remote_to_local_mono_ns = -offset
                                self._clock_uncertainty_ns = uncertainty
                                self._clock_map_updated_local_ns = recv_mono
                        elif kind == "fault":
                            raise RuntimeError(f"gateway fault: {payload}")
                        elif kind == "command_receipt":
                            # No command is emitted by this bridge in P1.
                            self.get_logger().warning("unexpected command receipt on read-only session")
                        elif kind == "control_result":
                            request_id = payload.get("request_id")
                            with self._pending_control_lock:
                                pending = self._pending_control_calls.pop(request_id, None)
                            if (pending is None
                                    or payload.get("session_id") != self._session_id
                                    or pending.session_id != self._session_id):
                                self.get_logger().warning(
                                    "ignored unmatched or stale control RPC result"
                                )
                                continue
                            if not valid_control_receipt(payload):
                                pending.result = {
                                    "accepted": False,
                                    "decision": "REJECTED_INVALID_GATEWAY_CONTROL_RESULT",
                                    "reason": "gateway result violated the control receipt contract; delivery unconfirmed, do not retry",
                                }
                                pending.event.set()
                                raise RuntimeError(
                                    "gateway control RPC result violated the receipt contract"
                                )
                            pending.result = {
                                "accepted": payload["accepted"],
                                "decision": str(payload.get("decision", "REJECTED_UNSPECIFIED")),
                                "reason": str(payload.get("reason", "")),
                            }
                            pending.event.set()
                        else:
                            self.get_logger().warning(f"ignored gateway message type {kind!r}")
                    self._set_connection(False, "bridge shutting down")
                    self._update_runtime_owner(status="reconnecting", session_id=None)
            except Exception as exc:  # reconnect loop; no command retry exists here
                self._set_connection(False, f"gateway disconnected: {type(exc).__name__}: {exc}")
                self._update_runtime_owner(
                    status="reconnecting", session_id=None,
                    last_disconnect_reason=f"{type(exc).__name__}: {exc}"[:256],
                )
                self._fail_pending_control_calls(self._last_status)
                while True:
                    try:
                        stale = self._control_request_queue.get_nowait()
                    except queue.Empty:
                        break
                    pending = stale.get("pending")
                    if pending is not None and not pending.event.is_set():
                        pending.result = {
                            "accepted": False,
                            "decision": "REJECTED_GATEWAY_SESSION_LOST",
                            "reason": self._last_status,
                        }
                        pending.event.set()
                if not self.stop_event.is_set():
                    self.get_logger().warning(self._last_status)
                    self.stop_event.wait(1.0)

    def _publish_state(self, payload: dict) -> None:
        receive_ns = time.perf_counter_ns()
        with self._state_metric_lock:
            metric = self._state_metrics
            metric["received_count"] += 1
            metric["first_received_ns"] = metric["first_received_ns"] or receive_ns
            metric["last_received_ns"] = receive_ns
            sequence = payload.get("sequence")
            if type(sequence) is int:
                previous = metric["last_wire_sequence"]
                if previous and sequence > previous:
                    metric["wire_sequence_gaps"] += sequence - previous - 1
                metric["last_wire_sequence"] = sequence
        try:
            from laptop.ros_gateway.wire_protocol import validate_state_snapshot_envelope
            now_mono = time.perf_counter_ns()
            with self.clock_lock:
                offset = self._remote_to_local_mono_ns
                uncertainty = self._clock_uncertainty_ns
                map_updated = self._clock_map_updated_local_ns
            if map_updated is None or now_mono - map_updated > 1_500_000_000:
                raise ValueError("source clock mapping is missing or stale")
            sequence = validate_state_snapshot_envelope(
                payload,
                session_id=self._session_id,
                last_sequence=self._last_state_sequence,
                now_monotonic_ns=now_mono,
                remote_to_local_offset_ns=offset,
                max_age_ns=120_000_000,
                mapping_uncertainty_ns=uncertainty,
            )
            positions = payload["joint_position_rad"]
            enabled = payload["motor_enabled"]
            if not isinstance(positions, list) or len(positions) != 6:
                raise ValueError("joint_position_rad must contain six values")
            if not isinstance(enabled, list) or len(enabled) != 6:
                raise ValueError("motor_enabled must contain six values")
            if any(type(value) is not bool for value in enabled):
                raise ValueError("motor_enabled values must be booleans")
            positions = [float(value) for value in positions]
            if not all(math.isfinite(value) for value in positions):
                raise ValueError("joint position includes a non-finite value")
            source_ns = payload["host_monotonic_ns"]
            if type(source_ns) is not int or source_ns < 0:
                raise ValueError("invalid source monotonic timestamp")
            if type(offset) is not int or uncertainty < 0:
                raise ValueError("source-to-WSL monotonic clock mapping is unavailable")
            source_local_ns = source_ns + offset
            if source_local_ns <= 0 or source_local_ns > now_mono + uncertainty:
                raise ValueError("mapped state timestamp is outside the local monotonic clock")
            joint_pair_stamps = payload.get("joint_pair_monotonic_ns", [source_ns] * 3)
            motor_stamps = payload.get("motor_monotonic_ns", [source_ns] * 6)
            if len(joint_pair_stamps) != 3 or len(motor_stamps) != 6:
                raise ValueError("CAN timestamp arrays have invalid lengths")
            can_stamps = [
                payload.get("status_monotonic_ns", source_ns),
                *joint_pair_stamps,
                payload.get("gripper_monotonic_ns", source_ns),
                *motor_stamps,
            ]
            if any(type(value) is not int or value < 0 for value in can_stamps):
                raise ValueError("CAN timestamps must be non-negative integers")
            if type(payload.get("feedback_fresh", False)) is not bool:
                raise ValueError("feedback_fresh must be a boolean")
            if type(payload.get("hardware_connected", False)) is not bool:
                raise ValueError("hardware_connected must be a boolean")
            if type(payload.get("fake", False)) is not bool:
                raise ValueError("fake must be a boolean")
            stamp = self._mapped_stamp(source_ns)
            snapshot = StateSnapshot()
            snapshot.header.stamp = stamp
            snapshot.session_id = str(payload["session_id"])
            snapshot.sequence = int(payload["sequence"])
            snapshot.source_monotonic_ns = source_ns
            snapshot.source_clock_id = "windows_perf_counter_ns"
            snapshot.source_local_monotonic_ns = source_local_ns
            snapshot.source_local_clock_id = "wsl_perf_counter_ns"
            snapshot.source_clock_mapping_uncertainty_ns = uncertainty
            snapshot.feedback_fresh = bool(payload.get("feedback_fresh", False))
            snapshot.joint_skew_ns = int(payload.get("joint_skew_ns", 0))
            snapshot.status_monotonic_ns = int(payload.get("status_monotonic_ns", source_ns))
            snapshot.joint_pair_monotonic_ns = list(joint_pair_stamps)
            snapshot.gripper_monotonic_ns = int(payload.get("gripper_monotonic_ns", source_ns))
            snapshot.motor_monotonic_ns = list(motor_stamps)
            snapshot.joint_position_rad = positions
            snapshot.gripper_raw = int(payload["gripper_raw"])
            snapshot.gripper_nominal_m = float(payload["gripper_nominal_m"])
            snapshot.ctrl_mode_raw = int(payload["ctrl_mode_raw"])
            snapshot.arm_status_raw = int(payload["arm_status_raw"])
            snapshot.error_code = int(payload["error_code"])
            snapshot.motor_enabled = [bool(value) for value in enabled]
            snapshot.hardware_connected = bool(payload["hardware_connected"])
            snapshot.fake = bool(payload.get("fake", False))
            publish_start_ns = time.perf_counter_ns()
            self.state_pub.publish(snapshot)
            publish_end_ns = time.perf_counter_ns()
            with self._state_metric_lock:
                metric = self._state_metrics
                metric["published_count"] += 1
                metric["first_published_ns"] = metric["first_published_ns"] or publish_start_ns
                metric["last_published_ns"] = publish_start_ns
                metric["publish_ms"].append((publish_end_ns - publish_start_ns) / 1e6)
            self._hardware_connected = snapshot.hardware_connected
            self._feedback_fresh = snapshot.feedback_fresh
            self._fake_profile = snapshot.fake
            self._connected = True

            joints = JointState()
            joints.header.stamp = stamp
            joints.name = [f"joint{i}" for i in range(1, 7)]
            joints.position = positions
            self.joint_pub.publish(joints)
            with self._state_snapshot_lock:
                self._last_state_sequence = sequence
                self._last_state_source_local_ns = source_local_ns
                self._last_state_mapping_uncertainty_ns = uncertainty
                self._last_state_feedback_fresh = snapshot.feedback_fresh
                self._recent_state_sources.append((
                    snapshot.session_id, sequence, source_local_ns, uncertainty,
                    snapshot.feedback_fresh,
                ))
            self._set_connection(True, "feedback received" if not snapshot.fake else "FAKE_ONLY feedback")
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            with self._state_metric_lock:
                self._state_metrics["rejected_count"] += 1
                self._state_metrics["last_rejection"] = str(exc)[:256]
            with self._state_snapshot_lock:
                self._last_state_feedback_fresh = False
            self._feedback_fresh = False
            self._last_status = f"rejected malformed, stale, or duplicate state: {exc}"
            self.get_logger().warning(self._last_status)

    def _mapped_stamp(self, source_monotonic_ns: int):
        with self.clock_lock:
            offset = self._remote_to_local_mono_ns
        local_now = time.perf_counter_ns()
        ros_now = self.get_clock().now().nanoseconds
        if offset is None:
            return self.get_clock().now().to_msg()
        source_local_ns = source_monotonic_ns + offset
        stamp_ns = ros_now + (source_local_ns - local_now)
        stamp_ns = max(0, stamp_ns)
        from builtin_interfaces.msg import Time
        stamp = Time()
        stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        return stamp

    def _image_worker(self, role: str, port: int) -> None:
        project_root = os.environ.get(
            "PI05_PROJECT_ROOT", "/mnt/e/01_NUS/02_5003/12_pi0.5"
        )
        if project_root not in sys.path:
            sys.path.insert(0, project_root)
        from laptop.ros_gateway.wire_protocol import recv_image_frame, validate_image_source_time

        while not self.stop_event.is_set():
            try:
                with socket.create_connection((self.host, port), timeout=self.connect_timeout_s) as sock:
                    sock.settimeout(2.0)
                    while not self.stop_event.is_set():
                        frame = recv_image_frame(sock)
                        with self.clock_lock:
                            offset = self._remote_to_local_mono_ns
                            uncertainty = self._clock_uncertainty_ns
                            map_updated = self._clock_map_updated_local_ns
                        now_mono = time.perf_counter_ns()
                        if map_updated is None or now_mono - map_updated > 1_500_000_000:
                            self._camera_ready = False
                            continue
                        with self.sequence_lock:
                            previous = self._image_sequences.get(role, 0)
                        try:
                            validate_image_source_time(
                                frame,
                                expected_role=role,
                                last_sequence=previous,
                                now_monotonic_ns=now_mono,
                                remote_to_local_offset_ns=offset,
                                max_age_ns=250_000_000,
                                mapping_uncertainty_ns=uncertainty,
                            )
                        except ValueError as exc:
                            self._camera_ready = False
                            self._last_status = f"rejected stale or duplicate {role} image: {exc}"
                            continue
                        # Consume and validate every raw source frame, but only
                        # publish the newest eligible frame at the selected
                        # ROS cadence (30 Hz in the camera deployment profile).
                        if not self.image_rate_gates[role].accept(now_mono):
                            continue
                        dropped = max(0, frame.sequence - previous - 1) if previous else 0
                        stamp = self._mapped_stamp(frame.host_monotonic_ns)
                        image = Image()
                        image.header = Header()
                        image.header.stamp = stamp
                        image.header.frame_id = role
                        image.height = frame.height
                        image.width = frame.width
                        image.encoding = "bgr8"
                        image.is_bigendian = 0
                        image.step = frame.stride
                        # ROSIDL stores uint8 sequences as array('B'). Building a
                        # Python int list for every 640x480 BGR frame adds a
                        # large per-frame conversion and serialization cost.
                        image.data = array("B", frame.pixels)
                        publish_start_ns = time.perf_counter_ns()
                        self.image_pubs[role].publish(image)
                        publish_end_ns = time.perf_counter_ns()
                        with self._image_metric_lock:
                            metric = self._image_publish_metrics[role]
                            metric["count"] += 1
                            metric["first_ns"] = metric["first_ns"] or publish_start_ns
                            metric["last_ns"] = publish_end_ns
                            metric["last_sequence"] = frame.sequence
                            metric["publish_ms"].append(
                                (publish_end_ns - publish_start_ns) / 1_000_000
                            )

                        metadata = CameraFrameMetadata()
                        metadata.header = image.header
                        metadata.role = role
                        metadata.session_id = self._session_id
                        role_identity = self.camera_identity.get(role, {})
                        metadata.device_name = str(role_identity.get("device_name", "source-gateway"))
                        metadata.sequence = frame.sequence
                        metadata.source_monotonic_ns = frame.host_monotonic_ns
                        metadata.source_clock_id = "windows_perf_counter_ns"
                        if type(offset) is not int:
                            self._camera_ready = False
                            continue
                        source_local_ns = frame.host_monotonic_ns + offset
                        if source_local_ns <= 0 or source_local_ns > now_mono + uncertainty:
                            self._camera_ready = False
                            continue
                        metadata.source_local_monotonic_ns = source_local_ns
                        metadata.source_local_clock_id = "wsl_perf_counter_ns"
                        with self.clock_lock:
                            metadata.clock_mapping_uncertainty_ns = self._clock_uncertainty_ns
                        metadata.dropped_frames = dropped
                        metadata.encoding = frame.encoding
                        metadata.width = frame.width
                        metadata.height = frame.height
                        self.metadata_pub.publish(metadata)
                        with self.sequence_lock:
                            self._image_sequences[role] = frame.sequence
                        self._camera_ready = True
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.get_logger().warning(f"{role} image disconnected: {type(exc).__name__}: {exc}")
                    self.stop_event.wait(1.0)

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "pi05/windows_gateway"
        status.hardware_id = self.host
        if not self._connected:
            status.level = DiagnosticStatus.ERROR
        elif self._fake_profile:
            status.level = DiagnosticStatus.WARN
        elif not self._hardware_connected:
            status.level = DiagnosticStatus.ERROR
        elif not self._feedback_fresh:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = self._last_status
        status.values = [
            KeyValue(key="session_id", value=self._session_id),
            KeyValue(key="hardware_commands_enabled", value="false"),
            KeyValue(key="connected", value=str(self._connected).lower()),
            KeyValue(key="hardware_connected", value=str(self._hardware_connected).lower()),
            KeyValue(key="feedback_fresh", value=str(self._feedback_fresh).lower()),
            KeyValue(key="camera_ready", value=str(self._camera_ready).lower()),
            KeyValue(key="fake_profile", value=str(self._fake_profile).lower()),
        ]
        now_ns = time.perf_counter_ns()
        with self._state_metric_lock:
            metric = self._state_metrics
            timing = {
                **{f"state_{key}": metric[key] for key in (
                    "received_count", "published_count", "rejected_count", "wire_sequence_gaps",
                    "last_wire_sequence", "partial_read_timeouts", "idle_read_timeouts", "last_rejection",
                )},
                **self._gateway_timing,
            }
            for stage in ("received", "published"):
                first = metric[f"first_{stage}_ns"]
                last = metric[f"last_{stage}_ns"]
                count = metric[f"{stage}_count"]
                timing[f"state_{stage}_rate_hz"] = (
                    (count - 1) * 1e9 / (last - first) if first and last and last > first else 0.0
                )
            latencies = sorted(metric["publish_ms"])
            timing["state_publish_call_p95_ms"] = (
                latencies[int((len(latencies)-1)*.95)] if latencies else 0.0
            )
            status.values.extend(KeyValue(key=key, value=str(value)) for key, value in timing.items())
        with self._image_metric_lock:
            for role, metric in self._image_publish_metrics.items():
                count = metric["count"]
                first_ns = metric["first_ns"]
                elapsed_s = (
                    (now_ns - first_ns) / 1_000_000_000 if first_ns else 0.0
                )
                latencies = sorted(metric["publish_ms"])
                p95_index = min(
                    len(latencies) - 1,
                    int((len(latencies) - 1) * 0.95),
                ) if latencies else None
                values = {
                    f"image_{role}_published_count": str(count),
                    f"image_{role}_publish_call_rate_hz": (
                        f"{count / elapsed_s:.3f}" if elapsed_s > 0 else "0"
                    ),
                    f"image_{role}_publish_call_p95_ms": (
                        f"{latencies[p95_index]:.3f}" if p95_index is not None else "0"
                    ),
                    f"image_{role}_last_sequence": str(metric["last_sequence"]),
                }
                status.values.extend(
                    KeyValue(key=key, value=value) for key, value in values.items()
                )
        array.status = [status]
        self.status_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReadOnlyBridge()
    node.start_workers()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        # Launch shutdown can invalidate the context before spin observes the
        # executor shutdown event. Suppress only its known RCLError after the
        # context is already invalid; propagate all other failures.
        if rclpy.ok() or type(exc).__name__ != "RCLError":
            raise
    finally:
        node.stop_workers()
        node._fail_pending_control_calls("ROS bridge is shutting down")
        node.close_runtime_owner()
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
