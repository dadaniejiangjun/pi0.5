"""Read-only ROS subscriptions and bounded length-prefixed MessagePack stdio."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
import queue
import struct
import sys
import threading
import time
from typing import Any, BinaryIO, Callable, Mapping

from .contracts import MAX_MESSAGE_BYTES, image_header_stamp_ns, wire_envelope_sample
from .pairing import BoundedObservationAssembler


PROTOCOL = "piper_live_stdio_v1"
LIFECYCLE_TOPIC = "/gamepad/recording_lifecycle"
TARGET_EVENTS_TOPIC = "/gamepad/target_events"
INPUT_STATUS_TOPIC = "/gamepad/input_status"
RB_BUTTON_INDEX = 10
TAIL_NS = 1_000_000_000
MAX_TAIL_NS = 10_000_000_000
OUTPUT_QUEUE_SIZE = 8
OUTPUT_CONTROL_QUEUE_SIZE = 128
OUTPUT_DIAGNOSTIC_QUEUE_SIZE = 256
OUTPUT_CONTROL_BACKPRESSURE_S = 0.05
INPUT_QUEUE_SIZE = 32
_STDIN_EOF = object()
TOPIC_TYPES = {
    "/piper/state_snapshot": "pi05_piper_interfaces/msg/StateSnapshot",
    "/piper/cameras/frame_metadata": "pi05_piper_interfaces/msg/CameraFrameMetadata",
    "/piper/cameras/base/image_raw": "sensor_msgs/msg/Image",
    "/piper/cameras/wrist/image_raw": "sensor_msgs/msg/Image",
    "/gamepad/command_send_events": "std_msgs/msg/String",
    "/piper/command_receipt": "pi05_piper_interfaces/msg/CommandReceipt",
    "/joy": "sensor_msgs/msg/Joy",
    TARGET_EVENTS_TOPIC: "std_msgs/msg/String",
    INPUT_STATUS_TOPIC: "std_msgs/msg/String",
    LIFECYCLE_TOPIC: "std_msgs/msg/String",
}
REQUIRED_INTERFACE_FIELDS = {
    "StateSnapshot": {
        "header", "session_id", "sequence", "source_monotonic_ns",
        "source_clock_id", "source_local_monotonic_ns",
        "source_local_clock_id", "source_clock_mapping_uncertainty_ns",
        "feedback_fresh", "joint_position_rad", "gripper_nominal_m",
        "hardware_connected", "fake",
    },
    "CameraFrameMetadata": {
        "header", "role", "session_id", "device_name", "sequence",
        "source_monotonic_ns", "source_clock_id", "source_local_monotonic_ns",
        "source_local_clock_id", "clock_mapping_uncertainty_ns", "encoding",
        "width", "height",
    },
    "CommandReceipt": {
        "header", "request_id", "observation_id", "decision", "reason",
        "sequence", "source_monotonic_ns", "adapter_accepted", "can_tx_calls",
    },
}


class ProtocolError(ValueError):
    """Raised for malformed or oversized stdio protocol frames."""


class LengthPrefixedMsgpackIO:
    """Read/write MessagePack maps prefixed by a four-byte big-endian length."""

    def __init__(
        self,
        reader: BinaryIO,
        writer: BinaryIO,
        *,
        max_message_bytes: int = MAX_MESSAGE_BYTES,
        pack: Callable[[Any], bytes] | None = None,
        unpack: Callable[[bytes], Any] | None = None,
    ) -> None:
        if type(max_message_bytes) is not int or not 0 < max_message_bytes <= MAX_MESSAGE_BYTES:
            raise ValueError("max_message_bytes must be within the 64 MiB protocol limit")
        self.reader = reader
        self.writer = writer
        self.max_message_bytes = max_message_bytes
        self._pack = pack or _msgpack_pack
        self._unpack = unpack or _msgpack_unpack
        self._write_lock = threading.Lock()

    def receive(self) -> dict[str, Any] | None:
        header = _read_exact(self.reader, 4, allow_eof=True)
        if header is None:
            return None
        length = struct.unpack(">I", header)[0]
        if length <= 0 or length > self.max_message_bytes:
            raise ProtocolError(f"incoming frame length {length} is outside the allowed range")
        payload = _read_exact(self.reader, length)
        assert payload is not None
        value = self._unpack(payload)
        if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
            raise ProtocolError("incoming MessagePack frame must be a string-keyed map")
        return value

    def send(self, value: Mapping[str, Any]) -> None:
        if not isinstance(value, Mapping):
            raise ProtocolError("outgoing frame must be a mapping")
        payload = self._pack(dict(value))
        if not isinstance(payload, bytes):
            raise ProtocolError("MessagePack encoder did not return bytes")
        if not payload or len(payload) > self.max_message_bytes:
            raise ProtocolError(f"outgoing frame length {len(payload)} exceeds the message limit")
        frame = struct.pack(">I", len(payload)) + payload
        with self._write_lock:
            self.writer.write(frame)
            self.writer.flush()


@dataclass(frozen=True)
class _OutboundEntry:
    sequence: int
    enqueued_at: float
    payload: dict[str, Any]


class OutboundQueueSaturated(RuntimeError):
    """A bounded critical-message queue stayed full through its wait budget."""


class BoundedOutboundQueue:
    """Bounded, ordered sample/control queues with coalesced diagnostics.

    Control messages can overtake queued samples so a large image write does
    not make lifecycle events wait behind the sample backlog. STOP_TAIL_COMPLETE
    remains a barrier and is sent only after older samples have drained.
    """

    def __init__(self, *, sample_capacity: int = OUTPUT_QUEUE_SIZE,
                 control_capacity: int = OUTPUT_CONTROL_QUEUE_SIZE,
                 diagnostic_capacity: int = OUTPUT_DIAGNOSTIC_QUEUE_SIZE,
                 control_backpressure_s: float = OUTPUT_CONTROL_BACKPRESSURE_S):
        if type(sample_capacity) is not int or sample_capacity <= 0:
            raise ValueError("sample_capacity must be a positive integer")
        if type(control_capacity) is not int or control_capacity <= 0:
            raise ValueError("control_capacity must be a positive integer")
        if type(diagnostic_capacity) is not int or diagnostic_capacity <= 0:
            raise ValueError("diagnostic_capacity must be a positive integer")
        if control_backpressure_s < 0:
            raise ValueError("control_backpressure_s must be non-negative")
        self.sample_capacity = sample_capacity
        self.control_capacity = control_capacity
        self.diagnostic_capacity = diagnostic_capacity
        self.control_backpressure_s = float(control_backpressure_s)
        self._condition = threading.Condition()
        self._samples: deque[_OutboundEntry] = deque()
        self._controls: deque[_OutboundEntry] = deque()
        self._input_statuses: deque[_OutboundEntry] = deque()
        self._diagnostics: dict[str, _OutboundEntry] = {}
        self._overflow_control: _OutboundEntry | None = None
        self._fatal_message: _OutboundEntry | None = None
        self._next_sequence = 0
        self._closed = False
        self._failure: str | None = None
        self._counters = {
            "outbound_events_dropped": 0,
            "outbound_samples_dropped": 0,
            "outbound_input_status_dropped": 0,
            "outbound_control_events_dropped": 0,
            "outbound_diagnostic_events_dropped": 0,
            "outbound_diagnostic_events_coalesced": 0,
            "outbound_control_queue_saturations": 0,
        }
        self._control_queue_peak_depth = 0
        self._sample_queue_peak_depth = 0
        self._input_status_queue_peak_depth = 0
        self._control_queue_max_wait_ms = 0.0
        self._sample_queue_max_wait_ms = 0.0
        self._diagnostic_queue_max_wait_ms = 0.0

    @staticmethod
    def _diagnostic_key(payload: Mapping[str, Any]) -> str | None:
        if payload.get("type") == "status" and payload.get("status") == "PAIRING_STATUS":
            pairing = payload.get("pairing")
            if not isinstance(pairing, Mapping) or pairing.get("episode_tail_complete") is not True:
                return "pairing_status"
        return None

    @staticmethod
    def _is_input_status_frame(payload: Mapping[str, Any]) -> bool:
        detail = payload.get("payload")
        return (payload.get("type") == "event" and payload.get("event") == "INPUT_STATUS"
                and isinstance(detail, Mapping) and detail.get("event") == "input_frame"
                and detail.get("connected") is True)

    def _new_entry(self, payload: Mapping[str, Any]) -> _OutboundEntry:
        entry = _OutboundEntry(self._next_sequence, time.monotonic(), dict(payload))
        self._next_sequence += 1
        return entry

    def _record_drop_locked(self, payload: Mapping[str, Any]) -> None:
        self._counters["outbound_events_dropped"] += 1
        if payload.get("type") == "sample":
            self._counters["outbound_samples_dropped"] += 1
        elif self._is_input_status_frame(payload):
            self._counters["outbound_input_status_dropped"] += 1
        elif self._diagnostic_key(payload) is not None:
            self._counters["outbound_diagnostic_events_dropped"] += 1
        else:
            self._counters["outbound_control_events_dropped"] += 1

    def put(self, payload: Mapping[str, Any]) -> str:
        if not isinstance(payload, Mapping):
            raise TypeError("outbound payload must be a mapping")
        message = dict(payload)
        is_sample = message.get("type") == "sample"
        is_input_status = self._is_input_status_frame(message)
        diagnostic_key = self._diagnostic_key(message)
        deadline = time.monotonic() + self.control_backpressure_s
        with self._condition:
            if self._closed:
                raise RuntimeError("outbound queue is closed")
            if self._failure is not None:
                self._record_drop_locked(message)
                raise OutboundQueueSaturated(self._failure)
            if is_sample:
                if len(self._samples) >= self.sample_capacity:
                    self._record_drop_locked(message)
                    return "sample_dropped"
                self._samples.append(self._new_entry(message))
                self._sample_queue_peak_depth = max(
                    self._sample_queue_peak_depth, len(self._samples))
                self._condition.notify_all()
                return "accepted"
            if is_input_status:
                if len(self._input_statuses) >= self.diagnostic_capacity:
                    self._record_drop_locked(message)
                    return "diagnostic_dropped"
                self._input_statuses.append(self._new_entry(message))
                self._input_status_queue_peak_depth = max(
                    self._input_status_queue_peak_depth, len(self._input_statuses))
                self._condition.notify_all()
                return "accepted"
            if diagnostic_key is not None:
                entry = self._new_entry(message)
                if diagnostic_key in self._diagnostics:
                    self._counters["outbound_diagnostic_events_coalesced"] += 1
                    self._diagnostics[diagnostic_key] = entry
                    self._condition.notify_all()
                    return "diagnostic_coalesced"
                self._diagnostics[diagnostic_key] = entry
                self._condition.notify_all()
                return "accepted"

            while len(self._controls) >= self.control_capacity:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    entry = self._new_entry(message)
                    self._counters["outbound_control_queue_saturations"] += 1
                    self._failure = (
                        "critical outbound queue saturated after bounded backpressure; "
                        f"event={message.get('event', message.get('status', message.get('type', 'UNKNOWN')))}"
                    )
                    if self._overflow_control is None:
                        self._overflow_control = entry
                    else:
                        self._record_drop_locked(message)
                    self._control_queue_peak_depth = max(
                        self._control_queue_peak_depth,
                        len(self._controls) + int(self._overflow_control is not None)
                        + int(self._fatal_message is not None),
                    )
                    self._condition.notify_all()
                    raise OutboundQueueSaturated(self._failure)
                self._condition.wait(remaining)
                if self._closed:
                    raise RuntimeError("outbound queue is closed")
                if self._failure is not None:
                    raise OutboundQueueSaturated(self._failure)
            self._controls.append(self._new_entry(message))
            self._control_queue_peak_depth = max(
                self._control_queue_peak_depth, len(self._controls))
            self._condition.notify_all()
            return "accepted"

    def post_fatal(self, payload: Mapping[str, Any]) -> None:
        """Store one bounded terminal error outside normal queue capacity."""
        with self._condition:
            if self._fatal_message is None:
                self._fatal_message = self._new_entry(payload)
                self._control_queue_peak_depth = max(
                    self._control_queue_peak_depth,
                    len(self._controls) + int(self._overflow_control is not None) + 1,
                )
            self._condition.notify_all()

    @staticmethod
    def _is_tail_barrier(entry: _OutboundEntry) -> bool:
        payload = entry.payload
        if payload.get("type") == "error":
            return True
        if payload.get("type") != "status":
            return False
        if payload.get("status") == "STOP_TAIL_COMPLETE":
            return True
        pairing = payload.get("pairing")
        return (payload.get("status") == "PAIRING_STATUS"
                and isinstance(pairing, Mapping)
                and pairing.get("episode_tail_complete") is True)

    def _critical_head_locked(self) -> tuple[str, _OutboundEntry] | None:
        candidates: list[tuple[str, _OutboundEntry]] = []
        if self._controls:
            candidates.append(("control", self._controls[0]))
        if self._overflow_control is not None:
            candidates.append(("overflow", self._overflow_control))
        if self._fatal_message is not None:
            candidates.append(("fatal", self._fatal_message))
        return min(candidates, key=lambda item: item[1].sequence) if candidates else None

    def _pop_critical_locked(self, source: str) -> _OutboundEntry:
        if source == "control":
            entry = self._controls.popleft()
            self._condition.notify_all()
            return entry
        if source == "overflow":
            assert self._overflow_control is not None
            entry, self._overflow_control = self._overflow_control, None
            return entry
        assert source == "fatal" and self._fatal_message is not None
        entry, self._fatal_message = self._fatal_message, None
        return entry

    def _pop_sample_locked(self) -> _OutboundEntry:
        entry = self._samples.popleft()
        self._condition.notify_all()
        return entry

    def _pop_input_status_locked(self) -> _OutboundEntry:
        entry = self._input_statuses.popleft()
        self._condition.notify_all()
        return entry

    def get(self, timeout: float | None = None) -> dict[str, Any] | None:
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while True:
                critical = self._critical_head_locked()
                entry: _OutboundEntry | None = None
                source = ""
                if critical is not None:
                    source, candidate = critical
                    sample = self._samples[0] if self._samples else None
                    input_status = self._input_statuses[0] if self._input_statuses else None
                    diagnostic = (
                        min(self._diagnostics.items(), key=lambda item: item[1].sequence)
                        if self._diagnostics else None
                    )
                    pending_before_barrier = [
                        ("sample", sample) for sample in (sample,)
                        if sample is not None and sample.sequence < candidate.sequence
                    ] + [
                        ("input_status", input_status) for input_status in (input_status,)
                        if input_status is not None and input_status.sequence < candidate.sequence
                    ]
                    if (diagnostic is not None
                            and diagnostic[1].sequence < candidate.sequence):
                        pending_before_barrier.append(
                            (f"diagnostic:{diagnostic[0]}", diagnostic[1]))
                    if self._is_tail_barrier(candidate) and pending_before_barrier:
                        source, _ = min(pending_before_barrier, key=lambda item: item[1].sequence)
                        if source == "sample":
                            entry = self._pop_sample_locked()
                        elif source == "input_status":
                            entry = self._pop_input_status_locked()
                        else:
                            key = source.split(":", 1)[1]
                            entry = self._diagnostics.pop(key)
                    else:
                        entry = self._pop_critical_locked(source)
                else:
                    options: list[tuple[str, _OutboundEntry]] = []
                    if self._samples:
                        options.append(("sample", self._samples[0]))
                    if self._input_statuses:
                        options.append(("input_status", self._input_statuses[0]))
                    if self._diagnostics:
                        key, diagnostic = min(
                            self._diagnostics.items(), key=lambda item: item[1].sequence)
                        options.append((f"diagnostic:{key}", diagnostic))
                    if options:
                        source, candidate = min(options, key=lambda item: item[1].sequence)
                        if source == "sample":
                            entry = self._pop_sample_locked()
                        elif source == "input_status":
                            entry = self._pop_input_status_locked()
                        else:
                            key = source.split(":", 1)[1]
                            entry = self._diagnostics.pop(key)
                if entry is not None:
                    waited_ms = max(0.0, (time.monotonic() - entry.enqueued_at) * 1000.0)
                    if source == "sample":
                        self._sample_queue_max_wait_ms = max(
                            self._sample_queue_max_wait_ms, waited_ms)
                    elif source == "input_status" or source.startswith("diagnostic:"):
                        self._diagnostic_queue_max_wait_ms = max(
                            self._diagnostic_queue_max_wait_ms, waited_ms)
                    else:
                        self._control_queue_max_wait_ms = max(
                            self._control_queue_max_wait_ms, waited_ms)
                    return entry.payload
                if self._closed:
                    return None
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return None
                self._condition.wait(remaining)

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def metrics(self) -> dict[str, int | float]:
        with self._condition:
            return {
                **self._counters,
                "outbound_control_queue_depth": (
                    len(self._controls) + int(self._overflow_control is not None)
                    + int(self._fatal_message is not None)),
                "outbound_sample_queue_depth": len(self._samples),
                "outbound_input_status_queue_depth": len(self._input_statuses),
                "outbound_diagnostic_queue_depth": len(self._input_statuses) + len(self._diagnostics),
                "outbound_control_queue_peak_depth": self._control_queue_peak_depth,
                "outbound_sample_queue_peak_depth": self._sample_queue_peak_depth,
                "outbound_input_status_queue_peak_depth": self._input_status_queue_peak_depth,
                "outbound_control_max_queue_wait_ms": round(
                    self._control_queue_max_wait_ms, 3),
                "outbound_sample_max_queue_wait_ms": round(
                    self._sample_queue_max_wait_ms, 3),
                "outbound_diagnostic_max_queue_wait_ms": round(
                    self._diagnostic_queue_max_wait_ms, 3),
            }


class ReadOnlyRosDatasetAdapter:
    """ROS topic adapter injected with a node, types, QoS, and output sink."""

    def __init__(
        self,
        node: Any,
        *,
        message_types: Mapping[str, Any],
        sensor_qos: Any,
        emit: Callable[[dict[str, Any]], None],
        assembler: BoundedObservationAssembler | None = None,
        arrival_clock: Callable[[], int] = time.monotonic_ns,
        event_qos: int = 100,
        small_event_qos: int = 10,
    ) -> None:
        self.node = node
        self.types = dict(message_types)
        self.sensor_qos = sensor_qos
        self.event_qos = event_qos
        self.small_event_qos = small_event_qos
        self.emit = emit
        self.assembler = assembler or BoundedObservationAssembler()
        self.arrival_clock = arrival_clock
        self.lifecycle_publisher: Any = None
        self.subscriptions: list[Any] = []
        self._topics_by_label = {
            "state_snapshot": "/piper/state_snapshot",
            "camera_metadata": "/piper/cameras/frame_metadata",
            "base_image": "/piper/cameras/base/image_raw",
            "wrist_image": "/piper/cameras/wrist/image_raw",
            "command_send_events": "/gamepad/command_send_events",
            "command_receipt": "/piper/command_receipt",
            "joy": "/joy",
            "operator_events": TARGET_EVENTS_TOPIC,
            "input_status": INPUT_STATUS_TOPIC,
        }
        self._callback_counts: dict[str, int] = {
            "state_snapshot": 0,
            "camera_metadata": 0,
            "base_image": 0,
            "wrist_image": 0,
            "command_send_events": 0,
            "command_receipt": 0,
            "joy": 0,
            "operator_events": 0,
            "input_status": 0,
        }
        self.rb_pressed = False
        self.latest_state_session_id: str | None = None
        self.latest_state_sequence: int | None = None
        self.camera_identities: dict[str, dict[str, Any]] = {
            role: {"status": "UNKNOWN", "session_id": None, "device_name": None}
            for role in ("base", "wrist")
        }
        self._reported_tail_complete = False
        self._last_status_ns = 0
        self._last_status_fingerprint: tuple[Any, ...] | None = None
        self._command_queue: queue.Queue[Any] | None = None
        self._stdin_eof = False
        self._closed = False
        self._last_stopped_run_id: str | None = None
        self._counters: dict[str, int] = {
            "outbound_events_dropped": 0,
            "bad_event_json": 0,
        }

    def attach(self, *, timer_period_s: float = 0.01) -> None:
        """Create only the existing lifecycle publisher and read subscriptions."""
        required = (
            "StateSnapshot", "CameraFrameMetadata", "CommandReceipt",
            "Image", "Joy", "String",
        )
        missing = [name for name in required if name not in self.types]
        if missing:
            raise RuntimeError(f"message_types are missing: {missing}")
        String = self.types["String"]
        self.lifecycle_publisher = self.node.create_publisher(
            String, LIFECYCLE_TOPIC, self.small_event_qos
        )
        self.subscriptions = [
            self.node.create_subscription(
                self.types["StateSnapshot"], "/piper/state_snapshot",
                self.on_state, self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["CameraFrameMetadata"], "/piper/cameras/frame_metadata",
                self.on_metadata, self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["Image"], "/piper/cameras/base/image_raw",
                lambda message: self.on_image("base", message), self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["Image"], "/piper/cameras/wrist/image_raw",
                lambda message: self.on_image("wrist", message), self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["String"], "/gamepad/command_send_events",
                self.on_send_event, self.event_qos,
            ),
            self.node.create_subscription(
                self.types["CommandReceipt"], "/piper/command_receipt",
                self.on_receipt, self.event_qos,
            ),
            self.node.create_subscription(
                self.types["Joy"], "/joy", self.on_joy, self.sensor_qos,
            ),
            self.node.create_subscription(
                self.types["String"], TARGET_EVENTS_TOPIC,
                lambda message: self.on_status_event("OPERATOR_EVENT", message),
                self.small_event_qos,
            ),
            self.node.create_subscription(
                self.types["String"], INPUT_STATUS_TOPIC,
                lambda message: self.on_status_event("INPUT_STATUS", message),
                self.small_event_qos,
            ),
        ]
        self.node.create_timer(timer_period_s, self.on_timer)

    def ready_message(self, *, ros_domain_id: int | None) -> dict[str, Any]:
        abi = inspect_interface_abi(self.types)
        return {
            "type": "status",
            "protocol": PROTOCOL,
            "status": "READY" if abi["valid"] else "INTERFACE_ABI_INVALID",
            "max_message_bytes": MAX_MESSAGE_BYTES,
            "ros_domain_id": ros_domain_id if ros_domain_id is not None else "UNKNOWN",
            "topics": dict(TOPIC_TYPES),
            "interface_abi": abi,
            "camera_identities": {
                role: dict(value) for role, value in self.camera_identities.items()
            },
            "ros_receive_diagnostics": self.ros_receive_diagnostics(),
            "pairing": self.assembler.status(),
            "safety_boundary": "read-only subscriptions; lifecycle String publisher only; no robot service or CAN API",
        }

    def ros_receive_diagnostics(self) -> dict[str, Any]:
        """Report process-local subscription matches and callback totals."""
        matched_publishers: dict[str, int | None] = {}
        get_count = getattr(self.node, "count_publishers", None)
        for label, topic in self._topics_by_label.items():
            try:
                matched_publishers[label] = int(get_count(topic)) if callable(get_count) else None
            except Exception:
                matched_publishers[label] = None
        return {
            "matched_publishers": matched_publishers,
            "callback_counts": dict(self._callback_counts),
        }

    def on_state(self, message: Any) -> None:
        self._callback_counts["state_snapshot"] += 1
        arrival_ns = self.arrival_clock()
        state = ros_state_to_dict(message)
        self.latest_state_session_id = state.get("session_id")
        self.latest_state_sequence = state.get("sequence")
        self.assembler.add_state(state, arrival_ns)

    def on_metadata(self, message: Any) -> None:
        self._callback_counts["camera_metadata"] += 1
        arrival_ns = self.arrival_clock()
        metadata = ros_metadata_to_dict(message)
        role = metadata.get("role")
        if role in self.camera_identities:
            previous = self.camera_identities[role]
            current = {
                "status": "OBSERVED",
                "session_id": metadata.get("session_id"),
                "device_name": metadata.get("device_name"),
            }
            if previous != current:
                self.camera_identities[role] = current
                self.emit({
                    "type": "status",
                    "status": "CAMERA_IDENTITY_OBSERVED",
                    "role": role,
                    "session_id": current["session_id"],
                    "device_name": current["device_name"],
                    "sequence": metadata.get("sequence"),
                    "source_local_clock_id": metadata.get("source_local_clock_id"),
                })
        self.assembler.add_metadata(metadata, arrival_ns)

    def on_image(self, role: str, message: Any) -> None:
        label = f"{role}_image"
        if label in self._callback_counts:
            self._callback_counts[label] += 1
        self.assembler.add_image(
            role, message, image_header_stamp_ns(message), self.arrival_clock()
        )

    def on_send_event(self, message: Any) -> None:
        self._callback_counts["command_send_events"] += 1
        payload = _decode_json_message(message, self._counters)
        if payload is not None:
            self.assembler.add_send_event(payload, self.arrival_clock())

    def on_receipt(self, message: Any) -> None:
        self._callback_counts["command_receipt"] += 1
        self.assembler.add_receipt(ros_receipt_to_dict(message), self.arrival_clock())

    def on_joy(self, message: Any) -> None:
        self._callback_counts["joy"] += 1
        buttons = tuple(getattr(message, "buttons", ()))
        pressed = len(buttons) > RB_BUTTON_INDEX and bool(buttons[RB_BUTTON_INDEX])
        rising = pressed and not self.rb_pressed
        self.rb_pressed = pressed
        if rising:
            self.emit({
                "type": "event",
                "event": "RB_RISING_EDGE",
                "run_id": self.assembler.status().get("run_id"),
                "source_session_id": self.latest_state_session_id or "UNKNOWN",
                "source_state_sequence": self.latest_state_sequence,
                "source_monotonic_ns": self.arrival_clock(),
                "button_index": RB_BUTTON_INDEX,
                "can_tx": False,
            })

    def on_status_event(self, name: str, message: Any) -> None:
        label = "operator_events" if name == "OPERATOR_EVENT" else "input_status"
        if label in self._callback_counts:
            self._callback_counts[label] += 1
        text = str(getattr(message, "data", ""))
        try:
            payload: Any = json.loads(text)
        except (TypeError, json.JSONDecodeError):
            self._counters["bad_event_json"] += 1
            payload = {"raw": text, "parse_error": True}
        self.emit({
            "type": "event",
            "event": name,
            "run_id": (
                payload.get("run_id") if isinstance(payload, Mapping)
                and isinstance(payload.get("run_id"), str)
                else self.assembler.status().get("run_id") or self._last_stopped_run_id
            ),
            "arrival_monotonic_ns": self.arrival_clock(),
            "payload": payload,
        })

    def enqueue_command(self, command: Mapping[str, Any]) -> None:
        if self._command_queue is None:
            self._command_queue = queue.Queue(maxsize=INPUT_QUEUE_SIZE)
        try:
            self._command_queue.put_nowait(dict(command))
        except queue.Full:
            self.emit({"type": "error", "error": "command_queue_full"})

    def mark_stdin_eof(self) -> None:
        self._stdin_eof = True

    def on_timer(self) -> None:
        now_ns = self.arrival_clock()
        self._process_commands(now_ns)
        for sample in self.assembler.poll(now_ns):
            try:
                self.emit(wire_envelope_sample(sample))
            except Exception as exc:
                self._counters["sample_encode_errors"] = self._counters.get("sample_encode_errors", 0) + 1
                self.emit({"type": "error", "error": "sample_encode_failed", "detail": str(exc)})
        status = self.assembler.status()
        tail_complete = status.get("episode_tail_complete") and not self._reported_tail_complete
        if tail_complete:
            self._reported_tail_complete = True
            # Send the final cumulative adapter counters before the terminal
            # barrier so the Windows lifecycle can apply every observed drop.
            self.emit({"type": "status", "status": "PAIRING_STATUS", "pairing": status,
                       "adapter_counts": dict(self._counters),
                       "ros_receive_diagnostics": self.ros_receive_diagnostics()})
            self.emit({
                "type": "status", "status": "STOP_TAIL_COMPLETE",
                "run_id": self._last_stopped_run_id,
                "pairing": status,
            })
        if (not tail_complete
                and (self._last_status_ns == 0 or now_ns - self._last_status_ns >= 1_000_000_000)):
            self._last_status_ns = now_ns
            fingerprint = (
                status.get("episode_active"), status.get("pending_states"),
                status.get("image_bytes_cached"),
                status.get("counts", {}).get("sample_emitted"),
                status.get("counts", {}).get("action_valid"),
                status.get("counts", {}).get("observation_valid"),
            )
            self._last_status_fingerprint = fingerprint
            self.emit({"type": "status", "status": "PAIRING_STATUS", "pairing": status,
                      "adapter_counts": dict(self._counters),
                      "ros_receive_diagnostics": self.ros_receive_diagnostics()})

    def _process_commands(self, now_ns: int) -> None:
        if self._command_queue is None:
            return
        while True:
            try:
                command = self._command_queue.get_nowait()
            except queue.Empty:
                return
            if command is _STDIN_EOF:
                self.mark_stdin_eof()
                return
            if isinstance(command, Mapping) and command.get("_internal_error"):
                self.emit({
                    "type": "error", "error": "stdin_protocol_error",
                    "detail": str(command["_internal_error"]),
                })
                self.mark_stdin_eof()
                return
            try:
                self.process_command(command, now_ns=now_ns)
            except Exception as exc:
                self.emit({
                    "type": "error", "error": "command_rejected",
                    "command": command, "detail": str(exc),
                })

    def process_command(self, command: Mapping[str, Any], *, now_ns: int | None = None) -> None:
        if command.get("type") != "command":
            raise ProtocolError("command frame type must be 'command'")
        name = command.get("command")
        run_id = command.get("run_id")
        if name not in ("START", "STOP", "OVER"):
            raise ProtocolError("command must be START, STOP, or OVER")
        if not isinstance(run_id, str) or not run_id:
            raise ProtocolError("lifecycle command requires run_id")
        if now_ns is None:
            now_ns = self.arrival_clock()
        active = self.assembler.status().get("run_id")
        if name == "START":
            if active is not None:
                raise ProtocolError("cannot START while another run is active")
            task = command.get("task")
            if not isinstance(task, str) or not task.strip():
                raise ProtocolError("START requires a non-empty task")
            self.assembler.start_episode(run_id, task)
            self.rb_pressed = False
            self._reported_tail_complete = False
            payload = self._lifecycle_payload("CAPTURE_START", run_id, now_ns, None)
        else:
            if active != run_id:
                raise ProtocolError("STOP/OVER run_id does not match the active episode")
            tail_ns = command.get("tail_ns", TAIL_NS)
            if type(tail_ns) is not int or not 0 <= tail_ns <= MAX_TAIL_NS:
                raise ProtocolError(f"tail_ns must be an integer from 0 to {MAX_TAIL_NS}")
            reason = command.get("reason")
            if not isinstance(reason, str) or not reason:
                reason = "operator_stop" if name == "STOP" else "operator_over"
            payload = self._lifecycle_payload("CAPTURE_STOP_REQUESTED", run_id, now_ns, reason)
            self.assembler.stop_episode(now_ns, tail_ns=tail_ns)
            self._last_stopped_run_id = run_id
        self._publish_lifecycle(payload)
        self.emit({
            "type": "event", "event": "LIFECYCLE_PUBLISH_CALL", "run_id": run_id,
            "lifecycle_event": payload, "publish_called": True,
            "acknowledged": False,
        })

    def _publish_lifecycle(self, payload: dict[str, Any]) -> None:
        if self.lifecycle_publisher is None:
            raise RuntimeError("lifecycle publisher is not attached")
        message = self.types["String"]()
        message.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        self.lifecycle_publisher.publish(message)

    @staticmethod
    def _lifecycle_payload(event: str, run_id: str, monotonic_ns: int,
                           reason: str | None) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema": "piper_gamepad_recording_lifecycle_v1",
            "event": event,
            "run_id": run_id,
            "capture_monotonic_ns": monotonic_ns,
        }
        if reason is not None:
            payload["reason"] = reason
        if event == "CAPTURE_STOP_REQUESTED":
            payload["target_generation_action"] = "REVOKE_AND_HOLD"
        return payload


def inspect_interface_abi(message_types: Mapping[str, Any]) -> dict[str, Any]:
    """Check generated interface fields required by pairing and source maps."""
    reports: dict[str, Any] = {}
    missing_by_type: dict[str, list[str]] = {}
    for name, required in REQUIRED_INTERFACE_FIELDS.items():
        message_type = message_types.get(name)
        fields = _message_field_names(message_type) if message_type is not None else None
        if fields is None:
            reports[name] = {"status": "UNKNOWN", "fields": None}
            missing_by_type[name] = sorted(required)
            continue
        missing = sorted(required - fields)
        reports[name] = {
            "status": "VALID" if not missing else "MISSING_FIELDS",
            "fields": sorted(fields),
            "required_fields_missing": missing,
        }
        if missing:
            missing_by_type[name] = missing
    return {
        "valid": not missing_by_type,
        "interfaces": reports,
        "missing_fields": missing_by_type,
    }


def validate_ros_domain(*, mode: str, ros_domain_id: str | None,
                        ros_localhost_only: str | None,
                        site_domain_confirmed: bool = False) -> int:
    """Fail closed before rclpy.init on unsafe or unconfirmed domains."""
    if mode not in ("fake", "replay", "live"):
        raise ValueError("mode must be fake, replay, or live")
    try:
        domain = int(ros_domain_id) if ros_domain_id is not None else None
    except ValueError as exc:
        raise ValueError("ROS_DOMAIN_ID must be an integer") from exc
    if domain is None:
        raise ValueError("ROS_DOMAIN_ID is required; use isolated domain 231")
    if domain == 222:
        if mode != "live" or not site_domain_confirmed:
            raise ValueError("domain 222 is refused without explicit live site confirmation")
    elif domain != 231:
        raise ValueError("only isolated domain 231 is allowed for fake/replay mode")
    if ros_localhost_only != "1":
        raise ValueError("ROS_LOCALHOST_ONLY=1 is required for this adapter")
    return domain


def ros_state_to_dict(message: Any) -> dict[str, Any]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    return {
        "header_stamp_ns": _ros_stamp_ns(stamp),
        "header_frame_id": str(getattr(header, "frame_id", "")),
        "session_id": str(message.session_id),
        "sequence": int(message.sequence),
        "source_monotonic_ns": int(message.source_monotonic_ns),
        "source_clock_id": str(message.source_clock_id),
        "source_local_monotonic_ns": int(message.source_local_monotonic_ns),
        "source_local_clock_id": str(message.source_local_clock_id),
        "source_clock_mapping_uncertainty_ns": int(message.source_clock_mapping_uncertainty_ns),
        "feedback_fresh": bool(message.feedback_fresh),
        "joint_position_rad": [float(value) for value in message.joint_position_rad],
        "gripper_nominal_m": float(message.gripper_nominal_m),
        "gripper_raw": int(message.gripper_raw),
        "hardware_connected": bool(message.hardware_connected),
        "fake": bool(message.fake),
    }


def ros_metadata_to_dict(message: Any) -> dict[str, Any]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    return {
        "header_stamp_ns": _ros_stamp_ns(stamp),
        "header_frame_id": str(getattr(header, "frame_id", "")),
        "role": str(message.role),
        "session_id": str(message.session_id),
        "device_name": str(message.device_name),
        "sequence": int(message.sequence),
        "source_monotonic_ns": int(message.source_monotonic_ns),
        "source_clock_id": str(message.source_clock_id),
        "source_local_monotonic_ns": int(message.source_local_monotonic_ns),
        "source_local_clock_id": str(message.source_local_clock_id),
        "clock_mapping_uncertainty_ns": int(message.clock_mapping_uncertainty_ns),
        "encoding": str(message.encoding),
        "width": int(message.width),
        "height": int(message.height),
    }


def ros_receipt_to_dict(message: Any) -> dict[str, Any]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    return {
        "header_stamp_ns": _ros_stamp_ns(stamp),
        "header_frame_id": str(getattr(header, "frame_id", "")),
        "request_id": str(message.request_id),
        "observation_id": str(message.observation_id),
        "decision": str(message.decision),
        "reason": str(message.reason),
        "sequence": int(message.sequence),
        "source_monotonic_ns": int(message.source_monotonic_ns),
        "adapter_accepted": bool(message.adapter_accepted),
        "feedback_matched": bool(message.feedback_matched),
        "can_tx_calls": int(message.can_tx_calls),
    }


def create_ros_adapter(
    *,
    node_factory: Callable[[str], Any] | None = None,
    emit: Callable[[dict[str, Any]], None],
    assembler: BoundedObservationAssembler | None = None,
) -> tuple[Any, ReadOnlyRosDatasetAdapter, dict[str, Any]]:
    """Create the lazy-import ROS node and adapter after domain preflight."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Joy
    from std_msgs.msg import String
    from pi05_piper_interfaces.msg import CameraFrameMetadata, CommandReceipt, StateSnapshot

    types = {
        "StateSnapshot": StateSnapshot,
        "CameraFrameMetadata": CameraFrameMetadata,
        "CommandReceipt": CommandReceipt,
        "Image": Image,
        "Joy": Joy,
        "String": String,
    }
    abi = inspect_interface_abi(types)
    if not abi["valid"]:
        raise RuntimeError(f"installed ROS interface ABI does not match: {abi['missing_fields']}")

    class AdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("pi05_live_dataset_read_only_adapter")

    node = node_factory("pi05_live_dataset_read_only_adapter") if node_factory else AdapterNode()
    adapter = ReadOnlyRosDatasetAdapter(
        node, message_types=types, sensor_qos=qos_profile_sensor_data,
        emit=emit, assembler=assembler,
    )
    adapter.attach()
    return node, adapter, types


def run_stdio_adapter(
    *,
    reader: BinaryIO | None = None,
    writer: BinaryIO | None = None,
    rclpy_module: Any | None = None,
    node_factory: Callable[[str], Any] | None = None,
    mode: str = "fake",
    site_domain_confirmed: bool = False,
    max_message_bytes: int = MAX_MESSAGE_BYTES,
) -> int:
    """Run the isolated adapter; stdout carries framed protocol only."""
    reader = reader or sys.stdin.buffer
    writer = writer or sys.stdout.buffer
    domain = validate_ros_domain(
        mode=mode,
        ros_domain_id=os.environ.get("ROS_DOMAIN_ID"),
        ros_localhost_only=os.environ.get("ROS_LOCALHOST_ONLY"),
        site_domain_confirmed=site_domain_confirmed,
    )
    if rclpy_module is None:
        import rclpy as rclpy_module
    io = LengthPrefixedMsgpackIO(reader, writer, max_message_bytes=max_message_bytes)
    outgoing = BoundedOutboundQueue()
    input_commands: queue.Queue[Any] = queue.Queue(maxsize=INPUT_QUEUE_SIZE)
    io_failure: list[BaseException] = []
    adapter_ref: list[ReadOnlyRosDatasetAdapter | None] = [None]

    def emit(payload: dict[str, Any]) -> None:
        outgoing.put(payload)

    def write_loop() -> None:
        while True:
            item = outgoing.get()
            if item is None:
                return
            try:
                outbound_item = item
                if item.get("type") == "status" and item.get("status") == "PAIRING_STATUS":
                    adapter = adapter_ref[0]
                    if adapter is not None:
                        outbound_item = dict(item)
                        adapter_counts = dict(adapter._counters)
                        adapter_counts.update(outgoing.metrics())
                        outbound_item["adapter_counts"] = adapter_counts
                io.send(outbound_item)
            except BaseException as exc:  # retain the failure for the main loop
                io_failure.append(exc)
                return

    def read_loop() -> None:
        while True:
            try:
                command = io.receive()
            except BaseException as exc:
                try:
                    input_commands.put_nowait({"_internal_error": str(exc)})
                except queue.Full:
                    pass
                return
            if command is None:
                try:
                    input_commands.put_nowait(_STDIN_EOF)
                except queue.Full:
                    pass
                return
            input_commands.put(command)

    writer_thread = threading.Thread(target=write_loop, name="live-pairing-stdout", daemon=True)
    reader_thread = threading.Thread(target=read_loop, name="live-pairing-stdin", daemon=True)
    writer_thread.start()
    reader_thread.start()
    return_code = 0
    initialized_here = False
    node = None
    try:
        if not rclpy_module.ok():
            rclpy_module.init(args=None)
            initialized_here = True
        node, adapter, types = create_ros_adapter(
            node_factory=node_factory, emit=emit,
        )
        adapter_ref[0] = adapter
        adapter._command_queue = input_commands
        outgoing.put(adapter.ready_message(ros_domain_id=domain))
        while rclpy_module.ok():
            rclpy_module.spin_once(node, timeout_sec=0.05)
            if io_failure:
                raise RuntimeError(f"stdio output failed: {io_failure[0]}")
            if adapter._stdin_eof:
                break
        if adapter._stdin_eof:
            emit({"type": "status", "status": "INPUT_EOF", "pairing": adapter.assembler.status()})
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        outgoing.post_fatal({"type": "error", "error": "adapter_failed", "detail": str(exc)})
        return_code = 1
    else:
        return_code = 0
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if initialized_here and rclpy_module.ok():
                rclpy_module.shutdown()
        except Exception:
            pass
        outgoing.close()
        writer_thread.join(timeout=2.0)
    return return_code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("fake", "replay", "live"), default="fake")
    parser.add_argument("--site-domain-confirmed", action="store_true")
    parser.add_argument("--stdio", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--max-message-bytes", type=int, default=MAX_MESSAGE_BYTES)
    args = parser.parse_args(argv)
    return run_stdio_adapter(
        mode=args.mode,
        site_domain_confirmed=args.site_domain_confirmed,
        max_message_bytes=args.max_message_bytes,
    )


def _msgpack_pack(value: Any) -> bytes:
    try:
        import msgpack
    except ImportError as exc:
        raise RuntimeError("MessagePack is required by the WSL stdio adapter") from exc
    return msgpack.packb(value, use_bin_type=True, strict_types=False)


def _msgpack_unpack(value: bytes) -> Any:
    try:
        import msgpack
    except ImportError as exc:
        raise RuntimeError("MessagePack is required by the WSL stdio adapter") from exc
    return msgpack.unpackb(value, raw=False, strict_map_key=False)


def _read_exact(reader: BinaryIO, count: int, *, allow_eof: bool = False) -> bytes | None:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = reader.read(remaining)
        if not chunk:
            if allow_eof and not chunks:
                return None
            raise ProtocolError("truncated length-prefixed stdio frame")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _message_field_names(message_type: Any) -> set[str] | None:
    if message_type is None:
        return None
    method = getattr(message_type, "get_fields_and_field_types", None)
    if callable(method):
        try:
            return set(method().keys())
        except Exception:
            return None
    fields = getattr(message_type, "_fields_and_field_types", None)
    if isinstance(fields, Mapping):
        return set(fields)
    return None


def _ros_stamp_ns(stamp: Any) -> int:
    if stamp is None:
        return 0
    return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )


def _decode_json_message(message: Any, counters: dict[str, int]) -> dict[str, Any] | None:
    value = str(getattr(message, "data", ""))
    try:
        decoded = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        counters["bad_event_json"] += 1
        return None
    return decoded if isinstance(decoded, dict) else None


if __name__ == "__main__":
    raise SystemExit(main())
