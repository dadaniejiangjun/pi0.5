"""Manual episode lifecycle for live and replay LeRobot capture.

This module has no ROS or LeRobot imports. It owns only episode boundaries and
passes adapter samples to a writer; it never publishes an action or starts a
camera source.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
import re
import time
import uuid
from typing import Any, Callable, Mapping, Protocol
from .online_proxy import OnlineNextState, SEMANTICS as PROXY_SEMANTICS


PROTOCOL = "piper_live_stdio_v1"
LIFECYCLE_SCHEMA = "piper_gamepad_recording_lifecycle_v1"
ALLOWED_LABELS = {"SAVE", "RETAKE", "SUCCESS", "FAILURE"}
DEFAULT_TAIL_S = 1.0
RB_BUTTON_INDEX = 10
DEFAULT_FINISH_TIMEOUT_S = 180.0
MAX_FINISH_TIMEOUT_S = 300.0
QUALITY_STATUS_INTERVAL_S = 5.0


class CaptureError(RuntimeError):
    """A fail-closed capture or preflight error."""


class Adapter(Protocol):
    def send_command(self, command: str, *, run_id: str, task: str = "",
                     reason: str = "", tail_ns: int | None = None) -> None: ...

    def close(self) -> None: ...


class Writer(Protocol):
    def start_episode(self, run_id: str, task: str,
                      metadata: dict[str, Any] | None = None, **kwargs: Any) -> dict[str, Any]: ...

    def submit(self, sample: Mapping[str, Any]) -> bool: ...

    def finish_episode(self, label: str = "UNREVIEWED", *,
                       events: list[dict[str, Any]] | None = None,
                       **kwargs: Any) -> dict[str, Any]: ...

    def abort_episode(self, reason: str, *, events: list[dict[str, Any]] | None = None,
                      **kwargs: Any) -> dict[str, Any]: ...

    def status(self) -> dict[str, Any]: ...

    def close(self) -> dict[str, Any]: ...


def new_run_id(now: datetime | None = None) -> str:
    """Return a collision-resistant run ID using the existing manifest format."""
    stamp = (now or datetime.now(timezone.utc)).astimezone(timezone.utc)
    return f"{stamp.strftime('%Y%m%dT%H%M%SZ')}_{uuid.uuid4().hex[:8]}"


def validate_run_id(value: Any) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,96}", value):
        raise CaptureError("run_id must contain 1-96 ASCII letters, digits, '_' or '-'")
    return value


def _number_list(value: Any, *, name: str, size: int) -> list[float]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise CaptureError(f"sample {name} must contain exactly {size} values")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise CaptureError(f"sample {name} contains a non-numeric value")
        number = float(item)
        if not math.isfinite(number):
            raise CaptureError(f"sample {name} contains a non-finite value")
        result.append(number)
    return result


def _camera_valid(sample: Mapping[str, Any]) -> dict[str, bool]:
    value = sample.get("camera_valid")
    if isinstance(value, Mapping):
        roles = {name: value.get(name) for name in ("base", "wrist")}
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        roles = {"base": value[0], "wrist": value[1]}
    else:
        raise CaptureError("sample camera_valid must contain base and wrist validity")
    if any(type(flag) is not bool for flag in roles.values()):
        raise CaptureError("sample camera_valid flags must be booleans")
    return {name: bool(flag) for name, flag in roles.items()}


def validate_sample(sample: Any, *, run_id: str, task: str) -> dict[str, Any]:
    """Validate a paired sample without inferring or filling missing evidence."""
    if not isinstance(sample, Mapping):
        raise CaptureError("paired sample must be a mapping")
    sample_run_id = validate_run_id(sample.get("run_id"))
    if sample_run_id != run_id:
        raise CaptureError(f"sample run_id {sample_run_id!r} does not match active episode {run_id!r}")
    if sample.get("task") != task:
        raise CaptureError("sample task does not match the active episode task")
    _number_list(sample.get("state"), name="state", size=7)
    _number_list(sample.get("action"), name="action", size=7)
    if type(sample.get("action_valid")) is not bool:
        raise CaptureError("sample action_valid must be an explicit boolean")
    if type(sample.get("observation_valid")) is not bool:
        raise CaptureError("sample observation_valid must be an explicit boolean")
    cameras = _camera_valid(sample)
    images = sample.get("images")
    if not isinstance(images, Mapping):
        raise CaptureError("sample images must contain base and wrist RGB frames")
    normalized_images: dict[str, Any] = {}
    for role in ("base", "wrist"):
        image = images.get(role)
        if not isinstance(image, Mapping) or image.get("encoding") != "rgb8":
            raise CaptureError(f"sample {role} image must use rgb8 encoding")
        shape = image.get("shape")
        if (not isinstance(shape, (list, tuple)) or len(shape) != 3
                or any(type(value) is not int or value <= 0 for value in shape)
                or shape[2] != 3):
            raise CaptureError(f"sample {role} image shape must be [height, width, 3]")
        pixels = image.get("data")
        if not isinstance(pixels, (bytes, bytearray, memoryview)):
            raise CaptureError(f"sample {role} image data must be RGB bytes")
        expected_bytes = shape[0] * shape[1] * shape[2]
        if len(pixels) != expected_bytes:
            raise CaptureError(f"sample {role} image byte length does not match its shape")
        current_shape = tuple(shape)
        try:
            import numpy as np
            normalized_images[role] = np.frombuffer(pixels, dtype=np.uint8).reshape(current_shape).copy()
        except (ImportError, ValueError) as exc:
            raise CaptureError(f"could not decode {role} RGB image bytes: {exc}") from exc
    source_map = sample.get("source_map")
    if not isinstance(source_map, Mapping):
        raise CaptureError("sample source_map must preserve source timestamps and pairing evidence")
    state_source = source_map.get("state")
    has_source_time = ("source_monotonic_ns" in source_map
                       or "state_source_monotonic_ns" in source_map
                       or isinstance(state_source, Mapping)
                       and "source_local_monotonic_ns" in state_source)
    if not has_source_time:
        raise CaptureError("sample source_map has no source monotonic timestamp")
    if not cameras["base"] or not cameras["wrist"]:
        # The row can still be stored with its explicit mask for review, but no
        # invalid frame is upgraded to a valid observation here.
        pass
    normalized = dict(sample)
    normalized["images"] = normalized_images
    return normalized


def validate_ready_status(status: Any, *, allow_domain_222: bool = False) -> dict[str, Any]:
    if not isinstance(status, Mapping) or status.get("type") != "status":
        raise CaptureError("ROS adapter did not return a status envelope")
    if status.get("status") != "READY" or status.get("protocol") != PROTOCOL:
        raise CaptureError(f"ROS adapter is not ready: {status.get('status', 'UNKNOWN')}")
    domain = status.get("ros_domain_id")
    if type(domain) is not int:
        raise CaptureError("ROS_DOMAIN_ID is unknown; refusing live capture")
    if domain == 222 and not allow_domain_222:
        raise CaptureError("domain 222 requires an explicit live site confirmation")
    if domain not in (231, 222):
        raise CaptureError(f"unsupported ROS domain {domain}; expected 231 or confirmed 222")
    topic_types = status.get("topic_types", status.get("topics"))
    if not isinstance(topic_types, Mapping):
        raise CaptureError("ROS adapter did not report topic/message types")
    required = {
        "/piper/state_snapshot": "pi05_piper_interfaces/msg/StateSnapshot",
        "/piper/cameras/frame_metadata": "pi05_piper_interfaces/msg/CameraFrameMetadata",
        "/piper/cameras/base/image_raw": "sensor_msgs/msg/Image",
        "/piper/cameras/wrist/image_raw": "sensor_msgs/msg/Image",
        "/gamepad/command_send_events": "std_msgs/msg/String",
        "/piper/command_receipt": "pi05_piper_interfaces/msg/CommandReceipt",
        "/gamepad/recording_lifecycle": "std_msgs/msg/String",
        "/gamepad/target_events": "std_msgs/msg/String",
        "/joy": "sensor_msgs/msg/Joy",
    }
    mismatches = {topic: (topic_types.get(topic), expected)
                  for topic, expected in required.items()
                  if topic_types.get(topic) != expected}
    if mismatches:
        raise CaptureError(f"ROS topic/message ABI mismatch: {mismatches}")
    abi = status.get("interface_abi")
    if isinstance(abi, Mapping) and abi.get("valid") is not True:
        raise CaptureError("ROS interface field ABI was not confirmed")
    schema = status.get("state_schema")
    if schema not in (None, "UNKNOWN", "piper_state_snapshot_v1"):
        raise CaptureError(f"unsupported state schema {schema!r}")
    return dict(status)


def validate_camera_identity_status(status: Any) -> dict[str, Any]:
    if not isinstance(status, Mapping) or status.get("type") != "status":
        raise CaptureError("camera identity status is malformed")
    if status.get("status") != "CAMERA_METADATA_READY":
        raise CaptureError("camera identities are not ready")
    roles = status.get("camera_identities")
    if not isinstance(roles, Mapping):
        raise CaptureError("camera identity status has no role map")
    identities = {}
    for role in ("base", "wrist"):
        item = roles.get(role)
        if not isinstance(item, Mapping):
            raise CaptureError(f"camera identity for {role} is unknown")
        if item.get("status") != "OBSERVED":
            raise CaptureError(f"camera identity for {role} is not observed")
        device_name = item.get("device_name")
        session_id = item.get("session_id")
        if (not isinstance(device_name, str) or not device_name.strip()
                or device_name.strip().upper() == "UNKNOWN"):
            raise CaptureError(f"camera identity for {role} is unknown")
        if (not isinstance(session_id, str) or not session_id.strip()
                or session_id.strip().upper() == "UNKNOWN"):
            raise CaptureError(f"camera session for {role} is unknown")
        identities[role] = dict(item)
    if identities["base"]["session_id"] != identities["wrist"]["session_id"]:
        raise CaptureError("base and wrist camera metadata belong to different sessions")
    return identities


class CaptureCoordinator:
    """Connect the human episode lifecycle to a read-only adapter and writer."""

    def __init__(self, adapter: Adapter, writer: Writer, *, operator: str, task: str,
                 manual_lifecycle: bool = True, continuous_episodes: bool = False,
                 duration_s: float = 20.0, max_capture_duration_s: float | None = None,
                 tail_s: float = DEFAULT_TAIL_S, allow_domain_222: bool = False,
                 lifecycle_timeout_s: float = 20.0,
                 finish_timeout_s: float = DEFAULT_FINISH_TIMEOUT_S,
                 online_next_state: bool = False,
                 run_id_factory: Callable[[], str] = new_run_id,
                 clock: Callable[[], float] = time.monotonic,
                 emit: Callable[[str], None] = print):
        if not operator.strip():
            raise ValueError("operator name must not be empty")
        if not task.strip():
            raise ValueError("task must not be empty")
        if not manual_lifecycle and not 0.1 <= duration_s <= 120:
            raise ValueError("fixed duration must be in (0, 120] seconds")
        if max_capture_duration_s is not None and not 1 <= max_capture_duration_s <= 3600:
            raise ValueError("maximum manual duration must be in [1, 3600] seconds")
        if not 0 <= tail_s <= 10:
            raise ValueError("tail duration must be in [0, 10] seconds")
        if not 1 <= lifecycle_timeout_s <= 120:
            raise ValueError("lifecycle timeout must be in [1, 120] seconds")
        if (not math.isfinite(finish_timeout_s)
                or not 1 <= finish_timeout_s <= MAX_FINISH_TIMEOUT_S):
            raise ValueError(
                f"finish timeout must be in [1, {MAX_FINISH_TIMEOUT_S:g}] seconds"
            )
        if continuous_episodes and not manual_lifecycle:
            raise ValueError("continuous episodes require manual lifecycle")
        self.adapter = adapter
        self.writer = writer
        self.online_next_state = online_next_state
        self.proxy_buffer = OnlineNextState()
        self.operator = operator.strip()
        self.task = task.strip()
        self.manual_lifecycle = manual_lifecycle
        self.continuous_episodes = continuous_episodes
        self.duration_s = float(duration_s)
        self.max_capture_duration_s = max_capture_duration_s
        self.tail_s = float(tail_s)
        self.lifecycle_timeout_s = float(lifecycle_timeout_s)
        self.finish_timeout_s = float(finish_timeout_s)
        self.allow_domain_222 = bool(allow_domain_222)
        self.run_id_factory = run_id_factory
        self.clock = clock
        self.emit = emit
        self.state = EpisodeState(manual=manual_lifecycle,
                                  max_capture_duration_s=max_capture_duration_s,
                                  tail_s=tail_s)
        self.ros_ready: dict[str, Any] | None = None
        self.camera_identities: dict[str, Any] | None = None
        self._camera_identity_parts: dict[str, dict[str, Any]] = {}
        self.scope_info: dict[str, Any] = {
            "status": "UNKNOWN",
            "reason": "the gamepad recording topics do not expose the active control-scope expiry",
            "task_scope_id": None,
            "remaining_s": None,
        }
        self.latest_pairing_status: dict[str, Any] | None = None
        self.latest_adapter_counts: dict[str, Any] | None = None
        self.latest_ros_receive_diagnostics: dict[str, Any] | None = None
        self._adapter_drop_seen = 0
        self.last_error: str | None = None
        self.last_finalize_failure: str | None = None
        self.closed = False
        self._last_sample_error: str | None = None
        self._last_quality_emit_at: float | None = None

    @property
    def ready(self) -> bool:
        return self.ros_ready is not None and self.camera_identities is not None

    def _episode_metadata(self) -> dict[str, Any]:
        return {
            "operator": self.operator,
            "task": self.task,
            "capture_backend": "live_lerobot_read_only_ros_adapter",
            "ros_domain_id": self.ros_ready.get("ros_domain_id") if self.ros_ready else None,
            "ros_profile": self.ros_ready.get("ros_profile") if self.ros_ready else "UNKNOWN",
            "ros_topic_types": (self.ros_ready.get("topic_types", self.ros_ready.get("topics", {}))
                                if self.ros_ready else {}),
            "interface_abi": self.ros_ready.get("interface_abi") if self.ros_ready else "UNKNOWN",
            "camera_identities": self.camera_identities,
            "scope_info": dict(self.scope_info),
            "action_semantics": (PROXY_SEMANTICS if self.online_next_state else
                                 "matched actual send/receipt evidence; missing evidence remains invalid"),
            "online_next_state": self.online_next_state,
            "physical_stop_verified": False,
            "robot_motion_commanded_by_recorder": False,
        }

    def _require_ready(self) -> None:
        if not self.ready:
            missing = []
            if self.ros_ready is None:
                missing.append("ROS schema")
            if self.camera_identities is None:
                missing.extend(role for role in ("base", "wrist")
                               if role not in self._camera_identity_parts)
                if not missing:
                    missing.append("camera identity confirmation")
            raise CaptureError(
                "Preflight incomplete: " + ", ".join(missing)
                + ". Start cameras in another PowerShell: "
                + r".\gamepad_capture\scripts\gamepad_teleop.ps1 -Mode Cameras"
                + "; keep that window open, then wait for CAPTURE_READY before START."
            )
        if self.last_error:
            raise CaptureError(self.last_error)

    def _observe_camera_identity(self, role: Any, identity: Mapping[str, Any]) -> None:
        if role not in ("base", "wrist"):
            raise CaptureError(f"unexpected camera role {role!r}")
        identity_status = identity.get("status", "OBSERVED")
        current = {
            "status": "OBSERVED" if identity_status == "CAMERA_IDENTITY_OBSERVED" else identity_status,
            "session_id": identity.get("session_id"),
            "device_name": identity.get("device_name"),
        }
        if current["status"] != "OBSERVED":
            raise CaptureError(f"camera identity for {role} is not observed")
        previous = self._camera_identity_parts.get(role)
        if previous is not None and previous != current:
            if self.state.state in ("STARTING", "CAPTURING", "STOP_REQUESTED", "TAIL_RECORDING"):
                self.state.partial_reasons.append("camera_identity_changed_during_episode")
                self._abort_active("camera_identity_changed_during_episode")
                raise CaptureError(f"camera identity for {role} changed during an active episode")
            self.camera_identities = None
        self._camera_identity_parts[role] = current
        if set(self._camera_identity_parts) == {"base", "wrist"}:
            aggregate = {
                "type": "status",
                "status": "CAMERA_METADATA_READY",
                "camera_identities": self._camera_identity_parts,
            }
            self.camera_identities = validate_camera_identity_status(aggregate)
            self.emit("CAMERAS_READY: base+wrist identities reported by ROS metadata")

    def _emit_quality_line(self, *, force: bool = False) -> None:
        if self.state.state not in (
                "STARTING", "CAPTURING", "STOP_REQUESTED", "TAIL_RECORDING"):
            return
        now = self.clock()
        if (not force and self._last_quality_emit_at is not None
                and now - self._last_quality_emit_at < QUALITY_STATUS_INTERVAL_S):
            return
        self._last_quality_emit_at = now
        run_id = self.state.run_id or "UNKNOWN"
        pairing = self.latest_pairing_status or {}
        pairing_counts = pairing.get("counts", {}) if isinstance(pairing, Mapping) else {}
        if not isinstance(pairing_counts, Mapping):
            pairing_counts = {}
        writer = self.writer.status()
        receive = self.latest_ros_receive_diagnostics or {}
        matched = receive.get("matched_publishers", {})
        callbacks = receive.get("callback_counts", {})
        if not isinstance(matched, Mapping):
            matched = {}
        if not isinstance(callbacks, Mapping):
            callbacks = {}
        receive_labels = ("state_snapshot", "camera_metadata", "base_image", "wrist_image")
        match_text = ",".join(f"{name}:{matched.get(name, 'UNKNOWN')}"
                              for name in receive_labels)
        callback_text = ",".join(f"{name}:{callbacks.get(name, 'UNKNOWN')}"
                                 for name in receive_labels)
        camera_status = "UNKNOWN"
        if self.camera_identities:
            camera_status = "base=OBSERVED,wrist=OBSERVED"
        else:
            observed = sorted(self._camera_identity_parts)
            camera_status = ",".join(f"{role}=OBSERVED" for role in observed) or "base=UNKNOWN,wrist=UNKNOWN"
        self.emit(
            f"CAPTURE_QUALITY: state={self.state.state}; run_id={run_id}; "
            f"frames={self.state.counters['frames']}; "
            f"observation_valid={self.state.counters['observation_valid']}; "
            f"action_valid={self.state.counters['action_valid']}; "
            f"camera_pair_valid={self.state.counters['camera_pair_valid']}; "
            f"writer_dropped={self.state.counters['writer_dropped']}; "
            f"transport_dropped={self.state.counters['transport_dropped']}; "
            f"adapter_dropped={self.state.counters['adapter_dropped']}; "
            f"paired_emitted={pairing_counts.get('sample_emitted', 'UNKNOWN')}; "
            f"action_masked={pairing_counts.get('action_masked', 'UNKNOWN')}; "
            f"observation_masked={pairing_counts.get('observation_masked', 'UNKNOWN')}; "
            f"pending_states={pairing.get('pending_states', 'UNKNOWN')}; "
            f"ros_matches={match_text}; ros_callbacks={callback_text}; "
            f"cameras={camera_status}; "
            f"writer_queue={writer.get('queue_depth', 'UNKNOWN')}/"
            f"{writer.get('queue_capacity', 'UNKNOWN')}; "
            f"scope_remaining_s=UNKNOWN; physical_stop_verified=false"
        )

    def start_episode(self) -> str:
        self._require_ready()
        run_id = validate_run_id(self.run_id_factory())
        if run_id == self.state.run_id:
            raise CaptureError("run_id generator repeated the previous episode ID")
        self.state.begin(run_id, now=self.clock())
        self.proxy_buffer = OnlineNextState()
        self._last_quality_emit_at = None
        try:
            writer_start = self.writer.start_episode(run_id, self.task, self._episode_metadata())
            if isinstance(writer_start, Mapping):
                start_evidence = dict(writer_start)
                writer_status = writer_start.get("status")
            else:
                start_evidence = {
                    "type": type(writer_start).__name__,
                    "repr": repr(writer_start),
                }
                writer_status = None
            self.state.events.append({
                "owner": "windows_recorder",
                "event": "WRITER_START_RESULT",
                "run_id": run_id,
                "result": start_evidence,
            })
            if writer_status != "RECORDING":
                observed = writer_status if writer_status is not None else "UNKNOWN"
                raise CaptureError(
                    f"writer start did not confirm RECORDING (status={observed})"
                )
            self.adapter.send_command("START", run_id=run_id, task=self.task)
        except Exception as exc:
            error = f"episode start failed: {exc}"
            self.last_error = error
            self.state.events.append({
                "owner": "windows_recorder",
                "event": "EPISODE_START_FAILED",
                "run_id": run_id,
                "error": error,
            })
            self._abort_active("episode_start_failed")
            raise CaptureError(error) from exc
        self.emit(f"CAPTURE_STARTING: run_id={run_id}; waiting for lifecycle publish record")
        return run_id

    def request_stop(self, reason: str, *, over: bool = False) -> None:
        if self.state.state not in ("STARTING", "CAPTURING") or not self.state.run_id:
            raise CaptureError("there is no active episode to stop")
        if self.state.state == "STOP_REQUESTED":
            return
        run_id = self.state.run_id
        self.state.request_stop(reason, over=over, now=self.clock())
        self.state.events.append({"owner": "windows_recorder", "event": "STOP_COMMAND_SENT",
                                  "run_id": run_id, "reason": reason, "over": over})
        try:
            self.adapter.send_command("OVER" if over else "STOP", run_id=run_id,
                                      reason=reason, tail_ns=round(self.tail_s * 1e9))
        except Exception as exc:
            self.state.partial_reasons.append("stop_command_transport_failed")
            self.last_error = f"stop command transport failed: {exc}"
            raise CaptureError(self.last_error) from exc
        self.emit(f"CAPTURE_STOP_REQUESTED: run_id={run_id}; reason={reason}; physical_stop_verified=false")

    def command(self, command: str) -> str:
        value = apply_episode_command(self.state, command)
        if value == "IGNORED":
            return value
        if value == "START":
            return self.start_episode()
        if value == "OVER_EXIT":
            self.state.state = "CLOSED"
            self.emit("SEQUENCE_OVER: no active episode")
            return value
        if value in ("STOP", "OVER"):
            self.request_stop(f"terminal_{value.lower()}", over=value == "OVER")
            return value
        self.state.mark_labeled(value)
        self.emit(f"LABEL_ACCEPTED: {value} | "
                  + {'SUCCESS': '成功', 'FAILURE': '失败', 'RETAKE': '重录',
                     'SAVE': '保存但未判定成功'}[value])
        return self._finalize_episode(label=value)

    def tick(self, *, now: float | None = None) -> None:
        current = self.clock() if now is None else float(now)
        if self.state.state == "STARTING" and self.state.started_at is not None:
            if current - self.state.started_at >= self.lifecycle_timeout_s:
                self._abort_active("capture_start_publish_timeout")
                raise CaptureError("timed out waiting for the CAPTURE_START lifecycle publish record")
        if self.state.state in ("STOP_REQUESTED", "TAIL_RECORDING"):
            requested = self.state.stop_requested_at
            if requested is not None and current - requested >= self.lifecycle_timeout_s:
                self._abort_active("capture_stop_tail_timeout")
                raise CaptureError("timed out waiting for the STOP_TAIL_COMPLETE status")
        if self.state.state != "CAPTURING" or self.state.started_at is None:
            return
        limit = self.max_capture_duration_s if self.manual_lifecycle else self.duration_s
        if limit is not None and current - self.state.started_at >= float(limit):
            reason = "max_duration_reached" if self.manual_lifecycle else "requested_duration_complete"
            self.request_stop(reason)

    def _finalize_episode(self, *, label: str) -> dict[str, Any]:
        if self.online_next_state:
            self.state.events.append({'event': 'ONLINE_PROXY_FINISHED',
                                      'run_id': self.state.run_id,
                                      **self.proxy_buffer.finish()})
        quality = self.state.summary()
        quality["scope_info"] = dict(self.scope_info)
        quality_event = {
            "owner": "windows_recorder",
            "event": "LIFECYCLE_QUALITY_SUMMARY",
            "run_id": self.state.run_id,
            "quality": quality,
            "pairing_status": self.latest_pairing_status,
            "adapter_counts": self.latest_adapter_counts,
            "ros_receive_diagnostics": self.latest_ros_receive_diagnostics,
            "writer_pre_finalize_status": self.writer.status(),
        }
        label_payload: dict[str, Any] = {
            "demonstration_label": label,
            "label_source": "operator" if label in ALLOWED_LABELS else "none",
            "human_labeled": label in ALLOWED_LABELS,
            "outcome": ("success" if label == "SUCCESS" else
                        "failure" if label == "FAILURE" else "unknown"),
            "successful_complete_end": label == "SUCCESS",
            "complete_closeout_label_present": label == "SUCCESS",
            "success_is_operator_assertion": label == "SUCCESS",
            "retake_requested": label == "RETAKE",
            "training_eligible": False,
            "physical_stop_verified": False,
            "robot_execution_verified": False,
        }
        run_id = self.state.run_id or "UNKNOWN"
        self.emit(
            f"EPISODE_INPUT_ENDED: run_id={run_id}; frames={self.state.counters['frames']}; "
            "waiting for writer queue drain; physical_stop_verified=false"
        )

        progress_messages = {
            "queue_drained": (
                "WRITER_QUEUE_DRAINED: queued samples written; episode/video finalization continues"
            ),
            "video_encoding_started": "WRITER_VIDEO_ENCODING: 双路视频编码中；可准备下一轮物体，暂不要操作机械臂",
            "video_encoding_complete": "WRITER_VIDEO_ENCODING_COMPLETE: video encoder returned",
            "episode_save_started": "WRITER_EPISODE_SAVE_STARTED: committing LeRobot episode",
            "dataset_saved": (
                "WRITER_DATASET_SAVED: episode files written; provenance commit is pending"
            ),
            "finish_timed_out": "WRITER_FINISH_TIMEOUT: pending partial data is retained",
        }

        def report_progress(phase: str) -> None:
            message = progress_messages.get(phase)
            if message is not None:
                self.emit(f"{message}; run_id={run_id}; physical_stop_verified=false")

        report = self.writer.finish_episode(
            label=label_payload, events=[*self.state.events, quality_event],
            timeout_s=self.finish_timeout_s, progress_callback=report_progress)
        writer_status = report.get("status")
        timings = report.get('performance', {}).get('timings', {})
        if timings:
            self.emit('EPISODE_TIMING: ' + '; '.join(
                f'{name}={timings[name].get("total_s", 0):.2f}s'
                for name in ('video_encode', 'save_episode_total', 'sidecar_commit_and_cleanup', 'finish_total')
                if name in timings))
        initial_summary = self.state.summary()
        complete = (not initial_summary["partial_reasons"]
                  and self.state.counters["writer_dropped"] == 0
                  and self.state.counters["transport_dropped"] == 0
                  and self.state.counters["adapter_dropped"] == 0
                  and writer_status == "SAVED_REVIEW_ONLY"
                  and report.get("dropped_frames", 0) == 0)
        status = "SAVED_REVIEW_ONLY" if complete else "INCOMPLETE"
        if complete:
            self.state.mark_finalized(continuous=self.continuous_episodes)
            self.emit(
                f"EPISODE_SAVED: status=SAVED_REVIEW_ONLY; run_id={run_id}; "
                "dataset and provenance commit complete; physical_stop_verified=false"
            )
        else:
            reasons = list(initial_summary["partial_reasons"])
            counters = initial_summary["counters"]
            for counter_name in ("writer_dropped", "transport_dropped", "adapter_dropped"):
                count = counters[counter_name]
                if count > 0 and not any(counter_name in reason for reason in reasons):
                    reasons.append(f"{counter_name}={count}")
            if writer_status != "SAVED_REVIEW_ONLY":
                reasons.append(f"writer_status={writer_status or 'UNKNOWN'}")
            report_dropped = report.get("dropped_frames", 0)
            if report_dropped > 0:
                reasons.append(f"writer_report_dropped_frames={report_dropped}")
            if report.get("error"):
                reasons.append(f"writer_error={report['error']}")
            reason = "; ".join(reasons) or "finalization_quality_gate_failed"
            self.last_finalize_failure = f"episode finalization incomplete: {reason}"
            if "writer_finalize_incomplete" not in self.state.partial_reasons:
                self.state.partial_reasons.append("writer_finalize_incomplete")
            self.state.state = "CLOSED"
            pending_path = report.get("pending_path")
            pending_note = f"; pending={pending_path}" if pending_path else ""
            self.emit(
                f"EPISODE_FINALIZE_INCOMPLETE: reason={reason}; "
                f"partial=true; run_id={run_id}{pending_note}; physical_stop_verified=false"
            )
        summary = self.state.summary()
        self.emit(
            f"EPISODE_QUALITY: status={status}; run_id={summary['run_id']}; "
            f"frames={summary['counters']['frames']}; "
            f"observation_valid={summary['counters']['observation_valid']}; "
            f"camera_pair_valid={summary['counters']['camera_pair_valid']}; "
            f"action_valid={summary['counters']['action_valid']}; "
            f"writer_dropped={summary['counters']['writer_dropped']}; "
            f"label={summary['episode_label']}; physical_stop_verified=false"
        )
        if self.state.state == "WAITING_FOR_NEXT_START":
            self.emit("SEQUENCE_READY: enter START for another episode or OVER to finish")
        return {"status": status, "summary": summary, "writer": report}

    def _abort_active(self, reason: str) -> None:
        if self.state.run_id and self.state.state not in ("CLOSED", "WAITING_FOR_START",
                                                          "WAITING_FOR_NEXT_START"):
            self.state.partial_reasons.append(reason)
            self.state.events.append({"owner": "windows_recorder", "event": "CAPTURE_ABORTED",
                                      "run_id": self.state.run_id, "reason": reason})
            quality = self.state.summary()
            quality["scope_info"] = dict(self.scope_info)
            try:
                events = [*self.state.events, {
                    "owner": "windows_recorder", "event": "LIFECYCLE_QUALITY_SUMMARY",
                    "run_id": self.state.run_id, "quality": quality,
                    "pairing_status": self.latest_pairing_status,
                    "adapter_counts": self.latest_adapter_counts,
                    "ros_receive_diagnostics": self.latest_ros_receive_diagnostics,
                    "writer_pre_abort_status": self.writer.status(),
                }]
                self.writer.abort_episode(reason, events=events)
            except Exception as exc:
                self.last_error = f"writer abort failed: {exc}"
                self.emit(f"RECORDER_ABORT_FAILED: {self.last_error}")
            finally:
                self.state.state = "CLOSED"

    def record_transport_drops(self, count: int) -> None:
        if type(count) is not int or count <= 0:
            return
        if self.state.state in ("STARTING", "CAPTURING", "STOP_REQUESTED", "TAIL_RECORDING"):
            self.state.counters["transport_dropped"] += count
            if "stdio_transport_drop" not in self.state.partial_reasons:
                self.state.partial_reasons.append("stdio_transport_drop")

    def handle(self, envelope: Any) -> None:
        """Consume one WSL protocol envelope or paired sample."""
        if not isinstance(envelope, Mapping):
            raise CaptureError("adapter returned a non-map protocol envelope")
        kind = envelope.get("type")
        if kind == "status":
            status = envelope.get("status")
            if status == "READY":
                self.ros_ready = validate_ready_status(
                    envelope, allow_domain_222=self.allow_domain_222)
                receive = envelope.get("ros_receive_diagnostics")
                if isinstance(receive, Mapping):
                    self.latest_ros_receive_diagnostics = dict(receive)
                abi = envelope.get("interface_abi")
                if not isinstance(abi, Mapping) or abi.get("valid") is not True:
                    raise CaptureError("ROS message schema/field ABI was not confirmed")
                self.emit(f"ROS_READY: domain={self.ros_ready['ros_domain_id']}; schema=PASS")
                for role, identity in self.ros_ready.get("camera_identities", {}).items():
                    if isinstance(identity, Mapping) and identity.get("status") == "OBSERVED":
                        self._observe_camera_identity(role, identity)
            elif status == "CAMERA_IDENTITY_OBSERVED":
                role = envelope.get("role")
                self._observe_camera_identity(role, envelope)
            elif status == "CAMERA_METADATA_READY":
                identities = envelope.get("camera_identities", {})
                for role, identity in identities.items():
                    if isinstance(identity, Mapping):
                        self._observe_camera_identity(role, identity)
            elif status == "PAIRING_STATUS":
                pairing = envelope.get("pairing")
                if isinstance(pairing, Mapping):
                    self.latest_pairing_status = dict(pairing)
                receive = envelope.get("ros_receive_diagnostics")
                if isinstance(receive, Mapping):
                    self.latest_ros_receive_diagnostics = dict(receive)
                counts = envelope.get("adapter_counts")
                if isinstance(counts, Mapping):
                    self.latest_adapter_counts = dict(counts)
                    current_drops = counts.get("outbound_events_dropped", 0)
                    if type(current_drops) is int and current_drops > self._adapter_drop_seen:
                        delta = current_drops - self._adapter_drop_seen
                        self._adapter_drop_seen = current_drops
                        if self.state.state in ("STARTING", "CAPTURING", "STOP_REQUESTED", "TAIL_RECORDING"):
                            self.state.counters["adapter_dropped"] += delta
                            if "ros_adapter_output_drop" not in self.state.partial_reasons:
                                self.state.partial_reasons.append("ros_adapter_output_drop")
                self._emit_quality_line()
            elif status == "STOP_TAIL_COMPLETE":
                if self.state.run_id is None or envelope.get("run_id") != self.state.run_id:
                    raise CaptureError("tail completion run_id does not match active episode")
                self.state.events.append(dict(envelope))
                self.state.mark_tail_complete()
                if not self.manual_lifecycle:
                    self.state.state = "FINALIZING"
                    self._finalize_episode(label="UNREVIEWED")
                else:
                    self.emit(
                        f"EPISODE_AWAITING_LABEL: run_id={self.state.run_id}; "
                        "输入 SUCCESS=成功 / FAILURE=失败 / RETAKE=重录 / SAVE=未判定，然后回车。"
                        "输错可按退格修改；回车后显示 LABEL_ACCEPTED 确认。"
                    )
            elif status in ("ERROR", "FAILED", "STREAM_ERROR", "ADAPTER_CLOSED"):
                detail = str(envelope.get("error", envelope.get("detail", status)))
                self.last_error = detail
                self._abort_active(f"adapter_{str(status).lower()}")
                raise CaptureError(f"ROS adapter {status}: {detail}")
            return
        if kind == "event":
            event = envelope.get("event")
            run_id = envelope.get("run_id")
            if event == "LIFECYCLE_PUBLISH_CALL":
                lifecycle = envelope.get("lifecycle_event")
                if not isinstance(lifecycle, Mapping) or lifecycle.get("schema") != LIFECYCLE_SCHEMA:
                    raise CaptureError("adapter lifecycle publish record has an unknown schema")
                if envelope.get("publish_called") is not True:
                    raise CaptureError("lifecycle envelope does not confirm a publish() call")
                if run_id != self.state.run_id or lifecycle.get("run_id") != self.state.run_id:
                    raise CaptureError("lifecycle publish record has the wrong run_id")
                self.state.events.append(dict(envelope))
                if lifecycle.get("event") == "CAPTURE_START":
                    self.state.mark_started(now=self.clock())
                    self.emit(f"CAPTURE_START_PUBLISH_CALLED: run_id={self.state.run_id}; "
                              "recording lifecycle publish() called; waiting for operator status events")
                elif lifecycle.get("event") == "CAPTURE_STOP_REQUESTED":
                    if lifecycle.get("target_generation_action") != "REVOKE_AND_HOLD":
                        raise CaptureError("CAPTURE_STOP_REQUESTED lacks REVOKE_AND_HOLD")
                    self.state.mark_tail()
                    self.emit(f"CAPTURE_TAIL: run_id={self.state.run_id}; tail={self.tail_s:g}s")
                else:
                    raise CaptureError(f"unsupported lifecycle event {lifecycle.get('event')!r}")
            elif event == "OPERATOR_EVENT":
                payload = envelope.get("payload")
                if not isinstance(payload, Mapping):
                    raise CaptureError("operator event has no JSON object payload")
                record = dict(payload)
                record.update({"owner": "operator", "arrival_monotonic_ns": envelope.get("arrival_monotonic_ns")})
                self.state.events.append(record)
                if payload.get("status") == "CONTROL_SCOPE_EXPIRED":
                    self.scope_info = {"status": "EXPIRED", "task_scope_id": payload.get("task_scope_id"),
                                       "remaining_s": 0, "source_event": record}
                    self.state.partial_reasons.append("control_scope_expired_observed")
                if payload.get("status") in (
                        "RECORDING_PAUSED_SOFTWARE_ONLY",
                        "RECORDING_RESUMED_SOFTWARE_ONLY"):
                    self.emit(f"CAPTURE_OPERATOR_ACK: status={payload['status']}; "
                              "acknowledgment=software_only; physical_stop_verified=false")
                scope_id = payload.get("task_scope_id")
                if isinstance(scope_id, str) and scope_id:
                    self.scope_info = {"status": "ID_REPORTED_REMAINING_UNKNOWN",
                                       "task_scope_id": scope_id, "remaining_s": None}
            elif event == "RB_RISING_EDGE":
                self.state.events.append(dict(envelope))
                if (self.state.run_id and self.state.state in ("STARTING", "CAPTURING")
                        and (run_id in (None, self.state.run_id))):
                    if envelope.get("button_index") != RB_BUTTON_INDEX:
                        raise CaptureError("RB stop event has an unexpected button index")
                    self.request_stop("rb_rising_edge")
            else:
                self.state.events.append(dict(envelope))
            return
        if kind == "sample":
            if self.state.state not in ("CAPTURING", "TAIL_RECORDING"):
                return
            sample = validate_sample(envelope.get("sample"), run_id=self.state.run_id or "",
                                     task=self.task)
            if self.online_next_state:
                sample = self.proxy_buffer.push(sample)
                if sample is None:
                    return
            accepted = self.writer.submit(sample)
            self.state.record_sample(sample, accepted=accepted)
            return
        if kind in ("error", "closed"):
            detail = str(envelope.get("error", envelope.get("reason", kind)))
            self.last_error = detail
            self._abort_active(f"adapter_{kind}")
            raise CaptureError(detail)
        raise CaptureError(f"unsupported adapter envelope type {kind!r}")

    def close(self) -> dict[str, Any]:
        if self.closed:
            return {
                "status": "INCOMPLETE" if self.last_finalize_failure else "CLOSED",
                "error": self.last_finalize_failure,
            }
        close_error = None
        try:
            if self.state.state not in ("CLOSED", "WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
                self._abort_active("recorder_closed_before_episode_finalized")
        except Exception as exc:
            close_error = str(exc)
        try:
            writer_report = self.writer.close()
        except Exception as exc:
            writer_report = {"status": "CLOSE_FAILED", "error": str(exc)}
        if writer_report.get("status") != "CLOSED":
            close_error = close_error or (
                f"writer close did not complete: {writer_report.get('status', 'UNKNOWN')}"
            )
        if self.last_finalize_failure:
            close_error = close_error or self.last_finalize_failure
        try:
            self.adapter.close()
        except Exception as exc:
            close_error = close_error or str(exc)
        self.closed = True
        return {"status": "CLOSED" if close_error is None else "INCOMPLETE",
                "writer": writer_report, "error": close_error}


@dataclass
class EpisodeState:
    """Small lifecycle state machine used by the live CLI and offline tests."""

    manual: bool = True
    max_capture_duration_s: float | None = None
    tail_s: float = DEFAULT_TAIL_S
    state: str = "WAITING_FOR_START"
    run_id: str | None = None
    started_at: float | None = None
    stop_requested_at: float | None = None
    stop_reason: str | None = None
    terminal_over: bool = False
    label: str | None = None
    events: list[dict[str, Any]] = field(default_factory=list)
    counters: dict[str, int] = field(default_factory=lambda: {
        "frames": 0, "observation_valid": 0, "action_valid": 0,
        "camera_pair_valid": 0, "writer_dropped": 0, "transport_dropped": 0,
        "adapter_dropped": 0,
    })
    partial_reasons: list[str] = field(default_factory=list)

    def begin(self, run_id: str, *, now: float | None = None) -> None:
        if self.state not in ("WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
            raise CaptureError(f"cannot start an episode while state is {self.state}")
        self.run_id = validate_run_id(run_id)
        self.started_at = time.monotonic() if now is None else float(now)
        self.stop_requested_at = None
        self.stop_reason = None
        self.terminal_over = False
        self.label = None
        self.events = []
        self.counters = {"frames": 0, "observation_valid": 0, "action_valid": 0,
                         "camera_pair_valid": 0, "writer_dropped": 0,
                         "transport_dropped": 0, "adapter_dropped": 0}
        self.partial_reasons = []
        self.state = "STARTING"

    def mark_started(self, *, now: float | None = None) -> None:
        if self.state != "STARTING":
            raise CaptureError(f"unexpected capture start event in state {self.state}")
        self.started_at = time.monotonic() if now is None else float(now)
        self.state = "CAPTURING"

    def request_stop(self, reason: str, *, over: bool = False,
                     now: float | None = None) -> None:
        if self.state not in ("STARTING", "CAPTURING"):
            raise CaptureError(f"cannot stop an episode while state is {self.state}")
        self.stop_reason = str(reason)
        self.stop_requested_at = time.monotonic() if now is None else float(now)
        self.terminal_over = bool(over)
        self.state = "STOP_REQUESTED"

    def mark_tail(self) -> None:
        if self.state != "STOP_REQUESTED":
            raise CaptureError(f"cannot start tail recording while state is {self.state}")
        self.state = "TAIL_RECORDING"

    def mark_tail_complete(self) -> None:
        if self.state not in ("TAIL_RECORDING", "STOP_REQUESTED"):
            raise CaptureError(f"tail completed while state is {self.state}")
        self.state = "AWAITING_LABEL" if self.manual else "FINALIZING"

    def mark_labeled(self, label: str) -> None:
        value = str(label).strip().upper()
        if value not in ALLOWED_LABELS:
            raise CaptureError(f"episode label must be one of {', '.join(sorted(ALLOWED_LABELS))}")
        if self.state != "AWAITING_LABEL":
            raise CaptureError(f"cannot label an episode while state is {self.state}")
        self.label = value
        self.state = "FINALIZING"

    def mark_finalized(self, *, continuous: bool) -> None:
        if self.state != "FINALIZING":
            raise CaptureError(f"cannot finalize an episode while state is {self.state}")
        self.state = "WAITING_FOR_NEXT_START" if continuous and not self.terminal_over else "CLOSED"

    def record_sample(self, sample: Mapping[str, Any], *, accepted: bool) -> None:
        if self.state not in ("CAPTURING", "TAIL_RECORDING"):
            return
        self.counters["frames"] += 1
        self.counters["observation_valid"] += int(bool(sample.get("observation_valid")))
        self.counters["action_valid"] += int(bool(sample.get("action_valid")))
        cameras = _camera_valid(sample)
        self.counters["camera_pair_valid"] += int(cameras["base"] and cameras["wrist"])
        if not accepted:
            self.counters["writer_dropped"] += 1
            if "writer_queue_full" not in self.partial_reasons:
                self.partial_reasons.append("writer_queue_full")

    def summary(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "run_id": self.run_id,
            "capture_stop_reason": self.stop_reason,
            "episode_label": self.label or "UNREVIEWED",
            "physical_stop_verified": False,
            "operator_ack": {
                "paused_software_only": any(
                    item.get("status") == "RECORDING_PAUSED_SOFTWARE_ONLY" for item in self.events
                ),
                "resumed_software_only": any(
                    item.get("status") == "RECORDING_RESUMED_SOFTWARE_ONLY" for item in self.events
                ),
            },
            "counters": dict(self.counters),
            "partial_reasons": list(self.partial_reasons),
        }


def apply_episode_command(state: EpisodeState, command: str) -> str:
    """Validate the user-facing manual lifecycle command vocabulary."""
    value = str(command).strip().upper()
    if value not in ("START", "STOP", "OVER", *ALLOWED_LABELS):
        return "IGNORED"
    if value == "START":
        if state.state not in ("WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
            raise CaptureError("START is only accepted while waiting for an episode")
        return "START"
    if value == "OVER" and state.state in ("WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
        return "OVER_EXIT"
    if value in ("STOP", "OVER"):
        if state.state not in ("CAPTURING", "STARTING"):
            raise CaptureError(f"{value} is only accepted while an episode is active")
        return value
    if state.state != "AWAITING_LABEL":
        raise CaptureError(f"{value} is only accepted after the capture tail completes")
    return value
