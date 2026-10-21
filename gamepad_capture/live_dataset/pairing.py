"""Bounded observation/action pairing for live and replayed gamepad samples."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import hashlib
import heapq
import json
import math
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

import numpy as np

from gamepad_capture.tools.dataset.bag import (
    RosbagReader,
    decode_command_receipt_bytes,
    verify_source,
)
from gamepad_capture.tools.dataset.cdr import (
    decode_camera_metadata,
    decode_image,
    decode_image_header,
    decode_state_snapshot,
    decode_std_string,
)
from gamepad_capture.tools.dataset.semantics import (
    group_command_events,
    raw_target_to_nominal_action,
)

from .contracts import (
    CAMERA_ROLES,
    ContractError,
    action_vector,
    image_header_stamp_ns,
    image_to_rgb,
    state_vector,
    validate_rgb_image,
)


DEFAULT_HZ = 10
DEFAULT_MAX_ACTION_AGE_MS = 250
DEFAULT_CAMERA_MATCH_MS = 60
DEFAULT_PAIR_WAIT_MS = 300
DEFAULT_REQUEST_EVENT_GRACE_MS = 350
DEFAULT_CACHE_AGE_MS = 2_000
DEFAULT_MAX_TOTAL_IMAGE_BYTES = 96 * 1024 * 1024
COMMAND_EVENT_NAMES = {
    "TARGET_INTENT",
    "CAN_WRITE_INTENT",
    "CAN_KERNEL_ACCEPTED",
    "CAN_DELIVERY_UNKNOWN",
    "SERVICE_RESULT",
}


@dataclass
class _Timed:
    payload: dict[str, Any]
    arrival_ns: int


@dataclass
class _Image:
    rgb: np.ndarray
    header_stamp_ns: int
    header_frame_id: str
    source_encoding: str
    arrival_ns: int
    sha256: str


@dataclass
class _PendingState:
    state: dict[str, Any]
    arrival_ns: int


@dataclass
class _RequestGroup:
    events: list[_Timed]
    receipts: list[_Timed]
    first_arrival_ns: int
    finalized: bool = False
    result: dict[str, Any] | None = None


class BoundedObservationAssembler:
    """Join one-use camera frames and actual send receipts to fresh states.

    Callbacks only append bounded records. ``poll`` performs pairing after a
    finite wait, so no callback sleeps while waiting for a receipt or image.
    The default row grid is nominal 10 Hz; source monotonic timestamps remain
    authoritative and are never rewritten to that nominal grid.
    """

    def __init__(
        self,
        *,
        sample_hz: int = DEFAULT_HZ,
        max_action_age_ms: int = DEFAULT_MAX_ACTION_AGE_MS,
        camera_match_ms: int = DEFAULT_CAMERA_MATCH_MS,
        pair_wait_ms: int = DEFAULT_PAIR_WAIT_MS,
        request_event_grace_ms: int = DEFAULT_REQUEST_EVENT_GRACE_MS,
        cache_age_ms: int = DEFAULT_CACHE_AGE_MS,
        max_states: int = 64,
        max_metadata_per_role: int = 64,
        max_images_per_role: int = 24,
        max_request_groups: int = 256,
        max_total_image_bytes: int = DEFAULT_MAX_TOTAL_IMAGE_BYTES,
        expected_session_id: str | None = None,
    ) -> None:
        for name, value in (
            ("sample_hz", sample_hz),
            ("max_action_age_ms", max_action_age_ms),
            ("camera_match_ms", camera_match_ms),
            ("pair_wait_ms", pair_wait_ms),
            ("request_event_grace_ms", request_event_grace_ms),
            ("cache_age_ms", cache_age_ms),
            ("max_states", max_states),
            ("max_metadata_per_role", max_metadata_per_role),
            ("max_images_per_role", max_images_per_role),
            ("max_request_groups", max_request_groups),
            ("max_total_image_bytes", max_total_image_bytes),
        ):
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if cache_age_ms < pair_wait_ms:
            raise ValueError("cache_age_ms must be at least pair_wait_ms")
        self.sample_hz = sample_hz
        self.period_ns = 1_000_000_000 // sample_hz
        self.max_action_age_ns = max_action_age_ms * 1_000_000
        self.camera_match_ns = camera_match_ms * 1_000_000
        self.pair_wait_ns = pair_wait_ms * 1_000_000
        self.request_event_grace_ns = request_event_grace_ms * 1_000_000
        self.request_group_timeout_ns = self.pair_wait_ns + self.request_event_grace_ns
        self.cache_age_ns = cache_age_ms * 1_000_000
        self.max_states = max_states
        self.max_metadata_per_role = max_metadata_per_role
        self.max_images_per_role = max_images_per_role
        self.max_request_groups = max_request_groups
        self.max_total_image_bytes = max_total_image_bytes
        self.expected_session_id = expected_session_id

        self._active_run_id: str | None = None
        self._run_session_id: str | None = None
        self._task = ""
        self._stop_tail_deadline_ns: int | None = None
        self._episode_ended = False
        self._pending_states: deque[_PendingState] = deque()
        self._metadata: dict[str, deque[_Timed]] = {
            role: deque() for role in CAMERA_ROLES
        }
        self._images: dict[str, deque[_Image]] = {role: deque() for role in CAMERA_ROLES}
        self._images_by_header: dict[str, dict[int, _Image]] = {
            role: {} for role in CAMERA_ROLES
        }
        self._request_groups: dict[str, _RequestGroup] = {}
        self._used_camera_frames: set[tuple[str, int]] = set()
        self._state_by_session_sequence: dict[tuple[str, int], dict[str, Any]] = {}
        self._state_history_order: deque[tuple[str, int]] = deque()
        self._last_state_order: dict[str, tuple[int, int]] = {}
        self._sample_origin_ns: int | None = None
        self._last_sample_slot = -1
        self._frame_index = 0
        self._latest_arrival_ns = 0
        self._image_bytes = 0
        self._delivery_cutoff_ns: int | None = None
        self._counts: defaultdict[str, int] = defaultdict(int)

    def start_episode(self, run_id: str, task: str, *,
                      expected_session_id: str | None = None) -> None:
        """Start a fresh run and clear all prior frame/action associations."""
        if not isinstance(run_id, str) or not run_id.strip() or len(run_id) > 128:
            raise ValueError("run_id must be a non-empty string of at most 128 characters")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task must be a non-empty string")
        self._clear_episode_buffers()
        self._active_run_id = run_id
        self._task = task
        if expected_session_id is not None:
            self.expected_session_id = expected_session_id
        self._run_session_id = self.expected_session_id
        self._stop_tail_deadline_ns = None
        self._episode_ended = False

    def stop_episode(self, now_ns: int, *, tail_ns: int = 1_000_000_000) -> None:
        """Keep pairing the same run through a bounded post-stop tail."""
        if self._active_run_id is None:
            return
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        if type(tail_ns) is not int or tail_ns < 0:
            raise ValueError("tail_ns must be a non-negative integer")
        self._stop_tail_deadline_ns = now_ns + tail_ns + self.request_group_timeout_ns

    def add_state(self, state: Mapping[str, Any], arrival_ns: int) -> bool:
        """Add a state snapshot. Invalid sequence ordering is rejected."""
        if self._active_run_id is None:
            self._counts["input_ignored_outside_episode"] += 1
            return False
        if not _valid_arrival(arrival_ns):
            self._counts["state_bad_arrival"] += 1
            return False
        try:
            record = dict(state)
            session_id = _required_text(record, "session_id")
            sequence = _required_int(record, "sequence")
            source_ns = _required_int(record, "source_local_monotonic_ns")
            clock_id = _required_text(record, "source_local_clock_id")
            state_vector(record)
        except (ContractError, TypeError, ValueError) as exc:
            self._counts["state_rejected_invalid"] += 1
            self._last_error = str(exc)
            return False
        if sequence <= 0 or source_ns <= 0:
            self._counts["state_rejected_invalid"] += 1
            self._last_error = "state sequence and source timestamp must be positive"
            return False
        if self.expected_session_id and session_id != self.expected_session_id:
            self._counts["state_cross_session"] += 1
            return False
        if self._run_session_id is None:
            self._run_session_id = session_id
        elif session_id != self._run_session_id:
            self._counts["state_cross_session"] += 1
            return False
        previous = self._last_state_order.get(session_id)
        if previous is not None and (sequence <= previous[0] or source_ns <= previous[1]):
            self._counts["state_rejected_duplicate_or_out_of_order"] += 1
            return False
        if not clock_id:
            self._counts["state_rejected_invalid"] += 1
            return False
        self._last_state_order[session_id] = (sequence, source_ns)
        self._state_by_session_sequence[(session_id, sequence)] = record
        self._state_history_order.append((session_id, sequence))
        while len(self._state_history_order) > self.max_states * 2:
            old = self._state_history_order.popleft()
            self._state_by_session_sequence.pop(old, None)
        # Keep all 20 Hz feedback for command provenance, but only enqueue
        # one observation per 100 ms source-clock slot for the 10 Hz writer.
        # Small source jitter must not turn alternate 50 ms frames into a
        # drifting 150 ms cadence. Never duplicate/fill absent observations.
        self._counts['state_received'] += 1
        self._touch(arrival_ns)
        if self._sample_origin_ns is None:
            self._sample_origin_ns = source_ns
        slot = (source_ns-self._sample_origin_ns+self.period_ns//10)//self.period_ns
        if slot <= self._last_sample_slot:
            self._counts['state_skipped_sampling_rate'] += 1
            return False
        self._last_sample_slot = slot
        self._counts['state_selected_for_sampling'] += 1
        self._pending_states.append(_PendingState(record, arrival_ns))
        if len(self._pending_states) > self.max_states:
            self._pending_states.popleft()
            self._counts["state_dropped_buffer_full"] += 1
        return True

    def add_metadata(self, metadata: Mapping[str, Any], arrival_ns: int) -> bool:
        """Add one camera metadata record with its source/session identity."""
        if self._active_run_id is None:
            self._counts["input_ignored_outside_episode"] += 1
            return False
        if not _valid_arrival(arrival_ns):
            self._counts["metadata_bad_arrival"] += 1
            return False
        try:
            record = dict(metadata)
            role = _required_text(record, "role")
            if role not in CAMERA_ROLES:
                raise ContractError("metadata role must be base or wrist")
            _required_text(record, "session_id")
            _required_text(record, "source_local_clock_id")
            header_stamp_ns = _required_int(record, "header_stamp_ns")
            sequence = _required_int(record, "sequence")
            source_ns = _required_int(record, "source_local_monotonic_ns")
            uncertainty_ns = _required_int(record, "clock_mapping_uncertainty_ns")
            if header_stamp_ns <= 0 or sequence <= 0 or source_ns <= 0:
                raise ContractError("metadata timestamps and sequence must be positive")
            if uncertainty_ns < 0:
                raise ContractError("metadata clock uncertainty cannot be negative")
            if int(record.get("width", 0)) <= 0 or int(record.get("height", 0)) <= 0:
                raise ContractError("metadata width and height must be positive")
            if record.get("encoding") not in ("bgr8", "rgb8"):
                raise ContractError("metadata encoding must be bgr8 or rgb8")
            if self._run_session_id and record["session_id"] != self._run_session_id:
                self._counts["metadata_cross_session"] += 1
                return False
        except (ContractError, TypeError, ValueError) as exc:
            self._counts["metadata_rejected_invalid"] += 1
            self._last_error = str(exc)
            return False
        records = self._metadata[role]
        if (_camera_frame_key(role, record) in self._used_camera_frames or any(
            int(item.payload.get("header_stamp_ns", -1)) == record["header_stamp_ns"]
            and item.payload.get("session_id") == record.get("session_id")
            for item in records
        )):
            self._counts["metadata_duplicate_header"] += 1
            return False
        records.append(_Timed(record, arrival_ns))
        while len(records) > self.max_metadata_per_role:
            records.popleft()
            self._counts["metadata_dropped_buffer_full"] += 1
        self._counts[f"metadata_{role}_received"] += 1
        self._touch(arrival_ns)
        return True

    def add_image(self, role: str, image: Any, header_stamp_ns: int,
                  arrival_ns: int) -> bool:
        """Add a decoded or ROS Image; image buffers are copied and bounded."""
        if self._active_run_id is None:
            self._counts["input_ignored_outside_episode"] += 1
            return False
        if role not in CAMERA_ROLES or not _valid_arrival(arrival_ns):
            self._counts["image_rejected_invalid_identity"] += 1
            return False
        try:
            stamp = int(header_stamp_ns)
            if not _is_rgb_array(image) and isinstance(image, Mapping):
                if image.get("header_stamp_ns") is not None and int(
                        image["header_stamp_ns"]) != stamp:
                    raise ContractError("image and supplied header_stamp_ns differ")
            elif not _is_rgb_array(image):
                if image_header_stamp_ns(image) != stamp:
                    raise ContractError("image and supplied header_stamp_ns differ")
            source_encoding = _image_encoding(image)
            rgb = image_to_rgb(image) if not _is_rgb_array(image) else validate_rgb_image(image)
            frame_id = _image_frame_id(image)
        except (ContractError, TypeError, ValueError) as exc:
            self._counts[f"image_{role}_rejected_invalid"] += 1
            self._last_error = str(exc)
            return False
        if stamp < 0:
            self._counts[f"image_{role}_rejected_invalid"] += 1
            return False
        if (stamp in self._images_by_header[role]
                or _camera_frame_key(role, {"header_stamp_ns": stamp})
                in self._used_camera_frames):
            self._counts[f"image_{role}_duplicate_header"] += 1
            return False
        digest = hashlib.sha256(memoryview(rgb)).hexdigest()
        entry = _Image(rgb, stamp, frame_id, source_encoding, arrival_ns, digest)
        self._images[role].append(entry)
        self._images_by_header[role][stamp] = entry
        self._image_bytes += int(rgb.nbytes)
        self._counts[f"image_{role}_received"] += 1
        self._touch(arrival_ns)
        self._trim_images(role)
        while self._image_bytes > self.max_total_image_bytes:
            if not self._evict_oldest_image():
                break
            self._counts["image_dropped_memory_limit"] += 1
        return True

    def add_send_event(self, event: Mapping[str, Any], arrival_ns: int) -> bool:
        """Add actual transport events; preview-only events are ignored."""
        if self._active_run_id is None:
            self._counts["input_ignored_outside_episode"] += 1
            return False
        if not _valid_arrival(arrival_ns):
            self._counts["send_event_bad_arrival"] += 1
            return False
        record = dict(event)
        name = record.get("event")
        if name not in COMMAND_EVENT_NAMES:
            self._counts["non_send_event_ignored"] += 1
            return False
        request_id = record.get("request_id")
        if not isinstance(request_id, str) or not request_id:
            self._counts["send_event_missing_request_id"] += 1
            return False
        monotonic_ns = record.get("monotonic_ns")
        if type(monotonic_ns) is not int or monotonic_ns <= 0:
            self._counts["send_event_missing_monotonic_ns"] += 1
            return False
        group = self._request_groups.get(request_id)
        if group is None:
            if len(self._request_groups) >= self.max_request_groups:
                self._evict_oldest_group()
                self._counts["request_group_dropped_buffer_full"] += 1
            group = _RequestGroup([], [], arrival_ns)
            self._request_groups[request_id] = group
        elif group.finalized:
            self._counts["send_event_late_after_finalization"] += 1
            return False
        group.events.append(_Timed(record, arrival_ns))
        self._counts["send_event_received"] += 1
        self._touch(arrival_ns)
        return True

    def add_receipt(self, receipt: Mapping[str, Any], arrival_ns: int) -> bool:
        """Add a command receipt without waiting in the ROS callback."""
        if self._active_run_id is None:
            self._counts["input_ignored_outside_episode"] += 1
            return False
        if not _valid_arrival(arrival_ns):
            self._counts["receipt_bad_arrival"] += 1
            return False
        record = dict(receipt)
        request_id = record.get("request_id")
        if not isinstance(request_id, str) or not request_id:
            self._counts["receipt_missing_request_id"] += 1
            return False
        group = self._request_groups.get(request_id)
        if group is None:
            if len(self._request_groups) >= self.max_request_groups:
                self._evict_oldest_group()
                self._counts["request_group_dropped_buffer_full"] += 1
            group = _RequestGroup([], [], arrival_ns)
            self._request_groups[request_id] = group
        elif group.finalized:
            self._counts["receipt_late_after_finalization"] += 1
            return False
        group.receipts.append(_Timed(record, arrival_ns))
        self._counts["receipt_received"] += 1
        self._touch(arrival_ns)
        return True

    def poll(self, now_ns: int) -> list[dict[str, Any]]:
        """Return finalized rows whose bounded pairing window has elapsed."""
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        self._touch(now_ns)
        self._prune(now_ns)
        self._finalize_ready_request_groups(now_ns)
        output: list[dict[str, Any]] = []
        while self._pending_states:
            pending = self._pending_states[0]
            if now_ns - pending.arrival_ns < self.pair_wait_ns:
                break
            request_deadline_ns = self._pending_request_deadline_ns(pending.state)
            if request_deadline_ns is not None and now_ns < request_deadline_ns:
                self._counts["state_deferred_for_request_events"] += 1
                break
            self._pending_states.popleft()
            sample = self._assemble(pending, now_ns)
            if sample is None:
                continue
            output.append(sample)
            self._frame_index += 1
            self._counts["sample_emitted"] += 1
        if (self._active_run_id is not None
                and self._stop_tail_deadline_ns is not None
                and now_ns >= self._stop_tail_deadline_ns):
            self._active_run_id = None
            self._stop_tail_deadline_ns = None
            self._episode_ended = True
            self._counts["episode_tail_complete"] += 1
        return output

    def status(self) -> dict[str, Any]:
        """Return bounded buffer sizes, masks, and rejection counters."""
        return {
            "schema": "piper_live_pairing_status_v1",
            "run_id": self._active_run_id,
            "episode_active": self._active_run_id is not None,
            "episode_tail_complete": self._episode_ended,
            "sample_hz_nominal": self.sample_hz,
            "max_action_age_ms": self.max_action_age_ns // 1_000_000,
            "camera_match_ms": self.camera_match_ns // 1_000_000,
            "pair_wait_ms": self.pair_wait_ns // 1_000_000,
            "request_event_grace_ms": self.request_event_grace_ns // 1_000_000,
            "request_group_timeout_ms": self.request_group_timeout_ns // 1_000_000,
            "stop_tail_deadline_monotonic_ns": self._stop_tail_deadline_ns,
            "pending_states": len(self._pending_states),
            "metadata_cached": {role: len(self._metadata[role]) for role in CAMERA_ROLES},
            "images_cached": {role: len(self._images[role]) for role in CAMERA_ROLES},
            "image_bytes_cached": self._image_bytes,
            "request_groups_cached": len(self._request_groups),
            "delivery_incomplete_cutoff_monotonic_ns": self._delivery_cutoff_ns,
            "counts": dict(self._counts),
            "last_error": getattr(self, "_last_error", None),
        }

    def _assemble(self, pending: _PendingState, now_ns: int) -> dict[str, Any] | None:
        state = pending.state
        session_id = str(state.get("session_id", ""))
        try:
            state_values = state_vector(state)
        except ContractError:
            self._counts["sample_rejected_bad_state"] += 1
            return None
        state_valid = bool(
            state.get("feedback_fresh") is True
            and state.get("fake") is False
            and state.get("hardware_connected") is True
            and session_id
            and (not self._run_session_id or session_id == self._run_session_id)
            and (not self.expected_session_id or session_id == self.expected_session_id)
            and state.get("source_local_clock_id")
        )
        camera_entries: dict[str, _Timed | None] = {}
        images: dict[str, _Image | None] = {}
        camera_valid: dict[str, bool] = {}
        for role in CAMERA_ROLES:
            chosen = self._choose_camera(role, state, now_ns)
            camera_entries[role] = chosen
            image_entry = None
            if chosen is not None:
                image_entry = self._images_by_header[role].get(
                    int(chosen.payload.get("header_stamp_ns", -1)))
            images[role] = image_entry
            camera_valid[role] = bool(
                chosen is not None and image_entry is not None
                and self._metadata_matches_state(chosen.payload, state)
                and self._camera_delta_valid(chosen.payload, state)
                and image_entry.source_encoding == chosen.payload.get("encoding")
                and image_entry.rgb.shape[:2] == (
                    int(chosen.payload.get("height", -1)),
                    int(chosen.payload.get("width", -1)),
                )
            )
        if any(images[role] is None for role in CAMERA_ROLES):
            self._counts["sample_rejected_missing_image"] += 1
            # The selected frame belongs to this state attempt even when the
            # other view is missing. Consume it so a later row cannot fill
            # that gap with a repeated old frame.
            self._consume_selected_cameras(camera_entries)
            return None
        pair_delta_ns = None
        if all(camera_entries[role] is not None for role in CAMERA_ROLES):
            pair_delta_ns = abs(
                int(camera_entries["base"].payload["source_local_monotonic_ns"])
                - int(camera_entries["wrist"].payload["source_local_monotonic_ns"])
            )
        pair_uncertainty = 0
        if all(camera_entries[role] is not None for role in CAMERA_ROLES):
            pair_uncertainty = sum(
                int(camera_entries[role].payload.get("clock_mapping_uncertainty_ns", -1))
                for role in CAMERA_ROLES
            )
        pair_valid = (
            pair_delta_ns is not None and pair_uncertainty >= 0
            and pair_delta_ns + pair_uncertainty <= self.camera_match_ns
        )
        observation_valid = bool(state_valid and all(camera_valid.values()) and pair_valid)
        action, action_reason = self._action_for_state(state)
        # Action validity describes the actual send/receipt contract. Camera
        # quality is reported independently by observation_valid/camera_valid.
        action_valid = bool(action is not None and action_reason is None and state_valid)
        if not state_valid:
            action_valid = False
            action_reason = action_reason or "state_feedback_not_fresh_or_session_invalid"
        if action is None or not action_valid:
            action_values = np.zeros(7, dtype=np.float32)
        else:
            try:
                action_values = action_vector(raw_target_to_nominal_action(
                    action["joint_raw_0p001deg"], action["gripper_raw"]))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                action_values = np.zeros(7, dtype=np.float32)
                action_valid = False
                action_reason = f"classified_action_payload_invalid: {exc}"
                self._counts["action_payload_rejected"] += 1

        base_image = images["base"]
        wrist_image = images["wrist"]
        assert base_image is not None and wrist_image is not None
        source_map = {
            "schema": "piper_live_source_map_v1",
            "frame_index": self._frame_index,
            "run_id": self._active_run_id,
            "task": self._task,
            "state": _state_provenance(state, pending.arrival_ns),
            # Keep measured 20 Hz feedback near t+100ms, independently of
            # which state was selected for the next 10 Hz observation.
            "future_state_candidates": [
                _state_provenance(candidate, 0)
                for candidate in self._state_by_session_sequence.values()
                if candidate.get("session_id") == session_id
                and candidate.get("source_local_clock_id") == state.get("source_local_clock_id")
                and 75_000_000 <= (candidate["source_local_monotonic_ns"]
                                  - state["source_local_monotonic_ns"]) <= 125_000_000
            ],
            "images": {
                role: self._camera_provenance(
                    role, camera_entries[role], images[role], state
                )
                for role in CAMERA_ROLES
            },
            "camera_pair_delta_ns": pair_delta_ns,
            "camera_pair_limit_ns": self.camera_match_ns,
            "camera_pair_valid": pair_valid,
            "observation_valid": observation_valid,
            "observation_invalid_reasons": _observation_reasons(
                state, state_valid, camera_valid, pair_valid,
                camera_entries, images, self.expected_session_id,
            ),
            "action": self._action_provenance(action, action_reason, state),
            "arrival_clock": "adapter_process_monotonic_ns",
            "pairing_wait_ns": max(0, now_ns - pending.arrival_ns),
            "timestamp_note": "Source monotonic clocks are compared only when their IDs match; camera exposure synchronization is not established.",
        }
        self._consume_selected_cameras(camera_entries)
        sample = {
            "state": state_values,
            "action": action_values,
            "images": {
                "base": base_image.rgb,
                "wrist": wrist_image.rgb,
            },
            "action_valid": action_valid,
            "observation_valid": observation_valid,
            "camera_valid": [camera_valid["base"], camera_valid["wrist"]],
            "task": self._task,
            "run_id": self._active_run_id,
            "source_map": source_map,
        }
        self._counts["observation_valid"] += int(observation_valid)
        self._counts["action_valid"] += int(action_valid)
        self._counts["action_masked"] += int(not action_valid)
        self._counts["observation_masked"] += int(not observation_valid)
        return sample

    def _choose_camera(self, role: str, state: Mapping[str, Any],
                       now_ns: int) -> _Timed | None:
        state_session = str(state.get("session_id", ""))
        state_clock = str(state.get("source_local_clock_id", ""))
        state_ns = _int_or_none(state.get("source_local_monotonic_ns"))
        if not state_session or not state_clock or state_ns is None:
            return None
        candidates: list[tuple[int, int, _Timed]] = []
        for entry in self._metadata[role]:
            meta = entry.payload
            if entry.arrival_ns > now_ns:
                continue
            if not self._metadata_matches_state(meta, state):
                continue
            frame_key = _camera_frame_key(role, meta)
            if frame_key in self._used_camera_frames:
                continue
            header_ns = _int_or_none(meta.get("header_stamp_ns"))
            source_ns = _int_or_none(meta.get("source_local_monotonic_ns"))
            if header_ns is None or source_ns is None:
                continue
            if header_ns not in self._images_by_header[role]:
                continue
            image = self._images_by_header[role][header_ns]
            if image.arrival_ns > now_ns:
                continue
            delta = abs(source_ns - state_ns)
            candidates.append((delta, int(meta.get("sequence", 0)), entry))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]

    def _metadata_matches_state(self, metadata: Mapping[str, Any],
                                state: Mapping[str, Any]) -> bool:
        if metadata.get("role") not in CAMERA_ROLES:
            return False
        if metadata.get("session_id") != state.get("session_id"):
            return False
        if metadata.get("source_local_clock_id") != state.get("source_local_clock_id"):
            return False
        return True

    def _camera_delta_valid(self, metadata: Mapping[str, Any],
                            state: Mapping[str, Any]) -> bool:
        image_ns = _int_or_none(metadata.get("source_local_monotonic_ns"))
        state_ns = _int_or_none(state.get("source_local_monotonic_ns"))
        image_uncertainty = _int_or_none(metadata.get("clock_mapping_uncertainty_ns"))
        state_uncertainty = _int_or_none(state.get("source_clock_mapping_uncertainty_ns"))
        if None in (image_ns, state_ns, image_uncertainty, state_uncertainty):
            return False
        if image_uncertainty < 0 or state_uncertainty < 0:
            return False
        return abs(image_ns - state_ns) + image_uncertainty + state_uncertainty <= self.camera_match_ns

    def _action_for_state(self, state: Mapping[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
        session_id = str(state.get("session_id", ""))
        sequence = _int_or_none(state.get("sequence"))
        if sequence is None:
            return None, "source_state_sequence_missing"
        candidates: list[tuple[str, _RequestGroup]] = []
        for request_id, group in self._request_groups.items():
            intents = [item.payload for item in group.events
                       if item.payload.get("event") == "TARGET_INTENT"]
            if any(
                intent.get("source_session_id") == session_id
                and intent.get("source_state_sequence") == sequence
                for intent in intents
            ):
                candidates.append((request_id, group))
        if len(candidates) != 1:
            if len(candidates) > 1:
                return None, "multiple_requests_share_source_state_sequence"
            return None, "no_unique_complete_kernel_accepted_target_for_state"
        request_id, group = candidates[0]
        if not group.finalized or not group.result:
            return None, "command_receipt_or_send_batch_pending"
        result = group.result
        request = next((item for item in result.get("requests", [])
                        if item.get("request_id") == request_id), None)
        if request is None:
            return None, "command_group_classification_missing"
        if self._delivery_cutoff_ns is not None and int(
                request.get("first_event_monotonic_ns", 0)) >= self._delivery_cutoff_ns:
            return request, "after_delivery_incomplete_cutoff"
        if request.get("action_valid") is not True:
            reasons = request.get("reasons") or []
            return request, ";".join(str(reason) for reason in reasons) or "action_invalid"
        return request, None

    def _pending_request_deadline_ns(
        self, state: Mapping[str, Any]
    ) -> int | None:
        """Bound row emission behind matching or earlier in-flight requests.

        A pending request can still establish a delivery-incomplete cutoff for
        this state or any later state in the same session. Hold those rows
        until the request group completes or its finite timeout expires. A
        receipt sequence is only a wait hint; action validity still requires
        the classified target intent, complete send batch, service result, and
        receipt.
        """
        session_id = state.get("session_id")
        sequence = _int_or_none(state.get("sequence"))
        if not isinstance(session_id, str) or sequence is None:
            return None
        deadlines: list[int] = []
        for group in self._request_groups.values():
            if group.finalized:
                continue
            may_affect_state = False
            for item in group.events:
                event = item.payload
                if (event.get("source_session_id") == session_id
                        and type(event.get("source_state_sequence")) is int
                        and event["source_state_sequence"] <= sequence):
                    may_affect_state = True
                    break
            if not may_affect_state:
                # Receipts commonly carry the source state sequence before the
                # send-event topic delivers its TARGET_INTENT. Use that value
                # only to defer rows, never to create an action label.
                may_affect_state = any(
                    type(receipt.payload.get("sequence")) is int
                    and receipt.payload["sequence"] <= sequence
                    for receipt in group.receipts
                )
            if may_affect_state:
                deadlines.append(group.first_arrival_ns + self.request_group_timeout_ns)
        return max(deadlines) if deadlines else None

    def _finalize_ready_request_groups(self, now_ns: int) -> None:
        for request_id, group in list(self._request_groups.items()):
            if group.finalized:
                continue
            if not group.events:
                # A receipt without source send evidence is not enough to
                # infer a target or delivery state.
                if now_ns - group.first_arrival_ns >= self.request_group_timeout_ns:
                    group.finalized = True
                    group.result = {"requests": [], "incomplete_request_count": 1}
                continue
            first = min(group.events, key=lambda item: int(item.payload.get("monotonic_ns", 0)))
            intent = next((item.payload for item in group.events
                           if item.payload.get("event") == "TARGET_INTENT"), None)
            state_map: dict[int, dict[str, Any]] = {}
            if intent is not None:
                seq = intent.get("source_state_sequence")
                session = intent.get("source_session_id")
                if type(seq) is int and isinstance(session, str):
                    state = self._state_by_session_sequence.get((session, seq))
                    if state is not None:
                        state_map[seq] = state
            expected = (str(intent.get("source_session_id")) if intent is not None
                        and isinstance(intent.get("source_session_id"), str)
                        else self.expected_session_id or "")
            records = [item.payload for item in group.events]
            receipts = [item.payload for item in group.receipts]
            try:
                classified = group_command_events(
                    records, receipts, state_map,
                    expected_session_id=expected,
                    max_action_age_ns=self.max_action_age_ns,
                )
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                group.finalized = True
                group.result = {"requests": [], "classification_error": str(exc)}
                self._counts["request_group_classification_error"] += 1
                if any(item.payload.get("event") in {
                    "CAN_WRITE_INTENT", "CAN_KERNEL_ACCEPTED", "CAN_DELIVERY_UNKNOWN"
                } for item in group.events):
                    first_ns = min(
                        int(item.payload.get("monotonic_ns", 0))
                        for item in group.events
                    )
                    self._delivery_cutoff_ns = (
                        first_ns if self._delivery_cutoff_ns is None
                        else min(first_ns, self._delivery_cutoff_ns)
                    )
                    self._counts["delivery_incomplete"] += 1
                continue
            request = next((item for item in classified["requests"]
                            if item.get("request_id") == request_id), None)
            has_unknown = any(item.payload.get("event") == "CAN_DELIVERY_UNKNOWN"
                              for item in group.events)
            complete = bool(request and request.get("complete") is True)
            timed_out = now_ns - group.first_arrival_ns >= self.request_group_timeout_ns
            if not complete and not has_unknown and not timed_out:
                continue
            group.finalized = True
            group.result = classified
            self._counts["request_group_finalized"] += 1
            if request and request.get("delivery_incomplete"):
                cutoff = int(request.get("first_event_monotonic_ns", first.arrival_ns))
                self._delivery_cutoff_ns = (
                    cutoff if self._delivery_cutoff_ns is None
                    else min(self._delivery_cutoff_ns, cutoff)
                )
                self._counts["delivery_incomplete"] += 1

    def _action_provenance(self, action: dict[str, Any] | None,
                           invalid_reason: str | None,
                           state: Mapping[str, Any]) -> dict[str, Any]:
        if action is None:
            request_id = None
            event_records: list[dict[str, Any]] = []
            receipt_records: list[dict[str, Any]] = []
        else:
            request_id = action.get("request_id")
            group = self._request_groups.get(str(request_id))
            event_records = [dict(item.payload) for item in group.events] if group else []
            receipt_records = [dict(item.payload) for item in group.receipts] if group else []
        return {
            "request_id": request_id,
            "source_session_id": state.get("session_id"),
            "source_state_sequence": state.get("sequence"),
            "action_valid": bool(action is not None and invalid_reason is None),
            "invalid_reason": invalid_reason,
            "delivery_incomplete_cutoff_monotonic_ns": self._delivery_cutoff_ns,
            "classified_request": action,
            "send_events": event_records,
            "receipts": receipt_records,
            "semantics": "five matching CAN_KERNEL_ACCEPTED calls, accepted service result, one adapter-accepted five-call receipt; no physical execution claim",
        }

    def _camera_provenance(self, role: str, metadata: _Timed | None,
                           image: _Image | None,
                           state: Mapping[str, Any]) -> dict[str, Any]:
        if metadata is None:
            return {"role": role, "valid": False, "reason": "camera_metadata_missing"}
        value = metadata.payload
        state_ns = _int_or_none(state.get("source_local_monotonic_ns"))
        frame_ns = _int_or_none(value.get("source_local_monotonic_ns"))
        delta_ns = None if state_ns is None or frame_ns is None else frame_ns - state_ns
        return {
            "role": role,
            "session_id": value.get("session_id"),
            "device_name": value.get("device_name"),
            "sequence": value.get("sequence"),
            "header_stamp_ns": value.get("header_stamp_ns"),
            "header_frame_id": image.header_frame_id if image else None,
            "source_monotonic_ns": value.get("source_monotonic_ns"),
            "source_clock_id": value.get("source_clock_id"),
            "source_local_monotonic_ns": value.get("source_local_monotonic_ns"),
            "source_local_clock_id": value.get("source_local_clock_id"),
            "clock_mapping_uncertainty_ns": value.get("clock_mapping_uncertainty_ns"),
            "arrival_monotonic_ns": metadata.arrival_ns,
            "image_arrival_monotonic_ns": image.arrival_ns if image else None,
            "state_delta_ns": delta_ns,
            "state_delta_abs_ns": abs(delta_ns) if delta_ns is not None else None,
            "rgb_pixels_sha256": image.sha256 if image else None,
            "width": value.get("width"),
            "height": value.get("height"),
            "encoding": value.get("encoding"),
            "valid": bool(
                image is not None
                and self._metadata_matches_state(value, state)
                and self._camera_delta_valid(value, state)
                and image.source_encoding == value.get("encoding")
                and image.rgb.shape[:2] == (
                    int(value.get("height", -1)), int(value.get("width", -1))
                )
            ),
        }

    def _consume_selected_cameras(
        self, camera_entries: Mapping[str, _Timed | None]
    ) -> None:
        for role in CAMERA_ROLES:
            entry = camera_entries[role]
            if entry is None:
                continue
            self._used_camera_frames.add(_camera_frame_key(role, entry.payload))
            self._remove_image(role, int(entry.payload["header_stamp_ns"]))
            self._remove_metadata(role, entry)

    def _clear_episode_buffers(self) -> None:
        self._pending_states.clear()
        for role in CAMERA_ROLES:
            self._metadata[role].clear()
            self._images[role].clear()
            self._images_by_header[role].clear()
        self._request_groups.clear()
        self._used_camera_frames.clear()
        self._state_by_session_sequence.clear()
        self._state_history_order.clear()
        self._last_state_order.clear()
        self._sample_origin_ns = None
        self._last_sample_slot = -1
        self._run_session_id = self.expected_session_id
        self._image_bytes = 0
        self._delivery_cutoff_ns = None
        self._frame_index = 0

    def _touch(self, arrival_ns: int) -> None:
        self._latest_arrival_ns = max(self._latest_arrival_ns, arrival_ns)

    def _prune(self, now_ns: int) -> None:
        threshold = now_ns - self.cache_age_ns
        for role in CAMERA_ROLES:
            while self._metadata[role] and self._metadata[role][0].arrival_ns < threshold:
                self._metadata[role].popleft()
                self._counts[f"metadata_{role}_expired"] += 1
            while self._images[role] and self._images[role][0].arrival_ns < threshold:
                old = self._images[role].popleft()
                self._images_by_header[role].pop(old.header_stamp_ns, None)
                self._image_bytes -= int(old.rgb.nbytes)
                self._counts[f"image_{role}_expired"] += 1
        for request_id, group in list(self._request_groups.items()):
            if group.finalized and now_ns - group.first_arrival_ns > self.cache_age_ns:
                self._request_groups.pop(request_id, None)

    def _trim_images(self, role: str) -> None:
        while len(self._images[role]) > self.max_images_per_role:
            self._evict_image(role)
            self._counts[f"image_{role}_dropped_count_limit"] += 1

    def _evict_image(self, role: str) -> bool:
        if not self._images[role]:
            return False
        entry = self._images[role].popleft()
        self._images_by_header[role].pop(entry.header_stamp_ns, None)
        self._image_bytes -= int(entry.rgb.nbytes)
        return True

    def _evict_oldest_image(self) -> bool:
        choices = [role for role in CAMERA_ROLES if self._images[role]]
        if not choices:
            return False
        role = min(choices, key=lambda item: self._images[item][0].arrival_ns)
        return self._evict_image(role)

    def _remove_image(self, role: str, header_stamp_ns: int) -> None:
        image = self._images_by_header[role].pop(header_stamp_ns, None)
        if image is None:
            return
        try:
            self._images[role].remove(image)
        except ValueError:
            pass
        self._image_bytes -= int(image.rgb.nbytes)

    def _remove_metadata(self, role: str, chosen: _Timed) -> None:
        try:
            self._metadata[role].remove(chosen)
        except ValueError:
            pass

    def _evict_oldest_group(self) -> None:
        if not self._request_groups:
            return
        request_id = min(
            self._request_groups,
            key=lambda key: self._request_groups[key].first_arrival_ns,
        )
        self._request_groups.pop(request_id, None)


def replay_rosbag(
    manifest_path: Path,
    bag_root: Path | None = None,
    *,
    max_action_age_ms: int = DEFAULT_MAX_ACTION_AGE_MS,
    camera_match_ms: int = DEFAULT_CAMERA_MATCH_MS,
    pair_wait_ms: int = DEFAULT_PAIR_WAIT_MS,
    request_event_grace_ms: int = DEFAULT_REQUEST_EVENT_GRACE_MS,
) -> Iterator[dict[str, Any]]:
    """Stream one immutable historical bag through the live assembler.

    The SQLite reader is opened read-only. Messages from six existing topics
    are merged by bag timestamp; image payloads are decoded one at a time and
    bounded by the same buffers used in live capture.
    """
    from gamepad_capture.tools.dataset.bag import (
        resolve_manifest_bag_root,
        verify_source,
    )

    manifest_path = Path(manifest_path)
    resolved = resolve_manifest_bag_root(manifest_path, bag_root)
    source = verify_source(manifest_path, resolved, verify_hashes=True)
    manifest = source["manifest"]
    run_id = str(manifest["run_id"])
    task = str(manifest.get("task", ""))
    session_id = str(manifest.get("session_id", ""))
    if not task or not session_id:
        raise ValueError("manifest requires non-empty task and session_id")
    reader = RosbagReader(source["db_path"])
    topics = {
        "/piper/state_snapshot": "state",
        "/piper/cameras/frame_metadata": "metadata",
        "/piper/cameras/base/image_raw": "base",
        "/piper/cameras/wrist/image_raw": "wrist",
        "/gamepad/command_send_events": "send_event",
        "/piper/command_receipt": "receipt",
    }
    missing = [topic for topic in topics if topic not in reader.topic_ids]
    if missing:
        reader.close()
        raise ValueError(f"historical bag lacks required topics: {missing}")
    assembler = BoundedObservationAssembler(
        max_action_age_ms=max_action_age_ms,
        camera_match_ms=camera_match_ms,
        pair_wait_ms=pair_wait_ms,
        request_event_grace_ms=request_event_grace_ms,
        expected_session_id=session_id,
    )
    assembler.start_episode(run_id, task, expected_session_id=session_id)
    try:
        merged = _merge_topic_streams(reader, topics)
        first_bag_ns: int | None = None
        last_arrival_ns = 0
        for bag_ns, topic, data in merged:
            if first_bag_ns is None:
                first_bag_ns = bag_ns
            arrival_ns = max(1, bag_ns - first_bag_ns + 1)
            last_arrival_ns = max(last_arrival_ns, arrival_ns)
            kind = topics[topic]
            if kind == "state":
                assembler.add_state(decode_state_snapshot(data), arrival_ns)
            elif kind == "metadata":
                assembler.add_metadata(decode_camera_metadata(data), arrival_ns)
            elif kind in CAMERA_ROLES:
                header = decode_image_header(data)
                _, decoded = decode_image(data)
                decoded["header_stamp_ns"] = header.stamp_ns
                decoded["header_frame_id"] = header.frame_id
                assembler.add_image(kind, decoded, header.stamp_ns, arrival_ns)
            elif kind == "send_event":
                event = json.loads(decode_std_string(data))
                if isinstance(event, dict):
                    assembler.add_send_event(event, arrival_ns)
            elif kind == "receipt":
                assembler.add_receipt(decode_command_receipt_bytes(data), arrival_ns)
            yield from assembler.poll(arrival_ns)
        if last_arrival_ns:
            yield from assembler.poll(
                last_arrival_ns
                + (pair_wait_ms + request_event_grace_ms) * 1_000_000
            )
    finally:
        reader.close()


def _merge_topic_streams(reader: RosbagReader,
                         topics: Mapping[str, str]) -> Iterator[tuple[int, str, bytes]]:
    """Merge lazily ordered topic cursors without loading message payloads."""
    iterators: list[Iterator[tuple[int, str, bytes]]] = []
    for topic in topics:
        source = reader.iter_topic(topic)
        iterators.append(_tag_topic(topic, source))
    yield from heapq.merge(*iterators, key=lambda item: (item[0], item[1]))


def _tag_topic(topic: str, source: Iterable[tuple[int, bytes]]) -> Iterator[tuple[int, str, bytes]]:
    for timestamp, data in source:
        yield timestamp, topic, data


def _required_text(record: Mapping[str, Any], key: str) -> str:
    value = record.get(key)
    if not isinstance(value, str) or not value:
        raise ContractError(f"{key} must be a non-empty string")
    return value


def _required_int(record: Mapping[str, Any], key: str) -> int:
    value = record.get(key)
    if type(value) is not int:
        raise ContractError(f"{key} must be an integer")
    return value


def _int_or_none(value: Any) -> int | None:
    return int(value) if type(value) is int else None


def _valid_arrival(value: Any) -> bool:
    return type(value) is int and value >= 0


def _is_rgb_array(value: Any) -> bool:
    return isinstance(value, np.ndarray)


def _image_frame_id(image: Any) -> str:
    header = getattr(image, "header", None)
    if header is not None:
        return str(getattr(header, "frame_id", ""))
    if isinstance(image, Mapping):
        return str(image.get("header_frame_id", ""))
    return ""


def _image_encoding(image: Any) -> str:
    if isinstance(image, Mapping):
        return str(image.get("encoding", "rgb8" if _is_rgb_array(image) else ""))
    return str(getattr(image, "encoding", "rgb8" if _is_rgb_array(image) else ""))


def _camera_frame_key(role: str, metadata: Mapping[str, Any]) -> tuple[str, int]:
    """Identify a camera frame by its per-role header stamp, not metadata sequence."""
    return role, int(metadata.get("header_stamp_ns", -1))


def _state_provenance(state: Mapping[str, Any], arrival_ns: int) -> dict[str, Any]:
    return {
        "session_id": state.get("session_id"),
        "sequence": state.get("sequence"),
        "header_stamp_ns": state.get("header_stamp_ns"),
        "header_frame_id": state.get("header_frame_id"),
        "source_monotonic_ns": state.get("source_monotonic_ns"),
        "source_clock_id": state.get("source_clock_id"),
        "source_local_monotonic_ns": state.get("source_local_monotonic_ns"),
        "source_local_clock_id": state.get("source_local_clock_id"),
        "source_clock_mapping_uncertainty_ns": state.get("source_clock_mapping_uncertainty_ns"),
        "feedback_fresh": state.get("feedback_fresh"),
        "hardware_connected": state.get("hardware_connected"),
        "fake": state.get("fake"),
        "arrival_monotonic_ns": arrival_ns,
        "values_float32": state_vector(state).tolist(),
    }


def _observation_reasons(
    state: Mapping[str, Any],
    state_valid: bool,
    camera_valid: Mapping[str, bool],
    pair_valid: bool,
    camera_entries: Mapping[str, _Timed | None],
    images: Mapping[str, _Image | None],
    expected_session_id: str | None,
) -> list[str]:
    reasons: list[str] = []
    if state.get("feedback_fresh") is not True:
        reasons.append("state_feedback_not_fresh")
    if state.get("fake") is not False:
        reasons.append("state_marked_fake_or_unknown")
    if state.get("hardware_connected") is False:
        reasons.append("state_hardware_disconnected")
    if expected_session_id and state.get("session_id") != expected_session_id:
        reasons.append("state_session_mismatch")
    if not state_valid:
        reasons.append("state_invalid")
    for role in CAMERA_ROLES:
        meta = camera_entries[role]
        if meta is None:
            reasons.append(f"{role}_metadata_missing_or_clock_mismatch")
        elif images[role] is None:
            reasons.append(f"{role}_image_missing")
        elif not camera_valid[role]:
            reasons.append(f"{role}_state_delta_or_clock_uncertainty_exceeded")
    if not pair_valid:
        reasons.append("camera_pair_skew_exceeded_or_missing")
    return reasons
