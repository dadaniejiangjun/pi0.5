"""Pure rolling queue for already-mapped position-command targets.

This core only manages timing and replacement of typed command targets. It
does not map policy feedback proxies, authorize a robot, publish ROS commands,
or access a transport. The Windows SafetyBridge remains the final gate.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers


UPDATE_PERIOD_NS = 100_000_000
ACTION_COLUMNS = 7
ACTION_SEMANTICS = "absolute_joint_position_command"
ACTION_UNITS = ("rad", "rad", "rad", "rad", "rad", "rad", "m")
MAX_TASK_DURATION_NS = 120_000_000_000


class RollingQueueStopped(RuntimeError):
    """The task queue stopped after invalid or expired work."""


@dataclass(frozen=True)
class MappedCommandTarget:
    request_id: str
    target_time_monotonic_ns: int
    target_position: tuple[float, ...]
    target_expiry_monotonic_ns: int | None = None

    def __post_init__(self) -> None:
        if type(self.target_time_monotonic_ns) is not int or self.target_time_monotonic_ns <= 0:
            raise ValueError("target_time_monotonic_ns must be a positive integer")
        derived_expiry = self.target_time_monotonic_ns + UPDATE_PERIOD_NS
        if self.target_expiry_monotonic_ns is None:
            object.__setattr__(self, "target_expiry_monotonic_ns", derived_expiry)
        elif (type(self.target_expiry_monotonic_ns) is not int
              or self.target_expiry_monotonic_ns != derived_expiry):
            raise ValueError("action expiry must equal original target time plus one 10 Hz slot")


@dataclass(frozen=True)
class MappedCommandChunk:
    sequence: int
    task_scope_id: str
    model_profile: str
    checkpoint_id: str
    source_observation_id: str
    source_state_sequence: int
    action_semantics: str
    units: tuple[str, ...]
    hardware_units_verified: bool
    mapping_verified: bool
    targets: tuple[MappedCommandTarget, ...]


class RollingCommandQueue:
    """Keep only the latest uncommitted command targets for one task.

    ``take_due`` removes and returns a target, which marks it committed to the
    caller. A later chunk can replace all remaining uncommitted targets but
    cannot retract a target already returned. Stale targets are dropped.
    """

    def __init__(self) -> None:
        self._task_scope_id = ""
        self._model_profile = ""
        self._checkpoint_id = ""
        self._started_ns = 0
        self._deadline_ns = 0
        self._last_chunk_sequence = 0
        self._last_state_sequence = 0
        self._seen_observations: set[str] = set()
        self._seen_requests: set[str] = set()
        self._pending: list[MappedCommandTarget] = []
        self._stopped_reason = ""
        self.replaced_count = 0
        self.expired_count = 0
        self.committed_count = 0

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def stopped_reason(self) -> str:
        return self._stopped_reason

    def start_task(
        self,
        *,
        task_scope_id: str,
        model_profile: str,
        checkpoint_id: str,
        started_monotonic_ns: int,
        max_duration_s: int = 120,
    ) -> None:
        if self._started_ns:
            raise RollingQueueStopped("queue task can be started only once")
        for value, name in (
            (task_scope_id, "task_scope_id"),
            (model_profile, "model_profile"),
            (checkpoint_id, "checkpoint_id"),
        ):
            if not isinstance(value, str) or not value.isascii() or not value.strip() or len(value) > 256:
                raise ValueError(f"{name} must be non-empty bounded ASCII text")
        if type(started_monotonic_ns) is not int or started_monotonic_ns <= 0:
            raise ValueError("started_monotonic_ns must be a positive integer")
        if type(max_duration_s) is not int or not 1 <= max_duration_s <= 120:
            raise ValueError("max_duration_s must be within 1..120 seconds")
        self._task_scope_id = task_scope_id
        self._model_profile = model_profile
        self._checkpoint_id = checkpoint_id
        self._started_ns = started_monotonic_ns
        self._deadline_ns = started_monotonic_ns + max_duration_s * 1_000_000_000

    def replace_future(self, chunk: MappedCommandChunk, *, now_monotonic_ns: int) -> int:
        """Replace queued targets with a newer, validated mapped chunk.

        Returns the number of uncommitted targets superseded. Any contract
        violation latches a stop and clears the future queue.
        """
        self._require_running(now_monotonic_ns)
        try:
            validated, stale_rows, all_request_ids = self._validate_chunk(
                chunk, now_monotonic_ns=now_monotonic_ns
            )
        except (TypeError, ValueError) as exc:
            self.stop(f"chunk_rejected:{exc}")
            raise RollingQueueStopped(self._stopped_reason) from exc

        superseded = len(self._pending)
        self._pending = list(validated)
        self.replaced_count += superseded
        self.expired_count += stale_rows
        self._last_chunk_sequence = chunk.sequence
        self._last_state_sequence = chunk.source_state_sequence
        self._seen_observations.add(chunk.source_observation_id)
        self._seen_requests.update(all_request_ids)
        return superseded

    def take_due(self, *, now_monotonic_ns: int) -> MappedCommandTarget | None:
        """Return the next due target, dropping targets that missed their window."""
        self._require_running(now_monotonic_ns)
        while self._pending:
            target = self._pending[0]
            if now_monotonic_ns >= target.target_expiry_monotonic_ns:
                self._pending.pop(0)
                self.expired_count += 1
                continue
            if target.target_time_monotonic_ns > now_monotonic_ns:
                return None
            self._pending.pop(0)
            self.committed_count += 1
            return target
        return None

    def stop(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("stop reason must be non-empty text")
        self._pending.clear()
        if not self._stopped_reason:
            self._stopped_reason = reason[:256]

    def _require_running(self, now_monotonic_ns: int) -> None:
        if not self._started_ns:
            raise RollingQueueStopped("queue task has not started")
        if self._stopped_reason:
            raise RollingQueueStopped(self._stopped_reason)
        if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
            self.stop("invalid_scheduler_time")
            raise RollingQueueStopped(self._stopped_reason)
        if now_monotonic_ns < self._started_ns:
            self.stop("scheduler_time_precedes_task_start")
            raise RollingQueueStopped(self._stopped_reason)
        if now_monotonic_ns >= self._deadline_ns:
            self.stop("task_deadline_reached")
            raise RollingQueueStopped(self._stopped_reason)

    def _validate_chunk(
        self,
        chunk: MappedCommandChunk,
        *,
        now_monotonic_ns: int,
    ) -> tuple[tuple[MappedCommandTarget, ...], int, tuple[str, ...]]:
        if type(chunk) is not MappedCommandChunk:
            raise TypeError("typed MappedCommandChunk is required")
        if type(chunk.sequence) is not int or chunk.sequence <= self._last_chunk_sequence:
            raise ValueError("chunk sequence must increase")
        if chunk.task_scope_id != self._task_scope_id:
            raise ValueError("chunk task scope does not match")
        if chunk.model_profile != self._model_profile or chunk.checkpoint_id != self._checkpoint_id:
            raise ValueError("chunk model identity does not match the task")
        if (not isinstance(chunk.source_observation_id, str)
                or not chunk.source_observation_id.isascii()
                or not chunk.source_observation_id
                or len(chunk.source_observation_id) > 256):
            raise ValueError("chunk observation ID must be bounded ASCII text")
        if chunk.source_observation_id in self._seen_observations:
            raise ValueError("chunk observation ID has already been used")
        if (type(chunk.source_state_sequence) is not int
                or chunk.source_state_sequence <= self._last_state_sequence):
            raise ValueError("source state sequence must increase")
        if chunk.action_semantics != ACTION_SEMANTICS:
            raise ValueError("chunk must contain mapped absolute position commands, not policy proxies")
        if tuple(chunk.units) != ACTION_UNITS:
            raise ValueError("mapped command units must be six radians and one meter")
        if chunk.hardware_units_verified is not True or chunk.mapping_verified is not True:
            raise ValueError("hardware units and command mapping must be verified")
        targets = tuple(chunk.targets)
        if not 1 <= len(targets) <= 16:
            raise ValueError("a rolling chunk must contain 1..16 targets")
        if any(type(target) is not MappedCommandTarget for target in targets):
            raise TypeError("chunk rows must be typed MappedCommandTarget values")
        request_ids: list[str] = []
        times: list[int] = []
        for target in targets:
            if (not isinstance(target.request_id, str) or not target.request_id.isascii()
                    or not target.request_id.strip() or len(target.request_id) > 128):
                raise ValueError("target request IDs must be bounded ASCII text")
            if target.request_id in self._seen_requests or target.request_id in request_ids:
                raise ValueError("target request ID has already been used")
            position = tuple(target.target_position)
            if len(position) != ACTION_COLUMNS:
                raise ValueError("target must contain six joints and one gripper value")
            for value in position:
                if (isinstance(value, bool) or not isinstance(value, numbers.Real)
                        or not math.isfinite(float(value))):
                    raise ValueError("target values must be finite numbers")
            request_ids.append(target.request_id)
            times.append(target.target_time_monotonic_ns)
        if any(right - left != UPDATE_PERIOD_NS for left, right in zip(times, times[1:])):
            raise ValueError("target rows must be spaced at 100 ms")
        if times[-1] - now_monotonic_ns > self._deadline_ns - now_monotonic_ns:
            raise ValueError("target time exceeds the task deadline")
        fresh = tuple(
            target for target in targets
            if now_monotonic_ns < target.target_expiry_monotonic_ns
        )
        if not fresh:
            raise ValueError("all targets in the new chunk are already expired")
        return fresh, len(targets) - len(fresh), tuple(request_ids)
