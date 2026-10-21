"""Bounded in-process task requests for the Windows PiperSession owner thread.

This queue is not a network protocol or an authorization issuer. The current
Windows gateway does not expose it to TCP clients and continues to reject every
wire command. Callers must provide typed requests created inside the trusted
Windows process; PiperSession and SafetyBridge still validate each operation.
"""
from __future__ import annotations

from concurrent.futures import Future
from dataclasses import dataclass
import queue
import threading
import time
from typing import Any, Callable

from laptop.safety.task_scope import TaskControlSnapshot, TaskScopedPositionAuthorization
from laptop.transport.position_commands import (
    MAX_POSITION_APPROVAL_WINDOW_NS,
    PositionCommandBatch,
)


class OwnerQueueError(RuntimeError):
    """A request cannot be queued or executed by the session owner."""


class OwnerQueueFull(OwnerQueueError):
    """The one pending request slot is already occupied."""


@dataclass(frozen=True)
class _Request:
    request_id: str
    operation: str
    task_scope_id: str
    valid_until_monotonic_ns: int | None
    authorization: TaskScopedPositionAuthorization | None = None
    snapshot: TaskControlSnapshot | None = None
    batch: PositionCommandBatch | None = None
    target_factory: Callable[[Any], tuple[TaskControlSnapshot, PositionCommandBatch]] | None = None
    future: Future | None = None
    manual_target: Any = None


class SessionOwnerTaskQueue:
    """Run typed begin/target/end calls only from one claimed owner thread.

    One pending request is retained at a time. Requests expire before execution,
    identifiers cannot be reused, and exceptions abort the local task scope.
    Aborting revokes future software permits; it is not a physical robot stop.
    """

    def __init__(self) -> None:
        self._queue: queue.Queue[_Request] = queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self._owner_ident: int | None = None
        self._closed = False
        self._used_request_ids: set[str] = set()
        self._active_task_scope_id = ""

    @property
    def active_task_scope_id(self) -> str:
        with self._lock:
            return self._active_task_scope_id

    def claim_owner_thread(self) -> None:
        """Bind queue pumping to the calling thread exactly once."""
        ident = threading.get_ident()
        with self._lock:
            if self._owner_ident is not None and self._owner_ident != ident:
                raise OwnerQueueError("session owner thread is already claimed")
            if self._closed:
                raise OwnerQueueError("session owner queue is closed")
            self._owner_ident = ident

    def submit_begin(
        self, *, request_id: str,
        authorization: TaskScopedPositionAuthorization,
    ) -> Future:
        if type(authorization) is not TaskScopedPositionAuthorization:
            raise TypeError("begin request requires a typed in-process task authorization")
        return self._submit(_Request(
            request_id=request_id,
            operation="begin",
            task_scope_id=authorization.task_scope_id,
            valid_until_monotonic_ns=authorization.valid_until_monotonic_ns,
            authorization=authorization,
            future=Future(),
        ))

    def submit_target(
        self, *, request_id: str, task_scope_id: str,
        snapshot: TaskControlSnapshot, batch: PositionCommandBatch,
    ) -> Future:
        if type(snapshot) is not TaskControlSnapshot:
            raise TypeError("target request requires a typed task feedback snapshot")
        if type(batch) is not PositionCommandBatch:
            raise TypeError("target request requires a typed position command batch")
        proposal = batch.proposal
        if proposal.task_scope_id != task_scope_id:
            raise ValueError("target task scope does not match its position proposal")
        return self._submit(_Request(
            request_id=request_id,
            operation="target",
            task_scope_id=task_scope_id,
            valid_until_monotonic_ns=proposal.valid_until_monotonic_ns,
            snapshot=snapshot,
            batch=batch,
            future=Future(),
        ))

    def submit_target_factory(
        self, *, request_id: str, task_scope_id: str,
        valid_until_monotonic_ns: int,
        factory: Callable[[Any], tuple[TaskControlSnapshot, PositionCommandBatch]],
    ) -> Future:
        """Run a locally configured target mapper on the PiperSession owner.

        The caller must validate/map all remote timestamps into the Windows
        monotonic clock before submitting. The factory receives the live
        session on its owner thread; it must read a fresh task snapshot and
        construct a typed command batch locally. A remote client cannot supply
        the callable, feedback snapshot, authorization object, or command batch
        through this API.
        """
        if not callable(factory):
            raise TypeError("target factory must be a local callable")
        if type(valid_until_monotonic_ns) is not int or valid_until_monotonic_ns <= 0:
            raise ValueError("target factory deadline must be a positive Windows monotonic time")
        now = time.perf_counter_ns()
        if valid_until_monotonic_ns <= now:
            raise OwnerQueueError("target factory request is already expired")
        if valid_until_monotonic_ns - now > MAX_POSITION_APPROVAL_WINDOW_NS:
            raise ValueError("target factory deadline must be within the 250 ms local window")
        return self._submit(_Request(
            request_id=request_id,
            operation="target_factory",
            task_scope_id=task_scope_id,
            valid_until_monotonic_ns=valid_until_monotonic_ns,
            target_factory=factory,
            future=Future(),
        ))

    def submit_manual_commissioning(self, target) -> Future:
        """Queue an exact locally authorized P2 target, never a network callable."""
        from laptop.ros_gateway.manual_commissioning import ManualCommissioningTarget
        if type(target) is not ManualCommissioningTarget:
            raise TypeError("typed local manual commissioning target required")
        now = time.perf_counter_ns()
        if not 0 < target.valid_until_monotonic_ns-now <= MAX_POSITION_APPROVAL_WINDOW_NS:
            raise OwnerQueueError("manual commissioning deadline expired or too long")
        if self.active_task_scope_id:
            raise OwnerQueueError("manual commissioning cannot share an active model task")
        return self._submit(_Request(
            request_id=target.request_id, operation="manual_commissioning",
            task_scope_id="", valid_until_monotonic_ns=target.valid_until_monotonic_ns,
            manual_target=target, future=Future(),
        ))

    def submit_end(self, *, request_id: str, task_scope_id: str) -> Future:
        return self._submit(_Request(
            request_id=request_id,
            operation="end",
            task_scope_id=task_scope_id,
            valid_until_monotonic_ns=None,
            future=Future(),
        ))

    def _submit(self, request: _Request) -> Future:
        request_id = request.request_id
        if (not isinstance(request_id, str) or not request_id.isascii()
                or not request_id.strip() or len(request_id) > 128):
            raise ValueError("request_id must be bounded non-empty ASCII text")
        with self._lock:
            if self._closed:
                raise OwnerQueueError("session owner queue is closed")
            if self._owner_ident is None:
                raise OwnerQueueError("session owner thread has not been claimed")
            if request_id in self._used_request_ids:
                raise OwnerQueueError("request_id has already been used")
            try:
                self._queue.put_nowait(request)
            except queue.Full as exc:
                raise OwnerQueueFull("one owner-thread request is already pending") from exc
            self._used_request_ids.add(request_id)
        return request.future

    def pump_one(self, session: Any, *, now_monotonic_ns: int | None = None) -> bool:
        """Execute at most one request; call only from the PiperSession owner."""
        self._require_owner(allow_closed=True)
        with self._lock:
            if self._closed:
                # Target admission is closed; the owner can keep receiving CAN.
                return False
        now = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
        if type(now) is not int or now <= 0:
            raise ValueError("now_monotonic_ns must be a positive integer")
        try:
            request = self._queue.get_nowait()
        except queue.Empty:
            return False
        future = request.future
        assert future is not None
        if request.valid_until_monotonic_ns is not None and now >= request.valid_until_monotonic_ns:
            future.set_exception(OwnerQueueError("owner-thread request expired before execution"))
            if request.operation in {"target", "target_factory"}:
                self._abort_active_scope(session)
            return True
        try:
            if request.operation == "manual_commissioning":
                if self.active_task_scope_id:
                    raise OwnerQueueError("manual commissioning cannot share an active model task")
                result = request.manual_target.execute(session)
            elif request.operation == "begin":
                if self.active_task_scope_id:
                    raise OwnerQueueError("another task scope is already active")
                assert request.authorization is not None
                result = session.begin_task_position_control(request.authorization)
                with self._lock:
                    self._active_task_scope_id = request.task_scope_id
            elif request.operation == "target":
                if self.active_task_scope_id != request.task_scope_id:
                    raise OwnerQueueError("target does not match the active task scope")
                assert request.snapshot is not None and request.batch is not None
                result = session.send_task_position(snapshot=request.snapshot, batch=request.batch)
            elif request.operation == "target_factory":
                if self.active_task_scope_id != request.task_scope_id:
                    raise OwnerQueueError("target does not match the active task scope")
                assert request.target_factory is not None
                snapshot, batch = request.target_factory(session)
                if type(snapshot) is not TaskControlSnapshot:
                    raise TypeError("target factory must return a typed task feedback snapshot")
                if type(batch) is not PositionCommandBatch:
                    raise TypeError("target factory must return a typed position command batch")
                if batch.proposal.task_scope_id != request.task_scope_id:
                    raise OwnerQueueError("factory proposal does not match the active task scope")
                completed_ns = (
                    time.perf_counter_ns()
                    if now_monotonic_ns is None else now_monotonic_ns
                )
                if completed_ns >= request.valid_until_monotonic_ns:
                    raise OwnerQueueError("target factory expired before CAN dispatch")
                result = session.send_task_position(snapshot=snapshot, batch=batch)
            elif request.operation == "end":
                if self.active_task_scope_id != request.task_scope_id:
                    raise OwnerQueueError("end request does not match the active task scope")
                result = session.end_task_position_control(task_scope_id=request.task_scope_id)
                with self._lock:
                    self._active_task_scope_id = ""
            else:  # pragma: no cover - requests are private and constructed above
                raise OwnerQueueError("unsupported owner-thread operation")
            future.set_result(result)
        except Exception as exc:
            future.set_exception(exc)
            self._abort_active_scope(session)
        return True

    def close(self) -> None:
        """Reject queued work without replaying or executing it."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
        while True:
            try:
                request = self._queue.get_nowait()
            except queue.Empty:
                return
            if request.future is not None and not request.future.done():
                request.future.set_exception(OwnerQueueError("session owner queue closed before execution"))

    def abort_active_scope(self, session: Any) -> None:
        """Revoke the active software permit from the owner thread on shutdown."""
        self._require_owner(allow_closed=True)
        self._abort_active_scope(session)

    def _require_owner(self, *, allow_closed: bool = False) -> None:
        ident = threading.get_ident()
        with self._lock:
            if self._owner_ident is None or self._owner_ident != ident:
                raise OwnerQueueError("owner-thread operation called from a non-owner thread")
            if self._closed and not allow_closed:
                raise OwnerQueueError("session owner queue is closed")

    def _abort_active_scope(self, session: Any) -> None:
        scope_id = self.active_task_scope_id
        if not scope_id:
            return
        try:
            session.end_task_position_control(task_scope_id=scope_id)
        except Exception:
            # The bridge may already have revoked the permit or latched a fault.
            pass
        finally:
            with self._lock:
                self._active_task_scope_id = ""
