"""Windows-local approval and TCP-session binding for position RPCs.

This authority is an in-process component. It never accepts an authorization
object or approval token from MsgPack. A local caller must interactively
confirm the exact session and already-active task scope. The production CLI
does not construct this component; installing it does not enable a target
mapper, bypass SafetyBridge, or make a physical stop claim.
"""

from __future__ import annotations

from dataclasses import dataclass
import hmac
import re
import threading
import time
from typing import Callable

from laptop.safety.task_scope import TaskScopedPositionAuthorization


_ID_PATTERN = re.compile(r"[A-Za-z0-9_-]{1,128}\Z", re.ASCII)


def _validate_id(value: object, name: str) -> None:
    if not isinstance(value, str) or _ID_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{name} must be bounded ASCII identifier text")


@dataclass(frozen=True)
class LocalPositionSessionBinding:
    """Non-secret receipt for an operator-approved in-process binding."""

    session_id: str
    task_scope_id: str
    authorization_id: str
    valid_until_monotonic_ns: int


@dataclass(frozen=True)
class _ActiveBinding:
    session_id: str
    task_scope_id: str
    authorization: TaskScopedPositionAuthorization
    valid_until_monotonic_ns: int


class WindowsLocalPositionSessionAuthority:
    """Bind one TCP session to an already-active local task authorization.

    ``active_task_scope_provider`` must read the task scope from the local
    Windows owner. ``input_fn`` and ``output_fn`` are local-console functions;
    the caller cannot pass them over the network. The confirmation phrase
    identifies both the session and task scope, is checked again against live
    owner state after input, and grants no more than the existing task permit's
    remaining lifetime.
    """

    def __init__(self, *, active_task_scope_provider: Callable[[], str],
                 monotonic_ns: Callable[[], int] = time.perf_counter_ns) -> None:
        if not callable(active_task_scope_provider) or not callable(monotonic_ns):
            raise TypeError("local task-scope provider and monotonic clock are required")
        self._active_task_scope_provider = active_task_scope_provider
        self._monotonic_ns = monotonic_ns
        self._lock = threading.RLock()
        self._binding: _ActiveBinding | None = None

    @staticmethod
    def confirmation_phrase(session_id: str, task_scope_id: str) -> str:
        _validate_id(session_id, "session_id")
        _validate_id(task_scope_id, "task_scope_id")
        return f"AUTHORIZE POSITION SESSION {session_id} TASK {task_scope_id}"

    def issue_from_console(
        self, *, session_id: str,
        authorization: TaskScopedPositionAuthorization,
        input_fn: Callable[[str], str] = input,
        output_fn: Callable[[str], object] = print,
    ) -> LocalPositionSessionBinding:
        """Ask a local operator to bind the exact session to an active scope."""
        _validate_id(session_id, "session_id")
        if type(authorization) is not TaskScopedPositionAuthorization:
            raise TypeError("binding requires a typed in-process task authorization")
        if not callable(input_fn) or not callable(output_fn):
            raise TypeError("local console input and output callbacks are required")

        now_ns = self._monotonic_ns()
        self._validate_live_scope(authorization, now_ns)
        phrase = self.confirmation_phrase(session_id, authorization.task_scope_id)
        output_fn(
            "This approval binds position RPCs to this local TCP session and the "
            "already-active task until its existing expiry. It does not verify "
            "physical movement or safe-stop behavior."
        )
        entered = input_fn(f"Type exactly: {phrase}\n> ")
        if not isinstance(entered, str) or not hmac.compare_digest(entered, phrase):
            raise PermissionError("local position-session confirmation did not match")

        confirmed_ns = self._monotonic_ns()
        self._validate_live_scope(authorization, confirmed_ns)
        binding = _ActiveBinding(
            session_id=session_id,
            task_scope_id=authorization.task_scope_id,
            authorization=authorization,
            valid_until_monotonic_ns=authorization.valid_until_monotonic_ns,
        )
        with self._lock:
            self._expire_locked(confirmed_ns)
            if self._binding is not None:
                raise PermissionError("another TCP session already holds the local position binding")
            self._binding = binding
        return LocalPositionSessionBinding(
            session_id=binding.session_id,
            task_scope_id=binding.task_scope_id,
            authorization_id=authorization.authorization_id,
            valid_until_monotonic_ns=binding.valid_until_monotonic_ns,
        )

    def authorize_session(self, session_id: str, task_scope_id: str) -> bool:
        """Return true only for the exact live local session/task binding."""
        try:
            _validate_id(session_id, "session_id")
            _validate_id(task_scope_id, "task_scope_id")
            now_ns = self._monotonic_ns()
            active_scope = self._active_task_scope_provider()
        except Exception:
            return False
        if not isinstance(active_scope, str):
            return False
        if active_scope != task_scope_id:
            with self._lock:
                if (self._binding is not None
                        and self._binding.task_scope_id != active_scope):
                    self._binding = None
            return False
        with self._lock:
            binding = self._binding
            if binding is None:
                return False
            if (binding.session_id != session_id
                    or binding.task_scope_id != task_scope_id):
                return False
            if now_ns >= binding.valid_until_monotonic_ns:
                self._binding = None
                return False
            try:
                binding.authorization.validate_active(now_monotonic_ns=now_ns)
            except (TypeError, ValueError, PermissionError):
                self._binding = None
                return False
            return True

    def revoke_session(self, session_id: str) -> bool:
        """Drop a binding when the matching TCP session closes."""
        with self._lock:
            if self._binding is None or self._binding.session_id != session_id:
                return False
            self._binding = None
            return True

    def revoke_task_scope(self, task_scope_id: str) -> bool:
        """Drop the binding when the locally active task ends or is aborted."""
        with self._lock:
            if self._binding is None or self._binding.task_scope_id != task_scope_id:
                return False
            self._binding = None
            return True

    def _validate_live_scope(
        self, authorization: TaskScopedPositionAuthorization, now_ns: int,
    ) -> None:
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError("local monotonic clock returned an invalid timestamp")
        authorization.validate_active(now_monotonic_ns=now_ns)
        active_scope = self._active_task_scope_provider()
        if active_scope != authorization.task_scope_id:
            raise PermissionError("task authorization is not the active Windows owner scope")

    def _expire_locked(self, now_ns: int) -> None:
        binding = self._binding
        if binding is None:
            return
        try:
            binding.authorization.validate_active(now_monotonic_ns=now_ns)
        except (TypeError, ValueError, PermissionError):
            self._binding = None
