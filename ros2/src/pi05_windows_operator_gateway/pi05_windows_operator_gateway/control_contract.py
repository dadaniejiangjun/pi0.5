"""Session, authorization, replay, freshness, and fake-only proposal rules."""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import secrets
import threading
import time
import uuid


MAX_STATE_AGE_NS = 120_000_000
MAX_REQUEST_VALIDITY_MS = 5_000
AUTHORIZATION_TTL_NS = 5_000_000_000
MAX_REMEMBERED_REQUESTS = 1024
SUPPORTED_OPERATIONS = frozenset({"set_joint_target", "set_gripper"})


@dataclass(frozen=True)
class ProposalReceipt:
    accepted: bool
    decision: str
    reason: str
    request_id: str
    hardware_control_allowed: bool = False
    can_tx_calls: int = 0
    simulated_only: bool = True

    def as_dict(self) -> dict:
        return {
            "accepted": self.accepted,
            "decision": self.decision,
            "reason": self.reason,
            "request_id": self.request_id,
            "hardware_control_allowed": False,
            "can_tx_calls": 0,
            "simulated_only": self.simulated_only,
        }


@dataclass
class _Authorization:
    app_session_id: str
    source_session_id: str
    source_state_sequence: int
    operation: str
    task_scope_id: str
    issued_monotonic_ns: int
    expires_monotonic_ns: int


class ProposalAuthority:
    """In-memory, single-session proposal authority with no hardware sink."""

    def __init__(self, *, control_mode: str = "disabled", clock_ns=time.monotonic_ns):
        if control_mode not in {"disabled", "fake"}:
            raise ValueError("control_mode must be 'disabled' or 'fake'")
        self.control_mode = control_mode
        self._clock_ns = clock_ns
        self._lock = threading.RLock()
        self._app_session_id = ""
        self._authorizations: dict[str, _Authorization] = {}
        self._used_request_ids: OrderedDict[str, None] = OrderedDict()

    def open_session(self) -> dict:
        """Rotate the one active session, revoking grants and replay history."""
        with self._lock:
            self._app_session_id = uuid.uuid4().hex
            self._authorizations.clear()
            self._used_request_ids.clear()
            return {
                "protocol_version": 1,
                "session_id": self._app_session_id,
                "control_mode": self.control_mode,
                "hardware_control_allowed": False,
            }

    def authorize(self, payload: dict, *, state: dict) -> dict:
        with self._lock:
            self._prune_expired()
            session_id = payload.get("session_id")
            operation = payload.get("operation")
            sequence = payload.get("source_state_sequence")
            task_scope_id = payload.get("task_scope_id")
            if session_id != self._app_session_id or not session_id:
                return self._deny("REJECTED_SESSION_MISMATCH", "operator session is not active")
            if payload.get("operator_confirmed") is not True:
                return self._deny("REJECTED_OPERATOR_CONFIRMATION_REQUIRED", "explicit operator confirmation is required")
            if operation not in SUPPORTED_OPERATIONS:
                return self._deny("REJECTED_OPERATION_UNSUPPORTED", "only joint and gripper target proposals are supported")
            if (not isinstance(task_scope_id, str) or not task_scope_id.isascii()
                    or not task_scope_id.strip() or len(task_scope_id) > 128):
                return self._deny("REJECTED_INVALID_TASK_SCOPE", "task_scope_id must be bounded non-empty ASCII text")
            if payload.get("source_state_session_id") != state.get("source_session_id"):
                return self._deny("REJECTED_SOURCE_SESSION_MISMATCH", "request is not bound to the current source feedback session")
            freshness = self._validate_source_state(state, sequence)
            if freshness:
                return self._deny(*freshness)
            grant_id = secrets.token_urlsafe(24)
            issued_ns = self._clock_ns()
            self._authorizations[grant_id] = _Authorization(
                app_session_id=session_id,
                source_session_id=str(state.get("source_session_id", "")),
                source_state_sequence=sequence,
                operation=operation,
                task_scope_id=task_scope_id,
                issued_monotonic_ns=issued_ns,
                expires_monotonic_ns=issued_ns + AUTHORIZATION_TTL_NS,
            )
            return {
                "accepted": True,
                "decision": "ONE_REQUEST_AUTHORIZED",
                "authorization_id": grant_id,
                "expires_in_ms": AUTHORIZATION_TTL_NS // 1_000_000,
                "hardware_control_allowed": False,
            }

    def propose(self, payload: dict, *, state: dict) -> dict:
        with self._lock:
            self._prune_expired()
            request_id = payload.get("request_id")
            if (not isinstance(request_id, str) or not request_id.isascii()
                    or not request_id.strip() or len(request_id) > 128):
                return self._deny("REJECTED_INVALID_REQUEST_ID", "request_id must be non-empty bounded ASCII text", request_id="")
            if request_id in self._used_request_ids:
                return self._deny("REJECTED_REPLAY", "request_id was already used", request_id=request_id)
            if payload.get("session_id") != self._app_session_id or not self._app_session_id:
                return self._deny("REJECTED_SESSION_MISMATCH", "operator session is not active", request_id=request_id)
            self._remember(request_id)
            validity_ms = payload.get("validity_ms")
            if type(validity_ms) is not int or not 1 <= validity_ms <= MAX_REQUEST_VALIDITY_MS:
                return self._deny("REJECTED_INVALID_EXPIRY", "validity_ms must be in 1..5000", request_id=request_id)
            if payload.get("operation") not in SUPPORTED_OPERATIONS:
                return self._deny("REJECTED_OPERATION_UNSUPPORTED", "operation is outside the supported proposal surface", request_id=request_id)
            task_scope_id = payload.get("task_scope_id")
            if (not isinstance(task_scope_id, str) or not task_scope_id.isascii()
                    or not task_scope_id.strip() or len(task_scope_id) > 128):
                return self._deny("REJECTED_INVALID_TASK_SCOPE", "task_scope_id must be bounded non-empty ASCII text", request_id=request_id)
            if payload.get("source_state_session_id") != state.get("source_session_id"):
                return self._deny("REJECTED_SOURCE_SESSION_MISMATCH", "request is not bound to the current source feedback session", request_id=request_id)
            args_error = self._validate_arguments(payload.get("operation"), payload.get("arguments"))
            if args_error:
                return self._deny("REJECTED_INVALID_ARGUMENTS", args_error, request_id=request_id)
            freshness = self._validate_source_state(state, payload.get("source_state_sequence"))
            if freshness:
                return self._deny(*freshness, request_id=request_id)
            grant_id = payload.get("authorization_id")
            grant = self._authorizations.pop(grant_id, None) if isinstance(grant_id, str) else None
            if grant is None:
                return self._deny("REJECTED_AUTHORIZATION_REQUIRED", "request needs a current one-use authorization", request_id=request_id)
            if (grant.app_session_id != self._app_session_id
                    or grant.source_session_id != str(state.get("source_session_id", ""))
                    or grant.source_state_sequence != payload.get("source_state_sequence")
                    or grant.operation != payload.get("operation")
                    or grant.task_scope_id != task_scope_id):
                return self._deny("REJECTED_AUTHORIZATION_SCOPE_MISMATCH", "authorization is bound to another state or operation", request_id=request_id)
            request_deadline_ns = min(
                grant.expires_monotonic_ns,
                grant.issued_monotonic_ns + validity_ms * 1_000_000,
            )
            if self._clock_ns() >= request_deadline_ns:
                return self._deny("REJECTED_REQUEST_EXPIRED", "request expired before the server received it", request_id=request_id)
            if self.control_mode == "disabled":
                return self._deny("REJECTED_CONTROL_NOT_DEPLOYED", "WSL control dispatcher is not deployed; no command was sent", request_id=request_id, simulated_only=False)
            return ProposalReceipt(
                accepted=True,
                decision="SIMULATED_ACCEPTED",
                reason="fake proposal backend accepted the envelope; no ROS control service or CAN path was called",
                request_id=request_id,
                simulated_only=True,
            ).as_dict()

    @staticmethod
    def _validate_arguments(operation: str, arguments) -> str:
        if not isinstance(arguments, dict):
            return "arguments must be a map"
        if operation == "set_joint_target":
            values = arguments.get("joint_position_rad")
            if not isinstance(values, list) or len(values) != 6:
                return "joint_position_rad must contain six values"
            if not all(_finite_number(value) for value in values):
                return "joint_position_rad values must be finite numbers"
        elif operation == "set_gripper":
            if not _finite_number(arguments.get("gripper_nominal_m")):
                return "gripper_nominal_m must be a finite number"
        return ""

    def _validate_source_state(self, state: dict, sequence) -> tuple[str, str] | None:
        if not isinstance(state, dict) or not state.get("available"):
            return "REJECTED_STATE_UNAVAILABLE", "no current robot-state snapshot is available"
        if type(sequence) is not int or sequence != state.get("sequence"):
            return "REJECTED_STALE_STATE_SEQUENCE", "request does not reference the current state sequence"
        if not isinstance(state.get("source_session_id"), str) or not state["source_session_id"]:
            return "REJECTED_STATE_SESSION_MISSING", "state snapshot has no source session identity"
        if state.get("feedback_fresh") is not True:
            return "REJECTED_STATE_STALE", "latest feedback is not marked fresh"
        if state.get("numeric_state_valid") is not True:
            return "REJECTED_INVALID_STATE", "source state contains non-finite or missing joint/gripper values"
        if state.get("hardware_connected") is not True and state.get("fake") is not True:
            return "REJECTED_HARDWARE_STATE_UNAVAILABLE", "state is neither connected hardware feedback nor an explicit fake snapshot"
        age_ns = state.get("age_ns")
        if type(age_ns) is not int or age_ns < 0 or age_ns > MAX_STATE_AGE_NS:
            return "REJECTED_STATE_STALE", "source feedback is older than 120 ms"
        return None

    def _prune_expired(self) -> None:
        now = self._clock_ns()
        for grant_id, grant in list(self._authorizations.items()):
            if grant.expires_monotonic_ns <= now:
                self._authorizations.pop(grant_id, None)

    def _remember(self, request_id: str) -> None:
        self._used_request_ids[request_id] = None
        self._used_request_ids.move_to_end(request_id)
        while len(self._used_request_ids) > MAX_REMEMBERED_REQUESTS:
            self._used_request_ids.popitem(last=False)

    @staticmethod
    def _deny(decision: str, reason: str, *, request_id: str = "", simulated_only: bool = False) -> dict:
        return ProposalReceipt(
            accepted=False,
            decision=decision,
            reason=reason,
            request_id=request_id,
            simulated_only=simulated_only,
        ).as_dict()


def _finite_number(value) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))
