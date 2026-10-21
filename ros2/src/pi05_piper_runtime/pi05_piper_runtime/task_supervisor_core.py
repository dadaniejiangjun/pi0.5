"""Pure task-attempt supervisor core for the ROS RunTask action.

This module tracks a task-scoped permit, proposal/receipt correlation, wall-time
limits, and explicit operator outcome labels. It never creates a robot command
or establishes that a physical stop is effective. The Windows SafetyBridge
remains responsible for per-position authorization and every hardware gate.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading
from typing import Any

from pi05_piper_bridge.proposal_receipt_core import (
    ProposalRejected,
    proposal_payload,
)


MAX_TASK_DURATION_S = 120
MAX_TASK_AUTHORIZATION_LIFETIME_NS = 120_000_000_000


class TaskRejected(ValueError):
    """A task request, proposal, receipt, or outcome failed its contract."""


class TaskPhase(str, Enum):
    RUNNING = "RUNNING"
    STOP_REQUIRED = "STOP_REQUIRED"
    CANCEL_REQUESTED = "CANCEL_REQUESTED"
    SUCCEEDED_OPERATOR_LABEL = "SUCCEEDED_OPERATOR_LABEL"
    FAILED_OPERATOR_LABEL = "FAILED_OPERATOR_LABEL"
    PARTIAL_OPERATOR_LABEL = "PARTIAL_OPERATOR_LABEL"
    TIMED_OUT = "TIMED_OUT"


@dataclass(frozen=True)
class RunTaskGoal:
    task: str
    model_profile: str
    checkpoint_id: str
    record_full_evidence: bool
    max_duration_s: int
    authorization_id: str


@dataclass(frozen=True)
class TaskAuthorization:
    """Exact, short-lived task permit supplied by an operator-facing issuer.

    The permit issuer must create this only after explicit operator approval.
    This data object is not a cryptographic credential and does not replace
    SafetyBridge's one-use authorization for each physical position target.
    """

    authorization_id: str
    task_scope_id: str
    task: str
    model_profile: str
    checkpoint_id: str
    record_full_evidence: bool
    max_duration_s: int
    issued_monotonic_ns: int
    valid_until_monotonic_ns: int
    operator_attestation_id: str


@dataclass(frozen=True)
class TaskProposal:
    request_id: str
    task_scope_id: str
    model_profile: str
    checkpoint_id: str
    source_request_id: str
    source_observation_id: str
    source_state_sequence: int
    created_monotonic_ns: int
    valid_until_monotonic_ns: int
    source_clock_id: str
    target: tuple[float, ...]
    action_semantics: str
    units: tuple[str, ...]
    hardware_units_verified: bool
    mapping_verified: bool


@dataclass(frozen=True)
class TaskReceipt:
    request_id: str
    observation_id: str
    decision: str
    adapter_accepted: bool
    feedback_matched: bool
    can_tx_calls: int


@dataclass(frozen=True)
class TaskOutcomeLabel:
    outcome: str
    operator_id: str
    evidence_id: str
    label_id: str


@dataclass(frozen=True)
class TaskAttempt:
    trial_id: str
    task_scope_id: str
    goal: RunTaskGoal
    started_monotonic_ns: int
    deadline_monotonic_ns: int
    phase: TaskPhase = TaskPhase.RUNNING
    target_updates: int = 0
    matched_feedback_updates: int = 0
    last_state_sequence: int = 0
    pending: tuple[tuple[str, str], ...] = ()
    used_request_ids: frozenset[str] = frozenset()
    stop_reason: str = ""
    outcome_label_id: str = ""

    @property
    def success(self) -> bool:
        return self.phase is TaskPhase.SUCCEEDED_OPERATOR_LABEL

    def progress(self, *, now_monotonic_ns: int) -> dict[str, Any]:
        now = _positive_int(now_monotonic_ns, "now_monotonic_ns")
        if now < self.started_monotonic_ns:
            raise TaskRejected("task progress time precedes its start")
        elapsed_ns = now - self.started_monotonic_ns
        fraction = min(1.0, elapsed_ns / (self.deadline_monotonic_ns - self.started_monotonic_ns))
        return {
            "phase": self.phase.value,
            "elapsed_s": elapsed_ns / 1_000_000_000.0,
            "target_updates": self.target_updates,
            "matched_feedback_updates": self.matched_feedback_updates,
            "progress": fraction,
        }

    def register_proposal(self, proposal: Any, *, now_monotonic_ns: int) -> "TaskAttempt":
        self._require_running(now_monotonic_ns)
        try:
            payload = proposal_payload(proposal, now_monotonic_ns=now_monotonic_ns)
        except (ProposalRejected, TypeError, ValueError) as exc:
            return self._require_stop(f"proposal_rejected:{exc}")
        request_id = payload["request_id"]
        if payload["task_scope_id"] != self.task_scope_id:
            return self._require_stop("proposal_task_scope_mismatch")
        if payload["model_profile"] != self.goal.model_profile:
            return self._require_stop("proposal_model_profile_mismatch")
        if payload["checkpoint_id"] != self.goal.checkpoint_id:
            return self._require_stop("proposal_checkpoint_mismatch")
        if not payload["hardware_units_verified"] or not payload["mapping_verified"]:
            return self._require_stop("proposal_units_or_mapping_unverified")
        if request_id in self.used_request_ids:
            return self._require_stop("proposal_request_id_reused")
        if not payload["observation_id"]:
            return self._require_stop("proposal_observation_id_missing")
        state_sequence = payload["source_state_sequence"]
        if state_sequence <= self.last_state_sequence:
            return self._require_stop("proposal_state_sequence_not_increasing")
        if self.pending:
            return self._require_stop("proposal_arrived_before_previous_receipt")
        new_pending = ((request_id, payload["observation_id"]),)
        return _replace_attempt(
            self,
            pending=new_pending,
            used_request_ids=self.used_request_ids | {request_id},
            last_state_sequence=state_sequence,
        )

    def record_receipt(self, receipt: TaskReceipt, *, now_monotonic_ns: int) -> "TaskAttempt":
        self._require_running(now_monotonic_ns)
        if not isinstance(receipt, TaskReceipt):
            return self._require_stop("receipt_has_wrong_type")
        if (not isinstance(receipt.request_id, str) or not receipt.request_id
                or not isinstance(receipt.observation_id, str) or not receipt.observation_id):
            return self._require_stop("receipt_identifiers_invalid")
        pending = dict(self.pending)
        expected_observation = pending.get(receipt.request_id)
        if expected_observation is None:
            return self._require_stop("receipt_request_id_not_pending")
        if receipt.observation_id != expected_observation:
            return self._require_stop("receipt_observation_id_mismatch")
        if type(receipt.adapter_accepted) is not bool or type(receipt.feedback_matched) is not bool:
            return self._require_stop("receipt_acceptance_fields_must_be_boolean")
        if (type(receipt.can_tx_calls) is not int or receipt.can_tx_calls < 0
                or not isinstance(receipt.decision, str) or not receipt.decision):
            return self._require_stop("receipt_fields_invalid")
        if (not receipt.adapter_accepted or not receipt.feedback_matched
                or receipt.can_tx_calls < 1):
            return self._require_stop(f"receipt_not_accepted_and_feedback_matched:{receipt.decision}")
        return _replace_attempt(
            self,
            pending=(),
            target_updates=self.target_updates + 1,
            matched_feedback_updates=self.matched_feedback_updates + 1,
        )

    def record_operator_outcome(
        self, label: TaskOutcomeLabel, *, now_monotonic_ns: int,
    ) -> "TaskAttempt":
        self._require_running(now_monotonic_ns)
        if not isinstance(label, TaskOutcomeLabel):
            return self._require_stop("outcome_label_has_wrong_type")
        if self.pending:
            return self._require_stop("cannot_label_outcome_with_unresolved_command")
        if self.matched_feedback_updates < 1:
            return self._require_stop("cannot_label_task_before_matched_hardware_feedback")
        if not all(isinstance(value, str) and value.strip() for value in (
            label.operator_id, label.evidence_id, label.label_id,
        )):
            return self._require_stop("outcome_label_requires_operator_evidence_and_id")
        if not isinstance(label.outcome, str):
            return self._require_stop("outcome_must_be_success_failure_or_partial")
        phases = {
            "success": TaskPhase.SUCCEEDED_OPERATOR_LABEL,
            "failure": TaskPhase.FAILED_OPERATOR_LABEL,
            "partial": TaskPhase.PARTIAL_OPERATOR_LABEL,
        }
        phase = phases.get(label.outcome)
        if phase is None:
            return self._require_stop("outcome_must_be_success_failure_or_partial")
        return _replace_attempt(self, phase=phase, outcome_label_id=label.label_id)

    def request_cancel(self, *, now_monotonic_ns: int) -> "TaskAttempt":
        self._require_running(now_monotonic_ns)
        return _replace_attempt(
            self,
            phase=TaskPhase.CANCEL_REQUESTED,
            stop_reason="cancel_requested_physical_stop_unverified",
        )

    def expire(self, *, now_monotonic_ns: int) -> "TaskAttempt":
        now = _positive_int(now_monotonic_ns, "now_monotonic_ns")
        if self.phase is not TaskPhase.RUNNING:
            raise TaskRejected("only a running task can expire")
        if now < self.deadline_monotonic_ns:
            raise TaskRejected("task deadline has not been reached")
        return _replace_attempt(
            self,
            phase=TaskPhase.TIMED_OUT,
            stop_reason="task_deadline_reached_physical_stop_unverified",
        )

    def _require_running(self, now_monotonic_ns: int) -> None:
        now = _positive_int(now_monotonic_ns, "now_monotonic_ns")
        if self.phase is not TaskPhase.RUNNING:
            raise TaskRejected(f"task is not running: {self.phase.value}")
        if now < self.started_monotonic_ns:
            raise TaskRejected("event time precedes task start")
        if now >= self.deadline_monotonic_ns:
            raise TaskRejected("task deadline reached; expire the task before processing more events")

    def _require_stop(self, reason: str) -> "TaskAttempt":
        return _replace_attempt(self, phase=TaskPhase.STOP_REQUIRED, stop_reason=reason)


class TaskAuthorizationRegistry:
    """Consume each task authorization id at most once in this process."""

    def __init__(self) -> None:
        self._used_ids: set[str] = set()
        self._lock = threading.Lock()

    def begin_task(
        self,
        goal: RunTaskGoal,
        authorization: TaskAuthorization,
        *,
        now_monotonic_ns: int,
        trial_id: str,
    ) -> TaskAttempt:
        """Create an attempt only when a fresh permit exactly scopes the goal."""
        with self._lock:
            if isinstance(authorization, TaskAuthorization) and authorization.authorization_id in self._used_ids:
                raise TaskRejected("task authorization id has already been used")
            attempt = _create_task_attempt(
                goal, authorization,
                now_monotonic_ns=now_monotonic_ns,
                trial_id=trial_id,
            )
            self._used_ids.add(authorization.authorization_id)
            return attempt


def _create_task_attempt(
    goal: RunTaskGoal,
    authorization: TaskAuthorization,
    *,
    now_monotonic_ns: int,
    trial_id: str,
) -> TaskAttempt:
    """Validate and create a task attempt; called only by the one-use registry."""
    now = _positive_int(now_monotonic_ns, "now_monotonic_ns")
    if not isinstance(goal, RunTaskGoal) or not isinstance(authorization, TaskAuthorization):
        raise TaskRejected("typed RunTaskGoal and TaskAuthorization are required")
    for value, name in ((trial_id, "trial_id"), (goal.task, "task"),
                        (goal.model_profile, "model_profile"),
                        (goal.checkpoint_id, "checkpoint_id"),
                        (goal.authorization_id, "authorization_id"),
                        (authorization.authorization_id, "authorization_id"),
                        (authorization.task_scope_id, "task_scope_id"),
                        (authorization.operator_attestation_id, "operator_attestation_id")):
        if not isinstance(value, str) or not value.strip() or len(value) > 256:
            raise TaskRejected(f"{name} must be non-empty bounded text")
    if type(goal.record_full_evidence) is not bool or type(authorization.record_full_evidence) is not bool:
        raise TaskRejected("record_full_evidence must be boolean")
    if type(goal.max_duration_s) is not int or not 1 <= goal.max_duration_s <= MAX_TASK_DURATION_S:
        raise TaskRejected("max_duration_s must be between 1 and 120 seconds")
    if type(authorization.max_duration_s) is not int:
        raise TaskRejected("authorization max_duration_s must be an integer")
    bound_goal = (
        goal.authorization_id == authorization.authorization_id
        and goal.task == authorization.task
        and goal.model_profile == authorization.model_profile
        and goal.checkpoint_id == authorization.checkpoint_id
        and goal.record_full_evidence == authorization.record_full_evidence
        and goal.max_duration_s == authorization.max_duration_s
    )
    if not bound_goal:
        raise TaskRejected("task authorization does not bind the exact RunTask goal")
    issued = _positive_int(authorization.issued_monotonic_ns, "issued_monotonic_ns")
    expires = _positive_int(authorization.valid_until_monotonic_ns, "valid_until_monotonic_ns")
    duration_ns = goal.max_duration_s * 1_000_000_000
    if (issued > now or expires <= now or expires <= issued
            or expires - issued > MAX_TASK_AUTHORIZATION_LIFETIME_NS
            or expires - issued > duration_ns):
        raise TaskRejected(
            "task authorization is future-dated, expired, or longer than its task duration "
            "(maximum 120 seconds)"
        )
    return TaskAttempt(
        trial_id=trial_id,
        task_scope_id=authorization.task_scope_id,
        goal=goal,
        started_monotonic_ns=now,
        deadline_monotonic_ns=now + duration_ns,
    )


def _positive_int(value: Any, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise TaskRejected(f"{name} must be a positive integer")
    return value


def _replace_attempt(attempt: TaskAttempt, **changes: Any) -> TaskAttempt:
    from dataclasses import replace
    return replace(attempt, **changes)
