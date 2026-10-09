"""One-step operator consent bound to a policy identity and safety envelope.

Human confirmation happens before policy inference, so it does not make a
100 ms policy target expire while someone reads a screen. After inference,
the grant can mint one exact, short-lived authorization only when the prepared
target still matches the approved task, artifacts, site gates, and limits.

This module does not verify that physical-site attestations are true and does
not open CAN. A caller must still send through the dedicated SafetyBridge.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import threading
import time
from typing import Callable
from uuid import uuid4

from laptop.control.contracts import (
    NEXT_OBSERVED_STATE_PROXY,
    PolicyPrediction,
    RobotStateSnapshot,
)
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.supervised_policy_step import PreparedSupervisedPolicyStep
from laptop.safety.bridge import SafetyLimits
from laptop.transport.position_commands import (
    MAX_POSITION_APPROVAL_WINDOW_NS,
    P12_SITE_GATE_NAMES,
    PositionBatchWriteReceipt,
    SupervisedPositionAuthorization,
)


ONE_STEP_CONFIRMATION_PHRASE = "AUTHORIZE ONE POSITION STEP"
MAX_ONE_STEP_GRANT_LIFETIME_NS = 30_000_000_000
DEFAULT_ONE_STEP_GRANT_LIFETIME_NS = 10_000_000_000
SHORT_LOOP_MAX_STEPS = 3
SHORT_LOOP_CONFIRMATION_PHRASE = "AUTHORIZE 3 POSITION STEPS"


@dataclass(frozen=True)
class OneStepOperatorGrant:
    """A one-use approval for one bounded batch from one locked policy.

    The operator approves the task, artifact identities, and exact SafetyLimits
    before inference. The grant never authorizes a second batch or widens those
    limits. ``authorize`` rechecks the concrete prediction and target, then
    creates the existing 250 ms proposal-bound token for immediate sending.
    """

    grant_id: str
    task_description: str
    checkpoint_identity: str
    config_identity: str
    normalization_identity: str
    dataset_identity: str
    gripper_effort_raw: int
    issued_at_monotonic_ns: int
    expires_at_monotonic_ns: int
    limits: SafetyLimits
    site_gates: P12GateEvidence
    operator_confirmed: bool
    _used: bool = field(default=False, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def __post_init__(self) -> None:
        for name in (
            "grant_id",
            "task_description",
            "checkpoint_identity",
            "config_identity",
            "normalization_identity",
            "dataset_identity",
        ):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} is required")
        if (type(self.gripper_effort_raw) is not int
                or not 0 <= self.gripper_effort_raw <= 5000):
            raise ValueError("gripper_effort_raw must be an integer in [0, 5000]")
        if type(self.issued_at_monotonic_ns) is not int or self.issued_at_monotonic_ns <= 0:
            raise ValueError("issued_at_monotonic_ns must be positive")
        if (type(self.expires_at_monotonic_ns) is not int
                or self.expires_at_monotonic_ns <= self.issued_at_monotonic_ns):
            raise ValueError("grant expiry must follow its issue time")
        if (self.expires_at_monotonic_ns - self.issued_at_monotonic_ns
                > MAX_ONE_STEP_GRANT_LIFETIME_NS):
            raise ValueError("one-step operator grant lifetime exceeds 30 seconds")
        if type(self.limits) is not SafetyLimits:
            raise TypeError("limits must be SafetyLimits")
        if type(self.site_gates) is not P12GateEvidence:
            raise TypeError("site_gates must be P12GateEvidence")
        if self.site_gates.missing:
            raise PermissionError("site verification is incomplete: "
                                  + ", ".join(self.site_gates.missing))
        if type(self.operator_confirmed) is not bool or not self.operator_confirmed:
            raise PermissionError("explicit operator confirmation is required")

    @property
    def used(self) -> bool:
        with self._lock:
            return self._used

    def to_worker_ticket(self) -> OneStepOperatorGrantTicket:
        """Transfer this consent once to a disposable worker process."""
        with self._lock:
            if self._used:
                raise PermissionError("one-step operator grant has already been consumed")
            object.__setattr__(self, "_used", True)
            return OneStepOperatorGrantTicket(
                grant_id=self.grant_id,
                task_description=self.task_description,
                checkpoint_identity=self.checkpoint_identity,
                config_identity=self.config_identity,
                normalization_identity=self.normalization_identity,
                dataset_identity=self.dataset_identity,
                gripper_effort_raw=self.gripper_effort_raw,
                issued_at_monotonic_ns=self.issued_at_monotonic_ns,
                expires_at_monotonic_ns=self.expires_at_monotonic_ns,
                limits=self.limits,
                site_gates=self.site_gates,
                operator_confirmed=self.operator_confirmed,
            )

    def authorize(
        self,
        prediction: PolicyPrediction,
        prepared: PreparedSupervisedPolicyStep,
        *,
        active_task_description: str,
        inference_started_monotonic_ns: int,
        limits: SafetyLimits,
        now_monotonic_ns: int | None = None,
    ) -> SupervisedPositionAuthorization:
        """Mint one proposal-bound token while the prepared target is current."""
        if type(prediction) is not PolicyPrediction:
            raise TypeError("prediction must be a PolicyPrediction")
        if type(prepared) is not PreparedSupervisedPolicyStep:
            raise TypeError("prepared must be a PreparedSupervisedPolicyStep")
        if type(limits) is not SafetyLimits or limits != self.limits:
            raise PermissionError("runtime SafetyLimits differ from the approved envelope")
        now_ns = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError("now_monotonic_ns must be positive")
        if (type(inference_started_monotonic_ns) is not int
                or inference_started_monotonic_ns <= 0):
            raise ValueError("inference_started_monotonic_ns must be positive")

        with self._lock:
            if self._used:
                raise PermissionError("one-step operator grant has already been consumed")
            if now_ns < self.issued_at_monotonic_ns or now_ns > self.expires_at_monotonic_ns:
                raise PermissionError("one-step operator grant is not current")
            if (self.issued_at_monotonic_ns > inference_started_monotonic_ns
                    or inference_started_monotonic_ns > prediction.received_monotonic_ns):
                raise PermissionError("operator grant must precede policy inference")
            if not self.operator_confirmed or self.site_gates.missing:
                raise PermissionError("operator consent or site verification is incomplete")
            if active_task_description != self.task_description:
                raise PermissionError("active task differs from the approved task")
            if not prediction.prediction_only or prediction.hardware_units_verified:
                raise PermissionError("policy output must remain a nominal prediction")
            if prediction.action_semantics != NEXT_OBSERVED_STATE_PROXY:
                raise PermissionError("policy output is not a derived next-state target")
            if prediction.request_sequence != prepared.snapshot.sequence:
                raise ValueError("prediction does not match the prepared source state")
            if prediction.request_id != prepared.proposal.source_request_id:
                raise ValueError("prediction does not match the prepared request")
            if (prediction.checkpoint_identity != self.checkpoint_identity
                    or prediction.config_identity != self.config_identity
                    or prediction.normalization_identity != self.normalization_identity
                    or prediction.dataset_identity != self.dataset_identity):
                raise PermissionError("policy artifact identity differs from the approved lock")
            if prepared.preview.gripper_ctrl_args[1] != self.gripper_effort_raw:
                raise PermissionError("prepared gripper effort differs from the approved effort")
            if (prepared.preview.checkpoint_identity != self.checkpoint_identity
                    or prepared.preview.config_identity != self.config_identity):
                raise PermissionError("prepared preview differs from the approved policy")
            if prepared.preview.source_state_sequence != prepared.snapshot.sequence:
                raise ValueError("preview does not match the prepared source state")
            if prepared.proposal.target != prepared.preview.joint_target_rad_nominal + (
                prepared.preview.gripper_target_m_nominal,
            ):
                raise ValueError("proposal target differs from the reviewed preview")
            if prepared.proposal.valid_until_monotonic_ns <= now_ns:
                raise PermissionError("prepared policy target has expired")
            if prepared.preview.target_monotonic_ns <= now_ns:
                raise PermissionError("policy target timestamp has expired")
            if now_ns < prediction.received_monotonic_ns:
                raise ValueError("policy prediction is from the future")
            if prediction.response_latency_ns > limits.max_prediction_response_latency_ns:
                raise PermissionError("policy response latency exceeds the approved limit")
            if (now_ns - prediction.received_monotonic_ns
                    > limits.max_prediction_response_latency_ns):
                raise PermissionError("policy prediction is stale")
            if (now_ns < prepared.snapshot.received_monotonic_ns
                    or now_ns - prepared.snapshot.received_monotonic_ns
                    > limits.max_state_age_ns):
                raise PermissionError("robot feedback is stale")
            if (not prepared.snapshot.connected or not prepared.snapshot.status_ok
                    or not prepared.snapshot.units_verified):
                raise PermissionError("robot feedback is not verified and ready")
            if prepared.proposal.target != prepared.batch.proposal.target:
                raise ValueError("position batch is not bound to the approved target")

            for current, target, low, high, max_delta in zip(
                prepared.snapshot.state7,
                prepared.proposal.target,
                self.limits.joint_min_rad + (self.limits.gripper_min_m,),
                self.limits.joint_max_rad + (self.limits.gripper_max_m,),
                self.limits.max_joint_delta_rad + (self.limits.max_gripper_delta_m,),
            ):
                if not low <= current <= high or not low <= target <= high:
                    raise PermissionError("prepared target is outside the approved envelope")
                if abs(target - current) > max_delta:
                    raise PermissionError("prepared target exceeds the approved step delta")

            auth_until_ns = min(
                prepared.proposal.valid_until_monotonic_ns,
                now_ns + MAX_POSITION_APPROVAL_WINDOW_NS,
            )
            if auth_until_ns <= now_ns:
                raise PermissionError("no fresh position authorization window remains")
            authorization = SupervisedPositionAuthorization(
                approval_id=f"{self.grant_id}:{prepared.proposal.proposal_id}",
                proposal_id=prepared.proposal.proposal_id,
                source_request_id=prepared.proposal.source_request_id,
                source_state_sequence=prepared.proposal.source_state_sequence,
                target=prepared.proposal.target,
                approved_at_monotonic_ns=now_ns,
                valid_until_monotonic_ns=auth_until_ns,
                operator_confirmed=True,
                site_gate_attestations=tuple(asdict(self.site_gates).items()),
            )
            object.__setattr__(self, "_used", True)
            return authorization


@dataclass(frozen=True)
class ShortLoopOperatorGrant:
    """One explicit, 3-step session grant requiring feedback between steps.

    Every command still gets its own proposal-bound 250 ms authorization via
    ``OneStepOperatorGrant``. This session object only permits another step
    after the previous write receipt and a newer, healthy state snapshot arrive.
    The caller must abort the session on any send error or missing feedback.
    """

    grant_id: str
    task_description: str
    checkpoint_identity: str
    config_identity: str
    normalization_identity: str
    dataset_identity: str
    gripper_effort_raw: int
    issued_at_monotonic_ns: int
    expires_at_monotonic_ns: int
    limits: SafetyLimits
    site_gates: P12GateEvidence
    operator_confirmed: bool
    max_steps: int = SHORT_LOOP_MAX_STEPS
    _authorized_steps: int = field(default=0, init=False, repr=False)
    _pending_proposal_id: str | None = field(default=None, init=False, repr=False)
    _pending_source_sequence: int | None = field(default=None, init=False, repr=False)
    _initial_state7: tuple[float, ...] | None = field(default=None, init=False, repr=False)
    _confirmed_snapshot: RobotStateSnapshot | None = field(default=None, init=False, repr=False)
    _last_write_ns: int | None = field(default=None, init=False, repr=False)
    _transferred: bool = field(default=False, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def __post_init__(self) -> None:
        for name in (
            "grant_id", "task_description", "checkpoint_identity", "config_identity",
            "normalization_identity", "dataset_identity",
        ):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} is required")
        if type(self.gripper_effort_raw) is not int or not 0 <= self.gripper_effort_raw <= 5000:
            raise ValueError("gripper_effort_raw must be an integer in [0, 5000]")
        if type(self.issued_at_monotonic_ns) is not int or self.issued_at_monotonic_ns <= 0:
            raise ValueError("issued_at_monotonic_ns must be positive")
        if (type(self.expires_at_monotonic_ns) is not int
                or self.expires_at_monotonic_ns <= self.issued_at_monotonic_ns):
            raise ValueError("grant expiry must follow its issue time")
        if (self.expires_at_monotonic_ns - self.issued_at_monotonic_ns
                > MAX_ONE_STEP_GRANT_LIFETIME_NS):
            raise ValueError("short-loop grant lifetime exceeds 30 seconds")
        if type(self.limits) is not SafetyLimits:
            raise TypeError("limits must be SafetyLimits")
        if type(self.site_gates) is not P12GateEvidence:
            raise TypeError("site_gates must be P12GateEvidence")
        if self.site_gates.missing:
            raise PermissionError("site verification is incomplete: "
                                  + ", ".join(self.site_gates.missing))
        if type(self.operator_confirmed) is not bool or not self.operator_confirmed:
            raise PermissionError("explicit operator confirmation is required")
        if type(self.max_steps) is not int or self.max_steps != SHORT_LOOP_MAX_STEPS:
            raise ValueError(f"short-loop step count must be exactly {SHORT_LOOP_MAX_STEPS}")

    @property
    def authorized_steps(self) -> int:
        with self._lock:
            return self._authorized_steps

    @property
    def steps_remaining(self) -> int:
        with self._lock:
            return self.max_steps - self._authorized_steps

    @property
    def finished(self) -> bool:
        with self._lock:
            return (self._authorized_steps == self.max_steps
                    and self._pending_proposal_id is None)

    def to_worker_ticket(self) -> ShortLoopOperatorGrantTicket:
        """Transfer this session consent once to a disposable worker process."""
        with self._lock:
            if self._transferred or self._authorized_steps:
                raise PermissionError("short-loop grant has already been transferred or used")
            object.__setattr__(self, "_transferred", True)
            return ShortLoopOperatorGrantTicket(
                grant_id=self.grant_id,
                task_description=self.task_description,
                checkpoint_identity=self.checkpoint_identity,
                config_identity=self.config_identity,
                normalization_identity=self.normalization_identity,
                dataset_identity=self.dataset_identity,
                gripper_effort_raw=self.gripper_effort_raw,
                issued_at_monotonic_ns=self.issued_at_monotonic_ns,
                expires_at_monotonic_ns=self.expires_at_monotonic_ns,
                limits=self.limits,
                site_gates=self.site_gates,
                operator_confirmed=self.operator_confirmed,
                max_steps=self.max_steps,
            )

    def authorize(
        self,
        prediction: PolicyPrediction,
        prepared: PreparedSupervisedPolicyStep,
        *,
        active_task_description: str,
        inference_started_monotonic_ns: int,
        limits: SafetyLimits,
        now_monotonic_ns: int | None = None,
    ) -> SupervisedPositionAuthorization:
        """Authorize one bounded batch inside the confirmed short-loop session."""
        if type(prediction) is not PolicyPrediction:
            raise TypeError("prediction must be a PolicyPrediction")
        if type(prepared) is not PreparedSupervisedPolicyStep:
            raise TypeError("prepared must be a PreparedSupervisedPolicyStep")
        if type(limits) is not SafetyLimits:
            raise TypeError("limits must be SafetyLimits")
        now_ns = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError("now_monotonic_ns must be positive")
        with self._lock:
            if self._transferred:
                raise PermissionError("short-loop grant was transferred to a worker")
            if self._authorized_steps >= self.max_steps:
                raise PermissionError("short-loop grant has no position steps remaining")
            if self._pending_proposal_id is not None:
                raise PermissionError("fresh post-send feedback is required before another step")
            if now_ns < self.issued_at_monotonic_ns or now_ns > self.expires_at_monotonic_ns:
                raise PermissionError("short-loop operator grant is not current")
            if self._confirmed_snapshot is not None:
                confirmed = self._confirmed_snapshot
                if (prepared.snapshot.sequence != confirmed.sequence
                        or prepared.snapshot.captured_monotonic_ns
                        < confirmed.captured_monotonic_ns
                        or prepared.snapshot.received_monotonic_ns
                        < confirmed.received_monotonic_ns):
                    raise PermissionError(
                        "next step must use the confirmed post-send state or a newer snapshot"
                    )
                if (self._last_write_ns is None
                        or prepared.snapshot.captured_monotonic_ns <= self._last_write_ns
                        or prepared.snapshot.received_monotonic_ns <= self._last_write_ns):
                    raise PermissionError(
                        "next step observation must follow the prior position batch"
                    )
            anchor = self._initial_state7 or prepared.snapshot.state7
            for initial, current, target, max_delta in zip(
                anchor,
                prepared.snapshot.state7,
                prepared.proposal.target,
                self.limits.max_joint_delta_rad + (self.limits.max_gripper_delta_m,),
            ):
                if abs(current - initial) > max_delta:
                    raise PermissionError(
                        "short-loop state exceeds the total displacement budget "
                        "from its initial state"
                    )
                if abs(target - initial) > max_delta:
                    raise PermissionError(
                        "short-loop target exceeds the total displacement budget "
                        "from its initial state"
                    )

            step_grant = OneStepOperatorGrant(
                grant_id=f"{self.grant_id}:step-{self._authorized_steps + 1}",
                task_description=self.task_description,
                checkpoint_identity=self.checkpoint_identity,
                config_identity=self.config_identity,
                normalization_identity=self.normalization_identity,
                dataset_identity=self.dataset_identity,
                gripper_effort_raw=self.gripper_effort_raw,
                issued_at_monotonic_ns=self.issued_at_monotonic_ns,
                expires_at_monotonic_ns=self.expires_at_monotonic_ns,
                limits=self.limits,
                site_gates=self.site_gates,
                operator_confirmed=self.operator_confirmed,
            )
            authorization = step_grant.authorize(
                prediction,
                prepared,
                active_task_description=active_task_description,
                inference_started_monotonic_ns=inference_started_monotonic_ns,
                limits=limits,
                now_monotonic_ns=now_ns,
            )
            object.__setattr__(self, "_authorized_steps", self._authorized_steps + 1)
            if self._initial_state7 is None:
                object.__setattr__(self, "_initial_state7", prepared.snapshot.state7)
            object.__setattr__(self, "_pending_proposal_id", prepared.proposal.proposal_id)
            object.__setattr__(self, "_pending_source_sequence", prepared.snapshot.sequence)
            return authorization

    def record_post_send_feedback(
        self,
        prepared: PreparedSupervisedPolicyStep,
        receipt: PositionBatchWriteReceipt,
        feedback_snapshot: RobotStateSnapshot,
        *,
        now_monotonic_ns: int | None = None,
    ) -> None:
        """Unlock the next session step only after fresh, healthy feedback."""
        if type(prepared) is not PreparedSupervisedPolicyStep:
            raise TypeError("prepared must be a PreparedSupervisedPolicyStep")
        if type(receipt) is not PositionBatchWriteReceipt:
            raise TypeError("receipt must be a PositionBatchWriteReceipt")
        if type(feedback_snapshot) is not RobotStateSnapshot:
            raise TypeError("feedback_snapshot must be a RobotStateSnapshot")
        now_ns = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError("now_monotonic_ns must be positive")
        with self._lock:
            if self._pending_proposal_id is None:
                raise PermissionError("there is no pending short-loop position batch")
            if (receipt.proposal_id != self._pending_proposal_id
                    or receipt.proposal_id != prepared.proposal.proposal_id
                    or receipt.source_state_sequence != self._pending_source_sequence
                    or receipt.target != prepared.proposal.target):
                raise PermissionError("write receipt does not match the pending proposal")
            receipt_frames = tuple((write.arbitration_id, write.data) for write in receipt.writes)
            prepared_frames = tuple((frame.arbitration_id, frame.data)
                                    for frame in prepared.batch.frames)
            if receipt_frames != prepared_frames:
                raise PermissionError("write receipt frame bytes differ from the prepared batch")
            latest_write_ns = receipt.writes[-1].adapter_accepted_monotonic_ns
            if latest_write_ns > now_ns:
                raise PermissionError("final adapter write timestamp is in the future")
            if (feedback_snapshot.captured_monotonic_ns <= latest_write_ns
                    or feedback_snapshot.received_monotonic_ns <= latest_write_ns):
                raise PermissionError(
                    "post-send observation does not follow the final adapter write"
                )
            if feedback_snapshot.sequence <= int(self._pending_source_sequence):
                raise PermissionError("post-send feedback state sequence did not advance")
            if (not feedback_snapshot.connected or not feedback_snapshot.status_ok
                    or not feedback_snapshot.units_verified):
                raise PermissionError("post-send robot feedback is not healthy and verified")
            if (feedback_snapshot.received_monotonic_ns > now_ns
                    or now_ns - feedback_snapshot.received_monotonic_ns
                    > self.limits.max_state_age_ns):
                raise PermissionError("post-send robot feedback is stale")
            for value, low, high in zip(
                feedback_snapshot.state7,
                self.limits.joint_min_rad + (self.limits.gripper_min_m,),
                self.limits.joint_max_rad + (self.limits.gripper_max_m,),
            ):
                if not low <= value <= high:
                    raise PermissionError("post-send feedback is outside the approved envelope")
            assert self._initial_state7 is not None
            for initial, value, max_delta in zip(
                self._initial_state7,
                feedback_snapshot.state7,
                self.limits.max_joint_delta_rad + (self.limits.max_gripper_delta_m,),
            ):
                if abs(value - initial) > max_delta:
                    raise PermissionError(
                        "post-send feedback exceeds the total displacement budget "
                        "from its initial state"
                    )
            object.__setattr__(self, "_confirmed_snapshot", feedback_snapshot)
            object.__setattr__(self, "_last_write_ns", latest_write_ns)
            object.__setattr__(self, "_pending_proposal_id", None)
            object.__setattr__(self, "_pending_source_sequence", None)


@dataclass(frozen=True)
class ShortLoopOperatorGrantTicket:
    """Pickle-safe transfer for one explicitly approved 3-step session."""

    grant_id: str
    task_description: str
    checkpoint_identity: str
    config_identity: str
    normalization_identity: str
    dataset_identity: str
    gripper_effort_raw: int
    issued_at_monotonic_ns: int
    expires_at_monotonic_ns: int
    limits: SafetyLimits
    site_gates: P12GateEvidence
    operator_confirmed: bool
    max_steps: int

    def materialize(self) -> ShortLoopOperatorGrant:
        return ShortLoopOperatorGrant(
            grant_id=self.grant_id,
            task_description=self.task_description,
            checkpoint_identity=self.checkpoint_identity,
            config_identity=self.config_identity,
            normalization_identity=self.normalization_identity,
            dataset_identity=self.dataset_identity,
            gripper_effort_raw=self.gripper_effort_raw,
            issued_at_monotonic_ns=self.issued_at_monotonic_ns,
            expires_at_monotonic_ns=self.expires_at_monotonic_ns,
            limits=self.limits,
            site_gates=self.site_gates,
            operator_confirmed=self.operator_confirmed,
            max_steps=self.max_steps,
        )


@dataclass(frozen=True)
class OneStepOperatorGrantTicket:
    """Pickle-safe one-use consent transfer for one bounded worker process."""

    grant_id: str
    task_description: str
    checkpoint_identity: str
    config_identity: str
    normalization_identity: str
    dataset_identity: str
    gripper_effort_raw: int
    issued_at_monotonic_ns: int
    expires_at_monotonic_ns: int
    limits: SafetyLimits
    site_gates: P12GateEvidence
    operator_confirmed: bool

    def materialize(self) -> OneStepOperatorGrant:
        """Rebuild a process-local grant after the parent transfers consent."""
        return OneStepOperatorGrant(
            grant_id=self.grant_id,
            task_description=self.task_description,
            checkpoint_identity=self.checkpoint_identity,
            config_identity=self.config_identity,
            normalization_identity=self.normalization_identity,
            dataset_identity=self.dataset_identity,
            gripper_effort_raw=self.gripper_effort_raw,
            issued_at_monotonic_ns=self.issued_at_monotonic_ns,
            expires_at_monotonic_ns=self.expires_at_monotonic_ns,
            limits=self.limits,
            site_gates=self.site_gates,
            operator_confirmed=self.operator_confirmed,
        )


def issue_one_step_operator_grant(
    *,
    task_description: str,
    checkpoint_identity: str,
    config_identity: str,
    normalization_identity: str,
    dataset_identity: str,
    gripper_effort_raw: int,
    limits: SafetyLimits,
    site_gates: P12GateEvidence,
    confirmation_phrase: str,
    now_monotonic_ns: int | None = None,
    lifetime_ns: int = DEFAULT_ONE_STEP_GRANT_LIFETIME_NS,
) -> OneStepOperatorGrant:
    """Record explicit approval for one bounded batch before policy inference."""
    if confirmation_phrase != ONE_STEP_CONFIRMATION_PHRASE:
        raise PermissionError("operator did not confirm the one-step physical action")
    if type(lifetime_ns) is not int or not 0 < lifetime_ns <= MAX_ONE_STEP_GRANT_LIFETIME_NS:
        raise ValueError("grant lifetime must be within 30 seconds")
    issued_ns = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
    return OneStepOperatorGrant(
        grant_id=uuid4().hex,
        task_description=task_description,
        checkpoint_identity=checkpoint_identity,
        config_identity=config_identity,
        normalization_identity=normalization_identity,
        dataset_identity=dataset_identity,
        gripper_effort_raw=gripper_effort_raw,
        issued_at_monotonic_ns=issued_ns,
        expires_at_monotonic_ns=issued_ns + lifetime_ns,
        limits=limits,
        site_gates=site_gates,
        operator_confirmed=True,
    )


def request_one_step_operator_grant(
    *,
    task_description: str,
    checkpoint_identity: str,
    config_identity: str,
    normalization_identity: str,
    dataset_identity: str,
    gripper_effort_raw: int,
    limits: SafetyLimits,
    site_gates: P12GateEvidence,
    input_fn: Callable[[str], str] = input,
    output_fn: Callable[[str], object] = print,
    lifetime_ns: int = DEFAULT_ONE_STEP_GRANT_LIFETIME_NS,
) -> OneStepOperatorGrant:
    """Show the bounded operation and require an exact local confirmation."""
    if type(limits) is not SafetyLimits:
        raise TypeError("limits must be SafetyLimits")
    if type(site_gates) is not P12GateEvidence:
        raise TypeError("site_gates must be P12GateEvidence")
    if type(lifetime_ns) is not int or not 0 < lifetime_ns <= MAX_ONE_STEP_GRANT_LIFETIME_NS:
        raise ValueError("grant lifetime must be within 30 seconds")
    if site_gates.missing:
        raise PermissionError("site verification is incomplete: "
                              + ", ".join(site_gates.missing))
    scope = (
        "This approves one 7D absolute-position batch for the task below. "
        "The policy chooses the target from the next fresh observation. "
        "The batch must stay inside the displayed position and per-step limits. "
        "No second batch is authorized."
    )
    lines = (
        scope,
        f"Task: {task_description}",
        f"Checkpoint: {checkpoint_identity}",
        f"Policy config: {config_identity}",
        f"Normalization: {normalization_identity}",
        f"Dataset: {dataset_identity}",
        f"Gripper SDK effort setting (raw): {gripper_effort_raw}",
        f"Joint position limits (rad): {limits.joint_min_rad} .. {limits.joint_max_rad}",
        f"Joint single-step deltas (rad): {limits.max_joint_delta_rad}",
        f"Gripper position limits (m): {limits.gripper_min_m} .. {limits.gripper_max_m}",
        f"Gripper single-step delta (m): {limits.max_gripper_delta_m}",
        "P12 site checks asserted complete by the caller: "
        + ", ".join(name for name, value in asdict(site_gates).items() if value),
        "The software cannot verify those site checks; confirm only after the "
        "onsite checklist is complete.",
        f"Grant lifetime after confirmation (s): {lifetime_ns / 1e9:g}",
        f'Type "{ONE_STEP_CONFIRMATION_PHRASE}" to approve, or anything else to cancel.',
    )
    for line in lines:
        output_fn(line)
    phrase = input_fn("> ")
    return issue_one_step_operator_grant(
        task_description=task_description,
        checkpoint_identity=checkpoint_identity,
        config_identity=config_identity,
        normalization_identity=normalization_identity,
        dataset_identity=dataset_identity,
        gripper_effort_raw=gripper_effort_raw,
        limits=limits,
        site_gates=site_gates,
        confirmation_phrase=phrase,
        lifetime_ns=lifetime_ns,
    )


def issue_short_loop_operator_grant(
    *,
    task_description: str,
    checkpoint_identity: str,
    config_identity: str,
    normalization_identity: str,
    dataset_identity: str,
    gripper_effort_raw: int,
    limits: SafetyLimits,
    site_gates: P12GateEvidence,
    confirmation_phrase: str,
    now_monotonic_ns: int | None = None,
    lifetime_ns: int = DEFAULT_ONE_STEP_GRANT_LIFETIME_NS,
) -> ShortLoopOperatorGrant:
    """Issue explicit consent for at most three feedback-coupled batches."""
    if confirmation_phrase != SHORT_LOOP_CONFIRMATION_PHRASE:
        raise PermissionError("operator did not confirm the three-step policy session")
    if type(lifetime_ns) is not int or not 0 < lifetime_ns <= MAX_ONE_STEP_GRANT_LIFETIME_NS:
        raise ValueError("short-loop grant lifetime must be within 30 seconds")
    issued_ns = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
    return ShortLoopOperatorGrant(
        grant_id=uuid4().hex,
        task_description=task_description,
        checkpoint_identity=checkpoint_identity,
        config_identity=config_identity,
        normalization_identity=normalization_identity,
        dataset_identity=dataset_identity,
        gripper_effort_raw=gripper_effort_raw,
        issued_at_monotonic_ns=issued_ns,
        expires_at_monotonic_ns=issued_ns + lifetime_ns,
        limits=limits,
        site_gates=site_gates,
        operator_confirmed=True,
        max_steps=SHORT_LOOP_MAX_STEPS,
    )


def request_short_loop_operator_grant(
    *,
    task_description: str,
    checkpoint_identity: str,
    config_identity: str,
    normalization_identity: str,
    dataset_identity: str,
    gripper_effort_raw: int,
    limits: SafetyLimits,
    site_gates: P12GateEvidence,
    input_fn: Callable[[str], str] = input,
    output_fn: Callable[[str], object] = print,
    lifetime_ns: int = DEFAULT_ONE_STEP_GRANT_LIFETIME_NS,
) -> ShortLoopOperatorGrant:
    """Display a fixed three-step budget and require exact session consent."""
    if type(limits) is not SafetyLimits:
        raise TypeError("limits must be SafetyLimits")
    if type(site_gates) is not P12GateEvidence:
        raise TypeError("site_gates must be P12GateEvidence")
    if type(lifetime_ns) is not int or not 0 < lifetime_ns <= MAX_ONE_STEP_GRANT_LIFETIME_NS:
        raise ValueError("short-loop grant lifetime must be within 30 seconds")
    if site_gates.missing:
        raise PermissionError("site verification is incomplete: "
                              + ", ".join(site_gates.missing))
    lines = (
        "This approves at most three successive 7D absolute-position batches.",
        "Each batch uses a new policy inference and must stay inside the displayed limits.",
        "Total displacement from the initial measured state stays within one "
        "displayed per-step delta.",
        "A new healthy feedback snapshot after the previous batch is required before the next.",
        "Any send error, timeout, or missing fresh feedback ends the session; do not retry.",
        f"Task: {task_description}",
        f"Checkpoint: {checkpoint_identity}",
        f"Policy config: {config_identity}",
        f"Normalization: {normalization_identity}",
        f"Dataset: {dataset_identity}",
        f"Gripper SDK effort setting (raw): {gripper_effort_raw}",
        f"Joint position limits (rad): {limits.joint_min_rad} .. {limits.joint_max_rad}",
        f"Per-step joint deltas (rad): {limits.max_joint_delta_rad}",
        f"Gripper position limits (m): {limits.gripper_min_m} .. {limits.gripper_max_m}",
        f"Per-step gripper delta (m): {limits.max_gripper_delta_m}",
        "P12 site checks asserted complete by the caller: "
        + ", ".join(name for name, value in asdict(site_gates).items() if value),
        "The software cannot verify those site checks; confirm only after the "
        "onsite checklist is complete.",
        f"Session lifetime after confirmation (s): {lifetime_ns / 1e9:g}",
        f'Type "{SHORT_LOOP_CONFIRMATION_PHRASE}" to approve, or anything else to cancel.',
    )
    for line in lines:
        output_fn(line)
    phrase = input_fn("> ")
    return issue_short_loop_operator_grant(
        task_description=task_description,
        checkpoint_identity=checkpoint_identity,
        config_identity=config_identity,
        normalization_identity=normalization_identity,
        dataset_identity=dataset_identity,
        gripper_effort_raw=gripper_effort_raw,
        limits=limits,
        site_gates=site_gates,
        confirmation_phrase=phrase,
        lifetime_ns=lifetime_ns,
    )
