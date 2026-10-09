"""Prepare and, with a one-use approval, send one derived policy target.

The prepare function has no hardware side effects. Sending is a separate call
that requires the live SafetyBridge and an authorization bound to the exact
proposal. This module does not verify the truth of physical-site attestations.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

from laptop.control.contracts import (
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
    TimingContract,
)
from laptop.robot_client.policy_sdk_preview import (
    PolicySdkPreview,
    preview_policy_sdk_values,
)
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.verified_policy_command import (
    command_proposal_from_verified_preview,
)
from laptop.safety.bridge import SafetyBridge, SafetyLimits
from laptop.transport.position_commands import (
    CanTxFrame,
    PositionBatchWriteReceipt,
    PositionCommandBatch,
    SupervisedPositionAuthorization,
)


@dataclass(frozen=True)
class PreparedSupervisedPolicyStep:
    """A checked proposal and its four encoded frames, not yet transmitted."""

    snapshot: RobotStateSnapshot
    preview: PolicySdkPreview
    proposal: CommandProposal
    batch: PositionCommandBatch

    def __post_init__(self) -> None:
        if type(self.snapshot) is not RobotStateSnapshot:
            raise TypeError("snapshot must be a RobotStateSnapshot")
        if type(self.preview) is not PolicySdkPreview:
            raise TypeError("preview must be a PolicySdkPreview")
        if type(self.proposal) is not CommandProposal:
            raise TypeError("proposal must be a CommandProposal")
        if type(self.batch) is not PositionCommandBatch:
            raise TypeError("batch must be a PositionCommandBatch")
        if self.batch.proposal != self.proposal:
            raise ValueError("batch is not bound to the prepared proposal")
        if (self.preview.source_state_sequence != self.snapshot.sequence
                or self.proposal.source_state_sequence != self.snapshot.sequence):
            raise ValueError("prepared step does not match its source state sequence")
        if self.preview.request_id != self.proposal.source_request_id:
            raise ValueError("prepared step does not match its policy request")


def prepare_supervised_policy_step(
    prediction: PolicyPrediction,
    *,
    snapshot: RobotStateSnapshot,
    evidence: P12GateEvidence,
    limits: SafetyLimits,
    gripper_effort_raw: int,
    timing: TimingContract = TimingContract(),
) -> PreparedSupervisedPolicyStep:
    """Convert one fresh future prediction row into a bounded four-frame batch.

    This function does not open CAN. It requires all P12 evidence before it
    creates a command proposal. The caller must still obtain a one-use operator
    authorization and use :func:`send_prepared_supervised_policy_step`.
    """
    if type(prediction) is not PolicyPrediction:
        raise TypeError("prediction must be a typed PolicyPrediction")
    if len(prediction.actions) != 16 or len(prediction.target_dt_s) != 16:
        raise ValueError("supervised policy step requires a complete 16x7 prediction")
    if type(snapshot) is not RobotStateSnapshot:
        raise TypeError("snapshot must be a RobotStateSnapshot")
    if type(evidence) is not P12GateEvidence:
        raise TypeError("evidence must be P12GateEvidence")
    if evidence.missing:
        raise PermissionError("site verification is incomplete: " + ", ".join(evidence.missing))
    if type(limits) is not SafetyLimits:
        raise TypeError("limits must be SafetyLimits")
    if type(timing) is not TimingContract:
        raise TypeError("timing must be a TimingContract")
    if prediction.request_sequence != snapshot.sequence:
        raise ValueError("prediction request sequence does not match current state")

    now_ns = time.perf_counter_ns()
    preview = preview_policy_sdk_values(
        prediction,
        current_state=snapshot,
        now_monotonic_ns=now_ns,
        gripper_effort_raw=gripper_effort_raw,
        max_joint_delta_rad=limits.max_joint_delta_rad,
        max_gripper_delta_m=limits.max_gripper_delta_m,
        configured_gripper_max_mm=limits.gripper_max_m * 1000.0,
        timing=timing,
    )
    proposal = command_proposal_from_verified_preview(
        preview,
        snapshot=snapshot,
        evidence=evidence,
        limits=limits,
        now_monotonic_ns=now_ns,
        timing=timing,
    )
    frames = tuple(
        CanTxFrame(frame.arbitration_id, frame.data)
        for frame in preview.frames
    )
    batch = PositionCommandBatch(proposal, frames)
    return PreparedSupervisedPolicyStep(
        snapshot=snapshot,
        preview=preview,
        proposal=proposal,
        batch=batch,
    )


def send_prepared_supervised_policy_step(
    prepared: PreparedSupervisedPolicyStep,
    *,
    bridge: SafetyBridge,
    authorization: SupervisedPositionAuthorization,
) -> PositionBatchWriteReceipt:
    """Transmit one prepared batch through SafetyBridge after explicit approval.

    The bridge rechecks freshness, limits, exact proposal binding, runtime mode,
    and the one-use authorization immediately before writing the four frames.
    """
    if type(prepared) is not PreparedSupervisedPolicyStep:
        raise TypeError("prepared must be a PreparedSupervisedPolicyStep")
    if type(bridge) is not SafetyBridge:
        raise TypeError("bridge must be the project's SafetyBridge")
    if type(authorization) is not SupervisedPositionAuthorization:
        raise TypeError("authorization must be a SupervisedPositionAuthorization")
    return bridge.send_supervised_position_batch(
        snapshot=prepared.snapshot,
        batch=prepared.batch,
        authorization=authorization,
    )
