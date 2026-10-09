"""Turn a verified Pi0.5 SDK preview into a short-lived command proposal.

This is the explicit semantic boundary between a derived next-state target
and an absolute PiPER position command. It creates no CAN frame and sends
nothing. A complete site-verification record is required before the target
can be represented as a command proposal.
"""

from __future__ import annotations

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    RobotStateSnapshot,
    TimingContract,
)
from laptop.robot_client.policy_sdk_preview import (
    DERIVED_ACTION_TARGET_KIND,
    PolicySdkPreview,
)
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.safety.bridge import SafetyLimits


def command_proposal_from_verified_preview(
    preview: PolicySdkPreview,
    *,
    snapshot: RobotStateSnapshot,
    evidence: P12GateEvidence,
    limits: SafetyLimits,
    now_monotonic_ns: int,
    timing: TimingContract = TimingContract(),
) -> CommandProposal:
    """Create a bounded-lifetime position proposal after all site gates pass.

    The caller must obtain ``preview`` from the typed SDK preview path and pass
    an independently reviewed ``P12GateEvidence`` record. The evidence object
    contains operator attestations; this function cannot inspect or authenticate
    the underlying site checks. It only creates a typed proposal; the current
    shared CAN transport still rejects generic command sends.
    """
    if type(preview) is not PolicySdkPreview:
        raise TypeError("preview must be a PolicySdkPreview")
    if type(snapshot) is not RobotStateSnapshot:
        raise TypeError("snapshot must be a RobotStateSnapshot")
    if type(evidence) is not P12GateEvidence:
        raise TypeError("evidence must be a P12GateEvidence")
    if type(limits) is not SafetyLimits:
        raise TypeError("limits must be a SafetyLimits instance")
    if type(timing) is not TimingContract:
        raise TypeError("timing must be a TimingContract")
    if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
        raise ValueError("now_monotonic_ns must be a positive integer")
    if evidence.missing:
        raise PermissionError("site verification is incomplete: " + ", ".join(evidence.missing))
    if not snapshot.units_verified:
        raise PermissionError("current PiPER state units are not verified")
    if not snapshot.connected or not snapshot.status_ok:
        raise ValueError("current PiPER state is disconnected or reports an error")
    if snapshot.received_monotonic_ns > now_monotonic_ns:
        raise ValueError("current PiPER state was received after the local clock")
    if now_monotonic_ns - snapshot.received_monotonic_ns > timing.max_state_age_ns:
        raise ValueError("current PiPER state is stale")
    if preview.source_state_sequence != snapshot.sequence:
        raise ValueError("preview was evaluated against a different PiPER state")
    if preview.action_semantics != "next_observed_state_proxy_nominal_units":
        raise ValueError("preview does not contain a derived next-state target")
    if preview.action_target_kind != DERIVED_ACTION_TARGET_KIND:
        raise ValueError("unsupported policy target kind")
    if (preview.hardware_control_allowed or preview.can_tx_allowed
            or preview.action_units_verified_on_hardware
            or preview.policy_to_robot_command_mapping_verified):
        raise ValueError("SDK preview must remain non-executable and nominal")
    if preview.evaluated_monotonic_ns < snapshot.received_monotonic_ns:
        raise ValueError("preview predates its source state")
    if now_monotonic_ns < preview.evaluated_monotonic_ns:
        raise ValueError("preview evaluation time is in the future")
    if (now_monotonic_ns - preview.evaluated_monotonic_ns
            > timing.max_prediction_response_latency_ns):
        raise ValueError("policy preview is stale")
    if preview.target_monotonic_ns <= now_monotonic_ns:
        raise ValueError("policy target has expired")
    if preview.configured_gripper_max_mm is None:
        raise ValueError("verified gripper travel limit is required")
    if preview.gripper_target_exceeds_configured_max is not False:
        raise ValueError("gripper target exceeds or was not checked against its verified limit")

    current_joints = snapshot.joint_position_rad
    target_joints = preview.joint_target_rad_nominal
    for index, (current, target, low, high, max_delta) in enumerate(zip(
        current_joints,
        target_joints,
        limits.joint_min_rad,
        limits.joint_max_rad,
        limits.max_joint_delta_rad,
    ), start=1):
        if not low <= current <= high:
            raise ValueError(f"joint_{index} feedback is outside the verified position range")
        if not low <= target <= high:
            raise ValueError(f"joint_{index} target is outside the verified position range")
        if abs(target - current) > max_delta:
            raise ValueError(f"joint_{index} target exceeds the verified single-step limit")
    current_gripper = snapshot.gripper_position_m
    target_gripper = preview.gripper_target_m_nominal
    if not limits.gripper_min_m <= current_gripper <= limits.gripper_max_m:
        raise ValueError("gripper feedback is outside the verified position range")
    if not limits.gripper_min_m <= target_gripper <= limits.gripper_max_m:
        raise ValueError("gripper target is outside the verified position range")
    if abs(target_gripper - current_gripper) > limits.max_gripper_delta_m:
        raise ValueError("gripper target exceeds the verified single-step limit")

    valid_until = min(
        now_monotonic_ns + timing.max_proposal_validity_ns,
        preview.target_monotonic_ns,
    )
    if valid_until <= now_monotonic_ns:
        raise ValueError("no valid command lifetime remains before the policy target")

    return CommandProposal(
        proposal_id=f"{preview.request_id}:{preview.action_index}:{now_monotonic_ns}",
        source_request_id=preview.request_id,
        source_state_sequence=snapshot.sequence,
        created_monotonic_ns=now_monotonic_ns,
        valid_until_monotonic_ns=valid_until,
        target=preview.joint_target_rad_nominal + (preview.gripper_target_m_nominal,),
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=True,
        mapping_verified=True,
    )
