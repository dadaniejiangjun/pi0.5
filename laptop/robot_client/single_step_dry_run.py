"""Offline single-step proposal rehearsal through SafetyBridge only.

The rehearsal never switches robot mode, enables a motor, opens CAN, sends a
command, or claims that physical stop/watchdog behavior is verified.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from typing import Sequence

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
)
from laptop.robot_client.joint_command_preview import preview_single_joint_step
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import FakeCanTransport


@dataclass(frozen=True)
class P12GateEvidence:
    """Offline summary of physical prerequisites; never a transmit token.

    Defaults are intentionally false. Even an all-true fixture only lets tests
    exercise the SafetyBridge preflight; this module accepts FakeCanTransport
    only and cannot enable CAN TX.

    ``command_target_source_verified`` is an operator attestation about the
    selected policy target's semantics and source; this class does not verify
    referenced evidence. For the derived-position route, the evidence may cite
    a checksum-bound next-observed-state proxy; original CAN command labels
    are not required. The separate ``policy_to_command_mapping_verified`` gate
    still requires physical validation before any supervised robot motion.
    """

    firmware_protocol_verified: bool = False
    joint_state_units_verified: bool = False
    gripper_units_verified: bool = False
    controller_limits_verified: bool = False
    mechanical_limits_verified: bool = False
    speed_acceleration_units_verified: bool = False
    low_speed_setting_verified: bool = False
    operator_and_estop_ready: bool = False
    watchdog_verified: bool = False
    disconnect_safe_hold_verified: bool = False
    camera_state_sync_verified: bool = False
    command_target_source_verified: bool = False
    policy_to_command_mapping_verified: bool = False

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if type(value) is not bool:
                raise ValueError(f"{name} must be a boolean")

    @property
    def missing(self) -> tuple[str, ...]:
        return tuple(name for name, value in asdict(self).items() if not value)


def preflight_hil_single_step(
    transport: FakeCanTransport,
    *,
    snapshot: RobotStateSnapshot,
    proposal: CommandProposal,
    p12_evidence: P12GateEvidence | None = None,
    operator_approved: bool = False,
    now_monotonic_ns: int,
) -> dict:
    """Run the P12 single-step preflight through SafetyBridge, without TX.

    This is the offline HIL gate path, not a hardware executor. It requires an
    exact FakeCanTransport and a CommandProposal; a PolicyPrediction is never
    accepted or converted. Physical P12 evidence and operator approval are
    recorded as blockers, while SafetyBridge still checks freshness, units,
    timing, sequence and software limits.
    """
    if type(transport) is not FakeCanTransport:
        raise TypeError("P12 offline preflight accepts only the exact FakeCanTransport")
    if not isinstance(snapshot, RobotStateSnapshot):
        raise TypeError("snapshot must be a RobotStateSnapshot")
    if type(proposal) is not CommandProposal:
        if isinstance(proposal, PolicyPrediction):
            raise TypeError("next-state proxy predictions are not HIL command proposals")
        raise TypeError("proposal must be a CommandProposal")
    if type(operator_approved) is not bool:
        raise ValueError("operator_approved must be boolean")
    evidence = p12_evidence or P12GateEvidence()
    if type(evidence) is not P12GateEvidence:
        raise TypeError("p12_evidence must be a P12GateEvidence")
    if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
        raise ValueError("now_monotonic_ns must be a positive laptop monotonic timestamp")

    send_attempts_before = transport.send_attempts
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
    )
    bridge.record_heartbeat(max(1, now_monotonic_ns - 1))
    try:
        decision = bridge.inspect_command_proposal(
            snapshot=snapshot,
            proposal=proposal,
            now_monotonic_ns=now_monotonic_ns,
        )
        blockers = list(evidence.missing)
        if not operator_approved:
            blockers.append("operator_single_step_approval_missing")
        blockers.extend(
            reason for reason in decision.reasons
            if reason != "runtime_mode_forbids_can_tx"
        )
        blockers = list(dict.fromkeys(blockers))
        status = (
            "OFFLINE_PREFLIGHT_CLEAR_TX_STILL_DISABLED"
            if not blockers else "OFFLINE_PREFLIGHT_BLOCKED"
        )
        audit_input = {
            "schema": "piper_p12_single_step_preflight_audit_v1",
            "evaluation_scope": "fake_can_offline_only",
            "runtime_mode": RuntimeMode.POLICY_REJECT_ONLY.value,
            "status": status,
            "decision_monotonic_ns": now_monotonic_ns,
            "snapshot": asdict(snapshot),
            "command_proposal": asdict(proposal),
            "p12_gate_evidence": asdict(evidence),
            "operator_approved": operator_approved,
            "blockers": blockers,
            "safety_decision": decision.audit_event(),
            "transport": {
                "implementation": type(transport).__name__,
                "connected": transport.connected,
                "send_attempts_before": send_attempts_before,
                "send_attempts_after": transport.send_attempts,
            },
            "hardware_control_allowed": False,
            "physical_p12_gates_verified": False,
            "readiness_for_robot_command": False,
            "can_tx_enabled": False,
            "transport_send_attempted": False,
        }
        audit_bytes = json.dumps(
            audit_input,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        preflight_audit = {
            **audit_input,
            "input_sha256": hashlib.sha256(audit_bytes).hexdigest(),
        }
        return {
            "schema": "piper_p12_single_step_preflight_v1",
            "evaluation_scope": "fake_can_offline_only",
            "status": status,
            "runtime_mode": RuntimeMode.POLICY_REJECT_ONLY.value,
            "p12_gate_evidence": asdict(evidence),
            "p12_gate_missing": list(evidence.missing),
            "operator_approved": operator_approved,
            "blockers": blockers,
            "safety_decision": decision.audit_event(),
            "preflight_audit": preflight_audit,
            "hardware_control_allowed": False,
            "physical_p12_gates_verified": False,
            "readiness_for_robot_command": False,
            "can_tx_enabled": False,
            "transport_send_attempted": False,
            "fake_transport_send_attempts": transport.send_attempts,
        }
    finally:
        bridge.close()


def rehearse_single_step(
    transport: FakeCanTransport,
    *,
    joint_index: int,
    delta_deg: float,
    state_units_verified: bool = False,
    mapping_verified: bool = False,
) -> dict:
    """Validate one nominal step; SafetyBridge remains the sole decision point.

    The flags default false because this offline rehearsal cannot verify the
    physical joint/encoder mapping or gripper units. Even when supplied true
    for fault-injection tests, RuntimeMode.POLICY_REJECT_ONLY still prohibits
    CAN TX and this routine still uses the exact FakeCanTransport only.
    """
    if type(transport) is not FakeCanTransport:
        raise TypeError("only the exact FakeCanTransport is accepted")
    preview = preview_single_joint_step(
        (0.0,) * 6, joint_index=joint_index, delta_deg=delta_deg
    )
    target = tuple(math.radians(value) for value in preview.target_deg) + (0.0,)
    result = _rehearse_proposal(
        transport,
        current_joints_rad=(0.0,) * 6,
        current_gripper_m=0.0,
        target=target,
        proposal_id="offline-joint-step-1",
        state_units_verified=state_units_verified,
        mapping_verified=mapping_verified,
    )
    result["target_joint_degrees_nominal"] = list(preview.target_deg)
    return result


def rehearse_gripper_step(
    transport: FakeCanTransport,
    *,
    current_gripper_m: float = 0.0,
    delta_m: float = 0.001,
    state_units_verified: bool = False,
    mapping_verified: bool = False,
) -> dict:
    """Validate a synthetic gripper-only step through the same SafetyBridge.

    The inputs are analysis values, not measured PiPER units. This routine
    accepts only an exact FakeCanTransport and never sends a CAN frame.
    """
    if type(transport) is not FakeCanTransport:
        raise TypeError("only the exact FakeCanTransport is accepted")
    values = (current_gripper_m, delta_m)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(float(value)) for value in values):
        raise ValueError("finite current gripper value and step are required")
    if delta_m == 0:
        raise ValueError("gripper step must be non-zero")
    target_gripper_m = float(current_gripper_m) + float(delta_m)
    result = _rehearse_proposal(
        transport,
        current_joints_rad=(0.0,) * 6,
        current_gripper_m=float(current_gripper_m),
        target=(0.0,) * 6 + (target_gripper_m,),
        proposal_id="offline-gripper-step-1",
        state_units_verified=state_units_verified,
        mapping_verified=mapping_verified,
    )
    result["target_gripper_m_nominal"] = target_gripper_m
    return result


def _rehearse_proposal(
    transport: FakeCanTransport,
    *,
    current_joints_rad: Sequence[float],
    current_gripper_m: float,
    target: Sequence[float],
    proposal_id: str,
    state_units_verified: bool,
    mapping_verified: bool,
) -> dict:
    now_ns = time.perf_counter_ns()
    sequence = 1
    state = RobotStateSnapshot(
        sequence=sequence,
        captured_monotonic_ns=now_ns - 10_000_000,
        received_monotonic_ns=now_ns - 5_000_000,
        joint_position_rad=tuple(current_joints_rad),
        gripper_position_m=current_gripper_m,
        status_ok=True,
        connected=True,
        units_verified=state_units_verified,
    )
    proposal = CommandProposal(
        proposal_id=proposal_id,
        source_request_id=f"offline-request-{sequence}",
        source_state_sequence=sequence,
        created_monotonic_ns=now_ns - 1_000_000,
        valid_until_monotonic_ns=now_ns + 100_000_000,
        target=target,
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=state_units_verified,
        mapping_verified=mapping_verified,
    )
    result = preflight_hil_single_step(
        transport,
        snapshot=state,
        proposal=proposal,
        now_monotonic_ns=now_ns,
    )
    decision = result["safety_decision"]
    result.update({
        "status": "SAFETY_BRIDGE_REJECTED_NON_EXECUTABLE",
        "target_nominal": list(target),
        "bridge_decision": decision,
        "reasons": list(decision["reasons"]),
        "accepted_for_offline_analysis": decision["accepted_for_offline_analysis"],
        "real_send_attempts": 0,
    })
    return result
