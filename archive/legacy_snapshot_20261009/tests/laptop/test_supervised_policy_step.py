from __future__ import annotations

import time
from dataclasses import replace

import pytest

from laptop.control.contracts import PolicyPrediction, RobotStateSnapshot
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.supervised_policy_step import (
    prepare_supervised_policy_step,
    send_prepared_supervised_policy_step,
)
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, SafetyLimits
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.position_commands import (
    P12_SITE_GATE_NAMES,
    CanTxFrame,
    SupervisedPositionAuthorization,
)


class MemoryPositionAdapter:
    """In-memory adapter; this fixture cannot access a CAN device."""

    def __init__(self) -> None:
        self.sent: list[CanTxFrame] = []
        self.closed = False

    def open_read_only(self) -> None:
        pass

    def receive_frame(self, timeout_ms: int):
        return None

    def send_diagnostic_query(self, query) -> bool:
        return True

    def send_position_frame(self, frame: CanTxFrame) -> bool:
        self.sent.append(frame)
        return True

    def close(self) -> None:
        self.closed = True


def _fixture():
    now = time.perf_counter_ns()
    observation_ns = now - 5_000_000
    snapshot = RobotStateSnapshot(
        sequence=17,
        captured_monotonic_ns=observation_ns,
        received_monotonic_ns=observation_ns + 1_000_000,
        joint_position_rad=(0.0,) * 6,
        gripper_position_m=0.05,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    action = (0.01, 0.01, -0.02, -0.02, 0.03, -0.03, 0.05)
    prediction = PolicyPrediction(
        request_id="policy-step-request-17",
        request_sequence=snapshot.sequence,
        observation_monotonic_ns=observation_ns,
        received_monotonic_ns=now - 1_000_000,
        actions=(action,) * 16,
        target_dt_s=tuple(0.1 * index for index in range(1, 17)),
        checkpoint_identity="checkpoint:fixture",
        config_identity="config:fixture",
        normalization_identity="normalization:fixture",
        dataset_identity="dataset:fixture",
    )
    limits = SafetyLimits(
        joint_min_rad=(-3.0,) * 6,
        joint_max_rad=(3.0,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.08,
        max_gripper_delta_m=0.02,
    )
    evidence = P12GateEvidence(**{name: True for name in P12_SITE_GATE_NAMES})
    return prediction, snapshot, limits, evidence


def _authorization(step) -> SupervisedPositionAuthorization:
    approved_ns = time.perf_counter_ns()
    return SupervisedPositionAuthorization(
        approval_id="test-one-use-approval",
        proposal_id=step.proposal.proposal_id,
        source_request_id=step.proposal.source_request_id,
        source_state_sequence=step.snapshot.sequence,
        target=step.proposal.target,
        approved_at_monotonic_ns=approved_ns,
        valid_until_monotonic_ns=min(
            approved_ns + 100_000_000,
            step.proposal.valid_until_monotonic_ns,
        ),
        operator_confirmed=True,
        site_gate_attestations=tuple((name, True) for name in P12_SITE_GATE_NAMES),
    )


def test_prediction_is_prepared_then_sent_as_one_fake_supervised_step():
    prediction, snapshot, limits, evidence = _fixture()
    prepared = prepare_supervised_policy_step(
        prediction,
        snapshot=snapshot,
        evidence=evidence,
        limits=limits,
        gripper_effort_raw=120,
    )

    assert prepared.proposal.source_state_sequence == snapshot.sequence
    assert prepared.preview.action_index == 0
    assert tuple(frame.arbitration_id for frame in prepared.batch.frames) == (
        0x155, 0x156, 0x157, 0x159,
    )
    assert prepared.preview.hardware_control_allowed is False

    adapter = MemoryPositionAdapter()
    bridge = SafetyBridge(
        mode=RuntimeMode.SUPERVISED_POLICY_STEP,
        transport=GsUsbPiperCanTransport(adapter_factory=lambda: adapter),
        limits=limits,
        supervised_position_tx_enabled=True,
    )
    bridge.start_read_only()
    bridge.record_heartbeat(time.perf_counter_ns())
    receipt = send_prepared_supervised_policy_step(
        prepared,
        bridge=bridge,
        authorization=_authorization(prepared),
    )

    assert tuple(frame.arbitration_id for frame in adapter.sent) == (
        0x155, 0x156, 0x157, 0x159,
    )
    assert receipt.approval_id == "test-one-use-approval"
    assert len(receipt.writes) == 4
    assert bridge.state is BridgeState.READY
    bridge.close()


def test_prepare_rejects_incomplete_site_gates_before_creating_batch():
    prediction, snapshot, limits, _ = _fixture()
    with pytest.raises(PermissionError, match="site verification is incomplete"):
        prepare_supervised_policy_step(
            prediction,
            snapshot=snapshot,
            evidence=P12GateEvidence(),
            limits=limits,
            gripper_effort_raw=120,
        )


def test_prepare_rejects_stale_state_sequence_before_creating_batch():
    prediction, snapshot, limits, evidence = _fixture()
    wrong_sequence = RobotStateSnapshot(
        sequence=snapshot.sequence + 1,
        captured_monotonic_ns=snapshot.captured_monotonic_ns,
        received_monotonic_ns=snapshot.received_monotonic_ns,
        joint_position_rad=snapshot.joint_position_rad,
        gripper_position_m=snapshot.gripper_position_m,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    with pytest.raises(ValueError, match="request sequence"):
        prepare_supervised_policy_step(
            prediction,
            snapshot=wrong_sequence,
            evidence=evidence,
            limits=limits,
            gripper_effort_raw=120,
        )


def test_prepare_rejects_incomplete_policy_horizon():
    prediction, snapshot, limits, evidence = _fixture()
    truncated = replace(
        prediction,
        actions=(prediction.actions[0],),
        target_dt_s=(prediction.target_dt_s[0],),
    )
    with pytest.raises(ValueError, match="complete 16x7 prediction"):
        prepare_supervised_policy_step(
            truncated,
            snapshot=snapshot,
            evidence=evidence,
            limits=limits,
            gripper_effort_raw=120,
        )
