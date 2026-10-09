from __future__ import annotations

from dataclasses import replace
import pickle
import time

import pytest

from laptop.control.contracts import PolicyPrediction, RobotStateSnapshot
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.supervised_policy_session import (
    ONE_STEP_CONFIRMATION_PHRASE,
    SHORT_LOOP_CONFIRMATION_PHRASE,
    request_one_step_operator_grant,
    request_short_loop_operator_grant,
)
from laptop.robot_client.supervised_policy_step import (
    prepare_supervised_policy_step,
    send_prepared_supervised_policy_step,
)
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, SafetyLimits
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.position_commands import CanTxFrame, P12_SITE_GATE_NAMES


TASK = "Move the black rod a single bounded step toward the green mat"


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


def _limits_and_gates():
    limits = SafetyLimits(
        joint_min_rad=(-3.0,) * 6,
        joint_max_rad=(3.0,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.08,
        max_gripper_delta_m=0.02,
    )
    site_gates = P12GateEvidence(**{name: True for name in P12_SITE_GATE_NAMES})
    return limits, site_gates


def _fixture(limits, site_gates, grant_issued_ns):
    time.sleep(0.002)
    started_ns = time.perf_counter_ns()
    inference_started_ns = started_ns - 500_000
    snapshot = RobotStateSnapshot(
        sequence=4,
        captured_monotonic_ns=started_ns - 4_000_000,
        received_monotonic_ns=started_ns - 3_000_000,
        joint_position_rad=(0.0,) * 6,
        gripper_position_m=0.05,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    prediction = PolicyPrediction(
        request_id="session-request-4",
        request_sequence=snapshot.sequence,
        observation_monotonic_ns=started_ns - 1_000_000,
        received_monotonic_ns=started_ns - 100_000,
        actions=((0.01, 0.01, -0.02, -0.02, 0.03, -0.03, 0.05),) * 16,
        target_dt_s=tuple(0.1 * index for index in range(1, 17)),
        checkpoint_identity="checkpoint:session-test",
        config_identity="config:session-test",
        normalization_identity="normalization:session-test",
        dataset_identity="dataset:session-test",
    )
    assert grant_issued_ns < inference_started_ns < prediction.received_monotonic_ns
    return inference_started_ns, prediction, snapshot


def _request_grant(
    limits,
    site_gates,
    phrase=ONE_STEP_CONFIRMATION_PHRASE,
    lifetime_ns=10_000_000_000,
):
    shown: list[str] = []
    grant = request_one_step_operator_grant(
        task_description=TASK,
        checkpoint_identity="checkpoint:session-test",
        config_identity="config:session-test",
        normalization_identity="normalization:session-test",
        dataset_identity="dataset:session-test",
        gripper_effort_raw=120,
        limits=limits,
        site_gates=site_gates,
        input_fn=lambda prompt: phrase,
        output_fn=shown.append,
        lifetime_ns=lifetime_ns,
    )
    assert TASK in "\n".join(shown)
    assert "No second batch is authorized." in "\n".join(shown)
    assert "software cannot verify those site checks" in "\n".join(shown)
    assert "Gripper SDK effort setting (raw): 120" in "\n".join(shown)
    return grant


def test_pre_inference_operator_grant_authorizes_and_sends_one_exact_batch():
    limits, site_gates = _limits_and_gates()
    grant = _request_grant(limits, site_gates)
    inference_started_ns, prediction, snapshot = _fixture(
        limits, site_gates, grant.issued_at_monotonic_ns,
    )
    prepared = prepare_supervised_policy_step(
        prediction,
        snapshot=snapshot,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    authorization = grant.authorize(
        prediction,
        prepared,
        active_task_description=TASK,
        inference_started_monotonic_ns=inference_started_ns,
        limits=limits,
    )

    assert grant.used is True
    assert authorization.approved_at_monotonic_ns >= prepared.proposal.created_monotonic_ns
    assert authorization.valid_until_monotonic_ns <= prepared.proposal.valid_until_monotonic_ns
    assert authorization.valid_until_monotonic_ns - authorization.approved_at_monotonic_ns <= 250_000_000

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
        authorization=authorization,
    )
    assert tuple(frame.arbitration_id for frame in adapter.sent) == (0x155, 0x156, 0x157, 0x159)
    assert receipt.approval_id == authorization.approval_id
    bridge.close()

    with pytest.raises(PermissionError, match="already been consumed"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=inference_started_ns,
            limits=limits,
        )


def test_operator_must_type_exact_phrase_and_complete_site_gates():
    limits, site_gates = _limits_and_gates()
    with pytest.raises(PermissionError, match="did not confirm"):
        _request_grant(limits, site_gates, phrase="yes")

    with pytest.raises(PermissionError, match="site verification is incomplete"):
        _request_grant(limits, P12GateEvidence())


def test_grant_transfers_once_to_picklable_worker_ticket():
    limits, site_gates = _limits_and_gates()
    grant = _request_grant(limits, site_gates)

    ticket = pickle.loads(pickle.dumps(grant.to_worker_ticket()))
    worker_grant = ticket.materialize()

    assert grant.used is True
    assert worker_grant.used is False
    assert worker_grant.gripper_effort_raw == 120
    with pytest.raises(PermissionError, match="already been consumed"):
        grant.to_worker_ticket()


def test_grant_rejects_wrong_task_and_artifact_without_consuming():
    limits, site_gates = _limits_and_gates()
    grant = _request_grant(limits, site_gates)
    inference_started_ns, prediction, snapshot = _fixture(
        limits, site_gates, grant.issued_at_monotonic_ns,
    )
    prepared = prepare_supervised_policy_step(
        prediction,
        snapshot=snapshot,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    with pytest.raises(PermissionError, match="must precede policy inference"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=grant.issued_at_monotonic_ns - 1,
            limits=limits,
        )
    with pytest.raises(PermissionError, match="active task"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description="Different task",
            inference_started_monotonic_ns=inference_started_ns,
            limits=limits,
        )
    with pytest.raises(PermissionError, match="SafetyLimits differ"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=inference_started_ns,
            limits=replace(limits, max_gripper_delta_m=0.01),
        )
    wrong_policy = replace(prediction, checkpoint_identity="checkpoint:other")
    with pytest.raises(PermissionError, match="artifact identity"):
        grant.authorize(
            wrong_policy,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=inference_started_ns,
            limits=limits,
        )
    assert grant.used is False


def test_grant_binds_the_gripper_effort_setpoint():
    limits, site_gates = _limits_and_gates()
    grant = _request_grant(limits, site_gates)
    inference_started_ns, prediction, snapshot = _fixture(
        limits, site_gates, grant.issued_at_monotonic_ns,
    )
    prepared = prepare_supervised_policy_step(
        prediction,
        snapshot=snapshot,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=121,
    )

    with pytest.raises(PermissionError, match="differs from the approved effort"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=inference_started_ns,
            limits=limits,
        )


def test_grant_expires_and_cannot_extend_the_command_deadline():
    limits, site_gates = _limits_and_gates()
    grant = _request_grant(limits, site_gates)
    inference_started_ns, prediction, snapshot = _fixture(
        limits, site_gates, grant.issued_at_monotonic_ns,
    )
    prepared = prepare_supervised_policy_step(
        prediction,
        snapshot=snapshot,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    with pytest.raises(PermissionError, match="not current"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=inference_started_ns,
            limits=limits,
            now_monotonic_ns=grant.expires_at_monotonic_ns + 1,
        )

    with pytest.raises(ValueError, match="30 seconds"):
        _request_grant(limits, site_gates, lifetime_ns=30_000_000_001)


def _request_short_loop_grant(limits, site_gates,
                              phrase=SHORT_LOOP_CONFIRMATION_PHRASE):
    shown: list[str] = []
    grant = request_short_loop_operator_grant(
        task_description=TASK,
        checkpoint_identity="checkpoint:session-test",
        config_identity="config:session-test",
        normalization_identity="normalization:session-test",
        dataset_identity="dataset:session-test",
        gripper_effort_raw=120,
        limits=limits,
        site_gates=site_gates,
        input_fn=lambda prompt: phrase,
        output_fn=shown.append,
    )
    assert "at most three" in " ".join(shown)
    assert "new healthy feedback snapshot" in " ".join(shown)
    assert SHORT_LOOP_CONFIRMATION_PHRASE in " ".join(shown)
    return grant


def test_short_loop_grant_requires_exact_three_step_consent_and_transfers_once():
    limits, site_gates = _limits_and_gates()
    with pytest.raises(PermissionError, match="did not confirm"):
        _request_short_loop_grant(limits, site_gates, phrase=ONE_STEP_CONFIRMATION_PHRASE)

    grant = _request_short_loop_grant(limits, site_gates)
    assert grant.steps_remaining == 3
    ticket = pickle.loads(pickle.dumps(grant.to_worker_ticket()))
    worker_grant = ticket.materialize()
    assert worker_grant.steps_remaining == 3
    with pytest.raises(PermissionError, match="already been transferred"):
        grant.to_worker_ticket()


def test_short_loop_requires_fresh_post_send_feedback_before_next_authorization():
    limits, site_gates = _limits_and_gates()
    grant = _request_short_loop_grant(limits, site_gates)
    inference_started_ns, prediction, snapshot = _fixture(
        limits, site_gates, grant.issued_at_monotonic_ns,
    )
    prediction = replace(
        prediction,
        actions=tuple((0.08, 0.01, -0.02, -0.02, 0.03, -0.03, 0.05) for _ in range(16)),
    )
    prepared = prepare_supervised_policy_step(
        prediction,
        snapshot=snapshot,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    auth = grant.authorize(
        prediction,
        prepared,
        active_task_description=TASK,
        inference_started_monotonic_ns=inference_started_ns,
        limits=limits,
    )
    assert grant.steps_remaining == 2

    with pytest.raises(PermissionError, match="fresh post-send feedback"):
        grant.authorize(
            prediction,
            prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=inference_started_ns,
            limits=limits,
        )

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
        authorization=auth,
    )
    bridge.close()

    last_write_ns = receipt.writes[-1].adapter_accepted_monotonic_ns
    stale_feedback = RobotStateSnapshot(
        sequence=snapshot.sequence + 1,
        captured_monotonic_ns=last_write_ns - 200,
        received_monotonic_ns=last_write_ns - 100,
        joint_position_rad=snapshot.joint_position_rad,
        gripper_position_m=snapshot.gripper_position_m,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    with pytest.raises(PermissionError, match="does not follow the final adapter write"):
        grant.record_post_send_feedback(prepared, receipt, stale_feedback)

    now_ns = time.perf_counter_ns()
    fresh_feedback = RobotStateSnapshot(
        sequence=snapshot.sequence + 1,
        captured_monotonic_ns=last_write_ns + 1,
        received_monotonic_ns=now_ns,
        joint_position_rad=(0.08, 0.01, -0.02, -0.02, 0.03, -0.03),
        gripper_position_m=snapshot.gripper_position_m,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    grant.record_post_send_feedback(prepared, receipt, fresh_feedback, now_monotonic_ns=now_ns)
    assert grant.steps_remaining == 2

    time.sleep(0.002)
    drifted_ns = time.perf_counter_ns()
    drifted_snapshot = replace(
        fresh_feedback,
        captured_monotonic_ns=drifted_ns - 50_000,
        received_monotonic_ns=drifted_ns,
        joint_position_rad=(0.11, 0.01, -0.02, -0.02, 0.03, -0.03),
    )
    next_inference_started_ns = time.perf_counter_ns()
    next_response_ns = next_inference_started_ns + 100_000
    time.sleep(0.0002)
    drifted_prediction = replace(
        prediction,
        request_id="session-request-5-drifted",
        request_sequence=drifted_snapshot.sequence,
        observation_monotonic_ns=drifted_snapshot.captured_monotonic_ns + 1,
        received_monotonic_ns=next_response_ns,
        actions=tuple((0.10, 0.01, -0.02, -0.02, 0.03, -0.03, 0.05) for _ in range(16)),
    )
    drifted_prepared = prepare_supervised_policy_step(
        drifted_prediction,
        snapshot=drifted_snapshot,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    with pytest.raises(PermissionError, match="state exceeds the total displacement budget"):
        grant.authorize(
            drifted_prediction,
            drifted_prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=next_inference_started_ns,
            limits=limits,
        )

    next_prediction = replace(
        prediction,
        request_id="session-request-5",
        request_sequence=fresh_feedback.sequence,
        observation_monotonic_ns=drifted_snapshot.captured_monotonic_ns + 1,
        received_monotonic_ns=next_response_ns,
        actions=tuple((0.16, 0.01, -0.02, -0.02, 0.03, -0.03, 0.05) for _ in range(16)),
    )
    next_prepared = prepare_supervised_policy_step(
        next_prediction,
        snapshot=fresh_feedback,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    with pytest.raises(PermissionError, match="total displacement budget"):
        grant.authorize(
            next_prediction,
            next_prepared,
            active_task_description=TASK,
            inference_started_monotonic_ns=next_inference_started_ns,
            limits=limits,
        )

    bounded_prediction = replace(
        next_prediction,
        request_id="session-request-5-bounded",
        actions=tuple((0.10, 0.01, -0.02, -0.02, 0.03, -0.03, 0.05) for _ in range(16)),
    )
    bounded_prepared = prepare_supervised_policy_step(
        bounded_prediction,
        snapshot=fresh_feedback,
        evidence=site_gates,
        limits=limits,
        gripper_effort_raw=120,
    )
    next_auth = grant.authorize(
        bounded_prediction,
        bounded_prepared,
        active_task_description=TASK,
        inference_started_monotonic_ns=next_inference_started_ns,
        limits=limits,
    )
    assert next_auth.source_state_sequence == fresh_feedback.sequence
    assert grant.steps_remaining == 1
