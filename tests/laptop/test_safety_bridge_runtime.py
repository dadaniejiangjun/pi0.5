import pytest
import math

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
)
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, SafetyLimits
from laptop.transport.can_transport import (
    CanFrame,
    FakeCanTransport,
    PiperCanTransport,
    TransportOwnershipError,
)


NOW = 2_000_000_000


def limits():
    return SafetyLimits(
        joint_min_rad=(-3.0,) * 6,
        joint_max_rad=(3.0,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.10,
        max_gripper_delta_m=0.01,
    )


def snapshot(**overrides):
    values = {
        "sequence": 7,
        "captured_monotonic_ns": NOW - 20_000_000,
        "received_monotonic_ns": NOW - 10_000_000,
        "joint_position_rad": (0.0,) * 6,
        "gripper_position_m": 0.04,
        "status_ok": True,
        "connected": True,
        "units_verified": True,
    }
    values.update(overrides)
    return RobotStateSnapshot(**values)


def prediction(state=None, **overrides):
    state = state or snapshot()
    values = {
        "request_id": "request-7",
        "request_sequence": state.sequence,
        "observation_monotonic_ns": state.captured_monotonic_ns,
        "received_monotonic_ns": NOW - 5_000_000,
        "actions": ((0.0,) * 6 + (0.04,),),
        "target_dt_s": (0.5,),
        "checkpoint_identity": "cef166e2_two_step/1",
        "config_identity": "pi05_proxy_shadow_v1",
        "normalization_identity": "proxy_dataset_train_split_stats",
        "dataset_identity": "local/pi05_proxy_cef166e2_2fps",
    }
    values.update(overrides)
    return PolicyPrediction(**values)


def proposal(state=None, **overrides):
    state = state or snapshot()
    values = {
        "proposal_id": "proposal-7",
        "source_request_id": "request-7",
        "source_state_sequence": state.sequence,
        "created_monotonic_ns": NOW - 5_000_000,
        "valid_until_monotonic_ns": NOW + 100_000_000,
        "target": (0.01,) * 6 + (0.045,),
        "action_semantics": ABSOLUTE_JOINT_POSITION_COMMAND,
        "units": JOINT_GRIPPER_UNITS,
        "hardware_units_verified": True,
        "mapping_verified": True,
    }
    values.update(overrides)
    return CommandProposal(**values)


def bridge_for(mode=RuntimeMode.POLICY_REJECT_ONLY, safety_limits=None):
    transport = FakeCanTransport()
    bridge = SafetyBridge(
        mode=mode,
        transport=transport,
        limits=safety_limits or limits(),
    )
    bridge.record_heartbeat(NOW - 1_000_000)
    return bridge, transport


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("max_gripper_delta_m", math.nan),
        ("max_gripper_delta_m", math.inf),
        ("max_state_age_ns", math.nan),
        ("max_proposal_age_ns", math.inf),
        ("max_heartbeat_age_ns", math.nan),
        ("max_state_age_ns", True),
        ("max_heartbeat_age_ns", True),
        ("gripper_min_m", math.nan),
        ("gripper_max_m", math.inf),
        ("joint_min_rad", (False,) * 6),
        ("joint_max_rad", ("3.0",) * 6),
        ("max_joint_delta_rad", (True,) * 6),
    ],
)
def test_safety_limits_reject_nonfinite_and_noninteger_values(field_name, value):
    values = limits().__dict__.copy()
    values[field_name] = value
    with pytest.raises(ValueError):
        SafetyLimits(**values)


@pytest.mark.parametrize("mode", [RuntimeMode.SHADOW, RuntimeMode.POLICY_REJECT_ONLY])
def test_proxy_prediction_is_audited_but_never_sent(mode):
    bridge, transport = bridge_for(mode)
    state = snapshot()
    decision = bridge.inspect_prediction(
        snapshot=state, prediction=prediction(state), now_monotonic_ns=NOW
    )
    assert decision.accepted_for_offline_analysis is True
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert decision.bridge_state is BridgeState.READY
    assert "next_observed_state_proxy_is_not_command" in decision.reasons
    event = decision.audit_event()
    assert event["schema"] == "piper_safety_bridge_audit_v1"
    assert event["event_type"] == "policy_prediction_decision"
    assert event["decision_monotonic_ns"] == NOW
    assert len(event["input_sha256"]) == 64
    assert event["timing_contract"]["schema"] == "piper_timing_contract_v1"
    assert event["timing_contract"]["clock_domain"] == "laptop_perf_counter_ns"
    int(event["input_sha256"], 16)
    assert event["hardware_control_allowed"] is False
    assert event["transport_send_attempted"] is False
    assert transport.send_attempts == 0
    bridge.close()


def test_prediction_response_latency_over_contract_limit_fails_closed():
    values = limits().__dict__.copy()
    values["max_prediction_response_latency_ns"] = 10_000_000
    bridge, transport = bridge_for(safety_limits=SafetyLimits(**values))
    state = snapshot()

    decision = bridge.inspect_prediction(
        snapshot=state,
        prediction=prediction(state),
        now_monotonic_ns=NOW,
    )

    assert decision.accepted_for_offline_analysis is False
    assert "prediction_response_latency_exceeded" in decision.reasons
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert transport.send_attempts == 0


def test_prediction_with_passed_first_target_deadline_fails_closed():
    values = limits().__dict__.copy()
    values["max_state_age_ns"] = 1_000_000_000
    values["max_prediction_response_latency_ns"] = 1_000_000_000
    values["max_proposal_age_ns"] = 1_000_000_000
    bridge, transport = bridge_for(safety_limits=SafetyLimits(**values))
    state = snapshot(
        captured_monotonic_ns=NOW - 600_000_000,
        received_monotonic_ns=NOW - 590_000_000,
    )
    policy_result = prediction(
        state,
        received_monotonic_ns=NOW - 5_000_000,
        target_dt_s=(0.5,),
    )

    decision = bridge.inspect_prediction(
        snapshot=state,
        prediction=policy_result,
        now_monotonic_ns=NOW,
    )

    assert policy_result.target_timestamps_monotonic_ns[0] < NOW
    assert decision.accepted_for_offline_analysis is False
    assert "prediction_target_deadline_passed" in decision.reasons
    assert decision.hardware_control_allowed is False
    assert transport.send_attempts == 0


def test_unverified_prediction_units_reject_without_stopping_passive_receive():
    bridge, transport = bridge_for()
    bridge.start_read_only()
    state = snapshot(units_verified=False)

    decision = bridge.inspect_prediction(
        snapshot=state, prediction=prediction(state), now_monotonic_ns=NOW
    )

    assert decision.accepted_for_offline_analysis is False
    assert "state_units_unverified" in decision.reasons
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert decision.bridge_state is BridgeState.READY
    assert bridge.receive_frame() is None
    assert transport.send_attempts == 0
    bridge.close()


def test_prediction_received_timestamp_in_future_fails_closed():
    bridge, transport = bridge_for()
    state = snapshot()
    future_prediction = prediction(state, received_monotonic_ns=NOW + 1)
    decision = bridge.inspect_prediction(
        snapshot=state, prediction=future_prediction, now_monotonic_ns=NOW
    )
    assert decision.accepted_for_offline_analysis is False
    assert "prediction_received_from_future" in decision.reasons
    assert decision.hardware_control_allowed is False
    assert transport.send_attempts == 0


@pytest.mark.parametrize("bad_now", [True, 0, -1, math.nan, math.inf, "bad-clock"])
def test_prediction_with_invalid_current_clock_fails_closed_without_send(bad_now):
    bridge, transport = bridge_for()
    state = snapshot()

    decision = bridge.inspect_prediction(
        snapshot=state,
        prediction=prediction(state),
        now_monotonic_ns=bad_now,
    )

    assert decision.accepted_for_offline_analysis is False
    assert decision.decision_monotonic_ns == 0
    assert "invalid_current_time" in decision.reasons
    assert decision.bridge_state is BridgeState.FAULT_LATCHED
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert transport.send_attempts == 0


@pytest.mark.parametrize("bad_now", [True, 0, -1, math.nan, math.inf, "bad-clock"])
def test_command_proposal_with_invalid_current_clock_fails_closed_without_send(bad_now):
    bridge, transport = bridge_for()
    state = snapshot()

    decision = bridge.inspect_command_proposal(
        snapshot=state,
        proposal=proposal(state),
        now_monotonic_ns=bad_now,
    )

    assert decision.accepted_for_offline_analysis is False
    assert decision.decision_monotonic_ns == 0
    assert "invalid_current_time" in decision.reasons
    assert decision.bridge_state is BridgeState.FAULT_LATCHED
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert transport.send_attempts == 0


def test_audit_hash_binds_the_prediction_payload():
    state = snapshot()
    first_bridge, _ = bridge_for()
    second_bridge, _ = bridge_for()
    first = first_bridge.inspect_prediction(
        snapshot=state, prediction=prediction(state), now_monotonic_ns=NOW
    )
    changed = prediction(state, actions=((0.01,) * 7,))
    second = second_bridge.inspect_prediction(
        snapshot=state, prediction=changed, now_monotonic_ns=NOW
    )
    assert first.audit_event()["input_sha256"] != second.audit_event()["input_sha256"]


def test_even_valid_command_is_rejected_by_offline_runtime_without_send():
    bridge, transport = bridge_for()
    state = snapshot()
    decision = bridge.inspect_command_proposal(
        snapshot=state, proposal=proposal(state), now_monotonic_ns=NOW
    )
    assert decision.accepted_for_offline_analysis is True
    assert decision.hardware_control_allowed is False
    assert decision.reasons == ("runtime_mode_forbids_can_tx",)
    assert decision.audit_event()["event_type"] == "command_proposal_decision"
    assert decision.audit_event()["bridge_state"] == BridgeState.READY.value
    assert len(decision.audit_event()["input_sha256"]) == 64
    assert decision.audit_event()["timing_contract"]["max_proposal_validity_ns"] == 250_000_000
    assert transport.send_attempts == 0


@pytest.mark.parametrize(
    ("state_overrides", "proposal_overrides", "expected_reason"),
    [
        ({"connected": False}, {}, "robot_disconnected"),
        ({"status_ok": False}, {}, "robot_status_not_ok"),
        ({"units_verified": False}, {}, "state_units_unverified"),
        (
            {"received_monotonic_ns": NOW - 300_000_000, "captured_monotonic_ns": NOW - 310_000_000},
            {},
            "state_stale",
        ),
        ({}, {"source_state_sequence": 6}, "proposal_state_sequence_mismatch"),
        (
            {},
            {"created_monotonic_ns": NOW - 20_000_000},
            "proposal_created_before_source_state_received",
        ),
        ({}, {"hardware_units_verified": False}, "hardware_units_unverified"),
        ({}, {"mapping_verified": False}, "command_mapping_unverified"),
        ({}, {"target": (0.2,) + (0.0,) * 5 + (0.04,)}, "joint_1_delta_limit"),
        ({}, {"target": (0.0,) * 6 + (0.2,)}, "gripper_position_limit"),
    ],
)
def test_fault_injection_rejects_and_never_sends(
    state_overrides, proposal_overrides, expected_reason
):
    bridge, transport = bridge_for()
    state = snapshot(**state_overrides)
    decision = bridge.inspect_command_proposal(
        snapshot=state,
        proposal=proposal(state, **proposal_overrides),
        now_monotonic_ns=NOW,
    )
    assert decision.accepted_for_offline_analysis is False
    assert decision.hardware_control_allowed is False
    assert expected_reason in decision.reasons
    assert decision.bridge_state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0


def test_proposal_validity_lease_over_contract_limit_fails_closed():
    bridge, transport = bridge_for()
    state = snapshot()
    long_lease = proposal(
        state,
        valid_until_monotonic_ns=NOW + 1_000_000_000,
    )

    decision = bridge.inspect_command_proposal(
        snapshot=state,
        proposal=long_lease,
        now_monotonic_ns=NOW,
    )

    assert decision.accepted_for_offline_analysis is False
    assert "proposal_validity_window_exceeded" in decision.reasons
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert transport.send_attempts == 0


def test_bridge_fault_latches_until_instance_is_recreated():
    bridge, transport = bridge_for()
    state = snapshot()
    unsafe = proposal(state, target=(0.2,) + (0.0,) * 5 + (0.04,))
    first = bridge.inspect_command_proposal(
        snapshot=state, proposal=unsafe, now_monotonic_ns=NOW
    )
    assert first.bridge_state is BridgeState.FAULT_LATCHED
    assert "joint_1_delta_limit" in first.reasons

    next_now = NOW + 2_000_000
    bridge.record_heartbeat(next_now - 100_000)
    fresh_state = snapshot(
        sequence=8,
        captured_monotonic_ns=next_now - 500_000,
        received_monotonic_ns=next_now - 250_000,
    )
    fresh_proposal = proposal(
        fresh_state,
        created_monotonic_ns=next_now - 100_000,
        valid_until_monotonic_ns=next_now + 100_000_000,
    )
    second = bridge.inspect_command_proposal(
        snapshot=fresh_state,
        proposal=fresh_proposal,
        now_monotonic_ns=next_now,
    )
    assert second.accepted_for_offline_analysis is False
    assert "bridge_fault_latched" in second.reasons
    assert second.bridge_state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0

    bridge.close()
    assert bridge.state is BridgeState.CLOSED
    with pytest.raises(PermissionError, match="closed safety bridge"):
        bridge.record_heartbeat(next_now + 1)


def test_closed_bridge_remains_fail_closed_and_cannot_be_reopened():
    bridge, transport = bridge_for()
    bridge.close()
    state = snapshot()
    decision = bridge.inspect_command_proposal(
        snapshot=state, proposal=proposal(state), now_monotonic_ns=NOW
    )
    assert decision.accepted_for_offline_analysis is False
    assert "bridge_closed" in decision.reasons
    assert decision.bridge_state is BridgeState.CLOSED
    assert transport.send_attempts == 0


def test_replayed_state_is_rejected():
    bridge, transport = bridge_for()
    state = snapshot()
    first = bridge.inspect_command_proposal(
        snapshot=state, proposal=proposal(state), now_monotonic_ns=NOW
    )
    second = bridge.inspect_command_proposal(
        snapshot=state, proposal=proposal(state), now_monotonic_ns=NOW
    )
    assert first.accepted_for_offline_analysis is True
    assert second.accepted_for_offline_analysis is False
    assert "state_out_of_order_or_replayed" in second.reasons
    assert transport.send_attempts == 0


def test_hardware_modes_are_not_implemented_or_authorized():
    for mode in (
        RuntimeMode.HIL_SINGLE_STEP,
        RuntimeMode.SUPERVISED_POLICY_STEP,
        RuntimeMode.AUTONOMOUS,
    ):
        with pytest.raises(PermissionError):
            SafetyBridge(mode=mode, transport=FakeCanTransport(), limits=limits())


def test_missing_or_stale_heartbeat_fails_closed_without_send():
    state = snapshot()
    missing_transport = FakeCanTransport()
    missing = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=missing_transport,
        limits=limits(),
    )
    missing_decision = missing.inspect_command_proposal(
        snapshot=state, proposal=proposal(state), now_monotonic_ns=NOW
    )
    assert "heartbeat_missing" in missing_decision.reasons
    assert missing_transport.send_attempts == 0

    stale_transport = FakeCanTransport()
    stale_bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=stale_transport,
        limits=limits(),
    )
    stale_bridge.record_heartbeat(NOW - 300_000_000)
    stale_decision = stale_bridge.inspect_command_proposal(
        snapshot=state, proposal=proposal(state), now_monotonic_ns=NOW
    )
    assert "heartbeat_stale" in stale_decision.reasons
    assert stale_transport.send_attempts == 0


def test_watchdog_tick_audits_fresh_heartbeat_without_sending():
    bridge, transport = bridge_for()

    decision = bridge.watchdog_tick(NOW)

    assert decision.event_type == "watchdog_decision"
    assert decision.accepted_for_offline_analysis is True
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert decision.reasons == ()
    assert decision.bridge_state is BridgeState.READY
    assert len(decision.input_sha256) == 64
    audit = decision.audit_event()
    assert audit["schema"] == "piper_safety_bridge_audit_v1"
    assert audit["event_type"] == "watchdog_decision"
    assert audit["input_sha256"] == decision.input_sha256
    assert transport.send_attempts == 0


def test_watchdog_tick_latches_stale_heartbeat_and_recovery_does_not_clear_it():
    transport = FakeCanTransport()
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=transport,
        limits=limits(),
    )
    bridge.record_heartbeat(NOW - limits().max_heartbeat_age_ns - 1)

    stale = bridge.watchdog_tick(NOW)

    assert stale.accepted_for_offline_analysis is False
    assert "heartbeat_stale" in stale.reasons
    assert stale.bridge_state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0

    bridge.record_heartbeat(NOW + 10)
    recovered = bridge.watchdog_tick(NOW + 20)

    assert recovered.accepted_for_offline_analysis is False
    assert "bridge_fault_latched" in recovered.reasons
    assert recovered.bridge_state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0


@pytest.mark.parametrize("bad_now", [True, 0, -1])
def test_watchdog_tick_rejects_invalid_clock_and_latches(bad_now):
    bridge, transport = bridge_for()

    decision = bridge.watchdog_tick(bad_now)

    assert decision.accepted_for_offline_analysis is False
    assert "invalid_current_time" in decision.reasons
    assert decision.bridge_state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0


def test_watchdog_tick_rejects_future_heartbeat_without_send():
    bridge, transport = bridge_for()
    bridge.record_heartbeat(NOW + 10)

    decision = bridge.watchdog_tick(NOW)

    assert decision.accepted_for_offline_analysis is False
    assert "heartbeat_from_future" in decision.reasons
    assert decision.bridge_state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0


def test_record_heartbeat_rejects_boolean_timestamp():
    bridge, transport = bridge_for()

    with pytest.raises(ValueError, match="heartbeat timestamp"):
        bridge.record_heartbeat(True)

    assert transport.send_attempts == 0


def test_heartbeat_age_uses_its_own_limit_not_proposal_age():
    values = limits().__dict__.copy()
    values["max_heartbeat_age_ns"] = 50_000_000
    values["max_proposal_age_ns"] = 500_000_000
    transport = FakeCanTransport()
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=transport,
        limits=SafetyLimits(**values),
    )
    bridge.record_heartbeat(NOW - 60_000_000)

    decision = bridge.inspect_command_proposal(
        snapshot=snapshot(),
        proposal=proposal(),
        now_monotonic_ns=NOW,
    )

    assert "heartbeat_stale" in decision.reasons
    assert "proposal_stale" not in decision.reasons
    assert transport.send_attempts == 0


def test_closed_bridge_watchdog_tick_is_rejected_without_send():
    bridge, transport = bridge_for()
    bridge.close()

    decision = bridge.watchdog_tick(NOW)

    assert decision.accepted_for_offline_analysis is False
    assert "bridge_closed" in decision.reasons
    assert decision.bridge_state is BridgeState.CLOSED
    assert transport.send_attempts == 0


@pytest.mark.parametrize(
    ("fault", "expected_reason"),
    [
        ("usb_disconnect", "usb_disconnected"),
        ("transport_process_exit", "transport_process_exited"),
    ],
)
def test_transport_disconnect_faults_fail_closed_without_send(fault, expected_reason):
    bridge, transport = bridge_for()
    transport.inject_disconnect(fault)
    state = snapshot()
    decision = bridge.inspect_command_proposal(
        snapshot=state,
        proposal=proposal(state),
        now_monotonic_ns=NOW,
    )
    assert decision.accepted_for_offline_analysis is False
    assert decision.hardware_control_allowed is False
    assert decision.transport_send_attempted is False
    assert "can_transport_disconnected" in decision.reasons
    assert expected_reason in decision.reasons
    assert transport.send_attempts == 0


def test_fake_transport_only_accepts_known_disconnect_faults():
    transport = FakeCanTransport()
    with pytest.raises(ValueError, match="unsupported fake transport"):
        transport.inject_disconnect("arbitrary_failure")


def test_fake_transport_enforces_unique_owner():
    transport = FakeCanTransport()
    assert isinstance(transport, PiperCanTransport)
    transport.claim("bridge-a")
    with pytest.raises(TransportOwnershipError):
        transport.claim("bridge-b")


def test_safety_bridge_is_the_owner_for_read_only_can_frames():
    bridge, transport = bridge_for()
    frame = CanFrame(
        arbitration_id=0x2A1,
        data=b"\x00" * 8,
        is_extended_id=False,
        is_error_frame=False,
        received_monotonic_ns=NOW,
    )
    transport.enqueue_frame(frame)

    with pytest.raises(PermissionError, match="receive is not active"):
        bridge.receive_frame()

    bridge.start_read_only()
    assert bridge.receive_frame() == frame
    assert transport.owner == SafetyBridge.OWNER
    assert transport.send_attempts == 0

    bridge.close()
    assert transport.read_only_active is False
    assert transport.owner is None


def test_safety_bridge_latches_read_transport_faults():
    bridge, transport = bridge_for()
    bridge.start_read_only()
    transport.inject_disconnect("usb_disconnect")

    with pytest.raises(ConnectionError):
        bridge.receive_frame()

    assert bridge.state is BridgeState.FAULT_LATCHED
    assert transport.send_attempts == 0
    bridge.close()
