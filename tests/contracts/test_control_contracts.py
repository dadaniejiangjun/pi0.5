import pytest

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    LAPTOP_MONOTONIC_CLOCK,
    NEXT_OBSERVED_STATE_PROXY,
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
    TimingContract,
    command_from_proxy,
)


def make_prediction(**overrides):
    values = {
        "request_id": "request-1",
        "request_sequence": 4,
        "observation_monotonic_ns": 1_000_000_000,
        "received_monotonic_ns": 1_010_000_000,
        "actions": ((0.0,) * 7, (0.1,) * 7),
        "target_dt_s": (0.5, 1.0),
        "checkpoint_identity": "cef166e2_two_step/1",
        "config_identity": "pi05_proxy_shadow_v1",
        "normalization_identity": "proxy_dataset_train_split_stats",
        "dataset_identity": "local/pi05_proxy_cef166e2_2fps",
    }
    values.update(overrides)
    return PolicyPrediction(**values)


def make_snapshot(**overrides):
    values = {
        "sequence": 4,
        "captured_monotonic_ns": 1_000_000_000,
        "received_monotonic_ns": 1_010_000_000,
        "joint_position_rad": (0.0,) * 6,
        "gripper_position_m": 0.04,
        "status_ok": True,
        "connected": True,
        "units_verified": True,
    }
    values.update(overrides)
    return RobotStateSnapshot(**values)


def test_proxy_prediction_has_frozen_semantics_and_time_axis():
    prediction = make_prediction()
    assert prediction.action_semantics == NEXT_OBSERVED_STATE_PROXY
    assert prediction.target_dt_s == (0.5, 1.0)
    assert prediction.target_timestamps_monotonic_ns == (1_500_000_000, 2_000_000_000)
    assert prediction.response_latency_ns == 10_000_000
    assert prediction.prediction_only is True
    assert prediction.hardware_units_verified is False


def test_timing_contract_is_versioned_and_uses_laptop_monotonic_clock():
    contract = TimingContract()

    assert contract.schema == "piper_timing_contract_v1"
    assert contract.clock_domain == LAPTOP_MONOTONIC_CLOCK
    assert contract.first_target_must_be_future_at_evaluation is True
    assert contract.max_prediction_response_latency_ns == 250_000_000
    assert contract.max_proposal_validity_ns == 250_000_000

    with pytest.raises(ValueError, match="laptop monotonic clock"):
        TimingContract(clock_domain="wall_clock")
    with pytest.raises(ValueError, match="positive integer"):
        TimingContract(max_state_age_ns=True)


def test_proxy_target_offsets_must_resolve_to_distinct_monotonic_timestamps():
    with pytest.raises(ValueError, match="strictly increasing nanosecond offsets"):
        make_prediction(target_dt_s=(1.1e-9, 1.2e-9))


@pytest.mark.parametrize(
    "overrides",
    [
        {"actions": ((float("nan"),) + (0.0,) * 6,)},
        {"actions": ((0.0,) * 6,)},
        {"target_dt_s": (0.5, 0.5)},
        {"target_dt_s": (0.5,)},
        {"action_semantics": ABSOLUTE_JOINT_POSITION_COMMAND},
        {"hardware_units_verified": True},
    ],
)
def test_proxy_prediction_rejects_ambiguous_or_unsafe_contracts(overrides):
    with pytest.raises(ValueError):
        make_prediction(**overrides)


@pytest.mark.parametrize(
    "overrides",
    [
        {"sequence": True},
        {"captured_monotonic_ns": 1_000_000_000.0},
        {"received_monotonic_ns": 1_020_000_000.0},
        {"status_ok": 1},
        {"connected": 0},
        {"units_verified": "true"},
        {"gripper_position_m": True},
    ],
)
def test_state_snapshot_rejects_noncanonical_types(overrides):
    with pytest.raises(ValueError):
        make_snapshot(**overrides)


@pytest.mark.parametrize(
    "overrides",
    [
        {"request_sequence": True},
        {"observation_monotonic_ns": 1_000_000_000.0},
        {"received_monotonic_ns": False},
        {"target_dt_s": (True, 2.0)},
        {"prediction_only": 1},
        {"hardware_units_verified": 0},
        {"actions": ((True,) + (0.0,) * 6,)},
    ],
)
def test_policy_prediction_rejects_noncanonical_types(overrides):
    with pytest.raises(ValueError):
        make_prediction(**overrides)


def test_proxy_cannot_be_implicitly_converted_to_command():
    with pytest.raises(PermissionError, match="not a PiPER command"):
        command_from_proxy(make_prediction())


def test_command_proposal_requires_command_semantics_and_verified_fields_are_explicit():
    proposal = CommandProposal(
        proposal_id="proposal-1",
        source_request_id="request-1",
        source_state_sequence=4,
        created_monotonic_ns=1_020_000_000,
        valid_until_monotonic_ns=1_120_000_000,
        target=(0.0,) * 7,
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=False,
        mapping_verified=False,
    )
    assert proposal.hardware_units_verified is False
    assert proposal.mapping_verified is False


@pytest.mark.parametrize(
    "overrides",
    [
        {"source_state_sequence": True},
        {"created_monotonic_ns": 1_020_000_000.0},
        {"valid_until_monotonic_ns": 1_120_000_000.0},
        {"hardware_units_verified": 1},
        {"mapping_verified": 0},
        {"target": (False,) + (0.0,) * 6},
    ],
)
def test_command_proposal_rejects_noncanonical_types(overrides):
    values = {
        "proposal_id": "proposal-1",
        "source_request_id": "request-1",
        "source_state_sequence": 4,
        "created_monotonic_ns": 1_020_000_000,
        "valid_until_monotonic_ns": 1_120_000_000,
        "target": (0.0,) * 7,
        "action_semantics": ABSOLUTE_JOINT_POSITION_COMMAND,
        "units": JOINT_GRIPPER_UNITS,
        "hardware_units_verified": False,
        "mapping_verified": False,
    }
    values.update(overrides)
    with pytest.raises(ValueError):
        CommandProposal(**values)
