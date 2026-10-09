from server.training.action_target_contract import (
    has_raw_command_action_semantics,
    is_verified_raw_command_episode,
)


def _verified_episode(**overrides):
    row = {
        "source": "manual_teach",
        "success": True,
        "action_target_source": "observed_piper_can_command_frames",
        "action_target_verified": True,
        "command_target_provenance_verified": True,
        "camera_sync_verified": True,
        "gripper_verified": True,
        "units_verified": True,
    }
    row.update(overrides)
    return row


def test_derived_future_observed_state_is_not_raw_command_semantics():
    acceptance = {
        "action_semantics": "derived_future_observed_state_position_target_nominal_units",
    }
    episode = _verified_episode()
    assert not has_raw_command_action_semantics(acceptance)
    assert is_verified_raw_command_episode(episode)


def test_missing_raw_command_provenance_rejects_real_command_supervision():
    episode = _verified_episode(command_target_provenance_verified=False)
    assert not is_verified_raw_command_episode(episode)


def test_raw_command_acceptance_requires_the_exact_position_semantics():
    assert has_raw_command_action_semantics({
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
    })
    assert not has_raw_command_action_semantics({
        "action_semantics": "next_observed_state_proxy_nominal_units",
    })
