import json
from pathlib import Path

import pytest

from server.policy_server.proxy_config import ProxyPolicyConfig


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "configs" / "policy" / "pi05_proxy_shadow.json"
DERIVED_STATE_CONFIG_PATH = ROOT / "configs" / "policy" / "pi05_derived_state_10fps_shadow.json"


def test_default_proxy_config_is_two_step_prediction_only():
    config = ProxyPolicyConfig.load(CONFIG_PATH)
    assert config.experiment_name == "cef166e2_two_step"
    assert config.checkpoint_step == 1
    assert config.action_semantics == "next_observed_state_proxy_nominal_units"
    assert config.prediction_only is True
    assert config.hardware_control_allowed is False
    assert config.server_host == "127.0.0.1"
    assert config.dataset_variant == "2fps"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("hardware_control_allowed", True),
        ("prediction_only", False),
        ("action_semantics", "absolute_joint_position_rad_gripper_meter"),
        ("server_host", "0.0.0.0"),
        ("action_dimension", 6),
        ("first_target_dt_s", 0.04),
    ],
)
def test_proxy_config_fails_closed(tmp_path, field, value):
    data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    data[field] = value
    path = tmp_path / "config.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError):
        ProxyPolicyConfig.load(path)


def test_derived_state_shadow_is_separate_hash_pinned_and_prediction_only():
    config = ProxyPolicyConfig.load(DERIVED_STATE_CONFIG_PATH)
    assert config.schema == "pi05_derived_state_10fps_shadow_v1"
    assert config.dataset_identity == "local/cef166e2_proxy10fps_ds36"
    assert config.action_semantics == "derived_future_observed_state_position_target_nominal_units"
    assert config.source_fps == 10.0
    assert config.first_target_dt_s == 0.1
    assert config.checkpoint_step == 9
    assert config.expected_dataset_tree_sha256 == "9e0912029ec9d2f4c3e256ad1911416a75bc0bb461e0edb5f992e10585e7b8df"
    assert config.expected_readback_audit_sha256 == "7e5f7cf73feb5be7d7e6966598e0b59b69ef68d416e9c5bf62d07150cbcc6b0d"
    assert config.expected_checkpoint_sha256 == "852c3393ca6e8804d32ea79f743394cc0be522c08c5a05c7f82cd232351cf7d6"
    assert config.expected_normalization_sha256 == "d51ad915215b2735fbd12bb41169795bf0970bdab44d54df086eaed26847f35a"
    assert config.expected_openpi_commit == "215abfb217dbac7d5f1273282331b9b1866c0479"
    assert config.prediction_only is True
    assert config.hardware_control_allowed is False
    assert config.server_host == "127.0.0.1"
    assert config.dataset_variant == "10fps"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("action_semantics", "absolute_joint_position_rad_gripper_meter"),
        ("hardware_control_allowed", True),
        ("prediction_only", False),
        ("source_fps", 2.0),
        ("first_target_dt_s", 0.5),
        ("dataset_identity", "local/piper_v1"),
        ("expected_checkpoint_sha256", "unknown"),
        ("expected_openpi_commit", "not-a-commit"),
    ],
)
def test_derived_state_shadow_rejects_semantic_or_identity_drift(tmp_path, field, value):
    data = json.loads(DERIVED_STATE_CONFIG_PATH.read_text(encoding="utf-8"))
    data[field] = value
    path = tmp_path / "derived-state.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError):
        ProxyPolicyConfig.load(path)


def test_derived_state_shadow_requires_all_artifact_hash_pins(tmp_path):
    data = json.loads(DERIVED_STATE_CONFIG_PATH.read_text(encoding="utf-8"))
    del data["expected_checkpoint_sha256"]
    path = tmp_path / "derived-state.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="config keys mismatch"):
        ProxyPolicyConfig.load(path)
