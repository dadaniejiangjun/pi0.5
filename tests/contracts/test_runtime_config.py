import json
from pathlib import Path

import pytest

from laptop.runtime.config import RuntimeConfig
from laptop.runtime.modes import RuntimeMode


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "configs" / "runtime" / "shadow.json"


def test_runtime_defaults_to_non_transmitting_shadow():
    config = RuntimeConfig.load(CONFIG_PATH)
    assert config.mode is RuntimeMode.SHADOW
    assert config.can_tx_enabled is False
    assert config.single_can_owner == "safety-bridge"


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("can_tx_enabled", True, PermissionError),
        ("mode", "AUTONOMOUS", PermissionError),
        ("supervised_position_tx_enabled", True, PermissionError),
        ("single_can_owner", "policy-client", ValueError),
        ("max_state_age_ms", 0, ValueError),
    ],
)
def test_runtime_config_fails_closed(tmp_path, field, value, error):
    data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    data[field] = value
    path = tmp_path / "runtime.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(error):
        RuntimeConfig.load(path)


def test_supervised_mode_requires_only_its_dedicated_position_gate(tmp_path):
    data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    data["mode"] = RuntimeMode.SUPERVISED_POLICY_STEP.value
    data["supervised_position_tx_enabled"] = True
    path = tmp_path / "supervised-position.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    config = RuntimeConfig.load(path)
    assert config.mode is RuntimeMode.SUPERVISED_POLICY_STEP
    assert config.supervised_position_tx_enabled is True
    assert config.can_tx_enabled is False
    assert config.command_tx_enabled is False

    data["command_tx_enabled"] = True
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(PermissionError, match="dedicated position-TX gate"):
        RuntimeConfig.load(path)


def test_j1_commissioning_mode_requires_its_own_gate_and_rejects_other_tx_gates(tmp_path):
    data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    data["mode"] = RuntimeMode.SUPERVISED_J1_COMMISSIONING.value
    data["supervised_j1_commissioning_tx_enabled"] = True
    path = tmp_path / "j1-commissioning.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    config = RuntimeConfig.load(path)
    assert config.mode is RuntimeMode.SUPERVISED_J1_COMMISSIONING
    assert config.supervised_j1_commissioning_tx_enabled is True
    assert config.can_tx_enabled is False
    assert config.command_tx_enabled is False
    assert config.supervised_position_tx_enabled is False

    data["supervised_position_tx_enabled"] = True
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(PermissionError, match="only its dedicated one-shot TX gate"):
        RuntimeConfig.load(path)
