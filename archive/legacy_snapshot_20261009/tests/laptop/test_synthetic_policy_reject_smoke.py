from pathlib import Path
import inspect

import numpy as np
import pytest

from laptop.client import synthetic_policy_reject_smoke as smoke
from laptop.client.live_shadow import load_artifact_identity_lock


ROOT = Path(__file__).resolve().parents[2]


class FakeSocket:
    def close(self):
        self.closed = True


class FakePolicyClient:
    def __init__(self, metadata):
        self._ws = FakeSocket()
        self.metadata = metadata

    def get_server_metadata(self):
        return self.metadata


def metadata_from_lock(lock):
    return {
        **lock,
        "action_horizon": 16,
        "first_target_dt_s": 0.5,
        "prediction_only": True,
        "control_enabled": False,
        "robot_control_ready": False,
    }


def test_synthetic_server_prediction_reaches_reject_only_bridge_with_zero_tx(monkeypatch):
    lock_path = ROOT / "configs/runtime/pi05_proxy_shadow_identity_lock.json"
    metadata = metadata_from_lock(load_artifact_identity_lock(lock_path))
    client = FakePolicyClient(metadata)
    monkeypatch.setattr(smoke, "_check_loopback_listener", lambda *_: None)
    monkeypatch.setattr(
        smoke,
        "_bounded_prediction",
        lambda *_: {"actions": np.zeros((16, 7), dtype=np.float32)},
    )

    report = smoke.run(port=9667, client_factory=lambda: client)

    decision = report["safety_decision"]
    assert report["status"] == "PASS_SYNTHETIC_POLICY_REJECTED_NO_TX"
    assert decision["hardware_control_allowed"] is False
    assert decision["transport_send_attempted"] is False
    assert decision["bridge_state"] == "FAULT_LATCHED"
    assert "next_observed_state_proxy_is_not_command" in decision["reasons"]
    assert "robot_disconnected" in decision["reasons"]
    assert report["transport_send_attempts"] == 0
    assert report["can_data_tx_calls"] == 0
    assert client._ws.closed is True


def test_smoke_rejects_non_loopback_host_by_not_exposing_a_host_parameter():
    # The API intentionally accepts only a local forwarded port, never a host.
    assert "host" not in inspect.signature(smoke.run).parameters


def test_smoke_rejects_runtime_config_with_can_tx_enabled(monkeypatch):
    from laptop.runtime.config import RuntimeConfig
    from laptop.runtime.modes import RuntimeMode

    unsafe_runtime = RuntimeConfig(
        schema="piper_runtime_v1",
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        policy_config="configs/policy/pi05_proxy_shadow.json",
        can_tx_enabled=True,
        single_can_owner="safety-bridge",
        max_state_age_ms=250,
        max_prediction_age_ms=250,
        notes="test-only invalid config",
    )
    monkeypatch.setattr(
        smoke.RuntimeConfig, "load", lambda _path: unsafe_runtime
    )
    with pytest.raises(PermissionError, match="CAN and command transmission"):
        smoke.run()
