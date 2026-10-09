import json
from pathlib import Path

from laptop.robot_client.policy_reject_only import audit_report
from laptop.runtime.config import RuntimeConfig


ROOT = Path(__file__).resolve().parents[2]
RUNTIME = RuntimeConfig.load(ROOT / "configs/runtime/policy_reject_only.json")


def complete_prediction():
    observation_ns = 1_000_000_000
    return {
        "request_id": "shadow-0",
        "request_sequence": 0,
        "observation_built_ns": observation_ns,
        "response_ns": observation_ns + 20_000_000,
        "state": [0.0] * 6 + [0.04],
        "robot_status": {"err_code": 0},
        "can_timestamps_ns": {"status": observation_ns - 1},
        "target_dt_s": [0.5],
        "first_predicted_proxy": [0.01] * 6 + [0.045],
        "checkpoint_identity": "cef166e2_two_step/1",
        "config_identity": "pi05_proxy_shadow_v1",
        "normalization_identity": "proxy_dataset_train_split_stats",
        "dataset_identity": "local/pi05_proxy_cef166e2_2fps",
    }


def test_complete_proxy_reaches_bridge_but_is_rejected_without_send():
    result = audit_report(
        {"predictions": [complete_prediction()], "units_hardware_verified": False},
        RUNTIME,
    )
    decision = result["decisions"][0]
    assert decision["hardware_control_allowed"] is False
    assert decision["transport_send_attempted"] is False
    # Expected proxy/unit rejection is not a transport or bridge fault; it
    # must not stop a subsequent passive observation from being audited.
    assert decision["bridge_state"] == "READY"
    assert decision["audit_event"]["schema"] == "piper_safety_bridge_audit_v1"
    assert decision["audit_event"]["input_sha256"] == decision["input_sha256"]
    assert "next_observed_state_proxy_is_not_command" in decision["reasons"]
    assert "state_units_unverified" in decision["reasons"]
    assert result["transport_send_attempts"] == 0
    assert result["can_data_tx_calls"] == 0


def test_legacy_or_incomplete_report_fails_closed():
    result = audit_report({"predictions": [{"first_predicted_proxy": [0] * 7}]}, RUNTIME)
    assert result["decisions"][0]["reasons"] == [
        "malformed_prediction_contract:KeyError"
    ]
    decision = result["decisions"][0]
    assert decision["event_type"] == "malformed_policy_prediction_rejected"
    assert decision["bridge_state"] == "FAULT_LATCHED"
    assert decision["audit_event"]["hardware_control_allowed"] is False
    assert decision["audit_event"]["transport_send_attempted"] is False
    assert len(decision["input_sha256"]) == 64
    assert result["transport_send_attempts"] == 0


def test_malformed_prediction_items_fail_closed_with_stable_input_hashes():
    malformed = {"request_id": "bad-proxy", "state": [float("nan")]}
    first = audit_report({"predictions": [malformed]}, RUNTIME)["decisions"][0]
    second = audit_report({"predictions": [malformed]}, RUNTIME)["decisions"][0]
    changed = audit_report(
        {"predictions": [{"request_id": "bad-proxy", "state": [1.0]}]}, RUNTIME
    )["decisions"][0]
    assert first["input_sha256"] == second["input_sha256"]
    assert first["input_sha256"] != changed["input_sha256"]
    assert first["reasons"] == ["malformed_prediction_contract:KeyError"]
    json.dumps(first, allow_nan=False)


def test_malformed_root_list_and_non_mapping_items_are_audited():
    root = audit_report([complete_prediction()], RUNTIME)
    non_mapping = audit_report({"predictions": ["bad-entry"]}, RUNTIME)
    non_list = audit_report({"predictions": "bad-list"}, RUNTIME)
    for result in (root, non_mapping, non_list):
        decision = result["decisions"][0]
        assert decision["accepted_for_offline_analysis"] is False
        assert decision["hardware_control_allowed"] is False
        assert decision["transport_send_attempted"] is False
        assert decision["audit_event"]["bridge_state"] == "FAULT_LATCHED"
        assert len(decision["audit_event"]["input_sha256"]) == 64
        assert result["can_data_tx_calls"] == 0
        json.dumps(result, allow_nan=False)
