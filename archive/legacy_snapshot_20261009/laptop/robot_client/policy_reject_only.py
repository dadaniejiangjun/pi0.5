"""Replay a saved shadow report through SafetyBridge with CAN TX fixed at zero."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

from laptop.control.contracts import PolicyPrediction, RobotStateSnapshot
from laptop.runtime.config import RuntimeConfig
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import FakeCanTransport


def _decision_dict(decision: Any) -> dict:
    result = asdict(decision)
    result["mode"] = decision.mode.value
    result["bridge_state"] = decision.bridge_state.value
    result["reasons"] = list(decision.reasons)
    result["audit_event"] = decision.audit_event()
    return result


def _canonical_audit_value(value: Any) -> Any:
    """Normalize malformed JSON-like values so every rejection can be hashed."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return {"nonfinite_float": repr(value)}
    if isinstance(value, dict):
        return {
            str(key): _canonical_audit_value(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (list, tuple)):
        return [_canonical_audit_value(item) for item in value]
    return {"unsupported_type": type(value).__qualname__, "repr": repr(value)}


def _malformed_decision(item: Any, index: int | str, reason: str) -> dict:
    source_request_id = item.get("request_id") if isinstance(item, dict) else None
    if not isinstance(source_request_id, str) or not source_request_id.strip():
        source_request_id = f"index-{index}"
    encoded = json.dumps(
        {
            "schema": "piper_malformed_shadow_input_v1",
            "source_item": _canonical_audit_value(item),
        },
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    event_time_ns = time.perf_counter_ns()
    input_sha256 = hashlib.sha256(encoded).hexdigest()
    reasons = [reason]
    event = {
        "schema": "piper_safety_bridge_audit_v1",
        "event_type": "malformed_policy_prediction_rejected",
        "decision_monotonic_ns": event_time_ns,
        "input_sha256": input_sha256,
        "bridge_state": "FAULT_LATCHED",
        "mode": RuntimeMode.POLICY_REJECT_ONLY.value,
        "proposal_id": None,
        "source_request_id": source_request_id,
        "accepted_for_offline_analysis": False,
        "hardware_control_allowed": False,
        "transport_send_attempted": False,
        "reasons": reasons,
    }
    return {
        "mode": RuntimeMode.POLICY_REJECT_ONLY.value,
        "bridge_state": "FAULT_LATCHED",
        "proposal_id": None,
        "source_request_id": source_request_id,
        "accepted_for_offline_analysis": False,
        "hardware_control_allowed": False,
        "transport_send_attempted": False,
        "reasons": reasons,
        "event_type": event["event_type"],
        "decision_monotonic_ns": event_time_ns,
        "input_sha256": input_sha256,
        "audit_event": event,
    }


def audit_report(report: Any, runtime: RuntimeConfig) -> dict:
    if runtime.mode is not RuntimeMode.POLICY_REJECT_ONLY:
        raise PermissionError("this entrypoint requires POLICY_REJECT_ONLY mode")
    if not isinstance(report, dict):
        decisions = [
            _malformed_decision(report, "report", "malformed_report_root:TypeError")
        ]
        return {
            "schema": "piper_policy_reject_only_audit_v1",
            "mode": runtime.mode.value,
            "input_prediction_count": 0,
            "decision_count": 1,
            "decisions": decisions,
            "transport_send_attempts": 0,
            "can_data_tx_calls": 0,
            "hardware_control_allowed": False,
        }
    predictions = report.get("predictions", [])
    if not isinstance(predictions, list):
        decisions = [
            _malformed_decision(
                predictions, "predictions", "malformed_prediction_list:TypeError"
            )
        ]
        return {
            "schema": "piper_policy_reject_only_audit_v1",
            "mode": runtime.mode.value,
            "input_prediction_count": 0,
            "decision_count": 1,
            "decisions": decisions,
            "transport_send_attempts": 0,
            "can_data_tx_calls": 0,
            "hardware_control_allowed": False,
        }
    transport = FakeCanTransport()
    bridge = SafetyBridge(mode=runtime.mode, transport=transport,
                          limits=UNVERIFIED_ANALYSIS_LIMITS)
    decisions = []
    try:
        for index, item in enumerate(predictions):
            if not isinstance(item, dict):
                decisions.append(
                    _malformed_decision(
                        item, index, "malformed_prediction_contract:TypeError"
                    )
                )
                continue
            try:
                sequence = int(item["request_sequence"])
                observation_ns = int(item["observation_built_ns"])
                response_ns = int(item["response_ns"])
                state = tuple(float(value) for value in item["state"])
                if len(state) != 7:
                    raise ValueError("state must contain 7 values")
                status = item.get("robot_status", {})
                state_snapshot = RobotStateSnapshot(
                    sequence=sequence,
                    captured_monotonic_ns=observation_ns,
                    received_monotonic_ns=observation_ns,
                    joint_position_rad=state[:6],
                    gripper_position_m=state[6],
                    status_ok=status.get("err_code") in (0, "0"),
                    connected=bool(item.get("can_timestamps_ns")),
                    units_verified=report.get("units_hardware_verified") is True,
                )
                first_dt = float(item["target_dt_s"][0])
                policy_prediction = PolicyPrediction(
                    request_id=str(item["request_id"]),
                    request_sequence=sequence,
                    observation_monotonic_ns=observation_ns,
                    received_monotonic_ns=response_ns,
                    actions=(tuple(float(value) for value
                                   in item["first_predicted_proxy"]),),
                    target_dt_s=(first_dt,),
                    checkpoint_identity=str(item["checkpoint_identity"]),
                    config_identity=str(item["config_identity"]),
                    normalization_identity=str(item["normalization_identity"]),
                    dataset_identity=str(item["dataset_identity"]),
                )
                bridge.record_heartbeat(max(1, response_ns - 1))
                decision = bridge.inspect_prediction(
                    snapshot=state_snapshot,
                    prediction=policy_prediction,
                    now_monotonic_ns=response_ns,
                )
                decisions.append(_decision_dict(decision))
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                decisions.append(
                    _malformed_decision(
                        item,
                        index,
                        f"malformed_prediction_contract:{type(exc).__name__}",
                    )
                )
    finally:
        bridge.close()
    return {
        "schema": "piper_policy_reject_only_audit_v1",
        "mode": runtime.mode.value,
        "input_prediction_count": len(predictions),
        "decision_count": len(decisions),
        "decisions": decisions,
        "transport_send_attempts": transport.send_attempts,
        "can_data_tx_calls": 0,
        "hardware_control_allowed": False,
    }


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument(
        "--runtime-config",
        type=Path,
        default=root / "configs/runtime/policy_reject_only.json",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    runtime = RuntimeConfig.load(args.runtime_config)
    report = json.loads(args.report.read_text(encoding="utf-8"))
    result = audit_report(report, runtime)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
            stream.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
