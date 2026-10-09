"""Exercise synthetic observation -> remote proxy -> SafetyBridge reject-only.

This is an off-robot integration smoke. It only connects to a loopback
endpoint (normally a temporary SSH tunnel), uses synthetic inputs, and records
the SafetyBridge decision. It never opens cameras, a CAN adapter, or a PiPER SDK
interface, and has no command-transmit path.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time
from typing import Callable

import numpy as np

from laptop.client.live_shadow import (
    DEFAULT_IDENTITY_LOCK,
    build_policy_prediction,
    load_artifact_identity_lock,
)
from laptop.client.observation_schema import (
    make_piper_proxy_warmup_observation,
    validate_piper_semantic_observation,
)
from laptop.robot_client.policy_reject_only import audit_report
from laptop.runtime.config import RuntimeConfig
from laptop.runtime.modes import RuntimeMode

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNTIME_CONFIG = PROJECT_ROOT / "configs/runtime/policy_reject_only.json"


def _client_factory(port: int) -> Callable[[], object]:
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    return lambda: WebsocketClientPolicy(host="127.0.0.1", port=port)


def _check_loopback_listener(port: int, timeout_s: float) -> None:
    with socket.create_connection(("127.0.0.1", port), timeout=timeout_s):
        pass


def _bounded_prediction(client: object, observation: dict, timeout_s: float) -> dict:
    """Use OpenPI's existing packer with a bounded WebSocket receive."""
    from openpi_client import msgpack_numpy

    connection = client._ws
    connection.send(client._packer.pack(observation))
    response = connection.recv(timeout=timeout_s)
    if isinstance(response, str):
        raise RuntimeError("policy server returned an error frame")
    result = msgpack_numpy.unpackb(response)
    if not isinstance(result, dict):
        raise ValueError("policy response must be an object")
    actions = np.asarray(result.get("actions"))
    if actions.shape != (16, 7) or not np.isfinite(actions).all():
        raise ValueError("policy response must contain finite 16x7 proxy actions")
    return result


def _close_client(client: object | None) -> None:
    if client is None:
        return
    try:
        client._ws.close()
    except Exception:
        pass


def run(
    *,
    port: int = 9667,
    timeout_s: float = 10.0,
    runtime_config_path: Path = DEFAULT_RUNTIME_CONFIG,
    identity_lock_path: Path = DEFAULT_IDENTITY_LOCK,
    client_factory: Callable[[], object] | None = None,
) -> dict:
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port must be in [1, 65535]")
    if not 0.25 <= timeout_s <= 60:
        raise ValueError("timeout must be between 0.25 and 60 seconds")

    runtime = RuntimeConfig.load(runtime_config_path)
    if runtime.mode is not RuntimeMode.POLICY_REJECT_ONLY:
        raise PermissionError("smoke requires POLICY_REJECT_ONLY runtime mode")
    if runtime.can_tx_enabled or runtime.command_tx_enabled:
        raise PermissionError("CAN and command transmission must remain disabled")
    identity_lock = load_artifact_identity_lock(identity_lock_path)
    observation = make_piper_proxy_warmup_observation()
    validate_piper_semantic_observation(observation)

    _check_loopback_listener(port, min(timeout_s, 5.0))
    client = None
    try:
        client = (client_factory or _client_factory(port))()
        metadata = client.get_server_metadata()
        observation_ns = time.perf_counter_ns()
        result = _bounded_prediction(client, observation, timeout_s)
        response_ns = time.perf_counter_ns()
        prediction = build_policy_prediction(
            result,
            metadata,
            request_id=f"synthetic-reject-smoke-{observation_ns}",
            request_sequence=0,
            observation_ns=observation_ns,
            response_ns=response_ns,
            expected_identity_lock=identity_lock,
        )

        # No CAN timestamps or hardware status are supplied: synthetic state
        # must stay explicitly disconnected and hardware units unverified.
        source_record = {
            "request_id": prediction.request_id,
            "request_sequence": prediction.request_sequence,
            "observation_built_ns": prediction.observation_monotonic_ns,
            "response_ns": prediction.received_monotonic_ns,
            "state": np.asarray(observation["state"], dtype=np.float64).tolist(),
            "robot_status": {},
            "can_timestamps_ns": {},
            "target_dt_s": [prediction.target_dt_s[0]],
            "first_predicted_proxy": list(prediction.actions[0]),
            "checkpoint_identity": prediction.checkpoint_identity,
            "config_identity": prediction.config_identity,
            "normalization_identity": prediction.normalization_identity,
            "dataset_identity": prediction.dataset_identity,
        }
        audit = audit_report(
            {"predictions": [source_record], "units_hardware_verified": False},
            runtime,
        )
        decision = audit["decisions"][0]
        required_reasons = {
            "next_observed_state_proxy_is_not_command",
            "proxy_to_command_mapping_unverified",
            "runtime_mode_forbids_can_tx",
            "robot_disconnected",
            "state_units_unverified",
        }
        reasons = set(decision["reasons"])
        if not required_reasons.issubset(reasons):
            raise AssertionError(
                "reject-only decision omitted required fail-closed reasons: "
                + ", ".join(sorted(required_reasons - reasons))
            )
        if (
            decision["hardware_control_allowed"] is not False
            or decision["transport_send_attempted"] is not False
            or audit["transport_send_attempts"] != 0
            or audit["can_data_tx_calls"] != 0
        ):
            raise AssertionError("SafetyBridge did not remain zero-transmit")

        return {
            "schema": "piper_synthetic_policy_reject_smoke_v1",
            "status": "PASS_SYNTHETIC_POLICY_REJECTED_NO_TX",
            "runtime_mode": runtime.mode.value,
            "input_kind": "synthetic_dual_rgb_state",
            "action_shape": list(np.asarray(result["actions"]).shape),
            "first_predicted_proxy": list(prediction.actions[0]),
            "action_semantics": prediction.action_semantics,
            "server_checkpoint": metadata["checkpoint"],
            "server_config": metadata["config_schema"],
            "safety_decision": decision,
            "transport_send_attempts": audit["transport_send_attempts"],
            "can_data_tx_calls": audit["can_data_tx_calls"],
            "camera_opened": False,
            "can_opened": False,
            "hardware_control_allowed": False,
            "robot_command_sent": False,
        }
    finally:
        _close_client(client)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=9667)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--runtime-config", type=Path, default=DEFAULT_RUNTIME_CONFIG)
    parser.add_argument("--identity-lock", type=Path, default=DEFAULT_IDENTITY_LOCK)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = run(
        port=args.port,
        timeout_s=args.timeout_s,
        runtime_config_path=args.runtime_config,
        identity_lock_path=args.identity_lock,
    )
    encoded = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
