"""Private-tunnel π0.5 request with synthetic input; never controls PiPER."""

from __future__ import annotations

import argparse
import json

import numpy as np
from openpi_client.websocket_client_policy import WebsocketClientPolicy
from server.training.public_data_contract import PUBLIC_ACTION_SEMANTICS, PUBLIC_ACTION_UNITS


def offline_veto(state: np.ndarray, actions: np.ndarray) -> list[str]:
    """Explain why this smoke prediction must never be sent to hardware."""
    state = np.asarray(state)
    actions = np.asarray(actions)
    if state.shape != (7,) or actions.shape != (16, 7) or not np.isfinite(actions).all():
        return ["malformed_prediction"]
    reasons = ["local_joint_and_gripper_command_mapping_unverified"]
    if float(np.max(np.abs(actions[0, :6] - state[:6]))) > 1.0:
        reasons.append("first_step_exceeds_1_degree_smoke_limit")
    if not 0.0 <= float(actions[0, 6]) <= 1.0:
        reasons.append("gripper_outside_binary_fraction_range")
    return reasons


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18001)
    args = parser.parse_args()
    client = WebsocketClientPolicy(host="127.0.0.1", port=args.port)
    metadata = client.get_server_metadata()
    if metadata.get("control_enabled") is not False or metadata.get("robot_control_ready") is not False:
        raise ValueError("unexpected server safety metadata")
    if (metadata.get("action_semantics") != PUBLIC_ACTION_SEMANTICS
            or metadata.get("action_units") != list(PUBLIC_ACTION_UNITS)):
        raise ValueError("public-data action contract missing or mismatched")
    state = np.array([20.293, 80.439, -13.454, -15.456, -33.139, 88.388, 1.0], dtype=np.float32)
    blank = np.zeros((240, 320, 3), dtype=np.uint8)
    result = client.infer({
        "base_rgb": blank,
        "wrist_rgb": blank.copy(),
        "state": state,
        "prompt": "pick and lift",
    })
    actions = np.asarray(result["actions"])
    if actions.shape != (16, 7) or not np.isfinite(actions).all():
        raise ValueError(f"invalid prediction {actions.shape}")
    print(json.dumps({
        "status": "PRIVATE_TUNNEL_PREDICTION_ONLY_PASS",
        "input": "synthetic pose and blank cameras; no hardware sampled",
        "action_shape": list(actions.shape),
        "first_action": actions[0].tolist(),
        "max_first_step_joint_delta_deg": float(np.max(np.abs(actions[0, :6] - state[:6]))),
        "veto_reasons": offline_veto(state, actions),
        "control_enabled": False,
    }, indent=2))


if __name__ == "__main__":
    main()
