"""Serve the public-data pi0.5 smoke policy on localhost, prediction only."""

from __future__ import annotations

import argparse

from openpi.policies import policy_config
from openpi.serving.websocket_policy_server import WebsocketPolicyServer

from server.training.public_piper_smoke import make_config
from server.training.public_data_contract import PUBLIC_ACTION_SEMANTICS, PUBLIC_ACTION_UNITS


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8001)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise ValueError("invalid port")
    config = make_config()
    checkpoint = config.checkpoint_dir / "1"
    if not checkpoint.is_dir():
        raise FileNotFoundError(checkpoint)
    policy = policy_config.create_trained_policy(config, checkpoint)
    server = WebsocketPolicyServer(
        policy,
        host="127.0.0.1",
        port=args.port,
        metadata={
            "project": "07_pi0.5",
            "config": config.name,
            "checkpoint": str(checkpoint),
            "source_dataset": "angkul07/piper-single-arm-cotrain/mrfood1.tar",
            "action_semantics": PUBLIC_ACTION_SEMANTICS,
            "action_units": list(PUBLIC_ACTION_UNITS),
            "robot_control_ready": False,
            "control_enabled": False,
        },
    )
    print(f"PREDICTION_ONLY_LISTEN=127.0.0.1:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
