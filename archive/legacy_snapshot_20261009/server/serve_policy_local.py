#!/usr/bin/env python3
"""Run the official OpenPI WebSocket server on localhost only.

This wrapper does not modify the official checkout. It uses the official
policy/config/server classes and hard-codes the bind address to 127.0.0.1.
The default is the official pi05_aloha + pi05_base load sanity, not a PiPER
policy and not a robot-control path.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys

from openpi.policies import policy_config
from openpi.serving import websocket_policy_server
from openpi.training import config as training_config


LOCAL_HOST = "127.0.0.1"
DEFAULT_PORT = 8000
DEFAULT_CONFIG = "pi05_aloha"
DEFAULT_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_base"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--default-prompt", default=None)
    parser.add_argument("--host", default=LOCAL_HOST, help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.host != LOCAL_HOST:
        raise SystemExit("SECURITY_GATE=FAIL: host is fixed to 127.0.0.1")
    if not 1 <= args.port <= 65535:
        raise SystemExit("SECURITY_GATE=FAIL: invalid port")

    logging.basicConfig(level=logging.INFO, force=True)
    policy = policy_config.create_trained_policy(
        training_config.get_config(args.config),
        args.checkpoint,
        default_prompt=args.default_prompt,
    )
    server = websocket_policy_server.WebsocketPolicyServer(
        policy=policy,
        host=LOCAL_HOST,
        port=args.port,
        metadata={
            **policy.metadata,
            "project": "07_pi0.5",
            "bind_address": LOCAL_HOST,
            "config": args.config,
            "checkpoint": args.checkpoint,
            "piper_policy": False,
        },
    )
    print(f"LISTEN_ADDRESS={LOCAL_HOST}:{args.port}", flush=True)
    print("PIPER_CONTROL=DISABLED", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("SERVER_SHUTDOWN=OK", flush=True)
        return 0
    except Exception:
        logging.exception("LOCAL_POLICY_SERVER=FAIL")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
