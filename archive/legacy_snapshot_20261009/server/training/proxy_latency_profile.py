"""Offline loopback latency profile of the pinned PiPER proxy checkpoint.

Loads one already-converted episode on the GPU server. No cameras, CAN,
robot SDK, remote listener, or hardware command is involved.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import statistics
import threading
import time


def profile(*, num_steps: int = 10, warmup: int = 2, samples: int = 5) -> dict:
    if not 2 <= num_steps <= 10 or not 1 <= warmup <= 5 or not 1 <= samples <= 20:
        raise ValueError("invalid bounded profile settings")
    from server.training.proxy_piper_tiny_fit import DATA_ROOT, DATASET, inspect_dataset, make_config

    source = inspect_dataset()
    os.environ["HF_LEROBOT_HOME"] = str(DATA_ROOT)
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
    from openpi.policies import policy_config
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    import jax
    import numpy as np
    config = make_config()
    checkpoint = config.checkpoint_dir / "1"
    if not checkpoint.is_dir():
        raise FileNotFoundError(checkpoint)
    policy = policy_config.create_trained_policy(
        config, checkpoint, sample_kwargs={"num_steps": num_steps})
    original_sample = policy._sample_actions

    def synchronous_sample(*args, **kwargs):
        return jax.block_until_ready(original_sample(*args, **kwargs))

    policy._sample_actions = synchronous_sample  # Diagnostic timing only; no checkpoint mutation.
    dataset = LeRobotDataset(DATASET)
    row = dataset[20]
    observation = {
        "base_rgb": row["observation.images.image"].numpy(),
        "wrist_rgb": row["observation.images.wrist_image"].numpy(),
        "state": row["observation.state"].numpy(),
        "prompt": row["task"],
    }
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    server = WebsocketPolicyServer(policy, host="127.0.0.1", port=port,
                                   metadata={"prediction_only": True, "control_enabled": False})
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = WebsocketClientPolicy(host="127.0.0.1", port=port)
    if client.get_server_metadata() != {"prediction_only": True, "control_enabled": False}:
        raise ValueError("unexpected loopback server metadata")
    measurements = []
    for index in range(warmup + samples):
        start_ns = time.perf_counter_ns()
        result = client.infer(observation)
        roundtrip_ms = (time.perf_counter_ns() - start_ns) / 1e6
        model_ms = result.get("policy_timing", {}).get("infer_ms")
        server_ms = result.get("server_timing", {}).get("infer_ms")
        if (not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in (model_ms, server_ms, roundtrip_ms))
                or model_ms < 0 or server_ms < model_ms or roundtrip_ms < server_ms):
            raise ValueError("missing or inconsistent OpenPI timing")
        actions = np.asarray(result["actions"])
        if actions.shape != (16, 7) or not np.isfinite(actions).all():
            raise ValueError("invalid proxy prediction")
        if index >= warmup:
            measurements.append({
                "model_sampling_ms": model_ms,
                "server_non_model_ms": server_ms - model_ms,
                "client_server_residual_ms": roundtrip_ms - server_ms,
                "client_roundtrip_ms": roundtrip_ms,
            })
    if not measurements:
        raise ValueError("zero measured samples")
    return {
        "status": "OFFLINE_LOOPBACK_LATENCY_PROFILE",
        "source": source,
        "checkpoint": str(checkpoint),
        "num_steps": num_steps,
        "warmup_requests": warmup,
        "measured_requests": len(measurements),
        "median_ms": {key: statistics.median(item[key] for item in measurements)
                      for key in measurements[0]},
        "samples": measurements,
        "model_timer_jax_synchronized": True,
        "residual_includes": "client serialization, loopback transport, server serialization and scheduling",
        "real_ssh_tunnel_network_measured": False,
        "hardware_control_allowed": False,
        "robot_command_sent": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-steps", type=int, default=10)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--samples", type=int, default=5)
    args = parser.parse_args()
    print(json.dumps(profile(num_steps=args.num_steps, warmup=args.warmup,
                             samples=args.samples), indent=2))


if __name__ == "__main__":
    main()
