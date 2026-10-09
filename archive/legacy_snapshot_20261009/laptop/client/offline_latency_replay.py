"""Time saved dual-camera observations over a prediction-only SSH tunnel.

No CAN access, live cameras, PiPER SDK, or robot transport is used. Raw images
are sent only to a loopback policy server through an existing private tunnel.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import time

import numpy as np

from laptop.client.live_shadow import segment_latency
from laptop.client.observation_schema import PIPER_PROMPT, validate_piper_semantic_observation


def _first_rgb(path: Path) -> np.ndarray:
    import cv2
    camera = cv2.VideoCapture(str(path))
    try:
        ok, frame = camera.read()
        if not ok or frame is None:
            raise ValueError(f"no video frame: {path}")
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    finally:
        camera.release()


def replay(*, base_video: Path, wrist_video: Path, shadow_report: Path,
           port: int, samples: int = 3) -> dict:
    if not 1 <= samples <= 10 or not 1 <= port <= 65535:
        raise ValueError("invalid bounded replay settings")
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    prior = json.loads(shadow_report.read_text(encoding="utf-8"))
    if (prior.get("status") != "LIVE_SHADOW_CONTINUOUS_PASS"
            or prior.get("robot_command_sent") is not False
            or not prior.get("predictions")):
        raise ValueError("not a verified saved prediction-only shadow report")
    state = np.asarray(prior["predictions"][0]["state"], dtype=np.float32)
    observation = {
        "base_rgb": _first_rgb(base_video),
        "wrist_rgb": _first_rgb(wrist_video),
        "state": state,
        "prompt": PIPER_PROMPT,
    }
    validate_piper_semantic_observation(observation)
    client = WebsocketClientPolicy(host="127.0.0.1", port=port)
    metadata = client.get_server_metadata()
    if (metadata.get("prediction_only") is not True
            or metadata.get("control_enabled") is not False
            or metadata.get("robot_control_ready") is not False
            or metadata.get("action_semantics") != "next_observed_state_proxy_nominal_units"):
        raise PermissionError("not the approved prediction-only proxy server")
    measurements = []
    for _ in range(samples + 1):  # First request warms the path.
        start_ns = time.perf_counter_ns()
        result = client.infer(observation)
        roundtrip_ms = (time.perf_counter_ns() - start_ns) / 1e6
        action = np.asarray(result["actions"])
        if action.shape != (16, 7) or not np.isfinite(action).all():
            raise ValueError("invalid model prediction")
        measurements.append(segment_latency(result, roundtrip_ms))
    measured = measurements[1:]
    if not measured:
        raise ValueError("zero measured requests")
    return {
        "status": "RECORDED_OBSERVATION_TUNNEL_LATENCY",
        "samples": len(measured),
        "image_shapes": {key: list(observation[key].shape) for key in ("base_rgb", "wrist_rgb")},
        "median_ms": {key: statistics.median(item[key] for item in measured)
                      for key in ("model_sampling_ms", "server_non_model_ms",
                                  "client_server_residual_ms", "client_roundtrip_ms")},
        "measurements": measured,
        "cold_first_request_ms": measurements[0]["client_roundtrip_ms"],
        "model_checkpoint": metadata.get("checkpoint"),
        "hardware_control_allowed": False,
        "robot_command_sent": False,
        "can_accessed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-video", required=True, type=Path)
    parser.add_argument("--wrist-video", required=True, type=Path)
    parser.add_argument("--shadow-report", required=True, type=Path)
    parser.add_argument("--port", type=int, default=8003)
    parser.add_argument("--samples", type=int, default=3)
    args = parser.parse_args()
    print(json.dumps(replay(base_video=args.base_video, wrist_video=args.wrist_video,
                            shadow_report=args.shadow_report, port=args.port,
                            samples=args.samples), indent=2))


if __name__ == "__main__":
    main()
