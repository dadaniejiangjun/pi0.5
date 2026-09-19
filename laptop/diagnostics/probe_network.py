#!/usr/bin/env python3
"""Read-only localhost/SSH-tunnel health and one official payload probe."""

from __future__ import annotations

import argparse
import json
import socket
import time
import urllib.request
from pathlib import Path

import numpy as np
from openpi_client.websocket_client_policy import WebsocketClientPolicy


def dummy_aloha_observation(seed: int = 0) -> dict:
    rng = np.random.default_rng(seed)
    return {
        "state": np.ones((14,), dtype=np.float32),
        "images": {
            "cam_high": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
            "cam_low": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
            "cam_left_wrist": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
            "cam_right_wrist": rng.integers(0, 256, (3, 224, 224), dtype=np.uint8),
        },
        "prompt": "do something",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--requests", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    timings = []
    failures = []
    for index in range(args.requests):
        started = time.perf_counter()
        try:
            with urllib.request.urlopen(f"http://{args.host}:{args.port}/healthz", timeout=3) as response:
                if response.status != 200:
                    raise RuntimeError(f"healthz status {response.status}")
            timings.append((time.perf_counter() - started) * 1000.0)
        except Exception as exc:
            failures.append({"index": index, "error": repr(exc)})
    payload_ok = False
    payload_error = None
    try:
        with socket.create_connection((args.host, args.port), timeout=3):
            pass
        client = WebsocketClientPolicy(host=args.host, port=args.port)
        response = client.infer(dummy_aloha_observation())
        actions = np.asarray(response["actions"])
        payload_ok = actions.ndim == 2 and actions.shape[1] == 14 and np.isfinite(actions).all()
    except Exception as exc:
        payload_error = repr(exc)
    report = {
        "status": "PASS" if not failures and payload_ok else "FAIL",
        "host": args.host,
        "port": args.port,
        "healthz_request_count": args.requests,
        "healthz_successes": len(timings),
        "healthz_failures": failures,
        "healthz_median_ms": float(np.median(timings)) if timings else None,
        "websocket_payload_test": payload_ok,
        "websocket_payload_error": payload_error,
        "robot_accessed": False,
        "motion_tested": False,
    }
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
