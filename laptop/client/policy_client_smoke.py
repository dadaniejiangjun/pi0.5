#!/usr/bin/env python3
"""Run sequential official openpi-client requests against localhost."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import numpy as np
from openpi_client.websocket_client_policy import WebsocketClientPolicy

from observation_schema import make_official_aloha_dummy_observation


def _percentile(values: list[float], percentile: float) -> float | None:
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile)) if values else None


def run_roundtrip(host: str, port: int, request_count: int, seed: int = 0) -> dict:
    start_connect = time.perf_counter()
    client = WebsocketClientPolicy(host=host, port=port)
    connect_ms = (time.perf_counter() - start_connect) * 1000.0
    metadata = client.get_server_metadata()
    latencies: list[float] = []
    server_infer_ms: list[float] = []
    failures: list[dict] = []
    response_shape = None
    for index in range(request_count):
        observation = make_official_aloha_dummy_observation(seed=seed + index)
        started = time.perf_counter()
        try:
            response = client.infer(observation)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            actions = np.asarray(response["actions"])
            if actions.ndim != 2 or actions.shape[1] != 14 or not np.isfinite(actions).all():
                raise ValueError(f"invalid official ALOHA response shape/finite check: {actions.shape}")
            timing = response.get("server_timing", {})
            if "infer_ms" not in timing:
                raise ValueError("server_timing.infer_ms missing")
            latencies.append(elapsed_ms)
            server_infer_ms.append(float(timing["infer_ms"]))
            response_shape = list(actions.shape)
        except Exception as exc:
            failures.append({"index": index, "error": repr(exc)})
    payload = {
        "status": "PASS" if len(failures) == 0 and len(latencies) == request_count else "FAIL",
        "host": host,
        "port": port,
        "request_count": request_count,
        "successful_responses": len(latencies),
        "failures": failures,
        "connect_ms": connect_ms,
        "client_roundtrip_ms": {
            "mean": statistics.fmean(latencies) if latencies else None,
            "median": statistics.median(latencies) if latencies else None,
            "p90": _percentile(latencies, 90),
            "p95": _percentile(latencies, 95),
            "max": max(latencies) if latencies else None,
        },
        "server_infer_ms": {
            "mean": statistics.fmean(server_infer_ms) if server_infer_ms else None,
            "median": statistics.median(server_infer_ms) if server_infer_ms else None,
            "p90": _percentile(server_infer_ms, 90),
            "p95": _percentile(server_infer_ms, 95),
            "max": max(server_infer_ms) if server_infer_ms else None,
        },
        "response_shape": response_shape,
        "server_metadata": metadata,
        "policy_semantics": "official_pi05_aloha_sanity_only; not_piper_policy",
    }
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--requests", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = run_roundtrip(args.host, args.port, args.requests)
    text = json.dumps(report, indent=2, default=str) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
