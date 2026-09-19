#!/usr/bin/env python3
"""Measure a 25-request official base server run with GPU sampling."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from run_local_policy_server_smoke import ROOT, wait_for_listen


def gpu_sample() -> dict:
    result = subprocess.run(
        [
            "nvidia-smi",
            "--query-gpu=index,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits",
            "-i",
            "0",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    line = result.stdout.strip().splitlines()[0] if result.stdout.strip() else ""
    fields = [field.strip() for field in line.split(",")]
    if len(fields) == 3:
        return {"index": int(fields[0]), "memory_used_mib": int(fields[1]), "utilization_gpu_percent": int(fields[2])}
    return {"error": result.stderr.strip() or "nvidia-smi returned no sample"}


def main() -> int:
    request_count = 25
    port = 8000
    env = os.environ.copy()
    env.update(
        {
            "CUDA_VISIBLE_DEVICES": "0",
            "XLA_PYTHON_CLIENT_PREALLOCATE": "false",
            "XLA_PYTHON_CLIENT_MEM_FRACTION": "0.25",
            "HF_HOME": str(ROOT / "cache/huggingface"),
            "XDG_CACHE_HOME": str(ROOT / "cache/xdg"),
            "UV_CACHE_DIR": str(ROOT / "cache/uv"),
            "OPENPI_DATA_HOME": str(ROOT / "cache/openpi"),
            "PYTHONUNBUFFERED": "1",
        }
    )
    server = subprocess.Popen(
        [sys.executable, str(ROOT / "server/serve_policy_local.py"), "--port", str(port)],
        cwd=ROOT / "third_party/openpi",
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    ready, lines = wait_for_listen(server, 900.0)
    client_output = ROOT / "reports/local_client_performance_25.json"
    samples = []
    client = None
    if ready:
        client = subprocess.Popen(
            [
                sys.executable,
                str(ROOT / "laptop/client/policy_client_smoke.py"),
                "--host",
                "127.0.0.1",
                "--port",
                str(port),
                "--requests",
                str(request_count),
                "--output",
                str(client_output),
            ],
            cwd=ROOT / "laptop/client",
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        while client.poll() is None:
            samples.append(gpu_sample())
            time.sleep(0.25)
        samples.append(gpu_sample())
    if server.poll() is None:
        server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=30)
        except subprocess.TimeoutExpired:
            server.terminate()
            server.wait(timeout=10)
    if client is not None and client.stdout is not None:
        client.stdout.read()
    if server.stdout is not None:
        server.stdout.read()
    client_report = json.loads(client_output.read_text(encoding="utf-8")) if client_output.exists() else {}
    infer = client_report.get("server_infer_ms", {})
    warm_values = client_report.get("server_infer_ms", {}).get("values", [])
    # The client report intentionally stores summary values; 25 requests means
    # at least 24 warm requests after the first request in this fresh process.
    performance = {
        "status": "PASS" if ready and client_report.get("successful_responses") == request_count else "FAIL",
        "config": "pi05_aloha",
        "checkpoint": "gs://openpi-assets/checkpoints/pi05_base",
        "request_count": request_count,
        "warm_request_count_minimum": request_count - 1,
        "warm_request_gate": request_count - 1 >= 20,
        "server_infer_ms": infer,
        "client_roundtrip_ms": client_report.get("client_roundtrip_ms"),
        "response_shape": client_report.get("response_shape"),
        "gpu0_samples_during_client": samples,
        "peak_gpu0_memory_used_mib": max(
            (sample.get("memory_used_mib") for sample in samples if "memory_used_mib" in sample), default=None
        ),
        "peak_gpu0_utilization_percent": max(
            (sample.get("utilization_gpu_percent") for sample in samples if "utilization_gpu_percent" in sample), default=None
        ),
        "client_report": str(client_output),
        "piper_policy": False,
        "robot_accessed": False,
    }
    output = ROOT / "reports/pi05_server_performance.json"
    output.write_text(json.dumps(performance, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(performance, indent=2))
    return 0 if performance["status"] == "PASS" and performance["warm_request_gate"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
