#!/usr/bin/env python3
"""Start, audit, exercise, and cleanly stop the local policy server."""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def socket_snapshot(port: int) -> str:
    result = subprocess.run(["ss", "-lntp"], check=False, capture_output=True, text=True)
    lines = [line for line in result.stdout.splitlines() if f":{port}" in line]
    return "\n".join(lines)


def wait_for_listen(proc: subprocess.Popen, timeout: float) -> tuple[bool, list[str]]:
    lines: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.stdout is None:
            break
        readable, _, _ = select.select([proc.stdout], [], [], 1.0)
        if readable:
            line = proc.stdout.readline()
            if line:
                lines.append(line.rstrip())
                if line.startswith("LISTEN_ADDRESS=127.0.0.1:"):
                    return True, lines
        if proc.poll() is not None:
            break
    return False, lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--requests", type=int, default=100)
    parser.add_argument("--startup-timeout", type=float, default=900.0)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/local_policy_server_smoke.json")
    parser.add_argument("--client-output", type=Path, default=ROOT / "reports/local_client_roundtrip.json")
    args = parser.parse_args()

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
    server_cmd = [sys.executable, str(ROOT / "server/serve_policy_local.py"), "--port", str(args.port)]
    proc = subprocess.Popen(
        server_cmd,
        cwd=ROOT / "third_party/openpi",
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    started = time.time()
    ready, startup_lines = wait_for_listen(proc, args.startup_timeout)
    snapshot = socket_snapshot(args.port)
    local_only = any("127.0.0.1:" + str(args.port) in line or "::1:" + str(args.port) in line for line in snapshot.splitlines()) and not any(
        "0.0.0.0:" + str(args.port) in line or "*:" + str(args.port) in line for line in snapshot.splitlines()
    )
    healthz = False
    healthz_body = None
    if ready:
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{args.port}/healthz", timeout=10) as response:
                healthz = response.status == 200
                healthz_body = response.read().decode("utf-8", errors="replace")
        except Exception as exc:
            healthz_body = repr(exc)

    client_result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "laptop/client/policy_client_smoke.py"),
            "--host",
            "127.0.0.1",
            "--port",
            str(args.port),
            "--requests",
            str(args.requests),
            "--output",
            str(args.client_output),
        ],
        cwd=ROOT / "laptop/client",
        env=env,
        check=False,
        capture_output=True,
        text=True,
    ) if ready and healthz else None

    clean_shutdown = False
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=30)
            clean_shutdown = proc.returncode == 0
        except subprocess.TimeoutExpired:
            proc.terminate()
            proc.wait(timeout=10)
    remaining_output = ""
    if proc.stdout is not None:
        remaining_output = proc.stdout.read()
    log_path = ROOT / "logs/local_policy_server_smoke.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text("\n".join(startup_lines) + "\n" + remaining_output, encoding="utf-8")

    client_report = None
    if args.client_output.exists():
        try:
            client_report = json.loads(args.client_output.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            client_report = {"status": "INVALID_JSON"}
    passed = bool(
        ready
        and healthz
        and local_only
        and clean_shutdown
        and client_result is not None
        and client_result.returncode == 0
        and client_report
        and client_report.get("successful_responses") == args.requests
    )
    report = {
        "status": "PASS" if passed else "FAIL",
        "listen_address_required": f"127.0.0.1:{args.port}",
        "startup_ready": ready,
        "startup_seconds": time.time() - started,
        "healthz": healthz,
        "healthz_body": healthz_body,
        "socket_snapshot": snapshot,
        "socket_binding_local_only": local_only,
        "client_returncode": client_result.returncode if client_result else None,
        "client_report": client_report,
        "clean_shutdown": clean_shutdown,
        "server_log": str(log_path),
        "piper_policy": False,
        "robot_accessed": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, default=str))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
