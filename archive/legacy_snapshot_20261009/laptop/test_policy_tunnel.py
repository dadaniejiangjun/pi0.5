#!/usr/bin/env python3
"""Laptop-side SSH-tunnel readiness check; never a robot test."""

from __future__ import annotations

import argparse
import json
import socket
import urllib.request
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {
        "status": "READY_FOR_LAPTOP_TEST",
        "host": args.host,
        "port": args.port,
        "tcp_connectivity": False,
        "healthz": False,
        "robot_accessed": False,
        "motion_tested": False,
        "note": "Run only after the user establishes the SSH tunnel on the laptop.",
    }
    try:
        with socket.create_connection((args.host, args.port), timeout=3):
            result["tcp_connectivity"] = True
        with urllib.request.urlopen(f"http://{args.host}:{args.port}/healthz", timeout=3) as response:
            result["healthz"] = response.status == 200
    except OSError as exc:
        result["error"] = repr(exc)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    else:
        print(json.dumps(result, indent=2))
    return 0 if result["tcp_connectivity"] and result["healthz"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
