#!/usr/bin/env python3
"""Small wrapper around the dependency-light official client benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from policy_client_smoke import run_roundtrip


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--requests", type=int, default=20)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = run_roundtrip(args.host, args.port, args.requests)
    report["probe_type"] = "latency_probe"
    text = json.dumps(report, indent=2, default=str) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
