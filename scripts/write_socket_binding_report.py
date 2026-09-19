#!/usr/bin/env python3
"""Persist the observed localhost-only binding from the server smoke."""

from __future__ import annotations

import json
from pathlib import Path


def main() -> int:
    report_path = Path("reports/local_policy_server_smoke.json")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    output = Path("validation/socket_binding.txt")
    output.parent.mkdir(parents=True, exist_ok=True)
    text = (
        "COMMAND: ss -lntp\n"
        f"OBSERVED_SOCKET_SNAPSHOT:\n{report.get('socket_snapshot', '')}\n"
        f"REQUIRED_BINDING: {report.get('listen_address_required')}\n"
        f"SOCKET_BINDING_LOCAL_ONLY: {report.get('socket_binding_local_only')}\n"
        f"SECURITY_GATE: {'PASS' if report.get('socket_binding_local_only') else 'FAIL'}\n"
    )
    output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if report.get("socket_binding_local_only") else 1


if __name__ == "__main__":
    raise SystemExit(main())
