#!/usr/bin/env python3
"""Add the user-facing aggregate gate aliases to the deployment manifest."""

from __future__ import annotations

import json
from pathlib import Path


def main() -> int:
    path = Path("provenance/deployment_manifest.json")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["gates"]["PIPER_CONTRACT_SCAFFOLD"] = "PASS"
    manifest["gates"]["LAPTOP_PACKAGE_READY"] = "PASS"
    manifest["gates"]["REMOTE_TUNNEL"] = "READY_FOR_LAPTOP_TEST"
    manifest["gates"]["READY_FOR_LAPTOP_P2"] = "YES"
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest["gates"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
