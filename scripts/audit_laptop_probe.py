#!/usr/bin/env python3
"""Static safety audit for the project-local PiPER read-only probe."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path


FORBIDDEN_IDENTIFIERS = (
    "JointCtrl",
    "EndPoseCtrl",
    "GripperCtrl",
    "EnableArm",
    "DisableArm",
    "EmergencyStop",
    "ResetPiper",
    "ModeCtrl",
    "MotionCtrl",
)


def audit(source_path: Path) -> dict:
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(source_path))
    identifiers = [node.id for node in ast.walk(tree) if isinstance(node, ast.Name)]
    attributes = [node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)]
    strings = [node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    hits = sorted(
        {
            forbidden
            for forbidden in FORBIDDEN_IDENTIFIERS
            if forbidden in identifiers or forbidden in attributes or forbidden in strings or forbidden in source
        }
    )
    return {
        "source": str(source_path),
        "source_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest(),
        "forbidden_identifiers": list(FORBIDDEN_IDENTIFIERS),
        "forbidden_hits": hits,
        "read_only_probe_safety": "PASS" if not hits else "FAIL",
        "hardware_accessed": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=Path(__file__).parents[1] / "laptop/diagnostics/probe_piper_readonly.py",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.source)
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report["read_only_probe_safety"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
