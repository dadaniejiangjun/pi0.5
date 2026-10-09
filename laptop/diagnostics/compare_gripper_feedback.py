"""Compare two passive CAN logs with an operator-observed OEM gripper display.

This is feedback-scale evidence only. It never validates a control target,
opens CAN, or grants hardware actuation permission.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from pathlib import Path

from laptop.diagnostics.audit_feedback_log import audit


def _settled_gripper(path: Path) -> tuple[float, int]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ValueError("empty passive feedback log")
    start_ns = min(row["monotonic_ns"] for row in rows)
    samples = [row["gripper_raw"]["angle_or_stroke"] for row in rows
               if row["can_id"] == "0x2A8" and row["monotonic_ns"] >= start_ns + 500_000_000]
    if len(samples) < 2:
        raise ValueError("insufficient settled gripper feedback")
    return statistics.median(samples), len(samples)


def compare(before: Path, after: Path, *, oem_feedback_mm: float,
            oem_target_mm: float | None = None) -> dict:
    before, after = Path(before), Path(after)
    for path in (before, after):
        if audit(path)["status"] != "READ_CONTINUITY_PASS":
            raise ValueError(f"passive feedback continuity failed: {path}")
    if not 0 <= oem_feedback_mm <= 100 or (
        oem_target_mm is not None and not 0 <= oem_target_mm <= 100
    ):
        raise ValueError("invalid OEM display value")
    before_raw, before_n = _settled_gripper(before)
    after_raw, after_n = _settled_gripper(after)
    if before_raw == after_raw:
        raise ValueError("no observed gripper feedback change")
    observed_after_mm = after_raw / 1000.0
    display_error_mm = abs(observed_after_mm - oem_feedback_mm)
    return {
        "schema": "piper_gripper_feedback_comparison_v1",
        "status": "FEEDBACK_SCALE_OBSERVED" if display_error_mm <= 0.5 else "DISPLAY_MISMATCH",
        "before_log_sha256": hashlib.sha256(before.read_bytes()).hexdigest(),
        "after_log_sha256": hashlib.sha256(after.read_bytes()).hexdigest(),
        "before_raw_median": before_raw,
        "after_raw_median": after_raw,
        "raw_delta": after_raw - before_raw,
        "settled_sample_counts": [before_n, after_n],
        "candidate_feedback_scale": "1000 raw units per mm",
        "oem_feedback_mm": oem_feedback_mm,
        "after_raw_as_mm": observed_after_mm,
        "display_error_mm": display_error_mm,
        "oem_target_mm": oem_target_mm,
        "target_mapping_verified": False,
        "joint_motion_cause": "operator-reported manual pose adjustment",
        "hardware_control_allowed": False,
        "robot_command_sent": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--oem-feedback-mm", required=True, type=float)
    parser.add_argument("--oem-target-mm", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = compare(args.before, args.after, oem_feedback_mm=args.oem_feedback_mm,
                     oem_target_mm=args.oem_target_mm)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "FEEDBACK_SCALE_OBSERVED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
