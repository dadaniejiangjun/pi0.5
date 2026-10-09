"""Offline P2 continuity check for an ACK-only feedback JSONL log."""

from __future__ import annotations

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path


REQUIRED = ("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8")


def audit(path: Path) -> dict:
    timestamps = defaultdict(list)
    joints = set()
    grippers = set()
    modes = set()
    rows = []
    malformed = 0
    with Path(path).open(encoding="utf-8") as handle:
        for line in handle:
            try:
                row = json.loads(line)
                timestamps[row["can_id"]].append(row["monotonic_ns"])
                joints.add(tuple(row["joint_raw_0p001deg_nominal"]))
                grippers.add(row["gripper_raw"]["angle_or_stroke"])
                modes.add(row["status"]["ctrl_mode"])
                rows.append(row)
            except (ValueError, TypeError, KeyError):
                malformed += 1
    channel_stats = {}
    for can_id, times in sorted(timestamps.items()):
        intervals = [(b - a) / 1e6 for a, b in zip(times, times[1:])]
        channel_stats[can_id] = {
            "count": len(times),
            "duplicate_timestamp_pairs": sum(b == a for a, b in zip(times, times[1:])),
            "backward_timestamp_pairs": sum(b < a for a, b in zip(times, times[1:])),
            "median_interval_ms": statistics.median(intervals) if intervals else None,
            "max_interval_ms": max(intervals) if intervals else None,
        }
    missing = [can_id for can_id in REQUIRED if not timestamps[can_id]]
    sparse = [can_id for can_id in REQUIRED if len(timestamps[can_id]) < 2]
    long_gap = [can_id for can_id in REQUIRED
                if channel_stats.get(can_id, {}).get("max_interval_ms") is not None
                and channel_stats[can_id]["max_interval_ms"] > 100]
    span_s = ((max(max(times) for times in timestamps.values() if times)
               - min(min(times) for times in timestamps.values() if times)) / 1e9
              if any(timestamps.values()) else 0.0)
    coarse_clock = any(v["duplicate_timestamp_pairs"] for v in channel_stats.values())
    other_failure = bool(missing or sparse or long_gap or span_s < 2 or malformed
                         or any(v["backward_timestamp_pairs"] for v in channel_stats.values()))
    settled_rows = []
    if rows:
        settled_after_ns = min(row["monotonic_ns"] for row in rows) + 500_000_000
        settled_rows = [row for row in rows if row["monotonic_ns"] >= settled_after_ns]
    settled_joint_snapshots = {tuple(row["joint_raw_0p001deg_nominal"]) for row in settled_rows}
    settled_gripper_values = {row["gripper_raw"]["angle_or_stroke"] for row in settled_rows}
    settled_payload_variants = {
        can_id: len({row.get("raw_payload_hex") for row in settled_rows
                     if row.get("can_id") == can_id and row.get("raw_payload_hex") is not None})
        for can_id in REQUIRED
    }
    return {
        "status": ("INCOMPLETE" if other_failure else
                   "CLOCK_RESOLUTION_INSUFFICIENT" if coarse_clock else "READ_CONTINUITY_PASS"),
        "missing_ids": missing, "sparse_ids": sparse, "ids_with_gap_over_100ms": long_gap,
        "span_s": span_s, "malformed_rows": malformed, "channels": channel_stats,
        "distinct_joint_snapshots": len(joints), "distinct_gripper_values": len(grippers),
        "distinct_joint_snapshots_after_0p5s": len(settled_joint_snapshots),
        "distinct_gripper_values_after_0p5s": len(settled_gripper_values),
        "raw_payload_variants_after_0p5s": settled_payload_variants,
        "observed_modes": sorted(modes),
        "manual_motion_response_verified": False,
        "gripper_operation_verified": False,
        "note": "Variation is descriptive; operator must correlate a known manual event with timestamps.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    report = audit(args.log)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "READ_CONTINUITY_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
