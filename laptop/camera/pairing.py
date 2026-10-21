"""Best-effort camera pairing from shared host timestamps.

The project's DirectShow timestamps are taken immediately after
``VideoCapture.read``. They support host-time pairing and diagnostics, but are
not sensor exposure timestamps and cannot certify hardware synchronization.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable


def _field(value: Any, name: str) -> Any:
    if isinstance(value, dict):
        return value[name]
    return getattr(value, name)


def select_latest_camera_pair(
    base_candidates: Iterable[tuple[Any, Any]],
    wrist_candidates: Iterable[tuple[Any, Any]],
    *,
    max_skew_ns: int = 20_000_000,
) -> tuple[tuple[tuple[Any, Any], tuple[Any, Any], int] | None, int | None]:
    """Choose the newest unique-sequence pair inside a host-time tolerance.

    Candidates are ``(image, metadata)`` pairs. Selection prefers the newest
    common capture time, then the smallest skew. The second return value is the
    minimum skew across all candidate pairs, including rejected pairs.
    """
    if type(max_skew_ns) is not int or max_skew_ns <= 0:
        raise ValueError("max_skew_ns must be a positive integer")
    bases, wrists = list(base_candidates), list(wrist_candidates)
    best = None
    minimum_skew = None
    for base in bases:
        base_meta = base[1]
        base_ts = _field(base_meta, "source_monotonic_ns")
        base_seq = _field(base_meta, "sequence")
        if type(base_ts) is not int or base_ts <= 0:
            continue
        if type(base_seq) is not int or base_seq <= 0:
            continue
        for wrist in wrists:
            wrist_meta = wrist[1]
            wrist_ts = _field(wrist_meta, "source_monotonic_ns")
            wrist_seq = _field(wrist_meta, "sequence")
            if type(wrist_ts) is not int or wrist_ts <= 0:
                continue
            if type(wrist_seq) is not int or wrist_seq <= 0:
                continue
            skew = abs(base_ts - wrist_ts)
            if minimum_skew is None or skew < minimum_skew:
                minimum_skew = skew
            if skew > max_skew_ns:
                continue
            rank = (min(base_ts, wrist_ts), -skew, max(base_ts, wrist_ts))
            if best is None or rank > best[0]:
                best = (rank, (base, wrist, skew))
    return (best[1] if best is not None else None), minimum_skew


def _read_timestamps(path: Path) -> list[int]:
    values = []
    last = 0
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            timestamp = row.get("received_monotonic_ns", row.get("timestamp_ns"))
            if type(timestamp) is not int or timestamp <= last:
                raise ValueError(f"{path}:{line_number}: timestamp must be positive and increasing")
            values.append(timestamp)
            last = timestamp
    if len(values) < 2:
        raise ValueError(f"{path}: at least two timestamps are required")
    return values


def pair_timestamp_series(base: list[int], wrist: list[int]) -> list[tuple[int, int]]:
    """Monotonically pair closest available samples without reusing frames."""
    if any(type(value) is not int or value <= 0 for value in (*base, *wrist)):
        raise ValueError("timestamps must be positive integers")
    if any(right <= left for left, right in zip(base, base[1:])):
        raise ValueError("Base timestamps must be strictly increasing")
    if any(right <= left for left, right in zip(wrist, wrist[1:])):
        raise ValueError("Wrist timestamps must be strictly increasing")
    pairs = []
    i = j = 0
    while i < len(base) and j < len(wrist):
        current = abs(base[i] - wrist[j])
        next_base = abs(base[i + 1] - wrist[j]) if i + 1 < len(base) else math.inf
        next_wrist = abs(base[i] - wrist[j + 1]) if j + 1 < len(wrist) else math.inf
        if next_base < current and next_base <= next_wrist:
            i += 1
        elif next_wrist < current:
            j += 1
        else:
            pairs.append((base[i], wrist[j]))
            i += 1
            j += 1
    return pairs


def audit_timestamp_series(base: list[int], wrist: list[int], *, max_skew_ms: float = 20.0) -> dict:
    if not math.isfinite(max_skew_ms) or not 0 < max_skew_ms <= 100:
        raise ValueError("max_skew_ms must be within (0, 100]")
    if len(base) < 2 or len(wrist) < 2:
        raise ValueError("at least two timestamps per camera are required")
    pairs = pair_timestamp_series(base, wrist)
    if not pairs:
        raise ValueError("no monotonic Base/Wrist timestamp pairs could be formed")
    skews_ms = sorted(abs(a - b) / 1e6 for a, b in pairs)
    tolerance_ms = [5, 10, 15, 20, 25, 33, 50]
    return {
        "schema": "piper_host_timestamp_pair_audit_v1",
        "status": "HOST_TIMESTAMP_AUDIT_COMPLETE",
        "base_timestamp_count": len(base),
        "wrist_timestamp_count": len(wrist),
        "paired_count": len(pairs),
        "unpaired_base_count": len(base) - len(pairs),
        "unpaired_wrist_count": len(wrist) - len(pairs),
        "pair_skew_ms": {
            "median": statistics.median(skews_ms),
            "p95": skews_ms[int(0.95 * (len(skews_ms) - 1))],
            "max": max(skews_ms),
        },
        "coverage_by_limit_ms": {
            str(limit): {
                "paired_within_limit": sum(skew <= limit for skew in skews_ms),
                "fraction_of_pairs": sum(skew <= limit for skew in skews_ms) / len(skews_ms),
            }
            for limit in tolerance_ms
        },
        "selected_limit_ms": max_skew_ms,
        "selected_limit_pair_fraction": sum(skew <= max_skew_ms for skew in skews_ms) / len(skews_ms),
        "timestamp_clock": "shared Windows perf_counter host clock",
        "timestamp_semantics": "host_after_read_not_sensor_exposure_time",
        "hardware_exposure_sync_verified": False,
        "camera_can_sync_verified": False,
    }


def audit_timestamp_files(base_path: Path, wrist_path: Path, *, max_skew_ms: float = 20.0) -> dict:
    report = audit_timestamp_series(
        _read_timestamps(base_path), _read_timestamps(wrist_path),
        max_skew_ms=max_skew_ms,
    )
    report["base_timestamp_file"] = str(base_path)
    report["wrist_timestamp_file"] = str(wrist_path)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-jsonl", type=Path, required=True)
    parser.add_argument("--wrist-jsonl", type=Path, required=True)
    parser.add_argument("--max-skew-ms", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit_timestamp_files(args.base_jsonl, args.wrist_jsonl, max_skew_ms=args.max_skew_ms)
    rendered = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(rendered)
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
