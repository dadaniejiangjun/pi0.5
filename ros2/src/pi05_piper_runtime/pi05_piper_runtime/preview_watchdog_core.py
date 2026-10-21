"""Prediction-preview liveness checks for the read-only RunTask action.

This is a software liveness watchdog only. It does not stop or hold a robot.
"""
from __future__ import annotations


def preview_watchdog_fault(
    *,
    started_monotonic_ns: int,
    last_preview_monotonic_ns: int | None,
    now_monotonic_ns: int,
    first_preview_timeout_ns: int,
    preview_gap_timeout_ns: int,
) -> str | None:
    """Return a fail-closed reason when prediction previews stop arriving."""
    values = (
        (started_monotonic_ns, "started_monotonic_ns"),
        (now_monotonic_ns, "now_monotonic_ns"),
        (first_preview_timeout_ns, "first_preview_timeout_ns"),
        (preview_gap_timeout_ns, "preview_gap_timeout_ns"),
    )
    for value, name in values:
        if type(value) is not int or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
    if now_monotonic_ns < started_monotonic_ns:
        return "monotonic_clock_regressed"
    if last_preview_monotonic_ns is None:
        if now_monotonic_ns - started_monotonic_ns >= first_preview_timeout_ns:
            return "first_preview_timeout"
        return None
    if (type(last_preview_monotonic_ns) is not int
            or last_preview_monotonic_ns < started_monotonic_ns
            or last_preview_monotonic_ns > now_monotonic_ns):
        return "preview_timestamp_invalid"
    if now_monotonic_ns - last_preview_monotonic_ns >= preview_gap_timeout_ns:
        return "preview_gap_timeout"
    return None
