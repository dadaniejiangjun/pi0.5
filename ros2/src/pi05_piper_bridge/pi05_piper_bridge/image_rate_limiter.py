"""Small monotonic-clock gate for ROS image publication cadence."""
from __future__ import annotations


class ImageRateGate:
    """Publish at the requested average cadence without accumulating arrival jitter.

    The caller should feed every received frame into its validation path and
    call :meth:`accept` only after the frame is fresh and otherwise valid.
    Skipped frame sequence numbers are intentionally preserved as gaps in the
    next published metadata record.
    """

    def __init__(self, publish_hz: float) -> None:
        rate = float(publish_hz)
        if not 0.0 < rate <= 120.0:
            raise ValueError("publish_hz must be within (0, 120]")
        self.period_ns = int(1_000_000_000 / rate)
        # A nominal 30 Hz period is 33,333,333.33 ns. Allow a small early
        # tolerance so integer-nanosecond rounding does not drop valid frames.
        self.early_tolerance_ns = 1_000_000
        self.next_due_ns: int | None = None
        self.last_published_ns: int | None = None

    def accept(self, now_monotonic_ns: int) -> bool:
        now = int(now_monotonic_ns)
        if now <= 0:
            raise ValueError("now_monotonic_ns must be positive")
        if self.next_due_ns is None:
            self.last_published_ns = now
            self.next_due_ns = now + self.period_ns
            return True
        if now <= (self.last_published_ns or 0):
            return False
        if now + self.early_tolerance_ns < self.next_due_ns:
            return False
        self.last_published_ns = now
        self.next_due_ns += self.period_ns
        if self.next_due_ns <= now:
            missed = (now - self.next_due_ns) // self.period_ns + 1
            self.next_due_ns += missed * self.period_ns
        return True
