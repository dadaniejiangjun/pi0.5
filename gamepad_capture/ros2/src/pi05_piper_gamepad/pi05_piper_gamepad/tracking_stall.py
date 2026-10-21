"""Feedback-only tracking-stall heuristic; not a contact/torque detector."""

import math


class TrackingStallGuard:
    def __init__(self, *, window_s=1.0, error_deg=0.5,
                 progress_deg=0.1, retreat_deg=0.5):
        values = (window_s, error_deg, progress_deg, retreat_deg)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("stall settings must be finite and positive")
        self.window_s = window_s
        self.error = math.radians(error_deg)
        self.progress = math.radians(progress_deg)
        self.retreat = math.radians(retreat_deg)
        self.reset()

    def reset(self):
        self.windows = [None] * 6
        self.blocked = [None] * 6
        self.last_source = None

    def cancel_pending(self):
        # Neutral, invalid input or stale feedback cannot count as resistance.
        self.windows = [None] * 6

    def inspect(self, feedback, target, *, now_s, source):
        if (len(feedback) != 6 or len(target) != 6
                or not all(math.isfinite(v) for v in (*feedback, *target, now_s))):
            raise ValueError("stall check requires finite six-axis values and time")
        if source != self.last_source:
            self.last_source = source
            fresh = True
        else:
            fresh = False
        new, recovered, rejected = [], [], []
        for i, (q, goal) in enumerate(zip(feedback, target)):
            latch = self.blocked[i]
            if latch is not None:
                direction, anchor = latch
                if fresh and direction * (q-anchor) <= -self.retreat:
                    self.blocked[i] = None
                    recovered.append(i+1)
                elif direction * (goal-q) > 1e-7:
                    rejected.append(i+1)
            if self.blocked[i] is not None:
                self.windows[i] = None
                continue
            error = goal-q
            if abs(error) < self.error:
                self.windows[i] = None
                continue
            direction = 1 if error > 0 else -1
            window = self.windows[i]
            if window is None or window[2] != direction:
                self.windows[i] = (now_s, q, direction)
            elif fresh:
                started, anchor, _ = window
                # Any substantial actual motion restarts the observation window.
                if abs(q-anchor) >= self.progress:
                    self.windows[i] = (now_s, q, direction)
                elif now_s-started >= self.window_s:
                    self.blocked[i] = (direction, q)
                    self.windows[i] = None
                    new.append(i+1)
                    rejected.append(i+1)
        return dict(new_stalled_joints=new, recovered_joints=recovered,
                    blocked_joints=rejected, allowed=not rejected)
