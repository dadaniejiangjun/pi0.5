"""Fixed official reset/standby/CAN sequence; reset can cause loss of torque."""

from dataclasses import dataclass
import struct
import time

from laptop.control.feedback_acceptance import (
    ZERO_MODE_HELD_DRIFT_RAW,
    ZERO_MODE_STABLE_DURATION_NS,
    ZERO_MODE_TARGET_TOLERANCE_RAW,
)


STAGES = ("reset", "standby", "can_joint")
FEEDBACK_IDS = (0x2A1, 0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267))
MAX_AGE_NS = 120_000_000
ZERO_TOLERANCE_RAW = ZERO_MODE_TARGET_TOLERANCE_RAW  # Conservative precondition, not calibration.


@dataclass(frozen=True)
class ModeInitializationFrame:
    stage: str
    speed_percent: int = 5

    def __post_init__(self):
        if self.stage not in STAGES:
            raise ValueError("only reset, standby, and CAN joint initialization are permitted")
        if type(self.speed_percent) is not int or not 1 <= self.speed_percent <= 100:
            raise ValueError("controller speed must be an integer within 1..100 percent")
        if self.stage != 'can_joint' and self.speed_percent != 5:
            raise ValueError("speed override is only supported for CAN joint control")

    @property
    def arbitration_id(self):
        return 0x150 if self.stage == "reset" else 0x151

    @property
    def data(self):
        if self.stage == "reset":
            return bytes((2, 0, 0, 0, 0, 0, 0, 0))
        # ctrl_mode, MOVE J, selected speed (default 5%), position-speed mode.
        return bytes((int(self.stage == "can_joint"), 1, self.speed_percent, 0, 0, 0, 0, 0))


@dataclass(frozen=True)
class J6EnableFrame:
    """Exactly official EnableArm(motor_num=6, enable_flag=2)."""

    @property
    def arbitration_id(self):
        return 0x471

    @property
    def data(self):
        return bytes((6, 2, 0, 0, 0, 0, 0, 0))


def feedback_snapshot(
    feedback, *, after_ns=0, require_zero=False, require_stable=False, history=(),
    zero_tolerance_raw=ZERO_MODE_TARGET_TOLERANCE_RAW,
    stable_duration_ns=ZERO_MODE_STABLE_DURATION_NS,
    stability_drift_raw=ZERO_MODE_HELD_DRIFT_RAW,
    stable_max_gap_ns=MAX_AGE_NS,
):
    now = time.perf_counter_ns()
    if any(can_id not in feedback for can_id in FEEDBACK_IDS):
        raise PermissionError("all status, joint, and six motor feedback channels are required")
    if any(not after_ns < feedback[can_id][0] <= now
           or now - feedback[can_id][0] > MAX_AGE_NS for can_id in FEEDBACK_IDS):
        raise PermissionError("initialization feedback is stale or predates the last write")
    status = feedback[0x2A1][1]
    if status[1] != 0 or status[6:8] != bytes(2):
        raise PermissionError("robot status is not normal or reports an error")
    joints = tuple(value for can_id in (0x2A5, 0x2A6, 0x2A7)
                   for value in struct.unpack(">ii", feedback[can_id][1]))
    joint_stamps = [feedback[can_id][0] for can_id in (0x2A5, 0x2A6, 0x2A7)]
    if max(joint_stamps) - min(joint_stamps) > 20_000_000:
        raise PermissionError("joint feedback channels are not synchronized")
    if require_zero:
        if (type(zero_tolerance_raw) is not int or zero_tolerance_raw < 0
                or type(stable_duration_ns) is not int or stable_duration_ns <= 0
                or type(stability_drift_raw) is not int or stability_drift_raw < 0
                or type(stable_max_gap_ns) is not int or stable_max_gap_ns <= 0):
            raise ValueError("zero target and stability tolerances must be explicit non-negative values")
        if any(abs(value) > zero_tolerance_raw for value in joints):
            raise PermissionError(
                f"all six joints must be within {zero_tolerance_raw / 1000:g} nominal degrees of zero before reset/mode setting"
            )
    if require_zero or require_stable:
        samples = [(stamp, values) for stamp, values in history
                   if max(after_ns, now - stable_duration_ns - MAX_AGE_NS) < stamp <= now]
        if (len(samples) < 2
                or samples[-1][0] - samples[0][0] < stable_duration_ns
                or any(right[0]-left[0] > stable_max_gap_ns
                       for left, right in zip(samples, samples[1:]))):
            raise PermissionError(
                f"{stable_duration_ns / 1e6:g} ms of stable post-write joint feedback is required"
            )
        if any(max(values[axis] for _, values in samples)
               - min(values[axis] for _, values in samples) > stability_drift_raw for axis in range(6)):
            raise PermissionError(
                f"joint drift exceeds {stability_drift_raw / 1000:g} nominal degrees"
            )
    return {"ctrl_mode_raw": status[0], "teach_status_raw": status[3],
            "joint_raw_0p001deg": list(joints),
            "motor_enabled": [bool(feedback[can_id][1][5] & 0x40)
                              for can_id in range(0x261, 0x267)],
            "feedback_monotonic_ns": {hex(can_id): feedback[can_id][0]
                                      for can_id in FEEDBACK_IDS}}
