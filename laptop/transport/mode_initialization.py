"""Fixed official reset/standby/CAN sequence; reset can cause loss of torque."""

from dataclasses import dataclass
import struct
import time


STAGES = ("reset", "standby", "can_joint")
FEEDBACK_IDS = (0x2A1, 0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267))
MAX_AGE_NS = 120_000_000
ZERO_TOLERANCE_RAW = 1000  # Conservative software precondition, not a calibration.


@dataclass(frozen=True)
class ModeInitializationFrame:
    stage: str

    def __post_init__(self):
        if self.stage not in STAGES:
            raise ValueError("only reset, standby, and CAN joint initialization are permitted")

    @property
    def arbitration_id(self):
        return 0x150 if self.stage == "reset" else 0x151

    @property
    def data(self):
        if self.stage == "reset":
            return bytes((2, 0, 0, 0, 0, 0, 0, 0))
        # ctrl_mode, MOVE J, 5% speed, position-speed mode, four zero fields.
        return bytes((int(self.stage == "can_joint"), 1, 5, 0, 0, 0, 0, 0))


@dataclass(frozen=True)
class J6EnableFrame:
    """Exactly official EnableArm(motor_num=6, enable_flag=2)."""

    @property
    def arbitration_id(self):
        return 0x471

    @property
    def data(self):
        return bytes((6, 2, 0, 0, 0, 0, 0, 0))


def feedback_snapshot(feedback, *, after_ns=0, require_zero=False, history=()):
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
        if any(abs(value) > ZERO_TOLERANCE_RAW for value in joints):
            raise PermissionError("all six joints must be within nominal 1 degree of zero before reset/mode setting")
        samples = [(stamp, values) for stamp, values in history
                   if max(after_ns, now - 400_000_000) < stamp <= now]
        if len(samples) < 2 or samples[-1][0] - samples[0][0] < 300_000_000:
            raise PermissionError("300 ms of stable post-write joint feedback is required")
        if any(max(values[axis] for _, values in samples)
               - min(values[axis] for _, values in samples) > 50 for axis in range(6)):
            raise PermissionError("joint drift exceeds nominal 0.050 degree")
    return {"ctrl_mode_raw": status[0], "teach_status_raw": status[3],
            "joint_raw_0p001deg": list(joints),
            "motor_enabled": [bool(feedback[can_id][1][5] & 0x40)
                              for can_id in range(0x261, 0x267)],
            "feedback_monotonic_ns": {hex(can_id): feedback[can_id][0]
                                      for can_id in FEEDBACK_IDS}}
