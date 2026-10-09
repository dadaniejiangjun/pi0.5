"""Offline-only trajectory preflight. Never imports a CAN or robot sender.

The output is a timing *candidate*, not a PiPER command sequence. Recorded
feedback has not been established as a valid actuator target; the gripper is
deliberately excluded because its command mapping is not verified.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

from laptop.piper.unit_conversion import PIPER_NOMINAL_JOINT_BOUNDS_DEG


# Backwards-compatible name for existing offline tooling.
NOMINAL_JOINT_BOUNDS_DEG = PIPER_NOMINAL_JOINT_BOUNDS_DEG


def _frame(row: Mapping[str, object], index: int) -> tuple[int, tuple[float, ...], int]:
    if row.get("frame_index") != index or type(row.get("policy_timestamp_ns")) is not int:
        raise ValueError("invalid frame index or timestamp")
    joints = row.get("joint_rad_nominal_only")
    gripper = row.get("gripper_raw")
    if (not isinstance(joints, list) or len(joints) != 6
            or any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in joints)
            or type(gripper) is not int or gripper < 0):
        raise ValueError("invalid joint or gripper feedback")
    degrees = tuple(math.degrees(float(value)) for value in joints)
    return row["policy_timestamp_ns"], degrees, gripper


def analyze(
    rows: Sequence[Mapping[str, object]], *,
    planning_speed_deg_s: float = 10.0,
    planning_accel_deg_s2: float = 30.0,
) -> dict:
    """Report a joint-only, zero-end-velocity smoothstep timing candidate.

    The chosen planning limits are test parameters, not certified PiPER limits.
    Smoothstep is C1 at knots but has no jerk bound or collision check.
    """
    if len(rows) < 2 or any(not math.isfinite(x) or x <= 0 for x in
                            (planning_speed_deg_s, planning_accel_deg_s2)):
        raise ValueError("at least two frames and positive finite planning limits required")
    frames = [_frame(row, index) for index, row in enumerate(rows)]
    if any(not all(lo <= q <= hi for q, (lo, hi) in zip(joints, NOMINAL_JOINT_BOUNDS_DEG))
           for _, joints, _ in frames):
        raise ValueError("recorded joint exceeds official nominal position bounds")
    if any(b[0] <= a[0] for a, b in zip(frames, frames[1:])):
        raise ValueError("non-increasing frame timestamp")

    original_duration_s = (frames[-1][0] - frames[0][0]) / 1e9
    candidate_duration_s = 0.0
    maximum_observed_speed = 0.0
    maximum_step = 0.0
    maximum_gripper_step_raw = 0
    retimed_segments = 0
    worst_step = None
    for index, (before, after) in enumerate(zip(frames, frames[1:])):
        dt = (after[0] - before[0]) / 1e9
        joint_step = max(abs(right - left) for left, right in zip(before[1], after[1]))
        maximum_gripper_step_raw = max(maximum_gripper_step_raw, abs(after[2] - before[2]))
        maximum_observed_speed = max(maximum_observed_speed, joint_step / dt)
        if joint_step > maximum_step:
            maximum_step, worst_step = joint_step, [index, index + 1]
        # Cubic smoothstep peak |velocity| = 1.5*delta/T; peak |accel| = 6*delta/T^2.
        minimum_smoothstep_s = max(
            1.5 * joint_step / planning_speed_deg_s,
            math.sqrt(6.0 * joint_step / planning_accel_deg_s2),
        )
        candidate_segment_s = max(dt, minimum_smoothstep_s)
        candidate_duration_s += candidate_segment_s
        retimed_segments += candidate_segment_s > dt + 1e-12
    return {
        "status": "OFFLINE_JOINT_TIMING_CANDIDATE_ONLY",
        "frames": len(frames),
        "original_duration_s": round(original_duration_s, 3),
        "joint_only_candidate_duration_s": round(candidate_duration_s, 3),
        "planning_speed_deg_s": planning_speed_deg_s,
        "planning_accel_deg_s2": planning_accel_deg_s2,
        "planning_limits_hardware_verified": False,
        "nominal_position_bounds_passed": True,
        "nominal_position_bounds_hardware_verified": False,
        "max_observed_joint_speed_deg_s": round(maximum_observed_speed, 3),
        "max_adjacent_joint_step_deg": round(maximum_step, 3),
        "worst_step_frames": worst_step,
        "retimed_segments": retimed_segments,
        "max_adjacent_gripper_step_raw": maximum_gripper_step_raw,
        "gripper_command_mapping_verified": False,
        "complete_7d_replay_available": False,
        "current_to_start_transition_checked": False,
        "collision_checked": False,
        "command_semantics_verified": False,
        "can_data_tx_calls": 0,
        "hardware_control_allowed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames_jsonl", type=Path)
    args = parser.parse_args()
    with args.frames_jsonl.open(encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    print(json.dumps(analyze(rows), indent=2))


if __name__ == "__main__":
    main()
