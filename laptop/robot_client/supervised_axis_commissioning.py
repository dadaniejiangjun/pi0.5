"""One separately approved 1-degree commissioning step on J2-J6."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from laptop.robot_client.supervised_j1_commissioning import execute as joint_execute


def execute(*, axis, physical_estop_ready, low_speed_confirmed,
            motion_envelope_cleared, confirm, transport=None):
    if type(axis) is not int or axis not in range(2, 7):
        raise ValueError("axis must be J2 through J6")
    result = joint_execute(commissioning_j1_plus_1deg=True, selected_axis=axis,
        physical_estop_ready=physical_estop_ready, low_speed_confirmed=low_speed_confirmed,
        motion_envelope_cleared=motion_envelope_cleared, confirm=confirm, transport=transport)
    initial = result.get("current_joint_raw_0p001deg")
    final = result.get("last_joint_raw_0p001deg")
    if initial is not None and final is not None:
        delta = [b - a for a, b in zip(initial, final)]
        result["observed_delta_raw_0p001deg"] = delta
        result["nominal_response_ratio"] = delta[axis - 1] / (-1000 if axis == 3 else 1000)
        result["response_direction_and_magnitude_observed"] = bool(
            result["feedback_fresh"] and result["arm_healthy_in_can_ctrl"]
            and result["all_six_motors_enabled"]
            and not result["observed_overshoot_or_other_joint_motion"]
            and 0.5 <= result["nominal_response_ratio"] <= 1.05)
    result["physical_angle_scale_verified"] = False
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", type=int, choices=range(2, 7), required=True)
    parser.add_argument("--output", type=Path, required=True)
    for flag in ("physical-estop-ready", "low-speed-confirmed", "motion-envelope-cleared"):
        parser.add_argument("--" + flag, action="store_true")
    args = parser.parse_args()
    with args.output.open("x", encoding="utf-8") as report:
        started = datetime.now(timezone.utc).isoformat()
        report.write(json.dumps(dict(status="STARTED", started_utc=started)) + "\n")
        report.flush()
        try:
            result = execute(axis=args.axis, physical_estop_ready=args.physical_estop_ready,
                low_speed_confirmed=args.low_speed_confirmed,
                motion_envelope_cleared=args.motion_envelope_cleared,
                confirm=lambda preview: input("Confirm the printed single-axis scope: "))
        except Exception as exc:
            result = dict(status="STOPPED_NO_RETRY", reason=str(exc), error_type=type(exc).__name__)
        report.seek(0)
        report.truncate()
        report.write(json.dumps(dict(started_utc=started, **result), indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result.get("response_direction_and_magnitude_observed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
