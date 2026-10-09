"""Replay saved live-shadow predictions through the pure offline veto.

Nominal joint limits are diagnostic only. Unknown gripper limits and proxy
action semantics make every row ineligible for hardware control.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

from laptop.safety.offline_guard import SafetyPolicy, validate_candidate


def _nominal_joint_bounds(path: Path) -> tuple[tuple[float, ...], tuple[float, ...]]:
    text = path.read_text(encoding="utf-8")
    allowed_statuses = (
        "hardware_status: NOT_REAL_CONFIRMED_LIMITS",
        "hardware_status: PARTIAL_REAL_CONFIRMED_LIMITS",
    )
    if not any(status in text for status in allowed_statuses):
        raise ValueError("expected explicitly incomplete hardware joint limits")
    parsed = {}
    for joint, low, high in re.findall(
            r"joint([1-6]):\s*\{min:\s*([-\d.]+),\s*max:\s*([-\d.]+)\}", text):
        parsed[int(joint)] = (float(low), float(high))
    if set(parsed) != set(range(1, 7)) or "status: PENDING_GETSDKGRIPPERRANGEPARAM" not in text:
        raise ValueError("nominal limits file is incomplete or gripper status changed")
    return tuple(parsed[i][0] for i in range(1, 7)), tuple(parsed[i][1] for i in range(1, 7))


def audit(report_path: Path, nominal_limits: Path) -> dict:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if (report.get("status") != "LIVE_SHADOW_CONTINUOUS_PASS"
            or report.get("robot_command_sent") is not False
            or report.get("can_data_tx_calls") != 0
            or report.get("control_enabled") is not False
            or report.get("units_hardware_verified") is not False):
        raise ValueError("not an offline-only continuous shadow report")
    lower, upper = _nominal_joint_bounds(nominal_limits)
    policy = SafetyPolicy(
        joint_min_rad=lower, joint_max_rad=upper,
        gripper_min_m=0.0, gripper_max_m=1.0,  # Diagnostic placeholder; never hardware limits.
        max_joint_speed_rad_s=0.2, max_joint_accel_rad_s2=1.0,
        max_gripper_speed_m_s=0.02,
        max_state_age_ns=250_000_000, max_camera_age_ns=250_000_000,
        max_response_age_ns=250_000_000, max_camera_skew_ns=100_000_000,
        max_watchdog_gap_ns=250_000_000,
    )
    decisions = []
    for index, item in enumerate(report["predictions"]):
        issued, received = item["request_ns"], item["response_ns"]
        snapshot = {
            "request_id": f"shadow-{index}", "request_seq": index,
            "request_issued_ns": issued, "request_deadline_ns": issued + 500_000_000,
            "state_timestamp_ns": min(item["can_timestamps_ns"].values()),
            "base_timestamp_ns": item["base_timestamp_ns"],
            "wrist_timestamp_ns": item["wrist_timestamp_ns"],
            "watchdog_timestamp_ns": issued,
            "action_horizon_ns": 500_000_000,
            "ctrl_mode": item.get("robot_status", {}).get("ctrl_mode"),
            "arm_status": item.get("robot_status", {}).get("arm_status"),
            "err_code": item.get("robot_status", {}).get("err_code"),
            "joint_unit": "radian_nominal_unverified",
            "gripper_unit": "meter_nominal_unverified",
            "state": item["state"], "prior_joint_velocity_rad_s": [0.0] * 6,
        }
        response = {
            "request_id": f"shadow-{index}", "response_seq": index,
            "received_timestamp_ns": received,
            "joint_unit": "radian_nominal_unverified",
            "gripper_unit": "meter_nominal_unverified",
            "action": item["first_predicted_proxy"],
        }
        verdict = validate_candidate(snapshot, response, policy, now_ns=received,
                                     last_response_seq=index - 1)
        decisions.append({"index": index, "accepted_for_offline_analysis": verdict.accepted_for_offline_analysis,
                          "hardware_control_allowed": verdict.hardware_control_allowed,
                          "reasons": list(verdict.reasons)})
    reasons = Counter(reason for verdict in decisions for reason in verdict["reasons"])
    return {
        "status": "OFFLINE_SHADOW_VETO_AUDIT",
        "report_sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
        "nominal_limits_sha256": hashlib.sha256(nominal_limits.read_bytes()).hexdigest(),
        "samples": len(decisions), "rejected": sum(not row["accepted_for_offline_analysis"] for row in decisions),
        "reason_counts": dict(reasons), "decisions": decisions,
        "policy_limits_hardware_verified": False,
        "gripper_limits_hardware_verified": False,
        "proxy_action_is_verified_command": False,
        "hardware_control_allowed": False,
        "robot_command_sent": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--nominal-limits", type=Path, default=Path("configs/piper_joint_limits.yaml"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.report, args.nominal_limits)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
