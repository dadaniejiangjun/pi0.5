"""Pure fail-closed validation of a candidate PiPER 7D position action.

This module has no CAN or robot API. Passing a check is diagnostic only and
never grants permission to actuate hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

REQUIRED_ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_meter"


@dataclass(frozen=True)
class SafetyPolicy:
    joint_min_rad: tuple[float, ...]
    joint_max_rad: tuple[float, ...]
    gripper_min_m: float
    gripper_max_m: float
    max_joint_speed_rad_s: float
    max_joint_accel_rad_s2: float
    max_gripper_speed_m_s: float
    max_state_age_ns: int
    max_camera_age_ns: int
    max_response_age_ns: int
    max_camera_skew_ns: int
    max_watchdog_gap_ns: int
    max_action_horizon_ns: int = 500_000_000

    def __post_init__(self) -> None:
        if len(self.joint_min_rad) != 6 or len(self.joint_max_rad) != 6:
            raise ValueError("six joint bounds required")
        numbers = (*self.joint_min_rad, *self.joint_max_rad, self.gripper_min_m,
                   self.gripper_max_m, self.max_joint_speed_rad_s,
                   self.max_joint_accel_rad_s2, self.max_gripper_speed_m_s)
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
               for x in numbers):
            raise ValueError("policy limits must be finite numbers")
        if any(lo >= hi for lo, hi in zip(self.joint_min_rad, self.joint_max_rad)):
            raise ValueError("invalid joint bounds")
        if self.gripper_min_m >= self.gripper_max_m or min(
            self.max_joint_speed_rad_s, self.max_joint_accel_rad_s2,
            self.max_gripper_speed_m_s
        ) <= 0:
            raise ValueError("invalid gripper or rate bounds")
        clocks = (self.max_state_age_ns, self.max_camera_age_ns,
                  self.max_response_age_ns, self.max_camera_skew_ns,
                  self.max_watchdog_gap_ns, self.max_action_horizon_ns)
        if any(type(x) is not int or x <= 0 for x in clocks):
            raise ValueError("age limits must be positive integer nanoseconds")


@dataclass(frozen=True)
class Decision:
    accepted_for_offline_analysis: bool
    reasons: tuple[str, ...]
    hardware_control_allowed: bool = False


def _vector(value: object, length: int) -> tuple[float, ...] | None:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in value):
        return None
    return tuple(float(x) for x in value)


def _clock(value: object) -> int | None:
    return value if type(value) is int and value >= 0 else None


def validate_candidate(
    snapshot: Mapping[str, object],
    response: Mapping[str, object] | None,
    policy: SafetyPolicy,
    *,
    now_ns: int,
    last_response_seq: int,
) -> Decision:
    """Validate only; never return an SDK command or an actuation permit."""
    reasons: list[str] = []
    now = _clock(now_ns)
    if now is None or type(last_response_seq) is not int or last_response_seq < -1:
        return Decision(False, ("invalid_local_clock_or_sequence",))

    request_id = snapshot.get("request_id")
    request_seq = _clock(snapshot.get("request_seq"))
    issued = _clock(snapshot.get("request_issued_ns"))
    deadline = _clock(snapshot.get("request_deadline_ns"))
    state_ts = _clock(snapshot.get("state_timestamp_ns"))
    base_ts = _clock(snapshot.get("base_timestamp_ns"))
    wrist_ts = _clock(snapshot.get("wrist_timestamp_ns"))
    heartbeat_ts = _clock(snapshot.get("watchdog_timestamp_ns"))
    horizon = _clock(snapshot.get("action_horizon_ns"))
    if not isinstance(request_id, str) or not request_id:
        reasons.append("invalid_request_id")
    if request_seq is None:
        reasons.append("invalid_request_sequence")
    if issued is None or deadline is None or not issued < deadline or not issued <= now <= deadline:
        reasons.append("request_expired_or_unordered")
    for label, ts, limit in (
        ("state", state_ts, policy.max_state_age_ns),
        ("base", base_ts, policy.max_camera_age_ns),
        ("wrist", wrist_ts, policy.max_camera_age_ns),
        ("watchdog", heartbeat_ts, policy.max_watchdog_gap_ns),
    ):
        if ts is None or ts > now or now - ts > limit:
            reasons.append(f"{label}_stale_or_missing")
    if base_ts is None or wrist_ts is None or abs(base_ts - wrist_ts) > policy.max_camera_skew_ns:
        reasons.append("camera_skew")
    if horizon is None or horizon <= 0 or horizon > policy.max_action_horizon_ns:
        reasons.append("invalid_action_horizon")
    if snapshot.get("ctrl_mode") != "CAN_CTRL(0x1)":
        reasons.append("control_mode_unverified")
    if snapshot.get("arm_status") != "NORMAL(0x0)" or snapshot.get("err_code") != 0:
        reasons.append("robot_health_unverified")
    if snapshot.get("joint_unit") != "radian" or snapshot.get("gripper_unit") != "meter":
        reasons.append("unit_mismatch")
    if (snapshot.get("action_semantics") != REQUIRED_ACTION_SEMANTICS
            or response is not None and response.get("action_semantics") != REQUIRED_ACTION_SEMANTICS):
        reasons.append("action_semantics_unverified")
    if snapshot.get("units_hardware_verified") is not True:
        reasons.append("hardware_units_unverified")

    state = _vector(snapshot.get("state"), 7)
    prior_velocity = _vector(snapshot.get("prior_joint_velocity_rad_s"), 6)
    if state is None:
        reasons.append("invalid_state")
    if prior_velocity is None:
        reasons.append("invalid_prior_velocity")

    if response is None:
        reasons.append("response_missing")
        action = None
    else:
        response_seq = _clock(response.get("response_seq"))
        response_ts = _clock(response.get("received_timestamp_ns"))
        if response.get("request_id") != request_id or response_seq is None or (
            request_seq is not None and response_seq != request_seq
        ) or response_seq <= last_response_seq:
            reasons.append("response_out_of_order")
        if response_ts is None or issued is None or response_ts < issued or response_ts > now or (
            now - response_ts > policy.max_response_age_ns
        ):
            reasons.append("response_stale_or_missing")
        if response.get("joint_unit") != "radian" or response.get("gripper_unit") != "meter":
            reasons.append("response_unit_mismatch")
        action = _vector(response.get("action"), 7)
        if action is None:
            reasons.append("invalid_action")

    if state is not None:
        if any(not lo <= x <= hi for x, lo, hi in zip(
            state[:6], policy.joint_min_rad, policy.joint_max_rad
        )) or not policy.gripper_min_m <= state[6] <= policy.gripper_max_m:
            reasons.append("state_out_of_bounds")
    if action is not None:
        if any(not lo <= x <= hi for x, lo, hi in zip(
            action[:6], policy.joint_min_rad, policy.joint_max_rad
        )) or not policy.gripper_min_m <= action[6] <= policy.gripper_max_m:
            reasons.append("action_out_of_bounds")
    if state is not None and action is not None and prior_velocity is not None and horizon:
        seconds = horizon / 1_000_000_000
        velocity = tuple((target - current) / seconds for current, target in zip(state[:6], action[:6]))
        if any(abs(x) > policy.max_joint_speed_rad_s for x in velocity) or (
            abs(action[6] - state[6]) / seconds > policy.max_gripper_speed_m_s
        ):
            reasons.append("velocity_limit")
        if any(abs(v - previous) / seconds > policy.max_joint_accel_rad_s2
               for v, previous in zip(velocity, prior_velocity)):
            reasons.append("acceleration_limit")
    return Decision(not reasons, tuple(reasons))
