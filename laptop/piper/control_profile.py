"""Single source for verified PiPER task envelopes and parameter provenance.

The file records controller readbacks, SDK references, and operator UI values
separately. Only ``task_motion_envelope_deg`` and the explicitly tested
gripper command envelope are active software limits; they are not a claim that
every pose or jaw opening is mechanically safe.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import os
import math

import yaml


def _project_root() -> Path:
    configured = os.environ.get("PI05_PROJECT_ROOT")
    return Path(configured).resolve() if configured else Path(__file__).resolve().parents[2]


@lru_cache(maxsize=1)
def load_control_profile() -> dict:
    path = _project_root() / "configs" / "piper_joint_limits.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != "piper_joint_limits_v3":
        raise ValueError(f"unsupported PiPER control profile: {path}")
    return data


@dataclass(frozen=True)
class GripperControlLimits:
    minimum_raw: int
    maximum_raw: int
    maximum_step_raw: int
    effort_raw: int


def _gripper_limits(profile_name: str) -> GripperControlLimits:
    config = load_control_profile()["gripper"][profile_name]
    limits = GripperControlLimits(
        minimum_raw=config["minimum_raw"],
        maximum_raw=config["maximum_raw"],
        maximum_step_raw=config["maximum_step_raw"],
        effort_raw=config["effort_raw"],
    )
    if (any(type(value) is not int for value in vars(limits).values())
            or limits.minimum_raw < 0
            or limits.maximum_raw <= limits.minimum_raw
            or limits.maximum_step_raw <= 0
            or not 0 <= limits.effort_raw <= 5000):
        raise ValueError(f"invalid {profile_name} gripper limits")
    return limits


SINGLE_STEP_GRIPPER_LIMITS = _gripper_limits("single_step_commissioning")
CONTINUOUS_GRIPPER_LIMITS = _gripper_limits("continuous_task")
MANUAL_GAMEPAD_GRIPPER_LIMITS = _gripper_limits("manual_gamepad")
GRIPPER_NOMINAL_METERS_PER_RAW = float(
    load_control_profile()["gripper"]["nominal_meters_per_raw"]
)
if not math.isfinite(GRIPPER_NOMINAL_METERS_PER_RAW) or GRIPPER_NOMINAL_METERS_PER_RAW <= 0:
    raise ValueError("gripper nominal raw conversion must be positive and finite")


def _joint_bounds(section: str, *, source_unit: str) -> tuple[tuple[float, float], ...]:
    values = load_control_profile()[section]
    names = tuple(f"joint{i}" for i in range(1, 7))
    if source_unit not in {"rad", "deg"}:
        raise ValueError(f"unsupported joint bound unit: {source_unit}")
    factor = 180.0 / math.pi if source_unit == "rad" else 1.0
    result = tuple(
        (float(values[name]["min"]) * factor, float(values[name]["max"]) * factor)
        for name in names
    )
    if any(not math.isfinite(low) or not math.isfinite(high) or low >= high
           for low, high in result):
        raise ValueError(f"invalid {section} bounds")
    return result


def _controller_joint_bounds_deg() -> tuple[tuple[float, float], ...]:
    readback = load_control_profile()["hardware_query_evidence"]["controller_parameter_readback"]
    result = tuple(
        (float(readback[f"joint{index}"]["min_deg"]),
         float(readback[f"joint{index}"]["max_deg"]))
        for index in range(1, 7)
    )
    if any(not math.isfinite(low) or not math.isfinite(high) or low >= high
           for low, high in result):
        raise ValueError("invalid live controller joint limits")
    return result


SDK_NOMINAL_JOINT_BOUNDS_DEG = _joint_bounds("joints", source_unit="rad")
CONTROLLER_JOINT_LIMIT_READBACK_DEG = _controller_joint_bounds_deg()
_DECLARED_TASK_JOINT_BOUNDS_DEG = _joint_bounds(
    "task_motion_envelope_deg", source_unit="deg",
)
TASK_JOINT_BOUNDS_DEG = tuple(
    (max(sdk_low, controller_low), min(sdk_high, controller_high))
    for (sdk_low, sdk_high), (controller_low, controller_high)
    in zip(SDK_NOMINAL_JOINT_BOUNDS_DEG, CONTROLLER_JOINT_LIMIT_READBACK_DEG)
)
if any(low >= high for low, high in TASK_JOINT_BOUNDS_DEG):
    raise ValueError("SDK/controller joint-limit intersection is empty")
if any(
    not math.isclose(declared[0], effective[0], abs_tol=0.001)
    or not math.isclose(declared[1], effective[1], abs_tol=0.001)
    for declared, effective in zip(_DECLARED_TASK_JOINT_BOUNDS_DEG, TASK_JOINT_BOUNDS_DEG)
):
    raise ValueError("declared task joint limits differ from the SDK/controller intersection")
CONTINUOUS_GRIPPER_MIN_M = CONTINUOUS_GRIPPER_LIMITS.minimum_raw * GRIPPER_NOMINAL_METERS_PER_RAW
CONTINUOUS_GRIPPER_MAX_M = CONTINUOUS_GRIPPER_LIMITS.maximum_raw * GRIPPER_NOMINAL_METERS_PER_RAW
CONTINUOUS_GRIPPER_MAX_STEP_M = CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw * GRIPPER_NOMINAL_METERS_PER_RAW


def validate_continuous_gripper_target(source_raw: int, target_raw: int, effort_raw: int) -> None:
    """Apply the one shared continuous-task range, step, and effort policy."""
    limits = CONTINUOUS_GRIPPER_LIMITS
    if type(source_raw) is not int or type(target_raw) is not int:
        raise ValueError("continuous gripper positions must be integer raw values")
    if not limits.minimum_raw <= source_raw <= limits.maximum_raw:
        raise ValueError("continuous gripper source is outside the tested command envelope")
    if not limits.minimum_raw <= target_raw <= limits.maximum_raw:
        raise ValueError("continuous gripper target is outside the tested command envelope")
    if abs(target_raw - source_raw) > limits.maximum_step_raw:
        raise ValueError("continuous gripper step exceeds the configured raw step limit")
    if type(effort_raw) is not int or effort_raw != limits.effort_raw:
        raise ValueError("continuous gripper effort differs from the configured task value")


def controller_joint_limit_raw_to_deg(raw: int) -> float:
    """Decode controller angle-limit values (0.1 degree per raw unit)."""
    if type(raw) is not int:
        raise TypeError("controller joint-limit raw value must be an integer")
    scale = float(load_control_profile()["parameter_conversions"]["controller_joint_limit"]["degree_per_raw"])
    return raw * scale


def sdk_joint_target_raw_to_deg(raw: int) -> float:
    """Decode SDK joint-position targets (0.001 degree per raw unit)."""
    if type(raw) is not int:
        raise TypeError("SDK joint target raw value must be an integer")
    raw_per_degree = float(load_control_profile()["parameter_conversions"]["sdk_joint_target"]["raw_per_degree"])
    return raw / raw_per_degree


def controller_speed_query_raw_to_rad_s(raw: int) -> float:
    """Decode 0x473 controller speed readback using AgileX's 0.01 scale clarification."""
    if type(raw) is not int:
        raise TypeError("controller speed raw value must be an integer")
    scale = float(load_control_profile()["parameter_conversions"]["controller_speed_query"]["rad_s_per_raw"])
    return raw * scale


def sdk_speed_setter_raw_to_rad_s(raw: int) -> float:
    """Decode the separate SDK setter scale; runtime does not write this parameter."""
    if type(raw) is not int:
        raise TypeError("SDK speed setter raw value must be an integer")
    scale = float(load_control_profile()["parameter_conversions"]["sdk_speed_setter"]["rad_s_per_raw"])
    return raw * scale


def controller_acceleration_query_raw_to_rad_s2(raw: int) -> float:
    """Decode the SDK query-response scale, kept separate from its setter scale."""
    if type(raw) is not int:
        raise TypeError("controller acceleration raw value must be an integer")
    scale = float(load_control_profile()["parameter_conversions"]["controller_acceleration_query"]["rad_s2_per_raw"])
    return raw * scale


def controller_acceleration_setter_raw_to_rad_s2(raw: int) -> float:
    """Decode the separately documented SDK setter scale; do not use for readback."""
    if type(raw) is not int:
        raise TypeError("controller acceleration setter raw value must be an integer")
    scale = float(load_control_profile()["parameter_conversions"]["controller_acceleration_setter"]["rad_s2_per_raw"])
    return raw * scale
