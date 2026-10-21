"""Pure PiPER SDK-unit conversions.

The nominal scales match the current official SDK interface documentation,
but firmware-specific confirmation is still a P2 hardware audit item. No
function in this module opens CAN or calls a robot SDK.
"""

from __future__ import annotations

import math

import numpy as np

from laptop.piper.control_profile import (
    SDK_NOMINAL_JOINT_BOUNDS_DEG,
    TASK_JOINT_BOUNDS_DEG,
    load_control_profile,
)


_CONTROL_PROFILE = load_control_profile()
_CONVERSIONS = _CONTROL_PROFILE["parameter_conversions"]
_SDK_JOINT_RAW_PER_DEGREE = int(_CONVERSIONS["sdk_joint_target"]["raw_per_degree"])
GRIPPER_NOMINAL_METERS_PER_RAW = float(_CONTROL_PROFILE["gripper"]["nominal_meters_per_raw"])
DEFAULT_SDK_JOINT_DEG_PER_UNIT = 1.0 / _SDK_JOINT_RAW_PER_DEGREE
DEFAULT_SDK_GRIPPER_MM_PER_UNIT = GRIPPER_NOMINAL_METERS_PER_RAW * 1000.0
# Keep the SDK reference visible separately from the effective task envelope.
# In particular, controller J6 readback is +/-170 degrees while the SDK
# reference and current task envelope remain +/-120 degrees.
PIPER_SDK_NOMINAL_JOINT_BOUNDS_DEG = SDK_NOMINAL_JOINT_BOUNDS_DEG
PIPER_TASK_JOINT_BOUNDS_DEG = TASK_JOINT_BOUNDS_DEG
# Compatibility name used by existing guards; intentionally resolves to the
# narrower configured task envelope, not the controller's full readback.
PIPER_NOMINAL_JOINT_BOUNDS_DEG = PIPER_TASK_JOINT_BOUNDS_DEG


def _scale_or_error(scale: float, name: str) -> float:
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return scale


def sdk_joint_to_rad(raw, degrees_per_sdk_unit: float = DEFAULT_SDK_JOINT_DEG_PER_UNIT):
    """Convert integer SDK joint units (nominally 0.001 degree) to radians."""

    scale = _scale_or_error(degrees_per_sdk_unit, "degrees_per_sdk_unit")
    return np.asarray(raw, dtype=np.float64) * scale * math.pi / 180.0


def rad_to_sdk_joint(rad, degrees_per_sdk_unit: float = DEFAULT_SDK_JOINT_DEG_PER_UNIT):
    """Quantize radians to integer SDK joint units."""

    scale = _scale_or_error(degrees_per_sdk_unit, "degrees_per_sdk_unit")
    result = np.rint(np.asarray(rad, dtype=np.float64) * 180.0 / math.pi / scale).astype(np.int64)
    return int(result) if result.ndim == 0 else result


def sdk_gripper_to_m(raw, mm_per_sdk_unit: float = DEFAULT_SDK_GRIPPER_MM_PER_UNIT):
    """Convert integer SDK gripper units (nominally 0.001 mm) to meters."""

    scale = _scale_or_error(mm_per_sdk_unit, "mm_per_sdk_unit")
    return np.asarray(raw, dtype=np.float64) * scale * 1e-3


def m_to_sdk_gripper(meters, mm_per_sdk_unit: float = DEFAULT_SDK_GRIPPER_MM_PER_UNIT):
    """Quantize meters to integer SDK gripper units."""

    scale = _scale_or_error(mm_per_sdk_unit, "mm_per_sdk_unit")
    result = np.rint(np.asarray(meters, dtype=np.float64) / (scale * 1e-3)).astype(np.int64)
    return int(result) if result.ndim == 0 else result


# Exact scalar variants preserve current runtime arithmetic and Python round.
# Existing configurable NumPy APIs above intentionally retain their own behavior.
def joint_raw_to_nominal_rad(raw):
    """Python scalar raw * pi / 180000; no added coercion or validation."""
    return raw * math.pi / (180 * _SDK_JOINT_RAW_PER_DEGREE)


def nominal_rad_to_joint_raw(rad):
    """Python round(rad * 180000 / pi), retaining exceptions and ties-to-even."""
    return round(rad * (180 * _SDK_JOINT_RAW_PER_DEGREE) / math.pi)


def gripper_raw_to_nominal_m(raw):
    """Multiply by 1e-6 as in the shadow/data path, not jaw calibration."""
    return raw * GRIPPER_NOMINAL_METERS_PER_RAW


def gripper_raw_to_nominal_m_divided(raw):
    """Divide by 1e6 as in bounded runtime; preserve its arithmetic order."""
    return raw / (1.0 / GRIPPER_NOMINAL_METERS_PER_RAW)


def nominal_m_to_gripper_raw(meters):
    """Python round(meters * 1e6) as in bounded runtime."""
    return round(meters * (1.0 / GRIPPER_NOMINAL_METERS_PER_RAW))
