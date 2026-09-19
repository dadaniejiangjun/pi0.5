"""Pure PiPER SDK-unit conversions.

The nominal scales match the current official SDK interface documentation,
but firmware-specific confirmation is still a P2 hardware audit item. No
function in this module opens CAN or calls a robot SDK.
"""

from __future__ import annotations

import math

import numpy as np


DEFAULT_SDK_JOINT_DEG_PER_UNIT = 0.001
DEFAULT_SDK_GRIPPER_MM_PER_UNIT = 0.001


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
