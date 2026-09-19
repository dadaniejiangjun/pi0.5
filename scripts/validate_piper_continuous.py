#!/usr/bin/env python3
"""Overwrite the unit report with continuous random-value quantization checks."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from laptop.piper.unit_conversion import (
    DEFAULT_SDK_GRIPPER_MM_PER_UNIT,
    DEFAULT_SDK_JOINT_DEG_PER_UNIT,
    m_to_sdk_gripper,
    rad_to_sdk_joint,
    sdk_gripper_to_m,
    sdk_joint_to_rad,
)


def main() -> int:
    rng = np.random.default_rng(174)
    joint_values = rng.uniform(-np.pi, np.pi, size=10_000)
    joint_error = np.abs(sdk_joint_to_rad(rad_to_sdk_joint(joint_values)) - joint_values)
    gripper_values = rng.uniform(0.0, 0.1, size=10_000)
    gripper_error = np.abs(sdk_gripper_to_m(m_to_sdk_gripper(gripper_values)) - gripper_values)
    joint_bound = DEFAULT_SDK_JOINT_DEG_PER_UNIT * np.pi / 180.0 / 2.0
    gripper_bound = DEFAULT_SDK_GRIPPER_MM_PER_UNIT * 1e-3 / 2.0
    report = {
        "status": "PASS" if float(joint_error.max()) <= joint_bound and float(gripper_error.max()) <= gripper_bound else "FAIL",
        "samples": 10_000,
        "joint_max_abs_error_rad": float(joint_error.max()),
        "joint_half_quantization_bound_rad": joint_bound,
        "gripper_max_abs_error_m": float(gripper_error.max()),
        "gripper_half_quantization_bound_m": gripper_bound,
        "round_trip_direction": "canonical -> SDK integer -> canonical",
        "hardware_accessed": False,
        "scale_status": "NOMINAL_OFFICIAL_SCALE_PENDING_FIRMWARE_AUDIT",
    }
    path = Path("validation/piper_unit_roundtrip.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
