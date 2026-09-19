from __future__ import annotations

import unittest

import numpy as np

from laptop.piper.unit_conversion import (
    DEFAULT_SDK_GRIPPER_MM_PER_UNIT,
    DEFAULT_SDK_JOINT_DEG_PER_UNIT,
    m_to_sdk_gripper,
    rad_to_sdk_joint,
    sdk_gripper_to_m,
    sdk_joint_to_rad,
)


class PiperUnitRoundTripTest(unittest.TestCase):
    def test_10000_joint_values_quantize_within_half_unit(self) -> None:
        rng = np.random.default_rng(170)
        raw = rng.integers(-180_000, 180_001, size=10_000, dtype=np.int64)
        rad = sdk_joint_to_rad(raw)
        recovered = sdk_joint_to_rad(rad_to_sdk_joint(rad))
        half_unit = DEFAULT_SDK_JOINT_DEG_PER_UNIT * np.pi / 180.0 / 2.0
        np.testing.assert_array_less(np.abs(recovered - rad), half_unit + 1e-12)

    def test_10000_gripper_values_quantize_within_half_unit(self) -> None:
        rng = np.random.default_rng(171)
        raw = rng.integers(0, 100_001, size=10_000, dtype=np.int64)
        meters = sdk_gripper_to_m(raw)
        recovered = sdk_gripper_to_m(m_to_sdk_gripper(meters))
        half_unit = DEFAULT_SDK_GRIPPER_MM_PER_UNIT * 1e-3 / 2.0
        np.testing.assert_array_less(np.abs(recovered - meters), half_unit + 1e-15)

    def test_scale_is_explicitly_overridable(self) -> None:
        self.assertEqual(rad_to_sdk_joint(sdk_joint_to_rad(100, 0.01), 0.01), 100)
        self.assertEqual(m_to_sdk_gripper(sdk_gripper_to_m(100, 0.01), 0.01), 100)


if __name__ == "__main__":
    unittest.main()
