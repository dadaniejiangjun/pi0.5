from __future__ import annotations

import unittest

from laptop.robot_client.supervised_restore_gripper import (
    TARGET_GRIPPER_RAW, execute,
)


class SupervisedRestoreGripperTest(unittest.TestCase):
    def test_requires_specific_approval(self) -> None:
        with self.assertRaises(PermissionError):
            execute(operator_confirmed=False)

    def test_target_is_recorded_start_opening(self) -> None:
        self.assertEqual(TARGET_GRIPPER_RAW, 7100)


if __name__ == "__main__":
    unittest.main()
