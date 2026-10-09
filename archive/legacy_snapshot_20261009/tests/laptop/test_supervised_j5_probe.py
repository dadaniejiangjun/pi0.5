from __future__ import annotations

import unittest

from laptop.robot_client.supervised_j5_probe import EXPECTED_START_RAW, execute


class SupervisedJ5ProbeTest(unittest.TestCase):
    def test_explicit_approval_required(self) -> None:
        with self.assertRaises(PermissionError):
            execute(operator_confirmed=False)

    def test_expected_start_is_small_known_pose(self) -> None:
        self.assertEqual(len(EXPECTED_START_RAW), 6)
        self.assertEqual(EXPECTED_START_RAW[4], 0)
        self.assertLess(max(abs(q) for q in EXPECTED_START_RAW), 2000)


if __name__ == "__main__":
    unittest.main()
