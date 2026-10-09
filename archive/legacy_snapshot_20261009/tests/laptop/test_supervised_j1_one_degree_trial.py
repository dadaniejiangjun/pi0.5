from __future__ import annotations

import struct
import unittest

from laptop.robot_client.supervised_j1_one_degree_trial import execute, preflight
from laptop.robot_client.supervised_j1_step_trial import Observed


def ready_observation(j1_raw: int = 39) -> tuple[Observed, int]:
    observed = Observed()
    now = 1_000_000_000
    for stamp in (now - 80_000_000, now - 20_000_000):
        observed.add(0x2A1, bytes((1, 0, 1, 0, 0, 0, 0, 0)), stamp)
        observed.add(0x2A5, struct.pack(">ii", j1_raw, 0), stamp)
        for can_id in (0x2A6, 0x2A7):
            observed.add(can_id, struct.pack(">ii", 0, 0), stamp)
        for can_id in range(0x261, 0x267):
            observed.add(can_id, bytes((0, 0, 0, 0, 0, 0x40, 0, 0)), stamp)
    return observed, now


class SupervisedJ1OneDegreeTrialTest(unittest.TestCase):
    def test_prior_step_ready(self) -> None:
        observed, now = ready_observation()
        self.assertEqual(preflight(observed, now), (39, 0, 0, 0, 0, 0))

    def test_fresh_approval_required(self) -> None:
        with self.assertRaises(PermissionError):
            execute(operator_confirmed=False)

    def test_changed_start_or_other_axis_rejected(self) -> None:
        for start in (0, 1009):
            observed, now = ready_observation(start)
            with self.assertRaisesRegex(RuntimeError, "J1 is not stable"):
                preflight(observed, now)
        observed, now = ready_observation()
        observed.add(0x2A5, struct.pack(">ii", 39, 50), now)
        with self.assertRaisesRegex(RuntimeError, "J1 is not stable"):
            preflight(observed, now)

    def test_stale_or_competing_controller_rejected(self) -> None:
        observed, now = ready_observation()
        observed.add(0x155, bytes(8), now)
        with self.assertRaisesRegex(RuntimeError, "another controller"):
            preflight(observed, now)
        observed, now = ready_observation()
        with self.assertRaisesRegex(RuntimeError, "stale"):
            preflight(observed, now + 200_000_000)


if __name__ == "__main__":
    unittest.main()
