from __future__ import annotations

import struct
import unittest

from laptop.robot_client.supervised_j1_step_trial import Observed, execute, preflight


def ready_observation() -> tuple[Observed, int]:
    observed = Observed()
    clock = 1_000_000_000
    for t in (clock - 80_000_000, clock - 20_000_000):
        observed.add(0x2A1, bytes((1, 0, 1, 0, 0, 0, 0, 0)), t)
        for can_id in (0x2A5, 0x2A6, 0x2A7):
            observed.add(can_id, struct.pack(">ii", 0, 0), t)
        for can_id in range(0x261, 0x267):
            observed.add(can_id, bytes((0, 0, 0, 0, 0, 0x40, 0, 0)), t)
    return observed, clock


class SupervisedJ1StepTrialTest(unittest.TestCase):
    def test_zero_ready(self) -> None:
        observed, now = ready_observation()
        self.assertEqual(preflight(observed, now), (0,) * 6)

    def test_live_entry_requires_immediate_confirmation(self) -> None:
        with self.assertRaises(PermissionError):
            execute(operator_confirmed=False)

    def test_competing_controller_rejected(self) -> None:
        observed, now = ready_observation()
        observed.add(0x155, bytes(8), now)
        with self.assertRaisesRegex(RuntimeError, "another controller"):
            preflight(observed, now)

    def test_disabled_or_stale_rejected(self) -> None:
        observed, now = ready_observation()
        observed.add(0x261, bytes(8), now)
        with self.assertRaisesRegex(RuntimeError, "not all six"):
            preflight(observed, now)
        observed, now = ready_observation()
        with self.assertRaisesRegex(RuntimeError, "stale"):
            preflight(observed, now + 200_000_000)

    def test_nonzero_state_rejected(self) -> None:
        observed, now = ready_observation()
        observed.add(0x2A5, struct.pack(">ii", 50, 0), now)
        with self.assertRaisesRegex(RuntimeError, "zero pose"):
            preflight(observed, now)


if __name__ == "__main__":
    unittest.main()
