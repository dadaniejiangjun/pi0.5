from __future__ import annotations

import struct
import unittest

from laptop.robot_client.supervised_j1_step_trial import Observed
from laptop.robot_client.supervised_restore_start import (
    EXPECTED_CURRENT_RAW, MAX_SEGMENT_RAW, checked_feedback, execute,
    plan_waypoints, recorded_start_raw, within_segment,
)


def ready_observation() -> tuple[Observed, int]:
    now = 1_000_000_000
    observed = Observed()
    for stamp in (now - 80_000_000, now - 20_000_000):
        observed.add(0x2A1, bytes((1, 0, 1, 0, 0, 0, 0, 0)), stamp)
        for pair in range(3):
            observed.add(0x2A5 + pair,
                         struct.pack(">ii", *EXPECTED_CURRENT_RAW[2 * pair:2 * pair + 2]),
                         stamp)
        for can_id in range(0x261, 0x267):
            observed.add(can_id, bytes((0, 0, 0, 0, 0, 0x40, 0, 0)), stamp)
    return observed, now


class SupervisedRestoreStartTest(unittest.TestCase):
    def test_recorded_start_and_bounded_plan(self) -> None:
        target = recorded_start_raw()
        self.assertEqual(len(target), 6)
        self.assertLess(target[5], -19_000)
        waypoints = plan_waypoints(EXPECTED_CURRENT_RAW, target)
        self.assertEqual(waypoints[-1], target)
        self.assertLessEqual(len(waypoints), 100)
        self.assertEqual(waypoints[0][4], EXPECTED_CURRENT_RAW[4] + MAX_SEGMENT_RAW)
        prior = EXPECTED_CURRENT_RAW
        for waypoint in waypoints:
            self.assertLessEqual(max(abs(b - a) for a, b in zip(prior, waypoint)),
                                 MAX_SEGMENT_RAW)
            prior = waypoint

    def test_entry_requires_specific_authorization(self) -> None:
        with self.assertRaises(PermissionError):
            execute(operator_confirmed=False)

    def test_status_staleness_and_competing_controller_reject(self) -> None:
        observed, now = ready_observation()
        self.assertEqual(checked_feedback(observed, now), EXPECTED_CURRENT_RAW)
        with self.assertRaisesRegex(RuntimeError, "stale"):
            checked_feedback(observed, now + 200_000_000)
        observed, now = ready_observation()
        observed.add(0x155, bytes(8), now)
        with self.assertRaisesRegex(RuntimeError, "another controller"):
            checked_feedback(observed, now)

    def test_exact_own_echo_accepted_but_other_command_rejected(self) -> None:
        observed, now = ready_observation()
        payload = struct.pack(">ii", 1025, 383)
        observed.add(0x155, payload, now)
        self.assertEqual(checked_feedback(observed, now,
                         frozenset({(0x155, payload)})), EXPECTED_CURRENT_RAW)
        observed.add(0x156, bytes(8), now)
        with self.assertRaisesRegex(RuntimeError, "another controller"):
            checked_feedback(observed, now, frozenset({(0x155, payload)}))

    def test_envelope_rejects_overshoot_or_wrong_axis(self) -> None:
        prior = EXPECTED_CURRENT_RAW
        target = (1100, 100, -100, 100, 0, -500)
        self.assertTrue(within_segment(target, prior, target))
        self.assertFalse(within_segment((1500, 0, 0, 0, 0, 0), prior, target))
        self.assertFalse(within_segment((1009, 0, 0, 0, 0, 250), prior, target))


if __name__ == "__main__":
    unittest.main()
