from __future__ import annotations

import math
import unittest

from laptop.robot_client.replay_preflight import analyze


def row(index: int, j5_deg: float = 0.0, *, gripper: int = 0) -> dict:
    joints = [0.0, math.radians(10), math.radians(-10), 0.0,
              math.radians(j5_deg), 0.0]
    return {"frame_index": index, "policy_timestamp_ns": index * 500_000_000,
            "joint_rad_nominal_only": joints, "gripper_raw": gripper}


class ReplayPreflightTest(unittest.TestCase):
    def test_large_sampled_step_is_retimed_but_never_clears_hardware(self) -> None:
        report = analyze([row(0, -24.24), row(1, 31.457)])
        self.assertAlmostEqual(report["max_adjacent_joint_step_deg"], 55.697, places=3)
        self.assertGreater(report["joint_only_candidate_duration_s"], 8)
        self.assertEqual(report["retimed_segments"], 1)
        self.assertFalse(report["complete_7d_replay_available"])
        self.assertFalse(report["hardware_control_allowed"])
        self.assertEqual(report["can_data_tx_calls"], 0)

    def test_invalid_frames_fail_closed(self) -> None:
        cases = [
            [row(0), row(0)],
            [row(0), {**row(1), "policy_timestamp_ns": 0}],
            [row(0), {**row(1), "joint_rad_nominal_only": [float("nan")] * 6}],
            [row(0), row(1, 80)],
            [row(0), {**row(1), "gripper_raw": -1}],
        ]
        for rows in cases:
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                analyze(rows)

    def test_invalid_planning_limits_fail_closed(self) -> None:
        for bad in (0, -1, float("nan"), float("inf")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                analyze([row(0), row(1)], planning_speed_deg_s=bad)


if __name__ == "__main__":
    unittest.main()
