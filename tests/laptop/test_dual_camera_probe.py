from __future__ import annotations

import unittest

import numpy as np

from laptop.diagnostics.probe_dual_cameras import (
    _frame_content_sample,
    _image_content_summary,
    _nearest_skew_ms,
    _percentile,
    _summary,
)


class DualCameraProbeTest(unittest.TestCase):
    def test_percentiles_and_empty_series(self) -> None:
        self.assertIsNone(_percentile([], 95))
        self.assertEqual(_percentile([7], 95), 7)
        self.assertEqual(_percentile([0, 10, 20], 50), 10)
        self.assertEqual(_percentile([0, 10, 20], 95), 19)

    def test_nearest_skew_uses_host_arrival_times(self) -> None:
        self.assertIsNone(_nearest_skew_ms([], [1]))
        self.assertEqual(
            _nearest_skew_ms([0, 20_000_000], [5_000_000, 25_000_000]),
            {"median": 5.0, "p95": 5.0, "max": 5.0},
        )

    def test_summary_marks_long_arrival_gap_without_claiming_dropped_frame(self) -> None:
        result = _summary(0, [0, 33_000_000, 166_000_000], 0, {(480, 640, 3)}, 1.0)
        self.assertEqual(result["frames_ok"], 3)
        self.assertEqual(result["read_failures"], 0)
        self.assertEqual(result["interval_gt_100ms_count"], 1)
        self.assertEqual(result["shapes"], [[480, 640, 3]])

    def test_near_black_frames_are_not_misclassified_as_usable_camera_content(self) -> None:
        first = np.zeros((480, 640, 3), dtype=np.uint8)
        first[0, :, :] = 255
        second = np.zeros((480, 640, 3), dtype=np.uint8)
        second[:26, :, :] = 20

        samples = [_frame_content_sample(first), _frame_content_sample(second)]
        summary = _image_content_summary(samples)

        self.assertTrue(all(sample["near_black"] for sample in samples))
        self.assertEqual(summary["status"], "LOW_INFORMATION_NEAR_BLACK")
        self.assertEqual(summary["near_black_sample_count"], 2)

    def test_visible_signal_is_not_a_role_confirmation(self) -> None:
        frame = np.zeros((16, 16, 3), dtype=np.uint8)
        frame[:, 8:, :] = 180

        sample = _frame_content_sample(frame)
        summary = _image_content_summary([sample])

        self.assertFalse(sample["near_black"])
        self.assertEqual(summary["status"], "SIGNAL_PRESENT")
        self.assertFalse(summary["role_verified"])


if __name__ == "__main__":
    unittest.main()
