from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from laptop.diagnostics.compare_gripper_feedback import compare


class CompareGripperFeedbackTest(unittest.TestCase):
    def _log(self, path: Path, value: int) -> None:
        with path.open("w", encoding="utf-8") as handle:
            for index in range(3):
                for can_id in ("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8"):
                    handle.write(json.dumps({
                        "monotonic_ns": index * 1_000_000_000,
                        "can_id": can_id,
                        "joint_raw_0p001deg_nominal": [0] * 6,
                        "gripper_raw": {"angle_or_stroke": value},
                        "status": {"ctrl_mode": "CAN_CTRL(0x1)"},
                    }) + "\n")

    def test_feedback_scale_observed_but_control_not_unlocked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            before, after = Path(directory) / "before.jsonl", Path(directory) / "after.jsonl"
            self._log(before, 40200)
            self._log(after, 39000)
            with patch("laptop.diagnostics.compare_gripper_feedback.audit",
                       return_value={"status": "READ_CONTINUITY_PASS"}):
                result = compare(before, after, oem_feedback_mm=39.0, oem_target_mm=38.686)
            self.assertEqual(result["status"], "FEEDBACK_SCALE_OBSERVED")
            self.assertEqual(result["raw_delta"], -1200)
            self.assertEqual(result["after_raw_as_mm"], 39.0)
            self.assertFalse(result["target_mapping_verified"])
            self.assertFalse(result["hardware_control_allowed"])

    def test_no_change_and_display_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            before, after = Path(directory) / "before.jsonl", Path(directory) / "after.jsonl"
            self._log(before, 40200)
            self._log(after, 40200)
            with patch("laptop.diagnostics.compare_gripper_feedback.audit",
                       return_value={"status": "READ_CONTINUITY_PASS"}):
                with self.assertRaisesRegex(ValueError, "no observed gripper feedback change"):
                    compare(before, after, oem_feedback_mm=39.0)
                self._log(after, 39000)
                result = compare(before, after, oem_feedback_mm=37.0)
            self.assertEqual(result["status"], "DISPLAY_MISMATCH")


if __name__ == "__main__":
    unittest.main()
