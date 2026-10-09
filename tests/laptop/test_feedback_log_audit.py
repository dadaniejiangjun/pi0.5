from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from laptop.diagnostics.audit_feedback_log import REQUIRED, audit
from laptop.diagnostics.probe_piper_feedback_ack_only import probe


class FeedbackLogAuditTest(unittest.TestCase):
    def test_guard_does_not_create_log(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "feedback.jsonl"
            report = probe(5, approved=False, feedback_log=path)
            self.assertEqual(report["status"], "BLOCKED_APPROVAL_REQUIRED")
            self.assertFalse(path.exists())

    def test_offline_continuity_does_not_claim_manual_response(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "feedback.jsonl"
            with path.open("w", encoding="utf-8") as handle:
                for can_id in REQUIRED:
                    for ts in range(1_000_000_000, 3_100_000_000, 50_000_000):
                        handle.write(json.dumps({"can_id": can_id, "monotonic_ns": ts,
                                                 "joint_raw_0p001deg_nominal": [0] * 6,
                                                 "gripper_raw": {"angle_or_stroke": 0},
                                                 "status": {"ctrl_mode": "CAN_CTRL"}}) + "\n")
            report = audit(path)
            self.assertEqual(report["status"], "READ_CONTINUITY_PASS")
            self.assertFalse(report["manual_motion_response_verified"])
            self.assertEqual(report["distinct_joint_snapshots_after_0p5s"], 1)

    def test_coarse_clock_is_not_a_continuity_pass(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "feedback.jsonl"
            with path.open("w", encoding="utf-8") as handle:
                for can_id in REQUIRED:
                    for ts in [1_000_000_000, *range(1_000_000_000, 3_100_000_000, 50_000_000)]:
                        handle.write(json.dumps({"can_id": can_id, "monotonic_ns": ts,
                                                 "joint_raw_0p001deg_nominal": [0] * 6,
                                                 "gripper_raw": {"angle_or_stroke": 0},
                                                 "status": {"ctrl_mode": "CAN_CTRL"}}) + "\n")
            self.assertEqual(audit(path)["status"], "CLOCK_RESOLUTION_INSUFFICIENT")


if __name__ == "__main__":
    unittest.main()
