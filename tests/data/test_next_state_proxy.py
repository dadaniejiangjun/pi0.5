from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from laptop.recorder.align_raw import align
from laptop.recorder.label_episode import label_episode
from laptop.recorder.next_state_proxy import audit_transition_deltas, build
from tests.data.test_align_raw import fixture


class NextStateProxyTest(unittest.TestCase):
    def test_successful_preview_generates_next_feedback_not_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            preview = root / "preview"
            align(raw, preview)
            label = label_episode(raw, root / "outcomes", success=True,
                                  notes="Operator confirmed gripper-held task",
                                  operator="test-operator")
            report = build(preview, raw, Path(label["path"]), root / "proxy")
            rows = [json.loads(line) for line in
                    (root / "proxy/proxy_targets.jsonl").read_text().splitlines()]
            self.assertEqual(report["status"], "PROXY_ONLY")
            self.assertEqual(len(rows), 10)
            self.assertEqual(rows[0]["proxy_action"], rows[1]["state"])
            self.assertEqual(rows[0]["target_source_frame_index"], 1)
            self.assertEqual(report["target_offset_ns"], 100_000_000)
            self.assertEqual(report["transition_delta_audit"]["transitions"], 10)
            self.assertFalse(
                report["transition_delta_audit"]["physical_velocity_verified"]
            )
            self.assertFalse(report["formal_training_eligible"])
            self.assertFalse(report["hardware_control_allowed"])
            with self.assertRaisesRegex(ValueError, "unused"):
                build(preview, raw, Path(label["path"]), root / "proxy")

    def test_transition_delta_audit_flags_limits_without_changing_targets(self) -> None:
        target = [math.radians(1.1), 0.0, 0.0, 0.0, 0.0, 0.0, 0.002]
        record = {"state": [0.0] * 7, "proxy_action": target.copy()}

        report = audit_transition_deltas([record])

        self.assertFalse(report["within_current_first_step_software_caps"])
        self.assertEqual(report["transitions_with_any_joint_delta_over_limit"], 1)
        self.assertEqual(report["joint_transitions_over_limit_by_axis"]["J1"], 1)
        self.assertEqual(report["gripper_transitions_over_limit"], 1)
        self.assertFalse(report["targets_modified_or_clamped"])
        self.assertEqual(record["proxy_action"], target)
        self.assertFalse(report["hardware_units_verified"])

    def test_signed_gripper_feedback_is_preserved_as_observation(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, gripper_offset=-1000)
            preview = root / "preview"
            align(raw, preview)
            label = label_episode(raw, root / "outcomes", success=True,
                                  notes="Signed feedback fixture", operator="test-operator")
            build(preview, raw, Path(label["path"]), root / "proxy")
            rows = [json.loads(line) for line in
                    (root / "proxy/proxy_targets.jsonl").read_text().splitlines()]
            self.assertAlmostEqual(rows[0]["state"][6], -0.0009)
            self.assertAlmostEqual(rows[0]["proxy_action"][6], -0.0008)

    def test_failed_outcome_cannot_generate_proxy(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            preview = root / "preview"
            align(raw, preview)
            label = label_episode(raw, root / "outcomes", success=False,
                                  notes="Task failed", operator="test-operator")
            with self.assertRaisesRegex(ValueError, "successful"):
                build(preview, raw, Path(label["path"]), root / "proxy")


if __name__ == "__main__":
    unittest.main()
