"""Synthetic videos/CAN rows through alignment, conversion, and LeRobot readback."""

from __future__ import annotations

import json
import hashlib
import importlib.util
import tempfile
import unittest
from pathlib import Path

from laptop.recorder.align_raw import QC_FLAGS, align, plan_alignment
from laptop.recorder.piper_command_targets import ACTION_TARGET_SOURCE
from tests.data.test_align_raw import fixture


@unittest.skipUnless(importlib.util.find_spec("lerobot") is not None,
                     "LeRobot is installed only in the server environment")
class RawToLeRobotTest(unittest.TestCase):
    def test_synthetic_end_to_end_with_explicit_review_and_target(self) -> None:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        from server.dataset_tools.convert_piper_aligned import convert, inspect_collection

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, command_targets=True)
            preview_path = root / "preview"
            preview = align(raw, preview_path)
            with self.assertRaisesRegex(ValueError, "wrong aligned schema"):
                inspect_collection([preview_path], raw.parent)
            rows = [json.loads(line) for line in (preview_path / "frames.jsonl").read_text().splitlines()]
            raw_manifest = json.loads((raw / "manifest.json").read_text(encoding="utf-8"))
            plan = plan_alignment(raw)
            review = {
                "schema": "piper_alignment_review_v1",
                "raw_manifest_sha256": plan["raw_manifest_sha256"],
                "frame_map_sha256": preview["frame_map_sha256"],
                "success": True,
                "joint_degrees_per_raw": 0.001,
                "gripper_meters_per_raw": 1e-6,
                "action_offset_ns": 200_000_000,
                "action_target_source": ACTION_TARGET_SOURCE,
                "action_semantics": "absolute_joint_position_rad_gripper_meter",
                "command_can_raw_sha256": raw_manifest["files"]["command_can_raw.jsonl"]["sha256"],
                "command_hold_semantics": "zero_order_hold_latest_complete_absolute_target",
                **{flag: True for flag in QC_FLAGS},
            }
            review_path = root / "review.json"
            review_path.write_text(json.dumps(review), encoding="utf-8")
            aligned = root / "aligned001"
            align(raw, aligned, review_path=review_path)
            self.assertEqual((aligned / "review.json").read_bytes(), review_path.read_bytes())
            actions = [json.loads(line) for line in (aligned / "actions.jsonl").read_text().splitlines()]
            self.assertEqual(len(actions), len(rows))
            self.assertTrue(all(row["action_target_source"] == ACTION_TARGET_SOURCE for row in actions))
            checked = inspect_collection([aligned], raw.parent)
            output = root / "piper_derived"
            report = convert(checked, output, "offline/piper_alignment_test")
            self.assertEqual(report["status"], "CONVERTED_NEEDS_AUDIT")
            self.assertEqual(report["frame_count"], len(rows))
            self.assertFalse(report["training_eligible"])
            self.assertTrue(report["source_frame_extraction_verified"])
            self.assertTrue(report["command_target_provenance_verified"])
            acceptance = json.loads((output / "provenance/acceptance_manifest.json").read_text())
            self.assertEqual(acceptance["schema"], "piper_real_demo_acceptance_v1")
            self.assertEqual(acceptance["episodes"][0]["action_target_source"], ACTION_TARGET_SOURCE)
            self.assertTrue(acceptance["episodes"][0]["command_target_provenance_verified"])
            self.assertEqual(
                report["acceptance_manifest_sha256"],
                hashlib.sha256((output / "provenance/acceptance_manifest.json").read_bytes()).hexdigest(),
            )
            data = LeRobotDataset("offline/piper_alignment_test", root=output)
            self.assertEqual(len(data), len(rows))
            self.assertEqual(data[0]["observation.state"].shape[0], 7)
            self.assertEqual(data[0]["action"].shape[0], 7)
            source_map = [json.loads(line) for line in
                          (output / "provenance/source_map.jsonl").read_text().splitlines()]
            self.assertEqual(source_map[0]["source_refs"]["base_frame_id"], 1)
            self.assertEqual(source_map[0]["action_target_source"], ACTION_TARGET_SOURCE)
            self.assertEqual(source_map[0]["action_source_refs"]["joint_command_frames"][0]["can_id"], "0x155")
            aligned_rows_path = aligned / "frames.jsonl"
            aligned_rows = [json.loads(line) for line in aligned_rows_path.read_text().splitlines()]
            aligned_rows[0]["state"][0] += 0.1
            aligned_rows_path.write_text("".join(json.dumps(row) + "\n" for row in aligned_rows))
            with self.assertRaisesRegex(ValueError, "canonical state conversion mismatch"):
                inspect_collection([aligned], raw.parent)


if __name__ == "__main__":
    unittest.main()
