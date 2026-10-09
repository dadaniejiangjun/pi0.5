from __future__ import annotations

import json
import hashlib
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from laptop.recorder.align_raw import QC_FLAGS, align, plan_alignment
from laptop.recorder.piper_command_targets import ACTION_TARGET_SOURCE
from laptop.recorder.verify_source import verify_source
from tests.data.test_align_raw import fixture


class SourceVerificationTest(unittest.TestCase):
    def test_every_video_frame_and_can_reference_is_rechecked(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            aligned = root / "preview"
            align(raw, aligned)
            report = verify_source(raw, aligned)
            self.assertEqual(report["samples_checked"], 11)
            self.assertEqual(report["video_frames_checked"], 22)
            self.assertTrue(report["source_frame_extraction_verified"])
            self.assertFalse(report["training_eligible"])

    def test_rejects_timestamp_that_never_existed_in_raw_log(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            aligned = root / "preview"
            align(raw, aligned)
            path = aligned / "frames.jsonl"
            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            rows[0]["source_refs"]["gripper_timestamp_ns"] += 1
            path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "reference/value mismatch"):
                verify_source(raw, aligned)

    def test_rejects_image_from_wrong_video_frame(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            aligned = root / "preview"
            align(raw, aligned)
            cv2.imwrite(str(aligned / "base_000000.png"), np.zeros((48, 64, 3), np.uint8))
            with self.assertRaisesRegex(ValueError, "differs from source video"):
                verify_source(raw, aligned)

    def test_recomputes_each_action_from_raw_command_log_even_if_sidecars_are_rehashed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, command_targets=True)
            plan = plan_alignment(raw)
            raw_manifest = json.loads((raw / "manifest.json").read_text(encoding="utf-8"))
            review = {
                "schema": "piper_alignment_review_v1",
                "raw_manifest_sha256": plan["raw_manifest_sha256"],
                "frame_map_sha256": plan["frame_map_sha256"],
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

            action_path = aligned / "actions.jsonl"
            actions = [json.loads(line) for line in action_path.read_text().splitlines()]
            actions[0]["action"][0] += 0.01
            action_path.write_text("".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
                                                for row in actions), encoding="utf-8")
            frame_path = aligned / "frames.jsonl"
            frames = [json.loads(line) for line in frame_path.read_text().splitlines()]
            frames[0]["action"] = actions[0]["action"]
            frame_path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in frames),
                                  encoding="utf-8")
            metadata_path = aligned / "aligned_episode.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["actions_sha256"] = hashlib.sha256(action_path.read_bytes()).hexdigest()
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "cannot be reproduced from raw CAN"):
                verify_source(raw, aligned)


if __name__ == "__main__":
    unittest.main()
