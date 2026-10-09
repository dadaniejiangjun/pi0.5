from __future__ import annotations

import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from laptop.recorder.align_raw import QC_FLAGS, align, plan_alignment
from laptop.recorder.episode import EpisodeWriter


def fixture(root: Path, *, source: str = "manual_teach", frame_count: int = 12,
            episode_id: str = "raw001",
            command_targets: bool = False, gripper_offset: int = 0) -> Path:
    writer = EpisodeWriter(root / "raw", episode_id=episode_id, source=source)
    for role in ("base", "wrist"):
        video = cv2.VideoWriter(str(writer.path / f"{role}.mp4"),
                                cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
        if not video.isOpened():
            raise RuntimeError("test video encoder unavailable")
        for index in range(frame_count):
            pixel_value = (25 + index * 10 + (50 if role == "wrist" else 0)) % 256
            frame = np.full((48, 64, 3), pixel_value, np.uint8)
            video.write(frame)
        video.release()
    for index in range(frame_count):
        ts = 1_000_000_000 + index * 100_000_000
        writer.add("base", ts, {"frame_id": index})
        writer.add("wrist", ts + 1_000_000, {"frame_id": index})
        for offset, channel in enumerate(("joint_12", "joint_34", "joint_56"), start=2):
            writer.add(channel, ts + offset * 1_000_000,
                       {"raw_joint_pair": [index * 1000 + offset, index * 1000 + offset + 1]})
        writer.add("gripper", ts + 5_000_000,
                   {"angle_or_stroke": index * 100 + gripper_offset, "status_code": 64})
        writer.add("status", ts + 6_000_000,
                   {"ctrl_mode": "TEACHING_MODE(0x2)", "arm_status": "NORMAL(0x0)",
                    "teach_status": "STOP_RECORDING(0x2)", "err_code": 0})
    if command_targets:
        command_path = writer.path / "command_can_raw.jsonl"
        with command_path.open("x", encoding="utf-8") as handle:
            log_index = 0
            for tick in range(frame_count):
                base_ts = 1_000_000_000 + tick * 100_000_000
                if tick == 0:
                    payload = struct.pack(">iHBB", 10_000, 100, 0x01, 0x00)
                    handle.write(json.dumps({
                        "monotonic_ns": base_ts + 1, "can_id": "0x159",
                        "raw_payload_hex": payload.hex(), "is_extended_id": False,
                        "is_error_frame": False, "capture_source": "passive_usb_can_bus_receive",
                        "direction_verified": False,
                    }) + "\n")
                    log_index += 1
                for pair, can_id in enumerate((0x155, 0x156, 0x157)):
                    values = (tick * 1000 + pair * 10, tick * 1000 + pair * 10 + 1)
                    payload = struct.pack(">ii", *values)
                    handle.write(json.dumps({
                        "monotonic_ns": base_ts + 2 + pair, "can_id": f"0x{can_id:03X}",
                        "raw_payload_hex": payload.hex(), "is_extended_id": False,
                        "is_error_frame": False, "capture_source": "passive_usb_can_bus_receive",
                        "direction_verified": False,
                    }) + "\n")
                    log_index += 1
    writer.close()
    return writer.path


class RawAlignmentTest(unittest.TestCase):
    def test_thirty_fps_grid_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            raw = fixture(Path(temp))
            plan = plan_alignment(raw, fps=30)
            self.assertEqual(plan["fps"], 30)
            self.assertGreater(len(plan["frames"]), 11)

    def test_preview_extracts_real_video_indices_and_preserves_raw(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            original = hashlib.sha256((raw / "manifest.json").read_bytes()).hexdigest()
            output = root / "preview"
            report = align(raw, output)
            self.assertEqual(report["status"], "PREVIEW_ONLY")
            self.assertEqual(report["frame_count"], 11)
            self.assertFalse(report["training_eligible"])
            rows = [json.loads(line) for line in (output / "frames.jsonl").read_text().splitlines()]
            self.assertEqual(rows[0]["source_refs"]["base_frame_id"], 1)
            self.assertEqual(rows[-1]["source_refs"]["wrist_frame_id"], 11)
            self.assertEqual(rows[0]["policy_timestamp_ns"], 1_100_000_000)
            self.assertNotIn("action", rows[0])
            self.assertNotIn("state", rows[0])
            self.assertEqual(len(rows[0]["joint_rad_nominal_only"]), 6)
            image = cv2.imread(str(output / rows[0]["base_image"]))
            self.assertIsNotNone(image)
            self.assertAlmostEqual(float(image.mean()), 35.0, delta=3.0)
            self.assertEqual(hashlib.sha256((raw / "manifest.json").read_bytes()).hexdigest(), original)
            with self.assertRaisesRegex(ValueError, "unused"):
                align(raw, output)

    def test_reviewed_alignment_has_converter_fields_and_checksum_binding(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            preview = align(raw, root / "preview")
            preview_rows = [json.loads(line) for line in (root / "preview/frames.jsonl").read_text().splitlines()]
            actions = root / "targets.jsonl"
            with actions.open("x", encoding="utf-8") as handle:
                for index, row in enumerate(preview_rows):
                    handle.write(json.dumps({"frame_index": index,
                                             "action_target_timestamp_ns": row["policy_timestamp_ns"] + 100_000_000,
                                             "action": [index / 100] * 6 + [index / 1000]}) + "\n")
            review = {
                "schema": "piper_alignment_review_v1",
                "raw_manifest_sha256": hashlib.sha256((raw / "manifest.json").read_bytes()).hexdigest(),
                "frame_map_sha256": preview["frame_map_sha256"], "success": True,
                "joint_degrees_per_raw": 0.001, "gripper_meters_per_raw": 1e-6,
                "action_offset_ns": 100_000_000, "action_target_source": "operator_verified_target",
                "actions_file": actions.name,
                "actions_sha256": hashlib.sha256(actions.read_bytes()).hexdigest(),
                **{flag: True for flag in QC_FLAGS},
            }
            review_path = root / "review.json"
            review_path.write_text(json.dumps(review), encoding="utf-8")
            out = root / "aligned001"
            result = align(raw, out, review_path=review_path)
            self.assertEqual(result["status"], "ALIGNED_REVIEWED_NEEDS_CONVERTER_AUDIT")
            meta = json.loads((out / "aligned_episode.json").read_text())
            self.assertEqual(meta["schema"], "piper_aligned_episode_v1")
            self.assertFalse(meta["training_eligible"])
            first = json.loads((out / "frames.jsonl").read_text().splitlines()[0])
            self.assertEqual(first["source_refs"]["base_frame_id"], 1)
            self.assertEqual(len(first["state"]), 7)
            self.assertAlmostEqual(first["state"][6], 0.0001)
            self.assertEqual(first["action"], [0.0] * 7)
            self.assertEqual(first["action_target_timestamp_ns"] - first["policy_timestamp_ns"], 100_000_000)

    def test_command_frames_generate_per_frame_actions_from_hashed_raw_source(self) -> None:
        from laptop.recorder.piper_command_targets import ACTION_TARGET_SOURCE

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, command_targets=True)
            preview = align(raw, root / "preview")
            manifest = json.loads((raw / "manifest.json").read_text(encoding="utf-8"))
            review = {
                "schema": "piper_alignment_review_v1",
                "raw_manifest_sha256": hashlib.sha256((raw / "manifest.json").read_bytes()).hexdigest(),
                "frame_map_sha256": preview["frame_map_sha256"],
                "success": True,
                "joint_degrees_per_raw": 0.001,
                "gripper_meters_per_raw": 1e-6,
                "action_offset_ns": 200_000_000,
                "action_target_source": ACTION_TARGET_SOURCE,
                "action_semantics": "absolute_joint_position_rad_gripper_meter",
                "command_can_raw_sha256": manifest["files"]["command_can_raw.jsonl"]["sha256"],
                "command_hold_semantics": "zero_order_hold_latest_complete_absolute_target",
                **{flag: True for flag in QC_FLAGS},
            }
            review_path = root / "review.json"
            review_path.write_text(json.dumps(review), encoding="utf-8")
            aligned = root / "aligned001"
            align(raw, aligned, review_path=review_path)
            metadata = json.loads((aligned / "aligned_episode.json").read_text(encoding="utf-8"))
            self.assertTrue(metadata["action_target_provenance_verified"])
            self.assertEqual(metadata["action_target_source"], ACTION_TARGET_SOURCE)
            action_rows = [json.loads(line) for line in (aligned / "actions.jsonl").read_text().splitlines()]
            frames = [json.loads(line) for line in (aligned / "frames.jsonl").read_text().splitlines()]
            self.assertEqual(action_rows[0]["action_target_timestamp_ns"],
                             frames[0]["policy_timestamp_ns"] + 200_000_000)
            self.assertEqual(action_rows[0]["action_target_raw"][0], 2000)
            self.assertEqual(action_rows[0]["action_source_refs"]["joint_command_frames"][0]["can_id"],
                             "0x155")
            from laptop.recorder.verify_source import verify_source
            result = verify_source(raw, aligned)
            self.assertTrue(result["command_target_provenance_verified"])

    def test_missing_or_stale_review_and_tampered_raw_fail_before_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root)
            plan = plan_alignment(raw)
            review = root / "review.json"
            review.write_text(json.dumps({"schema": "piper_alignment_review_v1",
                                          "frame_map_sha256": "wrong",
                                          "raw_manifest_sha256": plan["raw_manifest_sha256"]}))
            with self.assertRaisesRegex(ValueError, "not bound"):
                align(raw, root / "aligned001", review_path=review)
            self.assertFalse((root / "aligned001").exists())
            (raw / "joint_12.jsonl").write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksum"):
                align(raw, root / "other")
            self.assertFalse((root / "other").exists())

    def test_passive_or_synthetic_raw_cannot_be_promoted_to_reviewed_demo(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, source="passive_wiring_smoke")
            plan = plan_alignment(raw)
            review = root / "review.json"
            review.write_text(json.dumps({"schema": "piper_alignment_review_v1",
                                          "frame_map_sha256": plan["frame_map_sha256"],
                                          "raw_manifest_sha256": plan["raw_manifest_sha256"],
                                          "success": True}))
            with self.assertRaisesRegex(ValueError, "successful operator"):
                align(raw, root / "aligned001", review_path=review)


if __name__ == "__main__":
    unittest.main()
