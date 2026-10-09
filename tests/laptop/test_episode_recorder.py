from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from laptop.recorder.episode import CHANNELS, EpisodeWriter, validate_episode


class EpisodeRecorderTest(unittest.TestCase):
    def test_synthetic_episode_is_rejected_for_training(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            writer = EpisodeWriter(Path(root), episode_id="test001")
            for channel in CHANNELS:
                writer.add(channel, 1, {"raw": 0})
                writer.add(channel, 2, {"raw": 1})
            report = writer.close()
            self.assertEqual(report["status"], "INCOMPLETE")
            self.assertFalse(report["training_eligible"])
            self.assertIn("validated action target absent", report["errors"])

    def test_rejects_nonmonotonic_and_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            writer = EpisodeWriter(Path(root), episode_id="test002")
            writer.add("status", 4, {})
            with self.assertRaises(ValueError):
                writer.add("status", 4, {})
            with self.assertRaises(FileExistsError):
                EpisodeWriter(Path(root), episode_id="test002")
            writer.close()

    def test_checksum_detects_corruption(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            writer = EpisodeWriter(Path(root), episode_id="test003")
            writer.close()
            with (writer.path / "instruction.txt").open("a", encoding="utf-8") as handle:
                handle.write("tamper")
            self.assertIn("checksum mismatch: instruction.txt", validate_episode(writer.path)["errors"])

    def test_feedback_only_operator_episode_is_integrity_valid_but_not_training_eligible(self) -> None:
        import cv2
        import numpy as np

        with tempfile.TemporaryDirectory() as temp:
            writer = EpisodeWriter(Path(temp), episode_id="operator001",
                                   source="operator_demonstration")
            for channel in CHANNELS:
                writer.add(channel, 1, {"raw": 0})
                writer.add(channel, 2, {"raw": 1})
            for role in ("base", "wrist"):
                video = cv2.VideoWriter(str(writer.path / f"{role}.mp4"),
                                        cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
                self.assertTrue(video.isOpened())
                video.write(np.full((48, 64, 3), 80, np.uint8))
                video.write(np.full((48, 64, 3), 81, np.uint8))
                video.release()
            report = writer.close()
            self.assertEqual(report["status"], "FILE_INTEGRITY_PASS")
            self.assertFalse(report["training_eligible"])
            self.assertFalse(report["hardware_control_allowed"])


if __name__ == "__main__":
    unittest.main()
