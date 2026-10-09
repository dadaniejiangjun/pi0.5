from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from laptop.recorder.episode import EpisodeWriter
from laptop.recorder.label_episode import label_episode
from laptop.recorder.capture_session import run_session
from laptop.recorder.passive_capture import assemble, capture
from laptop.transport.gs_usb_piper_can import CanReceiveMode


class PassiveCaptureTest(unittest.TestCase):
    def test_default_blocks_before_devices_or_episode(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            result = capture(Path(root), base_index=0, wrist_index=1)
            self.assertEqual(result["status"], "BLOCKED_APPROVAL_REQUIRED")
            self.assertFalse(list(Path(root).iterdir()))

    def test_duration_and_roles_checked_without_hardware(self) -> None:
        with self.assertRaises(ValueError):
            capture(Path("unused"), duration_s=10.1)
        with self.assertRaises(ValueError):
            capture(Path("unused"), duration_s=20.1, manual_teach=True)
        with self.assertRaises(ValueError):
            capture(Path("unused"), base_index=0, wrist_index=0)
        with self.assertRaises(ValueError):
            capture(Path("unused"), manual_teach=True, lead_in_s=10.1)

    def test_manual_capture_requires_approval_before_devices_or_episode(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            result = capture(Path(root), duration_s=20, manual_teach=True)
            self.assertEqual(result["status"], "BLOCKED_APPROVAL_REQUIRED")
            self.assertFalse(list(Path(root).iterdir()))

    def test_offline_assembly_preserves_camera_and_raw_feedback_times(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            writer = EpisodeWriter(Path(root), episode_id="passive1", source="passive_wiring_smoke")
            feedback_log = writer.path / "feedback_can_raw.jsonl"
            with feedback_log.open("w", encoding="utf-8") as handle:
                for can_id in ("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8"):
                    handle.write(json.dumps({"can_id": can_id, "monotonic_ns": 200,
                                             "raw_payload_hex": "0000000000000000",
                                             "joint_raw_0p001deg_nominal": [1, 2, 3, 4, 5, 6],
                                             "gripper_raw": {"angle_or_stroke": 7},
                                             "status": {"ctrl_mode": "TEACHING_MODE"}}) + "\n")
            assemble(writer, {"frame_timestamps_ns": {"base": [100], "wrist": [110]}}, feedback_log)
            self.assertEqual(writer.counts["base"], 1)
            self.assertEqual(writer.counts["wrist"], 1)
            self.assertEqual(writer.counts["joint_34"], 1)
            self.assertEqual(writer.counts["gripper"], 1)
            writer.close()
            self.assertEqual(json.loads((writer.path / "joint_34.jsonl").read_text())["raw_joint_pair"], [3, 4])

    def test_full_wiring_with_fake_camera_and_can(self) -> None:
        import cv2
        import numpy as np

        def fake_cameras(_base, _wrist, _duration, video_dir, *, start_gate, on_opened,
                         start_gate_timeout_s):
            self.assertEqual(start_gate_timeout_s, 10)
            on_opened(True)
            self.assertTrue(start_gate.wait(timeout=1), "camera must wait for CAN receiver")
            for role in ("base", "wrist"):
                video = cv2.VideoWriter(str(video_dir / f"{role}.mp4"),
                                        cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
                self.assertTrue(video.isOpened())
                for _ in range(10):
                    video.write(np.zeros((48, 64, 3), dtype=np.uint8))
                video.release()
            return {"status": "CONCURRENT_STREAM_PASS",
                    "frame_timestamps_ns": {"base": [1_000_000_000 + i * 100_000_000 for i in range(10)],
                                            "wrist": [1_005_000_000 + i * 100_000_000 for i in range(10)]}}

        def fake_can(_duration, approved, feedback_log, *, max_duration_s, on_receiver_ready,
                     command_log=None, adapter_bus=None, adapter_address=None,
                     receive_mode=CanReceiveMode.NORMAL_ACKING):
            self.assertTrue(approved)
            self.assertEqual(_duration, 10)
            self.assertEqual(max_duration_s, 10)
            on_receiver_ready()
            with feedback_log.open("x", encoding="utf-8") as handle:
                for i in range(10):
                    for offset, can_id in enumerate(("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8")):
                        handle.write(json.dumps({
                            "can_id": can_id, "monotonic_ns": 1_000_000_000 + i * 100_000_000 + offset,
                            "raw_payload_hex": "0000000000000000",
                            "joint_raw_0p001deg_nominal": [1, 2, 3, 4, 5, 6],
                            "gripper_raw": {"angle_or_stroke": 7},
                            "status": {"ctrl_mode": "TEACHING_MODE"},
                        }) + "\n")
            return {"status": "FEEDBACK_FIELDS_PRESENT"}

        with tempfile.TemporaryDirectory() as root:
            with patch("laptop.diagnostics.probe_dual_cameras.probe", fake_cameras), \
                 patch("laptop.diagnostics.probe_piper_feedback_ack_only.probe", fake_can):
                identity = {"schema": "piper_camera_role_identity_v1",
                            "roles": {"base": {"opencv_index": 0},
                                      "wrist": {"opencv_index": 1}}}
                result = capture(Path(root), approved=True, camera_identity=identity)
            self.assertEqual(result["status"], "PASSIVE_CAPTURE_RECORDED", result)
            self.assertEqual(result["integrity"]["decoded_video_frames"], {"base": 10, "wrist": 10})
            self.assertFalse(result["training_eligible"])
            self.assertEqual(result["can_data_tx_calls"], 0)
            episode = Path(result["path"])
            report = json.loads((episode / "camera_report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["camera_identity"], identity)
            manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8"))
            self.assertIn("camera_identity.json", manifest["files"])

    def test_camera_open_failure_never_starts_can(self) -> None:
        def fake_cameras(_base, _wrist, _duration, _video_dir, *, start_gate, on_opened,
                         start_gate_timeout_s):
            on_opened(False)
            return {"status": "OPEN_FAILED"}

        with tempfile.TemporaryDirectory() as root:
            with patch("laptop.diagnostics.probe_dual_cameras.probe", fake_cameras), \
                 patch("laptop.diagnostics.probe_piper_feedback_ack_only.probe") as can_probe:
                result = capture(Path(root), approved=True)
            can_probe.assert_not_called()
            self.assertEqual(result["status"], "PARTIAL")
            self.assertIn("both cameras must open", result["can_error"])

    def test_manual_capture_is_raw_and_not_training_eligible(self) -> None:
        import cv2
        import numpy as np

        def fake_cameras(_base, _wrist, _duration, video_dir, *, start_gate, on_opened,
                         start_gate_timeout_s):
            self.assertEqual(start_gate_timeout_s, 20)
            on_opened(True)
            self.assertTrue(start_gate.wait(timeout=1))
            for role in ("base", "wrist"):
                video = cv2.VideoWriter(str(video_dir / f"{role}.mp4"),
                                        cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
                self.assertTrue(video.isOpened())
                video.write(np.zeros((48, 64, 3), dtype=np.uint8))
                video.release()
            return {"status": "CONCURRENT_STREAM_PASS",
                    "frame_timestamps_ns": {"base": [1_000_000_000], "wrist": [1_001_000_000]}}

        def fake_can(duration, approved, feedback_log, *, max_duration_s, on_receiver_ready,
                     command_log=None, adapter_bus=None, adapter_address=None,
                     receive_mode=CanReceiveMode.NORMAL_ACKING):
            self.assertEqual(duration, 20)
            self.assertTrue(approved)
            self.assertEqual(max_duration_s, 20)
            self.assertEqual((adapter_bus, adapter_address), (2, 7))
            self.assertIs(receive_mode, CanReceiveMode.LISTEN_ONLY)
            on_receiver_ready()
            with feedback_log.open("x", encoding="utf-8") as handle:
                for offset, can_id in enumerate(("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8")):
                    handle.write(json.dumps({
                        "can_id": can_id, "monotonic_ns": 1_000_000_000 + offset,
                        "raw_payload_hex": "0000000000000000",
                        "joint_raw_0p001deg_nominal": [1, 2, 3, 4, 5, 6],
                        "gripper_raw": {"angle_or_stroke": 7},
                        "status": {"ctrl_mode": "TEACHING_MODE"},
                    }) + "\n")
            self.assertIsNotNone(command_log)
            command_frames = [
                ("0x159", struct.pack(">iHBB", 10_000, 100, 1, 0)),
                ("0x155", struct.pack(">ii", 1, 2)),
                ("0x156", struct.pack(">ii", 3, 4)),
                ("0x157", struct.pack(">ii", 5, 6)),
            ]
            with command_log.open("x", encoding="utf-8") as handle:
                for index, (can_id, payload) in enumerate(command_frames):
                    handle.write(json.dumps({
                        "monotonic_ns": 2_000_000_000 + index,
                        "can_id": can_id, "raw_payload_hex": payload.hex(),
                        "is_extended_id": False, "is_error_frame": False,
                        "capture_source": "passive_usb_can_bus_receive",
                        "direction_verified": False,
                    }) + "\n")
            return {"status": "FEEDBACK_FIELDS_PRESENT"}

        with tempfile.TemporaryDirectory() as root:
            with patch("laptop.diagnostics.probe_dual_cameras.probe", fake_cameras), \
                 patch("laptop.diagnostics.probe_piper_feedback_ack_only.probe", fake_can), \
                 patch("laptop.recorder.passive_capture.time.sleep"):
                result = capture(Path(root), duration_s=20, approved=True,
                                 manual_teach=True, lead_in_s=10,
                                 can_adapter_bus=2, can_adapter_address=7)
            self.assertEqual(result["status"], "MANUAL_CAPTURE_RECORDED")
            self.assertFalse(result["training_eligible"])
            episode = Path(root) / Path(result["path"]).name
            metadata = json.loads((episode / "metadata.json").read_text())
            self.assertEqual(metadata["source"], "manual_teach")
            events = [json.loads(line)["name"] for line in (episode / "events.jsonl").read_text().splitlines()]
            self.assertIn("capture_start", events)
            self.assertIn("capture_end", events)
            self.assertEqual(result["can_data_tx_calls"], 0)
            self.assertEqual(result["command_log"], str(episode / "command_can_raw.jsonl"))
            manifest = json.loads((episode / "manifest.json").read_text())
            self.assertIn("command_can_raw.jsonl", manifest["files"])

    def test_manual_command_capture_requires_exact_separate_listener(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(ValueError, "separate passive adapter"):
                capture(Path(root), duration_s=20, approved=True, manual_teach=True)
            self.assertFalse(list(Path(root).iterdir()))

    def test_feedback_only_operator_demo_needs_no_command_listener_and_remains_offline_only(self) -> None:
        import cv2
        import numpy as np

        def fake_cameras(_base, _wrist, duration, video_dir, *, start_gate, on_opened,
                         start_gate_timeout_s):
            self.assertEqual(duration, 15)
            self.assertEqual(start_gate_timeout_s, 15)
            on_opened(True)
            self.assertTrue(start_gate.wait(timeout=1))
            stamps = {"base": [], "wrist": []}
            for role in ("base", "wrist"):
                writer = cv2.VideoWriter(str(video_dir / f"{role}.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), 10, (64, 48))
                self.assertTrue(writer.isOpened())
                for index in range(15):
                    writer.write(np.full((48, 64, 3), index + 20, np.uint8))
                    stamps[role].append(1_010_000_000 + index * 100_000_000 + (0 if role == "base" else 1_000_000))
                writer.release()
            return {"status": "CONCURRENT_STREAM_PASS", "frame_timestamps_ns": stamps}

        def fake_can(duration, approved, feedback_log, *, max_duration_s, on_receiver_ready,
                     command_log=None, adapter_bus=None, adapter_address=None,
                     receive_mode=CanReceiveMode.NORMAL_ACKING):
            self.assertTrue(approved)
            self.assertEqual(duration, 15)
            self.assertEqual(max_duration_s, 60)
            self.assertIsNone(command_log)
            self.assertIsNone(adapter_bus)
            self.assertIsNone(adapter_address)
            self.assertIs(receive_mode, CanReceiveMode.NORMAL_ACKING)
            on_receiver_ready()
            with Path(feedback_log).open("x", encoding="utf-8") as handle:
                for index in range(16):
                    base = 1_000_000_000 + index * 100_000_000
                    for offset, can_id in enumerate(("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8")):
                        handle.write(json.dumps({
                            "can_id": can_id, "monotonic_ns": base + offset + 2_000_000,
                            "raw_payload_hex": "0000000000000000",
                            "joint_raw_0p001deg_nominal": [index] * 6,
                            "gripper_raw": {"angle_or_stroke": index, "status_code": 64},
                            "status": {"ctrl_mode": "TEACHING_MODE", "err_code": 0},
                        }) + "\n")
            return {"status": "FEEDBACK_FIELDS_PRESENT", "observed_command_id_counts": {}}

        with tempfile.TemporaryDirectory() as temp:
            identity = {"schema": "piper_camera_role_identity_v1",
                        "roles": {"base": {"opencv_index": 0}, "wrist": {"opencv_index": 1}}}
            with patch("laptop.diagnostics.probe_dual_cameras.probe", fake_cameras), \
                 patch("laptop.diagnostics.probe_piper_feedback_ack_only.probe", fake_can), \
                 patch("laptop.recorder.passive_capture.time.sleep"):
                result = capture(Path(temp) / "raw", approved=True, operator_demo=True,
                                 duration_s=15, lead_in_s=5, camera_identity=identity)
            self.assertEqual(result["status"], "OPERATOR_DEMO_CAPTURE_RECORDED", result)
            self.assertEqual(result["integrity"]["status"], "FILE_INTEGRITY_PASS")
            self.assertFalse(result["training_eligible"])
            self.assertEqual(result["can_data_tx_calls"], 0)
            manifest = json.loads((Path(result["path"]) / "manifest.json").read_text())
            self.assertEqual(manifest["source"], "operator_demonstration")
            label = label_episode(Path(result["path"]), Path(temp) / "outcomes",
                                  success=True, notes="observed placement", operator="test")
            self.assertTrue(label["success"])
            self.assertFalse(label["training_eligible"])

    def test_capture_session_labels_success_and_failure_separately(self) -> None:
        def fake_capture(raw_root, *, camera_identity, approved, operator_demo,
                         duration_s, lead_in_s):
            self.assertTrue(approved)
            self.assertTrue(operator_demo)
            writer = EpisodeWriter(raw_root, source="operator_demonstration")
            for channel in writer.streams:
                writer.add(channel, 1, {"raw": 0})
            writer.close(success=None)
            return {"status": "OPERATOR_DEMO_CAPTURE_RECORDED", "path": str(writer.path),
                    "can_data_tx_calls": 0, "robot_motion_commanded": False}

        responses = iter(("", "s", "rod placed", "", "f", "rod slipped"))
        with tempfile.TemporaryDirectory() as temp:
            identity = {"schema": "piper_camera_role_identity_v1"}
            raw_root = Path(temp) / "raw"
            session_root = Path(temp) / "sessions"
            outcome_root = Path(temp) / "outcomes"
            result = run_session(raw_root, session_root,
                                 camera_identity=identity, operator="test", episodes=2,
                                 outcome_root=outcome_root,
                                 approved=True, capture_fn=fake_capture,
                                 prompt_fn=lambda _message: next(responses), emit=lambda _message: None)
            self.assertEqual(result["status"], "COMPLETE")
            self.assertEqual(result["captured_episodes"], 2)
            self.assertEqual(result["successful_episodes"], 1)
            self.assertEqual(result["failed_or_partial_episodes"], 1)
            self.assertEqual(result["unlabeled_episodes"], 0)
            manifest = json.loads((Path(result["path"]) / "manifest.json").read_text())
            self.assertFalse(manifest["training_eligible"])
            self.assertFalse(manifest["raw_can_command_supervision"])
            self.assertEqual([row["success"] for row in manifest["episodes"]], [True, False])
            self.assertTrue(all(Path(row["outcome_label_path"]).parent == outcome_root
                                for row in manifest["episodes"]))
            self.assertEqual(manifest["outcome_root"], str(outcome_root))

    def test_capture_session_blocks_before_side_effects_without_approval(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            result = run_session(Path(temp) / "raw", Path(temp) / "sessions",
                                 camera_identity={"schema": "piper_camera_role_identity_v1"},
                                 operator="test", approved=False)
            self.assertEqual(result["status"], "BLOCKED_APPROVAL_REQUIRED")
            self.assertFalse((Path(temp) / "raw").exists())
            self.assertFalse((Path(temp) / "sessions").exists())


if __name__ == "__main__":
    unittest.main()
