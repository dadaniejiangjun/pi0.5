from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from laptop.diagnostics.camera_identity import bind_camera_roles
from laptop.diagnostics.probe_dual_cameras import probe_supervised
from laptop.recorder.episode import EpisodeWriter
from laptop.recorder.label_episode import label_episode, verify_label
from laptop.recorder.passive_capture import capture_supervised


IDENTITY = {
    "schema": "piper_camera_role_identity_v1",
    "roles": {"base": {"opencv_index": 0}, "wrist": {"opencv_index": 1}},
}


class CaptureSupervisionTest(unittest.TestCase):
    def test_identity_requires_present_devices_and_visual_role_confirmation(self) -> None:
        inventory = [
            {"FriendlyName": "Laptop", "InstanceId": "BASE-ID", "Status": "OK"},
            {"FriendlyName": "Dabai", "InstanceId": "WRIST-ID", "Status": "OK"},
        ]
        bound = bind_camera_roles(0, 1, base_device_id="BASE-ID",
                                  wrist_device_id="WRIST-ID",
                                  roles_visually_confirmed=True, inventory=inventory)
        self.assertEqual(bound["roles"]["wrist"]["windows_instance_id"], "WRIST-ID")
        self.assertFalse(bound["index_to_instance_id_automatically_proven"])
        with self.assertRaisesRegex(ValueError, "visual role"):
            bind_camera_roles(0, 1, base_device_id="BASE-ID",
                              wrist_device_id="WRIST-ID",
                              roles_visually_confirmed=False, inventory=inventory)
        with self.assertRaisesRegex(ValueError, "not present"):
            bind_camera_roles(0, 1, base_device_id="BASE-ID",
                              wrist_device_id="MISSING",
                              roles_visually_confirmed=True, inventory=inventory)

    def test_camera_timeout_reports_no_verified_video(self) -> None:
        with patch("laptop.diagnostics.bounded_process.run_bounded", return_value={
            "status": "TIMEOUT_TERMINATED", "result": None, "error": "read blocked",
        }):
            report = probe_supervised(0, 1, 10)
        self.assertEqual(report["status"], "TIMEOUT_TERMINATED")
        self.assertFalse(report["video_saved"])
        self.assertFalse(report["completion_verified"])

    def test_capture_timeout_writes_marker_outside_raw_episode(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with patch("laptop.diagnostics.bounded_process.run_bounded", return_value={
                "status": "TIMEOUT_TERMINATED", "result": None, "error": "driver blocked",
            }):
                report = capture_supervised(root, camera_identity=IDENTITY,
                                            duration_s=10, approved=True)
            marker = json.loads(Path(report["completion_marker"]).read_text(encoding="utf-8"))
            self.assertEqual(marker["supervisor_status"], "TIMEOUT_TERMINATED")
            self.assertFalse(marker["complete"])
            self.assertIsNone(marker["manifest_sha256"])
            self.assertFalse(Path(report["path"]).exists())
            self.assertFalse(report["training_eligible"])

    def test_outcome_label_is_separate_hash_bound_and_nonoverwriting(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            writer = EpisodeWriter(root / "raw", episode_id="episode001", source="manual_teach")
            writer.close(success=None)
            raw = writer.path
            before = hashlib.sha256((raw / "manifest.json").read_bytes()).hexdigest()
            report = label_episode(raw, root / "outcomes", success=False,
                                   notes="Gripper did not hold object", operator="lab-operator")
            self.assertEqual(verify_label(raw, Path(report["path"]))["status"], "OUTCOME_LABEL_BOUND")
            self.assertEqual(hashlib.sha256((raw / "manifest.json").read_bytes()).hexdigest(), before)
            self.assertFalse(report["success"])
            with self.assertRaises(FileExistsError):
                label_episode(raw, root / "outcomes", success=True,
                              notes="changed", operator="lab-operator")


if __name__ == "__main__":
    unittest.main()
