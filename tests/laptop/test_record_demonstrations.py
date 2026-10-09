import json
from pathlib import Path
from unittest.mock import patch

from laptop.recorder.record_demonstrations import build_parser, camera_identity, main
from laptop.recorder.capture_session import run_session
from laptop.recorder.episode import EpisodeWriter


def test_identity_resolves_current_indices_and_refuses_virtual():
    devices = [{"Index": 3, "Name": "ACER HD User Facing", "DevicePath": "physical-base"},
               {"Index": 4, "Name": "Dabai DC1", "DevicePath": "physical-wrist"}]
    identity = camera_identity(devices)
    assert identity["roles"]["base"]["opencv_index"] == 3
    devices[0]["DevicePath"] = None
    import pytest
    with pytest.raises(ValueError):
        camera_identity(devices)


def test_defaults_and_inert_without_record():
    args = build_parser().parse_args([])
    assert (args.episodes, args.duration, args.lead_in) == (1, 20, 5)
    with patch("laptop.recorder.record_demonstrations.inventory", side_effect=AssertionError):
        assert main([]) == 2


def test_task_is_in_immutable_raw_inventory(tmp_path):
    writer = EpisodeWriter(tmp_path, task="Place the cube in the tray.")
    writer.close()
    assert (writer.path / "instruction.txt").read_text().strip() == "Place the cube in the tray."
    metadata = json.loads((writer.path / "metadata.json").read_text())
    assert metadata["task_id"] == "custom_task"
    manifest = json.loads((writer.path / "manifest.json").read_text())
    assert "instruction.txt" in manifest["files"]


def test_session_propagates_task_and_reindexed_cameras_stops_partial(tmp_path):
    calls = []
    def capture(raw_root, **kwargs):
        calls.append(kwargs)
        return {"status": "PARTIAL", "can_data_tx_calls": 0,
                "robot_motion_commanded": False, "integrity": {"status": "INCOMPLETE"}}
    identity = {"schema": "piper_camera_role_identity_v1", "roles": {
        "base": {"opencv_index": 3}, "wrist": {"opencv_index": 4}}}
    report = run_session(tmp_path / "raw", tmp_path / "sessions",
                         outcome_root=tmp_path / "outcomes", camera_identity=identity,
                         operator="test", task="Custom task", episodes=5, approved=True,
                         capture_fn=capture, prompt_fn=lambda _: "", emit=lambda _: None)
    assert len(calls) == 1
    assert calls[0]["base_index"] == 3 and calls[0]["wrist_index"] == 4
    assert calls[0]["task"] == "Custom task"
    assert report["status"] == "PARTIAL_CAPTURE_STOPPED"
    assert report["episodes"][0]["capture_quality"]["status"] == "INCOMPLETE"
