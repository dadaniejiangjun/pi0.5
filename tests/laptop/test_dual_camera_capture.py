import json
from pathlib import Path
import numpy as np
import pytest
from laptop.camera.dual_camera import capture, resolve_roles

DEVICES = [{"Name": "ACER HD User Facing", "Index": 0, "DevicePath": "acer-usb"},
           {"Name": "Dabai DC1", "Index": 1, "DevicePath": "dabai-usb"},
           {"Name": "Camera (NVIDIA Broadcast)", "Index": 2}]


def fake_probe(base, wrist, duration, video_dir, *, backend_name, on_frame):
    assert (base, wrist, backend_name) == (0, 1, "DSHOW")
    for role in ("base", "wrist"):
        for index in range(3):
            image = np.full((12, 16, 3), 30+index, dtype=np.uint8)
            on_frame(role, image, 1_000_000_000+index*50_000_000, index)
    return {"status": "CONCURRENT_STREAM_PASS", "frame_timestamps_ns": None}


def test_names_resolve_changed_indices_without_virtual_fallback():
    moved = [dict(d, Index=5 if d["Name"].startswith("ACER") else d["Index"]) for d in DEVICES]
    assert resolve_roles(moved)["base"]["opencv_index"] == 5
    with pytest.raises(ValueError):
        resolve_roles(moved, base_index=0)
    with pytest.raises(ValueError):
        resolve_roles([DEVICES[1], DEVICES[2]])


def test_frames_keep_source_times_and_sample_rate(tmp_path):
    calls = []
    result = capture(tmp_path / "session", mode="frames", image_fps=10,
                     devices=DEVICES, probe_function=fake_probe,
                     on_frame=lambda role, frame, ns, index: calls.append((role, ns, index)))
    assert result["status"] == "CAPTURE_PASS"
    assert result["saved_images"] == {"base": 2, "wrist": 2}
    rows = [json.loads(line) for line in (tmp_path / "session/base_timestamps.jsonl").read_text().splitlines()]
    assert [r["received_monotonic_ns"] for r in rows] == [1_000_000_000, 1_050_000_000, 1_100_000_000]
    assert rows[1]["image_path"] is None
    assert (tmp_path / "session" / rows[2]["image_path"]).exists()
    assert len(calls) == 6
    assert not result["robot_accessed"]


def test_existing_session_refused_before_probe(tmp_path):
    directory = tmp_path / "existing"
    directory.mkdir()
    with pytest.raises(FileExistsError):
        capture(directory, devices=DEVICES, probe_function=lambda *a, **k: pytest.fail("opened cameras"))


def test_snapshot_last_frame_and_durable_report(tmp_path):
    import cv2
    out = tmp_path / "session"
    report = capture(out, mode="snapshot", devices=DEVICES, probe_function=fake_probe)
    assert report["status"] == "CAPTURE_PASS"
    assert cv2.imread(str(out / "base_snapshot.jpg")).shape == (12, 16, 3)
    assert json.loads((out / "report.json").read_text())["status"] == "CAPTURE_PASS"


def test_callback_failure_preserves_failed_report(tmp_path):
    out = tmp_path / "session"
    def fail(*args):
        raise RuntimeError("consumer failed")
    report = capture(out, devices=DEVICES, probe_function=fake_probe, on_frame=fail)
    assert report["status"] == "CAPTURE_FAILED"
    assert "consumer failed" in report["reason"]
    assert (out / "base_timestamps.jsonl").exists()
