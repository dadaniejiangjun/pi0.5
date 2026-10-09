from laptop.diagnostics import camera_stream_smoke


def test_stream_smoke_never_claims_roles_or_robot_access(monkeypatch):
    calls = []

    def fake_probe(*args, **kwargs):
        calls.append(kwargs["backend_name"])
        status = "OPEN_FAILED" if len(calls) == 1 else "CONCURRENT_STREAM_PASS"
        return {
            "status": status,
            "cameras": {"first": {"frames_ok": 10}, "second": {"frames_ok": 9}},
            "robot_accessed": False,
            "video_saved": False,
        }

    monkeypatch.setattr(
        camera_stream_smoke,
        "probe_supervised",
        fake_probe,
    )

    report = camera_stream_smoke.run()

    assert report["status"] == "CONCURRENT_STREAM_PASS"
    assert calls == ["DSHOW", "MSMF"]
    assert len(report["backend_attempts"]) == 2
    assert report["camera_roles_verified"] is False
    assert report["video_saved"] is False
    assert report["can_opened"] is False
    assert report["robot_command_sent"] is False


def test_stream_smoke_rejects_near_black_frames_even_when_reads_succeed(monkeypatch):
    calls = []

    def fake_probe(*args, **kwargs):
        calls.append(kwargs["backend_name"])
        if len(calls) == 1:
            return {
                "status": "CONCURRENT_STREAM_LOW_INFORMATION",
                "image_content_status": "LOW_INFORMATION_NEAR_BLACK",
                "cameras": {"base": {"frames_ok": 5}, "wrist": {"frames_ok": 5}},
                "robot_accessed": False,
                "video_saved": False,
            }
        return {"status": "OPEN_FAILED", "robot_accessed": False, "video_saved": False}

    monkeypatch.setattr(camera_stream_smoke, "probe_supervised", fake_probe)

    report = camera_stream_smoke.run()

    assert calls == ["DSHOW", "MSMF", "ANY"]
    assert report["status"] == "CONCURRENT_STREAM_LOW_INFORMATION"
    assert report["image_content_status"] == "LOW_INFORMATION_NEAR_BLACK"
    assert report["camera_usable_for_observation"] is False
    assert report["camera_roles_verified"] is False
    assert report["can_opened"] is False
    assert report["robot_command_sent"] is False
