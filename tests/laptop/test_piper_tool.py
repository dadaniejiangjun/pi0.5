import json
import pytest
from laptop.robot_client import piper_tool as tool


def test_existing_report_prevents_device_access(tmp_path, monkeypatch):
    output = tmp_path / "report.json"
    output.write_text("original")
    monkeypatch.setattr(tool, "dispatch", lambda args: pytest.fail("device access"))
    with pytest.raises(FileExistsError):
        tool.main(["status", "--output", str(output), "--read-approved"])
    assert output.read_text() == "original"


@pytest.mark.parametrize("degrees,small,large", [("0.1", True, False), ("1", False, True)])
def test_step_routes_exact_scope(tmp_path, monkeypatch, degrees, small, large):
    calls = []
    monkeypatch.setattr(tool, "step_j1", lambda **kwargs: calls.append(kwargs) or
                        {"status": "SINGLE_STEP_FEEDBACK_MATCH"})
    assert tool.main(["step-j1", "--degrees", degrees, "--output", str(tmp_path / "r.json"),
                      "--physical-estop-ready", "--low-speed-confirmed",
                      "--motion-envelope-cleared"]) == 0
    assert calls[0]["commissioning_j1_plus_0_1deg"] is small
    assert calls[0]["commissioning_j1_plus_1deg"] is large
    assert all(calls[0][name] for name in (
        "physical_estop_ready", "low_speed_confirmed", "motion_envelope_cleared"))


def test_read_approval_is_not_implicit(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(tool, "feedback_probe", lambda duration, approved:
                        calls.append((duration, approved)) or {"status": "BLOCKED_APPROVAL_REQUIRED"})
    output = tmp_path / "r.json"
    assert tool.main(["status", "--output", str(output)]) == 1
    assert calls == [(3.0, False)]
    assert json.loads(output.read_text())["operation"] == "status"


def test_failure_saved_without_retry(tmp_path, monkeypatch):
    calls = []
    def fail(args):
        calls.append(args)
        raise RuntimeError("delivery unknown")
    monkeypatch.setattr(tool, "dispatch", fail)
    output = tmp_path / "r.json"
    assert tool.main(["status", "--output", str(output)]) == 1
    assert len(calls) == 1
    assert json.loads(output.read_text())["automatic_retry"] is False


def test_unsupported_angle_rejected_before_operation(tmp_path, monkeypatch):
    monkeypatch.setattr(tool, "dispatch", lambda args: pytest.fail("device access"))
    with pytest.raises(SystemExit):
        tool.main(["step-j1", "--degrees", "2", "--output", str(tmp_path / "r.json")])


@pytest.mark.parametrize("opening", [1, 5])
def test_gripper_routes_exact_opening_and_separate_audit(tmp_path, monkeypatch, opening):
    calls = []
    monkeypatch.setattr(tool, "step_gripper", lambda **kwargs: calls.append(kwargs) or
                        {"status": "GRIPPER_FEEDBACK_MATCH"})
    output = tmp_path / "r.json"
    assert tool.main(["step-gripper", "--opening-mm", str(opening), "--output", str(output),
                      "--physical-estop-ready", "--low-speed-confirmed",
                      "--motion-envelope-cleared"]) == 0
    assert calls[0]["opening_mm"] == opening
    assert calls[0]["log_path"] == tmp_path / "r.json.can.jsonl"
