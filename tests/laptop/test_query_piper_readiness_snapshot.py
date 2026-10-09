from __future__ import annotations

from laptop.diagnostics import (
    query_all_joint_parameters,
    query_piper_firmware,
    query_piper_gripper_parameters,
)
from laptop.diagnostics.query_piper_readiness_snapshot import run


def _firmware(status="FIRMWARE_VERSION_READ", tx=1):
    return {
        "status": status,
        "safety": {
            "can_data_tx_calls": tx,
            "command_tx_enabled": False,
            "robot_motion_commanded": False,
            "robot_sdk_interface_constructed": False,
            "only_documented_firmware_query_sent": tx == 1,
            "mode_enable_motion_gripper_or_parameter_set_frames_sent": False,
        },
    }


def _joints(status="ALL_SIX_JOINTS_READ", tx=12):
    return {
        "status": status,
        "query_protocol": {"request_frames_sent": tx},
        "safety": {
            "can_data_tx_calls": tx,
            "command_tx_enabled": False,
            "robot_motion_commanded": False,
            "robot_sdk_interface_constructed": False,
            "only_documented_0x472_read_requests_sent": True,
            "mode_enable_reset_joint_target_gripper_or_setter_frames_sent": False,
        },
    }


def _gripper(status="GRIPPER_PARAMETERS_READ", tx=1):
    return {
        "status": status,
        "safety": {
            "can_data_tx_calls": tx,
            "query_frames_sent": tx,
            "command_tx_enabled": False,
            "robot_motion_commanded": False,
            "robot_sdk_interface_constructed": False,
            "only_documented_0x477_read_query_sent": tx <= 1,
            "mode_enable_reset_motion_or_gripper_frames_sent": False,
        },
    }


def test_snapshot_requires_explicit_approval_and_sends_nothing(monkeypatch):
    calls = []
    monkeypatch.setattr(query_piper_firmware, "query", lambda **kwargs: calls.append("firmware"))
    try:
        run(approved=False)
    except PermissionError as exc:
        assert "explicit approval" in str(exc)
    else:
        raise AssertionError("getter queries must require explicit approval")
    assert calls == []


def test_snapshot_completes_only_the_fixed_14_getter_sequence(monkeypatch):
    calls = []

    def firmware(**kwargs):
        calls.append(("firmware", kwargs))
        return _firmware()

    def joints(**kwargs):
        calls.append(("joints", kwargs))
        return _joints()

    def gripper(**kwargs):
        calls.append(("gripper", kwargs))
        return _gripper()

    monkeypatch.setattr(query_piper_firmware, "query", firmware)
    monkeypatch.setattr(query_all_joint_parameters, "query", joints)
    monkeypatch.setattr(query_piper_gripper_parameters, "query", gripper)

    report = run(approved=True)

    assert report["status"] == "ALL_GETTER_READS_COMPLETE"
    assert [name for name, _ in calls] == ["firmware", "joints", "gripper"]
    assert all(kwargs == {"approved": True} for _, kwargs in calls)
    assert report["query_budget"] == {
        "maximum_getter_frames": 14,
        "can_data_tx_calls": 14,
        "tx_count_known": True,
        "retries": 0,
        "later_stages_skipped_after_failure": False,
    }
    assert report["safety"]["command_tx_enabled"] is False
    assert report["safety"]["safe_hold_verified"] is False


def test_snapshot_stops_after_incomplete_stage(monkeypatch):
    calls = []
    monkeypatch.setattr(
        query_piper_firmware, "query",
        lambda **kwargs: (calls.append("firmware") or _firmware()),
    )
    monkeypatch.setattr(
        query_all_joint_parameters, "query",
        lambda **kwargs: (calls.append("joints") or _joints(status="INCOMPLETE_READ", tx=4)),
    )
    monkeypatch.setattr(
        query_piper_gripper_parameters, "query",
        lambda **kwargs: (calls.append("gripper") or _gripper()),
    )

    report = run(approved=True)

    assert report["status"] == "PARTIAL_GETTER_READS"
    assert report["failed_stage"] == "joints"
    assert calls == ["firmware", "joints"]
    assert report["query_budget"]["can_data_tx_calls"] == 5
    assert report["query_budget"]["tx_count_known"] is True
    assert report["query_budget"]["later_stages_skipped_after_failure"] is True


def test_snapshot_stops_and_marks_delivery_unknown_after_stage_exception(monkeypatch):
    calls = []

    def firmware(**kwargs):
        calls.append("firmware")
        raise TimeoutError("USB read timed out")

    monkeypatch.setattr(query_piper_firmware, "query", firmware)
    monkeypatch.setattr(
        query_all_joint_parameters, "query",
        lambda **kwargs: calls.append("joints"),
    )
    monkeypatch.setattr(
        query_piper_gripper_parameters, "query",
        lambda **kwargs: calls.append("gripper"),
    )

    report = run(approved=True)

    assert report["status"] == "PARTIAL_GETTER_READS"
    assert report["failed_stage"] == "firmware"
    assert report["query_budget"]["can_data_tx_calls"] is None
    assert report["query_budget"]["tx_count_known"] is False
    assert report["stages"]["firmware"]["safety"]["can_data_tx_calls"] == "unknown_after_exception"
    assert "No retry was attempted" in report["failure"]["delivery_note"]
    assert calls == ["firmware"]


def test_snapshot_rejects_stage_report_that_breaks_getter_only_invariant(monkeypatch):
    calls = []
    unsafe = _firmware()
    unsafe["safety"]["command_tx_enabled"] = True
    monkeypatch.setattr(
        query_piper_firmware, "query",
        lambda **kwargs: (calls.append("firmware") or unsafe),
    )
    monkeypatch.setattr(query_all_joint_parameters, "query", lambda **kwargs: calls.append("joints"))

    report = run(approved=True)

    assert report["status"] == "PARTIAL_GETTER_READS"
    assert report["failed_stage"] == "firmware"
    assert report["query_budget"]["tx_count_known"] is False
    assert "violates command_tx_enabled" in report["failure"]["error"]
    assert calls == ["firmware"]
