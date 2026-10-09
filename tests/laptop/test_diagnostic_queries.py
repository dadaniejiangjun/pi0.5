from __future__ import annotations

import pytest
import time
from pathlib import Path

from laptop.diagnostics.can_diagnostic_session import (
    CanDiagnosticSession,
    DiagnosticFeedbackPreflightError,
)
from laptop.runtime.config import RuntimeConfig
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, SafetyLimits
from laptop.transport.can_transport import CanFrame, FakeCanTransport
from laptop.transport.diagnostic_queries import (
    DiagnosticCanQuery,
    firmware_version_query,
    gripper_parameters_query,
    joint_limit_query,
)


NOW = 9_000_000_000
FEEDBACK = (0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8)


def limits() -> SafetyLimits:
    return SafetyLimits(
        joint_min_rad=(-3.2,) * 6,
        joint_max_rad=(3.2,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.10,
        max_gripper_delta_m=0.01,
    )


def diagnostic_bridge(*, enabled: bool = True):
    transport = FakeCanTransport(allow_recorded_diagnostic_queries=True)
    bridge = SafetyBridge(
        mode=RuntimeMode.DIAGNOSTIC_QUERY,
        transport=transport,
        limits=limits(),
        diagnostic_query_tx_enabled=enabled,
    )
    bridge.start_read_only()
    for can_id in FEEDBACK:
        payload = bytearray(8)
        if can_id == 0x2A1:
            payload[0] = 0x01
            payload[1] = 0x00
            payload[2] = 0x01
        transport.enqueue_frame(CanFrame(
            arbitration_id=can_id,
            data=bytes(payload),
            is_extended_id=False,
            is_error_frame=False,
            received_monotonic_ns=NOW - 10_000_000,
        ))
    for _ in FEEDBACK:
        assert bridge.receive_frame() is not None
    return bridge, transport


@pytest.mark.parametrize(
    "query,request_id,payload,response_id,name",
    [
        (firmware_version_query(), 0x4AF, "0100000000000000", 0x4AF, "firmware_version"),
        (joint_limit_query(3), 0x472, "0301000000000000", 0x473, "joint_angle_speed_limits"),
        (joint_limit_query(6, acceleration=True), 0x472, "0602000000000000", 0x47C, "joint_acceleration_limit"),
        (gripper_parameters_query(), 0x477, "0400000000000000", 0x47E, "gripper_teaching_parameters"),
    ],
)
def test_documented_getters_have_exact_allowlisted_contract(query, request_id, payload,
                                                             response_id, name):
    assert query.request_can_id == request_id
    assert query.payload.hex() == payload
    assert query.expected_response_can_id == response_id
    assert query.name == name
    assert len(query.payload_sha256) == 64


@pytest.mark.parametrize(
    "can_id,payload",
    [
        (0x155, bytes(8)),
        (0x159, bytes((0, 0, 0, 0, 0, 0, 1, 0))),
        (0x471, bytes((7, 2, 0, 0, 0, 0, 0, 0))),
        (0x472, bytes((1, 1, 0, 0, 0, 0, 0, 1))),
        (0x472, bytes((7, 1, 0, 0, 0, 0, 0, 0))),
        (0x477, bytes((4, 1, 0, 0, 0, 0, 0, 0))),
        (0x477, bytes((1, 0, 0, 0, 0, 0, 0, 0))),
    ],
)
def test_motion_setter_and_unreviewed_queries_cannot_be_constructed(can_id, payload):
    with pytest.raises(ValueError, match="not an allowlisted"):
        DiagnosticCanQuery(can_id, payload)


def test_joint_query_factory_rejects_boolean_or_out_of_range_joint():
    for value in (True, 0, 7, "2"):
        with pytest.raises(ValueError):
            joint_limit_query(value)


def test_diagnostic_config_is_separate_from_policy_reject_only():
    diagnostic = RuntimeConfig.load(__import__("pathlib").Path("configs/runtime/diagnostic_query.json"))
    reject_only = RuntimeConfig.load(__import__("pathlib").Path("configs/runtime/policy_reject_only.json"))
    assert diagnostic.mode is RuntimeMode.DIAGNOSTIC_QUERY
    assert diagnostic.can_tx_enabled is True
    assert diagnostic.command_tx_enabled is False
    assert reject_only.mode is RuntimeMode.POLICY_REJECT_ONLY
    assert reject_only.can_tx_enabled is False
    assert reject_only.command_tx_enabled is False


def test_query_only_mode_requires_explicit_enable():
    with pytest.raises(PermissionError, match="explicit runtime gate"):
        SafetyBridge(
            mode=RuntimeMode.DIAGNOSTIC_QUERY,
            transport=FakeCanTransport(),
            limits=limits(),
        )


def test_one_approved_query_passes_through_bridge_with_fresh_feedback():
    bridge, transport = diagnostic_bridge()
    query = firmware_version_query()
    decision = bridge.send_diagnostic_query(
        query, operator_approved=True, now_monotonic_ns=NOW
    )
    event = decision.audit_event()
    assert decision.query_sent is True
    assert decision.transport_attempted is True
    assert decision.hardware_control_allowed is False
    assert decision.command_tx_enabled is False
    assert event["event_type"] == "diagnostic_query_decision"
    assert event["query_sent"] is True
    assert event["request_can_id"] == "0x4AF"
    assert len(event["input_sha256"]) == 64
    assert transport.diagnostic_query_attempts == 1
    assert transport.recorded_diagnostic_queries == [query]
    bridge.close()


@pytest.mark.parametrize(
    "approved,now,expected_reason",
    [
        (False, NOW, "operator_query_approval_missing"),
        (True, NOW + 300_000_000, "diagnostic_feedback_stale_0x2A1"),
        (True, True, "invalid_current_time"),
    ],
)
def test_query_refuses_without_approval_or_fresh_valid_feedback(approved, now, expected_reason):
    bridge, transport = diagnostic_bridge()
    decision = bridge.send_diagnostic_query(
        firmware_version_query(), operator_approved=approved, now_monotonic_ns=now
    )
    assert decision.query_sent is False
    assert decision.transport_attempted is False
    assert expected_reason in decision.reasons
    assert transport.diagnostic_query_attempts == 0


def test_query_refuses_when_any_required_feedback_channel_is_missing():
    transport = FakeCanTransport(allow_recorded_diagnostic_queries=True)
    bridge = SafetyBridge(
        mode=RuntimeMode.DIAGNOSTIC_QUERY,
        transport=transport,
        limits=limits(),
        diagnostic_query_tx_enabled=True,
    )
    bridge.start_read_only()
    decision = bridge.send_diagnostic_query(
        firmware_version_query(), operator_approved=True, now_monotonic_ns=NOW
    )
    assert decision.query_sent is False
    assert not decision.transport_attempted
    assert "diagnostic_feedback_missing_0x2A8" in decision.reasons
    assert transport.diagnostic_query_attempts == 0


def test_query_refuses_if_arm_status_reports_error():
    bridge, transport = diagnostic_bridge()
    # Refresh the status frame with a non-zero official arm-status field.
    transport.enqueue_frame(CanFrame(
        arbitration_id=0x2A1,
        data=bytes((1, 7, 1, 0, 0, 0, 0, 0)),
        is_extended_id=False,
        is_error_frame=False,
        received_monotonic_ns=NOW - 1_000_000,
    ))
    bridge.receive_frame()
    decision = bridge.send_diagnostic_query(
        firmware_version_query(), operator_approved=True, now_monotonic_ns=NOW
    )
    assert decision.query_sent is False
    assert "arm_status_reports_error" in decision.reasons
    assert transport.diagnostic_query_attempts == 0


def test_policy_reject_only_never_sends_diagnostic_queries():
    transport = FakeCanTransport(allow_recorded_diagnostic_queries=True)
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=transport,
        limits=limits(),
    )
    decision = bridge.send_diagnostic_query(
        firmware_version_query(), operator_approved=True, now_monotonic_ns=NOW
    )
    assert not decision.query_sent
    assert not decision.transport_attempted
    assert "runtime_mode_forbids_diagnostic_query" in decision.reasons
    assert transport.diagnostic_query_attempts == 0


def test_non_diagnostic_modes_cannot_enable_diagnostic_query_channel():
    with pytest.raises(ValueError, match="cannot be enabled"):
        SafetyBridge(
            mode=RuntimeMode.POLICY_REJECT_ONLY,
            transport=FakeCanTransport(),
            limits=limits(),
            diagnostic_query_tx_enabled=True,
        )


def _fresh_feedback_transport(*, arm_error: bool = False) -> FakeCanTransport:
    transport = FakeCanTransport(allow_recorded_diagnostic_queries=True)
    now_ns = time.perf_counter_ns()
    for can_id in FEEDBACK:
        payload = bytearray(8)
        if can_id == 0x2A1 and arm_error:
            payload[1] = 1
        transport.enqueue_frame(CanFrame(
            arbitration_id=can_id,
            data=bytes(payload),
            is_extended_id=False,
            is_error_frame=False,
            received_monotonic_ns=now_ns,
        ))
    return transport


def test_diagnostic_session_preflights_feedback_and_uses_fake_transport():
    transport = _fresh_feedback_transport()
    with CanDiagnosticSession(
        approved=True,
        preflight_timeout_s=0.05,
        transport=transport,
    ) as session:
        assert set(frame.arbitration_id for frame in session.preflight_frames) == set(FEEDBACK)
        event = session.send_query(firmware_version_query())
        assert event["query_sent"] is True
        assert event["hardware_control_allowed"] is False
        assert transport.recorded_diagnostic_queries == [firmware_version_query()]
    assert transport.owner is None
    assert transport.read_only_active is False


def test_diagnostic_session_closes_on_missing_feedback_without_query_attempt():
    transport = FakeCanTransport(allow_recorded_diagnostic_queries=True)
    with pytest.raises(DiagnosticFeedbackPreflightError, match="no diagnostic query sent"):
        with CanDiagnosticSession(
            approved=True,
            preflight_timeout_s=0.001,
            transport=transport,
        ):
            pytest.fail("session must not open without the required feedback")
    assert transport.diagnostic_query_attempts == 0
    assert transport.owner is None
    assert transport.read_only_active is False


def test_diagnostic_session_closes_on_arm_error_feedback():
    transport = _fresh_feedback_transport(arm_error=True)
    with pytest.raises(DiagnosticFeedbackPreflightError, match="arm status reports an error"):
        with CanDiagnosticSession(
            approved=True,
            preflight_timeout_s=0.05,
            transport=transport,
        ):
            pytest.fail("session must reject arm error feedback")
    assert transport.diagnostic_query_attempts == 0
    assert transport.owner is None


def test_diagnostic_session_enforces_an_explicit_query_budget():
    transport = _fresh_feedback_transport()
    with CanDiagnosticSession(
        approved=True,
        preflight_timeout_s=0.05,
        max_queries=1,
        transport=transport,
    ) as session:
        session.send_query(firmware_version_query())
        with pytest.raises(PermissionError, match="budget exhausted"):
            session.send_query(firmware_version_query())
    assert transport.diagnostic_query_attempts == 1
    assert len(transport.recorded_diagnostic_queries) == 1


def test_diagnostic_query_scripts_do_not_bypass_the_shared_transport():
    root = Path(__file__).resolve().parents[2]
    scripts = (
        "query_piper_firmware.py",
        "query_joint_limits.py",
        "query_all_joint_parameters.py",
        "query_piper_gripper_parameters.py",
    )
    for script in scripts:
        source = (root / "laptop" / "diagnostics" / script).read_text(encoding="utf-8")
        assert "CanDiagnosticSession" in source
        assert "GsUsb.scan" not in source
        assert "device.send(" not in source
        assert "device.read(" not in source
        assert "_ensure_libusb_backend" not in source
