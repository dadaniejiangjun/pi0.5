import struct
import time

import pytest

from laptop.robot_client.supervised_mode_initialization import execute
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import CanFrame
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport, CanInitializationDeliveryUnknown
from laptop.transport.mode_initialization import FEEDBACK_IDS, STAGES, ModeInitializationFrame


class MemoryAdapter:
    short_usb_packets = 0

    def __init__(self, *, nonzero=False, fail_at=0, refuse_standby=False, stale=False,
                 competing=False):
        self.queue = []
        self.sent = []
        self.mode = 2
        self.enabled = True
        self.opened = False
        self.closed = False
        self.nonzero = nonzero
        self.fail_at = fail_at
        self.refuse_standby = refuse_standby
        self.stale = stale
        self.competing = competing

    def open_read_only(self):
        self.opened = True

    def receive_frame(self, timeout_ms):
        return self.queue.pop(0) if self.queue else None

    def send_mode_initialization_frame(self, frame):
        self.sent.append(frame)
        if self.fail_at == len(self.sent):
            raise OSError("USB delivery unknown")
        if frame.stage == "reset":
            self.enabled = False
        elif frame.stage == "standby" and not self.refuse_standby:
            self.mode = 0
        elif frame.stage == "can_joint":
            self.mode = 1
        return True

    def close(self):
        self.closed = True


def run_memory(monkeypatch, adapter, **flags):
    clock = [10_000_000_000]
    monkeypatch.setattr(time, "perf_counter_ns", lambda: clock[0])
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)

    def read_for(bridge):
        clock[0] += 600_000_000
        if adapter.stale and adapter.sent:
            return
        joints = (88160, 0, 0, 0, 0, 0) if adapter.nonzero else (0,) * 6
        for index in range(61):
            stamp = clock[0] - 361_000_000 + index * 6_000_000
            payloads = {0x2A1: bytes((adapter.mode, 0, 1, 0, 0, 0, 0, 0)),
                        0x2A5: struct.pack(">ii", *joints[:2]),
                        0x2A6: struct.pack(">ii", *joints[2:4]),
                        0x2A7: struct.pack(">ii", *joints[4:])}
            payloads.update({can_id: bytes((0, 0, 0, 0, 0, 64 if adapter.enabled else 0, 0, 0))
                             for can_id in range(0x261, 0x267)})
            adapter.queue.extend(CanFrame(can_id, data, False, False, stamp)
                                 for can_id, data in payloads.items())
        if adapter.competing:
            adapter.queue.append(CanFrame(0x155, bytes(8), False, False, clock[0] - 1))
        while adapter.queue:
            bridge.receive_frame(timeout_ms=1)

    attestations = dict(initialize_can_mode=True, operator_authorized_reset=True,
                        zero_pose_supported=True, physical_estop_ready=True,
                        motion_envelope_cleared=True)
    attestations.update(flags)
    return execute(transport=transport, read_for=read_for, **attestations)


def test_default_does_not_open_any_device():
    adapter = MemoryAdapter()
    result = execute(transport=GsUsbPiperCanTransport(adapter_factory=lambda: adapter))
    assert not adapter.opened
    assert result["can_data_tx_attempts"] == 0


@pytest.mark.parametrize("flag", ["operator_authorized_reset", "zero_pose_supported",
                                 "physical_estop_ready", "motion_envelope_cleared"])
def test_missing_physical_confirmation_blocks_before_open(monkeypatch, flag):
    adapter = MemoryAdapter()
    result = run_memory(monkeypatch, adapter, **{flag: False})
    assert result["can_opened"] is False
    assert adapter.sent == []


def test_fixed_sequence_reads_back_can_and_does_not_enable_motors(monkeypatch):
    adapter = MemoryAdapter()
    result = run_memory(monkeypatch, adapter)
    assert result["status"] == "CAN_CTRL_CONFIRMED"
    assert [frame.stage for frame in adapter.sent] == list(STAGES)
    assert result["can_data_tx_attempts"] == 3
    assert result["all_six_enabled"] is False
    assert result["motor_enable_command_sent"] is False
    assert adapter.closed


@pytest.mark.parametrize("fail_at", [1, 2, 3])
def test_unknown_delivery_stops_without_retry(monkeypatch, fail_at):
    adapter = MemoryAdapter(fail_at=fail_at)
    result = run_memory(monkeypatch, adapter)
    assert result["error_type"] == "CanInitializationDeliveryUnknown"
    assert len(adapter.sent) == fail_at
    assert result["automatic_retry"] is False
    assert adapter.closed


@pytest.mark.parametrize("kwargs,expected_writes", [
    ({"nonzero": True}, 0), ({"stale": True}, 1),
    ({"refuse_standby": True}, 2), ({"competing": True}, 0)])
def test_bad_pose_stale_feedback_or_missing_mode_confirmation_stops(monkeypatch, kwargs, expected_writes):
    adapter = MemoryAdapter(**kwargs)
    result = run_memory(monkeypatch, adapter)
    assert result["status"] == "INITIALIZATION_STOPPED_NO_RETRY"
    assert len(adapter.sent) == expected_writes


def test_already_can_does_not_reset(monkeypatch):
    adapter = MemoryAdapter(nonzero=True)
    adapter.mode = 1
    result = run_memory(monkeypatch, adapter)
    assert result["status"] == "ALREADY_CAN_CTRL_NO_WRITES"
    assert adapter.sent == []


def test_initialization_gate_is_not_available_in_shadow():
    with pytest.raises(ValueError):
        SafetyBridge(mode=RuntimeMode.SHADOW, transport=GsUsbPiperCanTransport(),
                     limits=UNVERIFIED_ANALYSIS_LIMITS,
                     supervised_mode_initialization_tx_enabled=True)


def test_transport_requires_fixed_order_and_consumes_reset_once():
    adapter = MemoryAdapter()
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    try:
        with pytest.raises(PermissionError):
            transport.send_mode_initialization_frame("safety-bridge", ModeInitializationFrame("standby"))
        transport.send_mode_initialization_frame("safety-bridge", ModeInitializationFrame("reset"))
        with pytest.raises(PermissionError):
            transport.send_mode_initialization_frame("safety-bridge", ModeInitializationFrame("reset"))
        assert len(adapter.sent) == 1
    finally:
        transport.stop_read_only("safety-bridge")


def test_unknown_reset_delivery_prevents_reopening_the_same_transport():
    adapter = MemoryAdapter(fail_at=1)
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    with pytest.raises(CanInitializationDeliveryUnknown):
        transport.send_mode_initialization_frame("safety-bridge", ModeInitializationFrame("reset"))
    with pytest.raises(ConnectionError):
        transport.start_read_only("safety-bridge")
    assert len(adapter.sent) == 1


def test_frames_match_installed_official_sdk_encoder():
    pytest.importorskip("piper_sdk")
    import can
    from piper_sdk.piper_msgs.msg_v2 import (PiperMessage, ArmMsgType,
                                           ArmMsgMotionCtrl_1, ArmMsgMotionCtrl_2)
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
    parser = C_PiperParserV2()
    for stage in STAGES:
        message = PiperMessage()
        if stage == "reset":
            message.type_ = ArmMsgType.PiperMsgMotionCtrl_1
            message.arm_motion_ctrl_1 = ArmMsgMotionCtrl_1(2, 0, 0)
        else:
            message.type_ = ArmMsgType.PiperMsgMotionCtrl_2
            message.arm_motion_ctrl_2 = ArmMsgMotionCtrl_2(int(stage == "can_joint"), 1, 5, 0)
        encoded = can.Message()
        parser.EncodeMessage(message, encoded)
        frame = ModeInitializationFrame(stage)
        assert encoded.arbitration_id == frame.arbitration_id
        assert bytes(encoded.data) == frame.data
