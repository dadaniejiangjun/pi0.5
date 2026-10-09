import struct
import time

import pytest

from laptop.robot_client.supervised_enable_j6 import execute
from laptop.transport.can_transport import CanFrame
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.mode_initialization import J6EnableFrame


class Adapter:
    short_usb_packets = 0

    def __init__(self, *, enabled=False, fail=False, stale=False, mode=1, nonzero=False):
        self.enabled = enabled
        self.fail = fail
        self.stale = stale
        self.mode = mode
        self.nonzero = nonzero
        self.queue = []
        self.sent = []
        self.opened = False
        self.closed = False

    def open_read_only(self):
        self.opened = True

    def receive_frame(self, timeout_ms):
        return self.queue.pop(0) if self.queue else None

    def send_j6_enable_frame(self, frame):
        self.sent.append(frame)
        if self.fail:
            raise OSError("unknown USB delivery")
        self.enabled = True
        return True

    def close(self):
        self.closed = True


def run(monkeypatch, adapter, **flags):
    clock = [20_000_000_000]
    monkeypatch.setattr(time, "perf_counter_ns", lambda: clock[0])
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)

    def read_for(bridge):
        clock[0] += 600_000_000
        if adapter.stale and adapter.sent:
            return
        for index in range(61):
            stamp = clock[0] - 361_000_000 + index * 6_000_000
            payloads = {0x2A1: bytes((adapter.mode, 0, 1, 2, 0, 0, 0, 0)),
                        0x2A5: struct.pack(">ii", 10000 if adapter.nonzero else 0, 0),
                        0x2A6: bytes(8), 0x2A7: bytes(8)}
            payloads.update({can_id: bytes((0, 0, 0, 0, 0,
                                          64 if can_id != 0x266 or adapter.enabled else 0, 0, 0))
                             for can_id in range(0x261, 0x267)})
            adapter.queue.extend(CanFrame(can_id, data, False, False, stamp)
                                 for can_id, data in payloads.items())
        while adapter.queue:
            bridge.receive_frame(timeout_ms=1)

    options = dict(enable_j6_once=True, operator_authorized_enable=True,
                   zero_pose_supported=True, physical_estop_ready=True,
                   motion_envelope_cleared=True)
    options.update(flags)
    return execute(transport=transport, read_for=read_for, **options)


def test_default_is_inert():
    assert execute()["can_opened"] is False


def test_j6_enable_is_one_exact_frame_and_all_six_are_read_back(monkeypatch):
    adapter = Adapter()
    result = run(monkeypatch, adapter)
    assert result["status"] == "ALL_SIX_ENABLED_CONFIRMED"
    assert len(adapter.sent) == 1
    assert adapter.sent[0].arbitration_id == 0x471
    assert adapter.sent[0].data == bytes((6, 2, 0, 0, 0, 0, 0, 0))
    assert result["final_feedback"]["motor_enabled"] == [True] * 6
    assert result["joint_delta_raw_0p001deg"] == [0] * 6
    assert adapter.closed


@pytest.mark.parametrize("kwargs,expected", [({"mode": 2}, 0), ({"nonzero": True}, 0),
                                           ({"fail": True}, 1), ({"stale": True}, 1)])
def test_bad_mode_pose_or_unknown_delivery_stops(monkeypatch, kwargs, expected):
    adapter = Adapter(**kwargs)
    result = run(monkeypatch, adapter)
    assert result["status"] == "ENABLE_STOPPED_NO_RETRY"
    assert len(adapter.sent) == expected
    assert not result["automatic_retry"]


def test_already_enabled_has_no_write(monkeypatch):
    adapter = Adapter(enabled=True)
    assert run(monkeypatch, adapter)["status"] == "ALREADY_ALL_ENABLED_NO_WRITES"
    assert adapter.sent == []


def test_no_site_confirmation_does_not_open(monkeypatch):
    adapter = Adapter()
    result = run(monkeypatch, adapter, zero_pose_supported=False)
    assert result["can_opened"] is False
    assert not adapter.opened


def test_enable_bytes_match_official_sdk():
    pytest.importorskip("piper_sdk")
    import can
    from piper_sdk.piper_msgs.msg_v2 import PiperMessage, ArmMsgType, ArmMsgMotorEnableDisableConfig
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgMotorEnableDisableConfig
    msg.arm_motor_enable = ArmMsgMotorEnableDisableConfig(6, 2)
    frame = can.Message()
    C_PiperParserV2().EncodeMessage(msg, frame)
    assert frame.arbitration_id == J6EnableFrame().arbitration_id
    assert bytes(frame.data) == J6EnableFrame().data
