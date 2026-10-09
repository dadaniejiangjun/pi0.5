import struct
import time
import pytest
from laptop.transport.gripper_commissioning import GripperOpeningStep
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


class Adapter:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.frames = []
    def open_read_only(self):
        pass
    def close(self):
        pass
    def send_position_frame(self, frame):
        self.frames.append(frame)
        return self.accepted


def test_exact_opening_frame_and_one_use():
    adapter = Adapter()
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    now = time.perf_counter_ns()
    command = GripperOpeningStep(0, now, now)
    receipt = transport.send_gripper_opening_once("safety-bridge", command)
    assert receipt.arbitration_id == 0x159
    assert struct.unpack(">iHBB", receipt.data) == (1000, 100, 1, 0)
    with pytest.raises(PermissionError):
        transport.send_gripper_opening_once("safety-bridge", command)
    assert len(adapter.frames) == 1
    transport.stop_read_only("safety-bridge")


def test_delivery_failure_latches_no_retry():
    adapter = Adapter(False)
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    now = time.perf_counter_ns()
    command = GripperOpeningStep(0, now, now)
    with pytest.raises(OSError):
        transport.send_gripper_opening_once("safety-bridge", command)
    assert transport.position_delivery_unknown
    assert not transport.connected
    with pytest.raises(PermissionError):
        transport.send_gripper_opening_once("safety-bridge", command)
    assert len(adapter.frames) == 1


@pytest.mark.parametrize("raw", [-1, 69001, True, 0.1])
def test_invalid_source_rejected(raw):
    now = time.perf_counter_ns()
    with pytest.raises(ValueError):
        GripperOpeningStep(raw, now, now).validate(now)


def test_stale_feedback_rejected():
    now = time.perf_counter_ns()
    with pytest.raises(PermissionError):
        GripperOpeningStep(0, now - 120_000_001, now).validate(now)


def test_five_mm_scope_is_exact_and_bounded():
    now = time.perf_counter_ns()
    step = GripperOpeningStep(0, now, now, 5000, 300)
    step.validate(now)
    assert struct.unpack(">iHBB", step.frame.data) == (5000, 300, 1, 0)
    with pytest.raises(ValueError):
        GripperOpeningStep(66000, now, now, 5000, 300).validate(now)
    with pytest.raises(ValueError):
        GripperOpeningStep(0, now, now, 5000, 100).validate(now)
