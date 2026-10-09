import struct
import time
import pytest
from laptop.transport.gripper_commissioning import OemGripperTestFrame
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


def test_exact_oem_scope_and_expiry():
    now=time.perf_counter_ns()
    cmd=OemGripperTestFrame(now,now)
    cmd.validate(now)
    assert cmd.frame.data==struct.pack('>iHBB',54015,0,1,0)
    with pytest.raises(ValueError):OemGripperTestFrame(now,now,70000).validate(now)
    with pytest.raises(PermissionError):cmd.validate(now+5_000_000_001)
    with pytest.raises(PermissionError):cmd.validate(now+120_000_001)


class Adapter:
    def __init__(self,ok=True):self.frames=[];self.ok=ok
    def open_read_only(self):pass
    def close(self):pass
    def send_position_frame(self,frame):self.frames.append(frame);return self.ok


def test_cadence_and_delivery_fault_are_fail_closed():
    a=Adapter();t=GsUsbPiperCanTransport(adapter_factory=lambda:a)
    t.claim('safety-bridge');t.start_read_only('safety-bridge')
    now=time.perf_counter_ns();cmd=OemGripperTestFrame(now,now)
    t.send_oem_gripper_test_frame('safety-bridge',cmd)
    with pytest.raises(PermissionError):t.send_oem_gripper_test_frame('safety-bridge',cmd)
    assert len(a.frames)==1
    t.stop_read_only('safety-bridge')
    a=Adapter(False);t=GsUsbPiperCanTransport(adapter_factory=lambda:a)
    t.claim('safety-bridge');t.start_read_only('safety-bridge')
    now=time.perf_counter_ns()
    with pytest.raises(OSError):t.send_oem_gripper_test_frame('safety-bridge',OemGripperTestFrame(now,now))
    assert t.position_delivery_unknown and not t.connected
