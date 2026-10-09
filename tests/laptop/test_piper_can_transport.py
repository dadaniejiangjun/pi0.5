from __future__ import annotations

from collections import deque
from types import ModuleType, SimpleNamespace
import sys

import pytest

from laptop.transport.can_transport import (
    CanFrame,
    PiperCanTransport,
    TransportOwnershipError,
)
from laptop.transport.diagnostic_queries import firmware_version_query
from laptop.transport.gs_usb_piper_can import CanReceiveMode, GsUsbPiperCanTransport
from laptop.transport import gs_usb_piper_can


class MemoryReceiveAdapter:
    def __init__(self) -> None:
        self.frames = deque()
        self.opened = False
        self.closed = False
        self.short_usb_packets = 0
        self.receive_error: Exception | None = None
        self.query_error: Exception | None = None
        self.query_result = True
        self.queries = []

    def open_read_only(self) -> None:
        self.opened = True

    def receive_frame(self, timeout_ms: int) -> CanFrame | None:
        assert timeout_ms > 0
        if self.receive_error is not None:
            raise self.receive_error
        return self.frames.popleft() if self.frames else None

    def close(self) -> None:
        self.closed = True

    def send_diagnostic_query(self, query) -> bool:
        if self.query_error is not None:
            raise self.query_error
        self.queries.append(query)
        return self.query_result


def sample_frame() -> CanFrame:
    return CanFrame(
        arbitration_id=0x2A1,
        data=bytes(range(8)),
        is_extended_id=False,
        is_error_frame=False,
        received_monotonic_ns=123_456_789,
    )


def test_real_transport_obeys_the_shared_owner_contract_without_hardware():
    adapter = MemoryReceiveAdapter()
    adapter.frames.append(sample_frame())
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    assert isinstance(transport, PiperCanTransport)

    with pytest.raises(TransportOwnershipError):
        transport.start_read_only("not-owner")

    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    assert adapter.opened is True
    assert transport.connected is True
    assert transport.receive_frame("safety-bridge") == sample_frame()
    assert transport.receive_frame("safety-bridge") is None

    with pytest.raises(PermissionError, match="command TX"):
        transport.send("safety-bridge", object())
    assert transport.send_attempts == 1
    assert not hasattr(adapter, "send")

    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")
    assert adapter.closed is True
    assert transport.owner is None
    assert transport.connected is False


def test_receive_failure_disconnects_transport_and_preserves_exclusive_owner():
    adapter = MemoryReceiveAdapter()
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    adapter.receive_error = OSError("device detached")

    with pytest.raises(OSError, match="detached"):
        transport.receive_frame("safety-bridge")

    assert transport.connected is False
    assert transport.disconnect_reason == "usb_disconnect"
    assert transport.owner == "safety-bridge"
    assert adapter.closed is True
    transport.release("safety-bridge")


def test_transport_sends_only_a_validated_diagnostic_query_through_owner():
    adapter = MemoryReceiveAdapter()
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    query = firmware_version_query()

    assert transport.send_diagnostic_query("safety-bridge", query) is True
    assert adapter.queries == [query]
    assert transport.send_attempts == 0
    with pytest.raises(TypeError, match="validated DiagnosticCanQuery"):
        transport.send_diagnostic_query("safety-bridge", object())
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


@pytest.mark.parametrize("failure", [OSError("usb write failed"), False])
def test_query_write_failure_is_delivery_unknown_and_closes_without_retry(failure):
    adapter = MemoryReceiveAdapter()
    if failure is False:
        adapter.query_result = False
    else:
        adapter.query_error = failure
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")

    with pytest.raises(Exception, match="delivery is unknown; do not retry"):
        transport.send_diagnostic_query("safety-bridge", firmware_version_query())

    assert transport.connected is False
    assert transport.disconnect_reason == "diagnostic_query_delivery_unknown"
    assert adapter.closed is True
    assert transport.send_attempts == 0
    transport.release("safety-bridge")


def test_listen_only_transport_refuses_getter_transmission_before_adapter_call():
    adapter = MemoryReceiveAdapter()
    transport = GsUsbPiperCanTransport(
        adapter_factory=lambda: adapter,
        receive_mode=CanReceiveMode.LISTEN_ONLY,
    )
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")

    with pytest.raises(PermissionError, match="unavailable in listen-only"):
        transport.send_diagnostic_query("safety-bridge", firmware_version_query())

    assert adapter.queries == []
    assert transport.send_attempts == 0
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


@pytest.mark.parametrize("timeout_ms", [True, 0, -1, 1001, 1.5])
def test_receive_timeout_must_be_a_bounded_integer(timeout_ms):
    transport = GsUsbPiperCanTransport(adapter_factory=MemoryReceiveAdapter)
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")

    with pytest.raises(ValueError, match="timeout_ms"):
        transport.receive_frame("safety-bridge", timeout_ms)

    assert transport.send_attempts == 0
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


def test_gs_usb_adapter_opens_in_receive_mode_and_never_writes(monkeypatch):
    class RawFrame:
        def __init__(self):
            self.arbitration_id = 0
            self.data = bytearray(8)
            self.can_dlc = 0
            self.is_extended_id = False
            self.is_error_frame = False

    class Device:
        def __init__(self):
            self.gs_usb = SimpleNamespace(idVendor=0x1D50, idProduct=0x606F)
            self.device_flags = 0
            self.start_flags = None
            self.stop_calls = 0

        def set_bitrate(self, bitrate):
            return bitrate == 1_000_000

        def start(self, *, flags):
            self.start_flags = flags

        def read(self, frame, *, timeout_ms):
            assert timeout_ms == 100
            frame.arbitration_id = 0x2A1
            frame.data[:] = bytes(range(8))
            frame.can_dlc = 8
            return True

        def stop(self):
            self.stop_calls += 1

    device = Device()

    class GsUsb:
        @staticmethod
        def scan():
            return [device]

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = RawFrame
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter()
    adapter.open_read_only()
    frame = adapter.receive_frame(100)
    adapter.close()

    assert device.start_flags == 0
    assert frame is not None
    assert frame.arbitration_id == 0x2A1
    assert frame.data == bytes(range(8))
    assert frame.received_monotonic_ns > 0
    assert device.stop_calls == 1
    assert not hasattr(device, "send")


def test_gs_usb_adapter_encodes_only_validated_getter_frame(monkeypatch):
    class RawFrame:
        def __init__(self, *, can_id=0, data=b""):
            self.arbitration_id = can_id
            self.data = bytearray(data)
            self.can_dlc = len(data)
            self.is_extended_id = False
            self.is_error_frame = False

    class Device:
        def __init__(self):
            self.gs_usb = SimpleNamespace(idVendor=0x1D50, idProduct=0x606F)
            self.device_flags = 0
            self.sent = []

        def set_bitrate(self, bitrate):
            return bitrate == 1_000_000

        def start(self, *, flags):
            assert flags == 0

        def send(self, frame):
            self.sent.append(frame)
            return True

        def stop(self):
            pass

    device = Device()

    class GsUsb:
        @staticmethod
        def scan():
            return [device]

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = RawFrame
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter()
    adapter.open_read_only()
    query = firmware_version_query()
    assert adapter.send_diagnostic_query(query) is True
    adapter.close()

    assert len(device.sent) == 1
    assert device.sent[0].arbitration_id == 0x4AF
    assert bytes(device.sent[0].data) == bytes.fromhex("0100000000000000")


def test_gs_usb_adapter_enters_exact_listen_only_mode_and_has_no_getter_tx(monkeypatch):
    listen_only_flag = 0x80

    class RawFrame:
        def __init__(self, *, can_id=0, data=b""):
            self.arbitration_id = can_id
            self.data = bytearray(data)
            self.can_dlc = len(data)
            self.is_extended_id = False
            self.is_error_frame = False

    class Device:
        def __init__(self):
            self.gs_usb = SimpleNamespace(idVendor=0x1D50, idProduct=0x606F)
            self.device_flags = 0
            self.device_capability = SimpleNamespace(feature=listen_only_flag)
            self.start_flags = None
            self.sent = []
            self.stop_calls = 0

        def set_bitrate(self, bitrate):
            return bitrate == 1_000_000

        def start(self, *, flags):
            self.start_flags = flags
            self.device_flags = flags

        def send(self, frame):
            self.sent.append(frame)
            return True

        def stop(self):
            self.stop_calls += 1

    device = Device()

    class GsUsb:
        @staticmethod
        def scan():
            return [device]

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = RawFrame
    constants_module = ModuleType("gs_usb.constants")
    constants_module.GS_CAN_MODE_LISTEN_ONLY = listen_only_flag
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setitem(sys.modules, "gs_usb.constants", constants_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter(CanReceiveMode.LISTEN_ONLY)
    adapter.open_read_only()
    with pytest.raises(PermissionError, match="unavailable in listen-only"):
        adapter.send_diagnostic_query(firmware_version_query())
    adapter.close()

    assert device.start_flags == listen_only_flag
    assert device.sent == []
    assert device.stop_calls == 1


def test_gs_usb_adapter_requires_exactly_one_supported_device(monkeypatch):
    class GsUsb:
        @staticmethod
        def scan():
            return []

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = object
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter()
    with pytest.raises(gs_usb_piper_can.CanAdapterNotUniqueError) as error:
        adapter.open_read_only()

    assert error.value.matching_count == 0


def test_gs_usb_adapter_selects_only_the_exact_usb_bus_and_address(monkeypatch):
    listen_only_flag = 0x80

    class RawFrame:
        def __init__(self):
            self.arbitration_id = 0
            self.data = bytearray(8)
            self.can_dlc = 0
            self.is_extended_id = False
            self.is_error_frame = False

    class Device:
        def __init__(self, bus, address):
            self.gs_usb = SimpleNamespace(idVendor=0x1D50, idProduct=0x606F,
                                          bus=bus, address=address)
            self.device_flags = 0
            self.device_capability = SimpleNamespace(feature=listen_only_flag)
            self.start_flags = None
            self.bitrate_calls = 0
            self.stop_calls = 0

        def set_bitrate(self, bitrate):
            self.bitrate_calls += 1
            return bitrate == 1_000_000

        def start(self, *, flags):
            self.start_flags = flags
            self.device_flags = flags

        def stop(self):
            self.stop_calls += 1

    control = Device(1, 4)
    passive = Device(2, 7)

    class GsUsb:
        @staticmethod
        def scan():
            return [control, passive]

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = RawFrame
    constants_module = ModuleType("gs_usb.constants")
    constants_module.GS_CAN_MODE_LISTEN_ONLY = listen_only_flag
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setitem(sys.modules, "gs_usb.constants", constants_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    listed = gs_usb_piper_can.enumerate_candlelight_usb_adapters()
    assert listed == [
        {"usb_bus": 1, "usb_address": 4, "vid_pid": "1d50:606f"},
        {"usb_bus": 2, "usb_address": 7, "vid_pid": "1d50:606f"},
    ]
    assert all(device.bitrate_calls == 0 and device.start_flags is None
               for device in (control, passive))

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter(
        CanReceiveMode.LISTEN_ONLY, adapter_bus=2, adapter_address=7,
    )
    adapter.open_read_only()
    adapter.close()

    assert control.bitrate_calls == 0
    assert control.start_flags is None
    assert control.stop_calls == 0
    assert passive.bitrate_calls == 1
    assert passive.start_flags == listen_only_flag
    assert passive.stop_calls == 1


def test_usb_adapter_selector_requires_both_identity_fields():
    with pytest.raises(ValueError, match="provided together"):
        GsUsbPiperCanTransport(adapter_bus=1)


def test_gs_usb_adapter_closes_if_receive_start_fails(monkeypatch):
    class Device:
        def __init__(self):
            self.gs_usb = SimpleNamespace(idVendor=0x1D50, idProduct=0x606F)
            self.device_flags = 0
            self.stop_calls = 0

        def set_bitrate(self, bitrate):
            return True

        def start(self, *, flags):
            raise OSError("adapter start failed")

        def stop(self):
            self.stop_calls += 1

    device = Device()

    class GsUsb:
        @staticmethod
        def scan():
            return [device]

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = object
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter()
    with pytest.raises(OSError, match="start failed"):
        adapter.open_read_only()

    assert device.stop_calls == 1
    assert adapter._device is None
    assert adapter._started is False

def test_close_releases_usb_interface(monkeypatch):
    import usb.util
    calls=[]
    usb_device=SimpleNamespace(_ctx=object())
    device=SimpleNamespace(gs_usb=usb_device,stop=lambda:calls.append('stop'))
    adapter=gs_usb_piper_can._GsUsbReceiveAdapter()
    adapter._device=device
    adapter._started=True
    monkeypatch.setattr(usb.util,'dispose_resources',lambda d:calls.append(('dispose',d)))
    adapter.close()
    assert calls==['stop',('dispose',usb_device)]
    adapter.close()
    assert len(calls)==2
