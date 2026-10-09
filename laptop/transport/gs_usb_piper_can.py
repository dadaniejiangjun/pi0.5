"""Bridge-owned gs_usb with separate getter, position, and initialization paths."""

from __future__ import annotations

import struct
import sys
import time
from enum import Enum
from pathlib import Path
from typing import Callable, Optional, Protocol

from laptop.control.contracts import CommandProposal
from laptop.transport.can_transport import (
    CanFrame,
    TransportOwnershipError,
)
from laptop.transport.diagnostic_queries import DiagnosticCanQuery
from laptop.transport.mode_initialization import ModeInitializationFrame, J6EnableFrame, STAGES
from laptop.transport.position_commands import (
    J1CommissioningAuthorization,
    J1CommissioningBatch,
    J1CommissioningSnapshot,
    J1CommissioningWriteReceipt,
    POSITION_FRAME_IDS,
    CanTxFrame,
    PositionBatchWriteReceipt,
    PositionCommandBatch,
    PositionFrameWrite,
    SupervisedPositionAuthorization,
    validate_j1_commissioning_execution,
    validate_position_execution_context,
)


class CanAdapterNotUniqueError(RuntimeError):
    def __init__(self, matching_count: int) -> None:
        super().__init__(f"expected exactly one candleLight adapter, found {matching_count}")
        self.matching_count = matching_count


class CanBitrateError(RuntimeError):
    pass


class CanListenOnlyUnsupportedError(RuntimeError):
    pass


class CanReceiveMode(str, Enum):
    NORMAL_ACKING = "NORMAL_ACKING"
    LISTEN_ONLY = "LISTEN_ONLY"


class CanQueryDeliveryUnknown(RuntimeError):
    """A query write failed after the adapter may have accepted the frame."""


class CanInitializationDeliveryUnknown(RuntimeError):
    """A reset/mode write may have arrived; never retry it automatically."""


class CanPositionDeliveryUnknown(RuntimeError):
    """A position batch may be partially delivered; automatic retry is forbidden."""

    def __init__(self, *, approval_id: str, proposal_id: str,
                 confirmed_writes: tuple[PositionFrameWrite, ...],
                 failed_frame: CanTxFrame,
                 failed_at_monotonic_ns: int) -> None:
        super().__init__(
            "PiPER position batch delivery is unknown; stop and verify feedback before any retry"
        )
        self.approval_id = approval_id
        self.proposal_id = proposal_id
        self.confirmed_writes = confirmed_writes
        self.confirmed_frame_count = len(confirmed_writes)
        self.failed_frame_id = failed_frame.arbitration_id
        self.failed_frame_data = failed_frame.data
        self.failed_at_monotonic_ns = failed_at_monotonic_ns


class CanJ1CommissioningDeliveryUnknown(RuntimeError):
    """A fixed J1 commissioning batch may be partially delivered; never retry."""

    def __init__(self, *, approval_id: str,
                 confirmed_writes: tuple[PositionFrameWrite, ...],
                 failed_frame: CanTxFrame,
                 failed_at_monotonic_ns: int) -> None:
        super().__init__(
            "J1 commissioning delivery is unknown; stop and verify read-only feedback before retry"
        )
        self.approval_id = approval_id
        self.confirmed_writes = confirmed_writes
        self.confirmed_frame_count = len(confirmed_writes)
        self.failed_frame_id = failed_frame.arbitration_id
        self.failed_frame_data = failed_frame.data
        self.failed_at_monotonic_ns = failed_at_monotonic_ns


class _ReceiveAdapter(Protocol):
    short_usb_packets: int

    def open_read_only(self) -> None: ...

    def receive_frame(self, timeout_ms: int) -> Optional[CanFrame]: ...

    def send_diagnostic_query(self, query: DiagnosticCanQuery) -> bool: ...

    def send_position_frame(self, frame: CanTxFrame) -> bool: ...

    def close(self) -> None: ...


def ensure_libusb_backend() -> None:
    """Load the Conda-bundled libusb DLL when Windows discovery misses it."""
    from usb.backend import libusb1

    if libusb1.get_backend() is not None:
        return
    if sys.platform == "win32":
        conda_dll = Path(sys.prefix) / "Library" / "bin" / "libusb-1.0.dll"
        if conda_dll.is_file() and libusb1.get_backend(
            find_library=lambda _name: str(conda_dll)
        ) is not None:
            return
    raise RuntimeError(
        "PyUSB could not load libusb; install/provide a compatible libusb-1.0 backend"
    )


def enumerate_candlelight_usb_adapters() -> list[dict[str, int | str]]:
    """List supported USB identities without setting bitrate or starting CAN."""
    ensure_libusb_backend()
    from gs_usb.gs_usb import GsUsb

    return [
        {
            "usb_bus": int(device.gs_usb.bus),
            "usb_address": int(device.gs_usb.address),
            "vid_pid": "1d50:606f",
        }
        for device in GsUsb.scan()
        if (device.gs_usb.idVendor, device.gs_usb.idProduct) == (0x1D50, 0x606F)
    ]


class _GsUsbReceiveAdapter:
    """Thin candleLight adapter with RX and validated getter-only writes."""

    def __init__(
        self,
        receive_mode: CanReceiveMode = CanReceiveMode.NORMAL_ACKING,
        *,
        adapter_bus: int | None = None,
        adapter_address: int | None = None,
    ) -> None:
        if not isinstance(receive_mode, CanReceiveMode):
            raise ValueError("unsupported CAN adapter receive mode")
        if (adapter_bus is None) != (adapter_address is None):
            raise ValueError("adapter_bus and adapter_address must be provided together")
        if adapter_bus is not None and (
            type(adapter_bus) is not int or adapter_bus < 0
            or type(adapter_address) is not int or adapter_address < 1
        ):
            raise ValueError("adapter bus must be >=0 and USB address must be >=1")
        self.receive_mode = receive_mode
        self.adapter_bus = adapter_bus
        self.adapter_address = adapter_address
        self._device = None
        self._frame_type = None
        self._started = False
        self._start_flags = 0
        self.short_usb_packets = 0

    def open_read_only(self) -> None:
        ensure_libusb_backend()
        from gs_usb.gs_usb import GsUsb, GsUsbFrame

        matches = [
            device for device in GsUsb.scan()
            if (device.gs_usb.idVendor, device.gs_usb.idProduct) == (0x1D50, 0x606F)
            and (self.adapter_bus is None or (
                device.gs_usb.bus == self.adapter_bus
                and device.gs_usb.address == self.adapter_address
            ))
        ]
        if len(matches) != 1:
            raise CanAdapterNotUniqueError(len(matches))
        device = matches[0]
        if not device.set_bitrate(1_000_000):
            raise CanBitrateError("candleLight adapter rejected 1 Mbps bitrate")
        self._start_flags = 0
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            from gs_usb.constants import GS_CAN_MODE_LISTEN_ONLY

            capability = device.device_capability
            if not capability.feature & GS_CAN_MODE_LISTEN_ONLY:
                raise CanListenOnlyUnsupportedError(
                    "candleLight adapter does not support listen-only mode"
                )
            self._start_flags = GS_CAN_MODE_LISTEN_ONLY
        self._device = device
        self._frame_type = GsUsbFrame
        self._started = True
        try:
            device.start(flags=self._start_flags)
            if device.device_flags != self._start_flags:
                raise RuntimeError("adapter did not enter the requested receive mode")
        except Exception:
            try:
                self.close()
            except Exception:
                pass
            raise

    def receive_frame(self, timeout_ms: int) -> Optional[CanFrame]:
        if not self._started or self._device is None or self._frame_type is None:
            raise RuntimeError("gs_usb receive adapter is not open")
        raw = self._frame_type()
        try:
            received = self._device.read(raw, timeout_ms=timeout_ms)
        except struct.error:
            self.short_usb_packets += 1
            return None
        if not received:
            return None
        return CanFrame(
            arbitration_id=raw.arbitration_id,
            data=bytes(raw.data[:raw.can_dlc]),
            is_extended_id=bool(raw.is_extended_id),
            is_error_frame=bool(raw.is_error_frame),
            received_monotonic_ns=time.perf_counter_ns(),
        )

    def send_diagnostic_query(self, query: DiagnosticCanQuery) -> bool:
        if not isinstance(query, DiagnosticCanQuery):
            raise TypeError("only a validated DiagnosticCanQuery may be sent")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("diagnostic getter TX is unavailable in listen-only mode")
        if not self._started or self._device is None or self._frame_type is None:
            raise RuntimeError("gs_usb diagnostic query requires an open receive session")
        frame = self._frame_type(
            can_id=query.request_can_id,
            data=query.payload,
        )
        return self._device.send(frame) is True

    def send_position_frame(self, frame: CanTxFrame) -> bool:
        if type(frame) is not CanTxFrame or frame.arbitration_id not in POSITION_FRAME_IDS:
            raise TypeError("only a validated PiPER position command frame may be sent")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("position command TX is unavailable in listen-only mode")
        if not self._started or self._device is None or self._frame_type is None:
            raise RuntimeError("position command requires an open normal-acking receive session")
        raw = self._frame_type(can_id=frame.arbitration_id, data=frame.data)
        return self._device.send(raw) is True

    def send_mode_initialization_frame(self, frame: ModeInitializationFrame) -> bool:
        if type(frame) is not ModeInitializationFrame:
            raise TypeError("a fixed initialization frame is required")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("initialization is unavailable in listen-only mode")
        if not self._started or self._device is None or self._frame_type is None:
            raise RuntimeError("initialization requires an open receive session")
        return self._device.send(self._frame_type(
            can_id=frame.arbitration_id, data=frame.data)) is True

    def send_j6_enable_frame(self, frame: J6EnableFrame) -> bool:
        if type(frame) is not J6EnableFrame:
            raise TypeError("only the fixed J6 enable frame is accepted")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("J6 enable is unavailable in listen-only mode")
        if not self._started or self._device is None or self._frame_type is None:
            raise RuntimeError("J6 enable requires an open receive session")
        return self._device.send(self._frame_type(
            can_id=frame.arbitration_id, data=frame.data)) is True

    def close(self) -> None:
        device, started = self._device, self._started
        self._device = None
        self._frame_type = None
        self._started = False
        if device is not None:
            try:
                if started:
                    device.stop()
            finally:
                # Stop CAN and release the Windows USB interface before a new owner.
                usb_device = getattr(device, "gs_usb", None)
                if usb_device is not None and hasattr(usb_device, "_ctx"):
                    from usb.util import dispose_resources
                    dispose_resources(usb_device)


class GsUsbPiperCanTransport:
    """Exclusive PiPER CAN transport with a narrow, one-use position path.

    ``start_read_only`` opens the uniquely identified candleLight adapter in
    normal receive mode (`flags=0`). ``send`` always rejects command proposals.
    The separate ``send_diagnostic_query`` accepts only exact whitelisted
    firmware/joint/gripper getter requests. Position commands require an exact
    four-frame batch plus a fresh one-use supervised authorization. Mode
    initialization has a separate once-only reset/standby/CAN sequence;
    generic proposal/frame transmission remains rejected.
    """

    def __init__(
        self,
        *,
        adapter_factory: Callable[[], _ReceiveAdapter] | None = None,
        receive_mode: CanReceiveMode = CanReceiveMode.NORMAL_ACKING,
        adapter_bus: int | None = None,
        adapter_address: int | None = None,
    ) -> None:
        if not isinstance(receive_mode, CanReceiveMode):
            raise ValueError("receive_mode must be a CanReceiveMode")
        if (adapter_bus is None) != (adapter_address is None):
            raise ValueError("adapter_bus and adapter_address must be provided together")
        if adapter_bus is not None and (
            type(adapter_bus) is not int or adapter_bus < 0
            or type(adapter_address) is not int or adapter_address < 1
        ):
            raise ValueError("adapter bus must be >=0 and USB address must be >=1")
        if adapter_factory is not None and adapter_bus is not None:
            raise ValueError("adapter selector cannot be combined with adapter_factory")
        self.receive_mode = receive_mode
        self.owner: Optional[str] = None
        self.send_attempts = 0
        self.connected = False
        self.disconnect_reason: Optional[str] = None
        self.short_usb_packets = 0
        self.position_send_attempts = 0
        self.position_frames_accepted = 0
        self.position_delivery_unknown = False
        self.j1_commissioning_send_attempts = 0
        self.j1_commissioning_frames_accepted = 0
        self._used_j1_commissioning_approval_ids: set[str] = set()
        self._used_position_approval_ids: set[str] = set()
        self.mode_initialization_attempts = 0
        self._mode_initialization_index = 0
        self._mode_initialization_started_ns = 0
        self._mode_initialization_delivery_unknown = False
        self.j6_enable_attempts = 0
        self._adapter_factory = (
            adapter_factory
            if adapter_factory is not None
            else lambda: _GsUsbReceiveAdapter(
                receive_mode=self.receive_mode,
                adapter_bus=adapter_bus,
                adapter_address=adapter_address,
            )
        )
        self._adapter: Optional[_ReceiveAdapter] = None

    def claim(self, owner: str) -> None:
        if not isinstance(owner, str) or not owner.strip():
            raise ValueError("owner is required")
        if self.owner is not None and self.owner != owner:
            raise TransportOwnershipError(f"transport already owned by {self.owner!r}")
        self.owner = owner

    def release(self, owner: str) -> None:
        self._require_owner(owner)
        if self.connected:
            raise RuntimeError("stop read-only receive before releasing the transport")
        self.owner = None

    def start_read_only(self, owner: str) -> None:
        self._require_owner(owner)
        if self.position_delivery_unknown or self._mode_initialization_delivery_unknown:
            raise ConnectionError(
                "control delivery is unknown; rebuild only after adapter enumeration and read-only feedback"
            )
        if self.connected:
            raise RuntimeError("read-only receive is already active")
        adapter = self._adapter_factory()
        try:
            adapter.open_read_only()
        except Exception:
            try:
                adapter.close()
            finally:
                self.connected = False
                self.disconnect_reason = "adapter_open_failed"
            raise
        self._adapter = adapter
        self.connected = True
        self.disconnect_reason = None

    def receive_frame(self, owner: str, timeout_ms: int = 100) -> Optional[CanFrame]:
        self._require_owner(owner)
        if type(timeout_ms) is not int or not 1 <= timeout_ms <= 1000:
            raise ValueError("timeout_ms must be an integer in [1, 1000]")
        if not self.connected or self._adapter is None:
            raise ConnectionError(self.disconnect_reason or "CAN receive transport is closed")
        try:
            frame = self._adapter.receive_frame(timeout_ms)
            self.short_usb_packets = self._adapter.short_usb_packets
            return frame
        except Exception:
            self.connected = False
            self.disconnect_reason = "usb_disconnect"
            self._close_adapter()
            raise

    def stop_read_only(self, owner: str) -> None:
        self._require_owner(owner)
        self._close_adapter()
        self.connected = False

    def send(self, owner: str, proposal: CommandProposal) -> None:
        self._require_owner(owner)
        self.send_attempts += 1
        raise PermissionError(
            "PiperCanTransport rejects robot command TX; CAN commands are disabled"
        )

    def send_mode_initialization_frame(self, owner: str, frame: ModeInitializationFrame) -> int:
        self._require_owner(owner)
        if owner != "safety-bridge" or type(frame) is not ModeInitializationFrame:
            raise PermissionError("initialization requires a bridge-owned fixed frame")
        if self._mode_initialization_delivery_unknown:
            raise ConnectionError("initialization delivery is unknown; no retry")
        if (not self.connected or self._adapter is None
                or self.receive_mode is CanReceiveMode.LISTEN_ONLY):
            raise PermissionError("initialization requires an open normal receive session")
        if (self._mode_initialization_index >= len(STAGES)
                or frame.stage != STAGES[self._mode_initialization_index]):
            raise PermissionError("initialization frames must be sent once in fixed order")
        now = time.perf_counter_ns()
        if self._mode_initialization_index == 0:
            self._mode_initialization_started_ns = now
        elif now - self._mode_initialization_started_ns > 10_000_000_000:
            raise PermissionError("initialization sequence exceeded ten seconds")
        sender = getattr(self._adapter, "send_mode_initialization_frame", None)
        if not callable(sender):
            raise PermissionError("adapter has no dedicated initialization path")
        self._mode_initialization_index += 1
        self.mode_initialization_attempts += 1
        try:
            if sender(frame) is not True:
                raise OSError("adapter did not confirm initialization write")
            return time.perf_counter_ns()
        except Exception as exc:
            self._mode_initialization_delivery_unknown = True
            self.connected = False
            self.disconnect_reason = "mode_initialization_delivery_unknown"
            self._close_adapter()
            raise CanInitializationDeliveryUnknown(
                "reset/mode delivery is unknown; stop, check robot, and do not retry") from exc

    def send_j6_enable_once(self, owner: str, frame: J6EnableFrame) -> int:
        self._require_owner(owner)
        if owner != "safety-bridge" or type(frame) is not J6EnableFrame:
            raise PermissionError("J6 enable requires a bridge-owned fixed frame")
        if self.j6_enable_attempts or self._mode_initialization_delivery_unknown:
            raise PermissionError("J6 enable is one-use; no retry")
        if (not self.connected or self._adapter is None
                or self.receive_mode is CanReceiveMode.LISTEN_ONLY):
            raise PermissionError("an open normal receive session is required")
        sender = getattr(self._adapter, "send_j6_enable_frame", None)
        if not callable(sender):
            raise PermissionError("adapter does not support the dedicated J6 enable path")
        self.j6_enable_attempts += 1
        try:
            if sender(frame) is not True:
                raise OSError("adapter did not confirm the J6 enable write")
            return time.perf_counter_ns()
        except Exception as exc:
            self._mode_initialization_delivery_unknown = True
            self.connected = False
            self.disconnect_reason = "j6_enable_delivery_unknown"
            self._close_adapter()
            raise CanInitializationDeliveryUnknown("J6 enable delivery unknown; no retry") from exc

    def send_oem_gripper_test_frame(self, owner, command):
        from laptop.transport.gripper_commissioning import OemGripperTestFrame
        self._require_owner(owner)
        if owner != 'safety-bridge' or type(command) is not OemGripperTestFrame:
            raise PermissionError('typed bridge-owned OEM gripper test required')
        if (self.position_delivery_unknown or not self.connected or self._adapter is None
                or self.receive_mode is CanReceiveMode.LISTEN_ONLY):
            raise PermissionError('connected normal transport without delivery fault required')
        now=time.perf_counter_ns()
        command.validate(now)
        first=getattr(self,'_oem_gripper_window',command.approved_monotonic_ns)
        count=getattr(self,'_oem_gripper_count',0)
        last=getattr(self,'_oem_gripper_last_ns',0)
        if first != command.approved_monotonic_ns or count >= 400 or (last and now-last < 10_000_000):
            raise PermissionError('one 5-second window, at most 400 frames, at least 10ms apart')
        self._oem_gripper_window=first
        self._oem_gripper_count=count+1
        self._oem_gripper_last_ns=now
        try:
            if self._adapter.send_position_frame(command.frame) is not True:
                raise OSError('OEM gripper frame delivery unknown')
            return PositionFrameWrite(command.frame.arbitration_id,command.frame.data,time.perf_counter_ns())
        except Exception:
            self._latch_position_delivery_unknown()
            raise

    def send_gripper_opening_once(self, owner, command):
        from laptop.transport.gripper_commissioning import GripperOpeningStep
        self._require_owner(owner)
        if owner != "safety-bridge" or type(command) is not GripperOpeningStep:
            raise PermissionError("typed bridge-owned gripper commissioning command required")
        if (self.position_delivery_unknown or not self.connected or self._adapter is None
                or self.receive_mode is CanReceiveMode.LISTEN_ONLY):
            raise PermissionError("normal connected transport without delivery fault required")
        if getattr(self, "gripper_opening_attempted", False):
            raise PermissionError("gripper opening is one attempt per transport")
        command.validate(time.perf_counter_ns())
        self.gripper_opening_attempted = True
        try:
            if self._adapter.send_position_frame(command.frame) is not True:
                raise OSError("gripper USB delivery not confirmed")
            return PositionFrameWrite(command.frame.arbitration_id, command.frame.data,
                                      time.perf_counter_ns())
        except Exception:
            self._latch_position_delivery_unknown()
            raise

    def send_supervised_position(
        self, owner: str, batch: PositionCommandBatch,
        authorization: SupervisedPositionAuthorization,
        current_state: object,
        limits: object,
    ) -> PositionBatchWriteReceipt:
        """Write exactly one approved four-frame position batch.

        The method deliberately does not retry. A failed/ambiguous USB write
        closes this transport instance and latches delivery-unknown state.
        """
        if owner != "safety-bridge":
            raise PermissionError("supervised position writes are owned by SafetyBridge")
        self._require_owner(owner)
        self.position_send_attempts += 1
        if type(batch) is not PositionCommandBatch:
            raise TypeError("batch must be a validated PositionCommandBatch")
        if type(authorization) is not SupervisedPositionAuthorization:
            raise TypeError("authorization must be a SupervisedPositionAuthorization")
        if self.position_delivery_unknown:
            raise ConnectionError("position delivery is unknown; do not retry")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("position command TX is unavailable in listen-only mode")
        if not self.connected or self._adapter is None:
            raise ConnectionError(self.disconnect_reason or "CAN receive transport is closed")
        adapter_send = getattr(self._adapter, "send_position_frame", None)
        if not callable(adapter_send):
            raise PermissionError("adapter does not expose the supervised position send path")
        if authorization.approval_id in self._used_position_approval_ids:
            raise PermissionError("position approval is one-use and has already been consumed")

        now = time.perf_counter_ns()
        validate_position_execution_context(
            batch.proposal, current_state, limits, now_monotonic_ns=now,
        )
        authorization.validate_for(batch.proposal, now_monotonic_ns=now)
        self._used_position_approval_ids.add(authorization.approval_id)
        writes: list[PositionFrameWrite] = []
        for frame in batch.frames:
            try:
                frame_now = time.perf_counter_ns()
                if (frame_now > authorization.valid_until_monotonic_ns
                        or frame_now > batch.proposal.valid_until_monotonic_ns):
                    if writes:
                        raise TimeoutError("position approval expired during the frame batch")
                    raise ValueError("position approval expired before the first frame")
                accepted = adapter_send(frame)
                if accepted is not True:
                    raise OSError("USB adapter did not confirm the frame write")
                accepted_at = time.perf_counter_ns()
                writes.append(PositionFrameWrite(
                    frame.arbitration_id, frame.data, accepted_at,
                ))
                self.position_frames_accepted += 1
            except ValueError as exc:
                if not writes:
                    raise
                failed_at = time.perf_counter_ns()
                self._latch_position_delivery_unknown()
                raise CanPositionDeliveryUnknown(
                    approval_id=authorization.approval_id,
                    proposal_id=batch.proposal.proposal_id,
                    confirmed_writes=tuple(writes),
                    failed_frame=frame,
                    failed_at_monotonic_ns=failed_at,
                ) from exc
            except Exception as exc:
                failed_at = time.perf_counter_ns()
                self._latch_position_delivery_unknown()
                raise CanPositionDeliveryUnknown(
                    approval_id=authorization.approval_id,
                    proposal_id=batch.proposal.proposal_id,
                    confirmed_writes=tuple(writes),
                    failed_frame=frame,
                    failed_at_monotonic_ns=failed_at,
                ) from exc

        return PositionBatchWriteReceipt(
            approval_id=authorization.approval_id,
            proposal_id=batch.proposal.proposal_id,
            source_request_id=batch.proposal.source_request_id,
            source_state_sequence=batch.proposal.source_state_sequence,
            target=batch.proposal.target,
            writes=tuple(writes),
        )

    def send_supervised_j1_commissioning_step(
        self,
        owner: str,
        snapshot: J1CommissioningSnapshot,
        batch: J1CommissioningBatch,
        authorization: J1CommissioningAuthorization,
    ) -> J1CommissioningWriteReceipt:
        """Write one fixed J1 step, optionally prefixed by exact CAN/MOVE_J/5%."""
        if owner != "safety-bridge":
            raise PermissionError("J1 commissioning writes are owned by SafetyBridge")
        self._require_owner(owner)
        self.j1_commissioning_send_attempts += 1
        if type(snapshot) is not J1CommissioningSnapshot:
            raise TypeError("snapshot must be a J1CommissioningSnapshot")
        if type(batch) is not J1CommissioningBatch:
            raise TypeError("batch must be a J1CommissioningBatch")
        if type(authorization) is not J1CommissioningAuthorization:
            raise TypeError("authorization must be a J1CommissioningAuthorization")
        if self.position_delivery_unknown:
            raise ConnectionError("prior position delivery is unknown; do not retry")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("J1 commissioning TX is unavailable in listen-only mode")
        if not self.connected or self._adapter is None:
            raise ConnectionError(self.disconnect_reason or "CAN receive transport is closed")
        if authorization.approval_id in self._used_j1_commissioning_approval_ids:
            raise PermissionError("J1 commissioning approval is one-use and already consumed")
        adapter_send = getattr(self._adapter, "send_position_frame", None)
        if not callable(adapter_send):
            raise PermissionError("adapter does not expose the supervised position send path")

        now_ns = time.perf_counter_ns()
        mode_sender = getattr(self._adapter, "send_mode_initialization_frame", None)
        if batch.configure_control_mode and not callable(mode_sender):
            raise PermissionError("adapter does not expose the fixed mode-setting path")
        validate_j1_commissioning_execution(
            snapshot, batch, authorization, now_monotonic_ns=now_ns,
        )
        self._used_j1_commissioning_approval_ids.add(authorization.approval_id)
        writes: list[PositionFrameWrite] = []
        for frame in batch.frames:
            frame_now = time.perf_counter_ns()
            if frame_now > authorization.valid_until_monotonic_ns and not writes:
                raise PermissionError("J1 commissioning approval expired before the first frame")
            try:
                if frame_now > authorization.valid_until_monotonic_ns:
                    raise TimeoutError("J1 commissioning approval expired during the batch")
                accepted = (mode_sender(ModeInitializationFrame("can_joint"))
                            if frame.arbitration_id == 0x151 else adapter_send(frame))
                if accepted is not True:
                    raise OSError("USB adapter did not confirm the J1 frame write")
                write = PositionFrameWrite(
                    frame.arbitration_id, frame.data, time.perf_counter_ns(),
                )
                writes.append(write)
                self.j1_commissioning_frames_accepted += 1
            except Exception as exc:
                failed_at = time.perf_counter_ns()
                self.position_delivery_unknown = True
                self.disconnect_reason = "j1_commissioning_delivery_unknown"
                try:
                    self._close_adapter()
                finally:
                    raise CanJ1CommissioningDeliveryUnknown(
                        approval_id=authorization.approval_id,
                        confirmed_writes=tuple(writes),
                        failed_frame=frame,
                        failed_at_monotonic_ns=failed_at,
                    ) from exc
        return J1CommissioningWriteReceipt(
            approval_id=authorization.approval_id,
            source_state_sequence=snapshot.sequence,
            target_joint_raw_0p001deg=batch.target_joint_raw_0p001deg,
            writes=tuple(writes),
        )

    def send_bounded_model_trial(self, owner, snapshot, batch):
        """Dedicated nominal SDK-unit trial; no generic command permission."""
        from laptop.transport.bounded_trial import BoundedTrialBatch
        from laptop.transport.position_commands import validate_j1_commissioning_snapshot
        if owner != "safety-bridge":
            raise PermissionError("trial is bridge-owned")
        self._require_owner(owner)
        if type(batch) is not BoundedTrialBatch:
            raise TypeError("typed trial batch required")
        now = time.perf_counter_ns()
        validate_j1_commissioning_snapshot(snapshot, now_monotonic_ns=now)
        batch.validate_time(now)
        if batch.source_sequence != snapshot.sequence or batch.current_joints != snapshot.joint_raw_0p001deg:
            raise PermissionError("trial source does not match received joints")
        if self.position_delivery_unknown or not self.connected or self._adapter is None:
            raise ConnectionError("trial transport unavailable; no retry")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("trial TX unavailable in listen-only mode")
        used = getattr(self, "_bounded_trial_requests", set())
        if batch.prediction_request_id in used or len(used) >= 3:
            raise PermissionError("trial budget or one-use request violated")
        used.add(batch.prediction_request_id)
        self._bounded_trial_requests = used
        writes = []
        try:
            # Exact CAN/MOVE_J/5% configuration; no reset or enable operation.
            if self._adapter.send_mode_initialization_frame(ModeInitializationFrame("can_joint")) is not True:
                raise OSError("mode frame acceptance unknown")
            writes.append(PositionFrameWrite(0x151, ModeInitializationFrame("can_joint").data, time.perf_counter_ns()))
            for frame in batch.frames:
                batch.validate_time(time.perf_counter_ns())
                if self._adapter.send_position_frame(frame) is not True:
                    raise OSError("trial frame acceptance unknown")
                writes.append(PositionFrameWrite(frame.arbitration_id, frame.data, time.perf_counter_ns()))
            return tuple(writes)
        except Exception:
            self._latch_position_delivery_unknown()
            raise

    def _latch_position_delivery_unknown(self) -> None:
        self.position_delivery_unknown = True
        self.disconnect_reason = "position_command_delivery_unknown"
        try:
            self._close_adapter()
        except Exception:
            self.connected = False

    def send_diagnostic_query(self, owner: str, query: DiagnosticCanQuery) -> bool:
        self._require_owner(owner)
        if not isinstance(query, DiagnosticCanQuery):
            raise TypeError("only a validated DiagnosticCanQuery may be sent")
        if self.receive_mode is CanReceiveMode.LISTEN_ONLY:
            raise PermissionError("diagnostic getter TX is unavailable in listen-only mode")
        if not self.connected or self._adapter is None:
            raise ConnectionError(self.disconnect_reason or "CAN receive transport is closed")
        try:
            sent = self._adapter.send_diagnostic_query(query)
        except Exception as exc:
            self.disconnect_reason = "diagnostic_query_delivery_unknown"
            self.connected = False
            try:
                self._close_adapter()
            finally:
                raise CanQueryDeliveryUnknown(
                    "diagnostic CAN query write failed; delivery is unknown; do not retry"
                ) from exc
        if sent is not True:
            self.disconnect_reason = "diagnostic_query_delivery_unknown"
            self.connected = False
            self._close_adapter()
            raise CanQueryDeliveryUnknown(
                "diagnostic CAN query write was not acknowledged; delivery is unknown; do not retry"
            )
        return True

    def _close_adapter(self) -> None:
        adapter, self._adapter = self._adapter, None
        if adapter is not None:
            try:
                adapter.close()
            finally:
                self.connected = False

    def _require_owner(self, owner: str) -> None:
        if self.owner != owner:
            raise TransportOwnershipError("caller does not own transport")
