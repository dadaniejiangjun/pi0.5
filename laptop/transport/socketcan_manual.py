"""Explicit manual SocketCAN adapter for the existing guarded batch transport."""
import struct
import time

from laptop.transport.socketcan_receive import SocketCanReceiveTransport
from laptop.transport.position_commands import CanTxFrame
from laptop.transport.mode_initialization import ModeInitializationFrame


class SocketCanManualAdapter:
    short_usb_packets = 0

    def __init__(self, interface, *, transmit_enabled=False, event_sink=None):
        self.receiver = SocketCanReceiveTransport(interface)
        self.transmit_enabled = transmit_enabled
        self.event_sink = event_sink
        self.context = {}
        self.tx_calls = 0
        self.failed = False
        self._lock_file = None

    def open_read_only(self):
        import fcntl
        # Cooperating manual owners cannot open the same interface twice.
        self._lock_file = open(f'/tmp/pi05-manual-{self.receiver.interface}.lock', 'a')
        try:
            fcntl.flock(self._lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.receiver.claim('native-manual')
            self.receiver.start_read_only('native-manual')
        except BaseException:
            self.close()
            raise

    def receive_frame(self, timeout_ms=100):
        return self.receiver.receive_frame('native-manual', timeout_ms)

    def _event(self, event, frame, **extra):
        record = dict(self.context, event=event, monotonic_ns=time.perf_counter_ns(),
                      can_id=frame.arbitration_id, data_hex=frame.data.hex(), **extra)
        if self.event_sink is not None:
            self.event_sink(record)

    def _send(self, frame):
        if self.transmit_enabled is not True or self.failed:
            raise PermissionError('manual TX disabled or delivery unknown latched')
        self._event('CAN_WRITE_INTENT', frame)
        try:
            raw = struct.pack('=IB3x8s', frame.arbitration_id, len(frame.data), frame.data)
            self.tx_calls += 1
            if self.receiver._socket.send(raw) != 16:
                raise OSError('short SocketCAN write')
            self._event('CAN_KERNEL_ACCEPTED', frame, physical_execution_verified=False)
            return True
        except BaseException as exc:
            self.failed = True
            self._event('CAN_DELIVERY_UNKNOWN', frame, reason=str(exc))
            raise

    def send_mode_initialization_frame(self, frame):
        if type(frame) is not ModeInitializationFrame or frame.stage != 'can_joint':
            raise PermissionError('only typed CAN/MOVE_J speed setting is permitted; no reset')
        return self._send(frame)

    def send_position_frame(self, frame):
        if type(frame) is not CanTxFrame or frame.arbitration_id not in (0x155, 0x156, 0x157, 0x159):
            raise PermissionError('only guarded joint/gripper position frames are permitted')
        return self._send(frame)

    def send_diagnostic_query(self, query):
        from laptop.transport.diagnostic_queries import DiagnosticCanQuery
        if type(query) is not DiagnosticCanQuery:
            raise PermissionError('only allowlisted non-motion getter requests')
        return self._send(CanTxFrame(query.request_can_id, query.payload))

    def close(self):
        if self.receiver.connected:
            self.receiver.stop_read_only('native-manual')
        if self._lock_file is not None:
            self._lock_file.close()
            self._lock_file = None
