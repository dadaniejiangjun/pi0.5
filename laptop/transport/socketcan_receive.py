"""Linux SocketCAN receive adapter for the existing PiperSession decoder.

There is no socket send operation. Control modes must use a separately
commissioned transport; opening this adapter cannot enable or move the arm.
"""
import re
import select
import socket
import struct
import time

from laptop.transport.can_transport import CanFrame


class SocketCanReceiveTransport:
    def __init__(self, interface="can0"):
        if not re.fullmatch(r"(?:can|vcan)[0-9]+", interface):
            raise ValueError("expected an explicit canN or vcanN interface")
        self.interface = interface
        self.owner = None
        self.send_attempts = 0
        self._socket = None
        self.disconnect_reason = None

    @property
    def connected(self):
        return self._socket is not None

    def claim(self, owner):
        if not owner or self.owner is not None:
            raise PermissionError("CAN receiver already owned or owner is empty")
        self.owner = owner

    def _check(self, owner):
        if owner != self.owner or self.owner is None:
            raise PermissionError("SocketCAN owner mismatch")

    def release(self, owner):
        self._check(owner)
        if self.connected:
            raise RuntimeError("stop receiver before releasing ownership")
        self.owner = None

    def start_read_only(self, owner):
        self._check(owner)
        if self.connected:
            raise RuntimeError("receiver already started")
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            sock.bind((self.interface,))
            sock.setblocking(False)
        except BaseException:
            sock.close()
            raise
        self._socket = sock

    def receive_frame(self, owner, timeout_ms=100):
        self._check(owner)
        if not self.connected:
            raise RuntimeError("receiver is not connected")
        if not 0 <= timeout_ms <= 5000:
            raise ValueError("timeout must be within 0..5000 ms")
        if not select.select([self._socket], [], [], timeout_ms / 1000)[0]:
            return None
        raw = self._socket.recv(16)
        stamp = time.perf_counter_ns()
        can_id, length, data = struct.unpack("=IB3x8s", raw)
        if can_id & 0x40000000:  # RTR is not feedback data.
            return None
        if length > 8:
            raise ValueError("CAN FD is not accepted by this classic CAN receiver")
        return CanFrame(
            arbitration_id=can_id & 0x1FFFFFFF, data=data[:length],
            is_extended_id=bool(can_id & 0x80000000),
            is_error_frame=bool(can_id & 0x20000000),
            received_monotonic_ns=stamp,
        )

    def stop_read_only(self, owner):
        self._check(owner)
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def send(self, *args, **kwargs):
        raise PermissionError("native receive-only transport has no transmit path")

    def send_supervised_position(self, *args, **kwargs):
        raise PermissionError("native receive-only transport has no transmit path")
