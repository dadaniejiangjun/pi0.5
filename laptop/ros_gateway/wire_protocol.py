"""Small MsgPack control frames and bounded raw-image frames for the ROS gateway.

This module is transport-only. It does not open cameras, CAN, or robot sessions.
"""
from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
from typing import Any, Mapping

import msgpack


PROTOCOL_VERSION = 1
MAX_CONTROL_BODY_BYTES = 1_048_576
MAX_IMAGE_BYTES = 64 * 1024 * 1024
MAX_IMAGE_DIMENSION = 8192
CONTROL_HEADER = struct.Struct("!I")
IMAGE_HEADER = struct.Struct("!4sBBHIIIQQQ")
IMAGE_MAGIC = b"P5IM"
IMAGE_VERSION = 1
IMAGE_ENCODING_BGR8 = 1
IMAGE_ROLES = {"base": 1, "wrist": 2}
IMAGE_ROLE_NAMES = {value: key for key, value in IMAGE_ROLES.items()}
CONTROL_TYPES = frozenset({
    "hello", "hello_ack", "heartbeat", "heartbeat_ack", "state_snapshot",
    "clock_sync_complete", "clock_sync_ack", "gateway_status",
    "command_proposal", "command_receipt", "control_request", "control_result",
    "fault",
})


@dataclass(frozen=True)
class RawImageFrame:
    role: str
    width: int
    height: int
    stride: int
    sequence: int
    host_monotonic_ns: int
    pixels: bytes
    encoding: str = "bgr8"


@dataclass(frozen=True)
class MonotonicClockEstimate:
    """A four-timestamp estimate of server time minus client time."""

    server_minus_client_offset_ns: int
    round_trip_ns: int
    uncertainty_ns: int


def estimate_monotonic_clock_offset(
    *, client_send_ns: int, server_receive_ns: int,
    server_send_ns: int, client_receive_ns: int,
) -> MonotonicClockEstimate:
    """Estimate monotonic-clock offset using a four-timestamp exchange.

    The returned offset is ``server_clock - client_clock``. The uncertainty
    is half the network round trip after subtracting server processing time.
    This estimate is for timestamp validation and deadline rebasing only; it
    does not prove clock synchronization accuracy or authorize hardware use.
    """
    stamps = (client_send_ns, server_receive_ns, server_send_ns, client_receive_ns)
    if any(type(stamp) is not int or stamp <= 0 for stamp in stamps):
        raise ValueError("clock timestamps must be positive monotonic integers")
    if server_send_ns < server_receive_ns:
        raise ValueError("server clock regressed during clock exchange")
    if client_receive_ns < client_send_ns:
        raise ValueError("client clock regressed during clock exchange")
    elapsed_ns = client_receive_ns - client_send_ns
    processing_ns = server_send_ns - server_receive_ns
    round_trip_ns = elapsed_ns - processing_ns
    if round_trip_ns < 0:
        raise ValueError("clock exchange has impossible negative network round trip")
    offset_ns = ((server_receive_ns - client_send_ns)
                 + (server_send_ns - client_receive_ns)) // 2
    return MonotonicClockEstimate(
        server_minus_client_offset_ns=offset_ns,
        round_trip_ns=round_trip_ns,
        uncertainty_ns=(round_trip_ns + 1) // 2,
    )


def encode_control_message(message_type: str, payload: Mapping[str, Any]) -> bytes:
    if message_type not in CONTROL_TYPES:
        raise ValueError(f"unsupported control message type: {message_type}")
    if not isinstance(payload, Mapping) or any(not isinstance(k, str) for k in payload):
        raise TypeError("payload must be a string-keyed map")
    body = msgpack.packb(
        {"version": PROTOCOL_VERSION, "type": message_type, "payload": dict(payload)},
        use_bin_type=True,
    )
    if len(body) > MAX_CONTROL_BODY_BYTES:
        raise ValueError("control body exceeds 1 MiB")
    return CONTROL_HEADER.pack(len(body)) + body


def decode_control_body(body: bytes) -> tuple[str, dict[str, Any]]:
    if not isinstance(body, bytes) or not body:
        raise ValueError("control body must be non-empty bytes")
    if len(body) > MAX_CONTROL_BODY_BYTES:
        raise ValueError("control body exceeds 1 MiB")
    try:
        message = msgpack.unpackb(body, raw=False, strict_map_key=True)
    except (ValueError, msgpack.UnpackValueError, msgpack.FormatError) as exc:
        raise ValueError(f"invalid MsgPack control body: {exc}") from exc
    if not isinstance(message, dict):
        raise ValueError("control message must be a map")
    if set(message) != {"version", "type", "payload"}:
        raise ValueError("unexpected control message fields")
    if message["version"] != PROTOCOL_VERSION:
        raise ValueError("unsupported control protocol version")
    message_type = message["type"]
    payload = message["payload"]
    if message_type not in CONTROL_TYPES:
        raise ValueError("unsupported control message type")
    if not isinstance(payload, dict) or any(not isinstance(k, str) for k in payload):
        raise ValueError("control payload must be a string-keyed map")
    return message_type, payload


class IncompleteFrameError(ConnectionError):
    """A stateless read consumed part of a frame; reconnect before retrying."""


class ControlMessageReader:
    """Bounded per-connection reader preserving partial frames across timeouts."""

    def __init__(self):
        self._buffer = bytearray()

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    def receive(self, sock):
        while len(self._buffer) < CONTROL_HEADER.size:
            self._read(sock, CONTROL_HEADER.size - len(self._buffer))
        (body_size,) = CONTROL_HEADER.unpack(self._buffer[:CONTROL_HEADER.size])
        if not 0 < body_size <= MAX_CONTROL_BODY_BYTES:
            raise ValueError(f"invalid control body length: {body_size}")
        frame_size = CONTROL_HEADER.size + body_size
        while len(self._buffer) < frame_size:
            self._read(sock, frame_size - len(self._buffer))
        body = bytes(self._buffer[CONTROL_HEADER.size:])
        self._buffer.clear()
        return decode_control_body(body)

    def _read(self, sock, size):
        chunk = sock.recv(size)
        if not chunk:
            raise EOFError(f"connection closed with {self.buffered_bytes} frame bytes buffered")
        self._buffer.extend(chunk)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    if type(size) is not int or size < 0:
        raise ValueError("size must be a non-negative integer")
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        try:
            chunk = sock.recv(remaining)
        except socket.timeout as exc:
            if chunks:
                raise IncompleteFrameError("timeout after partial frame; reconnect required") from exc
            raise
        if not chunk:
            raise EOFError(f"connection closed with {remaining} bytes remaining")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_control_message(sock: socket.socket) -> tuple[str, dict[str, Any]]:
    reader = ControlMessageReader()
    try:
        return reader.receive(sock)
    except socket.timeout as exc:
        if reader.buffered_bytes:
            raise IncompleteFrameError("timeout after partial control frame; reconnect required") from exc
        raise


def send_control_message(
    sock: socket.socket, message_type: str, payload: Mapping[str, Any]
) -> None:
    sock.sendall(encode_control_message(message_type, payload))


def encode_image_frame(frame: RawImageFrame) -> bytes:
    if frame.role not in IMAGE_ROLES:
        raise ValueError("image role must be base or wrist")
    if frame.encoding != "bgr8":
        raise ValueError("only uncompressed bgr8 images are supported")
    for name, value in (("width", frame.width), ("height", frame.height)):
        if type(value) is not int or not 1 <= value <= MAX_IMAGE_DIMENSION:
            raise ValueError(f"{name} is outside the supported range")
    if type(frame.stride) is not int or frame.stride < frame.width * 3:
        raise ValueError("stride is smaller than packed bgr8 row width")
    if frame.stride > frame.width * 3 + 65_536:
        raise ValueError("stride padding exceeds the supported bound")
    if type(frame.sequence) is not int or frame.sequence < 0:
        raise ValueError("sequence must be a non-negative integer")
    if type(frame.host_monotonic_ns) is not int or frame.host_monotonic_ns < 0:
        raise ValueError("host timestamp must be a non-negative integer")
    pixels = bytes(frame.pixels)
    expected = frame.stride * frame.height
    if expected > MAX_IMAGE_BYTES or len(pixels) != expected:
        raise ValueError("image payload length does not match bounded dimensions and stride")
    header = IMAGE_HEADER.pack(
        IMAGE_MAGIC, IMAGE_VERSION, IMAGE_ROLES[frame.role], IMAGE_ENCODING_BGR8,
        frame.width, frame.height, frame.stride, len(pixels), frame.sequence,
        frame.host_monotonic_ns,
    )
    return header + pixels


def decode_image_header(header: bytes) -> tuple[str, int, int, int, int, int, int]:
    if len(header) != IMAGE_HEADER.size:
        raise ValueError("invalid raw-image header size")
    magic, version, role_id, encoding, width, height, stride, payload_size, sequence, stamp = (
        IMAGE_HEADER.unpack(header)
    )
    if magic != IMAGE_MAGIC or version != IMAGE_VERSION:
        raise ValueError("unsupported raw-image framing")
    if role_id not in IMAGE_ROLE_NAMES or encoding != IMAGE_ENCODING_BGR8:
        raise ValueError("unsupported raw-image role or encoding")
    if not 1 <= width <= MAX_IMAGE_DIMENSION or not 1 <= height <= MAX_IMAGE_DIMENSION:
        raise ValueError("raw-image dimensions are outside the supported range")
    if stride < width * 3 or stride > width * 3 + 65_536:
        raise ValueError("raw-image stride is outside the supported range")
    if payload_size != stride * height or payload_size > MAX_IMAGE_BYTES:
        raise ValueError("raw-image payload size is invalid")
    return IMAGE_ROLE_NAMES[role_id], width, height, stride, payload_size, sequence, stamp


def recv_image_frame(sock: socket.socket) -> RawImageFrame:
    header = recv_exact(sock, IMAGE_HEADER.size)
    role, width, height, stride, payload_size, sequence, stamp = decode_image_header(header)
    try:
        pixels = recv_exact(sock, payload_size)
    except socket.timeout as exc:
        raise IncompleteFrameError("timeout after image header; reconnect required") from exc
    return RawImageFrame(role, width, height, stride, sequence, stamp, pixels)


def send_image_frame(sock: socket.socket, frame: RawImageFrame) -> None:
    sock.sendall(encode_image_frame(frame))


def _validate_mapped_source_time(
    stamps: list[int],
    *,
    now_monotonic_ns: int,
    remote_to_local_offset_ns: int | None,
    max_age_ns: int,
    mapping_uncertainty_ns: int,
) -> None:
    if remote_to_local_offset_ns is None:
        raise ValueError("source clock mapping is not available")
    if type(now_monotonic_ns) is not int or now_monotonic_ns < 0:
        raise ValueError("local monotonic time must be a non-negative integer")
    if type(max_age_ns) is not int or max_age_ns <= 0:
        raise ValueError("maximum source age must be positive")
    if type(mapping_uncertainty_ns) is not int or mapping_uncertainty_ns < 0:
        raise ValueError("clock mapping uncertainty must be non-negative")
    mapped = [stamp + remote_to_local_offset_ns for stamp in stamps]
    oldest = min(mapped)
    newest = max(mapped)
    if newest > now_monotonic_ns + mapping_uncertainty_ns:
        raise ValueError("source timestamp is in the future beyond clock uncertainty")
    if now_monotonic_ns - oldest > max_age_ns + mapping_uncertainty_ns:
        raise ValueError("source data is stale beyond the configured age")


def validate_state_snapshot_envelope(
    payload: Mapping[str, Any],
    *,
    session_id: str,
    last_sequence: int,
    now_monotonic_ns: int,
    remote_to_local_offset_ns: int | None,
    max_age_ns: int = 120_000_000,
    mapping_uncertainty_ns: int = 0,
) -> int:
    """Reject wrong-session, duplicate, stale, or incomplete gateway feedback."""
    if not isinstance(payload, Mapping):
        raise ValueError("state snapshot must be a map")
    if payload.get("session_id") != session_id or not session_id:
        raise ValueError("state snapshot session does not match active handshake")
    sequence = payload.get("sequence")
    if type(sequence) is not int or sequence <= 0:
        raise ValueError("state sequence must be a positive integer")
    if type(last_sequence) is not int or sequence <= last_sequence:
        raise ValueError("duplicate or out-of-order state sequence")
    if payload.get("feedback_fresh") is not True:
        raise ValueError("gateway marked feedback as stale or incomplete")

    source_stamp = payload.get("host_monotonic_ns")
    status_stamp = payload.get("status_monotonic_ns")
    gripper_stamp = payload.get("gripper_monotonic_ns")
    joint_stamps = payload.get("joint_pair_monotonic_ns")
    motor_stamps = payload.get("motor_monotonic_ns")
    if any(type(stamp) is not int or stamp < 0 for stamp in (source_stamp, status_stamp, gripper_stamp)):
        raise ValueError("state source timestamps must be non-negative integers")
    if not isinstance(joint_stamps, (list, tuple)) or len(joint_stamps) != 3:
        raise ValueError("state requires three joint-pair timestamps")
    if not isinstance(motor_stamps, (list, tuple)) or len(motor_stamps) != 6:
        raise ValueError("state requires six motor timestamps")
    all_stamps = [status_stamp, *joint_stamps, gripper_stamp, *motor_stamps]
    if any(type(stamp) is not int or stamp < 0 for stamp in all_stamps):
        raise ValueError("state channel timestamps must be non-negative integers")
    if source_stamp != max(all_stamps):
        raise ValueError("aggregate source timestamp does not match channel timestamps")
    _validate_mapped_source_time(
        all_stamps,
        now_monotonic_ns=now_monotonic_ns,
        remote_to_local_offset_ns=remote_to_local_offset_ns,
        max_age_ns=max_age_ns,
        mapping_uncertainty_ns=mapping_uncertainty_ns,
    )
    return sequence


def validate_image_source_time(
    frame: RawImageFrame,
    *,
    expected_role: str,
    last_sequence: int,
    now_monotonic_ns: int,
    remote_to_local_offset_ns: int | None,
    max_age_ns: int = 250_000_000,
    mapping_uncertainty_ns: int = 0,
) -> None:
    """Reject duplicate/out-of-order frames and frames outside the freshness window."""
    if frame.role != expected_role:
        raise ValueError(f"image role mismatch: expected {expected_role}, got {frame.role}")
    if type(last_sequence) is not int or frame.sequence <= last_sequence:
        raise ValueError("duplicate or out-of-order image sequence")
    _validate_mapped_source_time(
        [frame.host_monotonic_ns],
        now_monotonic_ns=now_monotonic_ns,
        remote_to_local_offset_ns=remote_to_local_offset_ns,
        max_age_ns=max_age_ns,
        mapping_uncertainty_ns=mapping_uncertainty_ns,
    )
