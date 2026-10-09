"""Length-prefixed msgpack protocol for future SafetyBridge process isolation.

The codec is transport-agnostic and performs no network or CAN I/O.  Message
types are deliberately small and versioned so malformed traffic fails closed.
"""

from __future__ import annotations

import struct
from typing import Any, Dict, Tuple

import msgpack


PROTOCOL_VERSION = 1
MAX_PAYLOAD_BYTES = 1_048_576
MESSAGE_TYPES = frozenset(
    {"state_snapshot", "command_proposal", "safety_decision", "heartbeat"}
)


def encode_message(message_type: str, payload: Dict[str, Any]) -> bytes:
    if message_type not in MESSAGE_TYPES:
        raise ValueError(f"unsupported message type: {message_type}")
    if not isinstance(payload, dict):
        raise TypeError("payload must be a dictionary")
    body = msgpack.packb(
        {"version": PROTOCOL_VERSION, "type": message_type, "payload": payload},
        use_bin_type=True,
    )
    if len(body) > MAX_PAYLOAD_BYTES:
        raise ValueError("payload exceeds protocol limit")
    return struct.pack("!I", len(body)) + body


def decode_message(frame: bytes) -> Tuple[str, Dict[str, Any]]:
    if len(frame) < 4:
        raise ValueError("truncated frame header")
    (length,) = struct.unpack("!I", frame[:4])
    if length > MAX_PAYLOAD_BYTES:
        raise ValueError("declared payload exceeds protocol limit")
    if len(frame) != length + 4:
        raise ValueError("frame length mismatch")
    message = msgpack.unpackb(frame[4:], raw=False, strict_map_key=True)
    if not isinstance(message, dict):
        raise ValueError("message body must be a map")
    if message.get("version") != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol version")
    message_type = message.get("type")
    payload = message.get("payload")
    if message_type not in MESSAGE_TYPES:
        raise ValueError("unsupported message type")
    if not isinstance(payload, dict):
        raise ValueError("payload must be a map")
    if set(message) != {"version", "type", "payload"}:
        raise ValueError("unexpected message fields")
    return message_type, payload
