import struct

import pytest

from laptop.runtime.bridge_protocol import (
    MAX_PAYLOAD_BYTES,
    decode_message,
    encode_message,
)


def test_protocol_round_trip():
    frame = encode_message("heartbeat", {"sequence": 3, "healthy": True})
    message_type, payload = decode_message(frame)
    assert message_type == "heartbeat"
    assert payload == {"sequence": 3, "healthy": True}


@pytest.mark.parametrize(
    "frame",
    [
        b"",
        struct.pack("!I", 10) + b"short",
        struct.pack("!I", MAX_PAYLOAD_BYTES + 1),
    ],
)
def test_protocol_rejects_truncated_or_oversized_frames(frame):
    with pytest.raises(ValueError):
        decode_message(frame)


def test_protocol_rejects_unknown_message_type():
    with pytest.raises(ValueError):
        encode_message("can_frame", {"data": b"forbidden"})
