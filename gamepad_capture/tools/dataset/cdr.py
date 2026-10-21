"""Small, dependency-free readers for the ROS 2 CDR messages used here.

The reader supports the little- and big-endian CDR encapsulations emitted by
ROS 2. It deliberately reads one SQLite message at a time so camera payloads
from a multi-gigabyte bag never accumulate in memory.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Any


class CDRDecodeError(ValueError):
    """Raised when a serialized CDR message is incomplete or malformed."""


@dataclass(frozen=True)
class Header:
    stamp_ns: int
    frame_id: str


class CDRReader:
    """Read primitive and ROS message fields from one serialized CDR sample."""

    def __init__(self, data: bytes | bytearray | memoryview) -> None:
        self.data = memoryview(data)
        if len(self.data) < 4:
            raise CDRDecodeError("CDR encapsulation header is truncated")
        if self.data[0] == 0 and self.data[1] == 1:
            self.endian = "<"
        elif self.data[0] == 0 and self.data[1] == 0:
            self.endian = ">"
        else:
            raise CDRDecodeError("unsupported CDR encapsulation kind")
        self.offset = 4

    def _align(self, size: int) -> None:
        # CDR alignment starts at the first byte after the 4-byte encapsulation.
        self.offset += (-(self.offset - 4)) % size
        if self.offset > len(self.data):
            raise CDRDecodeError("CDR alignment exceeds message length")

    def _unpack(self, fmt: str, size: int, label: str) -> Any:
        self._align(size)
        end = self.offset + size
        if end > len(self.data):
            raise CDRDecodeError(f"truncated CDR field: {label}")
        value = struct.unpack_from(self.endian + fmt, self.data, self.offset)[0]
        self.offset = end
        return value

    def u8(self, label: str = "uint8") -> int:
        return int(self._unpack("B", 1, label))

    def u32(self, label: str = "uint32") -> int:
        return int(self._unpack("I", 4, label))

    def i32(self, label: str = "int32") -> int:
        return int(self._unpack("i", 4, label))

    def u64(self, label: str = "uint64") -> int:
        return int(self._unpack("Q", 8, label))

    def i64(self, label: str = "int64") -> int:
        return int(self._unpack("q", 8, label))

    def f64(self, label: str = "float64") -> float:
        return float(self._unpack("d", 8, label))

    def boolean(self, label: str = "bool") -> bool:
        value = self.u8(label)
        if value not in (0, 1):
            raise CDRDecodeError(f"invalid CDR boolean value in {label}: {value}")
        return bool(value)

    def string(self, label: str = "string") -> str:
        length = self.u32(f"{label}.length")
        if length < 1:
            raise CDRDecodeError(f"empty CDR string storage in {label}")
        end = self.offset + length
        if end > len(self.data):
            raise CDRDecodeError(f"truncated CDR string: {label}")
        raw = self.data[self.offset:end].tobytes()
        self.offset = end
        if raw[-1] != 0:
            raise CDRDecodeError(f"CDR string is missing its terminator: {label}")
        try:
            return raw[:-1].decode("utf-8")
        except UnicodeDecodeError as exc:
            raise CDRDecodeError(f"invalid UTF-8 in CDR string: {label}") from exc

    def byte_sequence(self, label: str = "uint8[]") -> memoryview:
        length = self.u32(f"{label}.length")
        end = self.offset + length
        if end > len(self.data):
            raise CDRDecodeError(f"truncated CDR sequence: {label}")
        result = self.data[self.offset:end]
        self.offset = end
        return result

    def f64_sequence(self, label: str = "float64[]") -> list[float]:
        length = self.u32(f"{label}.length")
        return [self.f64(f"{label}[{index}]") for index in range(length)]

    def string_sequence(self, label: str = "string[]") -> list[str]:
        length = self.u32(f"{label}.length")
        return [self.string(f"{label}[{index}]") for index in range(length)]

    def header(self) -> Header:
        seconds = self.i32("header.stamp.sec")
        nanoseconds = self.u32("header.stamp.nanosec")
        if nanoseconds >= 1_000_000_000:
            raise CDRDecodeError("header nanoseconds are outside [0, 1e9)")
        return Header(seconds * 1_000_000_000 + nanoseconds,
                      self.string("header.frame_id"))


def decode_std_string(data: bytes | memoryview) -> str:
    reader = CDRReader(data)
    value = reader.string("data")
    _finish(reader)
    return value


def decode_std_bool(data: bytes | memoryview) -> bool:
    reader = CDRReader(data)
    value = reader.boolean("data")
    _finish(reader)
    return value


def decode_state_snapshot(data: bytes | memoryview) -> dict[str, Any]:
    reader = CDRReader(data)
    header = reader.header()
    result: dict[str, Any] = {
        "header_stamp_ns": header.stamp_ns,
        "header_frame_id": header.frame_id,
        "session_id": reader.string("session_id"),
        "sequence": reader.u64("sequence"),
        "source_monotonic_ns": reader.u64("source_monotonic_ns"),
        "source_clock_id": reader.string("source_clock_id"),
        "source_local_monotonic_ns": reader.u64("source_local_monotonic_ns"),
        "source_local_clock_id": reader.string("source_local_clock_id"),
        "source_clock_mapping_uncertainty_ns": reader.u64("source_clock_mapping_uncertainty_ns"),
        "feedback_fresh": reader.boolean("feedback_fresh"),
        "joint_skew_ns": reader.u64("joint_skew_ns"),
        "status_monotonic_ns": reader.u64("status_monotonic_ns"),
        "joint_pair_monotonic_ns": [reader.u64(f"joint_pair_monotonic_ns[{i}]") for i in range(3)],
        "gripper_monotonic_ns": reader.u64("gripper_monotonic_ns"),
        "motor_monotonic_ns": [reader.u64(f"motor_monotonic_ns[{i}]") for i in range(6)],
        "joint_position_rad": [reader.f64(f"joint_position_rad[{i}]") for i in range(6)],
        "gripper_raw": reader.i64("gripper_raw"),
        "gripper_nominal_m": reader.f64("gripper_nominal_m"),
        "ctrl_mode_raw": reader.u8("ctrl_mode_raw"),
        "arm_status_raw": reader.u8("arm_status_raw"),
        "error_code": reader.u32("error_code"),
        "motor_enabled": [reader.boolean(f"motor_enabled[{i}]") for i in range(6)],
        "hardware_connected": reader.boolean("hardware_connected"),
        "fake": reader.boolean("fake"),
    }
    _finish(reader)
    return result


def decode_camera_metadata(data: bytes | memoryview) -> dict[str, Any]:
    reader = CDRReader(data)
    header = reader.header()
    result = {
        "header_stamp_ns": header.stamp_ns,
        "header_frame_id": header.frame_id,
        "role": reader.string("role"),
        "session_id": reader.string("session_id"),
        "device_name": reader.string("device_name"),
        "sequence": reader.u64("sequence"),
        "source_monotonic_ns": reader.u64("source_monotonic_ns"),
        "source_clock_id": reader.string("source_clock_id"),
        "source_local_monotonic_ns": reader.u64("source_local_monotonic_ns"),
        "source_local_clock_id": reader.string("source_local_clock_id"),
        "clock_mapping_uncertainty_ns": reader.u64("clock_mapping_uncertainty_ns"),
        "dropped_frames": reader.u64("dropped_frames"),
        "encoding": reader.string("encoding"),
        "width": reader.u32("width"),
        "height": reader.u32("height"),
    }
    _finish(reader)
    return result


def decode_image_header(data: bytes | memoryview) -> Header:
    return CDRReader(data).header()


def decode_image(data: bytes | memoryview) -> tuple[Header, dict[str, Any]]:
    reader = CDRReader(data)
    header = reader.header()
    height = reader.u32("height")
    width = reader.u32("width")
    encoding = reader.string("encoding")
    is_bigendian = reader.u8("is_bigendian")
    step = reader.u32("step")
    pixels = reader.byte_sequence("data")
    _finish(reader)
    if width <= 0 or height <= 0 or step < width:
        raise CDRDecodeError("Image dimensions or row step are invalid")
    if len(pixels) != step * height:
        raise CDRDecodeError("Image payload size does not equal step * height")
    return header, {
        "height": height,
        "width": width,
        "encoding": encoding,
        "is_bigendian": is_bigendian,
        "step": step,
        "data": pixels,
    }


def decode_joint_state(data: bytes | memoryview) -> dict[str, Any]:
    reader = CDRReader(data)
    header = reader.header()
    result = {
        "header_stamp_ns": header.stamp_ns,
        "header_frame_id": header.frame_id,
        "name": reader.string_sequence("name"),
        "position": reader.f64_sequence("position"),
        "velocity": reader.f64_sequence("velocity"),
        "effort": reader.f64_sequence("effort"),
    }
    _finish(reader)
    return result


def decode_command_receipt(data: bytes | memoryview) -> dict[str, Any]:
    reader = CDRReader(data)
    header = reader.header()
    result = {
        "header_stamp_ns": header.stamp_ns,
        "header_frame_id": header.frame_id,
        "request_id": reader.string("request_id"),
        "observation_id": reader.string("observation_id"),
        "decision": reader.string("decision"),
        "reason": reader.string("reason"),
        "sequence": reader.u64("sequence"),
        "source_monotonic_ns": reader.u64("source_monotonic_ns"),
        "adapter_accepted": reader.boolean("adapter_accepted"),
        "feedback_matched": reader.boolean("feedback_matched"),
        "can_tx_calls": reader.u32("can_tx_calls"),
    }
    _finish(reader)
    return result


def _finish(reader: CDRReader) -> None:
    # CDR can end with up to seven alignment bytes. Ignore only zero padding.
    trailing = reader.data[reader.offset:].tobytes()
    if len(trailing) > 7 or any(trailing):
        raise CDRDecodeError("unexpected bytes after serialized message")
