"""Wire format and SDL-compatible mapping shared by Windows and ROS 2."""

from __future__ import annotations

import math
from typing import Any


INPUT_SCHEMA = "piper_gamepad_xinput_v1"
SDL_BUTTON_LAYOUT = (
    ("A", 0x1000), ("B", 0x2000), ("X", 0x4000), ("Y", 0x8000),
    ("BACK", 0x0020), ("GUIDE", 0x0400), ("START", 0x0010),
    ("LEFTSTICK", 0x0040), ("RIGHTSTICK", 0x0080),
    ("LEFTSHOULDER", 0x0100), ("RIGHTSHOULDER", 0x0200),
    ("DPAD_UP", 0x0001), ("DPAD_DOWN", 0x0002),
    ("DPAD_LEFT", 0x0004), ("DPAD_RIGHT", 0x0008),
)
XINPUT_BUTTON_MASK = sum(bit for _, bit in SDL_BUTTON_LAYOUT)


def normalize_stick(raw: int) -> float:
    value = int(raw)
    if value < 0:
        return max(-1.0, value / 32768.0)
    return min(1.0, value / 32767.0)


def joy_from_xinput(raw: dict[str, Any]) -> tuple[list[float], list[int]]:
    """Map XInput fields to ROS joy's SDL game-controller ordering."""
    mask = int(raw["buttons_mask"])
    axes = [
        normalize_stick(int(raw["left_x"])),
        -normalize_stick(int(raw["left_y"])),
        normalize_stick(int(raw["right_x"])),
        -normalize_stick(int(raw["right_y"])),
        int(raw["left_trigger"]) / 255.0,
        int(raw["right_trigger"]) / 255.0,
    ]
    buttons = [int(bool(mask & bit)) for _, bit in SDL_BUTTON_LAYOUT]
    return axes, buttons


def neutral_joy() -> tuple[list[float], list[int]]:
    return [0.0] * 6, [0] * len(SDL_BUTTON_LAYOUT)


def validate_input_frame(frame: Any) -> dict[str, Any]:
    if not isinstance(frame, dict) or frame.get("schema") != INPUT_SCHEMA:
        raise ValueError("unsupported input schema")
    session_id = frame.get("session_id")
    if not isinstance(session_id, str) or not session_id or len(session_id) > 64:
        raise ValueError("invalid session_id")
    for key in ("sequence", "source_time_ns", "device_index"):
        if isinstance(frame.get(key), bool) or not isinstance(frame.get(key), int):
            raise ValueError(f"{key} must be an integer")
    if frame["sequence"] < 0 or frame["source_time_ns"] <= 0:
        raise ValueError("invalid sequence or source timestamp")
    if frame["device_index"] not in range(4):
        raise ValueError("device_index must be 0..3")
    if not isinstance(frame.get("connected"), bool):
        raise ValueError("connected must be boolean")

    raw = frame.get("raw")
    if not isinstance(raw, dict):
        raise ValueError("raw input must be an object")
    raw_fields = (
        "buttons_mask", "left_trigger", "right_trigger",
        "left_x", "left_y", "right_x", "right_y",
    )
    if set(raw) != set(raw_fields):
        raise ValueError("raw input must contain exactly the seven XInput fields")
    for key in raw_fields:
        if isinstance(raw.get(key), bool) or not isinstance(raw.get(key), int):
            raise ValueError(f"raw.{key} must be an integer")
    if not 0 <= raw["buttons_mask"] <= 0xFFFF:
        raise ValueError("raw.buttons_mask must be in 0..65535")
    if raw["buttons_mask"] & ~XINPUT_BUTTON_MASK:
        raise ValueError("raw.buttons_mask contains unsupported button bits")
    for key in ("left_trigger", "right_trigger"):
        if not 0 <= raw[key] <= 255:
            raise ValueError(f"raw.{key} must be in 0..255")
    for key in ("left_x", "left_y", "right_x", "right_y"):
        if not -32768 <= raw[key] <= 32767:
            raise ValueError(f"raw.{key} must be in -32768..32767")

    axes, buttons = joy_from_xinput(raw) if frame["connected"] else neutral_joy()
    joy = frame.get("joy")
    if not isinstance(joy, dict) or joy.get("axes") != axes or joy.get("buttons") != buttons:
        raise ValueError("joy mapping does not match raw XInput state")
    if not frame["connected"] and any(raw.values()):
        raise ValueError("disconnected input must be neutral")
    if "packet_number" in frame and frame["packet_number"] is not None:
        if isinstance(frame["packet_number"], bool) or not isinstance(frame["packet_number"], int):
            raise ValueError("packet_number must be an integer or null")
        if frame["packet_number"] < 0:
            raise ValueError("packet_number must be non-negative")
    if frame["connected"] and frame.get("packet_number") is None:
        raise ValueError("connected input requires packet_number")
    if not frame["connected"] and frame.get("packet_number") is not None:
        raise ValueError("disconnected input must have a null packet_number")
    if frame.get("source_clock") != "windows_perf_counter_ns":
        raise ValueError("unsupported source clock")

    for axis in joy["axes"]:
        if isinstance(axis, bool) or not isinstance(axis, (int, float)):
            raise ValueError("joy axes must be numeric")
        if not math.isfinite(axis):
            raise ValueError("non-finite axis value")
        if not -1.0 <= axis <= 1.0:
            raise ValueError("joy axes must be in -1..1")
    if not isinstance(joy["buttons"], list) or any(
        type(button) is not int or button not in (0, 1)
        for button in joy["buttons"]
    ):
        raise ValueError("joy buttons must contain integer 0 or 1 values")
    return frame
