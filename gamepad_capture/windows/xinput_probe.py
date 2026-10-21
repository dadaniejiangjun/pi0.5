"""Read-only XInput probe; it never sends rumble or robot commands."""

from __future__ import annotations

import argparse
import ctypes
import json
import platform
import sys
import time
from pathlib import Path
from typing import Any


ERROR_SUCCESS = 0
ERROR_DEVICE_NOT_CONNECTED = 1167
COINIT_MULTITHREADED = 0
RPC_E_CHANGED_MODE = 0x80010106

BUTTON_BITS = (
    ("DPAD_UP", 0x0001),
    ("DPAD_DOWN", 0x0002),
    ("DPAD_LEFT", 0x0004),
    ("DPAD_RIGHT", 0x0008),
    ("START", 0x0010),
    ("BACK", 0x0020),
    ("LEFTSTICK", 0x0040),
    ("RIGHTSTICK", 0x0080),
    ("LEFTSHOULDER", 0x0100),
    ("RIGHTSHOULDER", 0x0200),
    ("GUIDE", 0x0400),
    ("A", 0x1000),
    ("B", 0x2000),
    ("X", 0x4000),
    ("Y", 0x8000),
)


class XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("buttons", ctypes.c_ushort),
        ("left_trigger", ctypes.c_ubyte),
        ("right_trigger", ctypes.c_ubyte),
        ("left_x", ctypes.c_short),
        ("left_y", ctypes.c_short),
        ("right_x", ctypes.c_short),
        ("right_y", ctypes.c_short),
    ]


class XInputState(ctypes.Structure):
    _fields_ = [
        ("packet_number", ctypes.c_uint32),
        ("gamepad", XInputGamepad),
    ]


def normalize_stick(raw: int) -> float:
    """Normalize a signed XInput stick axis without losing the -32768 endpoint."""
    value = int(raw)
    if value < 0:
        return max(-1.0, value / 32768.0)
    return min(1.0, value / 32767.0)


def gamepad_snapshot(gamepad: XInputGamepad, packet_number: int) -> dict[str, Any]:
    buttons_mask = int(gamepad.buttons)
    return {
        "packet_number": int(packet_number),
        "raw": {
            "buttons_mask": buttons_mask,
            "left_trigger": int(gamepad.left_trigger),
            "right_trigger": int(gamepad.right_trigger),
            "left_x": int(gamepad.left_x),
            "left_y": int(gamepad.left_y),
            "right_x": int(gamepad.right_x),
            "right_y": int(gamepad.right_y),
        },
        "sticks_normalized": {
            "left_x": normalize_stick(gamepad.left_x),
            "left_y": normalize_stick(gamepad.left_y),
            "right_x": normalize_stick(gamepad.right_x),
            "right_y": normalize_stick(gamepad.right_y),
        },
        "triggers_normalized": {
            "left": round(int(gamepad.left_trigger) / 255.0, 6),
            "right": round(int(gamepad.right_trigger) / 255.0, 6),
        },
        "buttons_pressed": [
            name for name, bit in BUTTON_BITS if buttons_mask & bit
        ],
    }


def load_xinput() -> tuple[Any, str]:
    """Load the first available Windows XInput DLL and bind XInputGetState."""
    if platform.system() != "Windows" or not hasattr(ctypes, "WinDLL"):
        raise RuntimeError("XInput probe must run with Windows Python")

    errors: list[str] = []
    for dll_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            dll = ctypes.WinDLL(dll_name)
            function = dll.XInputGetState
            function.argtypes = [ctypes.c_uint32, ctypes.POINTER(XInputState)]
            function.restype = ctypes.c_uint32
            return function, dll_name
        except (OSError, AttributeError) as exc:
            errors.append(f"{dll_name}: {exc}")
    raise RuntimeError("No usable XInput DLL found: " + "; ".join(errors))


def initialize_com() -> tuple[Any, bool]:
    """Initialize COM on the polling thread for XInput's Windows API backend."""
    ole32 = ctypes.WinDLL("ole32.dll")
    co_initialize = ole32.CoInitializeEx
    co_initialize.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    co_initialize.restype = ctypes.c_long
    co_uninitialize = ole32.CoUninitialize
    co_uninitialize.argtypes = []
    co_uninitialize.restype = None

    result = int(co_initialize(None, COINIT_MULTITHREADED)) & 0xFFFFFFFF
    if result in (0, 1):  # S_OK / S_FALSE both require a matching CoUninitialize.
        return ole32, True
    if result == RPC_E_CHANGED_MODE:
        return ole32, False  # The thread already has a different COM apartment.
    raise RuntimeError(f"CoInitializeEx failed with HRESULT 0x{result:08X}")


def _emit(target: Any, record: dict[str, Any]) -> None:
    line = json.dumps(record, separators=(",", ":"), ensure_ascii=False)
    print(line, file=target, flush=True)


def probe(duration_s: float, poll_hz: float, output_path: str | None = None) -> int:
    if duration_s <= 0 or duration_s > 300:
        raise ValueError("duration_s must be in (0, 300]")
    if poll_hz < 1 or poll_hz > 250:
        raise ValueError("poll_hz must be between 1 and 250")

    get_state, dll_name = load_xinput()
    ole32, should_uninitialize_com = initialize_com()
    output = open(output_path, "w", encoding="utf-8", newline="\n") if output_path else sys.stdout
    close_output = output is not sys.stdout
    started_ns = time.perf_counter_ns()
    deadline_ns = started_ns + int(duration_s * 1_000_000_000)
    period_ns = int(1_000_000_000 / poll_hz)
    previous: dict[int, tuple[bool, tuple[int, ...] | None]] = {}
    samples = {index: 0 for index in range(4)}
    changes = {index: 0 for index in range(4)}

    try:
        _emit(output, {
            "event": "probe_start",
            "schema": "piper_xinput_probe_v1",
            "dll": dll_name,
            "duration_s": duration_s,
            "poll_hz": poll_hz,
            "source_clock": "windows_perf_counter_ns",
            "timestamp_ns": started_ns,
            "note": "read_only_no_rumble_no_robot_command",
        })
        next_poll_ns = started_ns
        while next_poll_ns < deadline_ns:
            now_ns = time.perf_counter_ns()
            if now_ns < next_poll_ns:
                time.sleep((next_poll_ns - now_ns) / 1_000_000_000)
            next_poll_ns += period_ns

            for index in range(4):
                state = XInputState()
                result = int(get_state(index, ctypes.byref(state)))
                samples[index] += 1
                if result == ERROR_SUCCESS:
                    gamepad = state.gamepad
                    signature = (
                        int(state.packet_number), int(gamepad.buttons),
                        int(gamepad.left_trigger), int(gamepad.right_trigger),
                        int(gamepad.left_x), int(gamepad.left_y),
                        int(gamepad.right_x), int(gamepad.right_y),
                    )
                    current = (True, signature)
                    snapshot = gamepad_snapshot(gamepad, state.packet_number)
                elif result == ERROR_DEVICE_NOT_CONNECTED:
                    current = (False, None)
                    snapshot = None
                else:
                    current = (False, (result,))
                    snapshot = None

                if previous.get(index) != current:
                    changes[index] += 1
                    record: dict[str, Any] = {
                        "event": "controller_state",
                        "schema": "piper_xinput_probe_v1",
                        "timestamp_ns": time.perf_counter_ns(),
                        "index": index,
                        "connected": current[0],
                    }
                    if snapshot is not None:
                        record.update(snapshot)
                    elif current[1] is not None:
                        record["error_code"] = current[1][0]
                    _emit(output, record)
                    previous[index] = current

        ended_ns = time.perf_counter_ns()
        ever_connected = sorted(
            index for index, current in previous.items() if current[0]
        )
        _emit(output, {
            "event": "probe_summary",
            "schema": "piper_xinput_probe_v1",
            "timestamp_ns": ended_ns,
            "elapsed_s": round((ended_ns - started_ns) / 1_000_000_000, 3),
            "connected_indices_at_end": ever_connected,
            "poll_samples_by_index": samples,
            "state_transitions_by_index": changes,
        })
        return 0 if ever_connected else 2
    finally:
        if close_output:
            output.close()
        if should_uninitialize_com:
            ole32.CoUninitialize()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--poll-hz", type=float, default=50.0)
    parser.add_argument("--jsonl", help="write JSONL output to this path instead of stdout")
    args = parser.parse_args()
    try:
        return probe(args.duration_s, args.poll_hz, args.jsonl)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"XInput probe failed: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
