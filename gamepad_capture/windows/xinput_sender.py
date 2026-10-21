"""Poll XInput on Windows and send read-only gamepad state to ROS over TCP."""

from __future__ import annotations

import argparse
import ctypes
import json
from pathlib import Path
import socket
import sys
import time
import uuid

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from gamepad_capture.protocol import INPUT_SCHEMA, joy_from_xinput, neutral_joy
try:
    from .xinput_probe import (
        ERROR_DEVICE_NOT_CONNECTED,
        XInputState,
        gamepad_snapshot,
        initialize_com,
        load_xinput,
    )
except ImportError:  # Direct script execution from the PowerShell launcher.
    from xinput_probe import (
        ERROR_DEVICE_NOT_CONNECTED,
        XInputState,
        gamepad_snapshot,
        initialize_com,
        load_xinput,
    )


def build_frame(session_id: str, sequence: int, device_index: int,
                source_time_ns: int, state_result: int, state: XInputState) -> dict:
    connected = state_result == 0
    if connected:
        snapshot = gamepad_snapshot(state.gamepad, state.packet_number)
        raw = snapshot["raw"]
        axes, buttons = joy_from_xinput(raw)
        packet_number = int(state.packet_number)
    elif state_result == ERROR_DEVICE_NOT_CONNECTED:
        raw = {
            "buttons_mask": 0, "left_trigger": 0, "right_trigger": 0,
            "left_x": 0, "left_y": 0, "right_x": 0, "right_y": 0,
        }
        axes, buttons = neutral_joy()
        packet_number = None
    else:
        raise RuntimeError(f"XInputGetState failed with code 0x{state_result:08X}")

    return {
        "schema": INPUT_SCHEMA,
        "session_id": session_id,
        "sequence": sequence,
        "device_index": device_index,
        "source_clock": "windows_perf_counter_ns",
        "source_time_ns": source_time_ns,
        "connected": connected,
        "packet_number": packet_number,
        "raw": raw,
        "joy": {"axes": axes, "buttons": buttons},
    }


def run(host: str, port: int, device_index: int, poll_hz: float,
        duration_s: float) -> int:
    if not 1 <= port <= 65535:
        raise ValueError("port must be in 1..65535")
    if device_index not in range(4):
        raise ValueError("device_index must be 0..3")
    if not 1 <= poll_hz <= 100:
        raise ValueError("poll_hz must be in 1..100")
    if duration_s < 0 or duration_s > 3600:
        raise ValueError("duration_s must be 0 (until Ctrl+C) or in 0..3600")

    get_state, dll_name = load_xinput()
    ole32, should_uninitialize_com = initialize_com()
    session_id = uuid.uuid4().hex
    print(json.dumps({
        "event": "sender_start",
        "schema": INPUT_SCHEMA,
        "dll": dll_name,
        "session_id": session_id,
        "device_index": device_index,
        "target": f"{host}:{port}",
        "poll_hz": poll_hz,
        "duration_s": duration_s,
        "can_tx": 0,
        "robot_motion_commanded": False,
    }), flush=True)

    period_ns = int(1_000_000_000 / poll_hz)
    end_ns = time.perf_counter_ns() + int(duration_s * 1_000_000_000) if duration_s else None
    sequence = 0
    next_poll_ns = time.perf_counter_ns()
    connection = None
    last_connection_log = None
    next_connect_ns = 0
    try:
        while end_ns is None or time.perf_counter_ns() < end_ns:
            now_ns = time.perf_counter_ns()
            if now_ns < next_poll_ns:
                time.sleep((next_poll_ns - now_ns) / 1_000_000_000)
            next_poll_ns += period_ns

            state = XInputState()
            result = int(get_state(device_index, ctypes.byref(state)))
            source_time_ns = time.perf_counter_ns()
            frame = build_frame(session_id, sequence, device_index, source_time_ns,
                                result, state)
            sequence += 1

            if connection is None and now_ns >= next_connect_ns:
                try:
                    connection = socket.create_connection((host, port), timeout=0.25)
                    connection.settimeout(0.25)
                    if last_connection_log is not True:
                        print(f"connected to ROS ingress at {host}:{port}", flush=True)
                    last_connection_log = True
                except OSError as exc:
                    next_connect_ns = now_ns + 1_000_000_000
                    if last_connection_log is not False:
                        print(f"waiting for ROS ingress at {host}:{port}: {exc}",
                              file=sys.stderr, flush=True)
                    last_connection_log = False

            if connection is not None:
                try:
                    payload = json.dumps(frame, separators=(",", ":"), allow_nan=False)
                    connection.sendall(payload.encode("utf-8") + b"\n")
                except OSError as exc:
                    connection.close()
                    connection = None
                    next_connect_ns = time.perf_counter_ns() + 1_000_000_000
                    if last_connection_log is not False:
                        print(f"ROS ingress disconnected: {exc}", file=sys.stderr, flush=True)
                    last_connection_log = False

    except KeyboardInterrupt:
        pass
    finally:
        if connection is not None:
            connection.close()
        if should_uninitialize_com:
            ole32.CoUninitialize()
    print(json.dumps({"event": "sender_stopped", "session_id": session_id,
                      "last_sequence": sequence - 1, "can_tx": 0}), flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18901)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--poll-hz", type=float, default=50.0)
    parser.add_argument("--duration-s", type=float, default=0.0,
                        help="0 runs until Ctrl+C")
    args = parser.parse_args()
    try:
        return run(args.host, args.port, args.index, args.poll_hz, args.duration_s)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"XInput sender failed: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
