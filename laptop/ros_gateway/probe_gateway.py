"""Read-only probe for a Windows PiPER gateway control and image endpoint."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import socket
import time

from laptop.ros_gateway.wire_protocol import (
    recv_control_message,
    recv_image_frame,
    send_control_message,
)


def probe(host: str, control_port: int, image_port: int, *, frames: int = 2,
          timeout_s: float = 5.0, expected_role: str = "base") -> dict:
    if not 1 <= control_port <= 65535 or not 1 <= image_port <= 65535:
        raise ValueError("ports must be within 1..65535")
    if type(frames) is not int or not 1 <= frames <= 100:
        raise ValueError("frames must be within 1..100")
    if expected_role not in ("base", "wrist"):
        raise ValueError("expected_role must be base or wrist")
    if not 0.1 <= timeout_s <= 30:
        raise ValueError("timeout must be within 0.1..30 seconds")

    started_ns = time.perf_counter_ns()
    report = {
        "schema": "pi05_ros_gateway_probe_v1",
        "checked_utc": datetime.now(timezone.utc).isoformat(),
        "host": host,
        "control_port": control_port,
        "image_port": image_port,
        "hardware_access": False,
        "can_tx_calls": 0,
    }
    deadline = time.monotonic() + timeout_s
    with socket.create_connection((host, control_port), timeout=timeout_s) as control:
        control.settimeout(timeout_s)
        send_control_message(control, "hello", {
            "client_name": "pi05_ros_gateway_probe",
            "client_utc": report["checked_utc"],
        })
        ack = None
        state = None
        while time.monotonic() < deadline and (ack is None or state is None):
            kind, payload = recv_control_message(control)
            if kind == "hello_ack":
                ack = payload
            elif kind == "state_snapshot":
                state = payload
            elif kind == "fault":
                raise RuntimeError(f"gateway rejected probe: {payload}")
        if ack is None or state is None:
            raise TimeoutError("gateway handshake or state snapshot timed out")

        send_control_message(control, "heartbeat", {
            "session_id": ack.get("session_id", ""),
            "sequence": 1,
        })
        heartbeat_ack = None
        heartbeat_deadline = time.monotonic() + timeout_s
        while time.monotonic() < heartbeat_deadline and heartbeat_ack is None:
            kind, payload = recv_control_message(control)
            if kind == "heartbeat_ack":
                heartbeat_ack = payload
            elif kind == "fault":
                raise RuntimeError(f"gateway rejected heartbeat: {payload}")

    image_results = []
    with socket.create_connection((host, image_port), timeout=timeout_s) as images:
        images.settimeout(timeout_s)
        for _ in range(frames):
            frame = recv_image_frame(images)
            if frame.role != expected_role:
                raise ValueError(
                    f"image role mismatch: expected {expected_role}, got {frame.role}"
                )
            image_results.append({
                "role": frame.role,
                "sequence": frame.sequence,
                "shape": [frame.height, frame.width, 3],
                "encoding": frame.encoding,
                "host_monotonic_ns": frame.host_monotonic_ns,
                "payload_bytes": len(frame.pixels),
                "payload_sha256": hashlib.sha256(frame.pixels).hexdigest(),
            })

    report.update({
        "status": "PASS" if heartbeat_ack is not None and len(image_results) == frames else "FAIL",
        "session_id": ack.get("session_id"),
        "profile": ack.get("profile"),
        "gateway_hardware_access": ack.get("hardware_access"),
        "gateway_can_tx_calls": ack.get("can_tx_calls"),
        "state_snapshot": state,
        "heartbeat_ack": heartbeat_ack,
        "images": image_results,
        "elapsed_ms": round((time.perf_counter_ns() - started_ns) / 1e6, 3),
        "probe_sent_robot_command": False,
    })
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--control-port", type=int, required=True)
    parser.add_argument("--image-port", type=int, required=True)
    parser.add_argument("--frames", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--expected-role", choices=("base", "wrist"), default="base")
    args = parser.parse_args(argv)
    report = probe(
        args.host, args.control_port, args.image_port, frames=args.frames,
        timeout_s=args.timeout, expected_role=args.expected_role,
    )
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
