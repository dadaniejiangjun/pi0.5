"""Windows-only read-only PiPER/camera gateway for the ROS 2 bridge.

This process owns CAN receive and DirectShow cameras while running. It exposes
feedback and newest-image streams only; every command proposal is rejected.
It is intentionally not started by the default fake-gateway command.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import ipaddress
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import queue
import socket
import socketserver
import threading
import time
import uuid

from laptop.ros_gateway.wire_protocol import (
    RawImageFrame,
    estimate_monotonic_clock_offset,
    recv_control_message,
    send_control_message,
    send_image_frame,
)
from laptop.ros_gateway.session_owner_queue import SessionOwnerTaskQueue
from laptop.ros_gateway.control_rpc import read_only_control_result
from laptop.ros_gateway.control_rpc_worker import SessionControlRpcWorker
from laptop.ros_gateway.control_rpc_dispatcher import LocalPositionControlRpcDispatcher
from laptop.ros_gateway.runtime_owner_registry import (
    RuntimeOwnerRegistration,
    runtime_owner_directory,
)


READONLY_CAN_POLL_TIMEOUT_S = 0.05
CONTROL_OWNER_QUEUE_POLL_MAX_S = 0.005


def _can_poll_timeout_s(*, now_ns: int, next_state_ns: int,
                        control_requests_enabled: bool) -> float:
    """Bound owner-queue latency while preserving the read-only poll cadence."""
    timeout_s = READONLY_CAN_POLL_TIMEOUT_S
    if next_state_ns > now_ns:
        timeout_s = min(timeout_s, max(0.001, (next_state_ns - now_ns) / 1_000_000_000))
    if control_requests_enabled:
        timeout_s = min(timeout_s, CONTROL_OWNER_QUEUE_POLL_MAX_S)
    return timeout_s


def validate_hardware_acknowledgements(*, can_owner: str, no_motion: bool,
                                       camera_roles_confirmed: bool,
                                       cameras_enabled: bool = True,
                                       can_enabled: bool = True) -> None:
    if can_enabled and can_owner != "windows":
        raise ValueError("CAN ownership must be explicitly confirmed as windows")
    if can_enabled and not no_motion:
        raise ValueError("read-only CAN profile requires explicit no-motion confirmation")
    if cameras_enabled and not camera_roles_confirmed:
        raise ValueError("camera names and roles must be explicitly confirmed")
    if not can_enabled and not cameras_enabled:
        raise ValueError("at least one hardware input must be enabled")


def _validate_bind_host(host: str) -> None:
    try:
        address = ipaddress.IPv4Address(host)
    except ValueError as exc:
        raise ValueError("gateway bind host must be a concrete IPv4 address") from exc
    if address.is_unspecified or address.is_multicast:
        raise ValueError("gateway must bind to an explicit unicast interface")


def piper_state_to_wire(state: dict, sequence: int) -> dict:
    """Project one complete fresh PiperSession state into the version-1 schema."""
    if state.get("complete_fresh") is not True:
        raise ValueError("only complete fresh PiperSession snapshots may be published")
    raw_joints = state.get("joint_deg_nominal")
    timestamps = state.get("channel_timestamps_ns")
    gripper = state.get("gripper")
    if not isinstance(raw_joints, list) or len(raw_joints) != 6:
        raise ValueError("expected six joint position values")
    if not isinstance(timestamps, dict) or not isinstance(gripper, dict):
        raise ValueError("PiperSession state omitted source timestamps or gripper feedback")
    joint_deg = [float(value) for value in raw_joints]
    if not all(math.isfinite(value) for value in joint_deg):
        raise ValueError("joint state contains non-finite values")
    stamp = lambda can_id: int(timestamps[hex(can_id)])
    gripper_raw = int(gripper["position_raw"])
    oldest_stamp = min(int(value) for value in timestamps.values())
    source_stamp = max(int(value) for value in timestamps.values())
    return {
        "session_id": "",
        "sequence": sequence,
        "host_monotonic_ns": source_stamp,
        "feedback_fresh": True,
        "oldest_feedback_monotonic_ns": oldest_stamp,
        "joint_skew_ns": int(state.get("joint_skew_ns", 0)),
        "status_monotonic_ns": stamp(0x2A1),
        "joint_pair_monotonic_ns": [stamp(0x2A5), stamp(0x2A6), stamp(0x2A7)],
        "gripper_monotonic_ns": stamp(0x2A8),
        "motor_monotonic_ns": [stamp(can_id) for can_id in range(0x261, 0x267)],
        "joint_position_rad": [math.radians(value) for value in joint_deg],
        "gripper_raw": gripper_raw,
        "gripper_nominal_m": gripper_raw / 1_000_000.0,
        "ctrl_mode_raw": int(state["ctrl_mode_raw"]),
        "move_mode_raw": int(state["move_mode_raw"]),
        "teach_status_raw": int(state["teach_status_raw"]),
        "arm_status_raw": int(state["arm_status_raw"]),
        "error_code": int(state["error_code"]),
        "gripper_status_raw": int(gripper.get("status_raw", 0)),
        "motor_enabled": [bool(value) for value in state["motor_enabled"]],
        "hardware_connected": True,
        "fake": False,
        "units_hardware_verified": bool(state.get("units_hardware_verified", False)),
    }


class _ThreadingServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, handler, owner, role=None):
        self.owner = owner
        self.role = role
        super().__init__(address, handler)


class _ControlHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        owner = self.server.owner
        if not owner.control_client_slot.acquire(blocking=False):
            return
        owner._register_control_socket(self.request)
        try:
            self._serve(owner)
        finally:
            owner._unregister_control_socket(self.request)
            owner.control_client_slot.release()

    def _serve(self, owner) -> None:
        sock = self.request
        sock.settimeout(5.0)
        try:
            kind, hello = recv_control_message(sock)
            if kind != "hello":
                send_control_message(sock, "fault", {"reason": "hello_required"})
                return
            sock.settimeout(None)
            session_id = uuid.uuid4().hex
            set_active_session = getattr(owner, "set_active_control_session", None)
            if callable(set_active_session):
                set_active_session(session_id)
            update_owner = getattr(owner, "update_runtime_owner", None)
            if callable(update_owner):
                update_owner(
                    status="connected",
                    session_id=session_id,
                    client_name=str(hello.get("client_name", "unknown"))[:64],
                    client_address=str(self.client_address[0]),
                )
            send_lock = threading.Lock()

            def send(kind, payload):
                with send_lock:
                    send_control_message(sock, kind, payload)

            camera_identity = owner.camera_identity
            can_enabled = bool(getattr(owner, "can_enabled", True))
            cameras_enabled = bool(getattr(owner, "cameras_enabled", False))
            send("hello_ack", {
                "protocol_version": 1,
                "session_id": session_id,
                "profile": getattr(owner, "profile", "windows_read_only" if can_enabled else "windows_camera_only"),
                "hardware_access": bool(getattr(owner, "hardware_access", True)),
                "hardware_commands_enabled": False,
                "can_tx_calls": 0,
                "can_access_enabled": can_enabled,
                "camera_access_enabled": cameras_enabled,
                "camera_identity": camera_identity,
                "peer_name": str(hello.get("client_name", "unknown"))[:64],
                "gateway_monotonic_ns": time.perf_counter_ns(),
            })
            stopped = threading.Event()

            def publish_updates():
                state_sequence = -1
                last_status_ns = 0
                sent_count = 0
                skipped_count = 0
                last_send_ms = 0.0
                last_snapshot_wait_ms = 0.0
                metric = getattr(owner, "record_control_session_metric", None)
                try:
                    while not stopped.is_set() and not owner.stop_event.is_set():
                        now = time.perf_counter_ns()
                        status = owner.status_snapshot(now)
                        if now - last_status_ns >= 500_000_000:
                            send("gateway_status", {
                                "session_id": session_id,
                                **status,
                                "tcp_state_sent_count": sent_count,
                                "tcp_state_skipped_count": skipped_count,
                                "tcp_state_last_sequence": max(0, state_sequence),
                                "tcp_state_last_send_call_ms": last_send_ms,
                                "tcp_state_last_snapshot_wait_ms": last_snapshot_wait_ms,
                            })
                            if callable(metric):
                                metric(session_id, "gateway_status_sent")
                            last_status_ns = now
                        state = owner.latest_state()
                        if callable(metric):
                            metric(
                                session_id, "state_checked",
                                latest_sequence=(state.get("sequence") if state else None),
                                latest_available=state is not None,
                            )
                        if state is not None and state["sequence"] != state_sequence:
                            payload = dict(state)
                            payload["session_id"] = session_id
                            record_state = getattr(owner, "record_control_session_state", None)
                            cache_recorded = bool(record_state(session_id, payload)) if callable(record_state) else False
                            if callable(metric):
                                metric(
                                    session_id, "state_cache_attempted",
                                    cache_recorded=cache_recorded,
                                    latest_sequence=state["sequence"],
                                )
                            send_started_ns = time.perf_counter_ns()
                            generated_ns = payload.get("gateway_snapshot_monotonic_ns")
                            if type(generated_ns) is int:
                                last_snapshot_wait_ms = (send_started_ns - generated_ns) / 1e6
                            send("state_snapshot", payload)
                            last_send_ms = (time.perf_counter_ns() - send_started_ns) / 1e6
                            sent_count += 1
                            if state_sequence >= 0:
                                skipped_count += max(0, state["sequence"] - state_sequence - 1)
                            state_sequence = state["sequence"]
                            if callable(metric):
                                metric(
                                    session_id, "state_sent",
                                    sent_state_count=sent_count,
                                    sent_sequence=state_sequence,
                                )
                        changed = getattr(owner, "state_changed", None)
                        if changed is None:
                            stopped.wait(0.02)
                        else:
                            changed.wait(0.02)
                            changed.clear()
                except Exception as exc:
                    if callable(metric):
                        metric(
                            session_id, "writer_error",
                            writer_error=f"{type(exc).__name__}: {exc}"[:256],
                        )
                    try:
                        send("fault", {
                            "session_id": session_id,
                            "reason": "gateway_state_publisher_failed",
                        })
                    except Exception:
                        pass

            writer = threading.Thread(target=publish_updates, daemon=True)
            writer.start()
            clock_challenges: dict[int, tuple[int, int, int]] = {}
            control_request_ids: set[str] = set()
            handle_rpc = getattr(owner, "control_request_result", None)
            rpc_worker = (SessionControlRpcWorker(
                handle_rpc=handle_rpc, send=send, session_id=session_id,
                used_request_ids=control_request_ids, stopped=stopped,
            ) if callable(handle_rpc) else None)
            last_heartbeat_sequence = 0
            last_client_monotonic_ns = 0
            last_completed_clock_sequence = 0
            try:
                while not stopped.is_set() and not owner.stop_event.is_set():
                    kind, payload = recv_control_message(sock)
                    if kind == "heartbeat":
                        sequence = payload.get("sequence")
                        client_send_ns = payload.get("client_monotonic_ns")
                        if (payload.get("session_id") != session_id
                                or type(sequence) is not int
                                or sequence <= last_heartbeat_sequence
                                or type(client_send_ns) is not int
                                or client_send_ns <= last_client_monotonic_ns):
                            send("fault", {
                                "session_id": session_id,
                                "reason": "invalid_or_replayed_heartbeat",
                            })
                            return
                        server_receive_ns = time.perf_counter_ns()
                        server_send_ns = time.perf_counter_ns()
                        clock_challenges[sequence] = (
                            client_send_ns, server_receive_ns, server_send_ns,
                        )
                        for old_sequence in sorted(clock_challenges)[:-8]:
                            clock_challenges.pop(old_sequence, None)
                        last_heartbeat_sequence = sequence
                        last_client_monotonic_ns = client_send_ns
                        send("heartbeat_ack", {
                            "session_id": session_id,
                            "sequence": sequence,
                            "client_monotonic_ns": client_send_ns,
                            "gateway_receive_monotonic_ns": server_receive_ns,
                            "gateway_monotonic_ns": server_send_ns,
                        })
                    elif kind == "clock_sync_complete":
                        sequence = payload.get("sequence")
                        challenge = clock_challenges.get(sequence) if type(sequence) is int else None
                        accepted = False
                        reason = "clock_sync_challenge_missing_or_replayed"
                        estimate = None
                        if (payload.get("session_id") == session_id
                                and type(sequence) is int
                                and sequence > last_completed_clock_sequence
                                and challenge is not None):
                            client_send_ns, server_receive_ns, server_send_ns = challenge
                            if (payload.get("client_monotonic_ns") == client_send_ns
                                    and payload.get("gateway_receive_monotonic_ns") == server_receive_ns
                                    and payload.get("gateway_send_monotonic_ns") == server_send_ns):
                                client_receive_ns = payload.get("client_receive_monotonic_ns")
                                try:
                                    estimate = estimate_monotonic_clock_offset(
                                        client_send_ns=client_send_ns,
                                        server_receive_ns=server_receive_ns,
                                        server_send_ns=server_send_ns,
                                        client_receive_ns=client_receive_ns,
                                    )
                                    server_now_ns = time.perf_counter_ns()
                                    if (server_now_ns - server_send_ns > 1_500_000_000
                                            or client_receive_ns - client_send_ns > 1_500_000_000):
                                        estimate = None
                                        reason = "clock_sync_sample_expired"
                                    else:
                                        accepted = True
                                        reason = "accepted"
                                        last_completed_clock_sequence = sequence
                                except (TypeError, ValueError) as exc:
                                    reason = f"invalid_clock_sync_sample:{exc}"[:192]
                            else:
                                reason = "clock_sync_echo_does_not_match_challenge"
                            clock_challenges.pop(sequence, None)
                        result = {
                            "session_id": session_id,
                            "sequence": sequence if type(sequence) is int else 0,
                            "accepted": accepted,
                            "reason": reason,
                        }
                        if estimate is not None and accepted:
                            result.update({
                                "server_minus_client_offset_ns": estimate.server_minus_client_offset_ns,
                                "round_trip_ns": estimate.round_trip_ns,
                                "uncertainty_ns": estimate.uncertainty_ns,
                            })
                        send("clock_sync_ack", result)
                    elif kind == "command_proposal":
                        send("command_receipt", {
                            "session_id": session_id,
                            "request_id": str(payload.get("request_id", ""))[:128],
                            "decision": "REJECTED_READ_ONLY_PROFILE_NO_TX",
                            "reason": "Windows read-only profile has no command path",
                            "can_tx_calls": 0,
                        })
                    elif kind == "control_request":
                        if rpc_worker is not None:
                            rpc_worker.submit(payload)
                        else:
                            # Keep minimal in-process test owners and any older
                            # default owner fail-closed if they lack injection.
                            result = read_only_control_result(
                                payload, session_id=session_id,
                                used_request_ids=control_request_ids,
                            )
                            send("control_result", result)
                    else:
                        send("fault", {
                            "session_id": session_id,
                            "reason": "message_not_supported_by_read_only_gateway",
                        })
                        return
            except (EOFError, OSError, ValueError):
                pass
            finally:
                stopped.set()
                writer.join(timeout=0.5)
                if callable(update_owner):
                    update_owner(status="listening", session_id=None,
                                 client_name=None, client_address=None)
                close_session = getattr(owner, "control_session_closed", None)
                if callable(close_session):
                    close_session(session_id)
                if rpc_worker is not None:
                    rpc_worker.close()
        except (EOFError, OSError, ValueError):
            return


class _ImageHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        owner = self.server.owner
        role = self.server.role
        if not owner.image_client_slots[role].acquire(blocking=False):
            return
        last_sequence = -1
        next_send_ns = 0
        # Send the newest raw frame at the configured camera stream cadence.
        # The ROS bridge and policy node apply their own independent rates.
        period_ns = owner.image_send_period_ns
        self.request.settimeout(0.5)
        try:
            while not owner.stop_event.is_set():
                now = time.perf_counter_ns()
                frame = owner.latest_frame(role)
                if frame is None or frame["sequence"] == last_sequence:
                    owner.stop_event.wait(0.01)
                    continue
                if now < next_send_ns:
                    owner.stop_event.wait((next_send_ns - now) / 1_000_000_000)
                    continue
                scheduled_ns = next_send_ns or now
                image = frame["frame"]
                height, width = image.shape[:2]
                stride = int(image.strides[0])
                send_image_frame(self.request, RawImageFrame(
                    role=role,
                    width=int(width),
                    height=int(height),
                    stride=stride,
                    sequence=int(frame["sequence"]),
                    host_monotonic_ns=int(frame["host_monotonic_ns"]),
                    pixels=image.tobytes(order="C"),
                ))
                sent_ns = time.perf_counter_ns()
                last_sequence = int(frame["sequence"])
                # Keep a fixed image-send time grid. Polling or TCP send jitter does
                # not accumulate into every later frame; if a write misses a
                # slot, skip it instead of bursting queued images.
                next_send_ns = _advance_image_deadline_ns(
                    scheduled_ns, sent_ns, period_ns
                )
        except (OSError, ValueError):
            return
        finally:
            owner.image_client_slots[role].release()


def _camera_process_target(index: int | None, role: str, output_queue, stop_event,
                           source: str = "directshow",
                           expected_oak_device_id: str | None = None,
                           capture_fps: float = 30.0) -> None:
    """Own one DirectShow or OAK RGB source in a killable process."""
    try:
        if source == "directshow":
            from laptop.camera.source import open_camera, request_capture_mode
            cap = open_camera(index, backend_name="DSHOW")
            try:
                if not cap.isOpened():
                    raise RuntimeError(f"{role} DirectShow camera did not open")
                request_capture_mode(cap, width=640, height=480, fps=capture_fps)
                sequence = 0
                while not stop_event.is_set():
                    ok, frame = cap.read()
                    stamp = time.perf_counter_ns()
                    if not ok or frame is None:
                        raise RuntimeError(f"{role} DirectShow camera read failed")
                    _validate_camera_frame(role, frame)
                    sequence += 1
                    _put_latest_frame(output_queue, (sequence, stamp, frame.copy()))
            finally:
                cap.release()
            return

        if source != "oak-d-pro" or role != "base":
            raise ValueError("camera source must be DirectShow, or OAK-D Pro for Base")
        if not isinstance(expected_oak_device_id, str) or not expected_oak_device_id.strip():
            raise ValueError("OAK-D Pro Base requires an explicit expected device ID")
        from laptop.camera.oak_rgbd import OakRGBD
        camera = OakRGBD(width=640, height=480, fps=capture_fps)
        try:
            actual_id = str(camera.device_info.get("device_id", ""))
            if actual_id.casefold() != expected_oak_device_id.casefold():
                raise RuntimeError(
                    "connected DepthAI device ID does not match the configured OAK-D Pro"
                )
            camera.start()
            while not stop_event.is_set():
                sample = camera.read(timeout_s=2.0)
                if sample is None:
                    raise TimeoutError("OAK-D Pro RGB-D source timed out")
                frame = sample.rgb_bgr
                _validate_camera_frame(role, frame)
                _put_latest_frame(output_queue, (
                    int(sample.sequence_num),
                    int(sample.received_monotonic_ns),
                    frame.copy(),
                ))
        finally:
            camera.close()
    except KeyboardInterrupt:
        # Ctrl+C reaches child processes too on Windows consoles. Treat it as
        # an orderly camera stop so the parent can close the gateway cleanly.
        pass


def _validate_camera_frame(role: str, frame) -> None:
    if (getattr(frame, "ndim", None) != 3 or frame.shape[2] != 3
            or getattr(getattr(frame, "dtype", None), "name", None) != "uint8"):
        raise ValueError(f"{role} camera returned an unsupported image format")


def _resolve_gateway_camera_identity(devices, *, base_source: str,
                                     base_name: str, wrist_name: str,
                                     oak_device_id: str | None) -> dict:
    """Resolve current camera roles without treating an OpenCV index as identity."""
    from laptop.camera.dual_camera import resolve_roles

    if base_source == "directshow":
        if oak_device_id is not None:
            raise ValueError("--oak-device-id is only valid with --base-source oak-d-pro")
        return resolve_roles(devices, base_name=base_name, wrist_name=wrist_name)
    if base_source != "oak-d-pro":
        raise ValueError("base_source must be 'directshow' or 'oak-d-pro'")
    if not isinstance(oak_device_id, str) or not oak_device_id.strip():
        raise ValueError("OAK-D Pro Base requires an explicit --oak-device-id")
    wrist_matches = [
        item for item in devices
        if str(item.get("Name", "")).casefold() == wrist_name.casefold()
    ]
    if len(wrist_matches) != 1:
        raise ValueError(
            f"wrist: expected one DirectShow device named {wrist_name!r}, got {len(wrist_matches)}"
        )
    wrist = wrist_matches[0]
    return {
        "base": {
            "device_name": "OAK-D Pro RGB",
            "device_path": f"DepthAI:{oak_device_id}",
            "expected_device_id": oak_device_id,
            "binding": "DepthAI_expected_device_id_checked_before_streaming",
            "image_stream": "RGB BGR8; aligned depth is not sent on the ROS Base topic",
        },
        "wrist": {
            "opencv_index": wrist["Index"],
            "device_name": wrist["Name"],
            "device_path": wrist.get("DevicePath"),
            "binding": "current_DSHOW_name_enumeration",
        },
    }


def _put_latest_frame(output_queue, item) -> None:
    """Keep at most one pending image and prefer the newest captured frame."""
    try:
        output_queue.put_nowait(item)
    except queue.Full:
        try:
            output_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            output_queue.put_nowait(item)
        except queue.Full:
            pass


def _advance_image_deadline_ns(scheduled_ns: int, sent_ns: int,
                               period_ns: int = 33_333_333) -> int:
    """Advance the fixed image-send schedule and skip slots missed by a write."""
    if period_ns <= 0:
        raise ValueError("image send period must be positive")
    deadline_ns = scheduled_ns + period_ns
    if deadline_ns <= sent_ns:
        deadline_ns += ((sent_ns - deadline_ns) // period_ns + 1) * period_ns
    return deadline_ns


class WindowsReadOnlyGateway:
    """Windows owner for PiperSession receive and selectable Base/Wrist cameras."""

    def __init__(self, *, host: str, control_port: int,
                 base_image_port: int | None, wrist_image_port: int | None,
                 log_path: Path,
                 base_name: str = "ACER HD User Facing",
                 wrist_name: str = "Dabai DC1",
                 base_source: str = "directshow",
                 oak_device_id: str | None = None,
                 image_stream_hz: float = 30.0,
                 camera_capture_fps: float = 30.0,
                 state_snapshot_hz: float = 10.0,
                 can_enabled: bool = True,
                 raw_can_log: bool = False,
                  owner_registration: RuntimeOwnerRegistration | None = None,
                 local_position_target_factory=None,
                 local_control_session_authorizer=None,
                 local_position_session_authority=None,
                 local_session_factory=None) -> None:
        _validate_bind_host(host)
        if type(control_port) is not int or not 1 <= control_port <= 65535:
            raise ValueError("control port must be within 1..65535")
        camera_ports = (base_image_port, wrist_image_port)
        if any(port is None for port in camera_ports) and not all(
            port is None for port in camera_ports
        ):
            raise ValueError("both camera ports must be set or both cameras disabled")
        self.cameras_enabled = base_image_port is not None
        self.can_enabled = bool(can_enabled)
        if type(raw_can_log) is not bool:
            raise TypeError("raw_can_log must be a bool")
        self.raw_can_log = raw_can_log
        if local_session_factory is not None and not callable(local_session_factory):
            raise TypeError("local_session_factory must be callable")
        self.local_session_factory = local_session_factory
        if not self.cameras_enabled and not self.can_enabled:
            raise ValueError("at least one hardware input must be enabled")
        ports = (control_port, *(() if not self.cameras_enabled else camera_ports))
        if any(type(port) is not int or not 1 <= port <= 65535 for port in ports):
            raise ValueError("enabled ports must be within 1..65535")
        if len(set(ports)) != len(ports):
            raise ValueError("control and image ports must be unique")
        self.host = host
        self.control_port = control_port
        self.image_ports = {"base": base_image_port, "wrist": wrist_image_port}
        image_rate = float(image_stream_hz)
        if not math.isfinite(image_rate) or not 0.0 < image_rate <= 120.0:
            raise ValueError("image_stream_hz must be finite and within (0, 120]")
        self.image_stream_hz = image_rate
        self.image_send_period_ns = int(1_000_000_000 / image_rate)
        capture_rate = float(camera_capture_fps)
        if not math.isfinite(capture_rate) or not 0.0 < capture_rate <= 120.0:
            raise ValueError("camera_capture_fps must be finite and within (0, 120]")
        self.camera_capture_fps = capture_rate
        state_rate = float(state_snapshot_hz)
        if not math.isfinite(state_rate) or not 0.0 < state_rate <= 50.0:
            raise ValueError("state_snapshot_hz must be finite and within (0, 50]")
        self.state_snapshot_hz = state_rate
        self.state_snapshot_period_ns = int(1_000_000_000 / state_rate)
        self.log_path = Path(log_path)
        self.owner_registration = owner_registration
        self.base_name = base_name
        self.wrist_name = wrist_name
        if base_source not in {"directshow", "oak-d-pro"}:
            raise ValueError("base_source must be 'directshow' or 'oak-d-pro'")
        if base_source == "oak-d-pro" and not oak_device_id:
            raise ValueError("OAK-D Pro Base requires an explicit device ID")
        if base_source == "directshow" and oak_device_id is not None:
            raise ValueError("oak_device_id can only be used with OAK-D Pro Base")
        self.base_source = base_source
        self.oak_device_id = oak_device_id
        self.stop_event = threading.Event()
        self.state_changed = threading.Event()
        self.lock = threading.Lock()
        self._state = None
        self._recent_states = deque(maxlen=4)
        self._session_state_sources: dict[str, deque] = {}
        self._control_session_metrics: dict[str, dict] = {}
        self._state_sequence = 0
        self._can_connected = False
        self._can_error = (
            "CAN reader has not started" if self.can_enabled
            else "CAN access disabled by camera-only profile"
        )
        self._last_state_ns = 0
        self._state_oldest_ns = 0
        self._camera_health = {"base": False, "wrist": False}
        self._camera_error = {"base": "not started", "wrist": "not started"}
        self.camera_identity = {}
        self._frames = {}
        self._frame_sequences = {"base": 0, "wrist": 0}
        self.image_client_slots = {
            role: threading.BoundedSemaphore(1)
            for role, port in self.image_ports.items() if port is not None
        }
        self._servers = []
        self._served_servers = []
        self._server_threads = []
        self._workers = []
        self.control_client_slot = threading.BoundedSemaphore(1)
        self.active_control_sockets = set()
        self._active_control_session_id = ""
        self._camera_processes = {}
        self._camera_stop_events = {}
        self._camera_queues = {}
        # The CLI does not configure these callbacks. The only optional
        # dispatcher is process-local and still requires a locally authorized
        # TCP session, an active typed task scope, fresh source state, and a
        # typed target factory. The normal profile remains reject-only.
        self.task_request_queue = SessionOwnerTaskQueue()
        if (local_control_session_authorizer is not None
                and local_position_session_authority is not None):
            raise ValueError("choose one local position-session authorization source")
        self._local_position_session_authority = local_position_session_authority
        if local_position_session_authority is not None:
            local_control_session_authorizer = getattr(
                local_position_session_authority, "authorize_session", None
            )
            self._local_session_revoker = getattr(
                local_position_session_authority, "revoke_session", None
            )
            if (not callable(local_control_session_authorizer)
                    or not callable(self._local_session_revoker)):
                raise TypeError(
                    "local position-session authority needs authorize_session and revoke_session"
                )
        else:
            self._local_session_revoker = None
        if ((local_position_target_factory is None)
                != (local_control_session_authorizer is None)):
            raise ValueError(
                "local target factory and local session authorizer must be supplied together"
            )
        self.control_rpc_dispatcher = None
        if local_position_target_factory is not None:
            self.control_rpc_dispatcher = LocalPositionControlRpcDispatcher(
                task_request_queue=self.task_request_queue,
                latest_state_provider=self.latest_state,
                source_state_provider=self.recent_fresh_state,
                session_authorizer=local_control_session_authorizer,
                target_factory=local_position_target_factory,
            )

    def update_runtime_owner(self, *, status: str, session_id: str | None = None,
                             **details) -> bool:
        """Best-effort update of the shared, process-keyed owner record."""
        registration = self.owner_registration
        if registration is None:
            return False
        try:
            registration.update(status=status, session_id=session_id, **details)
            return True
        except (OSError, TypeError, ValueError):
            return False

    def control_request_result(self, payload, *, session_id: str,
                               used_request_ids: set[str]) -> dict:
        """Route to an injected local dispatcher or preserve the read-only default."""
        dispatcher = self.control_rpc_dispatcher
        if dispatcher is None:
            return read_only_control_result(
                payload, session_id=session_id,
                used_request_ids=used_request_ids,
            )
        return dispatcher.dispatch(
            payload, session_id=session_id,
            used_request_ids=used_request_ids,
        )

    def control_session_closed(self, session_id: str) -> None:
        """Revoke the optional local position binding when TCP disconnects."""
        with self.lock:
            self._session_state_sources.pop(session_id, None)
            if self._active_control_session_id == session_id:
                self._active_control_session_id = ""
        revoker = self._local_session_revoker
        if callable(revoker):
            revoker(session_id)

    @property
    def active_control_session_id(self) -> str:
        with self.lock:
            return self._active_control_session_id

    def set_active_control_session(self, session_id: str) -> None:
        if not isinstance(session_id, str) or not session_id or not session_id.isascii():
            raise ValueError("active control session ID must be bounded ASCII text")
        with self.lock:
            self._active_control_session_id = session_id

    def record_control_session_metric(self, session_id: str, event: str, **fields) -> None:
        """Retain bounded per-session TCP state-publication diagnostics."""
        if (not isinstance(session_id, str) or not session_id
                or not isinstance(event, str) or not event):
            return
        with self.lock:
            row = self._control_session_metrics.setdefault(
                session_id, {"event_counts": {},}
            )
            counts = row.setdefault("event_counts", {})
            counts[event] = counts.get(event, 0) + 1
            row.update(fields)
            while len(self._control_session_metrics) > 8:
                oldest_session = next(iter(self._control_session_metrics))
                if oldest_session == session_id:
                    break
                self._control_session_metrics.pop(oldest_session, None)

    def control_session_diagnostics(self, session_id: str) -> dict:
        """Return the retained diagnostics for one TCP control session."""
        with self.lock:
            row = self._control_session_metrics.get(session_id)
            if row is None:
                return {}
            return {**row, "event_counts": dict(row.get("event_counts", {}))}

    def _register_control_socket(self, sock) -> None:
        with self.lock:
            self.active_control_sockets.add(sock)

    def _unregister_control_socket(self, sock) -> None:
        with self.lock:
            self.active_control_sockets.discard(sock)

    def start(self) -> "WindowsReadOnlyGateway":
        identities = {}
        if self.cameras_enabled:
            if self.base_source == "oak-d-pro":
                try:
                    import depthai  # noqa: F401 - dependency preflight only
                except Exception as exc:
                    raise RuntimeError(
                        "OAK-D Pro Base requires DepthAI v3 in this gateway Python environment"
                    ) from exc
            # Resolve role names before camera workers open devices. DirectShow
            # indices and the OAK serial must match this run's explicit profile.
            from laptop.camera.dual_camera import inventory
            identities = _resolve_gateway_camera_identity(
                inventory(),
                base_source=self.base_source,
                base_name=self.base_name,
                wrist_name=self.wrist_name,
                oak_device_id=self.oak_device_id,
            )
        self.camera_identity = identities
        self._workers = []
        if self.can_enabled:
            self._workers.append(threading.Thread(
                target=self._can_worker, name="piper-session-owner", daemon=True
            ))
        endpoints = [((self.host, self.control_port), _ControlHandler, None)]
        if self.cameras_enabled:
            endpoints.extend((
                ((self.host, self.image_ports["base"]), _ImageHandler, "base"),
                ((self.host, self.image_ports["wrist"]), _ImageHandler, "wrist"),
            ))
        try:
            for address, handler, role in endpoints:
                self._servers.append(_ThreadingServer(address, handler, self, role))
        except OSError:
            for server in self._servers:
                server.server_close()
            self._servers.clear()
            raise
        # Publish the owner state after bind succeeds but before accepting a
        # client, so a fast bridge connection cannot be overwritten by startup.
        self.update_runtime_owner(status="listening", session_id=None)
        for server in self._servers:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self._server_threads.append(thread)
            self._served_servers.append(server)
        for worker in self._workers:
            worker.start()
        if self.cameras_enabled:
            context = mp.get_context("spawn")
            for role in ("base", "wrist"):
                output_queue = context.Queue(maxsize=1)
                child_stop = context.Event()
                process = context.Process(
                    target=_camera_process_target,
                    args=(
                        identities[role].get("opencv_index"), role, output_queue, child_stop,
                        self.base_source if role == "base" else "directshow",
                        self.oak_device_id if role == "base" else None,
                        self.camera_capture_fps,
                    ),
                    name=f"pi05-camera-{role}",
                    daemon=True,
                )
                self._camera_queues[role] = output_queue
                self._camera_stop_events[role] = child_stop
                self._camera_processes[role] = process
                process.start()
                reader = threading.Thread(
                    target=self._camera_queue_reader,
                    args=(role, process, output_queue),
                    name=f"camera-frame-reader-{role}",
                    daemon=True,
                )
                self._workers.append(reader)
                reader.start()
        return self

    def latest_state(self):
        with self.lock:
            if self._state is None:
                return None
            if time.perf_counter_ns() - self._state_oldest_ns > 120_000_000:
                return None
            return dict(self._state)

    def record_control_session_state(self, session_id: str, state: dict) -> bool:
        """Retain only feedback snapshots actually published in this TCP session."""
        if not isinstance(session_id, str) or not session_id or not isinstance(state, dict):
            return False
        if state.get("session_id") != session_id:
            return False
        oldest = state.get("oldest_feedback_monotonic_ns")
        if type(oldest) is not int or oldest <= 0:
            return False
        with self.lock:
            cache = self._session_state_sources.setdefault(session_id, deque(maxlen=4))
            cache.append((dict(state), oldest))
        return True

    def recent_fresh_state(self, sequence, session_id=None):
        """Resolve a fresh source retained in the requesting TCP session.

        Retain the same 120 ms oldest-feedback age bound, including motor/status
        timestamps. Cross-session source reuse is rejected. ``sequence=None``
        selects the newest fresh source from that session; this avoids racing
        a newly aggregated local CAN sequence that the TCP publisher has not
        sent yet.
        """
        if sequence is None and not session_id:
            return None
        if sequence is not None and type(sequence) is not int:
            return None
        now = time.perf_counter_ns()
        with self.lock:
            cache = (self._recent_states if session_id is None
                     else self._session_state_sources.get(session_id, ()))
            for wire, oldest in reversed(cache):
                if ((sequence is None or wire["sequence"] == sequence)
                        and 0 <= now-oldest <= 120_000_000):
                    if session_id is not None and wire.get("session_id") != session_id:
                        return None
                    return dict(wire)
        return None

    def recent_control_source(self, sequence, session_id):
        """Resolve a published observation within the 250 ms RPC lifetime.

        The observation must have been fresh when published. Hardware feedback
        used for dispatch is separately read through latest_state (120 ms).
        """
        now = time.perf_counter_ns()
        with self.lock:
            for wire, oldest in reversed(self._session_state_sources.get(session_id, ())):
                stamp = wire.get("gateway_snapshot_monotonic_ns")
                if (wire.get("sequence") == sequence
                        and wire.get("session_id") == session_id
                        and wire.get("feedback_fresh") is True
                        and type(stamp) is int
                        and 0 <= stamp - oldest <= 120_000_000
                        and 0 <= now - stamp <= 250_000_000):
                    return dict(wire)
        return None

    def latest_frame(self, role):
        with self.lock:
            value = self._frames.get(role)
            if value is None:
                return None
            result = dict(value)
            result["frame"] = value["frame"].copy()
            return result

    def status_snapshot(self, now_ns: int) -> dict:
        with self.lock:
            state_fresh = (
                self._state is not None
                and 0 <= now_ns - self._state_oldest_ns <= 120_000_000
            )
            cameras_ready = self.cameras_enabled and all(
                self._camera_health[role]
                and role in self._frames
                and 0 <= now_ns - self._frames[role]["host_monotonic_ns"] <= 250_000_000
                for role in ("base", "wrist")
            )
            reason = "read-only hardware feedback and cameras ready"
            if not self._can_connected:
                reason = self._can_error
            elif not state_fresh:
                reason = "CAN connected but complete feedback is stale or incomplete"
            elif self.cameras_enabled and not cameras_ready:
                reason = "; ".join(self._camera_error.values())
            elif not self.cameras_enabled:
                reason = "read-only hardware feedback ready; cameras disabled"
            return {
                "transport_connected": True,
                "hardware_connected": bool(self._can_connected and state_fresh),
                "feedback_fresh": state_fresh,
                "camera_ready": cameras_ready,
                "camera_enabled": self.cameras_enabled,
                "can_access_enabled": self.can_enabled,
                "fake": False,
                "can_tx_calls": 0,
                "reason": reason,
            }

    def _can_worker(self) -> None:
        from laptop.robot_client.piper_session import PiperSession
        session_factory = self.local_session_factory or PiperSession
        session = session_factory(
            log_path=self.log_path, log_rx_frames=self.raw_can_log,
        )
        try:
            self.task_request_queue.claim_owner_thread()
            session.connect()
            with self.lock:
                self._can_connected = True
                self._can_error = "CAN session connected; waiting for complete fresh feedback"
            state_period_ns = self.state_snapshot_period_ns
            next_state_ns = 0
            while not self.stop_event.is_set():
                poll_started_ns = time.perf_counter_ns()
                poll_timeout_s = _can_poll_timeout_s(
                    now_ns=poll_started_ns,
                    next_state_ns=next_state_ns,
                    control_requests_enabled=self.control_rpc_dispatcher is not None,
                )
                session.poll(poll_timeout_s)
                self.task_request_queue.pump_one(session)
                now = time.perf_counter_ns()
                if now < next_state_ns:
                    continue
                if next_state_ns == 0:
                    next_state_ns = now + state_period_ns
                else:
                    next_state_ns += state_period_ns
                    if next_state_ns <= now:
                        missed_periods = (now - next_state_ns) // state_period_ns + 1
                        next_state_ns += missed_periods * state_period_ns
                state = session.state(max_age_ms=120)
                if state.get("complete_fresh") is not True:
                    continue
                with self.lock:
                    self._state_sequence += 1
                    wire = piper_state_to_wire(state, self._state_sequence)
                    wire["gateway_snapshot_monotonic_ns"] = now
                session.record_state_snapshot(wire)
                with self.lock:
                    self._state = wire
                    stamps = [wire["status_monotonic_ns"], *wire["joint_pair_monotonic_ns"],
                              wire["gripper_monotonic_ns"], *wire["motor_monotonic_ns"]]
                    self._last_state_ns = now
                    self._state_oldest_ns = min(stamps)
                    self._recent_states.append((dict(wire), min(stamps)))
                self.state_changed.set()
        except Exception as exc:
            with self.lock:
                self._can_connected = False
                self._can_error = f"read-only CAN session failed: {type(exc).__name__}: {exc}"
        finally:
            try:
                self.task_request_queue.abort_active_scope(session)
            except Exception:
                pass
            self.task_request_queue.close()
            try:
                session.close()
            except Exception as exc:
                with self.lock:
                    self._can_connected = False
                    self._can_error = f"CAN close failed: {type(exc).__name__}: {exc}"

    def _camera_queue_reader(self, role: str, process, output_queue) -> None:
        while not self.stop_event.is_set():
            try:
                sequence, stamp, frame = output_queue.get(timeout=0.1)
                with self.lock:
                    self._frames[role] = {
                        "frame": frame,
                        "sequence": int(sequence),
                        "host_monotonic_ns": int(stamp),
                    }
                    self._camera_health[role] = True
                    self._camera_error[role] = "ready"
            except queue.Empty:
                if not process.is_alive():
                    with self.lock:
                        self._camera_health[role] = False
                        self._camera_error[role] = (
                            f"{role} camera process exited with code {process.exitcode}"
                        )
                    return

    def close(self) -> None:
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        self.task_request_queue.close()
        with self.lock:
            active_sockets = tuple(self.active_control_sockets)
            self.active_control_sockets.clear()
        for sock in active_sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        for event in self._camera_stop_events.values():
            event.set()
        for server in self._servers:
            if server in self._served_servers:
                server.shutdown()
            server.server_close()
        for process in self._camera_processes.values():
            process.join(timeout=2.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
            if process.is_alive() and hasattr(process, "kill"):
                process.kill()
                process.join(timeout=1.0)
        for thread in self._server_threads:
            thread.join(timeout=1.0)
        for worker in self._workers:
            worker.join(timeout=2.0)
        for output_queue in self._camera_queues.values():
            output_queue.close()
        self._servers.clear()
        self._served_servers.clear()
        self._server_threads.clear()
        self._workers.clear()
        self._camera_processes.clear()
        self._camera_stop_events.clear()
        self._camera_queues.clear()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("windows-read-only",), required=True)
    parser.add_argument(
        "--deployment-profile", choices=("p3_p4_shadow",), default="p3_p4_shadow",
        help="shared ROS runtime wiring profile; does not enable policy task control",
    )
    parser.add_argument(
        "--host", help="Windows interface address; defaults to the current WSL route gateway",
    )
    parser.add_argument("--control-port", type=int)
    parser.add_argument("--base-image-port", type=int)
    parser.add_argument("--wrist-image-port", type=int)
    parser.add_argument(
        "--image-stream-hz", type=float,
        help="maximum raw camera TCP send rate; ROS publication and policy rates are configured separately",
    )
    parser.add_argument("--camera-capture-fps", type=float)
    parser.add_argument("--state-snapshot-hz", type=float, default=10.0)
    parser.add_argument("--base-name")
    parser.add_argument("--wrist-name")
    parser.add_argument(
        "--base-source", choices=("directshow", "oak-d-pro"),
        help="select ACER/UVC DirectShow or an explicitly identified OAK-D Pro RGB source",
    )
    parser.add_argument(
        "--oak-device-id",
        help="required serial/device ID when --base-source oak-d-pro is selected",
    )
    parser.add_argument("--log-path", type=Path)
    parser.add_argument(
        "--raw-can-log", action="store_true",
        help="include every raw CAN RX frame in the JSONL audit log; high-volume, opt-in",
    )
    parser.add_argument("--confirm-can-owner", choices=("windows",))
    parser.add_argument("--confirm-no-motion", action="store_true")
    parser.add_argument("--confirm-camera-roles", action="store_true")
    parser.add_argument(
        "--camera-only", action="store_true",
        help="open named cameras and image streams without creating a CAN session",
    )
    parser.add_argument(
        "--disable-cameras", action="store_true",
        help="run read-only CAN feedback only; do not enumerate or open cameras",
    )
    args = parser.parse_args(argv)
    from laptop.ros_gateway.runtime_profiles import (
        load_runtime_profile,
        resolve_wsl_gateway_host,
    )

    deployment_profile, deployment_profile_hash = load_runtime_profile(args.deployment_profile)
    if args.host is None:
        args.host = resolve_wsl_gateway_host()
    gateway_profile = deployment_profile["gateway"]
    camera_profile = deployment_profile.get("cameras", {})
    rate_profile = deployment_profile.get("rates", {})
    args.control_port = (gateway_profile["control_port"] if args.control_port is None
                         else args.control_port)
    args.base_image_port = (gateway_profile["base_image_port"] if args.base_image_port is None
                            else args.base_image_port)
    args.wrist_image_port = (gateway_profile["wrist_image_port"] if args.wrist_image_port is None
                             else args.wrist_image_port)
    args.image_stream_hz = (gateway_profile["image_stream_hz"]
                            if args.image_stream_hz is None else args.image_stream_hz)
    args.camera_capture_fps = (rate_profile.get("camera_capture_hz", 30.0)
                               if args.camera_capture_fps is None else args.camera_capture_fps)
    args.base_name = (camera_profile.get("base", {}).get("device_name", "ACER HD User Facing")
                      if args.base_name is None else args.base_name)
    args.wrist_name = (camera_profile.get("wrist", {}).get("device_name", "Dabai DC1")
                       if args.wrist_name is None else args.wrist_name)
    args.base_source = (camera_profile.get("base", {}).get("source", "directshow")
                        if args.base_source is None else args.base_source)
    if args.camera_only and args.disable_cameras:
        parser.error("--camera-only cannot be combined with --disable-cameras")
    if args.disable_cameras and (args.base_source != "directshow" or args.oak_device_id):
        parser.error("Base camera source options cannot be used with --disable-cameras")
    can_enabled = not args.camera_only
    cameras_enabled = not args.disable_cameras
    gateway = None
    owner_registration = None
    try:
        validate_hardware_acknowledgements(
            can_owner=args.confirm_can_owner,
            no_motion=args.confirm_no_motion,
            camera_roles_confirmed=args.confirm_camera_roles,
            cameras_enabled=cameras_enabled,
            can_enabled=can_enabled,
        )
        log_path = args.log_path or Path("cache/ros_gateway") / (
            "piper-readonly-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".jsonl"
        )
        owner_registration = RuntimeOwnerRegistration(
            "windows_gateway",
            runtime_owner_directory(Path(__file__).resolve().parents[2]),
            details={
                "deployment_profile": args.deployment_profile,
                "deployment_profile_sha256": deployment_profile_hash,
                "ros_domain_id": deployment_profile.get("ros_domain_id"),
                "host": args.host,
                "control_port": args.control_port,
                "base_image_port": None if args.disable_cameras else args.base_image_port,
                "wrist_image_port": None if args.disable_cameras else args.wrist_image_port,
                "can_access_enabled": can_enabled,
                "camera_access_enabled": cameras_enabled,
                "base_name": args.base_name if cameras_enabled else None,
                "wrist_name": args.wrist_name if cameras_enabled else None,
                "base_source": args.base_source if cameras_enabled else None,
                "image_stream_hz": args.image_stream_hz if cameras_enabled else None,
                "camera_capture_fps": args.camera_capture_fps if cameras_enabled else None,
                "log_path": str(log_path),
            },
        )
        gateway = WindowsReadOnlyGateway(
            host=args.host,
            control_port=args.control_port,
            base_image_port=None if args.disable_cameras else args.base_image_port,
            wrist_image_port=None if args.disable_cameras else args.wrist_image_port,
            log_path=log_path,
            base_name=args.base_name,
            wrist_name=args.wrist_name,
            base_source=args.base_source,
            oak_device_id=args.oak_device_id,
            image_stream_hz=args.image_stream_hz,
            camera_capture_fps=args.camera_capture_fps,
            state_snapshot_hz=args.state_snapshot_hz,
            can_enabled=can_enabled,
            raw_can_log=args.raw_can_log,
            owner_registration=owner_registration,
        ).start()
        print(json.dumps({
            "status": "WINDOWS_READ_ONLY_GATEWAY_STARTED",
            "process_id": os.getpid(),
            "deployment_profile": args.deployment_profile,
            "deployment_profile_sha256": deployment_profile_hash,
            "profile": "windows_read_only" if can_enabled else "windows_camera_only",
            "host": args.host,
            "control_port": args.control_port,
            "image_ports": gateway.image_ports,
            "image_stream_hz": gateway.image_stream_hz,
            "camera_capture_fps": gateway.camera_capture_fps,
            "state_snapshot_hz": gateway.state_snapshot_hz,
            "camera_enabled": gateway.cameras_enabled,
            "can_access_enabled": gateway.can_enabled,
            "raw_can_log": gateway.raw_can_log,
            "camera_identity": gateway.camera_identity,
            "hardware_commands_enabled": False,
            "can_tx_calls": 0,
            "log_path": str(log_path),
            "runtime_owner_path": str(owner_registration.path),
        }, indent=2), flush=True)
        try:
            while not gateway.stop_event.wait(1.0):
                pass
        except KeyboardInterrupt:
            pass
        finally:
            gateway.close()
            owner_registration.close()
        return 0
    except Exception as exc:
        if gateway is not None:
            try:
                gateway.close()
            except Exception:
                pass
        if owner_registration is not None:
            owner_registration.close()
        print(json.dumps({
            "status": "WINDOWS_READ_ONLY_GATEWAY_FAILED_CLOSED",
            "error_type": type(exc).__name__,
            "reason": str(exc),
            "can_tx_calls": 0,
            "robot_motion": False,
        }, indent=2), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
