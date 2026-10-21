"""Fake-profile-only ROS proposal transport and command-receipt publisher.

This node never opens CAN. It only forwards typed proposals when the peer
handshake identifies the offline fake gateway and that gateway rejects them.
"""
from __future__ import annotations

import os
import queue
import socket
import sys
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pi05_piper_interfaces.msg import CommandProposal, CommandReceipt

from .proposal_receipt_core import ProposalRejected, proposal_payload


class ProposalReceiptBridge(Node):
    def __init__(
        self,
        *,
        host_override: str | None = None,
        control_port_override: int | None = None,
        allow_fake_profile_override: bool | None = None,
    ) -> None:
        super().__init__("pi05_fake_proposal_receipt_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("control_port", 8765)
        self.declare_parameter("connect_timeout_s", 1.0)
        self.declare_parameter("allow_fake_profile", False)
        self.host = str(
            self.get_parameter("host").value if host_override is None else host_override
        )
        self.control_port = int(
            self.get_parameter("control_port").value
            if control_port_override is None else control_port_override
        )
        self.connect_timeout_s = float(self.get_parameter("connect_timeout_s").value)
        self.allow_fake_profile = bool(
            self.get_parameter("allow_fake_profile").value
            if allow_fake_profile_override is None else allow_fake_profile_override
        )
        if self.host in ("", "0.0.0.0", "::"):
            raise ValueError("proposal bridge host must be an explicit destination")
        if not 1 <= self.control_port <= 65535:
            raise ValueError("control_port must be within 1..65535")
        if not 0.1 <= self.connect_timeout_s <= 10:
            raise ValueError("connect_timeout_s must be within [0.1, 10]")

        self.receipt_pub = self.create_publisher(
            CommandReceipt, "/piper/command_receipt", 10
        )
        self.proposal_sub = self.create_subscription(
            CommandProposal, "/piper/command_proposal", self._on_proposal, 10
        )
        self.stop_event = threading.Event()
        self.proposal_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        self._state_lock = threading.Lock()
        self._receipt_lock = threading.Lock()
        self._connected = False
        self._profile = ""
        self._hardware_commands_enabled = True
        self._session_id = ""
        self._receipt_sequence = 0
        self._seen_request_ids: set[str] = set()
        self._threads: list[threading.Thread] = []

    def start_workers(self) -> None:
        if self._threads:
            raise RuntimeError("proposal receipt bridge already started")
        worker = threading.Thread(
            target=self._control_worker,
            name="fake-proposal-receipt-worker",
            daemon=True,
        )
        self._threads.append(worker)
        worker.start()

    def stop_workers(self) -> None:
        self.stop_event.set()
        for worker in self._threads:
            worker.join(timeout=1.0)
        self._threads.clear()

    def _publish_receipt(
        self,
        *,
        request_id: str,
        observation_id: str = "",
        decision: str,
        reason: str,
        can_tx_calls: int = 0,
    ) -> None:
        with self._receipt_lock:
            self._receipt_sequence += 1
            sequence = self._receipt_sequence
        receipt = CommandReceipt()
        receipt.header.stamp = self.get_clock().now().to_msg()
        receipt.request_id = request_id[:128]
        receipt.observation_id = observation_id[:256]
        receipt.decision = decision[:64]
        receipt.reason = reason[:256]
        receipt.sequence = sequence
        receipt.source_monotonic_ns = time.perf_counter_ns()
        receipt.adapter_accepted = False
        receipt.feedback_matched = False
        receipt.can_tx_calls = max(0, int(can_tx_calls))
        self.receipt_pub.publish(receipt)

    def _on_proposal(self, message: CommandProposal) -> None:
        request_id = str(message.request_id)[:128]
        observation_id = str(message.source_observation_id)[:256]
        if not self.allow_fake_profile:
            self._publish_receipt(
                request_id=request_id,
                observation_id=observation_id,
                decision="REJECTED_FAKE_PROFILE_REQUIRED",
                reason="this bridge forwards proposals only when fake profile is explicitly enabled",
            )
            return
        try:
            payload = proposal_payload(message, now_monotonic_ns=time.perf_counter_ns())
        except ProposalRejected as exc:
            self._publish_receipt(
                request_id=request_id,
                observation_id=observation_id,
                decision="REJECTED_INVALID_PROPOSAL",
                reason=str(exc),
            )
            return
        with self._state_lock:
            if not self._connected or self._profile != "fake" or self._hardware_commands_enabled:
                self._publish_receipt(
                    request_id=request_id,
                    observation_id=observation_id,
                    decision="REJECTED_FAKE_GATEWAY_NOT_READY",
                    reason="proposal relay requires a connected fake profile with hardware commands disabled",
                )
                return
        if payload["request_id"] in self._seen_request_ids:
            self._publish_receipt(
                request_id=payload["request_id"],
                observation_id=observation_id,
                decision="REJECTED_DUPLICATE_REQUEST",
                reason="request IDs are single-use within this bridge session",
            )
            return
        self._seen_request_ids.add(payload["request_id"])
        if len(self._seen_request_ids) > 1024:
            self._seen_request_ids.clear()
            self._seen_request_ids.add(payload["request_id"])
        try:
            self.proposal_queue.put_nowait(payload)
        except queue.Full:
            self._publish_receipt(
                request_id=payload["request_id"],
                observation_id=observation_id,
                decision="REJECTED_QUEUE_FULL",
                reason="the one-entry proposal queue is occupied; no request was sent",
            )

    def _set_peer(self, *, connected: bool, profile: str = "", commands_enabled: bool = True,
                  session_id: str = "") -> None:
        with self._state_lock:
            self._connected = connected
            self._profile = profile
            self._hardware_commands_enabled = commands_enabled
            self._session_id = session_id

    def _drain_unsent(self, reason: str) -> None:
        while True:
            try:
                payload = self.proposal_queue.get_nowait()
            except queue.Empty:
                return
            self._publish_receipt(
                request_id=payload["request_id"],
                observation_id=payload["observation_id"],
                decision="REJECTED_DISCONNECTED_NO_RETRY",
                reason=reason,
            )

    def _control_worker(self) -> None:
        project_root = os.environ.get(
            "PI05_PROJECT_ROOT", "/mnt/e/01_NUS/02_5003/12_pi0.5"
        )
        if project_root not in sys.path:
            sys.path.insert(0, project_root)
        from laptop.ros_gateway.wire_protocol import recv_control_message, send_control_message

        while not self.stop_event.is_set():
            pending: dict[str, dict[str, Any]] = {}
            try:
                with socket.create_connection(
                    (self.host, self.control_port), timeout=self.connect_timeout_s
                ) as sock:
                    sock.settimeout(0.1)
                    send_control_message(sock, "hello", {
                        "client_name": "pi05_ros_fake_proposal_receipt_bridge",
                        "client_role": "fake_only_command_receipt_probe",
                        "hardware_commands_enabled": False,
                    })
                    sock.settimeout(self.connect_timeout_s)
                    kind, hello = recv_control_message(sock)
                    if kind != "hello_ack":
                        raise RuntimeError("gateway did not complete hello handshake")
                    profile = str(hello.get("profile", ""))
                    commands_enabled = hello.get("hardware_commands_enabled", True)
                    session_id = str(hello.get("session_id", ""))
                    if not session_id:
                        raise RuntimeError("gateway handshake omitted session_id")
                    self._set_peer(
                        connected=True, profile=profile,
                        commands_enabled=(commands_enabled is not False),
                        session_id=session_id,
                    )
                    if profile != "fake" or commands_enabled is not False:
                        raise RuntimeError("only the offline fake gateway is accepted")
                    sock.settimeout(0.05)
                    heartbeat_seq = 0
                    last_heartbeat_ns = 0
                    while not self.stop_event.is_set():
                        now = time.perf_counter_ns()
                        if now - last_heartbeat_ns >= 500_000_000:
                            heartbeat_seq += 1
                            last_heartbeat_ns = now
                            send_control_message(sock, "heartbeat", {
                                "session_id": session_id,
                                "sequence": heartbeat_seq,
                                "client_monotonic_ns": now,
                            })
                        try:
                            proposal = self.proposal_queue.get_nowait()
                        except queue.Empty:
                            proposal = None
                        if proposal is not None:
                            if time.perf_counter_ns() >= proposal["valid_until_monotonic_ns"]:
                                self._publish_receipt(
                                    request_id=proposal["request_id"],
                                    observation_id=proposal["observation_id"],
                                    decision="REJECTED_EXPIRED_BEFORE_SEND",
                                    reason="proposal expired while waiting in the queue",
                                )
                            else:
                                wire_payload = dict(proposal)
                                wire_payload.pop("observation_id", None)
                                wire_payload["session_id"] = session_id
                                try:
                                    send_control_message(sock, "command_proposal", wire_payload)
                                except OSError:
                                    self._publish_receipt(
                                        request_id=proposal["request_id"],
                                        observation_id=proposal["observation_id"],
                                        decision="DELIVERY_UNKNOWN_NO_RETRY",
                                        reason="proposal transport failed after send began; it will not be retried",
                                    )
                                    raise
                                pending[proposal["request_id"]] = proposal
                        try:
                            kind, payload = recv_control_message(sock)
                        except socket.timeout:
                            continue
                        except TimeoutError:
                            continue
                        if kind == "heartbeat_ack":
                            continue
                        if kind == "gateway_status" or kind == "state_snapshot":
                            continue
                        if kind == "command_receipt":
                            request_id = str(payload.get("request_id", ""))
                            proposal = pending.pop(request_id, None)
                            if proposal is None:
                                self.get_logger().warning("ignored receipt for an unknown request")
                                continue
                            decision = str(payload.get("decision", "INVALID_RECEIPT"))
                            reason = str(payload.get("reason", ""))
                            tx_calls = payload.get("can_tx_calls", 0)
                            if type(tx_calls) is not int or tx_calls < 0:
                                decision = "INVALID_RECEIPT"
                                reason = "gateway receipt has an invalid CAN TX count"
                                tx_calls = 0
                            self._publish_receipt(
                                request_id=request_id,
                                observation_id=proposal["observation_id"],
                                decision=decision,
                                reason=reason,
                                can_tx_calls=tx_calls,
                            )
                            continue
                        if kind == "fault":
                            raise RuntimeError(f"gateway fault: {payload}")
                        raise RuntimeError(f"unsupported gateway message: {kind}")
            except Exception as exc:
                for request_id, proposal in pending.items():
                    self._publish_receipt(
                        request_id=request_id,
                        observation_id=proposal["observation_id"],
                        decision="DELIVERY_UNKNOWN_NO_RETRY",
                        reason=f"gateway disconnected before receipt ({type(exc).__name__})",
                    )
                self._set_peer(connected=False)
                self._drain_unsent("fake gateway disconnected; queued requests are not retried")
                if not self.stop_event.is_set():
                    self.get_logger().warning(
                        f"fake gateway disconnected: {type(exc).__name__}: {exc}"
                    )
                    self.stop_event.wait(0.5)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = ProposalReceiptBridge()
    node.start_workers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_workers()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
