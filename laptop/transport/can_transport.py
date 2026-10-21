"""Shared PiPER CAN contract and its deterministic offline test double."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time
from typing import Deque, List, Optional, Protocol, runtime_checkable

from laptop.control.contracts import CommandProposal
from laptop.transport.diagnostic_queries import DiagnosticCanQuery
from laptop.transport.position_commands import (
    J1CommissioningAuthorization,
    J1CommissioningBatch,
    J1CommissioningSnapshot,
    J1CommissioningWriteReceipt,
    PositionBatchWriteReceipt,
    PositionCommandBatch,
    PositionFrameWrite,
    SupervisedPositionAuthorization,
    validate_position_execution_context,
    validate_j1_commissioning_execution,
)


class TransportOwnershipError(RuntimeError):
    pass


@dataclass(frozen=True)
class CanFrame:
    """One received CAN frame with its laptop monotonic arrival time."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool
    is_error_frame: bool
    received_monotonic_ns: int
    adapter_echo_id: Optional[int] = None

    @property
    def is_tx_echo(self) -> bool:
        return self.adapter_echo_id is not None and self.adapter_echo_id != 0xFFFFFFFF

    def __post_init__(self) -> None:
        if self.adapter_echo_id is not None and (
                type(self.adapter_echo_id) is not int
                or not 0 <= self.adapter_echo_id <= 0xFFFFFFFF):
            raise ValueError("adapter_echo_id must be an unsigned 32-bit integer or None")
        if (type(self.arbitration_id) is not int
                or not 0 <= self.arbitration_id <= 0x1FFFFFFF):
            raise ValueError("arbitration_id must be a valid CAN identifier")
        if not isinstance(self.data, bytes) or len(self.data) > 8:
            raise ValueError("CAN data must be at most 8 bytes")
        if type(self.is_extended_id) is not bool or type(self.is_error_frame) is not bool:
            raise ValueError("CAN frame flags must be booleans")
        if type(self.received_monotonic_ns) is not int or self.received_monotonic_ns <= 0:
            raise ValueError("received_monotonic_ns must be a positive integer")


@runtime_checkable
class PiperCanTransport(Protocol):
    """Exclusive-owner PiPER CAN interface used by SafetyBridge.

    Implementations may receive frames, but policy and command transmission
    remain disabled until the real-hardware gates pass.
    """

    owner: Optional[str]
    send_attempts: int

    @property
    def connected(self) -> bool: ...

    @property
    def disconnect_reason(self) -> Optional[str]: ...

    def claim(self, owner: str) -> None: ...

    def release(self, owner: str) -> None: ...

    def start_read_only(self, owner: str) -> None: ...

    def receive_frame(self, owner: str, timeout_ms: int = 100) -> Optional[CanFrame]: ...

    def stop_read_only(self, owner: str) -> None: ...

    def send(self, owner: str, proposal: CommandProposal) -> None: ...

    def send_supervised_position(
        self, owner: str, batch: PositionCommandBatch,
        authorization: SupervisedPositionAuthorization,
        current_state: object,
        limits: object,
    ) -> PositionBatchWriteReceipt: ...

    def send_supervised_j1_commissioning_step(
        self, owner: str, snapshot: J1CommissioningSnapshot,
        batch: J1CommissioningBatch, authorization: J1CommissioningAuthorization,
    ) -> J1CommissioningWriteReceipt: ...

    def send_diagnostic_query(self, owner: str, query: DiagnosticCanQuery) -> bool: ...


@dataclass
class FakeCanTransport:
    """Records attempted sends without ever touching a CAN adapter."""

    allow_recorded_send: bool = False
    allow_recorded_diagnostic_queries: bool = False
    allow_recorded_position_send: bool = False
    owner: Optional[str] = None
    send_attempts: int = 0
    recorded: List[CommandProposal] = field(default_factory=list)
    diagnostic_query_attempts: int = 0
    recorded_diagnostic_queries: List[DiagnosticCanQuery] = field(default_factory=list)
    position_send_attempts: int = 0
    recorded_position_batches: List[PositionBatchWriteReceipt] = field(default_factory=list)
    _used_position_approval_ids: set[str] = field(default_factory=set)
    connected: bool = True
    disconnect_reason: Optional[str] = None
    frames: Deque[CanFrame] = field(default_factory=deque)
    read_only_active: bool = False

    def claim(self, owner: str) -> None:
        if not owner.strip():
            raise ValueError("owner is required")
        if self.owner is not None and self.owner != owner:
            raise TransportOwnershipError(
                f"transport already owned by {self.owner!r}"
            )
        self.owner = owner

    def release(self, owner: str) -> None:
        if self.owner != owner:
            raise TransportOwnershipError("only the current owner can release transport")
        if self.read_only_active:
            raise RuntimeError("stop read-only receive before releasing the transport")
        self.owner = None

    def enqueue_frame(self, frame: CanFrame) -> None:
        if not isinstance(frame, CanFrame):
            raise TypeError("frame must be a CanFrame")
        self.frames.append(frame)

    def start_read_only(self, owner: str) -> None:
        self._require_owner(owner)
        if not self.connected:
            raise ConnectionError(self.disconnect_reason or "CAN transport disconnected")
        self.read_only_active = True

    def receive_frame(self, owner: str, timeout_ms: int = 100) -> Optional[CanFrame]:
        self._require_owner(owner)
        if type(timeout_ms) is not int or not 1 <= timeout_ms <= 1000:
            raise ValueError("timeout_ms must be an integer in [1, 1000]")
        if not self.connected:
            raise ConnectionError(self.disconnect_reason or "CAN transport disconnected")
        if not self.read_only_active:
            raise RuntimeError("read-only receive is not active")
        return self.frames.popleft() if self.frames else None

    def stop_read_only(self, owner: str) -> None:
        self._require_owner(owner)
        self.read_only_active = False

    def inject_disconnect(self, reason: str = "usb_disconnect") -> None:
        """Inject a named link loss into offline bridge tests only."""
        if reason not in {"usb_disconnect", "transport_process_exit"}:
            raise ValueError("unsupported fake transport disconnect reason")
        self.connected = False
        self.disconnect_reason = reason
        self.read_only_active = False

    def send(self, owner: str, proposal: CommandProposal) -> None:
        self.send_attempts += 1
        if self.owner != owner:
            raise TransportOwnershipError("sender does not own transport")
        if not self.connected:
            raise ConnectionError(self.disconnect_reason or "CAN transport disconnected")
        if not self.allow_recorded_send:
            raise PermissionError("fake transport is configured for zero-send testing")
        self.recorded.append(proposal)

    def send_supervised_position(
        self, owner: str, batch: PositionCommandBatch,
        authorization: SupervisedPositionAuthorization,
        current_state: object,
        limits: object,
    ) -> PositionBatchWriteReceipt:
        """In-memory rehearsal only; never writes CAN or touches USB hardware."""
        if owner != "safety-bridge":
            raise PermissionError("supervised position writes are owned by SafetyBridge")
        self.position_send_attempts += 1
        self._require_owner(owner)
        if type(batch) is not PositionCommandBatch:
            raise TypeError("batch must be a validated PositionCommandBatch")
        if type(authorization) is not SupervisedPositionAuthorization:
            raise TypeError("authorization must be a SupervisedPositionAuthorization")
        if not self.connected:
            raise ConnectionError(self.disconnect_reason or "CAN transport disconnected")
        if authorization.approval_id in self._used_position_approval_ids:
            raise PermissionError("position approval is one-use and has already been consumed")
        now = time.perf_counter_ns()
        validate_position_execution_context(
            batch.proposal, current_state, limits, now_monotonic_ns=now,
        )
        authorization.validate_for(batch.proposal, now_monotonic_ns=now)
        if not self.allow_recorded_position_send:
            raise PermissionError("fake transport is configured to reject position TX")
        self._used_position_approval_ids.add(authorization.approval_id)
        receipt = PositionBatchWriteReceipt(
            approval_id=authorization.approval_id,
            proposal_id=batch.proposal.proposal_id,
            source_request_id=batch.proposal.source_request_id,
            source_state_sequence=batch.proposal.source_state_sequence,
            target=batch.proposal.target,
            writes=tuple(
                PositionFrameWrite(frame.arbitration_id, frame.data, now + index)
                for index, frame in enumerate(batch.frames)
            ),
        )
        self.recorded_position_batches.append(receipt)
        return receipt

    def send_supervised_j1_commissioning_step(
        self, owner: str, snapshot: J1CommissioningSnapshot,
        batch: J1CommissioningBatch, authorization: J1CommissioningAuthorization,
    ) -> J1CommissioningWriteReceipt:
        del snapshot, batch, authorization
        self._require_owner(owner)
        raise PermissionError("fake transport cannot exercise a live commissioning sender")

    def send_diagnostic_query(self, owner: str, query: DiagnosticCanQuery) -> bool:
        self.diagnostic_query_attempts += 1
        if self.owner != owner:
            raise TransportOwnershipError("sender does not own transport")
        if not isinstance(query, DiagnosticCanQuery):
            raise TypeError("only a validated DiagnosticCanQuery may be sent")
        if not self.connected:
            raise ConnectionError(self.disconnect_reason or "CAN transport disconnected")
        if not self.allow_recorded_diagnostic_queries:
            raise PermissionError("fake transport is configured to reject diagnostic queries")
        self.recorded_diagnostic_queries.append(query)
        return True

    def _require_owner(self, owner: str) -> None:
        if self.owner != owner:
            raise TransportOwnershipError("caller does not own transport")
