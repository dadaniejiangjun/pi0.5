from __future__ import annotations

import math
import struct
import time

import pytest

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    RobotStateSnapshot,
)
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, SafetyLimits
from laptop.transport.can_transport import FakeCanTransport
from laptop.transport.gs_usb_piper_can import (
    CanPositionDeliveryUnknown,
    GsUsbPiperCanTransport,
)
from laptop.transport.position_commands import (
    P12_SITE_GATE_NAMES,
    CanTxFrame,
    PositionCommandBatch,
    SupervisedPositionAuthorization,
)


class MemoryPositionAdapter:
    def __init__(self, *, fail_at: int | None = None) -> None:
        self.closed = False
        self.sent: list[CanTxFrame] = []
        self.fail_at = fail_at

    def open_read_only(self) -> None:
        pass

    def receive_frame(self, timeout_ms: int):
        return None

    def send_diagnostic_query(self, query) -> bool:
        return True

    def send_position_frame(self, frame: CanTxFrame) -> bool:
        self.sent.append(frame)
        if self.fail_at == len(self.sent):
            raise OSError("simulated USB timeout")
        return True

    def close(self) -> None:
        self.closed = True


def _limits() -> SafetyLimits:
    return SafetyLimits(
        joint_min_rad=(-3.2,) * 6,
        joint_max_rad=(3.2,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.1,
        max_gripper_delta_m=0.1,
    )


def _bundle():
    base = time.perf_counter_ns()
    snapshot = RobotStateSnapshot(
        sequence=23,
        captured_monotonic_ns=base - 2_000_000,
        received_monotonic_ns=base - 1_000_000,
        joint_position_rad=(0.0,) * 6,
        gripper_position_m=0.05,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    target = (0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.05)
    proposal = CommandProposal(
        proposal_id="bridge-proposal-1",
        source_request_id="bridge-request-1",
        source_state_sequence=snapshot.sequence,
        created_monotonic_ns=base - 500_000,
        valid_until_monotonic_ns=base + 150_000_000,
        target=target,
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=True,
        mapping_verified=True,
    )
    raw = tuple(round(math.degrees(value) * 1000) for value in target[:6])
    frames = tuple(
        CanTxFrame(0x155 + index, struct.pack(">ii", *raw[2 * index:2 * index + 2]))
        for index in range(3)
    ) + (
        CanTxFrame(0x159, struct.pack(">iHBB", 50_000, 120, 1, 0)),
    )
    batch = PositionCommandBatch(proposal, frames)
    approved_at = time.perf_counter_ns()
    authorization = SupervisedPositionAuthorization(
        approval_id="bridge-approval-1",
        proposal_id=proposal.proposal_id,
        source_request_id=proposal.source_request_id,
        source_state_sequence=snapshot.sequence,
        target=proposal.target,
        approved_at_monotonic_ns=approved_at,
        valid_until_monotonic_ns=min(approved_at + 100_000_000,
                                     proposal.valid_until_monotonic_ns),
        operator_confirmed=True,
        site_gate_attestations=tuple((name, True) for name in P12_SITE_GATE_NAMES),
    )
    return snapshot, batch, authorization


def _bridge(adapter: MemoryPositionAdapter) -> SafetyBridge:
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    bridge = SafetyBridge(
        mode=RuntimeMode.SUPERVISED_POLICY_STEP,
        transport=transport,
        limits=_limits(),
        supervised_position_tx_enabled=True,
    )
    bridge.start_read_only()
    bridge.record_heartbeat(time.perf_counter_ns())
    return bridge


def test_safety_bridge_sends_one_authorized_position_batch_via_dedicated_mode():
    adapter = MemoryPositionAdapter()
    bridge = _bridge(adapter)
    snapshot, batch, authorization = _bundle()

    receipt = bridge.send_supervised_position_batch(
        snapshot=snapshot, batch=batch, authorization=authorization,
    )

    assert tuple(frame.arbitration_id for frame in adapter.sent) == (0x155, 0x156, 0x157, 0x159)
    assert receipt.approval_id == authorization.approval_id
    assert len(receipt.writes) == 4
    assert bridge.state is BridgeState.READY
    bridge.close()


def test_partial_write_latches_bridge_and_transport_without_retry():
    adapter = MemoryPositionAdapter(fail_at=2)
    bridge = _bridge(adapter)
    snapshot, batch, authorization = _bundle()

    with pytest.raises(CanPositionDeliveryUnknown) as caught:
        bridge.send_supervised_position_batch(
            snapshot=snapshot, batch=batch, authorization=authorization,
        )

    assert caught.value.confirmed_frame_count == 1
    assert bridge.state is BridgeState.FAULT_LATCHED
    assert adapter.closed is True
    assert len(adapter.sent) == 2
    bridge.close()


def test_supervised_bridge_mode_requires_explicit_gate_and_real_transport_type():
    with pytest.raises(PermissionError, match="explicit runtime gate"):
        SafetyBridge(
            mode=RuntimeMode.SUPERVISED_POLICY_STEP,
            transport=GsUsbPiperCanTransport(adapter_factory=MemoryPositionAdapter),
            limits=_limits(),
        )
    with pytest.raises(PermissionError, match="exact GsUsbPiperCanTransport"):
        SafetyBridge(
            mode=RuntimeMode.SUPERVISED_POLICY_STEP,
            transport=FakeCanTransport(),
            limits=_limits(),
            supervised_position_tx_enabled=True,
        )
