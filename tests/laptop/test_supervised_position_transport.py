from __future__ import annotations

import math
import struct
import time
from dataclasses import replace
from types import ModuleType, SimpleNamespace
import sys

import pytest

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    RobotStateSnapshot,
)
from laptop.transport.gs_usb_piper_can import (
    CanPositionDeliveryUnknown,
    CanReceiveMode,
    GsUsbPiperCanTransport,
)
from laptop.transport.can_transport import FakeCanTransport
from laptop.transport.position_commands import (
    P12_SITE_GATE_NAMES,
    POSITION_FRAME_IDS,
    CanTxFrame,
    PositionCommandBatch,
    SupervisedPositionAuthorization,
)
from laptop.transport import gs_usb_piper_can
from laptop.safety.bridge import SafetyLimits


class MemoryPositionAdapter:
    def __init__(self, *, fail_at: int | None = None, result: bool = True) -> None:
        self.opened = False
        self.closed = False
        self.sent: list[CanTxFrame] = []
        self.fail_at = fail_at
        self.result = result

    def open_read_only(self) -> None:
        self.opened = True

    def receive_frame(self, timeout_ms: int):
        return None

    def send_diagnostic_query(self, query) -> bool:
        return True

    def send_position_frame(self, frame: CanTxFrame) -> bool:
        self.sent.append(frame)
        if self.fail_at == len(self.sent):
            if self.result is False:
                return False
            raise OSError("simulated USB write timeout")
        return True

    def close(self) -> None:
        self.closed = True


def _proposal(*, target: tuple[float, ...] | None = None) -> CommandProposal:
    now = time.perf_counter_ns()
    selected = target or (0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.05)
    return CommandProposal(
        proposal_id="proposal-1",
        source_request_id="request-1",
        source_state_sequence=17,
        created_monotonic_ns=now - 1_000_000,
        valid_until_monotonic_ns=now + 200_000_000,
        target=selected,
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=True,
        mapping_verified=True,
    )


def _batch(proposal: CommandProposal | None = None) -> PositionCommandBatch:
    proposal = proposal or _proposal()
    joint_raw = tuple(round(math.degrees(value) * 1000.0)
                      for value in proposal.target[:6])
    frames = tuple(
        CanTxFrame(0x155 + index, struct.pack(">ii", *joint_raw[2 * index:2 * index + 2]))
        for index in range(3)
    ) + (
        CanTxFrame(0x159, struct.pack(">iHBB", round(proposal.target[6] * 1_000_000), 120, 1, 0)),
    )
    return PositionCommandBatch(proposal, frames)


def _authorization(
    proposal: CommandProposal,
    *, approval_id: str = "approval-1",
    all_gates: bool = True,
    approved_at: int | None = None,
    valid_until: int | None = None,
) -> SupervisedPositionAuthorization:
    now = time.perf_counter_ns()
    approved_at = now if approved_at is None else approved_at
    valid_until = approved_at + 150_000_000 if valid_until is None else valid_until
    return SupervisedPositionAuthorization(
        approval_id=approval_id,
        proposal_id=proposal.proposal_id,
        source_request_id=proposal.source_request_id,
        source_state_sequence=proposal.source_state_sequence,
        target=proposal.target,
        approved_at_monotonic_ns=approved_at,
        valid_until_monotonic_ns=valid_until,
        operator_confirmed=True,
        site_gate_attestations=tuple((name, all_gates) for name in P12_SITE_GATE_NAMES),
    )


def _context(proposal: CommandProposal) -> tuple[RobotStateSnapshot, SafetyLimits]:
    received = proposal.created_monotonic_ns - 100_000
    state = RobotStateSnapshot(
        sequence=proposal.source_state_sequence,
        captured_monotonic_ns=received - 100_000,
        received_monotonic_ns=received,
        joint_position_rad=(0.0,) * 6,
        gripper_position_m=0.05,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    limits = SafetyLimits(
        joint_min_rad=(-3.2,) * 6,
        joint_max_rad=(3.2,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.1,
        max_gripper_delta_m=0.1,
    )
    return state, limits


def _transport(adapter: MemoryPositionAdapter,
               mode: CanReceiveMode = CanReceiveMode.NORMAL_ACKING) -> GsUsbPiperCanTransport:
    transport = GsUsbPiperCanTransport(
        adapter_factory=lambda: adapter,
        receive_mode=mode,
    )
    transport.claim("safety-bridge")
    transport.start_read_only("safety-bridge")
    return transport


def test_one_use_authorization_writes_exact_frames_and_returns_monotonic_receipt():
    adapter = MemoryPositionAdapter()
    transport = _transport(adapter)
    batch = _batch()
    authorization = _authorization(batch.proposal)
    state, limits = _context(batch.proposal)

    receipt = transport.send_supervised_position(
        "safety-bridge", batch, authorization, state, limits,
    )

    assert tuple(frame.arbitration_id for frame in adapter.sent) == POSITION_FRAME_IDS
    assert tuple(write.arbitration_id for write in receipt.writes) == POSITION_FRAME_IDS
    assert tuple(write.data for write in receipt.writes) == tuple(frame.data for frame in batch.frames)
    assert [write.adapter_accepted_monotonic_ns for write in receipt.writes] == sorted(
        write.adapter_accepted_monotonic_ns for write in receipt.writes
    )
    assert receipt.target == batch.proposal.target
    assert transport.position_frames_accepted == 4
    assert transport.send_attempts == 0  # generic send() remains a separate rejected path

    with pytest.raises(PermissionError, match="one-use"):
        transport.send_supervised_position(
            "safety-bridge", batch, authorization, state, limits,
        )
    assert len(adapter.sent) == 4
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


def test_generic_send_still_rejects_even_when_position_path_is_available():
    adapter = MemoryPositionAdapter()
    transport = _transport(adapter)

    with pytest.raises(PermissionError, match="command TX"):
        transport.send("safety-bridge", _proposal())

    assert adapter.sent == []
    assert transport.send_attempts == 1
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


def test_fake_position_transport_records_only_in_memory_when_explicitly_enabled():
    transport = FakeCanTransport()
    transport.claim("safety-bridge")
    batch = _batch()
    authorization = _authorization(batch.proposal)
    state, limits = _context(batch.proposal)

    with pytest.raises(PermissionError, match="reject position TX"):
        transport.send_supervised_position(
            "safety-bridge", batch, authorization, state, limits,
        )
    assert transport.recorded_position_batches == []

    transport.allow_recorded_position_send = True
    receipt = transport.send_supervised_position(
        "safety-bridge", batch, authorization, state, limits,
    )
    assert transport.recorded_position_batches == [receipt]
    assert transport.position_send_attempts == 2
    assert transport.send_attempts == 0


def test_incomplete_site_gates_reject_before_any_frame_write():
    adapter = MemoryPositionAdapter()
    transport = _transport(adapter)
    batch = _batch()
    state, limits = _context(batch.proposal)

    with pytest.raises(PermissionError, match="all P12 site gates"):
        transport.send_supervised_position(
            "safety-bridge", batch,
            _authorization(batch.proposal, all_gates=False),
            state, limits,
        )

    assert adapter.sent == []
    assert transport.position_delivery_unknown is False
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


def test_transport_rechecks_feedback_sequence_and_position_step_limits():
    adapter = MemoryPositionAdapter()
    transport = _transport(adapter)
    batch = _batch()
    authorization = _authorization(batch.proposal)
    state, limits = _context(batch.proposal)

    with pytest.raises(ValueError, match="not created from this feedback sequence"):
        transport.send_supervised_position(
            "safety-bridge", batch, authorization,
            replace(state, sequence=state.sequence + 1), limits,
        )

    tight_limits = replace(limits, max_joint_delta_rad=(0.001,) * 6)
    with pytest.raises(ValueError, match="single-step limit"):
        transport.send_supervised_position(
            "safety-bridge", batch, authorization, state, tight_limits,
        )

    assert adapter.sent == []
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


def test_listen_only_and_expired_authorizations_never_write_position_frames():
    adapter = MemoryPositionAdapter()
    transport = _transport(adapter, CanReceiveMode.LISTEN_ONLY)
    batch = _batch()
    state, limits = _context(batch.proposal)
    with pytest.raises(PermissionError, match="listen-only"):
        transport.send_supervised_position(
            "safety-bridge", batch, _authorization(batch.proposal), state, limits,
        )
    assert adapter.sent == []
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")

    adapter = MemoryPositionAdapter()
    transport = _transport(adapter)
    batch = _batch()
    state, limits = _context(batch.proposal)
    now = time.perf_counter_ns()
    expired = _authorization(
        batch.proposal,
        approved_at=now - 300_000_000,
        valid_until=now - 50_000_000,
    )
    with pytest.raises(ValueError, match="stale or expired"):
        transport.send_supervised_position(
            "safety-bridge", batch, expired, state, limits,
        )
    assert adapter.sent == []
    transport.stop_read_only("safety-bridge")
    transport.release("safety-bridge")


@pytest.mark.parametrize("failure_result", [True, False])
def test_partial_or_unacknowledged_usb_write_latches_unknown_and_never_retries(failure_result):
    adapter = MemoryPositionAdapter(fail_at=2, result=failure_result)
    transport = _transport(adapter)
    batch = _batch()
    authorization = _authorization(batch.proposal)
    state, limits = _context(batch.proposal)

    with pytest.raises(CanPositionDeliveryUnknown) as caught:
        transport.send_supervised_position(
            "safety-bridge", batch, authorization, state, limits,
        )

    assert caught.value.confirmed_frame_count == 1
    assert caught.value.failed_frame_id == 0x156
    assert caught.value.confirmed_writes[0].data == batch.frames[0].data
    assert caught.value.failed_frame_data == batch.frames[1].data
    assert caught.value.failed_at_monotonic_ns > 0
    assert transport.position_delivery_unknown is True
    assert transport.disconnect_reason == "position_command_delivery_unknown"
    assert transport.connected is False
    assert adapter.closed is True
    assert len(adapter.sent) == 2
    with pytest.raises(ConnectionError, match="delivery is unknown"):
        transport.start_read_only("safety-bridge")
    with pytest.raises(ConnectionError, match="do not retry"):
        transport.send_supervised_position(
            "safety-bridge", batch, authorization, state, limits,
        )
    assert len(adapter.sent) == 2


def test_position_batch_rejects_bytes_that_do_not_match_the_proposal():
    proposal = _proposal()
    batch = _batch(proposal)
    wrong = list(batch.frames)
    wrong[0] = CanTxFrame(0x155, struct.pack(">ii", 999, 999))
    with pytest.raises(ValueError, match="do not encode"):
        PositionCommandBatch(proposal, tuple(wrong))


def test_position_authorization_rejects_boolean_target_values():
    proposal = _proposal()
    with pytest.raises(ValueError, match="seven finite values"):
        SupervisedPositionAuthorization(
            approval_id="bool-target",
            proposal_id=proposal.proposal_id,
            source_request_id=proposal.source_request_id,
            source_state_sequence=proposal.source_state_sequence,
            target=(True, 0, 0, 0, 0, 0, 0.05),
            approved_at_monotonic_ns=time.perf_counter_ns(),
            valid_until_monotonic_ns=time.perf_counter_ns() + 100_000_000,
            operator_confirmed=True,
            site_gate_attestations=tuple((name, True) for name in P12_SITE_GATE_NAMES),
        )


def test_sdk_adapter_constructs_exact_standard_position_frames(monkeypatch):
    class RawFrame:
        def __init__(self, *, can_id=0, data=b""):
            self.arbitration_id = can_id
            self.data = bytearray(data)
            self.can_dlc = len(data)
            self.is_extended_id = False
            self.is_error_frame = False

    class Device:
        def __init__(self):
            self.gs_usb = SimpleNamespace(idVendor=0x1D50, idProduct=0x606F)
            self.device_flags = 0
            self.sent = []

        def set_bitrate(self, bitrate):
            return bitrate == 1_000_000

        def start(self, *, flags):
            assert flags == 0

        def send(self, frame):
            self.sent.append(frame)
            return True

        def stop(self):
            pass

    device = Device()

    class GsUsb:
        @staticmethod
        def scan():
            return [device]

    package = ModuleType("gs_usb")
    package.__path__ = []
    gs_usb_module = ModuleType("gs_usb.gs_usb")
    gs_usb_module.GsUsb = GsUsb
    gs_usb_module.GsUsbFrame = RawFrame
    monkeypatch.setitem(sys.modules, "gs_usb", package)
    monkeypatch.setitem(sys.modules, "gs_usb.gs_usb", gs_usb_module)
    monkeypatch.setattr(gs_usb_piper_can, "ensure_libusb_backend", lambda: None)

    adapter = gs_usb_piper_can._GsUsbReceiveAdapter()
    adapter.open_read_only()
    frame = CanTxFrame(0x155, bytes.fromhex("0000000100000002"))
    assert adapter.send_position_frame(frame) is True
    adapter.close()

    assert len(device.sent) == 1
    assert device.sent[0].arbitration_id == 0x155
    assert bytes(device.sent[0].data) == frame.data
