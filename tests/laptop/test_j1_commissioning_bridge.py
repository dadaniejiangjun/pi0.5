from __future__ import annotations

import struct
import time
from dataclasses import replace

import pytest

from laptop.runtime.modes import RuntimeMode
from laptop.robot_client.supervised_j1_commissioning import _feedback_outcome, execute
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import CanFrame, FakeCanTransport
from laptop.transport.gs_usb_piper_can import (
    CanJ1CommissioningDeliveryUnknown,
    CanReceiveMode,
    GsUsbPiperCanTransport,
)
from laptop.transport.position_commands import (
    COMMISSIONING_J1_FRAME_IDS,
    J1CommissioningAuthorization,
    J1CommissioningBatch,
    validate_j1_commissioning_snapshot,
)


class MemoryJ1Adapter:
    def __init__(self, *, fail_at: int | None = None) -> None:
        self.incoming: list[CanFrame] = []
        self.sent = []
        self.fail_at = fail_at
        self.closed = False
        self.short_usb_packets = 0

    def open_read_only(self) -> None:
        pass

    def receive_frame(self, timeout_ms: int):
        return self.incoming.pop(0) if self.incoming else None

    def send_diagnostic_query(self, query) -> bool:
        return True

    def send_position_frame(self, frame) -> bool:
        self.sent.append(frame)
        if self.fail_at == len(self.sent):
            raise OSError("simulated USB write timeout")
        return True

    def close(self) -> None:
        self.closed = True

    def send_mode_initialization_frame(self, frame) -> bool:
        from laptop.transport.position_commands import CanTxFrame
        return self.send_position_frame(CanTxFrame(frame.arbitration_id, frame.data))


MEASURED_NONZERO_POSE_RAW = (88151, 165, -244, 15079, 10950, -14294)


def _bridge(adapter: MemoryJ1Adapter) -> SafetyBridge:
    transport = GsUsbPiperCanTransport(
        adapter_factory=lambda: adapter,
        receive_mode=CanReceiveMode.NORMAL_ACKING,
    )
    bridge = SafetyBridge(
        mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
        supervised_j1_commissioning_tx_enabled=True,
    )
    bridge.start_read_only()
    return bridge


def _queue_stable_feedback(
    adapter: MemoryJ1Adapter,
    *,
    moving: bool = False,
    joint_pose: tuple[int, ...] = MEASURED_NONZERO_POSE_RAW,
) -> None:
    start = time.perf_counter_ns() - 380_000_000
    for index in range(61):
        stamp = start + index * 5_000_000
        joints = list(joint_pose)
        if moving and index == 60:
            joints[0] += 100
        frames = [
            (0x2A1, bytes((1, 0, 1, 0, 0, 0, 0, 0))),
            (0x2A5, struct.pack(">ii", *joints[0:2])),
            (0x2A6, struct.pack(">ii", *joints[2:4])),
            (0x2A7, struct.pack(">ii", *joints[4:6])),
        ]
        frames.extend(
            (can_id, bytes((0, 0, 0, 0, 0, 0x40, 0, 0)))
            for can_id in range(0x261, 0x267)
        )
        adapter.incoming.extend(
            CanFrame(can_id, data, False, False, stamp) for can_id, data in frames
        )


def _snapshot(bridge: SafetyBridge, adapter: MemoryJ1Adapter):
    _queue_stable_feedback(adapter)
    while adapter.incoming:
        bridge.receive_frame(timeout_ms=1)
    return bridge.create_j1_commissioning_snapshot(
        physical_estop_ready=True,
        low_speed_confirmed=True,
        motion_envelope_cleared=True,
    )


def _consume_feedback(bridge: SafetyBridge, adapter: MemoryJ1Adapter) -> None:
    while adapter.incoming:
        bridge.receive_frame(timeout_ms=1)


def _authorization(batch: J1CommissioningBatch) -> J1CommissioningAuthorization:
    now = time.perf_counter_ns()
    return J1CommissioningAuthorization(
        approval_id="test-once",
        source_state_sequence=batch.source_state_sequence,
        target_joint_raw_0p001deg=batch.target_joint_raw_0p001deg,
        approved_at_monotonic_ns=now,
        valid_until_monotonic_ns=now + 200_000_000,
        operator_confirmed=True,
        physical_estop_ready=True,
        low_speed_confirmed=True,
        motion_envelope_cleared=True,
    )


def test_bridge_snapshot_and_batch_accept_stable_measured_nonzero_pose():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot)

    assert snapshot.joint_raw_0p001deg == MEASURED_NONZERO_POSE_RAW
    assert batch.target_joint_raw_0p001deg == (88251, 165, -244, 15079, 10950, -14294)
    assert tuple(frame.arbitration_id for frame in batch.frames) == COMMISSIONING_J1_FRAME_IDS
    assert tuple(struct.unpack(">ii", frame.data) for frame in batch.frames) == (
        (88251, 165), (-244, 15079), (10950, -14294),
    )
    bridge.close()


@pytest.mark.parametrize("axis", range(2, 7))
def test_selected_axis_one_degree_holds_all_other_joints(axis):
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    step = -1000 if axis == 3 else 1000
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot, selected_axis=axis,
        step_raw=step, configure_control_mode=True)
    assert [b-a for a,b in zip(batch.current_joint_raw_0p001deg,
                             batch.target_joint_raw_0p001deg)] == [
                                 step if i == axis - 1 else 0 for i in range(6)]
    bridge.record_heartbeat(time.perf_counter_ns())
    receipt = bridge.send_supervised_j1_commissioning_step(
        snapshot=snapshot, batch=batch, authorization=_authorization(batch))
    assert len(receipt.writes) == 4
    bridge.close()


@pytest.mark.parametrize("approval", [True, False])
def test_gripper_bridge_requires_approval_and_fresh_owned_feedback(approval):
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    _snapshot(bridge, adapter)
    adapter.incoming.append(CanFrame(0x2A8, struct.pack(">ihBB", 0, 0, 64, 0),
                                    False, False, time.perf_counter_ns()))
    bridge.receive_frame(timeout_ms=1)
    site = dict(physical_estop_ready=True, low_speed_confirmed=True, motion_envelope_cleared=True)
    if approval:
        receipt = bridge.send_gripper_opening_once(operator_approved=True, **site)
        assert receipt.arbitration_id == 0x159
        assert len(adapter.sent) == 1
    else:
        with pytest.raises(PermissionError):
            bridge.send_gripper_opening_once(operator_approved=False, **site)
        assert not adapter.sent
    bridge.close()


def test_one_use_authorization_sends_exact_fixed_frames_through_memory_adapter():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot)
    bridge.record_heartbeat(time.perf_counter_ns())

    receipt = bridge.send_supervised_j1_commissioning_step(
        snapshot=snapshot,
        batch=batch,
        authorization=_authorization(batch),
    )

    assert tuple(frame.arbitration_id for frame in adapter.sent) == COMMISSIONING_J1_FRAME_IDS
    assert tuple(write.arbitration_id for write in receipt.writes) == COMMISSIONING_J1_FRAME_IDS
    assert receipt.target_joint_raw_0p001deg == (88251, 165, -244, 15079, 10950, -14294)
    assert list(write.adapter_accepted_monotonic_ns for write in receipt.writes) == sorted(
        write.adapter_accepted_monotonic_ns for write in receipt.writes
    )
    bridge.close()


def test_changed_feedback_after_snapshot_rejects_without_writing():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot)
    # A newer sample changes J1 after the operator-visible snapshot.
    newer = CanFrame(0x2A5, struct.pack(">ii", 25, 0), False, False,
                     time.perf_counter_ns())
    adapter.incoming.append(newer)
    bridge.receive_frame(timeout_ms=1)
    bridge.record_heartbeat(time.perf_counter_ns())

    with pytest.raises(PermissionError, match="does not match bridge-received"):
        bridge.send_supervised_j1_commissioning_step(
            snapshot=snapshot, batch=batch, authorization=_authorization(batch),
        )
    assert adapter.sent == []
    bridge.close()


def test_sparse_history_does_not_count_as_a_continuous_stable_window():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    first_stamp = snapshot.stable_joint_samples[0][0]
    coarse_history = (
        (first_stamp, (0,) * 6),
        (first_stamp + 150_000_000, (0,) * 6),
        (first_stamp + 300_000_000, (0,) * 6),
    )
    sparse = replace(snapshot, stable_joint_samples=coarse_history)

    with pytest.raises(PermissionError, match="300 ms window"):
        validate_j1_commissioning_snapshot(
            sparse, now_monotonic_ns=time.perf_counter_ns(),
        )
    bridge.close()


def test_partial_memory_write_latches_delivery_unknown_and_never_retries():
    adapter = MemoryJ1Adapter(fail_at=2)
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot)
    bridge.record_heartbeat(time.perf_counter_ns())

    with pytest.raises(CanJ1CommissioningDeliveryUnknown) as error:
        bridge.send_supervised_j1_commissioning_step(
            snapshot=snapshot, batch=batch, authorization=_authorization(batch),
        )
    assert error.value.confirmed_frame_count == 1
    assert adapter.closed
    assert len(adapter.sent) == 2
    assert bridge.state.value == "FAULT_LATCHED"
    with pytest.raises(PermissionError, match="not ready"):
        bridge.send_supervised_j1_commissioning_step(
            snapshot=snapshot, batch=batch, authorization=_authorization(batch),
        )
    assert len(adapter.sent) == 2
    bridge.close()


def test_feedback_drift_above_tolerance_rejects_step():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    _queue_stable_feedback(adapter, moving=True)
    _consume_feedback(bridge, adapter)
    with pytest.raises(PermissionError, match="drift at most 0.050 degree"):
        bridge.create_j1_commissioning_snapshot(
            physical_estop_ready=True,
            low_speed_confirmed=True,
            motion_envelope_cleared=True,
        )
    bridge.close()


def test_j1_step_rejects_command_target_beyond_nominal_limit():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    _queue_stable_feedback(adapter, joint_pose=(150_000, 165, -244, 15079, 10950, -14294))
    _consume_feedback(bridge, adapter)
    snapshot = bridge.create_j1_commissioning_snapshot(
        physical_estop_ready=True,
        low_speed_confirmed=True,
        motion_envelope_cleared=True,
    )
    with pytest.raises(ValueError, match="outside conservative SDK joint bounds"):
        J1CommissioningBatch.for_fixed_j1_step(snapshot)
    bridge.close()


def test_feedback_outside_conservative_nominal_joint_bounds_is_rejected():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    _queue_stable_feedback(adapter, joint_pose=(88151, -1, -244, 15079, 10950, -14294))
    _consume_feedback(bridge, adapter)
    with pytest.raises(PermissionError, match="outside conservative SDK position bounds"):
        bridge.create_j1_commissioning_snapshot(
            physical_estop_ready=True,
            low_speed_confirmed=True,
            motion_envelope_cleared=True,
        )
    bridge.close()


def _post_feedback(joints: tuple[int, ...], stamp: int) -> dict[int, tuple[int, bytes]]:
    latest = {
        0x2A1: (stamp, bytes((1, 0, 1, 0, 0, 0, 0, 0))),
        0x2A5: (stamp, struct.pack(">ii", *joints[0:2])),
        0x2A6: (stamp, struct.pack(">ii", *joints[2:4])),
        0x2A7: (stamp, struct.pack(">ii", *joints[4:6])),
    }
    latest.update({
        can_id: (stamp, bytes((0, 0, 0, 0, 0, 0x40, 0, 0)))
        for can_id in range(0x261, 0x267)
    })
    return latest


def test_post_feedback_matches_increment_from_measured_nonzero_pose():
    now = time.perf_counter_ns()
    source = MEASURED_NONZERO_POSE_RAW
    target = (source[0] + 100, *source[1:])
    outcome = _feedback_outcome(
        _post_feedback(target, now - 1_000_000),
        [(now - 1_000_000, target)], source, target, now,
    )

    assert outcome["status"] == "SINGLE_STEP_FEEDBACK_MATCH"
    assert outcome["observed_overshoot_or_other_joint_motion"] is False


def test_post_feedback_in_wrong_direction_is_not_a_match():
    now = time.perf_counter_ns()
    source = MEASURED_NONZERO_POSE_RAW
    wrong = (source[0] - 100, *source[1:])
    target = (source[0] + 100, *source[1:])
    outcome = _feedback_outcome(
        _post_feedback(wrong, now - 1_000_000),
        [(now - 1_000_000, wrong)], source, target, now,
    )

    assert outcome["status"] == "FRAMES_ACCEPTED_FEEDBACK_UNCONFIRMED_NO_RETRY"
    assert outcome["observed_overshoot_or_other_joint_motion"] is True


def test_competing_command_is_latched_in_feedback_and_rejects_step():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    _queue_stable_feedback(adapter)
    _consume_feedback(bridge, adapter)
    adapter.incoming.append(CanFrame(0x155, bytes(8), False, False,
                                     time.perf_counter_ns()))
    bridge.receive_frame(timeout_ms=1)
    with pytest.raises(PermissionError, match="another controller"):
        bridge.create_j1_commissioning_snapshot(
            physical_estop_ready=True,
            low_speed_confirmed=True,
            motion_envelope_cleared=True,
        )
    assert adapter.sent == []
    bridge.close()


def test_commissioning_gate_rejects_fake_transport_and_default_gate():
    fake = FakeCanTransport()
    with pytest.raises(PermissionError, match="candleLight transport"):
        SafetyBridge(
            mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
            transport=fake,
            limits=UNVERIFIED_ANALYSIS_LIMITS,
            supervised_j1_commissioning_tx_enabled=True,
        )
    live = GsUsbPiperCanTransport(adapter_factory=lambda: MemoryJ1Adapter())
    with pytest.raises(PermissionError, match="explicit runtime gate"):
        SafetyBridge(
            mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
            transport=live,
            limits=UNVERIFIED_ANALYSIS_LIMITS,
        )


def test_live_entry_without_explicit_fixed_step_flag_does_not_open_transport():
    adapter = MemoryJ1Adapter()
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    with pytest.raises(PermissionError, match="explicit fixed-J1"):
        execute(
            commissioning_j1_plus_0_1deg=False,
            physical_estop_ready=True,
            low_speed_confirmed=True,
            motion_envelope_cleared=True,
            confirm=lambda _preview: "",
            transport=transport,
        )
    assert not transport.connected
    assert not adapter.sent
    assert not adapter.closed


def test_one_degree_batch_sets_fixed_5percent_then_joint_target():
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    batch = J1CommissioningBatch.for_fixed_j1_step(
        snapshot, step_raw=1000, configure_control_mode=True)
    assert batch.target_joint_raw_0p001deg == (snapshot.joint_raw_0p001deg[0] + 1000,
                                              *snapshot.joint_raw_0p001deg[1:])
    bridge.record_heartbeat(time.perf_counter_ns())
    receipt = bridge.send_supervised_j1_commissioning_step(
        snapshot=snapshot, batch=batch, authorization=_authorization(batch))
    assert [write.arbitration_id for write in receipt.writes] == [0x151, 0x155, 0x156, 0x157]
    assert receipt.writes[0].data == bytes((1, 1, 5, 0, 0, 0, 0, 0))
    assert struct.unpack(">ii", receipt.writes[1].data)[0] == batch.target_joint_raw_0p001deg[0]
    bridge.close()


@pytest.mark.parametrize("step", [0, -1000, 2000, 1000.0, True])
def test_commissioning_rejects_unselected_steps(step):
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    with pytest.raises(ValueError):
        J1CommissioningBatch.for_fixed_j1_step(snapshot, step_raw=step, configure_control_mode=True)
    assert adapter.sent == []
    bridge.close()


def test_one_degree_requires_mode_setting_and_rejects_another_speed():
    from laptop.transport.position_commands import CanTxFrame
    adapter = MemoryJ1Adapter()
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    with pytest.raises(ValueError, match="1-degree"):
        J1CommissioningBatch.for_fixed_j1_step(snapshot, step_raw=1000)
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot, step_raw=1000, configure_control_mode=True)
    with pytest.raises(ValueError, match="5%"):
        replace(batch, frames=(CanTxFrame(0x151, bytes((1, 1, 50, 0, 0, 0, 0, 0))), *batch.frames[1:]))
    bridge.close()


@pytest.mark.parametrize("fail_at", [1, 2, 3, 4])
def test_one_degree_partial_mode_or_joint_write_never_retries(fail_at):
    adapter = MemoryJ1Adapter(fail_at=fail_at)
    bridge = _bridge(adapter)
    snapshot = _snapshot(bridge, adapter)
    batch = J1CommissioningBatch.for_fixed_j1_step(snapshot, step_raw=1000, configure_control_mode=True)
    bridge.record_heartbeat(time.perf_counter_ns())
    with pytest.raises(CanJ1CommissioningDeliveryUnknown) as captured:
        bridge.send_supervised_j1_commissioning_step(
            snapshot=snapshot, batch=batch, authorization=_authorization(batch))
    assert captured.value.confirmed_frame_count == fail_at - 1
    assert len(adapter.sent) == fail_at
    assert adapter.closed
    bridge.close()
