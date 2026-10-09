"""Exercise the heartbeat watchdog after its producer process exits."""

from __future__ import annotations

import multiprocessing
import time

from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, SafetyLimits
from laptop.transport.can_transport import FakeCanTransport


def _publish_one_heartbeat_then_wait(output, heartbeat_ns: int) -> None:
    output.put(heartbeat_ns)
    time.sleep(30)


def test_worker_process_exit_leaves_stale_heartbeat_and_latches_bridge() -> None:
    context = multiprocessing.get_context("spawn")
    output = context.Queue()
    process = context.Process(
        target=_publish_one_heartbeat_then_wait,
        args=(output, 2_000_000_000),
    )
    transport = FakeCanTransport()
    limits = SafetyLimits(
        joint_min_rad=(-3.0,) * 6,
        joint_max_rad=(3.0,) * 6,
        max_joint_delta_rad=(0.1,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.10,
        max_gripper_delta_m=0.01,
        max_heartbeat_age_ns=50_000_000,
    )
    bridge = SafetyBridge(
        mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=transport,
        limits=limits,
    )

    try:
        process.start()
        heartbeat_ns = output.get(timeout=5)
        assert process.is_alive()
        process.terminate()
        process.join(timeout=5)

        assert not process.is_alive()
        assert process.exitcode != 0
        bridge.record_heartbeat(heartbeat_ns)

        decision = bridge.watchdog_tick(
            heartbeat_ns + limits.max_heartbeat_age_ns + 1
        )

        assert decision.accepted_for_offline_analysis is False
        assert decision.hardware_control_allowed is False
        assert decision.transport_send_attempted is False
        assert "heartbeat_stale" in decision.reasons
        assert decision.bridge_state is BridgeState.FAULT_LATCHED
        assert transport.send_attempts == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5)
        bridge.close()
        output.close()
        output.join_thread()
