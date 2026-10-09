from __future__ import annotations

import time
import unittest

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
)
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import FakeCanTransport


def make_inputs(now_ns: int, *, target=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04),
                valid_until_ns: int | None = None, mapping_verified: bool = True):
    snapshot = RobotStateSnapshot(
        sequence=1,
        captured_monotonic_ns=now_ns - 8_000_000,
        received_monotonic_ns=now_ns - 5_000_000,
        joint_position_rad=(0.0,) * 6,
        gripper_position_m=0.04,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    proposal = CommandProposal(
        proposal_id="fake-step-1",
        source_request_id="offline-sample-1",
        source_state_sequence=1,
        created_monotonic_ns=now_ns - 2_000_000,
        valid_until_monotonic_ns=valid_until_ns or now_ns + 20_000_000,
        target=target,
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=True,
        mapping_verified=mapping_verified,
    )
    return snapshot, proposal


def make_bridge(transport: FakeCanTransport) -> SafetyBridge:
    bridge = SafetyBridge(
        mode=RuntimeMode.FAKE_CAN_REHEARSAL,
        transport=transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
        fake_rehearsal_enabled=True,
    )
    return bridge


class FakeCanSingleStepRehearsalTest(unittest.TestCase):
    def test_only_explicit_fake_gates_allow_one_in_memory_record(self) -> None:
        transport = FakeCanTransport(allow_recorded_send=True)
        bridge = make_bridge(transport)
        now_ns = time.perf_counter_ns()
        bridge.record_heartbeat(now_ns - 1_000_000)
        snapshot, proposal = make_inputs(now_ns)
        decision = bridge.record_fake_single_step(
            snapshot=snapshot, proposal=proposal, now_monotonic_ns=now_ns)

        self.assertTrue(decision.accepted_for_offline_analysis)
        self.assertEqual(decision.event_type, "fake_command_recorded_no_can_tx")
        self.assertTrue(decision.transport_send_attempted)
        self.assertFalse(decision.hardware_control_allowed)
        audit = decision.audit_event()
        self.assertEqual(audit["transport_scope"], "fake_in_memory_record_only")
        self.assertFalse(audit["can_data_tx_enabled"])
        self.assertEqual(transport.recorded, [proposal])
        bridge.close()

    def test_stale_expired_unmapped_and_over_limit_proposals_reject_without_record(self) -> None:
        cases = (
            ("state_stale", lambda now: make_inputs(now - 1_000_000_000)),
            ("proposal_expired", lambda now: make_inputs(now, valid_until_ns=now - 1)),
            ("command_mapping_unverified", lambda now: make_inputs(now, mapping_verified=False)),
            ("joint_1_position_limit", lambda now: make_inputs(
                now, target=(3.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04))),
        )
        for expected, input_factory in cases:
            with self.subTest(expected=expected):
                transport = FakeCanTransport(allow_recorded_send=True)
                bridge = make_bridge(transport)
                now_ns = time.perf_counter_ns()
                bridge.record_heartbeat(now_ns - 1_000_000)
                snapshot, proposal = input_factory(now_ns)
                decision = bridge.record_fake_single_step(
                    snapshot=snapshot, proposal=proposal, now_monotonic_ns=now_ns)
                self.assertFalse(decision.accepted_for_offline_analysis)
                self.assertIn(expected, decision.reasons)
                self.assertEqual(transport.recorded, [])
                bridge.close()

    def test_replayed_state_transport_disconnect_proxy_and_missing_gate_fail_closed(self) -> None:
        with self.assertRaisesRegex(PermissionError, "two explicit"):
            SafetyBridge(mode=RuntimeMode.FAKE_CAN_REHEARSAL,
                         transport=FakeCanTransport(), limits=UNVERIFIED_ANALYSIS_LIMITS,
                         fake_rehearsal_enabled=False)
        class DerivedFakeTransport(FakeCanTransport):
            pass

        with self.assertRaisesRegex(PermissionError, "exact FakeCanTransport"):
            SafetyBridge(mode=RuntimeMode.FAKE_CAN_REHEARSAL,
                         transport=DerivedFakeTransport(allow_recorded_send=True),
                         limits=UNVERIFIED_ANALYSIS_LIMITS,
                         fake_rehearsal_enabled=True)

        transport = FakeCanTransport(allow_recorded_send=True)
        bridge = make_bridge(transport)
        now_ns = time.perf_counter_ns()
        bridge.record_heartbeat(now_ns - 1_000_000)
        snapshot, proposal = make_inputs(now_ns)
        accepted = bridge.record_fake_single_step(
            snapshot=snapshot, proposal=proposal, now_monotonic_ns=now_ns)
        self.assertTrue(accepted.accepted_for_offline_analysis)
        replayed = bridge.record_fake_single_step(
            snapshot=snapshot, proposal=proposal,
            now_monotonic_ns=time.perf_counter_ns())
        self.assertIn("state_out_of_order_or_replayed", replayed.reasons)
        self.assertEqual(len(transport.recorded), 1)
        bridge.close()

        for reason, expected in (("usb_disconnect", "usb_disconnected"),
                                 ("transport_process_exit", "transport_process_exited")):
            with self.subTest(disconnect=reason):
                disconnected = FakeCanTransport(allow_recorded_send=True)
                bridge = make_bridge(disconnected)
                now_ns = time.perf_counter_ns()
                bridge.record_heartbeat(now_ns - 1_000_000)
                snapshot, proposal = make_inputs(now_ns)
                disconnected.inject_disconnect(reason)
                decision = bridge.record_fake_single_step(
                    snapshot=snapshot, proposal=proposal, now_monotonic_ns=now_ns)
                self.assertFalse(decision.accepted_for_offline_analysis)
                self.assertIn(expected, decision.reasons)
                self.assertEqual(disconnected.recorded, [])
                self.assertEqual(disconnected.send_attempts, 0)
                bridge.close()

        proxy = PolicyPrediction(
            request_id="proxy", request_sequence=1,
            observation_monotonic_ns=10, received_monotonic_ns=11,
            actions=((0.0,) * 7,), target_dt_s=(0.1,),
            checkpoint_identity="checkpoint", config_identity="config",
            normalization_identity="stats", dataset_identity="dataset",
        )
        transport = FakeCanTransport(allow_recorded_send=True)
        bridge = make_bridge(transport)
        with self.assertRaisesRegex(TypeError, "not command proposals"):
            bridge.record_fake_single_step(
                snapshot=make_inputs(time.perf_counter_ns())[0], proposal=proxy,
                now_monotonic_ns=time.perf_counter_ns())
        self.assertEqual(transport.send_attempts, 0)
        bridge.close()


if __name__ == "__main__":
    unittest.main()
