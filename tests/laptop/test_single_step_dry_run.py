from __future__ import annotations

import hashlib
import json
import time
import unittest

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    JOINT_GRIPPER_UNITS,
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
)
from laptop.robot_client.single_step_dry_run import (
    P12GateEvidence,
    preflight_hil_single_step,
    rehearse_gripper_step,
    rehearse_single_step,
)
from laptop.transport.can_transport import FakeCanTransport


def preflight_inputs(now_ns: int):
    state = RobotStateSnapshot(
        sequence=7,
        captured_monotonic_ns=now_ns - 20_000_000,
        received_monotonic_ns=now_ns - 10_000_000,
        joint_position_rad=(0.0,) * 6,
        gripper_position_m=0.04,
        status_ok=True,
        connected=True,
        units_verified=True,
    )
    proposal = CommandProposal(
        proposal_id="preflight-proposal",
        source_request_id="preflight-request",
        source_state_sequence=state.sequence,
        created_monotonic_ns=now_ns - 5_000_000,
        valid_until_monotonic_ns=now_ns + 100_000_000,
        target=state.state7,
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS,
        hardware_units_verified=True,
        mapping_verified=True,
    )
    return state, proposal


class SingleStepDryRunTest(unittest.TestCase):
    def test_proposal_flows_through_bridge_and_fails_closed_by_default(self) -> None:
        fake = FakeCanTransport()
        result = rehearse_single_step(fake, joint_index=0, delta_deg=0.1)
        self.assertEqual(result["status"], "SAFETY_BRIDGE_REJECTED_NON_EXECUTABLE")
        self.assertFalse(result["accepted_for_offline_analysis"])
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["transport_send_attempted"])
        self.assertEqual(result["fake_transport_send_attempts"], 0)
        self.assertIn("state_units_unverified", result["reasons"])
        self.assertIn("hardware_units_unverified", result["reasons"])
        self.assertIn("command_mapping_unverified", result["reasons"])
        self.assertIn("runtime_mode_forbids_can_tx", result["reasons"])
        self.assertEqual(
            result["bridge_decision"]["event_type"], "command_proposal_decision")

    def test_verified_offline_proposal_still_cannot_send(self) -> None:
        fake = FakeCanTransport()
        result = rehearse_single_step(
            fake,
            joint_index=0,
            delta_deg=0.1,
            state_units_verified=True,
            mapping_verified=True,
        )
        self.assertTrue(result["accepted_for_offline_analysis"])
        self.assertEqual(result["reasons"], ["runtime_mode_forbids_can_tx"])
        self.assertFalse(result["hardware_control_allowed"])
        self.assertEqual(fake.send_attempts, 0)

    def test_real_transport_substitution_is_rejected(self) -> None:
        class NotTheFake(FakeCanTransport):
            pass

        with self.assertRaises(TypeError):
            rehearse_single_step(NotTheFake(), joint_index=0, delta_deg=0.1)

    def test_invalid_step_is_rejected_before_bridge_claim(self) -> None:
        fake = FakeCanTransport()
        with self.assertRaises(ValueError):
            rehearse_single_step(fake, joint_index=7, delta_deg=0.1)
        self.assertIsNone(fake.owner)
        self.assertEqual(fake.send_attempts, 0)

    def test_gripper_proposal_uses_same_bridge_and_fails_closed_by_default(self) -> None:
        fake = FakeCanTransport()
        result = rehearse_gripper_step(fake, current_gripper_m=0.01, delta_m=0.001)
        self.assertEqual(result["status"], "SAFETY_BRIDGE_REJECTED_NON_EXECUTABLE")
        self.assertAlmostEqual(result["target_gripper_m_nominal"], 0.011)
        self.assertIn("state_units_unverified", result["reasons"])
        self.assertIn("hardware_units_unverified", result["reasons"])
        self.assertIn("command_mapping_unverified", result["reasons"])
        self.assertIn("runtime_mode_forbids_can_tx", result["reasons"])
        self.assertEqual(
            result["bridge_decision"]["event_type"], "command_proposal_decision")
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["transport_send_attempted"])
        self.assertEqual(fake.send_attempts, 0)

    def test_joint_and_gripper_offline_verified_analysis_still_cannot_send(self) -> None:
        for rehearse, kwargs in (
            (rehearse_single_step, {"joint_index": 0, "delta_deg": 0.1}),
            (rehearse_gripper_step, {"current_gripper_m": 0.01, "delta_m": 0.001}),
        ):
            with self.subTest(rehearse=rehearse.__name__):
                fake = FakeCanTransport()
                result = rehearse(
                    fake, **kwargs,
                    state_units_verified=True,
                    mapping_verified=True,
                )
                self.assertTrue(result["accepted_for_offline_analysis"])
                self.assertEqual(result["reasons"], ["runtime_mode_forbids_can_tx"])
                self.assertFalse(result["hardware_control_allowed"])
                self.assertFalse(result["transport_send_attempted"])
                self.assertEqual(fake.send_attempts, 0)

    def test_gripper_rehearsal_rejects_nonfinite_step_before_bridge_claim(self) -> None:
        fake = FakeCanTransport()
        with self.assertRaisesRegex(ValueError, "finite"):
            rehearse_gripper_step(fake, delta_m=float("nan"))
        self.assertIsNone(fake.owner)
        self.assertEqual(fake.send_attempts, 0)

    def test_p12_preflight_reports_unverified_physical_gates_without_tx(self) -> None:
        fake = FakeCanTransport()
        now_ns = time.perf_counter_ns()
        state, proposal = preflight_inputs(now_ns)

        result = preflight_hil_single_step(
            fake,
            snapshot=state,
            proposal=proposal,
            now_monotonic_ns=now_ns,
        )

        self.assertEqual(result["status"], "OFFLINE_PREFLIGHT_BLOCKED")
        self.assertEqual(result["evaluation_scope"], "fake_can_offline_only")
        self.assertFalse(result["physical_p12_gates_verified"])
        self.assertFalse(result["readiness_for_robot_command"])
        self.assertIn("mechanical_limits_verified", result["p12_gate_missing"])
        self.assertIn("disconnect_safe_hold_verified", result["blockers"])
        self.assertIn("operator_single_step_approval_missing", result["blockers"])
        self.assertEqual(result["runtime_mode"], "POLICY_REJECT_ONLY")
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["can_tx_enabled"])
        self.assertFalse(result["transport_send_attempted"])
        self.assertEqual(fake.send_attempts, 0)

    def test_all_offline_gate_fixtures_still_cannot_enable_can_tx(self) -> None:
        fake = FakeCanTransport()
        now_ns = time.perf_counter_ns()
        state, proposal = preflight_inputs(now_ns)
        evidence = P12GateEvidence(
            **{name: True for name in P12GateEvidence.__dataclass_fields__}
        )

        result = preflight_hil_single_step(
            fake,
            snapshot=state,
            proposal=proposal,
            p12_evidence=evidence,
            operator_approved=True,
            now_monotonic_ns=now_ns,
        )

        self.assertEqual(
            result["status"], "OFFLINE_PREFLIGHT_CLEAR_TX_STILL_DISABLED"
        )
        self.assertEqual(result["evaluation_scope"], "fake_can_offline_only")
        self.assertEqual(result["blockers"], [])
        self.assertEqual(
            result["safety_decision"]["reasons"], ["runtime_mode_forbids_can_tx"]
        )
        self.assertFalse(result["can_tx_enabled"])
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["physical_p12_gates_verified"])
        self.assertFalse(result["readiness_for_robot_command"])
        self.assertEqual(result["fake_transport_send_attempts"], 0)
        self.assertEqual(fake.send_attempts, 0)

    def test_preflight_audit_hash_binds_proposal_gates_and_operator_approval(self) -> None:
        now_ns = time.perf_counter_ns()
        state, proposal = preflight_inputs(now_ns)
        all_verified_values = {
            name: True for name in P12GateEvidence.__dataclass_fields__
        }
        all_verified = P12GateEvidence(**all_verified_values)
        one_gate_missing = dict(all_verified_values)
        one_gate_missing["watchdog_verified"] = False

        def run(evidence: P12GateEvidence, operator_approved: bool) -> dict:
            fake = FakeCanTransport()
            result = preflight_hil_single_step(
                fake,
                snapshot=state,
                proposal=proposal,
                p12_evidence=evidence,
                operator_approved=operator_approved,
                now_monotonic_ns=now_ns,
            )
            self.assertEqual(fake.send_attempts, 0)
            self.assertFalse(result["can_tx_enabled"])
            self.assertFalse(result["hardware_control_allowed"])
            audit = result["preflight_audit"]
            self.assertEqual(audit["transport"]["send_attempts_before"], 0)
            self.assertEqual(audit["transport"]["send_attempts_after"], 0)
            self.assertFalse(audit["transport_send_attempted"])
            return result

        fully_approved = run(all_verified, operator_approved=True)
        gate_changed = run(
            P12GateEvidence(**one_gate_missing), operator_approved=True
        )
        approval_changed = run(all_verified, operator_approved=False)

        audit = dict(fully_approved["preflight_audit"])
        digest = audit.pop("input_sha256")
        recomputed = hashlib.sha256(
            json.dumps(
                audit,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        ).hexdigest()
        self.assertEqual(digest, recomputed)
        self.assertEqual(len({
            fully_approved["preflight_audit"]["input_sha256"],
            gate_changed["preflight_audit"]["input_sha256"],
            approval_changed["preflight_audit"]["input_sha256"],
        }), 3)

    def test_preflight_rejects_proxy_prediction_instead_of_coercing_command(self) -> None:
        fake = FakeCanTransport()
        now_ns = time.perf_counter_ns()
        state, _ = preflight_inputs(now_ns)
        prediction = PolicyPrediction(
            request_id="prediction-only",
            request_sequence=state.sequence,
            observation_monotonic_ns=state.captured_monotonic_ns,
            received_monotonic_ns=now_ns - 5_000_000,
            actions=(state.state7,),
            target_dt_s=(0.5,),
            checkpoint_identity="proxy-checkpoint",
            config_identity="proxy-config",
            normalization_identity="proxy-stats",
            dataset_identity="proxy-dataset",
        )

        with self.assertRaisesRegex(TypeError, "not HIL command proposals"):
            preflight_hil_single_step(
                fake,
                snapshot=state,
                proposal=prediction,
                now_monotonic_ns=now_ns,
            )

        self.assertIsNone(fake.owner)
        self.assertEqual(fake.send_attempts, 0)

    def test_p12_preflight_rejects_fake_transport_subclass(self) -> None:
        class NotTheFake(FakeCanTransport):
            pass

        now_ns = time.perf_counter_ns()
        state, proposal = preflight_inputs(now_ns)

        with self.assertRaisesRegex(TypeError, "exact FakeCanTransport"):
            preflight_hil_single_step(
                NotTheFake(),
                snapshot=state,
                proposal=proposal,
                now_monotonic_ns=now_ns,
            )


if __name__ == "__main__":
    unittest.main()
