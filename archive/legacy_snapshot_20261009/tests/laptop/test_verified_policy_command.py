from __future__ import annotations

import math
import unittest
from dataclasses import replace

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND,
    PolicyPrediction,
    RobotStateSnapshot,
)
from laptop.robot_client.policy_sdk_preview import preview_policy_sdk_values
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.verified_policy_command import command_proposal_from_verified_preview
from laptop.safety.bridge import SafetyLimits
from laptop.transport.position_commands import CanTxFrame, PositionCommandBatch


NOW_NS = 1_050_000_000


def _prediction() -> PolicyPrediction:
    return PolicyPrediction(
        request_id="preview-request-1",
        request_sequence=3,
        observation_monotonic_ns=1_000_000_000,
        received_monotonic_ns=1_010_000_000,
        actions=(
            (0.0, math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.0, 0.05),
            (0.01, math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.0, 0.051),
        ),
        target_dt_s=(0.1, 0.2),
        checkpoint_identity="checkpoint-fixture",
        config_identity="config-fixture",
        normalization_identity="stats-fixture",
        dataset_identity="dataset-fixture",
    )


def _snapshot(*, sequence: int = 4, received_ns: int = 1_040_000_000,
              units_verified: bool = True) -> RobotStateSnapshot:
    return RobotStateSnapshot(
        sequence=sequence,
        captured_monotonic_ns=received_ns - 1_000_000,
        received_monotonic_ns=received_ns,
        joint_position_rad=(0.0, math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.0),
        gripper_position_m=0.05,
        status_ok=True,
        connected=True,
        units_verified=units_verified,
    )


def _preview(*, configured_gripper_max_mm: float | None = 100.0):
    prediction = _prediction()
    snapshot = _snapshot()
    return preview_policy_sdk_values(
        prediction,
        current_state=snapshot,
        now_monotonic_ns=NOW_NS,
        gripper_effort_raw=120,
        max_joint_delta_rad=(0.02,) * 6,
        max_gripper_delta_m=0.002,
        configured_gripper_max_mm=configured_gripper_max_mm,
    )


def _complete_evidence() -> P12GateEvidence:
    return P12GateEvidence(**{
        name: True for name in P12GateEvidence.__dataclass_fields__
    })


def _limits(*, max_joint_delta: float = 0.02,
            max_gripper_delta: float = 0.002) -> SafetyLimits:
    return SafetyLimits(
        joint_min_rad=(-3.2,) * 6,
        joint_max_rad=(3.2,) * 6,
        max_joint_delta_rad=(max_joint_delta,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.1,
        max_gripper_delta_m=max_gripper_delta,
    )


class VerifiedPolicyCommandTest(unittest.TestCase):
    def test_converts_only_after_every_site_gate_and_limits_proposal_lifetime(self) -> None:
        preview = _preview()
        proposal = command_proposal_from_verified_preview(
            preview,
            snapshot=_snapshot(),
            evidence=_complete_evidence(),
            limits=_limits(),
            now_monotonic_ns=NOW_NS,
        )

        self.assertEqual(proposal.source_request_id, "preview-request-1")
        self.assertEqual(proposal.source_state_sequence, 4)
        self.assertEqual(proposal.action_semantics, ABSOLUTE_JOINT_POSITION_COMMAND)
        self.assertEqual(proposal.target,
                         (0.0, math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.0, 0.05))
        self.assertEqual(proposal.created_monotonic_ns, NOW_NS)
        self.assertEqual(proposal.valid_until_monotonic_ns, preview.target_monotonic_ns)
        self.assertTrue(proposal.hardware_units_verified)
        self.assertTrue(proposal.mapping_verified)

    def test_position_batch_preserves_sdk_preview_frame_bytes(self) -> None:
        preview = _preview()
        proposal = command_proposal_from_verified_preview(
            preview,
            snapshot=_snapshot(),
            evidence=_complete_evidence(),
            limits=_limits(),
            now_monotonic_ns=NOW_NS,
        )
        batch = PositionCommandBatch(
            proposal,
            tuple(CanTxFrame(frame.arbitration_id, frame.data) for frame in preview.frames),
        )

        self.assertEqual(
            tuple((frame.arbitration_id, frame.data) for frame in batch.frames),
            tuple((frame.arbitration_id, frame.data) for frame in preview.frames),
        )

    def test_missing_physical_evidence_never_becomes_command_semantics(self) -> None:
        preview = _preview()
        evidence = P12GateEvidence()
        with self.assertRaisesRegex(PermissionError, "site verification is incomplete"):
            command_proposal_from_verified_preview(
                preview, snapshot=_snapshot(), evidence=evidence,
                limits=_limits(),
                now_monotonic_ns=NOW_NS,
            )

    def test_stale_or_mismatched_state_and_unchecked_gripper_limit_reject(self) -> None:
        evidence = _complete_evidence()
        with self.subTest(reason="state sequence"):
            with self.assertRaisesRegex(ValueError, "different PiPER state"):
                command_proposal_from_verified_preview(
                    _preview(), snapshot=_snapshot(sequence=5), evidence=evidence,
                    limits=_limits(),
                    now_monotonic_ns=NOW_NS,
                )
        with self.subTest(reason="state age"):
            with self.assertRaisesRegex(ValueError, "state is stale"):
                command_proposal_from_verified_preview(
                    _preview(), snapshot=_snapshot(received_ns=NOW_NS - 300_000_000),
                    evidence=evidence, limits=_limits(), now_monotonic_ns=NOW_NS,
                )
        with self.subTest(reason="gripper limit missing"):
            with self.assertRaisesRegex(ValueError, "verified gripper travel limit"):
                command_proposal_from_verified_preview(
                    _preview(configured_gripper_max_mm=None), snapshot=_snapshot(),
                    evidence=evidence, limits=_limits(), now_monotonic_ns=NOW_NS,
                )

    def test_expired_prediction_is_rejected_and_never_transmitted(self) -> None:
        with self.assertRaisesRegex(ValueError, "policy target has expired"):
            command_proposal_from_verified_preview(
                _preview(), snapshot=_snapshot(), evidence=_complete_evidence(),
                limits=_limits(),
                now_monotonic_ns=1_150_000_000,
            )

    def test_verified_position_and_single_step_limits_are_applied_after_preview(self) -> None:
        prediction = _prediction()
        snapshot = _snapshot()
        preview = preview_policy_sdk_values(
            prediction,
            current_state=snapshot,
            now_monotonic_ns=NOW_NS,
            gripper_effort_raw=120,
            max_joint_delta_rad=(0.02,) * 6,
            max_gripper_delta_m=0.002,
            action_index=1,
            configured_gripper_max_mm=100.0,
        )
        with self.assertRaisesRegex(ValueError, "verified single-step limit"):
            command_proposal_from_verified_preview(
                preview, snapshot=snapshot, evidence=_complete_evidence(),
                limits=_limits(max_joint_delta=0.005), now_monotonic_ns=NOW_NS,
            )

    def test_observed_full_open_endpoint_above_100mm_is_rejected_before_proposal(self) -> None:
        base = _prediction()
        prediction = replace(
            base,
            actions=(
                base.actions[0],
                (*base.actions[1][:6], 0.1018),
            ),
        )
        snapshot = _snapshot()
        preview = preview_policy_sdk_values(
            prediction,
            current_state=snapshot,
            now_monotonic_ns=NOW_NS,
            gripper_effort_raw=120,
            max_joint_delta_rad=(0.02,) * 6,
            max_gripper_delta_m=0.06,
            configured_gripper_max_mm=100.0,
            action_index=1,
        )

        self.assertEqual(preview.gripper_ctrl_args[0], 101_800)
        self.assertTrue(preview.gripper_target_exceeds_configured_max)
        with self.assertRaisesRegex(ValueError, "gripper target exceeds"):
            command_proposal_from_verified_preview(
                preview,
                snapshot=snapshot,
                evidence=_complete_evidence(),
                limits=_limits(max_gripper_delta=0.06),
                now_monotonic_ns=NOW_NS,
            )


if __name__ == "__main__":
    unittest.main()
