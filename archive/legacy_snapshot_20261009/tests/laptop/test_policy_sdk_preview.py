from __future__ import annotations

import math
import unittest
from dataclasses import replace

from laptop.control.contracts import PolicyPrediction, RobotStateSnapshot
from laptop.robot_client.policy_sdk_preview import preview_policy_sdk_values


def prediction(*, actions=None) -> PolicyPrediction:
    selected_actions = actions or (
        (0.0, math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.0, 0.05),
        (0.1, math.pi / 2, -math.pi / 2, 0.0, 0.0, 0.0, 0.1018),
    )
    return PolicyPrediction(
        request_id="offline-preview-1",
        request_sequence=7,
        observation_monotonic_ns=1_000_000_000,
        received_monotonic_ns=1_010_000_000,
        actions=selected_actions,
        target_dt_s=tuple(0.1 * (index + 1) for index in range(len(selected_actions))),
        checkpoint_identity="step-9-sha256:fixture",
        config_identity="derived-state-10fps:fixture",
        normalization_identity="train-only-stats:fixture",
        dataset_identity="local/cef166e2_proxy10fps_ds36",
    )


def state_snapshot(values, *, received_ns: int, sequence: int = 7) -> RobotStateSnapshot:
    return RobotStateSnapshot(
        sequence=sequence,
        captured_monotonic_ns=received_ns - 1_000_000,
        received_monotonic_ns=received_ns,
        joint_position_rad=tuple(values[:6]),
        gripper_position_m=float(values[6]),
        status_ok=True,
        connected=True,
        units_verified=False,
    )


def preview(pred: PolicyPrediction, *, now_ns: int, state_values=None,
            state_received_ns: int | None = None, action_index=None,
            max_joint_delta_rad=(0.01,) * 6, max_gripper_delta_m=0.001,
            gripper_effort_raw=120, configured_gripper_max_mm=None):
    if state_values is None:
        state_values = pred.actions[0]
    if state_received_ns is None:
        state_received_ns = now_ns - 10_000_000
    return preview_policy_sdk_values(
        pred,
        current_state=state_snapshot(state_values, received_ns=state_received_ns),
        now_monotonic_ns=now_ns,
        gripper_effort_raw=gripper_effort_raw,
        max_joint_delta_rad=max_joint_delta_rad,
        max_gripper_delta_m=max_gripper_delta_m,
        action_index=action_index,
        configured_gripper_max_mm=configured_gripper_max_mm,
    )


class PolicySdkPreviewTest(unittest.TestCase):
    def test_rejects_old_or_slow_policy_responses_even_when_target_is_future(self) -> None:
        pred = prediction()
        long_horizon = replace(
            pred,
            actions=(pred.actions[0],),
            target_dt_s=(1.0,),
        )
        with self.assertRaisesRegex(ValueError, "response is stale"):
            preview(
                long_horizon,
                now_ns=1_310_000_000,
                state_received_ns=1_300_000_000,
                configured_gripper_max_mm=100.0,
            )

        slow_response = replace(
            pred,
            actions=(pred.actions[0],),
            received_monotonic_ns=1_300_000_000,
            target_dt_s=(1.0,),
        )
        with self.assertRaisesRegex(ValueError, "latency exceeds"):
            preview(
                slow_response,
                now_ns=1_310_000_000,
                state_received_ns=1_300_000_000,
                configured_gripper_max_mm=100.0,
            )

    def test_maps_seven_dimensions_to_v2_sdk_units_and_frames(self) -> None:
        result = preview(
            prediction(), now_ns=1_050_000_000,
            gripper_effort_raw=120, configured_gripper_max_mm=100.0,
        )
        self.assertEqual(result.action_index, 0)
        self.assertEqual(result.target_monotonic_ns, 1_100_000_000)
        self.assertEqual(result.action_target_kind,
                         "derived_future_observed_state_position_target")
        self.assertEqual(result.joint_ctrl_args_0p001deg,
                         (0, 90_000, -90_000, 0, 0, 0))
        self.assertEqual(result.gripper_ctrl_args, (50_000, 120, 0x01, 0x00))
        self.assertEqual([frame.arbitration_id for frame in result.frames],
                         [0x155, 0x156, 0x157, 0x159])
        self.assertEqual(result.frames[0].data.hex(), "0000000000015f90")
        self.assertEqual(result.frames[1].data.hex(), "fffea07000000000")
        self.assertEqual(result.frames[3].data.hex(), "0000c35000780100")
        self.assertFalse(result.hardware_control_allowed)
        self.assertFalse(result.can_tx_allowed)

    def test_frame_previews_match_installed_official_sdk_encoder(self) -> None:
        import can
        from piper_sdk.piper_msgs.msg_v2 import (
            ArmMsgGripperCtrl,
            ArmMsgJointCtrl,
            ArmMsgType,
            PiperMessage,
        )
        from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

        result = preview(
            prediction(), now_ns=1_050_000_000,
        )
        parser = C_PiperParserV2()
        message = PiperMessage()
        message.arm_joint_ctrl = ArmMsgJointCtrl(*result.joint_ctrl_args_0p001deg)
        for frame, msg_type in zip(
                result.frames[:3],
                (ArmMsgType.PiperMsgJointCtrl_12,
                 ArmMsgType.PiperMsgJointCtrl_34,
                 ArmMsgType.PiperMsgJointCtrl_56)):
            message.type_ = msg_type
            official = can.Message()
            self.assertTrue(parser.EncodeMessage(message, official))
            self.assertEqual(frame.arbitration_id, official.arbitration_id)
            self.assertEqual(frame.data, bytes(official.data))

        message.type_ = ArmMsgType.PiperMsgGripperCtrl
        message.arm_gripper_ctrl = ArmMsgGripperCtrl(*result.gripper_ctrl_args)
        official = can.Message()
        self.assertTrue(parser.EncodeMessage(message, official))
        self.assertEqual(result.frames[3].arbitration_id, official.arbitration_id)
        self.assertEqual(result.frames[3].data, bytes(official.data))

    def test_skips_expired_chunk_rows_and_flags_100mm_setting_without_clamping(self) -> None:
        pred = prediction()
        result = preview(
            pred, now_ns=1_150_000_000,
            state_values=pred.actions[1], state_received_ns=1_140_000_000,
            gripper_effort_raw=120, configured_gripper_max_mm=100.0,
        )
        self.assertEqual(result.action_index, 1)
        self.assertEqual(result.gripper_ctrl_args[0], 101_800)
        self.assertTrue(result.gripper_target_exceeds_configured_max)
        self.assertFalse(result.sdk_runtime_gripper_clamp_simulated)
        self.assertFalse(result.policy_to_robot_command_mapping_verified)

    def test_refuses_expired_rows_invalid_effort_negative_gripper_and_bad_joints(self) -> None:
        with self.assertRaisesRegex(ValueError, "expired"):
            preview(
                prediction(), now_ns=1_200_000_000,
                state_received_ns=1_190_000_000,
                action_index=1, gripper_effort_raw=120,
            )
        with self.assertRaisesRegex(ValueError, "effort"):
            preview(
                prediction(), now_ns=1_050_000_000,
                gripper_effort_raw=True,
            )
        with self.assertRaisesRegex(ValueError, "negative gripper"):
            preview(
                prediction(actions=((0, 0, 0, 0, 0, 0, -0.001),)),
                now_ns=1_050_000_000,
                max_gripper_delta_m=1.0, gripper_effort_raw=120,
            )
        with self.assertRaisesRegex(ValueError, "joint_2"):
            preview(
                prediction(actions=((0, -0.01, 0, 0, 0, 0, 0),)),
                now_ns=1_050_000_000, gripper_effort_raw=120,
            )

    def test_rejects_joint_and_gripper_increments_outside_preview_envelope(self) -> None:
        pred = prediction()
        with self.assertRaisesRegex(ValueError, "joint_1 target exceeds the preview"):
            preview(
                pred, now_ns=1_150_000_000, state_values=pred.actions[0],
                state_received_ns=1_140_000_000, action_index=1,
            )
        current = list(pred.actions[1])
        current[6] -= 0.002
        with self.assertRaisesRegex(ValueError, "gripper target exceeds the preview"):
            preview(
                pred, now_ns=1_150_000_000, state_values=current,
                state_received_ns=1_140_000_000, action_index=1,
            )

    def test_refuses_untyped_prediction_and_requires_explicit_effort(self) -> None:
        with self.assertRaisesRegex(TypeError, "PolicyPrediction"):
            preview_policy_sdk_values(
                {"actions": [[0] * 7]}, current_state=state_snapshot(
                    prediction().actions[0], received_ns=1_040_000_000),
                now_monotonic_ns=1_050_000_000, gripper_effort_raw=120,
                max_joint_delta_rad=(0.01,) * 6, max_gripper_delta_m=0.001,
            )
        with self.assertRaisesRegex(TypeError, "gripper_effort_raw"):
            preview_policy_sdk_values(  # type: ignore[call-arg]
                prediction(), current_state=state_snapshot(
                    prediction().actions[0], received_ns=1_040_000_000),
                now_monotonic_ns=1_050_000_000,
                max_joint_delta_rad=(0.01,) * 6, max_gripper_delta_m=0.001,
            )


if __name__ == "__main__":
    unittest.main()
