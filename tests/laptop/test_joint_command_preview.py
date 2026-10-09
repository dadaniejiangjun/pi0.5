from __future__ import annotations

import unittest

from laptop.robot_client.joint_command_preview import preview_single_joint_step


class JointCommandPreviewTest(unittest.TestCase):
    def test_exact_six_joint_frames(self) -> None:
        result = preview_single_joint_step(
            [0, 10, -10, 0, 0, 0], joint_index=0, delta_deg=0.1)
        self.assertEqual([frame.arbitration_id for frame in result.frames],
                         [0x155, 0x156, 0x157])
        self.assertEqual(result.frames[0].data.hex(), "0000006400002710")
        self.assertEqual(result.frames[1].data.hex(), "ffffd8f000000000")
        self.assertEqual(result.frames[2].data.hex(), "0000000000000000")
        self.assertFalse(result.hardware_control_allowed)

    def test_preview_matches_installed_official_parser(self) -> None:
        import can
        from piper_sdk.piper_msgs.msg_v2 import ArmMsgJointCtrl, ArmMsgType, PiperMessage
        from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

        result = preview_single_joint_step(
            [0, 10, -10, 0, 0, 0], joint_index=0, delta_deg=0.1)
        message = PiperMessage()
        message.arm_joint_ctrl = ArmMsgJointCtrl(100, 10000, -10000, 0, 0, 0)
        parser = C_PiperParserV2()
        for index, msg_type in enumerate((ArmMsgType.PiperMsgJointCtrl_12,
                                          ArmMsgType.PiperMsgJointCtrl_34,
                                          ArmMsgType.PiperMsgJointCtrl_56)):
            message.type_ = msg_type
            official = can.Message()
            self.assertTrue(parser.EncodeMessage(message, official))
            self.assertEqual(result.frames[index].arbitration_id, official.arbitration_id)
            self.assertEqual(result.frames[index].data, bytes(official.data))

    def test_held_joint_out_of_bounds_rejects_single_joint_step(self) -> None:
        # Current live feedback has J2=-1.995 and J3=+1.435 nominal degrees.
        with self.assertRaisesRegex(ValueError, "current joint outside"):
            preview_single_joint_step(
                [-52.852, -1.995, 1.435, -24.933, 29.091, 15.427],
                joint_index=0, delta_deg=0.1)

    def test_bad_step_or_target_fails_closed(self) -> None:
        base = [0, 10, -10, 0, 0, 0]
        for delta in (0, 0.3, -0.3, float("nan")):
            with self.subTest(delta=delta), self.assertRaises(ValueError):
                preview_single_joint_step(base, joint_index=0, delta_deg=delta)
        with self.assertRaises(ValueError):
            preview_single_joint_step([150, *base[1:]], joint_index=0, delta_deg=0.1)


if __name__ == "__main__":
    unittest.main()
