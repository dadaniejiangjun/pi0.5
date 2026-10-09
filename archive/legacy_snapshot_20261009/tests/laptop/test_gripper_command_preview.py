from __future__ import annotations

import unittest

from laptop.robot_client.gripper_command_preview import preview_opening


class GripperCommandPreviewTest(unittest.TestCase):
    def test_source_start_preview_matches_official_encoder(self) -> None:
        import can
        from piper_sdk.piper_msgs.msg_v2 import ArmMsgGripperCtrl, ArmMsgType, PiperMessage
        from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2

        preview = preview_opening(7100, effort_raw=0)
        message = PiperMessage()
        message.type_ = ArmMsgType.PiperMsgGripperCtrl
        message.arm_gripper_ctrl = ArmMsgGripperCtrl(7100, 0, 0x01, 0x00)
        official = can.Message()
        self.assertTrue(C_PiperParserV2().EncodeMessage(message, official))
        self.assertEqual(preview.arbitration_id, official.arbitration_id)
        self.assertEqual(preview.data, bytes(official.data))
        self.assertEqual(preview.data.hex(), "00001bbc00000100")
        self.assertTrue(preview.includes_gripper_enable)
        self.assertFalse(preview.hardware_control_allowed)

    def test_limits_reject_bad_values(self) -> None:
        for target in (-1, 70001, 1.0, True):
            with self.subTest(target=target), self.assertRaises(ValueError):
                preview_opening(target)
        for effort in (-1, 5001, 0.5, True):
            with self.subTest(effort=effort), self.assertRaises(ValueError):
                preview_opening(7100, effort_raw=effort)


if __name__ == "__main__":
    unittest.main()
