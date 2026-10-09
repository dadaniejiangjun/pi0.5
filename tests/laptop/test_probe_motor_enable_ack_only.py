from __future__ import annotations

import time
import unittest
from pathlib import Path

from laptop.diagnostics.probe_motor_enable_ack_only import (
    FEEDBACK_IDS,
    decode_motor_enable,
    probe,
)
from laptop.transport.can_transport import CanFrame, FakeCanTransport


class MotorEnableDecodeTest(unittest.TestCase):
    def test_sdk_bit_six_enable_mapping(self) -> None:
        payload = bytes((0, 0, 0, 0, 0, 0x40, 0, 0))
        self.assertEqual(decode_motor_enable(0x261, payload), (1, True, 0x40))
        self.assertEqual(decode_motor_enable(0x266, payload), (6, True, 0x40))
        self.assertEqual(decode_motor_enable(0x263, bytes(8)), (3, False, 0))

    def test_wrong_id_or_size_rejected(self) -> None:
        for can_id, payload in ((0x260, bytes(8)), (0x267, bytes(8)),
                                (0x261, bytes(7))):
            with self.subTest(can_id=can_id), self.assertRaises(ValueError):
                decode_motor_enable(can_id, payload)

    def test_probe_requires_approval_and_uses_safety_bridge_transport(self) -> None:
        blocked = probe(duration_s=0.01)
        self.assertEqual(blocked["status"], "BLOCKED_APPROVAL_REQUIRED")
        self.assertFalse(blocked["can_started"])

        transport = FakeCanTransport()
        now_ns = time.perf_counter_ns()
        for _ in range(2):
            for can_id in sorted(FEEDBACK_IDS):
                payload = bytearray(8)
                payload[5] = 0x40
                transport.enqueue_frame(CanFrame(
                    can_id, bytes(payload), False, False, now_ns
                ))
        result = probe(duration_s=0.001, approved=True, transport=transport)

        self.assertEqual(result["status"], "ALL_SIX_ENABLED")
        self.assertEqual(result["can_owner"], "safety-bridge")
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["command_tx_enabled"])
        self.assertEqual(transport.send_attempts, 0)
        self.assertIsNone(transport.owner)
        self.assertFalse(transport.read_only_active)

    def test_probe_source_has_no_direct_adapter_io(self) -> None:
        path = Path(__file__).resolve().parents[2] / "laptop" / "diagnostics" / "probe_motor_enable_ack_only.py"
        source = path.read_text(encoding="utf-8")
        self.assertIn("SafetyBridge", source)
        self.assertNotIn("GsUsb.scan", source)
        self.assertNotIn("device.read(", source)
        self.assertNotIn("device.send(", source)


if __name__ == "__main__":
    unittest.main()
