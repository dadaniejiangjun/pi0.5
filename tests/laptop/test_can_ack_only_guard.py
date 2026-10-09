from __future__ import annotations

import unittest
import time
from pathlib import Path

from laptop.diagnostics.probe_can_ack_only import probe
from laptop.transport.can_transport import CanFrame, FakeCanTransport


class AckOnlyGuardTest(unittest.TestCase):
    def test_default_blocks_before_usb_import_or_access(self) -> None:
        result = probe(5)
        self.assertEqual(result["status"], "BLOCKED_APPROVAL_REQUIRED")
        self.assertFalse(result["can_started"])

    def test_duration_is_bounded(self) -> None:
        with self.assertRaises(ValueError):
            probe(6)

    def test_approved_probe_uses_fake_transport_through_safety_bridge(self) -> None:
        transport = FakeCanTransport()
        now_ns = time.perf_counter_ns()
        transport.enqueue_frame(CanFrame(0x123, bytes(8), False, False, now_ns))
        transport.enqueue_frame(CanFrame(0x200, bytes(8), False, True, now_ns))

        result = probe(0.001, approved=True, transport=transport)

        self.assertEqual(result["status"], "ACK_ONLY_FRAMES_OBSERVED")
        self.assertEqual(result["data_frame_count"], 1)
        self.assertEqual(result["error_frame_count"], 1)
        self.assertEqual(result["can_owner"], "safety-bridge")
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["command_tx_enabled"])
        self.assertEqual(transport.send_attempts, 0)
        self.assertIsNone(transport.owner)
        self.assertFalse(transport.read_only_active)

    def test_probe_source_has_no_direct_adapter_io(self) -> None:
        path = Path(__file__).resolve().parents[2] / "laptop" / "diagnostics" / "probe_can_ack_only.py"
        source = path.read_text(encoding="utf-8")
        self.assertIn("SafetyBridge", source)
        self.assertNotIn("GsUsb.scan", source)
        self.assertNotIn("device.read(", source)
        self.assertNotIn("device.send(", source)


if __name__ == "__main__":
    unittest.main()
