from __future__ import annotations

import unittest
import time
from pathlib import Path

from laptop.diagnostics.probe_can_listen_only import probe
from laptop.transport.can_transport import CanFrame, FakeCanTransport


class ListenOnlyGuardTest(unittest.TestCase):
    def test_default_blocks_before_usb_import_or_access(self) -> None:
        result = probe(10, 1_000_000)
        self.assertEqual(result["status"], "BLOCKED_ACK_PEER_UNCONFIRMED")
        self.assertFalse(result["can_started"])

    def test_rejects_unsupported_bitrate(self) -> None:
        with self.assertRaises(ValueError):
            probe(10, 500_000)

    def test_approved_probe_uses_shared_safety_bridge_reader(self) -> None:
        transport = FakeCanTransport()
        transport.enqueue_frame(CanFrame(
            0x123, bytes(8), False, False, time.perf_counter_ns()
        ))
        result = probe(
            0.001,
            1_000_000,
            acknowledging_peer_confirmed=True,
            transport=transport,
        )
        self.assertEqual(result["status"], "PASSIVE_FRAMES_OBSERVED")
        self.assertEqual(result["can_owner"], "safety-bridge")
        self.assertEqual(result["safety_bridge_mode"], "POLICY_REJECT_ONLY")
        self.assertFalse(result["hardware_control_allowed"])
        self.assertEqual(result["can_tx_calls"], 0)
        self.assertEqual(transport.send_attempts, 0)
        self.assertIsNone(transport.owner)
        self.assertFalse(transport.read_only_active)

    def test_probe_source_does_not_open_adapter_directly(self) -> None:
        path = Path(__file__).resolve().parents[2] / "laptop" / "diagnostics" / "probe_can_listen_only.py"
        source = path.read_text(encoding="utf-8")
        self.assertIn("SafetyBridge", source)
        self.assertNotIn("GsUsb.scan", source)
        self.assertNotIn("device.read(", source)
        self.assertNotIn("device.send(", source)


if __name__ == "__main__":
    unittest.main()
