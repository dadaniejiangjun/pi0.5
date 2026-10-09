from __future__ import annotations

import unittest
import json
from pathlib import Path
import struct
import tempfile
import time

from laptop.diagnostics.probe_piper_feedback_ack_only import probe
from laptop.transport.can_transport import CanFrame, FakeCanTransport
from laptop.transport.gs_usb_piper_can import CanReceiveMode


class FeedbackAckOnlyGuardTest(unittest.TestCase):
    def test_default_blocks_before_usb_or_sdk_import(self) -> None:
        result = probe(5)
        self.assertEqual(result["status"], "BLOCKED_APPROVAL_REQUIRED")
        self.assertFalse(result["can_started"])

    def test_duration_is_bounded(self) -> None:
        with self.assertRaises(ValueError):
            probe(5.1)
        with self.assertRaises(ValueError):
            probe(10.1, max_duration_s=10)

    def test_ten_second_mode_still_requires_explicit_approval(self) -> None:
        result = probe(10, max_duration_s=10)
        self.assertEqual(result["status"], "BLOCKED_APPROVAL_REQUIRED")
        self.assertFalse(result["can_started"])

    def test_approved_read_uses_safety_bridge_and_releases_fake_transport(self) -> None:
        transport = FakeCanTransport()
        result = probe(0.001, approved=True, transport=transport)
        self.assertEqual(result["status"], "INCOMPLETE_FEEDBACK")
        self.assertEqual(result["safety_bridge_mode"], "POLICY_REJECT_ONLY")
        self.assertEqual(result["can_owner"], "safety-bridge")
        self.assertFalse(result["hardware_control_allowed"])
        self.assertFalse(result["command_tx_enabled"])
        self.assertEqual(transport.send_attempts, 0)
        self.assertIsNone(transport.owner)
        self.assertFalse(transport.read_only_active)

    def test_probe_source_uses_no_direct_adapter_io(self) -> None:
        path = Path(__file__).resolve().parents[2] / "laptop" / "diagnostics" / "probe_piper_feedback_ack_only.py"
        source = path.read_text(encoding="utf-8")
        self.assertIn("SafetyBridge", source)
        self.assertNotIn("transport.claim(", source)
        self.assertNotIn("transport.receive_frame(", source)
        self.assertNotIn("GsUsb.scan", source)
        self.assertNotIn("device.send(", source)

    def test_command_log_passively_records_only_raw_target_frames_without_tx(self) -> None:
        transport = FakeCanTransport()
        timestamp = time.perf_counter_ns()
        frames = (
            (0x159, struct.pack(">iHBB", 12_000, 250, 1, 0)),
            (0x155, struct.pack(">ii", 100, 200)),
            (0x156, struct.pack(">ii", 300, 400)),
            (0x157, struct.pack(">ii", 500, 600)),
        )
        for index, (can_id, payload) in enumerate(frames):
            transport.enqueue_frame(CanFrame(can_id, payload, False, False, timestamp + index))

        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "command_can_raw.jsonl"
            result = probe(0.01, approved=True, transport=transport, command_log=path,
                           receive_mode=CanReceiveMode.LISTEN_ONLY)
            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]

        self.assertEqual(len(rows), 4)
        self.assertEqual([row["can_id"] for row in rows],
                         ["0x159", "0x155", "0x156", "0x157"])
        self.assertEqual(rows[0]["raw_payload_hex"], frames[0][1].hex())
        self.assertTrue(all(row["direction_verified"] is False for row in rows))
        self.assertEqual(result["mode"], "listen_only_receive")
        self.assertEqual(result["observed_command_id_counts"]["0x155"], 1)
        self.assertFalse(result["command_targets_decoded"])
        self.assertEqual(result["can_data_tx_calls"], 0)
        self.assertEqual(transport.send_attempts, 0)


if __name__ == "__main__":
    unittest.main()
