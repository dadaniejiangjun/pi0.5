from __future__ import annotations

import unittest

from laptop.diagnostics.query_piper_firmware import (
    FIRMWARE_QUERY_CAN_ID,
    FIRMWARE_QUERY_PAYLOAD,
    decode_firmware_payloads,
    query,
)


class FirmwareQueryTest(unittest.TestCase):
    def test_query_frame_matches_documented_non_motion_request(self) -> None:
        self.assertEqual(FIRMWARE_QUERY_CAN_ID, 0x4AF)
        self.assertEqual(FIRMWARE_QUERY_PAYLOAD, bytes((1, 0, 0, 0, 0, 0, 0, 0)))

    def test_decodes_version_split_across_response_frames(self) -> None:
        payloads = [b"xxxxxS-V", b"1.5-2xxx"]
        self.assertEqual(decode_firmware_payloads(payloads), "S-V1.5-2")

    def test_rejects_missing_truncated_and_malformed_response(self) -> None:
        with self.assertRaisesRegex(ValueError, "marker"):
            decode_firmware_payloads([b"no-versn"])
        with self.assertRaisesRegex(ValueError, "truncated"):
            decode_firmware_payloads([b"12345S-V"])
        with self.assertRaisesRegex(ValueError, "8 bytes"):
            decode_firmware_payloads([b"S-V"])
        with self.assertRaisesRegex(ValueError, "non-printable"):
            decode_firmware_payloads([b"S-V1.5-\x00"])

    def test_query_requires_approval_before_loading_hardware_modules(self) -> None:
        with self.assertRaisesRegex(PermissionError, "explicit"):
            query(approved=False)
        with self.assertRaisesRegex(ValueError, "within"):
            query(approved=True, timeout_s=4.0)


if __name__ == "__main__":
    unittest.main()
