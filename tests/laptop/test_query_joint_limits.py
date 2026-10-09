from __future__ import annotations

import struct
import unittest

from laptop.diagnostics.query_joint_limits import decode_limit_payload, query, request_payload


class JointLimitQueryTest(unittest.TestCase):
    def test_exact_query_shape_and_restriction(self) -> None:
        self.assertEqual(request_payload(2).hex(), "0201000000000000")
        self.assertEqual(request_payload(3).hex(), "0301000000000000")
        for motor in (0, 1, 4, 7, True):
            with self.subTest(motor=motor), self.assertRaises(ValueError):
                request_payload(motor)

    def test_reply_decoding_and_rejects(self) -> None:
        payload = bytes((2,)) + struct.pack(">hhH", 1800, -20, 3000) + b"\x00"
        self.assertEqual(decode_limit_payload(payload, 2), {
            "motor_num": 2, "minimum_deg": -2.0,
            "maximum_deg": 180.0, "maximum_rad_s": 3.0})
        with self.assertRaises(ValueError):
            decode_limit_payload(payload, 3)
        with self.assertRaises(ValueError):
            decode_limit_payload(payload[:7], 2)

    def test_no_query_without_explicit_flag(self) -> None:
        with self.assertRaises(PermissionError):
            query(approved=False)


if __name__ == "__main__":
    unittest.main()
