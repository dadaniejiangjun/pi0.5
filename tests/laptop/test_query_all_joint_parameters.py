from __future__ import annotations

import struct
import unittest

from laptop.diagnostics.query_all_joint_parameters import (
    decode_acceleration_payload,
    decode_limit_payload,
    query,
    request_payload,
)


class AllJointParameterQueryTest(unittest.TestCase):
    def test_query_payloads_are_documented_read_queries_only(self) -> None:
        self.assertEqual(request_payload(1, 0x01).hex(), "0101000000000000")
        self.assertEqual(request_payload(6, 0x02).hex(), "0602000000000000")
        for motor in (0, 7, True, 1.0):
            with self.subTest(motor=motor), self.assertRaises(ValueError):
                request_payload(motor, 0x01)
        for content in (0, 3, True):
            with self.subTest(content=content), self.assertRaises(ValueError):
                request_payload(1, content)

    def test_angle_speed_fields_keep_raw_and_convert_units(self) -> None:
        payload = bytes((1,)) + struct.pack(">hhH", 1700, -1700, 3000) + b"\x00"
        self.assertEqual(decode_limit_payload(payload, 1), {
            "motor_num": 1,
            "minimum_raw_0p1deg": -1700,
            "maximum_raw_0p1deg": 1700,
            "minimum_deg": -170.0,
            "maximum_deg": 170.0,
            "maximum_speed_raw_0p001_rad_s": 3000,
            "maximum_rad_s": 3.0,
            "raw_payload_hex": payload.hex(),
        })
        with self.assertRaises(ValueError):
            decode_limit_payload(payload, 2)
        with self.assertRaises(ValueError):
            decode_limit_payload(bytes((1,)) + struct.pack(">hhH", 1700, -1700, 3001) + b"\x00", 1)

    def test_acceleration_fields_keep_raw_and_convert_sdk_unit(self) -> None:
        payload = bytes((4,)) + struct.pack(">H", 5000) + bytes(5)
        self.assertEqual(decode_acceleration_payload(payload, 4), {
            "motor_num": 4,
            "maximum_acceleration_raw": 5000,
            "maximum_rad_s2_sdk_documented": 5.0,
            "raw_payload_hex": payload.hex(),
        })
        with self.assertRaises(ValueError):
            decode_acceleration_payload(payload, 3)

    def test_no_transmit_without_explicit_flag(self) -> None:
        with self.assertRaises(PermissionError):
            query(approved=False)


if __name__ == "__main__":
    unittest.main()
