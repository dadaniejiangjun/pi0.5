from __future__ import annotations

import json
from pathlib import Path
import struct
import tempfile
import unittest

from laptop.recorder.piper_command_targets import (
    decode_command_timeline,
    target_at,
)


def write_log(path: Path, frames: list[tuple[int, int, bytes]]) -> None:
    with path.open("x", encoding="utf-8") as handle:
        for index, (timestamp, can_id, payload) in enumerate(frames):
            handle.write(json.dumps({
                "monotonic_ns": timestamp,
                "can_id": f"0x{can_id:03X}",
                "raw_payload_hex": payload.hex(),
                "is_extended_id": False,
                "is_error_frame": False,
                "capture_source": "passive_usb_can_bus_receive",
                "direction_verified": False,
            }) + "\n")


def complete_frames(base_ns: int = 1_000_000_000, *, offset: int = 0):
    return [
        (base_ns, 0x159, struct.pack(">iHBB", 12_000, 500, 1, 0)),
        (base_ns + 1, 0x155, struct.pack(">ii", 100 + offset, 200 + offset)),
        (base_ns + 2, 0x156, struct.pack(">ii", 300 + offset, 400 + offset)),
        (base_ns + 3, 0x157, struct.pack(">ii", 500 + offset, 600 + offset)),
    ]


class PiperCommandTargetTest(unittest.TestCase):
    def test_decodes_raw_frames_and_holds_latest_absolute_target(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "commands.jsonl"
            frames = complete_frames() + complete_frames(1_100_000_000, offset=100)
            write_log(path, frames)
            timeline = decode_command_timeline(path)

        self.assertEqual(len(timeline), 5)
        held = target_at(timeline, 1_050_000_000)
        self.assertEqual(held.target_raw, (100, 200, 300, 400, 500, 600, 12_000))
        updated = target_at(timeline, 1_100_000_003)
        self.assertEqual(updated.target_raw, (200, 300, 400, 500, 600, 700, 12_000))
        self.assertAlmostEqual(updated.target[0], struct.unpack(">i", struct.pack(">i", 200))[0]
                               * 0.001 * 3.141592653589793 / 180)
        self.assertEqual(updated.joint_command_refs[0]["can_id"], "0x155")
        self.assertEqual(updated.gripper_command_ref["can_id"], "0x159")

    def test_rejects_incomplete_duplicate_slow_or_wrong_frames(self) -> None:
        cases = (
            (complete_frames()[:-1], "incomplete 0x155-0x157"),
            ([complete_frames()[0], complete_frames()[1],
              (complete_frames()[1][0] + 1, complete_frames()[1][1], complete_frames()[1][2])],
             "duplicate joint"),
            ([(t + (100_000_000 if can_id in (0x156, 0x157) else 0), can_id, payload)
              for t, can_id, payload in complete_frames()], "timestamp span"),
            ([(t, 0x200, payload) for t, _can_id_value, payload in complete_frames()], "unexpected CAN ID"),
            ([(t, can_id, bytes(7)) for t, can_id, _payload in complete_frames()], "exactly 8 bytes"),
        )
        for frames, message in cases:
            with self.subTest(message=message), tempfile.TemporaryDirectory() as temp:
                path = Path(temp) / "commands.jsonl"
                write_log(path, frames)
                with self.assertRaisesRegex(ValueError, message):
                    decode_command_timeline(path)

    def test_rejects_disabled_gripper_and_future_target(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "commands.jsonl"
            frames = complete_frames()
            frames[0] = (frames[0][0], 0x159, struct.pack(">iHBB", 12_000, 0, 0, 0))
            write_log(path, frames)
            with self.assertRaisesRegex(ValueError, "enabled position"):
                decode_command_timeline(path)

        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "commands.jsonl"
            write_log(path, complete_frames())
            timeline = decode_command_timeline(path)
            with self.assertRaisesRegex(ValueError, "no complete command"):
                target_at(timeline, 999_999_999)

    def test_requires_passive_receive_source_and_unverified_direction(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "commands.jsonl"
            write_log(path, complete_frames())
            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            rows[0]["direction_verified"] = True
            path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "passive CAN receive"):
                decode_command_timeline(path)


if __name__ == "__main__":
    unittest.main()
