"""Offline-only preview of the PiPER V2 0x159 gripper target frame.

The official command includes a mandatory enable/disable field. This pure
encoder never sends CAN and cannot authorize a real gripper command.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class GripperPreview:
    arbitration_id: int
    data: bytes
    target_mm: float
    includes_gripper_enable: bool
    hardware_control_allowed: bool = False


def preview_opening(target_raw_0p001mm: int, *, effort_raw: int = 0) -> GripperPreview:
    if (type(target_raw_0p001mm) is not int or not 0 <= target_raw_0p001mm <= 70_000
            or type(effort_raw) is not int or not 0 <= effort_raw <= 5000):
        raise ValueError("bounded integer gripper stroke and effort required")
    # Byte 6 = 0x01 is the SDK's gripper-enable code, not an arm enable.
    payload = struct.pack(">iHBB", target_raw_0p001mm, effort_raw, 0x01, 0x00)
    return GripperPreview(0x159, payload, target_raw_0p001mm / 1000, True)
