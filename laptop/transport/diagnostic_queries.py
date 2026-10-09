"""Strict allowlist for non-motion PiPER diagnostic CAN queries.

Only protocol-defined getter requests are represented here. Joint targets,
mode changes, enable/disable, gripper control, resets, and parameter setters
cannot be constructed as a ``DiagnosticCanQuery``.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib


@dataclass(frozen=True)
class DiagnosticCanQuery:
    request_can_id: int
    payload: bytes

    def __post_init__(self) -> None:
        if type(self.request_can_id) is not int or not 0 <= self.request_can_id <= 0x7FF:
            raise ValueError("diagnostic query must use a standard 11-bit CAN ID")
        if not isinstance(self.payload, bytes) or len(self.payload) != 8:
            raise ValueError("diagnostic query payload must contain exactly 8 bytes")
        if not self._is_allowlisted(self.request_can_id, self.payload):
            raise ValueError("CAN frame is not an allowlisted non-motion diagnostic query")

    @staticmethod
    def _is_allowlisted(can_id: int, payload: bytes) -> bool:
        if can_id == 0x4AF:
            return payload == bytes((0x01, 0, 0, 0, 0, 0, 0, 0))
        if can_id == 0x472:
            return (
                1 <= payload[0] <= 6
                and payload[1] in (0x01, 0x02)
                and payload[2:] == bytes(6)
            )
        if can_id == 0x477:
            # Byte 0 = query gripper/teaching-pendant parameters; all fields
            # that could request a configuration change must remain zero.
            return payload == bytes((0x04, 0, 0, 0, 0, 0, 0, 0))
        return False

    @property
    def expected_response_can_id(self) -> int:
        if self.request_can_id == 0x4AF:
            return 0x4AF
        if self.request_can_id == 0x472:
            return 0x473 if self.payload[1] == 0x01 else 0x47C
        return 0x47E

    @property
    def name(self) -> str:
        if self.request_can_id == 0x4AF:
            return "firmware_version"
        if self.request_can_id == 0x472:
            return "joint_angle_speed_limits" if self.payload[1] == 0x01 else "joint_acceleration_limit"
        return "gripper_teaching_parameters"

    @property
    def payload_sha256(self) -> str:
        return hashlib.sha256(self.payload).hexdigest()


def firmware_version_query() -> DiagnosticCanQuery:
    return DiagnosticCanQuery(0x4AF, bytes((0x01, 0, 0, 0, 0, 0, 0, 0)))


def joint_limit_query(motor_num: int, *, acceleration: bool = False) -> DiagnosticCanQuery:
    if type(motor_num) is not int or not 1 <= motor_num <= 6:
        raise ValueError("motor_num must be an integer in [1, 6]")
    if type(acceleration) is not bool:
        raise ValueError("acceleration must be a boolean")
    search_content = 0x02 if acceleration else 0x01
    return DiagnosticCanQuery(
        0x472,
        bytes((motor_num, search_content, 0, 0, 0, 0, 0, 0)),
    )


def gripper_parameters_query() -> DiagnosticCanQuery:
    return DiagnosticCanQuery(0x477, bytes((0x04, 0, 0, 0, 0, 0, 0, 0)))
