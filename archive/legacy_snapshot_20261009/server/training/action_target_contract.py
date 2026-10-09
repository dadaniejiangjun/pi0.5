"""Semantic gates separating observed-state targets from recorded commands."""

from __future__ import annotations

from typing import Any


RAW_COMMAND_ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_meter"
RAW_COMMAND_TARGET_SOURCE = "observed_piper_can_command_frames"
_VERIFIED_TARGET_FIELDS = (
    "action_target_verified",
    "command_target_provenance_verified",
    "camera_sync_verified",
    "gripper_verified",
    "units_verified",
)


def is_verified_raw_command_episode(row: Any) -> bool:
    """Accept only successful manual episodes with verified command-frame targets."""
    return (
        isinstance(row, dict)
        and row.get("source") == "manual_teach"
        and row.get("success") is True
        and row.get("action_target_source") == RAW_COMMAND_TARGET_SOURCE
        and all(row.get(field) is True for field in _VERIFIED_TARGET_FIELDS)
    )


def has_raw_command_action_semantics(acceptance: Any) -> bool:
    """Do not let a derived future state masquerade as a measured command."""
    return (
        isinstance(acceptance, dict)
        and acceptance.get("action_semantics") == RAW_COMMAND_ACTION_SEMANTICS
    )
