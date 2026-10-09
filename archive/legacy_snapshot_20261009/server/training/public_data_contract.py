"""Explicitly distinguish public PiPER smoke data from local robot actions."""

from __future__ import annotations

PUBLIC_ACTION_SEMANTICS = "absolute_joint_degree_gripper_binary_open"
LOCAL_ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_meter"
PUBLIC_ACTION_UNITS = ("degree",) * 6 + ("binary_open_fraction",)
LOCAL_ACTION_UNITS = ("radian",) * 6 + ("meter",)


def inspect_public_acceptance(acceptance: dict, provenance: dict) -> dict[str, bool]:
    """Check identity and reviewed units; never infer compatibility from 7D shape."""
    return {
        "public_acceptance_schema": acceptance.get("schema") == "piper_public_data_acceptance_v1",
        "public_action_semantics": acceptance.get("action_semantics") == PUBLIC_ACTION_SEMANTICS,
        "public_units_reviewed": acceptance.get("units_reviewed") is True
        and acceptance.get("joint_unit") == "degree"
        and acceptance.get("gripper_unit") == "binary_open_fraction",
        "public_source_identity": bool(acceptance.get("source_revision"))
        and bool(acceptance.get("archive_sha256"))
        and provenance.get("source_revision") == acceptance.get("source_revision")
        and provenance.get("archive_sha256") == acceptance.get("archive_sha256"),
        "not_local_robot_contract": acceptance.get("hardware_control_allowed") is False
        and acceptance.get("action_semantics") != LOCAL_ACTION_SEMANTICS,
    }
