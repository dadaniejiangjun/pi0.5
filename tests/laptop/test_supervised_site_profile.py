from __future__ import annotations

import json

import pytest

from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.supervised_site_profile import (
    READY_STATUS,
    SITE_PROFILE_SCHEMA,
    load_supervised_site_profile,
)
from laptop.transport.position_commands import P12_SITE_GATE_NAMES


def _profile() -> dict:
    return {
        "schema": SITE_PROFILE_SCHEMA,
        "status": READY_STATUS,
        "site_id": "lab-bench-1",
        "robot_serial": "piper-test-serial",
        "firmware_version": "v2-test",
        "commissioned_by": "operator",
        "commissioned_at_utc": "2026-10-07T10:00:00+08:00",
        "gripper_effort_raw": 120,
        "limits": {
            "joint_min_rad": [-1.0, 0.1, -1.0, -1.0, -1.0, -1.0],
            "joint_max_rad": [1.0, 2.0, -0.1, 1.0, 1.0, 1.0],
            "max_joint_delta_rad": [0.01] * 6,
            "gripper_min_m": 0.0,
            "gripper_max_m": 0.05,
            "max_gripper_delta_m": 0.001,
            "max_state_age_ns": 250_000_000,
            "max_prediction_response_latency_ns": 250_000_000,
            "max_proposal_age_ns": 250_000_000,
            "max_proposal_validity_ns": 250_000_000,
            "max_heartbeat_age_ns": 250_000_000,
        },
        "p12_gate_evidence": {name: True for name in P12_SITE_GATE_NAMES},
        "evidence_refs": {name: f"validation/{name}.json" for name in P12_SITE_GATE_NAMES},
    }


def _write(tmp_path, profile: dict):
    path = tmp_path / "site-profile.json"
    path.write_text(json.dumps(profile), encoding="utf-8")
    return path


def test_loads_complete_site_profile_without_treating_refs_as_verified(tmp_path):
    loaded = load_supervised_site_profile(_write(tmp_path, _profile()))

    assert loaded.site_id == "lab-bench-1"
    assert loaded.gripper_effort_raw == 120
    assert loaded.limits.max_joint_delta_rad == (0.01,) * 6
    assert loaded.p12_evidence == P12GateEvidence(**{name: True for name in P12_SITE_GATE_NAMES})
    assert len(loaded.evidence_refs) == len(P12_SITE_GATE_NAMES)


def test_refuses_draft_and_incomplete_site_gates(tmp_path):
    draft = _profile()
    draft["status"] = "DRAFT"
    with pytest.raises(PermissionError, match="not declared ready"):
        load_supervised_site_profile(_write(tmp_path, draft))

    incomplete = _profile()
    incomplete["p12_gate_evidence"]["mechanical_limits_verified"] = False
    with pytest.raises(PermissionError, match="mechanical_limits_verified"):
        load_supervised_site_profile(_write(tmp_path, incomplete))


def test_requires_one_evidence_reference_per_gate(tmp_path):
    profile = _profile()
    del profile["evidence_refs"][P12_SITE_GATE_NAMES[0]]
    with pytest.raises(ValueError, match="evidence_refs"):
        load_supervised_site_profile(_write(tmp_path, profile))


def test_refuses_sdk_nominal_joint_range_as_site_envelope(tmp_path):
    profile = _profile()
    profile["limits"]["joint_min_rad"][0] = -3.0
    with pytest.raises(ValueError, match="official SDK nominal range"):
        load_supervised_site_profile(_write(tmp_path, profile))


def test_refuses_timezone_free_timestamp_and_unverified_limits_file(tmp_path):
    profile = _profile()
    profile["commissioned_at_utc"] = "2026-10-07T10:00:00"
    with pytest.raises(ValueError, match="include a timezone"):
        load_supervised_site_profile(_write(tmp_path, profile))

    legacy = tmp_path / "sdk-nominal-limits.json"
    legacy.write_text(json.dumps({
        "schema": "piper_joint_limits_v2",
        "status": "FULL_LIVE_CAN_READBACK_WITH_UNIT_CAVEATS",
        "limits_role": "official_sdk_nominal_reference_not_a_verified_mechanical_envelope",
    }), encoding="utf-8")
    with pytest.raises(ValueError, match="top-level fields"):
        load_supervised_site_profile(legacy)


def test_rejects_ambiguous_json_and_gripper_limit_above_sdk_cap(tmp_path):
    duplicate = tmp_path / "duplicate.json"
    duplicate.write_text('{"schema":"a","schema":"b"}', encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate JSON key"):
        load_supervised_site_profile(duplicate)

    non_finite = _profile()
    raw = json.dumps(non_finite).replace('"gripper_min_m": 0.0',
                                         '"gripper_min_m": NaN')
    path = tmp_path / "non-finite.json"
    path.write_text(raw, encoding="utf-8")
    with pytest.raises(ValueError, match="non-finite JSON number"):
        load_supervised_site_profile(path)

    profile = _profile()
    profile["limits"]["gripper_max_m"] = 0.101
    with pytest.raises(ValueError, match="100 mm SDK command cap"):
        load_supervised_site_profile(_write(tmp_path, profile))

    profile = _profile()
    profile["limits"]["max_joint_delta_rad"] = [0.02] * 6
    with pytest.raises(ValueError, match="1 degree per joint"):
        load_supervised_site_profile(_write(tmp_path, profile))

    profile = _profile()
    profile["gripper_effort_raw"] = True
    with pytest.raises(ValueError, match="explicit integer"):
        load_supervised_site_profile(_write(tmp_path, profile))
