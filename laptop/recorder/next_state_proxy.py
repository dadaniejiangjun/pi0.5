"""Build checksum-bound, offline-only next-observed-state proxy targets.

This never creates verified robot commands or modifies a raw episode.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from laptop.recorder.label_episode import verify_label
from laptop.recorder.verify_source import verify_source
from laptop.piper.control_profile import CONTINUOUS_GRIPPER_MAX_STEP_M
from laptop.robot_client.supervised_site_profile import FIRST_STEP_MAX_JOINT_DELTA_RAD

GRIPPER_METERS_PER_RAW_NOMINAL = 1e-6
SEMANTICS = "next_observed_state_proxy_nominal_units"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _state(row: dict) -> list[float]:
    joints = row.get("joint_rad_nominal_only")
    gripper = row.get("gripper_raw")
    if (not isinstance(joints, list) or len(joints) != 6
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in joints)
            or type(gripper) is not int):
        raise ValueError("missing finite six-joint and gripper feedback")
    # Preserve the signed feedback as observed. This nominal scale is still
    # unverified for hardware and is never a command conversion.
    return [float(value) for value in joints] + [gripper * GRIPPER_METERS_PER_RAW_NOMINAL]


def audit_transition_deltas(records: list[dict]) -> dict:
    """Summarize nominal observation-to-target jumps against current site caps.

    This is a data-quality report only. It does not establish physical velocity,
    hardware units, or a safe command mapping, and it never alters target values.
    """
    if not isinstance(records, list) or not records:
        raise ValueError("at least one proxy transition is required")
    joint_deltas: list[list[float]] = [[] for _ in range(6)]
    gripper_deltas: list[float] = []
    any_joint_over = 0
    gripper_over = 0
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise ValueError(f"proxy transition {index} must be an object")
        current, target = record.get("state"), record.get("proxy_action")
        if (not isinstance(current, list) or not isinstance(target, list)
                or len(current) != 7 or len(target) != 7
                or any(type(value) not in (int, float) or not math.isfinite(value)
                       for value in (*current, *target))):
            raise ValueError(f"proxy transition {index} must contain finite 7D state and target")
        deltas = [abs(float(target[joint]) - float(current[joint])) for joint in range(6)]
        gripper_delta = abs(float(target[6]) - float(current[6]))
        for joint, delta in enumerate(deltas):
            joint_deltas[joint].append(delta)
        gripper_deltas.append(gripper_delta)
        any_joint_over += any(delta > FIRST_STEP_MAX_JOINT_DELTA_RAD for delta in deltas)
        gripper_over += gripper_delta > CONTINUOUS_GRIPPER_MAX_STEP_M

    joint_exceed_by_axis = [
        sum(delta > FIRST_STEP_MAX_JOINT_DELTA_RAD for delta in axis)
        for axis in joint_deltas
    ]
    within = any_joint_over == 0 and gripper_over == 0
    return {
        "schema": "piper_proxy_transition_delta_audit_v1",
        "transitions": len(records),
        "comparison_limits": {
            "max_joint_delta_deg_per_proposal": math.degrees(FIRST_STEP_MAX_JOINT_DELTA_RAD),
            "max_gripper_delta_mm_per_proposal": CONTINUOUS_GRIPPER_MAX_STEP_M * 1000.0,
            "source": (
                "current continuous-task gripper step profile and supervised-site "
                "joint first-step cap"
            ),
        },
        "transitions_with_any_joint_delta_over_limit": any_joint_over,
        "joint_transitions_over_limit_by_axis": {
            f"J{index + 1}": count
            for index, count in enumerate(joint_exceed_by_axis)
        },
        "max_joint_delta_deg_by_axis": {
            f"J{index + 1}": math.degrees(max(axis))
            for index, axis in enumerate(joint_deltas)
        },
        "gripper_transitions_over_limit": gripper_over,
        "max_gripper_delta_mm": max(gripper_deltas) * 1000.0,
        "within_current_first_step_software_caps": within,
        "physical_velocity_verified": False,
        "hardware_units_verified": False,
        "targets_modified_or_clamped": False,
    }


def build(preview: Path, raw: Path, outcome: Path, output: Path) -> dict:
    preview, raw, outcome, output = (Path(value).resolve()
                                     for value in (preview, raw, outcome, output))
    if output.exists() or output.is_relative_to(raw) or output.is_relative_to(preview):
        raise ValueError("proxy output must be unused and separate from its sources")
    meta_path = preview / "aligned_episode.json"
    frames_path = preview / "frames.jsonl"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    label = verify_label(raw, outcome)
    if (meta.get("schema") != "piper_aligned_preview_v1"
            or meta.get("source") not in ("manual_teach", "operator_demonstration")
            or meta.get("raw_episode_id") != raw.name
            or not label["success"]):
        raise ValueError("proxy requires a labeled successful manual-teach preview")
    source = verify_source(raw, preview)
    if (source["source_frame_extraction_verified"] is not True
            or source["raw_manifest_sha256"] != meta.get("raw_manifest_sha256")):
        raise ValueError("preview source verification failed")
    rows = [json.loads(line) for line in frames_path.read_text(encoding="utf-8").splitlines()]
    if len(rows) < 2 or len(rows) != meta.get("frame_count"):
        raise ValueError("proxy needs at least two verified frames")
    period_ns = round(1_000_000_000 / meta["fps"])
    records = []
    for index, (current, following) in enumerate(zip(rows, rows[1:])):
        if (current.get("frame_index") != index
                or following.get("frame_index") != index + 1
                or following["policy_timestamp_ns"] - current["policy_timestamp_ns"] != period_ns):
            raise ValueError("proxy frames must be contiguous on the policy grid")
        records.append({
            "frame_index": index,
            "observation_timestamp_ns": current["policy_timestamp_ns"],
            "target_timestamp_ns": following["policy_timestamp_ns"],
            "base_image": current["base_image"],
            "wrist_image": current["wrist_image"],
            "state": _state(current),
            "proxy_action": _state(following),
            "target_source_frame_index": index + 1,
        })
    output.mkdir(parents=True)
    targets = output / "proxy_targets.jsonl"
    with targets.open("x", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, sort_keys=True) + "\n")
    manifest = {
        "schema": "piper_next_state_proxy_v1",
        "semantics": SEMANTICS,
        "raw_episode_id": raw.name,
        "preview_episode_id": preview.name,
        "raw_manifest_sha256": source["raw_manifest_sha256"],
        "preview_metadata_sha256": _sha256(meta_path),
        "preview_frames_sha256": _sha256(frames_path),
        "outcome_label_sha256": _sha256(outcome),
        "outcome_success": label["success"],
        "proxy_targets_sha256": _sha256(targets),
        "samples": len(records),
        "fps": meta["fps"],
        "target_offset_ns": period_ns,
        "joint_unit": "radian_nominal_unverified",
        "gripper_unit": "meter_nominal_unverified",
        "gripper_meters_per_raw_nominal": GRIPPER_METERS_PER_RAW_NOMINAL,
        "transition_delta_audit": audit_transition_deltas(records),
        "hardware_control_allowed": False,
        "formal_training_eligible": False,
        "note": "Offline imitation proxy only; next observed feedback is not a verified command.",
    }
    (output / "proxy_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return {"status": "PROXY_ONLY", "output": str(output), **manifest}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--outcome", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.preview, args.raw, args.outcome, args.output), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
