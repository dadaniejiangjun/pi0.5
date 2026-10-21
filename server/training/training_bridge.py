"""Prepare and audit an explicitly selected historical next-state window.

The existing 20 ms camera-pair rule rejects individual policy-grid rows. This
bridge records every rejected row and selects one longest contiguous valid
window, preserving source timestamps and frame references. It then reuses the
existing LeRobot converter, video readback, and strict dataset contract. It
never trains, loads a model, connects to ROS/CAN/cameras, or changes labels.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any, Sequence

from laptop.recorder import align_raw
from laptop.recorder.next_state_proxy import SEMANTICS, _state, audit_transition_deltas
from server.training import proxy_multi_episode_contract as proxy_contract


ROOT = Path(__file__).resolve().parents[2]
BRIDGE_ROOT = ROOT / "cache" / "training_bridge"
DEFAULT_RAW_ROOT = ROOT / "datasets" / "raw"
DEFAULT_OUTCOME_ROOT = ROOT / "datasets" / "manifests" / "outcomes"
DEFAULT_RUN_ID_PREFIX = "proxybridge_"
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{2,47}$")
STRICT_CONTRACT_DEFAULT_MIN_EPISODES = inspect.signature(
    proxy_contract.inspect_proxy_multi_episode
).parameters["min_episodes"].default
CAMERA_PAIR_TOLERANCE_MS = 20.0
WINDOW_POLICY = "longest_contiguous_valid_policy_grid_window"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _utc_run_id() -> str:
    return DEFAULT_RUN_ID_PREFIX + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _inside(path: str | Path, root: Path) -> bool:
    try:
        Path(path).expanduser().resolve().is_relative_to(root.resolve())
    except (OSError, RuntimeError, ValueError):
        return False
    return True


def _require_offline_environment() -> dict[str, str]:
    """Require the launcher to isolate HF and temp directories before Python."""
    required = ("HF_HOME", "HF_DATASETS_CACHE", "HF_HUB_CACHE", "TEMP", "TMP",
                "HF_LEROBOT_HOME")
    values = {name: os.environ.get(name, "") for name in required}
    missing = [name for name, value in values.items() if not value]
    if missing:
        raise ValueError(
            "start through scripts/prepare_proxy_candidate.ps1; missing isolated "
            f"environment values: {', '.join(missing)}"
        )
    if (os.environ.get("HF_HUB_OFFLINE") != "1"
            or os.environ.get("HF_DATASETS_OFFLINE") != "1"):
        raise ValueError("HF_HUB_OFFLINE and HF_DATASETS_OFFLINE must be set to 1")
    for name in required:
        if not _inside(values[name], BRIDGE_ROOT):
            raise ValueError(f"{name} must resolve inside {BRIDGE_ROOT}")
    return values


def _source_evidence(raw_root: Path, outcome_root: Path,
                     episode_ids: Sequence[str], pipeline) -> list[dict[str, Any]]:
    evidence = []
    for episode_id in episode_ids:
        raw_episode = raw_root / episode_id
        manifest_path = raw_episode / "manifest.json"
        label_path = outcome_root / f"{episode_id}.json"
        if not manifest_path.is_file() or not label_path.is_file():
            raise FileNotFoundError(
                f"episode {episode_id} needs its original raw manifest and outcome label"
            )
        verified = pipeline.verify_label(raw_episode, label_path)
        label = json.loads(label_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if verified.get("success") is not True or label.get("success") is not True:
            raise ValueError(f"episode {episode_id} lacks a hash-bound successful outcome label")
        if label.get("training_eligible") is not False:
            raise ValueError(
                f"episode {episode_id} must retain its recorded training_eligible=false label"
            )
        evidence.append({
            "episode_id": episode_id,
            "source_kind": manifest.get("source"),
            "raw_manifest_sha256": _sha256(manifest_path),
            "outcome_label_sha256": _sha256(label_path),
            "outcome": "success" if label.get("success") is True else "unknown",
            "operator": label.get("operator"),
            "labeled_utc": label.get("labeled_utc"),
            "action_target_verified": label.get("action_target_verified"),
            "training_eligible_as_recorded": label.get("training_eligible"),
            "notes_as_recorded": label.get("notes"),
            "verified_success": True,
            "source_training_admission_granted": False,
        })
    return evidence


def _runs(indices: list[int]) -> list[list[int]]:
    result: list[list[int]] = []
    for index in indices:
        if not result or index != result[-1][-1] + 1:
            result.append([index])
        else:
            result[-1].append(index)
    return result


def _filtered_plan(raw: Path, *, max_skew_ms: float) -> dict[str, Any]:
    """Use the current aligner's tolerances, retaining row-level rejects."""
    if (isinstance(max_skew_ms, bool) or not isinstance(max_skew_ms, (int, float))
            or not math.isfinite(max_skew_ms) or max_skew_ms <= 0 or max_skew_ms > 50):
        raise ValueError("max_skew_ms must be finite, positive, and no greater than 50")
    metadata, manifest, channels, raw_sha, command_sha = align_raw._raw_episode(raw)
    if metadata.get("source") not in ("manual_teach", "operator_demonstration"):
        raise ValueError("filtered proxy windows require a historical manual/operator episode")
    tolerance_ns = round(float(max_skew_ms) * 1_000_000)
    camera_tolerance_ns = round(CAMERA_PAIR_TOLERANCE_MS * 1_000_000)
    state_start = max(channels[name][0]["monotonic_ns"] for name in align_raw.STATE_CHANNELS)
    state_end = min(channels[name][-1]["monotonic_ns"] for name in align_raw.STATE_CHANNELS)
    selection_channels = dict(channels)
    if metadata.get("source") == "operator_demonstration":
        for role in ("base", "wrist"):
            selection_channels[role] = [
                row for row in channels[role]
                if state_start <= row["monotonic_ns"] <= state_end
            ]
            if len(selection_channels[role]) < 2:
                raise ValueError(f"fewer than two {role} frames in complete feedback window")
    start = max(rows[0]["monotonic_ns"] for rows in selection_channels.values())
    end = min(rows[-1]["monotonic_ns"] for rows in selection_channels.values())
    period_ns = 100_000_000
    first = ((start + period_ns - 1) // period_ns) * period_ns
    if first + period_ns > end:
        raise ValueError("raw channels have fewer than two complete aligned samples")
    stamps = {name: [row["monotonic_ns"] for row in rows]
              for name, rows in selection_channels.items()}
    accepted: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    grid_count = 0
    for grid_index, policy_ts in enumerate(range(first, end + 1, period_ns)):
        grid_count += 1
        row: dict[str, Any] = {
            "source_policy_grid_index": grid_index,
            "policy_timestamp_ns": policy_ts,
        }
        try:
            selected = {
                name: align_raw._nearest(rows, stamps[name], policy_ts,
                                         tolerance_ns, name)
                for name, rows in selection_channels.items()
            }
            state_ts = selected["joint_12"]["monotonic_ns"]
            state_skew_ns = max(abs(item["monotonic_ns"] - state_ts)
                                for item in selected.values())
            pair_skew_ns = abs(
                selected["base"]["monotonic_ns"] - selected["wrist"]["monotonic_ns"]
            )
            row.update({
                "base_frame_id": selected["base"]["frame_id"],
                "wrist_frame_id": selected["wrist"]["frame_id"],
                "base_timestamp_ns": selected["base"]["monotonic_ns"],
                "wrist_timestamp_ns": selected["wrist"]["monotonic_ns"],
                "base_wrist_host_timestamp_skew_ms": pair_skew_ns / 1e6,
                "state_max_skew_ms": state_skew_ns / 1e6,
            })
            reasons = []
            if state_skew_ns > tolerance_ns:
                reasons.append("selected_camera_or_state_sample_exceeds_existing_50ms_tolerance")
            if pair_skew_ns > camera_tolerance_ns:
                reasons.append("base_wrist_host_timestamp_pair_exceeds_existing_20ms_tolerance")
            if reasons:
                rejected.append({**row, "reasons": reasons})
                continue
            joint_raw: list[int] = []
            for name in ("joint_12", "joint_34", "joint_56"):
                pair = selected[name].get("raw_joint_pair")
                if not isinstance(pair, list) or len(pair) != 2 or any(type(v) is not int for v in pair):
                    raise ValueError(f"invalid raw joint pair: {name}")
                joint_raw.extend(pair)
            gripper_raw = selected["gripper"].get("angle_or_stroke")
            if type(gripper_raw) is not int:
                raise ValueError("invalid raw gripper value")
            source_refs = {
                f"{name}_frame_id": selected[name]["frame_id"]
                for name in ("base", "wrist")
            }
            source_refs.update({
                f"{name}_timestamp_ns": selected[name]["monotonic_ns"]
                for name in align_raw.STATE_CHANNELS
            })
            accepted.append({
                "frame_index": grid_index,
                "source_policy_grid_index": grid_index,
                "policy_timestamp_ns": policy_ts,
                "base_timestamp_ns": selected["base"]["monotonic_ns"],
                "wrist_timestamp_ns": selected["wrist"]["monotonic_ns"],
                "base_wrist_host_timestamp_skew_ms": pair_skew_ns / 1e6,
                "state_timestamp_ns": state_ts,
                "source_refs": source_refs,
                "joint_raw": joint_raw,
                "gripper_raw": gripper_raw,
                "status": {key: selected["status"].get(key)
                           for key in ("ctrl_mode", "arm_status", "teach_status", "err_code")},
            })
        except (ValueError, KeyError, TypeError) as exc:
            if row not in rejected:
                rejected.append({**row, "reasons": [str(exc)]})
    if len(accepted) < 3:
        raise ValueError(
            f"fewer than three valid 10 Hz rows remain after the existing 20 ms rule; "
            f"valid={len(accepted)}, rejected={len(rejected)}"
        )
    runs = _runs([row["source_policy_grid_index"] for row in accepted])
    chosen = max(runs, key=lambda item: (len(item), -item[0]))
    chosen_set = set(chosen)
    selected_rows = [row for row in accepted
                     if row["source_policy_grid_index"] in chosen_set]
    if len(selected_rows) < 3:
        raise ValueError("longest valid contiguous window has fewer than three rows")
    for index, row in enumerate(selected_rows):
        row["frame_index"] = index
    selected_bytes = "".join(
        json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
        for row in selected_rows
    ).encode("utf-8")
    return {
        "metadata": metadata, "manifest": manifest, "channels": channels,
        "raw_path": Path(raw).resolve(), "raw_manifest_sha256": raw_sha,
        "command_log_sha256": command_sha, "frames": selected_rows,
        "frame_map_sha256": hashlib.sha256(selected_bytes).hexdigest(),
        "fps": 10, "max_skew_ms": float(max_skew_ms),
        "max_camera_pair_skew_ms": CAMERA_PAIR_TOLERANCE_MS,
        "policy_grid_rows": grid_count, "valid_policy_grid_rows": len(accepted),
        "rejected_rows": sorted(rejected, key=lambda item: item["source_policy_grid_index"]),
        "selected_grid_window": [chosen[0], chosen[-1]],
        "valid_rows_excluded_outside_window": len(accepted) - len(selected_rows),
    }


def _write_filtered_preview(plan: dict[str, Any], output: Path) -> dict[str, Any]:
    """Write a source-bound preview for one original contiguous policy window."""
    if output.exists():
        raise FileExistsError(f"refusing to overwrite preview: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_name(output.name + ".INCOMPLETE")
    if pending.exists():
        raise FileExistsError(f"refusing to overwrite incomplete preview: {pending}")
    pending.mkdir()
    (pending / "alignment_rejections.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
                for row in plan["rejected_rows"]),
        encoding="utf-8",
    )
    for role in ("base", "wrist"):
        refs = [row["source_refs"][f"{role}_frame_id"] for row in plan["frames"]]
        align_raw._extract_video(
            plan["raw_path"] / f"{role}.mp4", pending, role, refs,
            len(plan["channels"][role]),
        )
    with (pending / "frames.jsonl").open("x", encoding="utf-8") as handle:
        for index, source in enumerate(plan["frames"]):
            row = dict(source)
            row["base_image"] = f"base_{index:06d}.png"
            row["wrist_image"] = f"wrist_{index:06d}.png"
            row["joint_rad_nominal_only"] = [
                align_raw.joint_raw_to_nominal_rad(value) for value in row.pop("joint_raw")
            ]
            handle.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    reject_path = pending / "alignment_rejections.jsonl"
    metadata = {
        "schema": "piper_aligned_preview_v1",
        "episode_id": output.name,
        "raw_episode_id": plan["raw_path"].name,
        "raw_manifest_sha256": plan["raw_manifest_sha256"],
        "source": plan["metadata"]["source"],
        "task_id": align_raw.TASK_ID,
        "task": align_raw.TASK,
        "fps": 10,
        "frame_count": len(plan["frames"]),
        "frame_map_sha256": plan["frame_map_sha256"],
        "timestamp_kind": "host_perf_counter_ns_after_read_not_camera_exposure",
        "max_skew_ms": plan["max_skew_ms"],
        "training_eligible": False,
        "max_camera_pair_skew_ms": CAMERA_PAIR_TOLERANCE_MS,
        "camera_pairing_semantics": "host_after_read_timestamp_pairing_not_exposure_sync",
        "success": None,
        **{flag: False for flag in align_raw.QC_FLAGS},
        "bridge_window_selection": {
            "schema": "piper_filtered_valid_policy_window_v1",
            "policy": WINDOW_POLICY,
            "window_start_grid_index": plan["selected_grid_window"][0],
            "window_end_grid_index": plan["selected_grid_window"][1],
            "window_start_timestamp_ns": plan["frames"][0]["policy_timestamp_ns"],
            "window_end_timestamp_ns": plan["frames"][-1]["policy_timestamp_ns"],
            "original_grid_rows": plan["policy_grid_rows"],
            "valid_grid_rows": plan["valid_policy_grid_rows"],
            "rejected_grid_rows": len(plan["rejected_rows"]),
            "rejected_rows_path": "alignment_rejections.jsonl",
            "rejected_rows_sha256": _sha256(reject_path),
            "valid_rows_excluded_outside_window": plan["valid_rows_excluded_outside_window"],
            "full_task_success_inferred_from_fragment": False,
        },
    }
    (pending / "aligned_episode.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    pending.rename(output)
    return metadata


def _build_filtered_proxy(preview: Path, raw: Path, outcome: Path,
                          output: Path) -> dict[str, Any]:
    """Derive next-observed-state rows only within the selected source window."""
    if output.exists():
        raise FileExistsError(f"refusing to overwrite proxy targets: {output}")
    meta = json.loads((preview / "aligned_episode.json").read_text(encoding="utf-8"))
    if (meta.get("raw_episode_id") != raw.name
            or meta.get("raw_manifest_sha256") != _sha256(raw / "manifest.json")
            or meta.get("bridge_window_selection", {}).get("policy") != WINDOW_POLICY):
        raise ValueError("filtered preview source identity or explicit window policy is invalid")
    rows = [json.loads(line) for line in (preview / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
    period_ns = 100_000_000
    if len(rows) < 3:
        raise ValueError("filtered proxy needs at least three source feedback rows")
    records = []
    for index, (current, following) in enumerate(zip(rows, rows[1:])):
        if (current.get("frame_index") != index
                or following.get("frame_index") != index + 1
                or following.get("source_policy_grid_index") != current.get("source_policy_grid_index") + 1
                or following["policy_timestamp_ns"] - current["policy_timestamp_ns"] != period_ns):
            raise ValueError("filtered preview does not preserve a contiguous original 10 Hz grid")
        records.append({
            "frame_index": index,
            "observation_timestamp_ns": current["policy_timestamp_ns"],
            "target_timestamp_ns": following["policy_timestamp_ns"],
            "base_image": current["base_image"],
            "wrist_image": current["wrist_image"],
            "state": _state(current),
            "proxy_action": _state(following),
            "target_source_frame_index": index + 1,
            "source_policy_grid_indices": [current["source_policy_grid_index"],
                                            following["source_policy_grid_index"]],
        })
    output.mkdir(parents=True)
    targets_path = output / "proxy_targets.jsonl"
    with targets_path.open("x", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
    label = json.loads(outcome.read_text(encoding="utf-8"))
    reject_path = preview / "alignment_rejections.jsonl"
    manifest = {
        "schema": "piper_next_state_proxy_v1",
        "semantics": SEMANTICS,
        "raw_episode_id": raw.name,
        "preview_episode_id": preview.name,
        "raw_manifest_sha256": meta["raw_manifest_sha256"],
        "preview_metadata_sha256": _sha256(preview / "aligned_episode.json"),
        "preview_frames_sha256": _sha256(preview / "frames.jsonl"),
        "outcome_label_sha256": _sha256(outcome),
        "outcome_success": label.get("success") is True,
        "proxy_targets_sha256": _sha256(targets_path),
        "samples": len(records),
        "fps": 10,
        "target_offset_ns": period_ns,
        "joint_unit": "radian_nominal_unverified",
        "gripper_unit": "meter_nominal_unverified",
        "gripper_meters_per_raw_nominal": 1e-6,
        "transition_delta_audit": audit_transition_deltas(records),
        "window_selection_policy": WINDOW_POLICY,
        "source_grid_window": meta["bridge_window_selection"],
        "alignment_rejections_sha256": _sha256(reject_path),
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
        "note": (
            "Offline proxy over a contiguous fragment of a hash-bound historical episode; "
            "the fragment does not independently establish complete task success."
        ),
    }
    (output / "proxy_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def prepare_candidate(*, episode_ids: Sequence[str], run_id: str | None = None,
                      raw_root: Path = DEFAULT_RAW_ROOT,
                      outcome_root: Path = DEFAULT_OUTCOME_ROOT,
                      max_skew_ms: float = 50.0,
                      window_policy: str,
                      report_path: Path | None = None) -> dict[str, Any]:
    """Convert selected labeled feedback into a review candidate and audit it."""
    environment = _require_offline_environment()
    if window_policy != WINDOW_POLICY:
        raise ValueError(f"window_policy must be explicitly set to {WINDOW_POLICY!r}")
    if not isinstance(episode_ids, (list, tuple)) or not episode_ids:
        raise ValueError("at least one explicit --episode-id is required")
    if len(set(episode_ids)) != len(episode_ids):
        raise ValueError("episode IDs must not contain duplicates")
    if run_id is None:
        run_id = _utc_run_id()
    if RUN_ID_RE.fullmatch(run_id) is None:
        raise ValueError("run_id must be a safe 3-48 character directory name")

    raw_root = Path(raw_root).resolve(strict=True)
    outcome_root = Path(outcome_root).resolve(strict=True)
    derived_root = BRIDGE_ROOT / "derived"
    dataset_home = BRIDGE_ROOT / "datasets"
    if report_path is None:
        report_path = BRIDGE_ROOT / "reports" / f"{run_id}.json"
    report_path = Path(report_path).expanduser().resolve()
    if not _inside(report_path, BRIDGE_ROOT):
        raise ValueError(f"report_path must resolve inside {BRIDGE_ROOT}")
    if report_path.exists():
        raise FileExistsError(f"refusing to overwrite bridge report: {report_path}")

    # Imports that load the converter/runtime occur only after the launcher has
    # set its isolated cache and temporary directories.
    from server.training import post_capture_pipeline as pipeline

    source_evidence = _source_evidence(
        raw_root, outcome_root, episode_ids, pipeline,
    )
    # Reuse the existing selector for checksum-bound source-label verification,
    # while keeping its ordinary all-rows alignment path unchanged.
    selection = pipeline.select_successful_episodes(
        raw_root, outcome_root, min_episodes=len(episode_ids),
        episode_ids=episode_ids,
    )
    if [row["episode_id"] for row in selection["successful"]] != sorted(episode_ids):
        raise ValueError("every explicitly selected episode must retain a successful source label")
    repo_id = f"local/{run_id}"
    run_root = derived_root / run_id
    dataset_path = dataset_home / repo_id.split("/")[0] / repo_id.split("/")[1]
    if run_root.exists():
        raise FileExistsError(f"refusing to overwrite derived run: {run_root}")
    if dataset_path.exists():
        raise FileExistsError(f"refusing to overwrite LeRobot dataset: {dataset_path}")
    run_root.mkdir(parents=True)
    pairs = []
    window_reports = []
    for item in selection["successful"]:
        episode_id = item["episode_id"]
        raw = item["raw"]
        preview = run_root / "previews" / episode_id
        proxy = run_root / "targets" / episode_id
        plan = _filtered_plan(raw, max_skew_ms=max_skew_ms)
        preview_meta = _write_filtered_preview(plan, preview)
        proxy_manifest = _build_filtered_proxy(preview, raw, item["outcome"], proxy)
        pairs.append((proxy, preview))
        window_reports.append({
            "episode_id": episode_id,
            "policy": window_policy,
            "source_policy_grid_window": plan["selected_grid_window"],
            "source_timestamp_window_ns": [
                plan["frames"][0]["policy_timestamp_ns"],
                plan["frames"][-1]["policy_timestamp_ns"],
            ],
            "original_policy_grid_rows": plan["policy_grid_rows"],
            "valid_policy_grid_rows": plan["valid_policy_grid_rows"],
            "rejected_policy_grid_rows": len(plan["rejected_rows"]),
            "rejection_rate": (len(plan["rejected_rows"]) / plan["policy_grid_rows"]),
            "rejected_rows_path": str(preview / "alignment_rejections.jsonl"),
            "rejected_rows_sha256": preview_meta["bridge_window_selection"]["rejected_rows_sha256"],
            "valid_rows_excluded_outside_window": plan["valid_rows_excluded_outside_window"],
            "selected_source_feedback_rows": len(plan["frames"]),
            "derived_proxy_transitions": proxy_manifest["samples"],
            "raw_timestamps_preserved": True,
            "source_raw_manifest_sha256": plan["raw_manifest_sha256"],
            "source_outcome_label_sha256": _sha256(item["outcome"]),
            "fragment_is_complete_task_demonstration": False,
        })
    prepared_conversion = pipeline.convert_many(pairs, dataset_path, repo_id)
    prepared = {
        "dataset_path": str(dataset_path),
        "dataset_sha256": proxy_contract.dataset_tree_sha256(dataset_path),
        "conversion": prepared_conversion,
    }
    if not _inside(dataset_path, BRIDGE_ROOT):
        raise ValueError("converted dataset escaped the local cache root")

    # Re-run the strict contract explicitly after conversion/readback. The
    # one-episode minimum is a software audit override; default training gates
    # retain their independent episode minimum and field-evidence requirements.
    strict_audit = proxy_contract.inspect_proxy_multi_episode(
        dataset_path, min_episodes=len(episode_ids),
        expected_dataset_sha256=prepared["dataset_sha256"],
    )
    if strict_audit.get("formal_training_eligible") is not False:
        raise ValueError("strict contract returned an unexpected training eligibility state")
    if strict_audit.get("hardware_control_allowed") is not False:
        raise ValueError("strict contract returned an unexpected hardware authorization state")

    readback_audit_path = dataset_path / "provenance" / "readback_audit.json"
    readback_rows_path = dataset_path / "provenance" / "readback_frames.jsonl"
    report: dict[str, Any] = {
        "schema": "piper_historical_next_state_proxy_bridge_v1",
        "status": "SOFTWARE_CONTRACT_PASS_FIELD_ADMISSION_PENDING",
        "run_id": run_id,
        "candidate_kind": "derived_next_observed_feedback_state",
        "window_policy": window_policy,
        "episode_ids": list(episode_ids),
        "window_selection": window_reports,
        "source_evidence": source_evidence,
        "dataset_path": str(dataset_path),
        "dataset_sha256": strict_audit["dataset_sha256"],
        "dataset_home": str(dataset_home.resolve()),
        "derived_root": str((derived_root / run_id).resolve()),
        "conversion": prepared["conversion"],
        "strict_contract": strict_audit,
        "strict_contract_min_episodes_applied": len(episode_ids),
        "strict_contract_default_min_episodes": STRICT_CONTRACT_DEFAULT_MIN_EPISODES,
        "readback": {
            "status": "PASS_READBACK",
            "audit_path": str(readback_audit_path),
            "audit_sha256": _sha256(readback_audit_path),
            "rows_path": str(readback_rows_path),
            "rows_sha256": _sha256(readback_rows_path),
            "frames": strict_audit["frames"],
            "decoded_rgb_frames_verified": True,
        },
        "formal_training_eligible": False,
        "training_started": False,
        "model_loaded": False,
        "field_data_admission": "PENDING_ONSITE_AND_FORMAL_DATA_REVIEW",
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
        "source_labels_modified": False,
        "network_install_attempted": False,
        "isolated_environment": environment,
        "limitations": [
            "The software contract validates derived-state structure, provenance, conversion, and readback only.",
            "A historical success label does not establish autonomous robot success or physical execution.",
            "Each converted episode is a filtered fragment and does not independently establish complete task success.",
            "The source label, action target, and nominal joint/gripper units remain as recorded and unverified.",
            "The one-episode software override does not change the strict contract or training entrypoint defaults.",
            "This report does not authorize training or promote formal training eligibility.",
            "Camera timestamps are host-after-read timestamps; the 20 ms pair limit is not exposure synchronization.",
        ],
    }
    report["report_path"] = str(report_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-id", action="append", required=True,
                        help="explicit hash-bound successful historical episode; repeat to add episodes")
    parser.add_argument("--run-id")
    parser.add_argument("--raw-root", type=Path, default=DEFAULT_RAW_ROOT)
    parser.add_argument("--outcome-root", type=Path, default=DEFAULT_OUTCOME_ROOT)
    parser.add_argument("--max-skew-ms", type=float, default=50.0)
    parser.add_argument("--window-policy", choices=(WINDOW_POLICY,), required=True,
                        help="explicitly select the longest contiguous valid 10 Hz source-grid window")
    parser.add_argument("--report-path", type=Path)
    args = parser.parse_args(argv)
    result = prepare_candidate(
        episode_ids=args.episode_id,
        run_id=args.run_id,
        raw_root=args.raw_root,
        outcome_root=args.outcome_root,
        max_skew_ms=args.max_skew_ms,
        window_policy=args.window_policy,
        report_path=args.report_path,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, PermissionError, ValueError, RuntimeError) as exc:
        print(f"training bridge stopped: {exc}", file=sys.stderr)
        raise SystemExit(2)
