"""Offline episode acceptance checks for live LeRobot capture sidecars."""

from __future__ import annotations

import math
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Iterable


CURRENT_PROXY_ACTION_SEMANTICS = "next_observed_state_proxy_nominal_units"


def _bool(value: Any) -> bool:
    """Return a conservative bool for scalar/list fields in JSON sidecars."""
    if hasattr(value, "numel") and callable(value.numel):
        if value.numel() == 1:
            return value.item() is True or value.item() == 1
        return False
    if hasattr(value, "item") and callable(value.item):
        try:
            scalar = value.item()
            return scalar is True or scalar == 1
        except (TypeError, ValueError):
            pass
    if isinstance(value, (list, tuple)):
        return len(value) == 1 and value[0] is True
    return value is True


def consecutive_valid_windows(
    rows: Iterable[dict[str, Any]],
    *,
    require_action: bool = True,
    require_observation: bool = True,
) -> list[dict[str, int]]:
    """Return contiguous frame ranges without joining across masked rows."""
    windows: list[dict[str, int]] = []
    start: int | None = None
    previous: int | None = None
    for index, row in enumerate(rows):
        camera = row.get("camera_valid", row.get("source_camera_valid", [False, False]))
        camera_ok = (camera is True or
                     isinstance(camera, (list, tuple)) and len(camera) == 2 and all(v is True for v in camera))
        valid = (camera_ok
                 and (not require_action or _bool(row.get("action_valid")))
                 and (not require_observation or _bool(row.get("observation_valid"))))
        if valid:
            if start is None:
                start = index
            previous = index
        elif start is not None and previous is not None:
            windows.append({"start_frame_index": start, "end_frame_index": previous,
                            "frames": previous - start + 1})
            start = None
            previous = None
    if start is not None and previous is not None:
        windows.append({"start_frame_index": start, "end_frame_index": previous,
                        "frames": previous - start + 1})
    return windows


def _finite_vector(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or not value:
        return False
    try:
        return all(math.isfinite(float(item)) for item in value)
    except (TypeError, ValueError, OverflowError):
        return False


def _row_command_evidence(row: dict[str, Any]) -> bool | None:
    """Read send-audit evidence without borrowing a next-state action mask."""
    if type(row.get("command_evidence_valid")) is bool:
        return bool(row["command_evidence_valid"])
    evidence = row.get("command_evidence")
    if isinstance(evidence, dict) and type(evidence.get("valid")) is bool:
        return bool(evidence["valid"])
    nested = row.get("source_map")
    action = nested.get("action") if isinstance(nested, dict) else None
    if isinstance(action, dict) and type(action.get("action_valid")) is bool:
        return bool(action["action_valid"])
    semantics = row.get("action_semantics")
    if isinstance(semantics, str) and (
        "send/receipt" in semantics
        or "kernel_accepted" in semantics
        or "kernel accepted" in semantics
    ):
        value = row.get("action_valid")
        return value if type(value) is bool else None
    return None


def _command_evidence_summary(
    rows: list[dict[str, Any]], *, label: dict[str, Any],
    metadata: dict[str, Any],
) -> dict[str, Any]:
    """Summarize send audit separately from next-state training labels."""
    supplied = metadata.get("command_evidence", label.get("command_evidence"))
    if isinstance(supplied, dict):
        blockers = supplied.get("blockers", supplied.get("reasons", []))
        valid = supplied.get("valid") is True or supplied.get("command_evidence_valid") is True
        status = supplied.get("status")
        if status not in {"COMPLETE", "PARTIAL", "MISSING", "UNKNOWN"}:
            status = "COMPLETE" if valid else "PARTIAL"
        normalized_blockers = (
            [str(item) for item in blockers] if isinstance(blockers, list) else []
        )
        if status != "COMPLETE" and not normalized_blockers:
            normalized_blockers = ["source_command_evidence_incomplete"]
        return {
            "valid": status == "COMPLETE",
            "status": status,
            "blockers": normalized_blockers,
            "source": "explicit",
        }

    evidence = [_row_command_evidence(row) for row in rows]
    known = [value for value in evidence if value is not None]
    if not known:
        status = "UNKNOWN"
        accepted_rows = 0
    else:
        accepted_rows = sum(known)
        if accepted_rows == len(rows) and len(known) == len(rows):
            status = "COMPLETE"
        elif accepted_rows:
            status = "PARTIAL"
        else:
            status = "MISSING"
    blockers = [] if status == "COMPLETE" else ["source_command_evidence_incomplete"]
    return {
        "valid": status == "COMPLETE",
        "status": status,
        "blockers": blockers,
        "accepted_command_rows": accepted_rows,
        "known_command_rows": len(known),
        "source": "row_audit",
    }


def assess_episode(
    rows: list[dict[str, Any]],
    *,
    label: dict[str, Any],
    metadata: dict[str, Any],
    dropped_frames: int,
    action_semantics: str,
    expected_action_semantics: str = CURRENT_PROXY_ACTION_SEMANTICS,
) -> dict[str, Any]:
    """Separate task outcome, training labels, command audit, and model fit.

    A successful disk write alone never makes an episode training eligible.
    Command evidence remains an independent audit: missing CAN receipts do not
    invalidate a complete next-observed-state label.
    """
    action_valid = [_bool(row.get("action_valid")) for row in rows]
    observation_valid = [_bool(row.get("observation_valid")) for row in rows]
    camera_values = [row.get("camera_valid", row.get("source_camera_valid", [False, False]))
                     for row in rows]
    camera_valid = [
        value is True or
        isinstance(value, (list, tuple)) and len(value) == 2 and all(item is True for item in value)
        for value in camera_values
    ]
    finite_state = [_finite_vector(row.get("state")) for row in rows]
    finite_action = [_finite_vector(row.get("action")) for row in rows]

    human_labeled = (label.get("human_labeled") is True
                     or label.get("label_source") in {"human", "operator"})
    task_success = (
        human_labeled
        and label.get("outcome") in {"success", "successful"}
        and label.get("successful_complete_end") is True
    )
    closeout_complete = (
        label.get("complete_closeout_label_present") is True
        and label.get("successful_complete_end") is True
    )
    timeline_complete = bool(rows) and dropped_frames == 0
    supervision_complete = bool(rows) and all(action_valid)
    observation_complete = bool(rows) and all(observation_valid) and all(camera_valid)
    finite_quality = all(finite_state) and all(
        (not valid) or finite_action[index]
        for index, valid in enumerate(action_valid)
    )
    quality_pass = (finite_quality and observation_complete and supervision_complete
                    and dropped_frames == 0)

    semantic_compatible = action_semantics == expected_action_semantics
    action_label_complete = supervision_complete and semantic_compatible
    action_label_blockers: list[str] = []
    if not supervision_complete:
        action_label_blockers.append("one_or_more_action_label_rows_are_masked")
    if not semantic_compatible:
        action_label_blockers.append(
            f"action_semantics_mismatch:{action_semantics}!={expected_action_semantics}"
        )
    command_evidence = _command_evidence_summary(rows, label=label, metadata=metadata)
    historical_review = (
        metadata.get("source_kind") == "historical_review"
        or metadata.get("review_only") is True
        or label.get("historical_review_only") is True
        or label.get("training_eligible") is False
    )

    reasons: list[str] = []
    if not task_success:
        reasons.append("explicit_human_success_label_missing")
    if not timeline_complete:
        reasons.append("writer_dropped_or_missing_timeline_frames")
    if not supervision_complete:
        reasons.append("one_or_more_action_label_rows_are_masked")
    if not observation_complete:
        reasons.append("one_or_more_observations_or_camera_pairs_are_masked")
    if not finite_quality:
        reasons.append("non_finite_state_or_valid_action_value")
    if not closeout_complete:
        reasons.append("successful_complete_closeout_label_missing")
    reasons.extend(reason for reason in action_label_blockers
                   if reason not in reasons)
    if historical_review:
        reasons.append("historical_review_package_cannot_be_promoted")

    training_eligible = bool(
        rows and task_success and timeline_complete and supervision_complete
        and observation_complete and finite_quality and closeout_complete
        and semantic_compatible and not historical_review
    )
    return {
        "schema": "piper_live_episode_acceptance_v1",
        "task_success": task_success,
        "timeline_complete": timeline_complete,
        "supervision_complete": supervision_complete,
        "action_label_complete": action_label_complete,
        "observation_complete": observation_complete,
        "quality_pass": quality_pass,
        "closeout_complete": closeout_complete,
        "action_semantics": action_semantics,
        "expected_action_semantics": expected_action_semantics,
        "action_semantics_compatible": semantic_compatible,
        "historical_review_only": historical_review,
        "command_evidence_valid": command_evidence["valid"],
        "command_evidence": command_evidence,
        "action_label_blockers": action_label_blockers,
        "command_evidence_blockers": command_evidence["blockers"],
        "training_eligible": training_eligible,
        "training_blockers": reasons,
        "frames": len(rows),
        "dropped_frames": int(dropped_frames),
        "action_valid_frames": sum(action_valid),
        "observation_valid_frames": sum(observation_valid),
        "camera_valid_frames": sum(camera_valid),
        "finite_state_frames": sum(finite_state),
        "finite_valid_action_frames": sum(
            finite_action[index] for index, valid in enumerate(action_valid) if valid
        ),
        "action_and_observation_valid_windows": consecutive_valid_windows(rows),
    }


def readback_preview(
    root: str | Path,
    *,
    run_id: str | None = None,
    episode_index: int | None = None,
    cache_root: str | Path | None = None,
    video_backend: str = "pyav",
) -> dict[str, Any]:
    """Load saved rows and decoded video through the pinned LeRobot loader.

    The function checks selected preview rows' mask/state/action against the
    local source map, both decoded video streams, and valid-only action
    statistics. It writes a review audit but never upgrades a label.
    """
    root = Path(root).expanduser().resolve()
    provenance = root / "provenance"
    identity = json.loads((provenance / "dataset_identity.json").read_text(encoding="utf-8"))
    sources = [json.loads(line) for line in
               (provenance / "run_sources.jsonl").read_text(encoding="utf-8").splitlines() if line]
    labels = [json.loads(line) for line in
              (provenance / "run_labels.jsonl").read_text(encoding="utf-8").splitlines() if line]
    if episode_index is None:
        matches = [row for row in sources if row.get("run_id") == run_id]
        if len(matches) != 1:
            raise ValueError("specify a run_id with exactly one saved episode or an episode_index")
        episode_index = int(matches[0]["episode_index"])
        run_id = str(matches[0]["run_id"])
    if run_id is None:
        matches = [row for row in sources if int(row.get("episode_index", -1)) == episode_index]
        if len(matches) != 1:
            raise ValueError("episode_index does not identify exactly one source record")
        run_id = str(matches[0]["run_id"])
    source = next((row for row in sources
                   if row.get("run_id") == run_id
                   and int(row.get("episode_index", -1)) == episode_index), None)
    label = next((row for row in labels
                  if row.get("run_id") == run_id
                  and int(row.get("episode_index", -1)) == episode_index), {})
    if source is None:
        raise ValueError(f"run {run_id} episode {episode_index} has no source sidecar")

    if cache_root is None:
        cache_root = root.parent / f".{root.name}_live_writer_cache"
    from gamepad_capture.tools.dataset import cli
    cli._configure_offline_cache(Path(cache_root))
    runtime = cli._check_runtime()
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(identity["repo_id"], root=root, video_backend=video_backend)
    if int(dataset.meta.total_episodes) <= episode_index:
        raise ValueError("LeRobot loader does not expose the requested episode")
    source_rows = [json.loads(line) for line in
                   (provenance / "source_map.jsonl").read_text(encoding="utf-8").splitlines() if line]
    rows = sorted((row for row in source_rows
                   if row.get("run_id") == run_id
                   and int(row.get("episode_index", -1)) == episode_index),
                  key=lambda row: int(row["episode_frame_index"]))
    if not rows:
        raise ValueError("source map has no rows for the requested episode")

    action_valid_count = sum(_bool(row.get("action_valid")) for row in rows)
    valid_index = next((i for i, row in enumerate(rows) if _bool(row.get("action_valid"))), None)
    selected = sorted({0, len(rows) - 1, valid_index} - {None})
    preview_samples = []
    for local_index in selected:
        row = rows[local_index]
        global_index = int(row["dataset_row_index"])
        sample = dataset[global_index]
        mask = _bool(sample["action_valid"])
        if mask != _bool(row.get("action_valid")):
            raise ValueError(f"action_valid readback mismatch at episode frame {local_index}")
        state = sample["observation.state"].detach().cpu().numpy().astype("float32")
        expected_state = row.get("state")
        if expected_state is not None and len(expected_state) == 7:
            import numpy as np
            if not np.allclose(state, np.asarray(expected_state, dtype=np.float32), rtol=0.0, atol=1e-6):
                raise ValueError(f"state readback mismatch at episode frame {local_index}")
        action = sample["action"].detach().cpu().numpy().astype("float32")
        if mask:
            import numpy as np
            expected_action = np.asarray(row["action"], dtype=np.float32)
            if not np.allclose(action, expected_action, rtol=0.0, atol=1e-6):
                raise ValueError(f"valid action readback mismatch at episode frame {local_index}")
        for key in ("observation.images.base", "observation.images.wrist"):
            image = sample[key]
            if image.ndim != 3 or image.shape[0] != 3:
                raise ValueError(f"decoded {key} has an unexpected shape")
            import torch
            if not bool(torch.isfinite(image).all()):
                raise ValueError(f"decoded {key} contains non-finite values")
        if local_index in selected:
            preview_samples.append({
                "episode_frame_index": local_index,
                "dataset_row_index": global_index,
                "state": state.tolist(),
                "action": action.tolist() if mask else None,
                "action_valid": mask,
                "image_shapes_chw": {
                    "base": list(sample["observation.images.base"].shape),
                    "wrist": list(sample["observation.images.wrist"].shape),
                },
            })

    import numpy as np
    stats = dataset.meta.episodes_stats[episode_index].get("action")
    if action_valid_count and stats is None:
        raise ValueError("valid actions exist but episode action statistics are missing")
    if stats is None:
        stats_count = 0
        stats_finite = True
    else:
        count_value = stats["count"]
        stats_count = int(count_value.reshape(-1)[0])
        stats_finite = bool(np.isfinite(np.asarray(stats["mean"])).all())
    if stats_count != action_valid_count:
        raise ValueError("episode action stats include masked placeholders or omit valid rows")
    if not stats_finite:
        raise ValueError("episode action stats contain non-finite values")

    report = {
        "schema": "piper_live_lerobot_readback_preview_v1",
        "status": "PASS_REVIEW_ONLY_LOADER_READBACK",
        "run_id": run_id,
        "episode_index": episode_index,
        "frames": len(rows),
        "action_valid_frames": action_valid_count,
        "sampled_source_state_values_verified": True,
        "sampled_valid_action_values_verified": True,
        "sampled_action_masks_verified": True,
        "decoded_base_video_verified": True,
        "decoded_wrist_video_verified": True,
        "action_stats_count_valid_rows_only": stats_count,
        "action_stats_finite": stats_finite,
        "readback_sampled_frame_indices": selected,
        "training_eligible": label.get("training_eligible") is True,
        "action_semantics": source.get(
            "action_semantics", "kernel_accepted_position_target_nominal_unverified"
        ),
        "current_proxy_contract_compatible": source.get("action_semantics") == CURRENT_PROXY_ACTION_SEMANTICS,
        "runtime": runtime,
        "preview_samples": preview_samples,
        "note": "Loader and video readback succeeded; this does not verify task success or robot execution.",
    }
    file_slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", run_id)[:96] or "run"
    file_hash = hashlib.sha256(run_id.encode("utf-8")).hexdigest()[:10]
    audit_path = provenance / f"{file_slug}_{file_hash}_readback_preview.json"
    temporary = audit_path.with_suffix(audit_path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                         encoding="utf-8")
    os.replace(temporary, audit_path)
    return report
