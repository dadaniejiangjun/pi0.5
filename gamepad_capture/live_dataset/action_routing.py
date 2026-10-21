"""Offline-only action-contract routing for saved PiPER dataset sidecars.

This module reads provenance JSON/JSONL only. It does not load LeRobot, ROS,
camera, USB, CAN, or model runtimes, and it never modifies a source dataset.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


PROXY_ACTION_SEMANTICS = "next_observed_state_proxy_nominal_units"
PROXY_ACTION_TARGET_KIND = "derived_future_observed_state_position_target"
PROXY_JOINT_UNITS = "radian_nominal_unverified"
PROXY_GRIPPER_UNITS = "meter_nominal_unverified"

# The first value is the current writer contract. The other spellings are
# preserved historical labels observed in offline export sidecars. They all
# identify kernel-accepted targets, not the next-observed-state proxy.
KERNEL_ACTION_SEMANTICS = "kernel_accepted_position_target_nominal_unverified"
_KERNEL_SEMANTICS_ALIASES = {
    KERNEL_ACTION_SEMANTICS,
    "socketcan_kernel_accepted_position_target_nominal_unverified",
    "position target decoded from 4 CAN_KERNEL_ACCEPTED frames; not physical execution",
    "position target decoded from 4 accepted CAN bytes; not physical execution",
}


def _jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at {path}:{line_number}: {exc}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"expected JSON object at {path}:{line_number}")
            rows.append(row)
    return rows


def _json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _bool(value: Any) -> bool | None:
    if type(value) is bool:
        return value
    if isinstance(value, (list, tuple)) and len(value) == 1 and type(value[0]) is bool:
        return value[0]
    return None


def _finite_vector(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        return False
    return all(type(item) in (int, float) and math.isfinite(float(item))
               for item in value)


def _has_proxy_target_hash(row: dict[str, Any]) -> bool:
    target = row.get("proxy_target")
    if not isinstance(target, dict):
        return False
    digest = target.get("action_float32_sha256")
    return (isinstance(digest, str) and len(digest) == 64
            and all(char in "0123456789abcdef" for char in digest.lower()))


def _has_action_payload(row: dict[str, Any]) -> bool:
    action = row.get("action")
    if _finite_vector(action):
        return True
    if isinstance(action, dict) and _finite_vector(action.get("target_nominal")):
        return True
    # The legacy proxy source map stores the protected action hash here; the
    # strict legacy dataset contract verifies values against LeRobot readback.
    return _has_proxy_target_hash(row)


def _action_row_state(row: dict[str, Any]) -> tuple[str, str | None]:
    if "action_valid" not in row:
        if _has_proxy_target_hash(row):
            return "valid", None
        return "unknown", "action_valid_missing"
    valid = _bool(row.get("action_valid"))
    if valid is False:
        return "masked", "action_valid_false"
    if valid is None:
        return "unknown", "action_valid_unknown"
    if not _has_action_payload(row):
        return "invalid", "valid_action_payload_missing_or_nonfinite"
    return "valid", None


def _semantic_category(value: Any) -> str:
    if value == PROXY_ACTION_SEMANTICS:
        return "proxy"
    if value in _KERNEL_SEMANTICS_ALIASES:
        return "kernel"
    return "unknown"


def _episode_key(row: dict[str, Any]) -> tuple[str, int | None]:
    identity = row.get("run_id") or row.get("raw_episode_id")
    index = row.get("episode_index")
    return (str(identity) if identity is not None else "", index if type(index) is int else None)


def _unit_fields(source: dict[str, Any], conversion: dict[str, Any]) -> dict[str, Any]:
    nested = source.get("metadata")
    nested = nested if isinstance(nested, dict) else {}
    return {
        "action_units": source.get("action_units", nested.get("action_units")),
        "joint_units": conversion.get("joint_units", source.get("joint_units")),
        "gripper_units": conversion.get("gripper_units", source.get("gripper_units")),
    }


def _source_identity(first_row: dict[str, Any], source: dict[str, Any],
                     conversion: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_id": source.get("run_id", first_row.get("run_id")),
        "episode_index": source.get("episode_index", first_row.get("episode_index")),
        "raw_episode_id": first_row.get("raw_episode_id"),
        "manifest_sha256": source.get("manifest_sha256"),
        "bag_db_sha256": source.get("bag_db_sha256"),
        "source_map_sha256": conversion.get("source_map_sha256"),
    }


def classify_episode(rows: Iterable[dict[str, Any]], *,
                     source: dict[str, Any] | None = None,
                     label: dict[str, Any] | None = None,
                     conversion: dict[str, Any] | None = None) -> dict[str, Any]:
    """Classify one episode without changing or returning action values.

    ``legacy_proxy_candidate`` means the action meaning can be routed to the
    existing strict proxy pipeline. It does not mean the episode passed that
    pipeline's provenance, label, split, image, or readback checks.
    """
    rows = list(rows)
    source = source or {}
    label = label or {}
    conversion = conversion or {}
    semantics_values = {
        value for value in (
            source.get("action_semantics"),
            conversion.get("action_semantics"),
            *(row.get("action_semantics") for row in rows),
        ) if isinstance(value, str) and value
    }
    categories = {_semantic_category(value) for value in semantics_values}
    missing_row_semantics = sum(not isinstance(row.get("action_semantics"), str)
                                or not row.get("action_semantics") for row in rows)

    valid_frames = 0
    masked_frames = 0
    unknown_frames = 0
    invalid_payload_frames = 0
    frame_reasons: dict[str, int] = {}
    masked_indices: list[int] = []
    observation_invalid = 0
    camera_invalid = 0
    for fallback_index, row in enumerate(rows):
        state, reason = _action_row_state(row)
        if state == "valid":
            valid_frames += 1
        elif state == "masked":
            masked_frames += 1
            masked_indices.append(int(row.get("episode_frame_index", fallback_index)))
        elif state == "invalid":
            invalid_payload_frames += 1
        else:
            unknown_frames += 1
        if reason:
            frame_reasons[reason] = frame_reasons.get(reason, 0) + 1
        if _bool(row.get("observation_valid")) is False:
            observation_invalid += 1
            frame_reasons["observation_valid_false"] = frame_reasons.get(
                "observation_valid_false", 0
            ) + 1
        camera = row.get("camera_valid")
        camera_is_valid = (camera is True or isinstance(camera, (list, tuple))
                           and len(camera) == 2 and all(_bool(item) is True for item in camera))
        if camera is not None and not camera_is_valid:
            camera_invalid += 1
            frame_reasons["camera_valid_false_or_incomplete"] = frame_reasons.get(
                "camera_valid_false_or_incomplete", 0
            ) + 1

    unit_fields = _unit_fields(source, conversion)
    action_target_kind = conversion.get(
        "action_target_kind", source.get("action_target_kind")
    )
    reasons: list[str] = []
    if not rows:
        reasons.append("episode_has_no_source_rows")
    if missing_row_semantics:
        reasons.append("action_semantics_missing_or_unknown")
    if "unknown" in categories or not categories:
        reasons.append("action_semantics_missing_or_unknown")
    if len(categories) > 1:
        reasons.append("conflicting_action_semantics_categories")
    if masked_frames:
        reasons.append("masked_action_rows_present")
    if unknown_frames:
        reasons.append("action_validity_unknown_or_missing")
    if invalid_payload_frames:
        reasons.append("valid_action_payload_missing_or_nonfinite")
    if observation_invalid:
        reasons.append("observation_invalid_frames_present")
    if camera_invalid:
        reasons.append("camera_invalid_frames_present")
    if label:
        human_labeled = (label.get("human_labeled") is True
                         or label.get("label_source") in {"human", "operator"})
        if (not human_labeled
                or label.get("outcome") not in {"success", "successful"}
                or label.get("successful_complete_end") is not True):
            reasons.append("explicit_human_success_label_missing")
        if (label.get("complete_closeout_label_present") is not True
                or label.get("successful_complete_end") is not True):
            reasons.append("successful_complete_closeout_label_missing")
        if label.get("delivery_incomplete") is True:
            reasons.append("delivery_incomplete")
        if label.get("training_eligible") is False:
            reasons.append("label_training_eligibility_false_as_recorded")

    if categories == {"kernel"}:
        reasons.append("kernel_target_not_compatible_with_proxy_pipeline")
        route = "kernel_review_only"
    elif categories == {"proxy"}:
        if action_target_kind != PROXY_ACTION_TARGET_KIND:
            reasons.append("proxy_action_target_kind_missing_or_mismatch")
        if unit_fields.get("joint_units") != PROXY_JOINT_UNITS:
            reasons.append("proxy_joint_units_missing_or_mismatch")
        if unit_fields.get("gripper_units") != PROXY_GRIPPER_UNITS:
            reasons.append("proxy_gripper_units_missing_or_mismatch")
        if (source.get("source_kind") == "historical_review"
                or source.get("review_only") is True
                or label.get("historical_review_only") is True):
            reasons.append("historical_review_package_cannot_be_promoted")
        route = "legacy_proxy_candidate" if not reasons else "rejected"
    else:
        route = "rejected"

    # Keep reasons deterministic while removing duplicate missing-semantic
    # reports when both the episode and its rows omit the value.
    reasons = list(dict.fromkeys(reasons))
    first = rows[0] if rows else {}
    return {
        "route": route,
        "training_eligible": False,
        "training_started": False,
        "frames": len(rows),
        "action_valid_frames": valid_frames,
        "masked_action_frames": masked_frames,
        "unknown_action_frames": unknown_frames,
        "invalid_action_payload_frames": invalid_payload_frames,
        "masked_frame_indices_preview": masked_indices[:12],
        "frame_rejection_reasons": frame_reasons,
        "observation_invalid_frames": observation_invalid,
        "camera_invalid_frames": camera_invalid,
        "action_semantics_values": sorted(semantics_values),
        "action_semantics_categories": sorted(categories),
        "action_target_kind": action_target_kind,
        "unit_fields_as_recorded": unit_fields,
        "source_identity": _source_identity(first, source, conversion),
        "label_training_eligible_as_recorded": label.get(
            "training_eligible", source.get("formal_training_eligible")
        ),
        "legacy_entrypoint": (
            "server.training.post_capture_pipeline / proxy_multi_episode_pilot"
            if route == "legacy_proxy_candidate" else None
        ),
        "legacy_contract_must_still_pass": route == "legacy_proxy_candidate",
        "reasons": reasons,
    }


def inspect_dataset(dataset_path: str | Path) -> dict[str, Any]:
    """Read provenance sidecars and return an episode-preserving route report."""
    root = Path(dataset_path).expanduser().resolve(strict=True)
    provenance = root / "provenance"
    source_map_path = provenance / "source_map.jsonl"
    if not source_map_path.is_file():
        raise FileNotFoundError(f"missing source map: {source_map_path}")
    rows = _jsonl(source_map_path)
    sources = _jsonl(provenance / "run_sources.jsonl")
    labels = _jsonl(provenance / "run_labels.jsonl")
    conversion = _json(provenance / "proxy_conversion.json")
    if not conversion:
        conversion = _json(provenance / "source_map_report.json")
    # Live source sidecars carry semantics per run. A dataset-level converter
    # contract is authoritative only for the legacy derived-proxy layout.
    if sources:
        conversion = {}

    source_lookup = {
        (str(item.get("run_id", "")), item.get("episode_index")): item
        for item in sources
    }
    label_lookup = {
        (str(item.get("run_id", "")), item.get("episode_index")): item
        for item in labels
    }
    grouped: dict[tuple[str, int | None], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(_episode_key(row), []).append(row)

    episode_results = []
    for key, episode_rows in sorted(grouped.items(), key=lambda item: (item[0][0], item[0][1] or -1)):
        run_id, episode_index = key
        source = source_lookup.get(key, {})
        label = label_lookup.get(key, {})
        # Legacy derived-state sidecars use raw_episode_id rather than run_id.
        raw_id = episode_rows[0].get("raw_episode_id")
        if not source and isinstance(conversion.get("source_episodes"), list):
            source = next((item for item in conversion["source_episodes"]
                           if isinstance(item, dict)
                           and (item.get("raw_episode_id") == raw_id
                                or item.get("episode_index") == episode_index)), {})
        episode_results.append(classify_episode(
            sorted(episode_rows, key=lambda row: int(row.get("episode_frame_index", 0))),
            source=source, label=label, conversion=conversion,
        ))

    routes = {name: [item for item in episode_results if item["route"] == name]
              for name in ("legacy_proxy_candidate", "kernel_review_only", "rejected")}
    hashes = {path.name: _sha256(path) for path in (
        source_map_path,
        provenance / "run_sources.jsonl",
        provenance / "run_labels.jsonl",
        provenance / "proxy_conversion.json",
        provenance / "source_map_report.json",
    ) if path.is_file()}
    return {
        "schema": "piper_action_training_route_report_v1",
        "status": "OFFLINE_ACTION_ROUTING_COMPLETE",
        "dataset_path": str(root),
        "dataset_training_eligible": False,
        "training_started": False,
        "historical_review_promoted": False,
        "source_map_rows": len(rows),
        "episodes": len(episode_results),
        "route_counts": {name: len(items) for name, items in routes.items()},
        "source_sidecar_sha256": hashes,
        "routes": routes,
        "notes": [
            "Routing classifies action semantics; it does not qualify a dataset for training.",
            "Masked placeholder values are never copied into a training route.",
            "The legacy proxy pipeline must still pass its strict dataset contract.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-path", type=Path, required=True)
    parser.add_argument("--report-path", type=Path,
                        help="optional new JSON report path; existing files are not overwritten")
    args = parser.parse_args(argv)
    report = inspect_dataset(args.dataset_path)
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.report_path is not None:
        output = args.report_path.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        report["report_path"] = str(output)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        with output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, PermissionError, ValueError) as exc:
        print(f"action routing stopped: {exc}")
        raise SystemExit(2)
