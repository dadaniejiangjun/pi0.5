"""Explicit dataset provenance, software fixtures, and training admission.

Historical review datasets remain review-only after format conversion. Synthetic
fixtures exercise software, never certify a robot policy or physical units.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

CAMERAS = ("observation.images.base", "observation.images.wrist")
JOINTS = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]
MANIFEST = "project_training_manifest.json"
MIN_FEEDBACK_EPISODES_FOR_FORMAL_ADMISSION = 5


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprint(root: str | Path) -> dict[str, str]:
    root = Path(root)
    return {str(p.relative_to(root)).replace("\\", "/"): sha256(p)
            for p in sorted(root.rglob("*")) if p.is_file()}


def write_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def contract() -> dict:
    return {"fps": 10, "action_horizon": 16, "state_order": JOINTS,
            "action_order": JOINTS, "camera_keys": list(CAMERAS),
            "color": "RGB", "units": "synthetic_unitless",
            "tail_policy": "upstream_repeat_with_action_is_pad",
            "mask_policy": "reject_any_invalid_before_training",
            "statistics_scope": "train_episodes_only"}


def admit_dataset(root: str | Path, *, software_only: bool = False) -> dict:
    """Fail closed: absence, false eligibility, or review origin is never bypassed."""
    root = Path(root)
    manifest = json.loads((root / MANIFEST).read_text(encoding="utf-8"))
    if manifest.get("schema") != "piper_lerobot_training_v1":
        raise ValueError("unknown training manifest schema")
    kind = manifest.get("source_kind")
    if kind == "synthetic":
        if not software_only or not manifest.get("software_training_eligible"):
            raise ValueError("synthetic dataset requires explicit software-only training")
        if manifest.get("training_eligible") is not False:
            raise ValueError("synthetic data must not acquire real training qualification")
    elif kind == "field_approved":
        if software_only or manifest.get("training_eligible") is not True:
            raise ValueError("field data requires explicit approved training eligibility")
        if manifest.get("field_admission_status") != "PASS":
            raise ValueError("field admission evidence absent")
        evidence = root / manifest.get("field_admission_evidence", "__missing__")
        if not evidence.is_file() or sha256(evidence) != manifest.get("field_admission_evidence_sha256"):
            raise ValueError("field admission evidence missing or changed")
    elif kind in ("verified_feedback_proxy", "synthetic_feedback_proxy"):
        synthetic_feedback = kind == "synthetic_feedback_proxy"
        if synthetic_feedback:
            if (not software_only or manifest.get("training_eligible") is not False
                    or manifest.get("software_training_eligible") is not True):
                raise ValueError("synthetic feedback proxy is admitted only as software-only data")
        elif software_only or manifest.get("training_eligible") is not True:
            raise ValueError("field feedback proxy requires the existing strict offline proxy report")
        if (manifest.get("formal_training_eligible") is not False
                or manifest.get("hardware_control_allowed") is not False
                or manifest.get("action_semantics") != "next_observed_state_proxy_nominal_units"
                or manifest.get("action_target_kind") != "derived_future_observed_state_position_target"
                or manifest.get("raw_command_supervision") is not False):
            raise ValueError("feedback proxy semantics or physical qualification changed")
        evidence = root / manifest.get("legacy_validation_evidence", "__missing__")
        if not evidence.is_file() or sha256(evidence) != manifest.get("legacy_validation_evidence_sha256"):
            raise ValueError("legacy proxy validation evidence missing or changed")
        legacy = json.loads(evidence.read_text(encoding="utf-8"))
        strict = legacy.get("strict_inspect", {})
        if (legacy.get("schema") != "piper_feedback_legacy_handoff_v1"
                or legacy.get("status") != "LEGACY_OFFLINE_PROXY_GATE_PASS"
                or strict.get("dataset_sha256") != manifest.get("legacy_validated_dataset_sha256")
                or strict.get("episodes", 0) < MIN_FEEDBACK_EPISODES_FOR_FORMAL_ADMISSION
                or strict.get("fps") != 10
                or strict.get("action_semantics") != "next_observed_state_proxy_nominal_units"
                or strict.get("action_target_kind") != "derived_future_observed_state_position_target"
                or strict.get("raw_command_supervision") is not False
                or strict.get("offline_training_allowed") is not True
                or strict.get("formal_training_eligible") is not False
                or strict.get("hardware_control_allowed") is not False
                or legacy.get("dataset_tree_sha256") != manifest.get("legacy_dataset_tree_sha256")
                or legacy.get("source_adapter_manifest_sha256") != manifest.get("source_adapter_manifest_sha256")):
            raise ValueError("legacy handoff report does not satisfy the existing offline gate")
        _verify_legacy_handoff(root, manifest, evidence)
    elif kind == "verified_online_feedback_proxy":
        from .online_data import verify_admission
        verify_admission(root, manifest, software_only=software_only)
    else:
        raise ValueError("review/proxy migration does not grant training qualification")
    if manifest.get("invalid_rows") != 0:
        raise ValueError("invalid masks cannot be ignored by the trainer")
    c = manifest["contract"]
    if c["fps"] != 10 or c["action_horizon"] != 16 or c["camera_keys"] != list(CAMERAS):
        raise ValueError("project shape/time contract mismatch")
    splits = manifest["splits"]
    if not splits["train"] or not splits["validation"] or set(splits["train"]) & set(splits["validation"]):
        raise ValueError("episode split missing or overlapping")
    origins = manifest["episode_origins"]
    if {origins[str(i)] for i in splits["train"]} & {origins[str(i)] for i in splits["validation"]}:
        raise ValueError("source origin leakage")
    for relative, expected in manifest["dataset_files"].items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root.resolve()) or not path.is_file() or sha256(path) != expected:
            raise ValueError(f"dataset file missing or changed: {relative}")
    return manifest


def valid_window_starts(rows: list[dict], horizon: int = 16, fps: int = 10) -> list[int]:
    """Strict windows only: no gap, episode crossing, invalid mask, or tail padding."""
    starts = []
    for i in range(len(rows) - horizon + 1):
        w = rows[i:i + horizon]
        if any(not all(r.get(k) is True for k in
                       ("action_valid", "observation_valid", "base_valid", "wrist_valid")) for r in w):
            continue
        if any(r["episode_index"] != w[0]["episode_index"] for r in w):
            continue
        if any(abs((b["timestamp"] - a["timestamp"]) - 1 / fps) > 1e-5 for a, b in zip(w, w[1:])):
            continue
        starts.append(i)
    return starts


def prepare_synthetic(root: str | Path, *, episodes: int = 4, frames: int = 32,
                      image_size: int = 64, seed: int = 19) -> Path:
    """Create independent episodes with official v3 writer, never copy field RGB."""
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = Path(root)
    if root.exists():
        raise FileExistsError(root)
    if episodes < 2 or frames < 16:
        raise ValueError("need independent train/validation episodes and complete H16")
    features = {"observation.state": {"dtype": "float32", "shape": (7,), "names": JOINTS},
                "action": {"dtype": "float32", "shape": (7,), "names": JOINTS}}
    features.update({k: {"dtype": "image", "shape": (image_size, image_size, 3),
                         "names": ["height", "width", "channels"]} for k in CAMERAS})
    ds = LeRobotDataset.create(repo_id="local/piper_synthetic_smoke", root=root,
                               fps=10, features=features, use_videos=False,
                               robot_type="piper_software_fixture")
    provenance = []
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[:image_size, :image_size]
    for ep in range(episodes):
        phase = rng.uniform(-1, 1, 7)
        for t in range(frames):
            state = np.sin(phase + t * .05).astype(np.float32)
            action = np.sin(phase + (t + 1) * .05).astype(np.float32)
            frame = {"observation.state": state, "action": action,
                     "task": "software fixture: follow a synthetic smooth target"}
            for cam_i, key in enumerate(CAMERAS):
                frame[key] = np.stack(((xx + t * 3 + ep * 17) % 256,
                                       (yy * 2 + cam_i * 73) % 256,
                                       np.full_like(xx, (ep * 41 + t + cam_i * 97) % 256)), -1).astype(np.uint8)
            ds.add_frame(frame)
            provenance.append({"episode_index": ep, "frame_index": t, "timestamp": t / 10,
                               "origin": f"synthetic_seed_{seed}_episode_{ep}",
                               "action_valid": True, "observation_valid": True,
                               "base_valid": True, "wrist_valid": True})
        ds.save_episode()
    ds.finalize()
    write_json(root / "provenance" / "synthetic_rows.json", provenance)
    m = {"schema": "piper_lerobot_training_v1", "source_kind": "synthetic",
         "training_eligible": False, "software_training_eligible": True,
         "claim": "software-only, no real robot qualification", "seed": seed,
         "invalid_rows": 0, "contract": contract(),
         "splits": {"train": list(range(episodes - 1)), "validation": [episodes - 1]},
         "episode_origins": {str(i): f"synthetic_seed_{seed}_episode_{i}" for i in range(episodes)},
         "strict_full_window_starts": valid_window_starts(provenance),
         "dataset_files": fingerprint(root)}
    write_json(root / MANIFEST, m)
    admit_dataset(root, software_only=True)
    return root / MANIFEST


def migrate_review_copy(source: str | Path, target: str | Path, *, repo_id: str) -> dict:
    """Read v2.1 parquet/video and write official v3; qualification stays false.

    The pinned 0.6.1 wheel no longer ships the old v30 migration script. This
    explicit adapter does not silently drop masks or recreate provenance.
    Images are decoded once to lossless PNG by the official writer.
    """
    import av
    import numpy as np
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    source, target = Path(source), Path(target)
    if target.exists() or target.resolve().is_relative_to(source.resolve()):
        raise ValueError("target must be a new separate copy")
    before = fingerprint(source)
    info = json.loads((source / "meta/info.json").read_text())
    if info["codebase_version"] != "v2.1" or info["fps"] != 10:
        raise ValueError("expected project v2.1 10Hz source")
    features = {k: dict(v) for k, v in info["features"].items()
                if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}
    for key, value in features.items():
        if value["dtype"] == "video":
            shape = value["shape"]
            if shape[0] == 3:
                c, h, w = shape
            elif shape[-1] == 3:
                h, w, c = shape
            else:
                raise ValueError("unsupported source RGB shape")
            features[key] = {"dtype": "image", "shape": (h, w, c),
                             "names": ["height", "width", "channels"]}
    ds = LeRobotDataset.create(repo_id=repo_id, root=target, fps=10,
                               features=features, use_videos=False, robot_type=info["robot_type"])
    provenance = target / "project_original_sidecars"
    provenance.mkdir()
    for file in source.rglob("*"):
        if file.is_file() and file.suffix.lower() in (".json", ".jsonl"):
            out = provenance / file.relative_to(source)
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(file, out)
    episodes = [json.loads(line) for line in (source / "meta/episodes.jsonl").read_text().splitlines()]
    row_checks = []
    for episode in episodes:
        if len(episode["tasks"]) != 1:
            raise ValueError("multi-task episode needs explicit per-row task mapping")
        ep = episode["episode_index"]
        fmt = {"episode_index": ep, "episode_chunk": ep // info["chunks_size"]}
        rows = pq.read_table(source / info["data_path"].format(**fmt)).to_pylist()
        video_frames = {}
        for key, feature in info["features"].items():
            if feature["dtype"] == "video":
                with av.open(str(source / info["video_path"].format(**fmt, video_key=key))) as video:
                    video_frames[key] = [f.to_ndarray(format="rgb24") for f in video.decode(video=0)]
                if len(video_frames[key]) != len(rows):
                    raise ValueError("video/parquet length mismatch")
        for i, row in enumerate(rows):
            if abs(row["timestamp"] - i / 10) > 1e-4:
                raise ValueError("source has timestamp gap; refusing implicit resampling")
            frame = {"task": episode["tasks"][0]}
            hashes = {}
            for key, feature in features.items():
                if key in video_frames:
                    value = video_frames[key][i]
                else:
                    value = np.asarray(row[key], dtype=feature["dtype"]).reshape(feature["shape"])
                frame[key] = value
                hashes[key] = hashlib.sha256(value.tobytes()).hexdigest()
            ds.add_frame(frame)
            row_checks.append({"episode_index": ep, "frame_index": i,
                               "source_timestamp": row["timestamp"], "field_sha256": hashes})
        ds.save_episode()
    ds.finalize()
    # Read every output value and every decoded RGB frame, rather than trust writer success.
    readback = LeRobotDataset(repo_id=repo_id, root=target)
    for j, check in enumerate(row_checks):
        row = readback[j]
        for key, expected in check["field_sha256"].items():
            value = row[key].numpy()
            if features[key]["dtype"] == "image":
                value = np.rint(value.transpose(1, 2, 0) * 255).astype(np.uint8)
            else:
                value = value.astype(features[key]["dtype"])
            if hashlib.sha256(value.tobytes()).hexdigest() != expected:
                raise ValueError(f"migration readback mismatch {j} {key}")
    write_json(target / "provenance/migration_rows.json", row_checks)
    if fingerprint(source) != before:
        raise RuntimeError("source fingerprint changed")
    result = {"schema": "piper_lerobot_training_v1", "source_kind": "review_migration",
              "training_eligible": False, "software_training_eligible": False,
              "source_fingerprint": before, "source_unchanged": True,
              "dataset_files": fingerprint(target), "contract": contract(),
              "readback_rows": len(row_checks), "all_field_hashes_match": True,
              "adapter": "project v2.1 reader to official v3 image writer"}
    result["contract"]["units"] = "nominal_unverified_preserved_from_source"
    result["contract"]["camera_keys"] = [k for k, v in features.items() if v["dtype"] == "image"]
    result["contract"]["state_order"] = features["observation.state"].get("names")
    result["contract"]["action_order"] = features["action"].get("names")
    result["original_features"] = info["features"]
    write_json(target / MANIFEST, result)
    return result


def inspect_feedback_episode(rows: list[dict], source: dict, label: dict) -> dict:
    """Check whether one captured episode can yield next-feedback proxy rows.

    This is an episode-level gate. It never concatenates valid fragments or
    borrows validity from the previous action/feedback row.
    """
    reasons: list[str] = []
    outcome = label.get("outcome")
    success = outcome in ("success", "successful") or label.get("success") is True
    complete = (label.get("successful_complete_end") is True
                or label.get("success") is True and label.get("complete_end") is True)
    closeout = (label.get("complete_closeout_label_present") is True
                or label.get("success") is True and label.get("complete_end") is True)
    if not (success and complete and closeout):
        reasons.append("explicit_successful_complete_closeout_missing")
    if label.get("delivery_incomplete") is not False:
        reasons.append("delivery_incomplete_or_unknown")
    if not rows:
        reasons.append("episode_has_no_rows")
    if len(rows) < 17:
        reasons.append("fewer_than_17_feedback_rows_for_H16_next_state_targets")

    invalid: dict[str, int] = {}
    for index, row in enumerate(rows):
        state = row.get("state")
        values = state.get("values_float32") if isinstance(state, dict) else None
        if (not isinstance(values, list) or len(values) != 7
                or any(type(v) not in (int, float) or not math.isfinite(float(v)) for v in values)):
            invalid["feedback_state_missing_or_nonfinite"] = invalid.get("feedback_state_missing_or_nonfinite", 0) + 1
        if not isinstance(state, dict) or state.get("feedback_fresh") is not True or state.get("fake") is not False:
            invalid["feedback_not_fresh_or_marked_fake"] = invalid.get("feedback_not_fresh_or_marked_fake", 0) + 1
        if row.get("observation_valid") is not True:
            invalid["observation_invalid_or_unknown"] = invalid.get("observation_invalid_or_unknown", 0) + 1
        camera_valid = row.get("camera_valid")
        if camera_valid is not True and not (isinstance(camera_valid, list) and len(camera_valid) == 2
                                             and all(v is True for v in camera_valid)):
            invalid["camera_invalid_or_unknown"] = invalid.get("camera_invalid_or_unknown", 0) + 1
        images = row.get("images")
        if not isinstance(images, dict) or any(not isinstance(images.get(k), dict)
                                                or not images[k].get("rgb_pixels_sha256")
                                                for k in ("base", "wrist")):
            invalid["camera_provenance_missing"] = invalid.get("camera_provenance_missing", 0) + 1
        if index:
            prev = rows[index - 1]
            prev_state = prev.get("state") if isinstance(prev, dict) else None
            stamp = state.get("source_local_monotonic_ns") if isinstance(state, dict) else None
            prev_stamp = prev_state.get("source_local_monotonic_ns") if isinstance(prev_state, dict) else None
            if type(stamp) is not int or type(prev_stamp) is not int or stamp <= prev_stamp:
                invalid["raw_feedback_timestamp_not_monotonic"] = invalid.get("raw_feedback_timestamp_not_monotonic", 0) + 1
            prev_grid, grid = prev.get("frame_timestamp_s"), row.get("frame_timestamp_s")
            if (type(prev_grid) not in (int, float) or type(grid) not in (int, float)
                    or abs((float(grid) - float(prev_grid)) - 0.1) > 1e-5):
                invalid["policy_grid_gap_or_non_10hz"] = invalid.get("policy_grid_gap_or_non_10hz", 0) + 1
            prev_session = prev_state.get("session_id") if isinstance(prev_state, dict) else None
            session = state.get("session_id") if isinstance(state, dict) else None
            if prev_session is not None and session != prev_session:
                invalid["session_boundary_inside_episode"] = invalid.get("session_boundary_inside_episode", 0) + 1
    if invalid:
        reasons.append("invalid_feedback_observation_or_camera_rows_present")
    return {"episode_id": source.get("run_id"), "episode_index": source.get("episode_index"),
            "frames": len(rows), "proxy_rows": max(0, len(rows) - 1),
            "accepted_as_complete_success_source": not reasons,
            "reasons": reasons, "frame_rejection_reasons": invalid,
            "source_training_eligible_as_recorded": source.get("formal_training_eligible"),
            "label_training_eligible_as_recorded": label.get("training_eligible")}


def _run_legacy_worker(python: str | Path, args: list[str], runtime_root: str | Path,
                       *, timeout_s: int = 1800) -> dict:
    python = Path(python).resolve(strict=True)
    runtime_root = Path(runtime_root).resolve()
    if not python.is_file():
        raise FileNotFoundError(python)
    cache = runtime_root / "process_cache"
    env = os.environ.copy()
    for key, directory in {
        "HF_HOME": cache / "hf_home", "HF_DATASETS_CACHE": cache / "hf_datasets",
        "HF_LEROBOT_HOME": cache / "hf_lerobot", "TEMP": cache / "temp", "TMP": cache / "temp",
    }.items():
        Path(directory).mkdir(parents=True, exist_ok=True)
        env[key] = str(directory)
    env.update({"HF_HUB_OFFLINE": "1", "HF_DATASETS_OFFLINE": "1",
                "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1"})
    project = Path(__file__).resolve().parents[2]
    previous_path = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(project) + (os.pathsep + previous_path if previous_path else "")
    command = [str(python), "-B", "-m", "server.lerobot_training.legacy_worker", *args]
    result = subprocess.run(command, cwd=project, env=env, capture_output=True,
                            text=True, timeout=timeout_s, check=False)
    if result.returncode != 0:
        details = (result.stderr or result.stdout)[-4000:]
        raise RuntimeError(f"legacy worker failed ({result.returncode}): {details}")
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("legacy worker returned no status")
    try:
        return json.loads(lines[-1])
    except json.JSONDecodeError as exc:
        raise RuntimeError("legacy worker did not return a JSON status") from exc


def _verify_legacy_handoff(root: Path, manifest: dict, evidence: Path) -> dict:
    python = manifest.get("legacy_python_executable")
    report = json.loads(evidence.read_text(encoding="utf-8"))
    dataset_path = report.get("dataset_path")
    result = _run_legacy_worker(python, ["verify", "--report", str(evidence),
                                         "--dataset", str(dataset_path),
                                         "--dataset-tree-sha256", str(manifest.get("legacy_dataset_tree_sha256"))],
                                root.parent / (root.name + ".legacy_stage"))
    if result.get("status") != "FRESH_LEGACY_INSPECT_PASS":
        raise ValueError("old-runtime inspect did not revalidate the legacy dataset")
    return result


def _validate_legacy_proxy_report(report_path: str | Path, accepted: list[dict],
                                  *, legacy_python: str | Path,
                                  runtime_root: str | Path) -> tuple[dict, str]:
    report_path = Path(report_path).resolve(strict=True)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    strict = report.get("strict_inspect", {})
    strict_episodes = strict.get("episodes")
    if (report.get("schema") != "piper_feedback_legacy_handoff_v1"
            or report.get("status") != "LEGACY_OFFLINE_PROXY_GATE_PASS"
            or strict.get("status") != "READY_FOR_OFFLINE_DERIVED_STATE_PILOT"
            or type(strict_episodes) is not int
            or strict_episodes < MIN_FEEDBACK_EPISODES_FOR_FORMAL_ADMISSION
            or strict.get("fps") != 10
            or strict.get("offline_training_allowed") is not True
            or strict.get("formal_training_eligible") is not False
            or strict.get("hardware_control_allowed") is not False):
        raise ValueError("old-runtime handoff is not a strict offline-pilot pass")
    bindings = report.get("source_episode_bindings", [])
    accepted_by_key = {(item["key"][0], item["key"][1]): item for item in accepted}
    binding_keys = [(item.get("source_run_id"), item.get("source_episode_index")) for item in bindings]
    if len(bindings) != len(accepted) or len(set(binding_keys)) != len(binding_keys):
        raise ValueError("legacy report source episode bindings are missing or duplicated")
    if set(binding_keys) != set(accepted_by_key):
        raise ValueError("legacy report source episode identities differ from accepted source")
    origin_hashes = [item.get("source_manifest_sha256") for item in bindings]
    requested_hashes = [accepted_by_key[key]["source"].get("manifest_sha256") for key in binding_keys]
    if (None in origin_hashes or len(set(origin_hashes)) != len(origin_hashes)
            or set(origin_hashes) != set(requested_hashes)):
        raise ValueError("legacy report is not bound one-to-one to unique source manifests")
    _run_legacy_worker(legacy_python, ["verify", "--report", str(report_path),
                                      "--dataset", str(report["dataset_path"]),
                                      "--dataset-tree-sha256", str(report["dataset_tree_sha256"])],
                       runtime_root)
    return report, sha256(report_path)


def _jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _hash_f32(values) -> str:
    import numpy as np
    return hashlib.sha256(np.asarray(values, dtype="<f4").tobytes(order="C")).hexdigest()


def prepare_proxy_from_feedback(source: str | Path, output: str | Path, *, repo_id: str,
                                legacy_python: str | Path | None = None) -> dict:
    """Run source normalization, the real old proxy pipeline, and v3 migration.

    The legacy phase calls the repository's existing post-capture pipeline in
    the pinned Python 3.10/LeRobot 0.1 environment. Its `prepare` phase performs
    `convert_many` and strict inspection; it never starts training.
    """
    import numpy as np
    from PIL import Image
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from .feedback_legacy_bridge import materialize_legacy_inputs

    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    if output.exists() or output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError("output must be a new separate directory from source")
    info_path = source / "meta/info.json"
    if not info_path.is_file():
        raise FileNotFoundError("source must be a LeRobot v2.1 dataset")
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if info.get("codebase_version") != "v2.1" or info.get("fps") != 10:
        raise ValueError("feedback route requires a LeRobot v2.1 source at 10 Hz")
    provenance = source / "provenance"
    required = {name: provenance / name for name in
                ("source_map.jsonl", "run_sources.jsonl", "run_labels.jsonl")}
    if any(not path.is_file() for path in required.values()):
        raise ValueError("feedback route requires source_map, run_sources, and run_labels")
    source_rows, run_sources, run_labels = (_jsonl(required[name]) for name in required)
    key = lambda row: (str(row.get("run_id", "")), row.get("episode_index"))
    source_by_key = {key(row): row for row in run_sources}
    label_by_key = {key(row): row for row in run_labels}
    episode_meta = _jsonl(source / "meta/episodes.jsonl")
    rows_by_key: dict[tuple[str, int], list[dict]] = {}
    for row in source_rows:
        rows_by_key.setdefault(key(row), []).append(row)
    checks, accepted = [], []
    for meta in episode_meta:
        episode_index = int(meta["episode_index"])
        candidates = [(k, v) for k, v in rows_by_key.items() if k[1] == episode_index]
        if len(candidates) != 1:
            checks.append({"episode_index": episode_index,
                           "accepted_as_complete_success_source": False,
                           "reasons": ["source_map_episode_identity_missing_or_ambiguous"]})
            continue
        episode_key, episode_rows = candidates[0]
        episode_rows = sorted(episode_rows, key=lambda row: int(row.get("episode_frame_index", -1)))
        source_record = source_by_key.get(episode_key, {})
        label = label_by_key.get(episode_key, source_record.get("label", {}))
        check = inspect_feedback_episode(episode_rows, source_record, label)
        checks.append(check)
        if check["accepted_as_complete_success_source"]:
            accepted.append({"key": episode_key, "rows": episode_rows, "meta": meta,
                             "source": source_record, "label": label})
    source_hashes = {name: sha256(path) for name, path in required.items()}
    if not accepted:
        return {"schema": "piper_feedback_proxy_prepare_v1", "status": "NO_COMPLETE_SUCCESS_EPISODES",
                "source_path": str(source), "source_sidecar_sha256": source_hashes,
                "episode_checks": checks, "output_written": False,
                "training_eligible": False, "training_started": False}
    if len(accepted) < MIN_FEEDBACK_EPISODES_FOR_FORMAL_ADMISSION:
        return {"schema": "piper_feedback_proxy_prepare_v1",
                "status": "BLOCKED_REQUIRES_FIVE_INDEPENDENT_COMPLETE_SUCCESS_EPISODES",
                "source_path": str(source), "accepted_episodes": len(accepted),
                "required_episodes": MIN_FEEDBACK_EPISODES_FOR_FORMAL_ADMISSION,
                "episode_checks": checks, "output_written": False,
                "training_eligible": False, "training_started": False}
    if legacy_python is None:
        raise ValueError("full feedback route requires --legacy-python for the pinned old runtime")
    stage = output.with_name(output.name + ".legacy_stage")
    if stage.exists():
        raise FileExistsError(f"legacy stage already exists: {stage}")
    stage.mkdir(parents=True)
    adapter = materialize_legacy_inputs(source, info, accepted, required, stage / "source_adapter")
    binding_path = adapter["source_adapter_manifest"]
    binding_sha = sha256(binding_path)
    run_id = "feedback_a_" + binding_sha[:16]
    legacy_report_path = stage / "legacy_validation_report.json"
    worker_args = ["prepare", "--raw-root", str(adapter["raw_root"]),
                   "--outcome-root", str(adapter["outcome_root"]),
                   "--derived-root", str(stage / "derived"),
                   "--dataset-home", str(stage / "legacy_hf"),
                   "--source-adapter-manifest", str(binding_path),
                   "--report", str(legacy_report_path), "--run-id", run_id,
                   "--repo-id", repo_id]
    worker_result = _run_legacy_worker(legacy_python, worker_args, stage / "legacy_runtime")
    legacy_report, legacy_report_sha = _validate_legacy_proxy_report(
        legacy_report_path, accepted, legacy_python=legacy_python,
        runtime_root=stage / "legacy_runtime")
    legacy_root = Path(legacy_report["dataset_path"]).resolve(strict=True)
    conversion = json.loads((legacy_root / "provenance/proxy_conversion.json").read_text(encoding="utf-8"))
    legacy_source_map = _jsonl(legacy_root / "provenance/source_map.jsonl")
    adapter_by_raw = {episode["legacy_raw_episode_id"]: episode
                      for episode in adapter["binding"]["episodes"]}
    accepted_by_key = {(item["key"][0], item["key"][1]): item for item in accepted}
    if len(adapter_by_raw) != len(accepted) or len(set(adapter_by_raw)) != len(adapter_by_raw):
        raise ValueError("source adapter contains duplicate raw episode IDs")

    output.parent.mkdir(parents=True, exist_ok=True)
    features = {"observation.state": {"dtype": "float32", "shape": (7,), "names": JOINTS},
                "action": {"dtype": "float32", "shape": (7,), "names": JOINTS}}
    source_map = []
    timing_ms = []
    window_rows = []
    target_episode_origins = {}
    for episode_index, legacy_episode in enumerate(conversion["source_episodes"]):
        raw_id = legacy_episode["raw_episode_id"]
        adapter_episode = adapter_by_raw.get(raw_id)
        if adapter_episode is None:
            raise ValueError("legacy converter emitted an episode outside source adapter")
        source_item = accepted_by_key[(adapter_episode["source_run_id"],
                                       adapter_episode["source_episode_index"])]
        preview = stage / "derived" / run_id / "previews" / raw_id
        proxy = stage / "derived" / run_id / "targets" / raw_id
        preview_rows = _jsonl(preview / "frames.jsonl")
        proxy_rows = _jsonl(proxy / "proxy_targets.jsonl")
        frame_lookup = {entry["source_feedback_timestamp_ns"]: entry
                        for entry in adapter_episode["frame_bindings"]}
        if len(frame_lookup) != len(adapter_episode["frame_bindings"]):
            raise ValueError("source adapter contains duplicate feedback timestamps")
        source_map_episode = sorted(
            (row for row in legacy_source_map if row.get("raw_episode_id") == raw_id),
            key=lambda row: row["dataset_row_index"])
        if len(source_map_episode) != len(proxy_rows) or len(proxy_rows) < 16:
            raise ValueError("legacy converter rows do not form a full feedback horizon")
        state_by_source_timestamp = {row["state"]["source_local_monotonic_ns"]: row
                                    for row in source_item["rows"]}
        for frame_index, (mapped, proxy_row) in enumerate(zip(source_map_episode, proxy_rows)):
            obs_frame_index = mapped["observation"]["aligned_frame_index"]
            target_frame_index = mapped["proxy_target"]["aligned_frame_index"]
            current_frame, target_frame = preview_rows[obs_frame_index], preview_rows[target_frame_index]
            current_timestamp = current_frame["source_refs"]["joint_12_timestamp_ns"]
            target_timestamp = target_frame["source_refs"]["joint_12_timestamp_ns"]
            current_binding, target_binding = frame_lookup[current_timestamp], frame_lookup[target_timestamp]
            if target_binding["source_frame_index"] != current_binding["source_frame_index"] + 1:
                raise ValueError("legacy alignment skipped/duplicated feedback; refusing proxy row")
            current_source = state_by_source_timestamp.get(current_binding["source_feedback_timestamp_ns"])
            target_source = state_by_source_timestamp.get(target_binding["source_feedback_timestamp_ns"])
            if current_source is None or target_source is None:
                raise ValueError("legacy feedback timestamps do not map to original source rows")
            state = np.asarray(proxy_row["state"], dtype=np.float32)
            target = np.asarray(proxy_row["proxy_action"], dtype=np.float32)
            if (_hash_f32(state) != mapped["observation"]["state_float32_sha256"]
                    or _hash_f32(target) != mapped["proxy_target"]["action_float32_sha256"]
                    or not np.array_equal(state, np.asarray(current_source["state"]["values_float32"], dtype=np.float32))
                    or not np.array_equal(target, np.asarray(target_source["state"]["values_float32"], dtype=np.float32))):
                raise ValueError("legacy proxy values do not exactly match adjacent source feedback rows")
            images = {}
            for camera, role in ((CAMERAS[0], "base"), (CAMERAS[1], "wrist")):
                image_path = preview / current_frame[f"{role}_image"]
                with Image.open(image_path) as image:
                    pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
                if camera not in features:
                    features[camera] = {"dtype": "image", "shape": pixels.shape,
                                        "names": ["height", "width", "channels"]}
                elif tuple(features[camera]["shape"]) != pixels.shape:
                    raise ValueError("source camera resolution changed within feedback set")
                images[camera] = pixels
            ds_frame = {"observation.state": state, "action": target,
                        **images, "task": source_item["meta"]["tasks"][0]}
            if episode_index == 0 and frame_index == 0:
                ds = LeRobotDataset.create(repo_id=repo_id, root=output, fps=10,
                                           features=features, use_videos=False, robot_type="piper")
            ds.add_frame(ds_frame)
            dt_ms = ((target_binding["source_feedback_timestamp_ns"]
                      - current_binding["source_feedback_timestamp_ns"]) / 1e6)
            timing_ms.append(dt_ms)
            source_map.append({
                "dataset_row_index": len(source_map), "episode_index": episode_index,
                "episode_frame_index": frame_index, "run_id": source_item["key"][0],
                "source_origin_sha256": source_item["source"]["manifest_sha256"],
                "action_semantics": "next_observed_state_proxy_nominal_units",
                "action_valid": True,
                "source_action_semantics": current_source.get("action_semantics"),
                "source_action_valid": current_source.get("action_valid"),
                "source_action_invalid_reason": current_source.get("action_invalid_reason"),
                "observation": {"source_episode_index": source_item["key"][1],
                                "source_frame_index": current_binding["source_frame_index"],
                                "timestamp_ns": current_binding["source_feedback_timestamp_ns"],
                                "source_map_row": current_source,
                                "legacy_source_map_row": mapped,
                                "state_float32_sha256": _hash_f32(state)},
                "proxy_target": {"source_episode_index": source_item["key"][1],
                                 "source_frame_index": target_binding["source_frame_index"],
                                 "timestamp_ns": target_binding["source_feedback_timestamp_ns"],
                                 "feedback_source": target_source["state"],
                                 "action_float32_sha256": _hash_f32(target)},
                "legacy_raw_episode_id": raw_id,
                "legacy_raw_manifest_sha256": legacy_episode["raw_manifest_sha256"],
            })
            window_rows.append({"episode_index": episode_index, "timestamp": frame_index / 10,
                                "action_valid": True, "observation_valid": True,
                                "base_valid": True, "wrist_valid": True})
        ds.save_episode()
        target_episode_origins[str(episode_index)] = source_item["source"]["manifest_sha256"]
    if output.exists() and not (output / "meta/info.json").is_file():
        raise FileExistsError(output)
    ds.finalize()
    out_provenance = output / "provenance"
    out_provenance.mkdir(exist_ok=True)
    for path, name in ((legacy_report_path, "legacy_validation_report.json"),
                       (binding_path, "source_adapter_manifest.json"),
                       (legacy_root / "provenance/proxy_conversion.json", "legacy_proxy_conversion.json"),
                       (legacy_root / "provenance/source_map.jsonl", "legacy_source_map.jsonl"),
                       (legacy_root / "provenance/readback_audit.json", "legacy_readback_audit.json"),
                       (legacy_root / "provenance/readback_frames.jsonl", "legacy_readback_frames.jsonl")):
        shutil.copy2(path, out_provenance / name)
    (out_provenance / "source_map.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
                for row in source_map), encoding="utf-8")
    from .feedback_legacy_bridge import sha256 as bridge_sha256
    legacy_report_sha = bridge_sha256(out_provenance / "legacy_validation_report.json")
    timing_report = {
        "grid_target_offset_ms": 100.0, "raw_feedback_pair_dt_ms_min": min(timing_ms),
        "raw_feedback_pair_dt_ms_p50": float(np.percentile(timing_ms, 50)),
        "raw_feedback_pair_dt_ms_p95": float(np.percentile(timing_ms, 95)),
        "raw_feedback_pair_dt_ms_max": max(timing_ms),
        "raw_feedback_pairs_outside_100ms_plus_minus_5ms_diagnostic_only": sum(
            abs(value - 100.0) > 5.0 for value in timing_ms),
        "raw_source_timestamps_preserved_per_row": True,
        "source_timestamp_tolerance_admission_gate": None,
    }
    source_hashes = {name: sha256(path) for name, path in required.items()}
    is_synthetic = all(item["source"].get("software_fixture") is True for item in accepted)
    training_manifest = {
        "schema": "piper_lerobot_training_v1",
        "source_kind": "synthetic_feedback_proxy" if is_synthetic else "verified_feedback_proxy",
        "training_eligible": not is_synthetic,
        "software_training_eligible": is_synthetic,
        "field_admission_status": "BLOCKED", "field_admission_evidence": None,
        "field_admission_evidence_sha256": None,
        "legacy_validation_evidence": "provenance/legacy_validation_report.json",
        "legacy_validation_evidence_sha256": legacy_report_sha,
        "legacy_validated_dataset_sha256": legacy_report["strict_inspect"]["dataset_sha256"],
        "legacy_dataset_tree_sha256": legacy_report["dataset_tree_sha256"],
        "legacy_python_executable": str(Path(legacy_python).resolve()),
        "source_adapter_manifest_sha256": legacy_report["source_adapter_manifest_sha256"],
        "claim": "offline next-feedback proxy; software qualification only, physical execution and units remain unverified",
        "invalid_rows": 0,
        "contract": {**contract(), "units": "radian_nominal_unverified_and_meter_nominal_unverified"},
        "splits": {"train": list(range(len(accepted) - 1)), "validation": [len(accepted) - 1]},
        "episode_origins": target_episode_origins,
        "strict_full_window_starts": valid_window_starts(window_rows),
        "feedback_target_timing": timing_report,
        "source_kind_detail": "v2.1 live-writer feedback normalized through legacy raw/proxy/converter/inspect and migrated to v3",
        "action_semantics": "next_observed_state_proxy_nominal_units",
        "action_target_kind": "derived_future_observed_state_position_target",
        "action_time_relation": "feedback_state_at_t_to_next_feedback_state_on_10hz_grid_t_plus_100ms",
        "action_horizon": 16, "first_target_dt_s": 0.1,
        "raw_command_supervision": False, "formal_training_eligible": False,
        "hardware_control_allowed": False, "robot_execution_verified": False,
        "offline_training_admission_status": "SOFTWARE_ONLY_PASS" if is_synthetic else "OFFLINE_DERIVED_STATE_PASS",
        "source_sidecar_sha256": source_hashes,
        "source_episodes": [{"run_id": item["key"][0], "episode_index": item["key"][1],
                             "manifest_sha256": item["source"].get("manifest_sha256"),
                             "bag_db_sha256": item["source"].get("bag_db_sha256"),
                             "label": item["label"],
                             "source_training_eligible_as_recorded": item["source"].get("formal_training_eligible"),
                             "label_training_eligible_as_recorded": item["label"].get("training_eligible")}
                            for item in accepted],
        "episode_checks": checks,
        "dataset_files": fingerprint(output),
    }
    write_json(output / MANIFEST, training_manifest)
    admit_dataset(output, software_only=is_synthetic)
    write_json(out_provenance / "feedback_proxy_report.json", {
        "schema": "piper_feedback_proxy_prepare_v1", "status": "LEGACY_OFFLINE_PROXY_GATE_PASS_V3_MIGRATED",
        "source_path": str(source), "output_path": str(output),
        "source_sidecar_sha256": source_hashes, "episode_checks": checks,
        "episodes_written": len(accepted), "frames_written": len(source_map),
        "strict_full_H16_starts": training_manifest["strict_full_window_starts"],
        "feedback_target_timing": timing_report,
        "training_eligible": not is_synthetic,
        "software_training_eligible": is_synthetic, "training_started": False,
        "legacy_worker": worker_result, "legacy_strict_inspect": legacy_report["strict_inspect"],
        "formal_training_eligible": False, "hardware_control_allowed": False,
        "robot_execution_verified": False,
    })
    return {"schema": "piper_feedback_proxy_prepare_v1",
            "status": "LEGACY_OFFLINE_PROXY_GATE_PASS_V3_MIGRATED",
            "output": str(output), "episodes": len(accepted), "frames": len(source_map),
            "strict_full_H16_starts": training_manifest["strict_full_window_starts"],
            "episode_checks": checks, "training_eligible": not is_synthetic,
            "software_training_eligible": is_synthetic,
            "training_started": False, "formal_training_eligible": False,
            "hardware_control_allowed": False}
