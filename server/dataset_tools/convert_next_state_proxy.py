"""Convert a checksum-bound next-state proxy to local LeRobot, offline only.

This is deliberately separate from the reviewed-action converter. Its output
must never be accepted by the formal PiPER training or hardware-control gates.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import re
import uuid

import numpy as np
from PIL import Image

DEFAULT_TASK = "Pick up the black rod and place it on the green desk mat."
TASK = DEFAULT_TASK
EXPECTED_DATASETS_VERSION = "3.6.0"
EXPECTED_LEROBOT_REVISION = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _file(root: Path, name: str) -> Path:
    if not isinstance(name, str) or Path(name).is_absolute():
        raise ValueError("image path must be relative")
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f"missing or escaping image: {name}")
    return path


def _seven(value: object) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.shape != (7,) or not np.isfinite(array).all():
        raise ValueError("state/proxy action must be finite 7D")
    return array


def _task(value: str | None) -> str:
    selected = DEFAULT_TASK if value is None else value.strip()
    if not selected or len(selected) > 512:
        raise ValueError("task must be a non-empty string of at most 512 characters")
    return selected


def _rgb_sha256(path: Path) -> str:
    with Image.open(path) as image:
        pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
    return hashlib.sha256(pixels.tobytes(order="C")).hexdigest()


def _jsonl_bytes(rows: list[dict]) -> bytes:
    return b"".join(
        (json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
        for row in rows
    )


def _numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _readback_image(value) -> np.ndarray:
    image = _numpy(value)
    if image.ndim != 3:
        raise ValueError("LeRobot readback image is not three-dimensional")
    if image.shape[0] == 3 and image.shape[-1] != 3:
        image = np.moveaxis(image, 0, -1)
    if image.shape[-1] != 3 or not np.isfinite(image).all():
        raise ValueError("LeRobot readback image is not finite RGB")
    if image.dtype != np.uint8:
        if np.issubdtype(image.dtype, np.floating) and image.size and image.max() <= 1.0:
            image = image * 255.0
        image = np.clip(image, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(image)


def _scalar(value, name: str) -> int:
    array = _numpy(value)
    if array.shape != ():
        raise ValueError(f"LeRobot {name} must be scalar")
    return int(array.item())


def _audit_multi_readback(dataset_root: Path, repo_id: str, checked: list[dict],
                          source_map: list[dict], *, dataset_name: str | None = None) -> dict:
    """Read every converted frame back; bind parquet values and video rows."""
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    info_path = dataset_root / "meta" / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    expected_count = sum(len(item["rows"]) for item in checked)
    if (info.get("robot_type") != "piper"
            or info.get("fps") != 10
            or info.get("total_episodes") != len(checked)
            or info.get("total_frames") != expected_count):
        raise ValueError("LeRobot metadata does not match the converted demonstration batch")
    dataset = LeRobotDataset(repo_id, root=dataset_root)
    if len(dataset) != expected_count:
        raise ValueError("LeRobot readback length differs from the conversion input")

    readback_rows = []
    pixel_errors = {"base": [], "wrist": []}
    for global_index, source_row in enumerate(source_map):
        item = dataset[global_index]
        episode_index = source_row["episode_index"]
        episode_frame_index = source_row["episode_frame_index"]
        if (_scalar(item["index"], "index") != global_index
                or _scalar(item["episode_index"], "episode_index") != episode_index
                or _scalar(item["frame_index"], "frame_index") != episode_frame_index):
            raise ValueError(f"LeRobot episode/frame order mismatch at row {global_index}")
        state = np.asarray(_numpy(item["observation.state"]), dtype=np.float32)
        action = np.asarray(_numpy(item["action"]), dtype=np.float32)
        state_hash = hashlib.sha256(state.astype("<f4", copy=False).tobytes(order="C")).hexdigest()
        action_hash = hashlib.sha256(action.astype("<f4", copy=False).tobytes(order="C")).hexdigest()
        if (state.shape != (7,) or action.shape != (7,)
                or not np.array_equal(state, np.asarray(item["observation.state"], dtype=np.float32))
                or state_hash != source_row["observation"]["state_float32_sha256"]
                or action_hash != source_row["proxy_target"]["action_float32_sha256"]):
            raise ValueError(f"LeRobot state/action readback differs from source at row {global_index}")
        row = {"dataset_row_index": global_index, "episode_index": episode_index,
               "episode_frame_index": episode_frame_index,
               "raw_episode_id": source_row["raw_episode_id"],
               "state_float32_sha256": state_hash,
               "action_float32_sha256": action_hash}
        for role, key in (("base", "observation.images.image"),
                          ("wrist", "observation.images.wrist_image")):
            decoded = _readback_image(item[key])
            source_path = _file(checked[episode_index]["preview"],
                                source_row["observation"]["images"][role]["preview_relpath"])
            with Image.open(source_path) as image:
                source_pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
            if decoded.shape != source_pixels.shape:
                raise ValueError(f"LeRobot {role} readback shape mismatch at row {global_index}")
            error = float(np.abs(decoded.astype(np.int16) - source_pixels.astype(np.int16)).mean())
            pixel_errors[role].append(error)
            decoded_sha = hashlib.sha256(decoded.tobytes(order="C")).hexdigest()
            row[f"{role}_source_rgb_sha256"] = source_row["observation"]["images"][role]["rgb_pixels_sha256"]
            row[f"{role}_decoded_rgb_sha256"] = decoded_sha
            row[f"{role}_mean_absolute_pixel_error"] = error
        readback_rows.append(row)

    readback_bytes = _jsonl_bytes(readback_rows)
    source_map_path = dataset_root / "provenance" / "source_map.jsonl"
    source_map_bytes = source_map_path.read_bytes()
    if source_map_bytes != _jsonl_bytes(source_map):
        raise ValueError("multi-episode source map changed before readback")
    provenance = dataset_root / "provenance"
    (provenance / "readback_frames.jsonl").write_bytes(readback_bytes)
    file_hashes = {}
    for file in sorted(dataset_root.rglob("*")):
        if file.is_file() and file.suffix in (".mp4", ".parquet"):
            file_hashes[file.relative_to(dataset_root).as_posix()] = sha256(file)
    report = {
        "schema": "piper_proxy_lerobot_multi_readback_audit_v1",
        "status": "PASS_READBACK",
        "dataset_path": dataset_name or dataset_root.name,
        "dataset_repo_id": repo_id,
        "fps": info["fps"],
        "episodes": info["total_episodes"],
        "frames": info["total_frames"],
        "split": "train_only",
        "action_semantics": "next_observed_state_proxy_nominal_units",
        "source_frame_extraction_verified": True,
        "state_values_verified": True,
        "action_values_verified": True,
        "episode_and_frame_order_verified": True,
        "decoded_rgb_frames_verified": True,
        "source_map_sha256": hashlib.sha256(source_map_bytes).hexdigest(),
        "readback_frames_path": "provenance/readback_frames.jsonl",
        "readback_frames_sha256": hashlib.sha256(readback_bytes).hexdigest(),
        "dataset_info_sha256": sha256(info_path),
        "media_and_parquet_sha256": file_hashes,
        "camera_pixel_error_mean_absolute": {
            role: float(np.mean(values)) if values else None
            for role, values in pixel_errors.items()
        },
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    audit_path = provenance / "readback_audit.json"
    audit_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def _conversion_runtime() -> dict:
    """Reject converter runtimes that produce artifacts the OpenPI lock can't read."""
    import datasets

    datasets_version = getattr(datasets, "__version__", None)
    if datasets_version != EXPECTED_DATASETS_VERSION:
        raise RuntimeError(
            f"datasets=={EXPECTED_DATASETS_VERSION} is required by the pinned OpenPI stack; "
            f"found {datasets_version or 'unknown'}"
        )
    try:
        distribution = importlib.metadata.distribution("lerobot")
        direct_url = json.loads(distribution.read_text("direct_url.json") or "{}")
    except (importlib.metadata.PackageNotFoundError, json.JSONDecodeError) as exc:
        raise RuntimeError("cannot verify the pinned LeRobot installation") from exc
    revision = direct_url.get("vcs_info", {}).get("commit_id")
    if revision != EXPECTED_LEROBOT_REVISION:
        raise RuntimeError(
            f"LeRobot revision {EXPECTED_LEROBOT_REVISION} is required; found {revision!r}"
        )
    return {
        "datasets_version": datasets_version,
        "lerobot_version": distribution.version,
        "lerobot_revision": revision,
    }


def inspect(proxy: Path, preview: Path) -> tuple[dict, list[dict], dict]:
    proxy, preview = proxy.resolve(), preview.resolve()
    manifest = json.loads((proxy / "proxy_manifest.json").read_text(encoding="utf-8"))
    if (manifest.get("schema") != "piper_next_state_proxy_v1"
            or manifest.get("semantics") != "next_observed_state_proxy_nominal_units"
            or manifest.get("formal_training_eligible") is not False
            or manifest.get("hardware_control_allowed") is not False
            or manifest.get("preview_episode_id") != preview.name):
        raise ValueError("not an offline-only next-state proxy")
    expected = {
        "proxy_targets_sha256": proxy / "proxy_targets.jsonl",
        "preview_metadata_sha256": preview / "aligned_episode.json",
        "preview_frames_sha256": preview / "frames.jsonl",
    }
    if any(sha256(path) != manifest.get(key) for key, path in expected.items()):
        raise ValueError("proxy or preview checksum mismatch")
    metadata = json.loads(expected["preview_metadata_sha256"].read_text(encoding="utf-8"))
    if (metadata.get("raw_manifest_sha256") != manifest.get("raw_manifest_sha256")
            or metadata.get("source") not in ("manual_teach", "operator_demonstration")
            or metadata.get("fps") != manifest.get("fps")):
        raise ValueError("preview identity mismatch")
    source = [json.loads(line) for line in expected["preview_frames_sha256"].read_text(encoding="utf-8").splitlines()]
    rows = [json.loads(line) for line in expected["proxy_targets_sha256"].read_text(encoding="utf-8").splitlines()]
    if len(rows) < 2 or len(rows) != manifest.get("samples") or len(source) != len(rows) + 1:
        raise ValueError("proxy length mismatch")
    shapes = {}
    for i, row in enumerate(rows):
        if (row.get("frame_index") != i or row.get("target_source_frame_index") != i + 1
                or row.get("observation_timestamp_ns") != source[i].get("policy_timestamp_ns")
                or row.get("target_timestamp_ns") != source[i + 1].get("policy_timestamp_ns")
                or row["target_timestamp_ns"] - row["observation_timestamp_ns"] != manifest.get("target_offset_ns")):
            raise ValueError(f"proxy time/source mismatch at {i}")
        if row.get("base_image") != source[i].get("base_image") or row.get("wrist_image") != source[i].get("wrist_image"):
            raise ValueError(f"proxy image reference mismatch at {i}")
        for channel, key in (("base", "base_image"), ("wrist", "wrist_image")):
            with Image.open(_file(preview, row[key])) as image:
                if image.mode != "RGB":
                    raise ValueError("image is not RGB")
                shape = np.asarray(image).shape
            if channel in shapes and shapes[channel] != shape:
                raise ValueError("camera resolution changed")
            shapes[channel] = shape
        _seven(row["state"])
        _seven(row["proxy_action"])
        if i + 1 < len(rows) and not np.allclose(row["proxy_action"], rows[i + 1]["state"], rtol=0, atol=1e-7):
            raise ValueError(f"proxy is not the next observed state at {i}")
    return manifest, rows, shapes


def build_source_map(proxy: Path, preview: Path, *, episode_index: int = 0,
                     dataset_row_offset: int = 0,
                     include_episode_identity: bool = False) -> tuple[list[dict], dict]:
    """Bind every future LeRobot row to its observation and target source rows."""
    proxy, preview = proxy.resolve(), preview.resolve()
    manifest, rows, _ = inspect(proxy, preview)
    preview_manifest = json.loads((preview / "aligned_episode.json").read_text(encoding="utf-8"))
    preview_rows = [
        json.loads(line)
        for line in (preview / "frames.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    if len(preview_rows) != len(rows) + 1:
        raise ValueError("source map needs one aligned target row after every proxy observation")

    source_map = []
    for index, row in enumerate(rows):
        observation, target = preview_rows[index], preview_rows[index + 1]
        if (observation.get("frame_index") != index
                or target.get("frame_index") != row.get("target_source_frame_index")
                or observation.get("policy_timestamp_ns") != row.get("observation_timestamp_ns")
                or target.get("policy_timestamp_ns") != row.get("target_timestamp_ns")):
            raise ValueError(f"source map row/timestamp mismatch at dataset row {index}")
        images = {}
        target_images = {}
        for role, image_field, frame_field in (
            ("base", "base_image", "base_frame_id"),
            ("wrist", "wrist_image", "wrist_frame_id"),
        ):
            observation_path = _file(preview, row[image_field])
            target_path = _file(preview, target[image_field])
            images[role] = {
                "preview_relpath": row[image_field],
                "rgb_pixels_sha256": _rgb_sha256(observation_path),
                "raw_video_frame_id": observation.get("source_refs", {}).get(frame_field),
            }
            target_images[role] = {
                "preview_relpath": target[image_field],
                "rgb_pixels_sha256": _rgb_sha256(target_path),
                "raw_video_frame_id": target.get("source_refs", {}).get(frame_field),
            }
        observation_state = _seven(row["state"])
        proxy_action = _seven(row["proxy_action"])
        mapped_row = {
            "dataset_row_index": dataset_row_offset + index,
            "episode_index": episode_index,
            "task_index": 0,
            "action_semantics": manifest["semantics"],
            "observation": {
                "aligned_frame_index": observation["frame_index"],
                "timestamp_ns": row["observation_timestamp_ns"],
                "source_refs": observation.get("source_refs", {}),
                "images": images,
                "state_float32_sha256": hashlib.sha256(
                    np.asarray(observation_state, dtype="<f4").tobytes(order="C")
                ).hexdigest(),
            },
            "proxy_target": {
                "aligned_frame_index": target["frame_index"],
                "timestamp_ns": row["target_timestamp_ns"],
                "source_refs": target.get("source_refs", {}),
                "images": target_images,
                "action_float32_sha256": hashlib.sha256(
                    np.asarray(proxy_action, dtype="<f4").tobytes(order="C")
                ).hexdigest(),
            },
        }
        if include_episode_identity:
            mapped_row["episode_frame_index"] = index
            mapped_row["raw_episode_id"] = manifest["raw_episode_id"]
        source_map.append(mapped_row)

    map_bytes = _jsonl_bytes(source_map)
    report = {
        "schema": "piper_proxy_lerobot_source_map_v1",
        "raw_episode_id": manifest["raw_episode_id"],
        "raw_manifest_sha256": manifest["raw_manifest_sha256"],
        "aligned_preview_episode_id": preview_manifest["episode_id"],
        "aligned_metadata_sha256": sha256(preview / "aligned_episode.json"),
        "aligned_frames_sha256": sha256(preview / "frames.jsonl"),
        "frame_map_sha256": preview_manifest["frame_map_sha256"],
        "proxy_manifest_sha256": sha256(proxy / "proxy_manifest.json"),
        "proxy_targets_sha256": sha256(proxy / "proxy_targets.jsonl"),
        "source_map_path": "provenance/source_map.jsonl",
        "source_map_rows": len(source_map),
        "source_map_sha256": hashlib.sha256(map_bytes).hexdigest(),
        "action_semantics": manifest["semantics"],
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    return source_map, report


def convert_many(episodes: list[tuple[Path, Path]], output: Path, repo_id: str,
                 *, task: str | None = None) -> dict:
    """Write a new 10fps train-only LeRobot dataset from successful demos.

    Each tuple is ``(proxy_directory, aligned_preview_directory)``. The raw
    and preview sources remain untouched. This is an offline derived-state
    pilot, not raw-command supervision or hardware authorization. A single
    verified demo is accepted for the first tiny-fit data-pipeline check.
    """
    if not episodes:
        raise ValueError("derived-state conversion requires at least one demonstration")
    selected_task = _task(task)
    checked: list[dict] = []
    seen_ids: set[str] = set()
    common_fps: int | None = None
    common_shapes: dict[str, tuple[int, ...]] | None = None
    for proxy, preview in episodes:
        proxy, preview = Path(proxy).resolve(), Path(preview).resolve()
        manifest, rows, shapes = inspect(proxy, preview)
        if manifest.get("fps") != 10:
            raise ValueError("the multi-demonstration pilot reuses the established 10fps route")
        if manifest.get("outcome_success") is not True:
            raise ValueError("every pilot training episode must carry an operator-verified success label")
        raw_episode_id = manifest.get("raw_episode_id")
        if not isinstance(raw_episode_id, str) or not raw_episode_id or raw_episode_id in seen_ids:
            raise ValueError("raw demonstration IDs must be present and unique")
        if common_fps is None:
            common_fps, common_shapes = manifest["fps"], shapes
        elif manifest["fps"] != common_fps or shapes != common_shapes:
            raise ValueError("all demonstrations must share fps and both camera resolutions")
        mapped, source_report = build_source_map(
            proxy, preview, episode_index=len(checked),
            dataset_row_offset=sum(len(item["rows"]) for item in checked),
            include_episode_identity=True,
        )
        checked.append({"proxy": proxy, "preview": preview, "manifest": manifest,
                        "rows": rows, "source_map": mapped, "source_report": source_report})
        seen_ids.add(raw_episode_id)

    output = Path(output).resolve()
    if not re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_-]+", repo_id):
        raise ValueError("repo_id must have namespace/name form")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing LeRobot dataset: {output}")
    for item in checked:
        if (output.is_relative_to(item["proxy"]) or output.is_relative_to(item["preview"])
                or item["proxy"].is_relative_to(output) or item["preview"].is_relative_to(output)):
            raise ValueError("dataset output and source episode paths must be separate")

    conversion_runtime = _conversion_runtime()
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    assert common_shapes is not None and common_fps is not None
    features = {
        "observation.images.image": {"dtype": "video", "shape": common_shapes["base"],
                                     "names": ["height", "width", "channel"]},
        "observation.images.wrist_image": {"dtype": "video", "shape": common_shapes["wrist"],
                                            "names": ["height", "width", "channel"]},
        "observation.state": {"dtype": "float32", "shape": (7,), "names": ["state"]},
        "action": {"dtype": "float32", "shape": (7,), "names": ["action"]},
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_name(output.name + ".INCOMPLETE-" + uuid.uuid4().hex[:12])
    dataset = LeRobotDataset.create(repo_id=repo_id, root=pending, robot_type="piper",
                                    fps=common_fps, features=features)
    all_source_map: list[dict] = []
    source_episodes = []
    total_frames = 0
    for episode_index, item in enumerate(checked):
        for row in item["rows"]:
            with Image.open(_file(item["preview"], row["base_image"])) as image:
                base = np.asarray(image.convert("RGB"))
            with Image.open(_file(item["preview"], row["wrist_image"])) as image:
                wrist = np.asarray(image.convert("RGB"))
            dataset.add_frame({"observation.images.image": base,
                               "observation.images.wrist_image": wrist,
                               "observation.state": _seven(row["state"]),
                               "action": _seven(row["proxy_action"]), "task": selected_task})
        dataset.save_episode()
        all_source_map.extend(item["source_map"])
        source_report = item["source_report"]
        source_episodes.append({
            "episode_index": episode_index,
            "raw_episode_id": item["manifest"]["raw_episode_id"],
            "raw_manifest_sha256": item["manifest"]["raw_manifest_sha256"],
            "aligned_preview_episode_id": source_report["aligned_preview_episode_id"],
            "aligned_metadata_sha256": source_report["aligned_metadata_sha256"],
            "aligned_frames_sha256": source_report["aligned_frames_sha256"],
            "frame_map_sha256": source_report["frame_map_sha256"],
            "outcome_label_sha256": item["manifest"].get("outcome_label_sha256"),
            "proxy_manifest_sha256": source_report["proxy_manifest_sha256"],
            "proxy_targets_sha256": source_report["proxy_targets_sha256"],
            "frames": len(item["rows"]),
            "outcome_success": True,
            "split": "train",
        })
        total_frames += len(item["rows"])

    provenance = pending / "provenance"
    provenance.mkdir()
    map_bytes = _jsonl_bytes(all_source_map)
    map_sha = hashlib.sha256(map_bytes).hexdigest()
    map_path = provenance / "source_map.jsonl"
    map_path.write_bytes(map_bytes)
    map_report = {
        "schema": "piper_proxy_lerobot_multi_source_map_v1",
        "source_episodes": source_episodes,
        "source_map_path": "provenance/source_map.jsonl",
        "source_map_rows": len(all_source_map),
        "source_map_sha256": map_sha,
        "action_semantics": "next_observed_state_proxy_nominal_units",
        "task": selected_task,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    (provenance / "source_map_report.json").write_text(
        json.dumps(map_report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    conversion = {
        "schema": "piper_proxy_lerobot_multi_episode_v1",
        "repo_id": repo_id,
        "fps": common_fps,
        "frames": total_frames,
        "episodes": len(source_episodes),
        "split": "train_only",
        "source_episodes": source_episodes,
        "source_map_path": "provenance/source_map.jsonl",
        "source_map_rows": len(all_source_map),
        "source_map_sha256": map_sha,
        "conversion_runtime": conversion_runtime,
        "action_semantics": "next_observed_state_proxy_nominal_units",
        "action_target_kind": "derived_future_observed_state_position_target",
        "action_time_relation": "observation_state_at_t_to_observed_state_at_t_plus_delta",
        "raw_command_supervision": False,
        "joint_units": "radian_nominal_unverified",
        "gripper_units": "meter_nominal_unverified",
        "source_frame_extraction_verified": True,
        "outcome_labels_verified": True,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
        "prediction_only": True,
    }
    (provenance / "proxy_conversion.json").write_text(
        json.dumps(conversion, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    readback_audit = _audit_multi_readback(
        pending, repo_id, checked, all_source_map, dataset_name=output.name)
    pending.rename(output)
    return {"status": "PROXY_MULTI_EPISODE_DATASET_OFFLINE_ONLY", "output": str(output),
            "repo_id": repo_id, "episodes": len(source_episodes), "frames": total_frames,
            "fps": common_fps, "source_map_sha256": map_sha,
            "source_episode_ids": [item["raw_episode_id"] for item in source_episodes],
            "action_semantics": conversion["action_semantics"],
            "readback_audit_status": readback_audit["status"],
            "formal_training_eligible": False, "hardware_control_allowed": False,
            "readback_audit_required_before_training": True}


def convert(proxy: Path, preview: Path, output: Path, repo_id: str,
            *, task: str | None = None) -> dict:
    selected_task = _task(task)
    manifest, rows, shapes = inspect(proxy, preview)
    source_map, source_map_report = build_source_map(proxy, preview)
    conversion_runtime = _conversion_runtime()
    output = output.resolve()
    if (output.exists() or not re.fullmatch(r"[A-Za-z0-9_-]+/[A-Za-z0-9_-]+", repo_id)
            or output.is_relative_to(proxy.resolve()) or output.is_relative_to(preview.resolve())):
        raise ValueError("output must be unused, separate, and have namespace/repo_id")
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    features = {
        "observation.images.image": {"dtype": "video", "shape": shapes["base"], "names": ["height", "width", "channel"]},
        "observation.images.wrist_image": {"dtype": "video", "shape": shapes["wrist"], "names": ["height", "width", "channel"]},
        "observation.state": {"dtype": "float32", "shape": (7,), "names": ["state"]},
        "action": {"dtype": "float32", "shape": (7,), "names": ["action"]},
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_name(output.name + ".INCOMPLETE-" + uuid.uuid4().hex[:12])
    dataset = LeRobotDataset.create(repo_id=repo_id, root=pending, robot_type="piper",
                                    fps=manifest["fps"], features=features)
    for row in rows:
        with Image.open(_file(preview, row["base_image"])) as image:
            base = np.asarray(image.convert("RGB"))
        with Image.open(_file(preview, row["wrist_image"])) as image:
            wrist = np.asarray(image.convert("RGB"))
        dataset.add_frame({"observation.images.image": base,
                           "observation.images.wrist_image": wrist,
                           "observation.state": _seven(row["state"]),
                           "action": _seven(row["proxy_action"]), "task": selected_task})
    dataset.save_episode()
    provenance = pending / "provenance"
    provenance.mkdir()
    source_map_path = provenance / "source_map.jsonl"
    source_map_path.write_bytes(_jsonl_bytes(source_map))
    source_map_report_path = provenance / "source_map_report.json"
    source_map_report_path.write_text(
        json.dumps(source_map_report, indent=2) + "\n", encoding="utf-8"
    )
    result = {"schema": "piper_proxy_lerobot_v1", "repo_id": repo_id,
              "proxy_manifest_sha256": sha256(proxy / "proxy_manifest.json"),
              "proxy_targets_sha256": manifest["proxy_targets_sha256"],
              "frames": len(rows), "episodes": 1, "split": "train_only",
              "source_map_sha256": source_map_report["source_map_sha256"],
              "source_map_rows": len(source_map),
              "conversion_runtime": conversion_runtime,
              "action_semantics": manifest["semantics"],
              "task": selected_task,
              "formal_training_eligible": False, "hardware_control_allowed": False}
    (provenance / "proxy_conversion.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    pending.rename(output)
    return {"status": "PROXY_DATASET_OFFLINE_ONLY", "output": str(output), **result}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proxy", type=Path)
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--episode-manifest", type=Path,
                        help="JSON object with episodes:[{proxy,preview}, ...] for multi-episode conversion")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--task", help="Exact task text copied from the source episode")
    parser.add_argument("--write-derived", action="store_true")
    args = parser.parse_args()
    if args.episode_manifest:
        if args.proxy is not None or args.preview is not None:
            parser.error("use either --episode-manifest or --proxy/--preview")
        batch_path = args.episode_manifest.resolve()
        batch = json.loads(batch_path.read_text(encoding="utf-8"))
        if batch.get("schema") != "piper_next_state_proxy_batch_v1" or not isinstance(batch.get("episodes"), list):
            raise ValueError("invalid multi-episode manifest")
        inputs = []
        for row in batch["episodes"]:
            if not isinstance(row, dict) or not isinstance(row.get("proxy"), str) or not isinstance(row.get("preview"), str):
                raise ValueError("each batch episode must name proxy and preview directories")
            inputs.append(tuple((batch_path.parent / row[key]).resolve() if not Path(row[key]).is_absolute()
                                else Path(row[key]).resolve() for key in ("proxy", "preview")))
        if not args.write_derived:
            summaries = []
            for proxy, preview in inputs:
                manifest, rows, _ = inspect(proxy, preview)
                if manifest.get("outcome_success") is not True:
                    raise ValueError("multi-episode training requires verified success labels")
                summaries.append({"raw_episode_id": manifest["raw_episode_id"],
                                  "fps": manifest["fps"], "frames": len(rows),
                                  "outcome_success": manifest["outcome_success"]})
            print(json.dumps({"status": "PREFLIGHT_PASS_NO_WRITE", "episodes": summaries,
                              "action_semantics": "next_observed_state_proxy_nominal_units",
                              "hardware_control_allowed": False}, indent=2))
        else:
            print(json.dumps(convert_many(inputs, args.output, args.repo_id,
                                         task=args.task), indent=2))
        return
    if args.proxy is None or args.preview is None:
        parser.error("single-episode mode requires --proxy and --preview")
    if not args.write_derived:
        manifest, rows, _ = inspect(args.proxy, args.preview)
        print(json.dumps({"status": "PREFLIGHT_PASS_NO_WRITE", "frames": len(rows),
                          "semantics": manifest["semantics"], "hardware_control_allowed": False}))
    else:
        print(json.dumps(convert(args.proxy, args.preview, args.output, args.repo_id,
                                 task=args.task), indent=2))


if __name__ == "__main__":
    main()
