"""Audit an offline proxy LeRobot dataset and optionally attach row-level provenance."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import uuid

import numpy as np

from server.dataset_tools.convert_next_state_proxy import (
    _file,
    _jsonl_bytes,
    _seven,
    build_source_map,
    EXPECTED_DATASETS_VERSION,
    EXPECTED_LEROBOT_REVISION,
    inspect,
    sha256,
)


def _verify_source_audit(proxy: Path, preview: Path, manifest: dict,
                         source_audit: dict) -> dict:
    if (source_audit.get("schema") != "piper_proxy_sampling_rate_comparison_v1"
            or source_audit.get("status") != "PASS_DATA_VARIANTS_VERIFIED"
            or source_audit.get("raw_episode_id") != manifest.get("raw_episode_id")):
        raise ValueError("source audit identity/status mismatch")
    variant_name = "baseline" if manifest.get("fps") == 2 else "candidate"
    variant = source_audit.get("variants", {}).get(variant_name)
    if (not isinstance(variant, dict)
            or variant.get("fps") != manifest.get("fps")
            or variant.get("proxy_directory") != proxy.name
            or variant.get("aligned_episode_id") != preview.name
            or variant.get("source_frame_extraction_verified") is not True):
        raise ValueError("source audit does not verify this proxy and preview")
    hashes = variant.get("source_hashes", {})
    expected = {
        "raw_manifest_sha256": manifest.get("raw_manifest_sha256"),
        "aligned_metadata_sha256": sha256(preview / "aligned_episode.json"),
        "aligned_frames_sha256": sha256(preview / "frames.jsonl"),
        "frame_map_sha256": json.loads(
            (preview / "aligned_episode.json").read_text(encoding="utf-8")
        ).get("frame_map_sha256"),
        "proxy_manifest_sha256": sha256(proxy / "proxy_manifest.json"),
        "proxy_targets_sha256": sha256(proxy / "proxy_targets.jsonl"),
    }
    if any(hashes.get(key) != value for key, value in expected.items()):
        raise ValueError("source audit hashes do not match the input files")
    return {"variant": variant_name, "source_hashes": hashes}


def _read_parquet_and_verify(dataset: Path, info: dict, rows: list[dict]) -> tuple[Path, dict]:
    import pyarrow.parquet as pq

    if (info.get("total_frames") != len(rows)
            or info.get("total_episodes") != 1
            or info.get("fps") != info.get("features", {}).get(
                "observation.images.image", {}).get("info", {}).get("video.fps")):
        raise ValueError("LeRobot info frame/episode/fps metadata mismatch")
    try:
        relative = info["data_path"].format(episode_chunk=0, episode_index=0)
    except (KeyError, ValueError) as exc:
        raise ValueError("unsupported LeRobot data_path template") from exc
    parquet_path = (dataset / relative).resolve()
    if not parquet_path.is_relative_to(dataset.resolve()) or not parquet_path.is_file():
        raise ValueError("LeRobot episode Parquet is missing or escapes dataset root")
    table = pq.read_table(parquet_path)
    required = {"observation.state", "action", "frame_index", "episode_index", "index"}
    if not required.issubset(table.column_names) or table.num_rows != len(rows):
        raise ValueError("LeRobot Parquet schema or row count mismatch")
    indexes = np.asarray(table["index"].to_numpy(), dtype=np.int64)
    frame_indexes = np.asarray(table["frame_index"].to_numpy(), dtype=np.int64)
    episode_indexes = np.asarray(table["episode_index"].to_numpy(), dtype=np.int64)
    if (not np.array_equal(indexes, np.arange(len(rows)))
            or not np.array_equal(frame_indexes, np.arange(len(rows)))
            or not np.array_equal(episode_indexes, np.zeros(len(rows), dtype=np.int64))):
        raise ValueError("LeRobot Parquet index/episode order mismatch")
    expected_state = np.stack([_seven(row["state"]) for row in rows])
    expected_action = np.stack([_seven(row["proxy_action"]) for row in rows])
    actual_state = np.asarray(table["observation.state"].to_pylist(), dtype=np.float32)
    actual_action = np.asarray(table["action"].to_pylist(), dtype=np.float32)
    if (not np.array_equal(actual_state, expected_state)
            or not np.array_equal(actual_action, expected_action)):
        raise ValueError("LeRobot Parquet state/action values do not match proxy rows")
    return parquet_path, {"rows": table.num_rows, "state_values_verified": True,
                          "action_values_verified": True, "index_order_verified": True}


def _decode_and_verify_videos(dataset: Path, preview: Path, info: dict,
                              source_map: list[dict]) -> tuple[list[dict], dict]:
    import av
    from PIL import Image

    info_template = info.get("video_path")
    if not isinstance(info_template, str):
        raise ValueError("LeRobot video_path template is missing")
    fps = info.get("fps")
    role_to_feature = {
        "base": "observation.images.image",
        "wrist": "observation.images.wrist_image",
    }
    readback_rows = [{"dataset_row_index": index} for index in range(len(source_map))]
    track_report = {}
    for role, feature in role_to_feature.items():
        try:
            relative = info_template.format(
                episode_chunk=0, video_key=feature, episode_index=0
            )
        except (KeyError, ValueError) as exc:
            raise ValueError("unsupported LeRobot video_path template") from exc
        video_path = (dataset / relative).resolve()
        if not video_path.is_relative_to(dataset.resolve()) or not video_path.is_file():
            raise ValueError(f"LeRobot {role} video is missing or escapes dataset root")
        feature_info = info.get("features", {}).get(feature, {}).get("info", {})
        if feature_info.get("video.fps") != fps:
            raise ValueError(f"LeRobot {role} video metadata fps mismatch")
        pts = []
        frame_errors = []
        frame_hashes = []
        with av.open(str(video_path)) as container:
            if len(container.streams.video) != 1:
                raise ValueError(f"LeRobot {role} video must contain exactly one stream")
            stream = container.streams.video[0]
            if (stream.codec_context.width, stream.codec_context.height) != (
                    feature_info.get("video.width"), feature_info.get("video.height")):
                raise ValueError(f"LeRobot {role} video resolution mismatch")
            if float(stream.average_rate) != float(fps):
                raise ValueError(f"LeRobot {role} video average rate mismatch")
            count = 0
            for frame in container.decode(stream):
                if count >= len(source_map):
                    raise ValueError(f"LeRobot {role} video has excess frames")
                decoded = frame.to_ndarray(format="rgb24")
                aligned_row = source_map[count]["observation"]
                source_info = aligned_row["images"][role]
                source_path = _file(preview, source_info["preview_relpath"])
                with Image.open(source_path) as image:
                    source_pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
                if decoded.shape != source_pixels.shape:
                    raise ValueError(f"LeRobot {role} decoded frame shape mismatch at {count}")
                frame_errors.append(float(np.abs(
                    decoded.astype(np.int16) - source_pixels.astype(np.int16)
                ).mean()))
                decoded_hash = hashlib.sha256(decoded.tobytes(order="C")).hexdigest()
                frame_hashes.append(decoded_hash)
                readback_rows[count][f"{role}_decoded_rgb_sha256"] = decoded_hash
                readback_rows[count][f"{role}_source_rgb_sha256"] = source_info["rgb_pixels_sha256"]
                readback_rows[count][f"{role}_mean_absolute_pixel_error"] = frame_errors[-1]
                pts.append(frame.pts)
                count += 1
        if count != len(source_map):
            raise ValueError(f"LeRobot {role} video frame count mismatch: {count}")
        if any(value is None for value in pts) or any(b <= a for a, b in zip(pts, pts[1:])):
            raise ValueError(f"LeRobot {role} video presentation timestamps are not monotonic")
        track_report[role] = {
            "path": relative.replace("\\", "/"),
            "sha256": sha256(video_path),
            "decoded_frames": count,
            "fps": fps,
            "pts_monotonic": True,
            "width": feature_info.get("video.width"),
            "height": feature_info.get("video.height"),
            "decoded_to_source_order_by_dataset_index": True,
            "mean_absolute_pixel_error_mean": statistics.fmean(frame_errors),
            "mean_absolute_pixel_error_p95": float(np.percentile(frame_errors, 95)),
            "decoded_rgb_sha256_chain": hashlib.sha256(
                "\n".join(frame_hashes).encode("ascii")
            ).hexdigest(),
        }
    return readback_rows, track_report


def _write_idempotent(path: Path, payload: bytes) -> None:
    if path.exists():
        if path.read_bytes() != payload:
            raise ValueError(f"refusing to overwrite conflicting provenance file: {path}")
        return
    temporary = path.with_name(path.name + ".tmp-" + uuid.uuid4().hex[:12])
    try:
        temporary.write_bytes(payload)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def audit_proxy_dataset(proxy: Path, preview: Path, dataset: Path,
                        source_audit: dict) -> tuple[dict, bytes, bytes, bytes]:
    proxy, preview, dataset = proxy.resolve(), preview.resolve(), dataset.resolve()
    manifest, rows, _ = inspect(proxy, preview)
    source_validation = _verify_source_audit(proxy, preview, manifest, source_audit)
    source_map, source_map_report = build_source_map(proxy, preview)
    if not dataset.is_dir():
        raise ValueError("LeRobot dataset directory does not exist")
    info_path = dataset / "meta" / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if (info.get("fps") != manifest.get("fps")
            or info.get("total_frames") != len(rows)
            or info.get("total_episodes") != 1):
        raise ValueError("LeRobot dataset identity/fps/length does not match proxy")
    conversion_path = dataset / "provenance" / "proxy_conversion.json"
    conversion = json.loads(conversion_path.read_text(encoding="utf-8"))
    if (conversion.get("proxy_manifest_sha256") != sha256(proxy / "proxy_manifest.json")
            or conversion.get("proxy_targets_sha256") != manifest.get("proxy_targets_sha256")
            or conversion.get("frames") != len(rows)
            or conversion.get("action_semantics") != manifest.get("semantics")):
        raise ValueError("LeRobot conversion provenance does not match proxy inputs")
    runtime = conversion.get("conversion_runtime", {})
    if (runtime.get("datasets_version") != EXPECTED_DATASETS_VERSION
            or runtime.get("lerobot_revision") != EXPECTED_LEROBOT_REVISION):
        raise ValueError("LeRobot conversion runtime is incompatible with the pinned OpenPI stack")
    parquet_path, parquet_report = _read_parquet_and_verify(dataset, info, rows)
    readback_rows, video_report = _decode_and_verify_videos(dataset, preview, info, source_map)

    source_map_bytes = _jsonl_bytes(source_map)
    readback_bytes = _jsonl_bytes(readback_rows)
    report = {
        "schema": "piper_proxy_lerobot_readback_audit_v1",
        "dataset_path": dataset.name,
        "dataset_repo_id": conversion.get("repo_id"),
        "raw_episode_id": source_map_report["raw_episode_id"],
        "fps": manifest["fps"],
        "frames": len(rows),
        "episodes": 1,
        "split": "train_only",
        "action_semantics": manifest["semantics"],
        "source_frame_extraction_verified": True,
        "source_audit_variant": source_validation["variant"],
        "source_audit_hashes": source_validation["source_hashes"],
        "source_map_path": "provenance/source_map.jsonl",
        "source_map_sha256": source_map_report["source_map_sha256"],
        "readback_frames_path": "provenance/readback_frames.jsonl",
        "readback_frames_sha256": hashlib.sha256(readback_bytes).hexdigest(),
        "dataset_info_sha256": sha256(info_path),
        "parquet_path": str(parquet_path.relative_to(dataset)).replace("\\", "/"),
        "parquet_sha256": sha256(parquet_path),
        "parquet": parquet_report,
        "video_tracks": video_report,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    report_bytes = (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8")
    if hashlib.sha256(source_map_bytes).hexdigest() != source_map_report["source_map_sha256"]:
        raise ValueError("source map serialization/hash mismatch")
    return report, source_map_bytes, readback_bytes, report_bytes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proxy", required=True, type=Path)
    parser.add_argument("--preview", required=True, type=Path)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--source-audit", required=True, type=Path)
    parser.add_argument("--write-provenance", action="store_true",
                        help="append source/readback sidecars; never overwrite conflicts")
    args = parser.parse_args()
    source_audit = json.loads(args.source_audit.read_text(encoding="utf-8"))
    report, source_map, readback, report_bytes = audit_proxy_dataset(
        args.proxy, args.preview, args.dataset, source_audit
    )
    if args.write_provenance:
        provenance = args.dataset.resolve() / "provenance"
        provenance.mkdir(exist_ok=True)
        _write_idempotent(provenance / "source_map.jsonl", source_map)
        _write_idempotent(provenance / "readback_frames.jsonl", readback)
        _write_idempotent(provenance / "readback_audit.json", report_bytes)
    print(json.dumps({"status": "READBACK_PASS_NO_WRITE", **report}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
