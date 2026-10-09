from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from server.training.proxy_dataset_contract import (
    ACTION_SEMANTICS,
    AUDITED_2FPS_BASELINE,
    EXPECTED_DATASETS_VERSION,
    EXPECTED_LEROBOT_REVISION,
    EXPECTED_LEROBOT_VERSION,
    PROXY_DATASETS,
    inspect_proxy_dataset,
)


RAW_EPISODE_ID = "cef166e2cc6240b397db241a916ece3a"
RAW_SHA = "1" * 64
ALIGNED_SHA = "2" * 64
PROXY_SHA = "3" * 64
TARGETS_SHA = "4" * 64
RGB_SHA = "5" * 64
ACTION_SHA = "6" * 64


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
                    encoding="utf-8")


def _make_fixture(root: Path, variant: str = "10fps") -> Path:
    spec = AUDITED_2FPS_BASELINE if variant == AUDITED_2FPS_BASELINE.variant else PROXY_DATASETS[variant]
    frames = spec.frames
    fps = spec.fps
    dataset = root / "datasets" / "lerobot" / spec.repo_id.rsplit("/", 1)[-1]
    parquet_rel = "data/chunk-000/episode_000000.parquet"
    base_video_rel = "videos/chunk-000/observation.images.image/episode_000000.mp4"
    wrist_video_rel = "videos/chunk-000/observation.images.wrist_image/episode_000000.mp4"
    parquet_path = dataset / parquet_rel
    parquet_path.parent.mkdir(parents=True, exist_ok=True)
    parquet_path.write_bytes(b"fixture-parquet-payload")
    base_video = dataset / base_video_rel
    base_video.parent.mkdir(parents=True, exist_ok=True)
    base_video.write_bytes(b"fixture-base-video-payload")
    wrist_video = dataset / wrist_video_rel
    wrist_video.parent.mkdir(parents=True, exist_ok=True)
    wrist_video.write_bytes(b"fixture-wrist-video-payload")

    info = {
        "robot_type": "piper",
        "total_episodes": 1,
        "total_frames": frames,
        "fps": fps,
        "splits": {"train": "0:1"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "observation.images.image": {
                "dtype": "video", "shape": [480, 640, 3],
                "info": {"video.fps": fps},
            },
            "observation.images.wrist_image": {
                "dtype": "video", "shape": [480, 640, 3],
                "info": {"video.fps": fps},
            },
            "observation.state": {"shape": [7]},
            "action": {"shape": [7]},
        },
    }
    info_path = dataset / "meta" / "info.json"
    _write_json(info_path, info)

    source_rows = []
    readback_rows = []
    for index in range(frames):
        source_rows.append({
            "dataset_row_index": index,
            "episode_index": 0,
            "action_semantics": ACTION_SEMANTICS,
            "observation": {
                "timestamp_ns": index * spec.action_interval_ns,
                "source_refs": {
                    "base_frame_id": index,
                    "wrist_frame_id": index,
                    "gripper_timestamp_ns": index + 1,
                    "joint_12_timestamp_ns": index + 1,
                    "joint_34_timestamp_ns": index + 1,
                    "joint_56_timestamp_ns": index + 1,
                    "status_timestamp_ns": index + 1,
                },
                "images": {
                    "base": {"preview_relpath": f"base_{index:06d}.png",
                             "rgb_pixels_sha256": RGB_SHA},
                    "wrist": {"preview_relpath": f"wrist_{index:06d}.png",
                              "rgb_pixels_sha256": RGB_SHA},
                },
            },
            "proxy_target": {
                "timestamp_ns": (index + 1) * spec.action_interval_ns,
                "action_float32_sha256": ACTION_SHA,
            },
        })
        readback_rows.append({
            "dataset_row_index": index,
            "base_source_rgb_sha256": RGB_SHA,
            "wrist_source_rgb_sha256": RGB_SHA,
            "base_decoded_rgb_sha256": RGB_SHA,
            "wrist_decoded_rgb_sha256": RGB_SHA,
        })
    source_map_path = dataset / "provenance" / "source_map.jsonl"
    readback_frames_path = dataset / "provenance" / "readback_frames.jsonl"
    _write_jsonl(source_map_path, source_rows)
    _write_jsonl(readback_frames_path, readback_rows)
    source_map_sha = _sha(source_map_path)
    readback_frames_sha = _sha(readback_frames_path)

    source_map_report = {
        "schema": "piper_proxy_lerobot_source_map_v1",
        "raw_episode_id": RAW_EPISODE_ID,
        "raw_manifest_sha256": RAW_SHA,
        "aligned_metadata_sha256": ALIGNED_SHA,
        "aligned_frames_sha256": ALIGNED_SHA,
        "frame_map_sha256": ALIGNED_SHA,
        "proxy_manifest_sha256": PROXY_SHA,
        "proxy_targets_sha256": TARGETS_SHA,
        "source_map_rows": frames,
        "source_map_sha256": source_map_sha,
        "action_semantics": ACTION_SEMANTICS,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    _write_json(dataset / "provenance" / "source_map_report.json", source_map_report)
    conversion = {
        "schema": "piper_proxy_lerobot_v1",
        "repo_id": spec.repo_id,
        "proxy_manifest_sha256": PROXY_SHA,
        "proxy_targets_sha256": TARGETS_SHA,
        "frames": frames,
        "episodes": 1,
        "split": "train_only",
        "source_map_sha256": source_map_sha,
        "source_map_rows": frames,
        "conversion_runtime": {
            "datasets_version": EXPECTED_DATASETS_VERSION,
            "lerobot_version": EXPECTED_LEROBOT_VERSION,
            "lerobot_revision": EXPECTED_LEROBOT_REVISION,
        },
        "action_semantics": ACTION_SEMANTICS,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    _write_json(dataset / "provenance" / "proxy_conversion.json", conversion)
    audit = {
        "schema": "piper_proxy_lerobot_readback_audit_v1",
        "dataset_path": dataset.name,
        "dataset_repo_id": spec.repo_id,
        "raw_episode_id": RAW_EPISODE_ID,
        "fps": fps,
        "frames": frames,
        "episodes": 1,
        "split": "train_only",
        "action_semantics": ACTION_SEMANTICS,
        "source_frame_extraction_verified": True,
        "source_audit_variant": spec.source_audit_variant,
        "source_audit_hashes": {
            "raw_manifest_sha256": RAW_SHA,
            "raw_files_sha256": {"raw_manifest.json": RAW_SHA},
            "aligned_metadata_sha256": ALIGNED_SHA,
            "aligned_frames_sha256": ALIGNED_SHA,
            "frame_map_sha256": ALIGNED_SHA,
            "outcome_label_sha256": ALIGNED_SHA,
            "proxy_manifest_sha256": PROXY_SHA,
            "proxy_targets_sha256": TARGETS_SHA,
        },
        "source_map_path": "provenance/source_map.jsonl",
        "source_map_sha256": source_map_sha,
        "readback_frames_path": "provenance/readback_frames.jsonl",
        "readback_frames_sha256": readback_frames_sha,
        "dataset_info_sha256": _sha(info_path),
        "parquet_path": parquet_rel,
        "parquet_sha256": _sha(parquet_path),
        "parquet": {
            "rows": frames, "index_order_verified": True,
            "state_values_verified": True, "action_values_verified": True,
        },
        "video_tracks": {
            "base": {
                "path": base_video_rel, "sha256": _sha(base_video),
                "decoded_frames": frames, "fps": fps,
                "pts_monotonic": True,
                "decoded_to_source_order_by_dataset_index": True,
                "width": 640, "height": 480,
            },
            "wrist": {
                "path": wrist_video_rel, "sha256": _sha(wrist_video),
                "decoded_frames": frames, "fps": fps,
                "pts_monotonic": True,
                "decoded_to_source_order_by_dataset_index": True,
                "width": 640, "height": 480,
            },
        },
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    }
    _write_json(dataset / "provenance" / "readback_audit.json", audit)
    return dataset


class ProxyDatasetContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.dataset = _make_fixture(Path(self.temp.name))

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_complete_source_audit_passes_and_returns_external_hashes(self) -> None:
        from server.training.proxy_dataset_contract import dataset_tree_sha256

        result = inspect_proxy_dataset(
            self.dataset,
            expected_dataset_tree_sha256=dataset_tree_sha256(self.dataset),
            expected_readback_audit_sha256=_sha(
                self.dataset / "provenance" / "readback_audit.json"),
        )
        self.assertEqual(result["fps"], 10)
        self.assertEqual(result["frames"], 199)
        self.assertFalse(result["formal_training_eligible"])
        self.assertFalse(result["hardware_control_allowed"])
        self.assertEqual(result["action_target_kind"], "derived_future_observed_state_position_target")
        self.assertEqual(result["action_time_relation"], "observation_state_at_t_to_observed_state_at_t_plus_delta")
        self.assertEqual(result["action_source"], "future_observed_joint_and_gripper_feedback")
        self.assertFalse(result["raw_command_supervision"])
        self.assertTrue(result["offline_tiny_fit_allowed"])
        self.assertTrue(result["prediction_only"])

    def test_ten_step_training_uses_zero_based_checkpoint_step(self) -> None:
        self.assertEqual(PROXY_DATASETS["10fps"].default_steps, 10)
        self.assertEqual(PROXY_DATASETS["10fps"].eval_step, 9)

    def test_audited_two_fps_baseline_uses_its_own_sampling_interval_and_identity(self) -> None:
        baseline = _make_fixture(Path(self.temp.name) / "baseline", "2fps-baseline-audited")
        result = inspect_proxy_dataset(baseline, variant="2fps-baseline-audited")
        self.assertEqual(result["repo_id"], AUDITED_2FPS_BASELINE.repo_id)
        self.assertEqual(result["fps"], 2)
        self.assertEqual(result["frames"], 39)
        self.assertEqual(result["source_audit_variant"], "baseline")
        self.assertFalse(result["formal_training_eligible"])
        self.assertFalse(result["hardware_control_allowed"])

    def test_wrong_external_dataset_hash_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "dataset tree hash"):
            inspect_proxy_dataset(self.dataset, expected_dataset_tree_sha256="0" * 64)

    def test_wrong_external_readback_hash_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "readback audit hash"):
            inspect_proxy_dataset(self.dataset, expected_readback_audit_sha256="0" * 64)

    def test_tampered_parquet_is_rejected(self) -> None:
        parquet = self.dataset / "data/chunk-000/episode_000000.parquet"
        parquet.write_bytes(parquet.read_bytes() + b"tampered")
        with self.assertRaisesRegex(ValueError, "Parquet file hash"):
            inspect_proxy_dataset(self.dataset)

    def test_wrong_runtime_revision_is_rejected(self) -> None:
        conversion_path = self.dataset / "provenance/proxy_conversion.json"
        conversion = json.loads(conversion_path.read_text(encoding="utf-8"))
        conversion["conversion_runtime"]["lerobot_revision"] = "wrong"
        _write_json(conversion_path, conversion)
        with self.assertRaisesRegex(ValueError, "pinned runtime"):
            inspect_proxy_dataset(self.dataset)

    def test_path_traversal_in_audit_is_rejected(self) -> None:
        audit_path = self.dataset / "provenance/readback_audit.json"
        audit = json.loads(audit_path.read_text(encoding="utf-8"))
        audit["parquet_path"] = "../outside.parquet"
        _write_json(audit_path, audit)
        with self.assertRaisesRegex(ValueError, "escapes the dataset"):
            inspect_proxy_dataset(self.dataset)

    def test_non_offline_training_flags_are_rejected(self) -> None:
        conversion_path = self.dataset / "provenance/proxy_conversion.json"
        conversion = json.loads(conversion_path.read_text(encoding="utf-8"))
        conversion["hardware_control_allowed"] = True
        _write_json(conversion_path, conversion)
        with self.assertRaisesRegex(ValueError, "conversion provenance"):
            inspect_proxy_dataset(self.dataset)


if __name__ == "__main__":
    unittest.main()
