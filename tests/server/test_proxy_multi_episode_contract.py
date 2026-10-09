from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from server.training.proxy_multi_episode_contract import (
    ACTION_SEMANTICS,
    EXPECTED_DATASETS_VERSION,
    EXPECTED_LEROBOT_REVISION,
    EXPECTED_LEROBOT_VERSION,
    dataset_tree_sha256,
    inspect_proxy_multi_episode,
)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fake_sha(seed: str) -> str:
    return hashlib.sha256(seed.encode("utf-8")).hexdigest()


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
                    encoding="utf-8")


def _fixture(root: Path) -> Path:
    dataset = root / "lerobot" / "five_successful_demos"
    info = {"robot_type": "piper", "total_episodes": 5, "total_frames": 10, "fps": 10,
            "features": {"observation.state": {"shape": [7]}, "action": {"shape": [7]}}}
    _write_json(dataset / "meta/info.json", info)
    parquet = dataset / "data/chunk-000/episode_000000.parquet"
    video = dataset / "videos/chunk-000/base.mp4"
    parquet.parent.mkdir(parents=True, exist_ok=True)
    video.parent.mkdir(parents=True, exist_ok=True)
    parquet.write_bytes(b"test-parquet-bytes")
    video.write_bytes(b"test-video-bytes")
    media = {"data/chunk-000/episode_000000.parquet": _sha(parquet),
             "videos/chunk-000/base.mp4": _sha(video)}

    episodes = [{
        "episode_index": index,
        "raw_episode_id": f"demo{index:03d}",
        "raw_manifest_sha256": _fake_sha(f"raw{index}"),
        "aligned_preview_episode_id": f"preview{index:03d}",
        "aligned_metadata_sha256": _fake_sha(f"aligned{index}"),
        "aligned_frames_sha256": _fake_sha(f"frames{index}"),
        "frame_map_sha256": _fake_sha(f"map{index}"),
        "outcome_label_sha256": _fake_sha(f"outcome{index}"),
        "proxy_manifest_sha256": _fake_sha(f"manifest{index}"),
        "proxy_targets_sha256": _fake_sha(f"targets{index}"),
        "frames": 2,
        "outcome_success": True,
        "split": "train",
    } for index in range(5)]

    source_rows, readback_rows = [], []
    for episode_index, episode in enumerate(episodes):
        for local_index in range(2):
            global_index = episode_index * 2 + local_index
            state_sha, action_sha = _fake_sha(f"state{global_index}"), _fake_sha(f"action{global_index}")
            timestamp = episode_index * 1_000_000_000 + local_index * 100_000_000
            source_rows.append({
                "dataset_row_index": global_index,
                "episode_index": episode_index,
                "episode_frame_index": local_index,
                "raw_episode_id": episode["raw_episode_id"],
                "action_semantics": ACTION_SEMANTICS,
                "observation": {
                    "timestamp_ns": timestamp,
                    "state_float32_sha256": state_sha,
                    "images": {role: {"preview_relpath": f"{role}/{local_index}.png",
                                       "rgb_pixels_sha256": _fake_sha(f"rgb{global_index}")}
                               for role in ("base", "wrist")},
                },
                "proxy_target": {"timestamp_ns": timestamp + 100_000_000,
                                 "action_float32_sha256": action_sha},
            })
            readback_rows.append({
                "dataset_row_index": global_index,
                "episode_index": episode_index,
                "episode_frame_index": local_index,
                "raw_episode_id": episode["raw_episode_id"],
                "state_float32_sha256": state_sha,
                "action_float32_sha256": action_sha,
                "base_source_rgb_sha256": _fake_sha(f"rgb{global_index}"),
                "base_decoded_rgb_sha256": _fake_sha(f"decodedbase{global_index}"),
                "base_mean_absolute_pixel_error": 1.0,
                "wrist_source_rgb_sha256": _fake_sha(f"rgb{global_index}"),
                "wrist_decoded_rgb_sha256": _fake_sha(f"decodedwrist{global_index}"),
                "wrist_mean_absolute_pixel_error": 1.0,
            })
    _write_jsonl(dataset / "provenance/source_map.jsonl", source_rows)
    _write_jsonl(dataset / "provenance/readback_frames.jsonl", readback_rows)
    source_map_sha = _sha(dataset / "provenance/source_map.jsonl")
    readback_sha = _sha(dataset / "provenance/readback_frames.jsonl")
    _write_json(dataset / "provenance/source_map_report.json", {
        "schema": "piper_proxy_lerobot_multi_source_map_v1",
        "source_episodes": episodes,
        "source_map_rows": 10,
        "source_map_sha256": source_map_sha,
        "action_semantics": ACTION_SEMANTICS,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    })
    _write_json(dataset / "provenance/proxy_conversion.json", {
        "schema": "piper_proxy_lerobot_multi_episode_v1",
        "repo_id": "local/five_successful_demos",
        "fps": 10, "frames": 10, "episodes": 5,
        "split": "train_only", "source_episodes": episodes,
        "source_map_rows": 10, "source_map_sha256": source_map_sha,
        "conversion_runtime": {"datasets_version": EXPECTED_DATASETS_VERSION,
                                "lerobot_version": EXPECTED_LEROBOT_VERSION,
                                "lerobot_revision": EXPECTED_LEROBOT_REVISION},
        "action_semantics": ACTION_SEMANTICS,
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
    })
    _write_json(dataset / "provenance/readback_audit.json", {
        "schema": "piper_proxy_lerobot_multi_readback_audit_v1",
        "status": "PASS_READBACK",
        "dataset_path": dataset.name,
        "dataset_repo_id": "local/five_successful_demos",
        "fps": 10, "episodes": 5, "frames": 10, "split": "train_only",
        "action_semantics": ACTION_SEMANTICS,
        "source_frame_extraction_verified": True,
        "state_values_verified": True,
        "action_values_verified": True,
        "episode_and_frame_order_verified": True,
        "decoded_rgb_frames_verified": True,
        "source_map_sha256": source_map_sha,
        "readback_frames_sha256": readback_sha,
        "dataset_info_sha256": _sha(dataset / "meta/info.json"),
        "media_and_parquet_sha256": media,
        "formal_training_eligible": False,
        "hardware_control_allowed": False,
    })
    return dataset


class ProxyMultiEpisodeContractTest(unittest.TestCase):
    def test_five_successful_readback_verified_episodes_pass_offline_gate(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            dataset = _fixture(Path(temp))
            result = inspect_proxy_multi_episode(dataset, expected_dataset_sha256=dataset_tree_sha256(dataset))
            self.assertEqual(result["status"], "READY_FOR_OFFLINE_DERIVED_STATE_PILOT")
            self.assertEqual(result["episodes"], 5)
            self.assertEqual(result["frames"], 10)
            self.assertEqual(result["split"], "train_only")
            self.assertFalse(result["raw_command_supervision"])
            self.assertFalse(result["hardware_control_allowed"])

    def test_wrong_units_or_raw_command_semantics_do_not_enter_derived_pilot(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            dataset = _fixture(Path(temp))
            conversion_path = dataset / "provenance/proxy_conversion.json"
            conversion = json.loads(conversion_path.read_text())
            conversion["raw_command_supervision"] = True
            _write_json(conversion_path, conversion)
            with self.assertRaisesRegex(ValueError, "isolated derived-state"):
                inspect_proxy_multi_episode(dataset)

    def test_non_next_frame_timing_is_rejected_even_with_updated_sidecar_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            dataset = _fixture(Path(temp))
            source_path = dataset / "provenance/source_map.jsonl"
            source_rows = [json.loads(line) for line in source_path.read_text().splitlines()]
            source_rows[0]["proxy_target"]["timestamp_ns"] += 1
            _write_jsonl(source_path, source_rows)
            source_sha = _sha(source_path)
            for relative, field in (("provenance/source_map_report.json", "source_map_sha256"),
                                    ("provenance/proxy_conversion.json", "source_map_sha256"),
                                    ("provenance/readback_audit.json", "source_map_sha256")):
                path = dataset / relative
                value = json.loads(path.read_text())
                value[field] = source_sha
                _write_json(path, value)
            with self.assertRaisesRegex(ValueError, "exactly next 10fps"):
                inspect_proxy_multi_episode(dataset)


if __name__ == "__main__":
    unittest.main()
