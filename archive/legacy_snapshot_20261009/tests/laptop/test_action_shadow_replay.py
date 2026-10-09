from __future__ import annotations

import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from laptop.client.action_shadow_replay import replay
from laptop.recorder.align_raw import QC_FLAGS, align, plan_alignment
from laptop.recorder.piper_command_targets import ACTION_TARGET_SOURCE
from tests.data.test_align_raw import fixture


def _verified_episode(root: Path) -> tuple[Path, Path]:
    raw = fixture(root, command_targets=True)
    preview = align(raw, root / "preview")
    manifest = json.loads((raw / "manifest.json").read_text(encoding="utf-8"))
    plan = plan_alignment(raw)
    review = {
        "schema": "piper_alignment_review_v1",
        "raw_manifest_sha256": plan["raw_manifest_sha256"],
        "frame_map_sha256": preview["frame_map_sha256"],
        "success": True,
        "joint_degrees_per_raw": 0.001,
        "gripper_meters_per_raw": 1e-6,
        "action_offset_ns": 200_000_000,
        "action_target_source": ACTION_TARGET_SOURCE,
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "command_can_raw_sha256": manifest["files"]["command_can_raw.jsonl"]["sha256"],
        "command_hold_semantics": "zero_order_hold_latest_complete_absolute_target",
        **{flag: True for flag in QC_FLAGS},
    }
    review_path = root / "review.json"
    review_path.write_text(json.dumps(review), encoding="utf-8")
    aligned = root / "aligned001"
    align(raw, aligned, review_path=review_path)
    return raw, aligned


def _metadata(**overrides) -> dict:
    result = {
        "artifact_identity_schema": "piper_policy_artifact_identity_v1",
        "config_schema": "pi05_piper_action_shadow_v1",
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "action_horizon": 16,
        "action_dimension": 7,
        "prediction_only": True,
        "hardware_control_allowed": False,
        "robot_control_ready": False,
        "control_enabled": False,
        "bind_address": "127.0.0.1",
        "checkpoint_identity_scope": "selected_numbered_openpi_checkpoint_tree",
        "checkpoint": "checkpoints/pi05_piper_local_tiny_fit/ckpt/1",
        "dataset": "local/piper_v1",
        "normalization_identity": "piper_action_train_split_stats",
        "openpi_commit": "a" * 40,
    }
    for key in (
        "checkpoint_sha256", "policy_config_sha256", "policy_pipeline_code_sha256",
        "training_config_code_sha256", "normalization_sha256", "dataset_tree_sha256",
    ):
        result[key] = hashlib.sha256(key.encode()).hexdigest()
    result.update(overrides)
    return result


class _FakePredictionClient:
    def __init__(self, metadata: dict, actions: np.ndarray):
        self.metadata = metadata
        self.actions = actions
        self.observations = []

    def get_server_metadata(self):
        return self.metadata

    def infer(self, observation: dict):
        self.observations.append(observation)
        return {"actions": self.actions}


class ActionShadowReplayTest(unittest.TestCase):
    def test_verified_frames_replay_prediction_only_and_report_command_error(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            raw, aligned = _verified_episode(Path(temp))
            client = _FakePredictionClient(_metadata(), np.zeros((16, 7), np.float32))
            report = replay(raw, aligned, client, max_samples=2)

            self.assertEqual(report["status"], "PREDICTION_ONLY_REPLAY_COMPLETED")
            self.assertEqual(report["sample_count"], 2)
            self.assertTrue(report["prediction_only"])
            self.assertFalse(report["hardware_control_allowed"])
            self.assertFalse(report["can_opened"])
            self.assertEqual(report["can_data_tx_calls"], 0)
            self.assertEqual(len(client.observations), 2)
            self.assertEqual(client.observations[0]["base_rgb"].shape, (48, 64, 3))
            self.assertEqual(client.observations[0]["wrist_rgb"].shape, (48, 64, 3))
            self.assertIn("joint_mae_rad", report["samples"][0])

    def test_hardware_capable_or_wrong_identity_server_is_rejected_before_inference(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            raw, aligned = _verified_episode(Path(temp))
            for metadata in (_metadata(hardware_control_allowed=True),
                             _metadata(action_semantics="next_observed_state_proxy_nominal_units")):
                with self.subTest(metadata=metadata):
                    client = _FakePredictionClient(metadata, np.zeros((16, 7), np.float32))
                    with self.assertRaises(PermissionError):
                        replay(raw, aligned, client, max_samples=1)
                    self.assertEqual(client.observations, [])

    def test_bad_response_shape_and_tampered_source_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            raw, aligned = _verified_episode(Path(temp))
            client = _FakePredictionClient(_metadata(), np.zeros((1, 7), np.float32))
            with self.assertRaises(ValueError):
                replay(raw, aligned, client, max_samples=1)

            command_log = raw / "command_can_raw.jsonl"
            lines = command_log.read_text(encoding="utf-8").splitlines()
            row = json.loads(lines[-1])
            payload = bytearray.fromhex(row["raw_payload_hex"])
            payload[0] ^= 0x01
            row["raw_payload_hex"] = payload.hex()
            lines[-1] = json.dumps(row)
            command_log.write_text("\n".join(lines) + "\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                replay(raw, aligned, client, max_samples=1)


if __name__ == "__main__":
    unittest.main()
