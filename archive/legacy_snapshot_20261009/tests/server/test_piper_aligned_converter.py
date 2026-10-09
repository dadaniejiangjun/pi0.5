from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from openpi import transforms

from server.dataset_tools.convert_piper_aligned import TASK, TASK_ID, UNITS, QC_FLAGS
from server.dataset_tools.convert_piper_aligned import convert, inspect_collection, inspect_episode, main
from server.training.check_piper_readiness import inspect as inspect_training
from server.training.piper_openpi_config import make_unregistered_config


def make_fixture(root: Path) -> tuple[Path, Path]:
    raw_root = root / "raw"
    raw = raw_root / "raw001"
    raw.mkdir(parents=True)
    marker = raw / "marker.txt"
    marker.write_text("synthetic raw source\n", encoding="utf-8")
    raw_manifest = {
        "episode_id": "raw001", "source": "synthetic_fixture",
        "files": {"marker.txt": {"sha256": hashlib.sha256(marker.read_bytes()).hexdigest()}},
    }
    manifest_path = raw / "manifest.json"
    manifest_path.write_text(json.dumps(raw_manifest), encoding="utf-8")
    aligned = root / "aligned001"
    aligned.mkdir()
    metadata = {
        "schema": "piper_aligned_episode_v1", "episode_id": "aligned001",
        "raw_episode_id": "raw001", "raw_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "source": "synthetic_fixture", "task_id": TASK_ID, "task": TASK,
        "success": True, "action_semantics": "absolute_joint_position",
        "action_target_source": "operator_verified_target", "state_units": UNITS, "action_units": UNITS,
        "fps": 10, "frame_count": 2, "action_offset_ns": 100_000_000,
        **{flag: True for flag in QC_FLAGS},
    }
    (aligned / "aligned_episode.json").write_text(json.dumps(metadata), encoding="utf-8")
    with (aligned / "frames.jsonl").open("w", encoding="utf-8") as handle:
        for index in range(2):
            for role in ("base", "wrist"):
                Image.fromarray(np.full((64, 64, 3), 20 + index, dtype=np.uint8)).save(
                    aligned / f"{role}_{index}.png"
                )
            ts = 1_000_000_000 + index * 100_000_000
            row = {
                "frame_index": index,
                "policy_timestamp_ns": ts,
                "base_timestamp_ns": ts,
                "wrist_timestamp_ns": ts + 1_000_000,
                "state_timestamp_ns": ts,
                "action_target_timestamp_ns": ts + 100_000_000,
                "base_image": f"base_{index}.png", "wrist_image": f"wrist_{index}.png",
                "state": [index / 10] * 7, "action": [(index + 1) / 10] * 7,
                "source_refs": {
                    "base_frame_id": index, "wrist_frame_id": index,
                    "joint_12_timestamp_ns": ts, "joint_34_timestamp_ns": ts,
                    "joint_56_timestamp_ns": ts, "gripper_timestamp_ns": ts,
                    "status_timestamp_ns": ts,
                },
            }
            handle.write(json.dumps(row) + "\n")
    return aligned, raw_root


class PiperAlignedConverterTest(unittest.TestCase):
    def test_cli_rejects_synthetic_without_creating_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            aligned, raw_root = make_fixture(root)
            output = root / "datasets" / "lerobot" / "piper_v1"
            argv = ["convert_piper_aligned", str(aligned), "--raw-root", str(raw_root),
                    "--output", str(output), "--write-derived"]
            with patch("sys.argv", argv):
                self.assertEqual(main(), 2)
            self.assertFalse(output.exists())

    def test_production_gate_rejects_fixture_and_missing_action_review(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            aligned, raw_root = make_fixture(Path(temp))
            with self.assertRaisesRegex(ValueError, "manual-teach"):
                inspect_episode(aligned, raw_root)
            metadata_path = aligned / "aligned_episode.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["action_target_verified"] = False
            metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "QC evidence"):
                inspect_episode(aligned, raw_root, allow_fixture=True)

    def test_detects_raw_tamper_and_path_escape(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            aligned, raw_root = make_fixture(Path(temp))
            (raw_root / "raw001" / "marker.txt").write_text("changed", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "raw source checksum"):
                inspect_episode(aligned, raw_root, allow_fixture=True)
        with tempfile.TemporaryDirectory() as temp:
            aligned, raw_root = make_fixture(Path(temp))
            frames = aligned / "frames.jsonl"
            rows = [json.loads(line) for line in frames.read_text(encoding="utf-8").splitlines()]
            rows[0]["base_image"] = "../outside.png"
            frames.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "outside episode"):
                inspect_episode(aligned, raw_root, allow_fixture=True)

    def test_rejects_time_mismatch_and_nonfinite_action(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            aligned, raw_root = make_fixture(Path(temp))
            frames = aligned / "frames.jsonl"
            rows = [json.loads(line) for line in frames.read_text(encoding="utf-8").splitlines()]
            rows[0]["action_target_timestamp_ns"] += 3_000_000
            frames.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "action offset mismatch"):
                inspect_episode(aligned, raw_root, allow_fixture=True)
        with tempfile.TemporaryDirectory() as temp:
            aligned, raw_root = make_fixture(Path(temp))
            frames = aligned / "frames.jsonl"
            rows = [json.loads(line) for line in frames.read_text(encoding="utf-8").splitlines()]
            rows[0]["action"][0] = float("nan")
            frames.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "finite 7D"):
                inspect_episode(aligned, raw_root, allow_fixture=True)

    def test_synthetic_full_conversion_and_openpi_readback(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            aligned, raw_root = make_fixture(root)
            checked = inspect_collection([aligned], raw_root, allow_fixture=True)
            output = root / "datasets" / "lerobot" / "piper_v1"
            report = convert(checked, output, "offline/piper_fixture")
            self.assertEqual(report["status"], "CONVERTED_NEEDS_AUDIT")
            self.assertTrue(report["synthetic_fixture"])
            self.assertFalse(report["training_eligible"])
            self.assertFalse(report["source_frame_extraction_verified"])
            provenance = output / "provenance"
            source_map = [json.loads(line) for line in (provenance / "source_map.jsonl").read_text().splitlines()]
            self.assertEqual(len(source_map), 2)
            self.assertEqual(source_map[0]["raw_episode_id"], "raw001")
            self.assertEqual(source_map[0]["base_image_relpath"], "base_0.png")
            self.assertEqual(source_map[1]["source_refs"]["wrist_frame_id"], 1)
            reopened = LeRobotDataset("offline/piper_fixture", root=output,
                                      delta_timestamps={"action": [0.0, 0.1]})
            self.assertEqual(len(reopened), 2)
            config = make_unregistered_config(
                repo_id="offline/piper_fixture", base_params_path="/nonexistent/params", assets_root=str(root)
            )
            data = config.data.create(root, config.model)
            prompted = transforms.PromptFromLeRobotTask(reopened.meta.tasks)(reopened[0])
            mapped = data.repack_transforms.inputs[0](prompted)
            model_input = data.data_transforms.inputs[0](mapped)
            self.assertEqual(model_input["actions"].shape, (2, 7))
            self.assertFalse(model_input["image_mask"]["right_wrist_0_rgb"])
            self.assertEqual(inspect_training(root)["status"], "BLOCKED")
            with self.assertRaisesRegex(ValueError, "unused output"):
                convert(checked, output, "offline/piper_fixture")


if __name__ == "__main__":
    unittest.main()
