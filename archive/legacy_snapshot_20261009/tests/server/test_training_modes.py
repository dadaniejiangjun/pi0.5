from __future__ import annotations

import json
import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from server.training.check_piper_readiness import inspect
from server.training.piper_train_entry import execute_training, prepare
from server.training.public_data_contract import (
    LOCAL_ACTION_SEMANTICS, PUBLIC_ACTION_SEMANTICS, inspect_public_acceptance,
)


class TrainingModesTest(unittest.TestCase):
    def _fixture(self, root: Path, *, success: bool = True, count: int = 1) -> tuple[Path, Path]:
        dataset = root / "datasets/lerobot/local/piper_v1"
        (dataset / "meta").mkdir(parents=True)
        (dataset / "provenance").mkdir()
        (dataset / "meta/info.json").write_text(json.dumps({"total_episodes": count}))
        acceptance = root / "datasets/manifests/piper_v1_acceptance.json"
        acceptance.parent.mkdir(parents=True)
        acceptance_doc = {
            "schema": "piper_real_demo_acceptance_v1", "repo_id": "local/piper_v1",
            "action_semantics": "absolute_joint_position_rad_gripper_meter",
            "episodes": [{"source": "manual_teach", "success": success,
                          "action_target_source": "observed_piper_can_command_frames",
                          "action_target_verified": True, "camera_sync_verified": True,
                          "command_target_provenance_verified": True,
                          "gripper_verified": True, "units_verified": True}] * count,
        }
        acceptance.write_text(json.dumps(acceptance_doc))
        acceptance_copy = dataset / "provenance/acceptance_manifest.json"
        acceptance_copy.write_bytes(acceptance.read_bytes())
        acceptance_sha256 = hashlib.sha256(acceptance.read_bytes()).hexdigest()
        (dataset / "provenance/conversion_manifest.json").write_text(json.dumps({
            "repo_id": "local/piper_v1", "source_map_sha256": "abc",
            "source_episodes": [{}] * count, "synthetic_fixture": False,
            "command_target_provenance_verified": True,
            "action_target_source": "observed_piper_can_command_frames",
            "acceptance_manifest": "provenance/acceptance_manifest.json",
            "acceptance_manifest_sha256": acceptance_sha256,
            "training_eligible": False,
        }))
        return dataset, acceptance

    def test_one_success_is_tiny_only(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset, acceptance = self._fixture(root)
            args = {"dataset_path": dataset, "acceptance_path": acceptance}
            self.assertEqual(inspect(root, "local-tiny-fit", **args)["status"], "READY_FOR_SEPARATE_REVIEW")
            self.assertEqual(inspect(root, "formal", **args)["status"], "BLOCKED")
            self.assertFalse(inspect(root, "local-tiny-fit", **args)["hardware_control_allowed"])

    def test_failed_episode_cannot_be_tiny_fit(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset, acceptance = self._fixture(root, success=False)
            self.assertEqual(inspect(root, "local-tiny-fit", dataset_path=dataset,
                                     acceptance_path=acceptance)["status"], "BLOCKED")

    def test_acceptance_sidecar_must_match_conversion_hash(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset, acceptance = self._fixture(root)
            acceptance.write_text(acceptance.read_text() + "\n", encoding="utf-8")
            report = inspect(root, "local-tiny-fit", dataset_path=dataset,
                             acceptance_path=acceptance)
            self.assertEqual(report["status"], "BLOCKED")
            self.assertFalse(report["checks"]["dataset_identity"])

    def test_debug_and_public_need_explicit_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset, acceptance = self._fixture(root)
            self.assertEqual(inspect(root, "synthetic-debug", dataset_path=dataset,
                                     acceptance_path=acceptance)["status"], "BLOCKED")
            self.assertEqual(inspect(root, "public-data", dataset_path=dataset,
                                     acceptance_path=acceptance)["status"], "BLOCKED")

    def test_public_units_and_source_must_match_and_stay_prediction_only(self) -> None:
        provenance = {"source_revision": "revision-1", "archive_sha256": "sha-1",
                      "synthetic_fixture": False}
        acceptance = {
            "schema": "piper_public_data_acceptance_v1",
            "action_semantics": PUBLIC_ACTION_SEMANTICS,
            "joint_unit": "degree", "gripper_unit": "binary_open_fraction",
            "units_reviewed": True, "source_revision": "revision-1",
            "archive_sha256": "sha-1", "hardware_control_allowed": False,
        }
        self.assertTrue(all(inspect_public_acceptance(acceptance, provenance).values()))
        for changed in (
            {"action_semantics": LOCAL_ACTION_SEMANTICS},
            {"joint_unit": "radian"},
            {"gripper_unit": "meter"},
            {"units_reviewed": False},
            {"archive_sha256": "other"},
            {"hardware_control_allowed": True},
        ):
            self.assertFalse(all(inspect_public_acceptance(
                {**acceptance, **changed}, provenance).values()))
        with self.assertRaisesRegex(PermissionError, "public-data uses"):
            execute_training(Path("unused"), mode="public-data",
                             allow_offline_training=True)

    def test_execution_requires_flag_and_readiness(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset, acceptance = self._fixture(root, success=False)
            with self.assertRaises(PermissionError):
                execute_training(root, mode="local-tiny-fit", dataset_path=dataset,
                                 acceptance_path=acceptance)
            with self.assertRaises(PermissionError):
                execute_training(root, mode="local-tiny-fit", dataset_path=dataset,
                                 acceptance_path=acceptance, allow_offline_training=True)


    def test_eligible_tiny_fit_reaches_offline_stats_only(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset, acceptance = self._fixture(root)
            script = root / "third_party/openpi/scripts/train.py"
            script.parent.mkdir(parents=True)
            script.write_text("def main(config): pass\n")
            params = root / "base_params"
            params.mkdir()
            calls = []
            fake = SimpleNamespace(main=lambda name, max_frames: calls.append((name, max_frames)))
            with patch("server.training.piper_train_entry._official_script", return_value=fake):
                report = execute_training(
                    root, mode="local-tiny-fit", phase="stats", repo_id="local/piper_v1",
                    dataset_path=dataset, acceptance_path=acceptance,
                    base_params_path=str(params), max_frames=8, allow_offline_training=True,
                )
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0][1], 8)
            self.assertEqual(report["phase"], "stats")
            self.assertFalse(report["hardware_control_allowed"])


if __name__ == "__main__":
    unittest.main()
