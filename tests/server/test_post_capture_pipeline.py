from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from laptop.recorder.label_episode import label_episode
from server.training import post_capture_pipeline as pipeline
from tests.data.test_align_raw import fixture


class PostCapturePipelineTest(unittest.TestCase):
    def _labeled_batch(self, root: Path, *, successes: int = 5,
                       failures: int = 1, unlabeled: int = 1):
        raw_root = root / "raw"
        outcome_root = root / "outcomes"
        raw_root.mkdir()
        for index in range(successes):
            raw = fixture(root / f"fixture-success-{index}", episode_id=f"success{index:03d}")
            raw.rename(raw_root / raw.name)
            label_episode(raw_root / raw.name, outcome_root, success=True,
                          notes="test success", operator="test-operator")
        for index in range(failures):
            raw = fixture(root / f"fixture-failure-{index}", episode_id=f"failure{index:03d}")
            raw.rename(raw_root / raw.name)
            label_episode(raw_root / raw.name, outcome_root, success=False,
                          notes="test failure", operator="test-operator")
        for index in range(unlabeled):
            raw = fixture(root / f"fixture-unlabeled-{index}", episode_id=f"unlabeled{index:03d}")
            raw.rename(raw_root / raw.name)
        return raw_root, outcome_root

    def test_selects_only_hash_bound_successes_and_retains_failure_ids(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw_root, outcomes = self._labeled_batch(root)
            result = pipeline.select_successful_episodes(raw_root, outcomes)
            self.assertEqual(len(result["successful"]), 5)
            self.assertEqual(result["failed_episode_ids"], ["failure000"])
            self.assertEqual(result["unlabeled_episode_ids"], ["unlabeled000"])

    def test_one_reviewed_success_is_enough_for_fast_offline_tiny_fit(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw_root, outcomes = self._labeled_batch(
                root, successes=1, failures=1, unlabeled=0,
            )
            result = pipeline.select_successful_episodes(raw_root, outcomes)
            self.assertEqual([x["episode_id"] for x in result["successful"]], ["success000"])
            self.assertEqual(result["failed_episode_ids"], ["failure000"])

    def test_prepares_one_dataset_from_successes_and_never_admits_failures(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw_root, outcomes = self._labeled_batch(root)
            aligned_calls = []
            proxy_calls = []

            def fake_align(raw: Path, output: Path, *, fps: int, max_skew_ms: float):
                output.mkdir(parents=True)
                aligned_calls.append((raw.name, fps, max_skew_ms))
                return {"status": "PREVIEW_ONLY"}

            def fake_proxy(preview: Path, raw: Path, outcome: Path, output: Path):
                output.mkdir(parents=True)
                proxy_calls.append((raw.name, outcome.name))
                return {"status": "PROXY_ONLY"}

            def fake_convert(pairs, output: Path, repo_id: str):
                output.mkdir(parents=True)
                return {"status": "PROXY_MULTI_EPISODE_DATASET_OFFLINE_ONLY",
                        "episodes": len(pairs), "repo_id": repo_id}

            with (patch.object(pipeline, "align", side_effect=fake_align),
                  patch.object(pipeline, "build_proxy", side_effect=fake_proxy),
                  patch.object(pipeline, "convert_many", side_effect=fake_convert),
                  patch.object(pipeline, "inspect_proxy_multi_episode", return_value={
                      "status": "READY_FOR_OFFLINE_DERIVED_STATE_PILOT",
                      "dataset_sha256": "a" * 64, "episodes": 5, "frames": 50,
                  })):
                result = pipeline.prepare_dataset(
                    raw_root=raw_root, outcome_root=outcomes,
                    derived_root=root / "derived", dataset_home=root / "incoming",
                    run_id="test_batch_001", repo_id="local/test_batch_001",
                )

            self.assertEqual(len(aligned_calls), 5)
            self.assertTrue(all(call[1] == 10 for call in aligned_calls))
            self.assertEqual(len(proxy_calls), 5)
            self.assertTrue(all("failure" not in raw_id for raw_id, _ in proxy_calls))
            self.assertEqual(result["successful_episode_ids"],
                             [f"success{index:03d}" for index in range(5)])
            self.assertEqual(result["failed_episode_ids"], ["failure000"])
            self.assertFalse(result["hardware_control_allowed"])
            self.assertEqual(result["can_data_tx_calls"], 0)
            self.assertTrue(Path(result["batch_manifest"]).is_file())

    def test_prepare_passes_one_success_to_the_tiny_fit_converter(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw_root, outcomes = self._labeled_batch(
                root, successes=1, failures=1, unlabeled=0,
            )
            converted_counts = []

            def fake_align(_raw: Path, output: Path, *, fps: int, max_skew_ms: float):
                output.mkdir(parents=True)
                return {"status": "PREVIEW_ONLY", "fps": fps,
                        "max_skew_ms": max_skew_ms}

            def fake_proxy(_preview: Path, _raw: Path, _outcome: Path, output: Path):
                output.mkdir(parents=True)
                return {"status": "PROXY_ONLY"}

            def fake_convert(pairs, output: Path, repo_id: str):
                converted_counts.append(len(pairs))
                output.mkdir(parents=True)
                return {"status": "PROXY_DATASET_OFFLINE_ONLY",
                        "episodes": len(pairs), "repo_id": repo_id}

            with (patch.object(pipeline, "align", side_effect=fake_align),
                  patch.object(pipeline, "build_proxy", side_effect=fake_proxy),
                  patch.object(pipeline, "convert_many", side_effect=fake_convert),
                  patch.object(pipeline, "inspect_proxy_multi_episode", return_value={
                      "status": "READY_FOR_OFFLINE_DERIVED_STATE_PILOT",
                      "dataset_sha256": "a" * 64, "episodes": 1, "frames": 10,
                  })):
                result = pipeline.prepare_dataset(
                    raw_root=raw_root, outcome_root=outcomes,
                    derived_root=root / "derived", dataset_home=root / "incoming",
                    run_id="test_one_success_001",
                    repo_id="local/test_one_success_001",
                )

            self.assertEqual(converted_counts, [1])
            self.assertEqual(result["successful_episode_ids"], ["success000"])
            self.assertEqual(result["failed_episode_ids"], ["failure000"])
            self.assertEqual(result["audit"]["episodes"], 1)

    def test_requires_one_success_before_creating_derived_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw_root, outcomes = self._labeled_batch(root, successes=0, failures=1, unlabeled=0)
            derived = root / "derived"
            with self.assertRaisesRegex(ValueError, "at least 1 checksum-bound successful"):
                pipeline.prepare_dataset(
                    raw_root=raw_root, outcome_root=outcomes,
                    derived_root=derived, dataset_home=root / "incoming",
                    run_id="test_batch_001",
                )
            self.assertFalse(derived.exists())

    def test_training_helper_runs_stats_train_eval_and_emits_shadow_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset = root / "incoming" / "local" / "test_batch"
            dataset.mkdir(parents=True)
            base_params = root / "base_params"
            base_params.mkdir()
            assets = root / "assets"
            checkpoint = root / "checkpoint" / "9"
            checkpoint.mkdir(parents=True)
            training_record = root / "training_provenance.json"
            training_record.write_text(
                '{"openpi_commit":"dddddddddddddddddddddddddddddddddddddddd"}\n',
                encoding="utf-8",
            )
            config = SimpleNamespace(assets_dirs=assets)
            execute_calls = []

            def fake_execute(_dataset, *, phase, **_kwargs):
                execute_calls.append(phase)
                if phase == "stats":
                    return {"phase": "stats", "norm_stats_sha256": "b" * 64}
                return {"phase": "train", "checkpoint": str(checkpoint),
                        "training_provenance": str(training_record),
                        "checkpoint_tree_sha256": "c" * 64,
                        "norm_stats_sha256": "b" * 64}

            with (patch.dict("os.environ", {}, clear=True),
                  patch.object(pipeline, "ROOT", root),
                  patch.object(pipeline.pilot, "prepare", return_value={
                      "repo_id": "local/test_batch", "hf_lerobot_home": str(dataset.parents[1]),
                      "dataset_sha256": "a" * 64, "episodes": 5, "frames": 50,
                  }),
                  patch.object(pipeline.pilot, "build_config", return_value=config),
                  patch.object(pipeline.pilot, "execute", side_effect=fake_execute),
                  patch.object(pipeline.pilot, "evaluate", return_value={
                      "status": "SAME_EPISODE_PREDICTION_ONLY",
                      "hardware_success_claim": False,
                  })):
                result = pipeline.train_evaluate_dataset(
                    dataset_path=dataset, base_params_path=base_params,
                    gpu_index="2", steps=20,
                )

            self.assertEqual(execute_calls, ["stats", "train"])
            self.assertEqual(result["status"], "TRAINED_AND_EVALUATED_PREDICTION_ONLY")
            self.assertEqual(result["evaluation"]["status"], "SAME_EPISODE_PREDICTION_ONLY")
            self.assertIn("server.serve_derived_state_multi_shadow",
                          result["shadow_service_command"])
            self.assertTrue(Path(result["client_identity_lock_path"]).is_file())
            self.assertEqual(result["client_identity_lock"]["schema"],
                             "piper_derived_state_multi_shadow_identity_lock_v1")
            self.assertFalse(result["hardware_control_allowed"])
            self.assertEqual(result["can_data_tx_calls"], 0)

    def test_openpi_norm_stats_path_includes_namespaced_repo_id(self) -> None:
        from server.training.proxy_multi_episode_pilot import (
            norm_stats_path,
            openpi_norm_stats_source_path,
        )

        config = SimpleNamespace(assets_dirs=Path("/isolated/assets/config"))
        self.assertEqual(
            norm_stats_path(config, "local/test_batch"),
            Path("/isolated/assets/config/piper/norm_stats.json"),
        )
        self.assertEqual(
            openpi_norm_stats_source_path(config, "local/test_batch"),
            Path("/isolated/assets/config/local/test_batch/norm_stats.json"),
        )

    def test_openpi_norm_stats_are_materialized_without_overwriting(self) -> None:
        from server.training.proxy_multi_episode_pilot import materialize_norm_stats

        with tempfile.TemporaryDirectory() as temp:
            assets = Path(temp) / "assets"
            config = SimpleNamespace(assets_dirs=assets)
            source = assets / "local" / "test_batch" / "norm_stats.json"
            source.parent.mkdir(parents=True)
            source.write_bytes(b"offline-stats")

            target = materialize_norm_stats(config, "local/test_batch")
            self.assertEqual(target.read_bytes(), b"offline-stats")
            self.assertTrue(source.is_file())
            with self.assertRaises(FileExistsError):
                materialize_norm_stats(config, "local/test_batch")

    def test_lerobot_evaluation_uses_exact_local_dataset_root(self) -> None:
        from server.training.proxy_multi_episode_pilot import local_lerobot_dataset_root

        with tempfile.TemporaryDirectory() as temp:
            dataset = Path(temp) / "local" / "test_batch"
            info = dataset / "meta" / "info.json"
            info.parent.mkdir(parents=True)
            info.write_text("{}", encoding="utf-8")
            self.assertEqual(local_lerobot_dataset_root({"dataset_path": dataset}), dataset.resolve())

            missing = Path(temp) / "wrong_root"
            missing.mkdir()
            with self.assertRaisesRegex(FileNotFoundError, "prepared local LeRobot dataset"):
                local_lerobot_dataset_root({"dataset_path": missing})


if __name__ == "__main__":
    unittest.main()
