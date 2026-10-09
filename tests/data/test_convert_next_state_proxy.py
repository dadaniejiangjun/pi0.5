from __future__ import annotations

import json
import importlib.util
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

from laptop.recorder.align_raw import align
from laptop.recorder.label_episode import label_episode
from laptop.recorder.next_state_proxy import build
from server.dataset_tools.convert_next_state_proxy import build_source_map, convert, convert_many, inspect
from server.training.proxy_multi_episode_contract import inspect_proxy_multi_episode
from server.training.proxy_multi_episode_pilot import prepare
from tests.data.test_align_raw import fixture


class ProxyConversionTest(unittest.TestCase):
    def _make(self, root: Path, *, source: str = "manual_teach",
              episode_id: str | None = "raw001") -> tuple[Path, Path]:
        root.mkdir(parents=True, exist_ok=True)
        raw = fixture(root / "raw", source=source, episode_id=episode_id)
        preview = root / "preview"
        align(raw, preview)
        label = label_episode(raw, root / "outcomes", success=True,
                              notes="gripper held object", operator="test")
        proxy = root / "proxy"
        build(preview, raw, Path(label["path"]), proxy)
        return proxy, preview

    def test_preflight_keeps_proxy_offline_only(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            manifest, rows, shapes = inspect(*self._make(Path(temp)))
            self.assertFalse(manifest["hardware_control_allowed"])
            self.assertFalse(manifest["formal_training_eligible"])
            self.assertEqual(len(rows), 10)
            self.assertEqual(set(shapes), {"base", "wrist"})

    def test_tampered_target_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            proxy, preview = self._make(Path(temp))
            path = proxy / "proxy_targets.jsonl"
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rows[0]["proxy_action"][0] += 1.0
            path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                inspect(proxy, preview)

    def test_source_map_binds_each_proxy_row_to_observation_and_target(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            proxy, preview = self._make(Path(temp))
            source_map, report = build_source_map(proxy, preview)
            aligned = [json.loads(line) for line in
                       (preview / "frames.jsonl").read_text().splitlines()]
            targets = [json.loads(line) for line in
                       (proxy / "proxy_targets.jsonl").read_text().splitlines()]

            self.assertEqual(len(source_map), len(targets))
            self.assertEqual(report["source_map_rows"], len(targets))
            self.assertRegex(report["proxy_targets_sha256"], r"^[a-f0-9]{64}$")
            for index, row in enumerate(source_map):
                self.assertEqual(row["dataset_row_index"], index)
                self.assertEqual(row["observation"]["aligned_frame_index"], aligned[index]["frame_index"])
                self.assertEqual(row["observation"]["source_refs"], aligned[index]["source_refs"])
                self.assertEqual(row["proxy_target"]["aligned_frame_index"], aligned[index + 1]["frame_index"])
                self.assertEqual(row["proxy_target"]["timestamp_ns"], targets[index]["target_timestamp_ns"])
                self.assertRegex(row["observation"]["images"]["base"]["rgb_pixels_sha256"], r"^[a-f0-9]{64}$")
            self.assertFalse(report["formal_training_eligible"])
            self.assertFalse(report["hardware_control_allowed"])

    def test_convert_many_admits_one_verified_demo_for_first_tiny_fit(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            pair = self._make(root / "one_demo")
            output = root / "hf_lerobot_home" / "offline" / "one_demo_proxy"

            with patch(
                "server.dataset_tools.convert_next_state_proxy._conversion_runtime",
                side_effect=RuntimeError("test stopped before LeRobot conversion"),
            ):
                with self.assertRaisesRegex(RuntimeError, "test stopped"):
                    convert_many([pair], output, "offline/one_demo_proxy")

            self.assertFalse(output.exists())

    def test_convert_many_rejects_empty_episode_list_before_runtime_or_write(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "must_not_be_created"
            with patch(
                "server.dataset_tools.convert_next_state_proxy._conversion_runtime",
            ) as runtime:
                with self.assertRaisesRegex(ValueError, "at least one demonstration"):
                    convert_many([], output, "offline/empty_proxy")
            runtime.assert_not_called()
            self.assertFalse(output.exists())

    @unittest.skipUnless(importlib.util.find_spec("lerobot") is not None,
                         "pinned LeRobot API is available in the isolated dataset environment")
    def test_convert_many_writes_complete_five_episode_readback(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            inputs = [self._make(root / f"demo_{index}",
                                 source="operator_demonstration",
                                 episode_id=f"demo{index:02d}")
                      for index in range(5)]
            output = root / "hf_lerobot_home" / "offline" / "five_demo_proxy"
            import datasets

            old_cache = datasets.config.HF_DATASETS_CACHE
            datasets.config.HF_DATASETS_CACHE = str(root / "hf_cache")
            try:
                result = convert_many(inputs, output, "offline/five_demo_proxy")
                audit = inspect_proxy_multi_episode(output, min_episodes=5)
                preflight = prepare(output, min_episodes=5)
                from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

                dataset = LeRobotDataset("offline/five_demo_proxy", root=output)
                first = dataset[0]
                last = dataset[len(dataset) - 1]
            finally:
                datasets.config.HF_DATASETS_CACHE = old_cache

            source_rows = [json.loads(line) for line in
                           (output / "provenance/source_map.jsonl").read_text().splitlines()]
            self.assertEqual(result["readback_audit_status"], "PASS_READBACK")
            self.assertEqual(audit["episodes"], 5)
            self.assertEqual(preflight["status"], "READY_FOR_OFFLINE_DERIVED_STATE_PILOT")
            self.assertEqual(result["frames"], audit["frames"])
            self.assertEqual(len(source_rows), audit["frames"])
            self.assertEqual(source_rows[0]["raw_episode_id"], "demo00")
            self.assertEqual(source_rows[-1]["raw_episode_id"], "demo04")
            self.assertEqual(tuple(first["observation.state"].shape), (7,))
            self.assertEqual(tuple(last["action"].shape), (7,))
            self.assertFalse(audit["hardware_control_allowed"])
            self.assertFalse(audit["formal_training_eligible"])

    def test_multi_episode_source_map_keeps_episode_boundaries_and_ids(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            first_proxy, first_preview = self._make(root / "first")
            raw = fixture(root / "second", source="operator_demonstration", episode_id="raw002")
            second_preview = root / "second_preview"
            align(raw, second_preview)
            label = label_episode(raw, root / "second_outcomes", success=True,
                                  notes="observed placement", operator="test")
            second_proxy = root / "second_proxy"
            build(second_preview, raw, Path(label["path"]), second_proxy)

            first_rows, _ = build_source_map(first_proxy, first_preview,
                                             episode_index=0, dataset_row_offset=0,
                                             include_episode_identity=True)
            second_rows, report = build_source_map(second_proxy, second_preview,
                                                   episode_index=1, dataset_row_offset=len(first_rows),
                                                   include_episode_identity=True)
            all_rows = first_rows + second_rows
            self.assertEqual(all_rows[0]["episode_index"], 0)
            self.assertEqual(all_rows[len(first_rows)]["episode_index"], 1)
            self.assertEqual(all_rows[len(first_rows)]["dataset_row_index"], len(first_rows))
            self.assertEqual(all_rows[0]["raw_episode_id"], "raw001")
            self.assertEqual(all_rows[len(first_rows)]["raw_episode_id"], "raw002")
            self.assertEqual(all_rows[len(first_rows)]["episode_frame_index"], 0)
            self.assertEqual(report["action_semantics"], "next_observed_state_proxy_nominal_units")

    def test_convert_many_rejects_duplicate_episode_ids_before_runtime_or_write(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            proxy, preview = self._make(root)
            output = root / "must_not_be_created"
            with self.assertRaisesRegex(ValueError, "unique"):
                convert_many([(proxy, preview), (proxy, preview)], output,
                             "offline/duplicate_episode_test")
            self.assertFalse(output.exists())

    def test_convert_many_rejects_failure_labeled_proxy(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            proxy, preview = self._make(root)
            manifest_path = proxy / "proxy_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["outcome_success"] = False
            manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "success label"):
                convert_many([(proxy, preview), (proxy, preview / "unused")],
                             root / "must_not_be_created", "offline/failure_test")

    @unittest.skipUnless(importlib.util.find_spec("datasets") is not None,
                         "datasets version is available for converter compatibility checks")
    def test_conversion_rejects_unpinned_datasets_before_writing(self) -> None:
        import datasets

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            proxy, preview = self._make(root)
            output = root / "must_not_be_created"
            with patch.object(datasets, "__version__", "4.8.5", create=True):
                with self.assertRaisesRegex(RuntimeError, "datasets==3.6.0 is required"):
                    convert(proxy, preview, output, "offline/unpinned_runtime_test")
            self.assertFalse(output.exists())

    @unittest.skipUnless(importlib.util.find_spec("datasets") is not None,
                         "datasets package presence can be checked")
    def test_conversion_rejects_unknown_datasets_version_before_writing(self) -> None:
        import datasets

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            proxy, preview = self._make(root)
            output = root / "must_not_be_created"
            with patch.object(datasets, "__version__", None, create=True):
                with self.assertRaisesRegex(RuntimeError, "found unknown"):
                    convert(proxy, preview, output, "offline/unknown_runtime_test")
            self.assertFalse(output.exists())

    @unittest.skipUnless(importlib.util.find_spec("lerobot") is not None,
                         "pinned LeRobot API is available in the isolated dataset environment")
    def test_conversion_persists_per_row_source_map(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            proxy, preview = self._make(root)
            output = root / "lerobot_proxy"
            import datasets

            old_cache = datasets.config.HF_DATASETS_CACHE
            datasets.config.HF_DATASETS_CACHE = str(root / "hf_cache")
            try:
                result = convert(proxy, preview, output, "offline/proxy_source_map_test")
                from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

                dataset = LeRobotDataset("offline/proxy_source_map_test", root=output)
                first = dataset[0]
                last = dataset[len(dataset) - 1]
            finally:
                datasets.config.HF_DATASETS_CACHE = old_cache
            source_map = [json.loads(line) for line in
                          (output / "provenance/source_map.jsonl").read_text().splitlines()]
            report = json.loads((output / "provenance/source_map_report.json").read_text())

            self.assertEqual(result["source_map_sha256"], report["source_map_sha256"])
            self.assertEqual(len(source_map), result["frames"])
            self.assertEqual(report["source_map_rows"], len(source_map))
            self.assertEqual(tuple(first["observation.images.image"].shape), (3, 48, 64))
            self.assertEqual(tuple(first["observation.images.wrist_image"].shape), (3, 48, 64))
            self.assertEqual(tuple(last["observation.images.image"].shape), (3, 48, 64))


if __name__ == "__main__":
    unittest.main()
