from __future__ import annotations

import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from server.dataset_tools.audit_training_data import bind_checkpoint, build_audit, verify_audit
from server.dataset_tools.convert_piper_aligned import convert, inspect_collection
from tests.server.test_piper_aligned_converter import make_fixture


class TrainingDataAuditTest(unittest.TestCase):
    def _dataset(self, root: Path, count: int) -> Path:
        aligned, raw_root = make_fixture(root)
        paths = [aligned]
        for number in range(2, count + 1):
            raw_id = f"raw{number:03d}"
            aligned_id = f"aligned{number:03d}"
            new_raw = raw_root / raw_id
            shutil.copytree(raw_root / "raw001", new_raw)
            manifest_path = new_raw / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["episode_id"] = raw_id
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            new_aligned = root / aligned_id
            shutil.copytree(aligned, new_aligned)
            meta_path = new_aligned / "aligned_episode.json"
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            meta["episode_id"] = aligned_id
            meta["raw_episode_id"] = raw_id
            meta["raw_manifest_sha256"] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
            paths.append(new_aligned)
        checked = inspect_collection(paths, raw_root, allow_fixture=True)
        dataset = root / "derived"
        convert(checked, dataset, "offline/piper_audit_fixture")
        return dataset

    def test_episode_split_train_only_stats_and_hash_binding(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset = self._dataset(root, 3)
            output = root / "audit"
            report = build_audit(dataset, output, seed=7, allow_fixture=True)
            self.assertEqual(len(report["train_episode_ids"]), 2)
            self.assertEqual(len(report["validation_episode_ids"]), 1)
            self.assertEqual(report["train_frame_count"], 4)
            self.assertEqual(report["validation_frame_count"], 2)
            self.assertFalse(report["training_eligible"])
            stats = json.loads((output / "norm_stats.json").read_text(encoding="utf-8"))
            self.assertEqual(stats["train_episode_ids"], report["train_episode_ids"])
            self.assertEqual(verify_audit(output)["status"], "HASH_BINDING_PASS")
            checkpoint = root / "checkpoint"
            checkpoint.mkdir()
            (checkpoint / "weights.bin").write_bytes(b"test-only")
            (checkpoint / "training_provenance.json").write_text(json.dumps({
                "dataset_sha256": report["dataset_sha256"],
                "norm_stats_sha256": report["norm_stats_sha256"],
            }), encoding="utf-8")
            bind_checkpoint(output, checkpoint)
            self.assertIsNotNone(verify_audit(output)["checkpoint_sha256"])
            (checkpoint / "weights.bin").write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError, "checkpoint changed"):
                verify_audit(output)

    def test_one_episode_has_no_fake_validation_and_detects_data_tamper(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset = self._dataset(root, 1)
            with self.assertRaisesRegex(ValueError, "lacks sample-level"):
                build_audit(dataset, root / "rejected")
            output = root / "audit"
            report = build_audit(dataset, output, allow_fixture=True)
            self.assertEqual(report["status"], "TRAIN_ONLY_NO_VALIDATION")
            self.assertEqual(report["validation_episode_ids"], [])
            self.assertEqual(report["train_frame_count"], 2)
            self.assertEqual(verify_audit(output)["status"], "HASH_BINDING_PASS")
            (dataset / "provenance/source_map.jsonl").write_text("tampered\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "dataset changed"):
                verify_audit(output)

    def test_mismatched_checkpoint_provenance_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            dataset = self._dataset(root, 1)
            output = root / "audit"
            report = build_audit(dataset, output, allow_fixture=True)
            checkpoint = root / "checkpoint"
            checkpoint.mkdir()
            (checkpoint / "training_provenance.json").write_text(json.dumps({
                "dataset_sha256": "wrong",
                "norm_stats_sha256": report["norm_stats_sha256"],
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "different dataset"):
                bind_checkpoint(output, checkpoint)


if __name__ == "__main__":
    unittest.main()
