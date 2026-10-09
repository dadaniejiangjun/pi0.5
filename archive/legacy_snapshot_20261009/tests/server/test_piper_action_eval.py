from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from server.training.piper_action_eval import (
    _check_checkpoint_provenance,
    _dataset_tree_sha256,
    _validation_rows,
)


class PiperActionEvalTest(unittest.TestCase):
    def test_checkpoint_identity_is_pinned_to_training_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            checkpoint = Path(temp) / "step-2"
            checkpoint.mkdir()
            (checkpoint / "params").write_bytes(b"trained params")
            (checkpoint / "assets" / "piper").mkdir(parents=True)
            stats_path = checkpoint / "assets" / "piper" / "norm_stats.json"
            stats_path.write_text("{}", encoding="utf-8")
            record = {
                "checkpoint_path": str(checkpoint),
                "checkpoint_identity_scope": "selected_numbered_openpi_checkpoint_tree",
                "norm_stats_sha256": hashlib.sha256(stats_path.read_bytes()).hexdigest(),
            }
            from server.policy_server.artifact_identity import tree_sha256
            record["checkpoint_sha256"] = tree_sha256(checkpoint)
            _check_checkpoint_provenance(checkpoint, record)
            (checkpoint / "params").write_bytes(b"changed params")
            with self.assertRaisesRegex(ValueError, "do not match"):
                _check_checkpoint_provenance(checkpoint, record)

    def test_uses_only_hash_bound_validation_episodes(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            dataset = Path(temp) / "dataset"
            provenance = dataset / "provenance"
            provenance.mkdir(parents=True)
            source = provenance / "source_map.jsonl"
            source.write_text("".join(json.dumps({
                "raw_episode_id": episode,
                "derived_episode_index": index,
                "derived_frame_index": 0,
            }) + "\n" for index, episode in enumerate(("raw_a", "raw_b", "raw_c"))),
                encoding="utf-8")
            (provenance / "conversion_manifest.json").write_text(json.dumps({
                "source_map_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            }), encoding="utf-8")
            audit = Path(temp) / "audit_manifest.json"
            audit.write_text(json.dumps({
                "dataset_sha256": _dataset_tree_sha256(dataset),
                "status": "SPLIT_AND_STATS_VERIFIED",
                "validation_episode_ids": ["raw_b"],
            }), encoding="utf-8")

            selected, scope = _validation_rows(dataset, audit)

        self.assertEqual(selected, [1])
        self.assertEqual(scope, "HELD_OUT_EPISODE_EVALUATION")

    def test_one_episode_is_explicitly_train_only_not_validation(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            dataset = Path(temp) / "dataset"
            provenance = dataset / "provenance"
            provenance.mkdir(parents=True)
            source = provenance / "source_map.jsonl"
            source.write_text(json.dumps({"raw_episode_id": "raw_a"}) + "\n", encoding="utf-8")
            (provenance / "conversion_manifest.json").write_text(json.dumps({
                "source_map_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            }), encoding="utf-8")
            audit = Path(temp) / "audit_manifest.json"
            audit.write_text(json.dumps({
                "dataset_sha256": _dataset_tree_sha256(dataset),
                "status": "TRAIN_ONLY_NO_VALIDATION",
                "validation_episode_ids": [],
            }), encoding="utf-8")
            selected, scope = _validation_rows(dataset, audit)

        self.assertEqual(selected, [0])
        self.assertEqual(scope, "TRAIN_ONLY_SAME_EPISODE_REPLAY_NO_VALIDATION")

    def test_rejects_an_audit_from_different_dataset(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            dataset = Path(temp) / "dataset"
            dataset.mkdir()
            (dataset / "file").write_text("current", encoding="utf-8")
            audit = Path(temp) / "audit_manifest.json"
            audit.write_text(json.dumps({
                "dataset_sha256": "wrong",
                "status": "TRAIN_ONLY_NO_VALIDATION",
                "validation_episode_ids": [],
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not bound"):
                _validation_rows(dataset, audit)


if __name__ == "__main__":
    unittest.main()
