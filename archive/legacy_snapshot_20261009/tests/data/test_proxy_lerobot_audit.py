from __future__ import annotations

import copy
import importlib.util
import tempfile
import unittest
from pathlib import Path

from laptop.recorder.align_raw import align
from laptop.recorder.label_episode import label_episode
from laptop.recorder.next_state_proxy import build
from server.dataset_tools.audit_proxy_lerobot import audit_proxy_dataset, _write_idempotent
from server.dataset_tools.compare_proxy_sampling_rates import compare_sampling_rates
from server.dataset_tools.convert_next_state_proxy import convert
from tests.data.test_align_raw import fixture


@unittest.skipUnless(importlib.util.find_spec("lerobot") is not None,
                     "pinned LeRobot API is available in the isolated dataset environment")
class ProxyLeRobotAuditTest(unittest.TestCase):
    def test_end_to_end_audit_binds_parquet_videos_and_source_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, frame_count=40)
            outcome = label_episode(raw, root / "outcomes", success=True,
                                    notes="synthetic successful fixture", operator="test")
            preview2 = root / "preview2"
            preview10 = root / "preview10"
            align(raw, preview2, fps=2)
            align(raw, preview10, fps=10)
            proxy2 = root / "proxy2"
            proxy10 = root / "proxy10"
            build(preview2, raw, Path(outcome["path"]), proxy2)
            build(preview10, raw, Path(outcome["path"]), proxy10)
            source_audit = compare_sampling_rates(
                raw=raw,
                outcome=Path(outcome["path"]),
                baseline_preview=preview2,
                baseline_proxy=proxy2,
                candidate_preview=preview10,
                candidate_proxy=proxy10,
            )
            dataset = root / "lerobot10"
            convert(proxy10, preview10, dataset, "offline/proxy_audit_test")

            report, source_map_bytes, readback_bytes, report_bytes = audit_proxy_dataset(
                proxy10, preview10, dataset, source_audit
            )
            self.assertTrue(report["source_frame_extraction_verified"])
            self.assertEqual(report["frames"], report["parquet"]["rows"])
            self.assertTrue(report["parquet"]["state_values_verified"])
            self.assertTrue(report["parquet"]["action_values_verified"])
            self.assertEqual(report["video_tracks"]["base"]["decoded_frames"], report["frames"])
            self.assertEqual(report["video_tracks"]["wrist"]["decoded_frames"], report["frames"])
            self.assertFalse(report["formal_training_eligible"])
            self.assertFalse(report["hardware_control_allowed"])

            provenance = dataset / "provenance"
            _write_idempotent(provenance / "source_map.jsonl", source_map_bytes)
            _write_idempotent(provenance / "readback_frames.jsonl", readback_bytes)
            _write_idempotent(provenance / "readback_audit.json", report_bytes)
            self.assertTrue((provenance / "source_map.jsonl").is_file())
            self.assertTrue((provenance / "readback_frames.jsonl").is_file())
            self.assertTrue((provenance / "readback_audit.json").is_file())
            _write_idempotent(provenance / "source_map.jsonl", source_map_bytes)

            tampered_audit = copy.deepcopy(source_audit)
            tampered_audit["variants"]["candidate"]["source_hashes"]["proxy_targets_sha256"] = "0" * 64
            with self.assertRaisesRegex(ValueError, "source audit hashes"):
                audit_proxy_dataset(proxy10, preview10, dataset, tampered_audit)


if __name__ == "__main__":
    unittest.main()
