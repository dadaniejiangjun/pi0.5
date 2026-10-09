from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from laptop.recorder.align_raw import align
from laptop.recorder.label_episode import label_episode
from laptop.recorder.next_state_proxy import build
from server.dataset_tools.compare_proxy_sampling_rates import compare_sampling_rates
from tests.data.test_align_raw import fixture


class ProxySamplingRateComparisonTest(unittest.TestCase):
    def test_paired_variants_bind_to_same_raw_and_keep_training_locked(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = fixture(root, frame_count=40)
            label = label_episode(
                raw, root / "outcomes", success=True,
                notes="synthetic test fixture outcome", operator="unit-test",
            )
            variants = {}
            for fps in (2, 10):
                preview = root / f"preview_{fps}fps"
                align(raw, preview, fps=fps)
                proxy = root / f"proxy_{fps}fps"
                build(preview, raw, Path(label["path"]), proxy)
                variants[fps] = (preview, proxy)

            report = compare_sampling_rates(
                raw=raw,
                outcome=Path(label["path"]),
                baseline_preview=variants[2][0],
                baseline_proxy=variants[2][1],
                candidate_preview=variants[10][0],
                candidate_proxy=variants[10][1],
            )

            baseline = report["variants"]["baseline"]
            candidate = report["variants"]["candidate"]
            self.assertEqual(report["status"], "PASS_DATA_VARIANTS_VERIFIED")
            self.assertEqual(baseline["fps"], 2)
            self.assertEqual(candidate["fps"], 10)
            self.assertEqual(
                baseline["source_hashes"]["raw_manifest_sha256"],
                candidate["source_hashes"]["raw_manifest_sha256"],
            )
            self.assertTrue(baseline["source_frame_extraction_verified"])
            self.assertTrue(candidate["source_frame_extraction_verified"])
            self.assertGreater(candidate["proxy_transition_count"],
                               baseline["proxy_transition_count"])
            self.assertEqual(candidate["proxy_target_offset_ns"], 100_000_000)
            self.assertEqual(baseline["proxy_target_offset_ns"], 500_000_000)
            self.assertFalse(report["scope"]["model_training_run"])
            self.assertFalse(report["scope"]["formal_training_eligible"])
            self.assertFalse(report["scope"]["hardware_control_allowed"])


if __name__ == "__main__":
    unittest.main()
