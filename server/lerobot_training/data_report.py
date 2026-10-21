"""Recheck migrated source fingerprints and emit compact local evidence."""
from __future__ import annotations
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from .data import MANIFEST, admit_dataset, fingerprint, sha256, write_json


def report(output: str | Path) -> dict:
    junit_path = Path("validation/lerobot_training_data_tests_20261019.xml")
    suite = ET.parse(junit_path).getroot().find("testsuite")
    assert suite is not None and suite.get("failures") == "0" and suite.get("errors") == "0"
    cases = [
        ("proxy67", "cache/training_bridge/datasets/local/proxybridge_0cd272a0_20261018_r2",
         "cache/codex/lerobot_training_20261019/proxy67_v3_r2"),
        ("kernel600", "gamepad_capture/data/lerobot/20261017T113409Z_72321bfd_kernel_accepted",
         "cache/codex/lerobot_training_20261019/kernel600_v3_r2")]
    results = []
    for name, source, target in cases:
        manifest = json.loads((Path(target) / MANIFEST).read_text())
        assert fingerprint(source) == manifest["source_fingerprint"]
        for relative, digest in manifest["dataset_files"].items():
            assert sha256(Path(target) / relative) == digest, relative
        try:
            admit_dataset(target, software_only=True)
        except ValueError as error:
            reason = str(error)
        else:
            raise AssertionError("review data unexpectedly admitted")
        results.append({"name": name, "source": source, "target": target,
                        "readback_rows": manifest["readback_rows"],
                        "all_field_hashes_match": manifest["all_field_hashes_match"],
                        "source_files": len(manifest["source_fingerprint"]),
                        "source_unchanged": True, "qualification": False,
                        "rejected_by_training_admission": reason,
                        "manifest_sha256": sha256(Path(target) / MANIFEST)})
    synthetic_root = Path("cache/codex/lerobot_training_20261019/synthetic_v1")
    synthetic = admit_dataset(synthetic_root, software_only=True)
    result = {"schema": "piper_lerobot_data_validation_v1", "date": "2026-10-19",
              "status": "PASS_SOFTWARE_ONLY", "migrations": results,
              "synthetic": {"root": str(synthetic_root), "episodes": 4, "frames": 128,
                            "manifest_sha256": sha256(synthetic_root / MANIFEST),
                            "splits": synthetic["splits"], "training_eligible": False,
                            "software_training_eligible": True},
              "processor_test": {"command": "pytest tests/server/test_lerobot_training_data.py -q",
                                 "tests_passed": int(suite.get("tests")),
                                 "junit_sha256": sha256(junit_path), "fresh_process_roundtrip": True,
                                 "corrupt_config_rejected": True,
                                 "train_only_statistics_verified": True},
              "hardware_accessed": False, "historical_data_trained": False,
              "limitations": ["PNG migration uses more storage than MP4",
                              "Nominal physical units and camera roles are not calibrated by this test",
                              "Official trainer can still pad tails; strict helper is separate",
                              "Windows TorchCodec DLL unavailable; PyAV decoding used"]}
    write_json(Path(output), result)
    return result


if __name__ == "__main__":
    print(json.dumps(report("validation/lerobot_training_data_20261019.json"), indent=2))
