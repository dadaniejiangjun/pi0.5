"""Mode-specific, read-only offline training readiness checks."""

from __future__ import annotations

import json
import hashlib
from pathlib import Path

from openpi.training import config as openpi_config

from server.training.piper_openpi_config import PiperOpenPIDataConfig
from server.training.public_data_contract import inspect_public_acceptance
from server.training.action_target_contract import (
    has_raw_command_action_semantics,
    is_verified_raw_command_episode,
)

MODES = ("synthetic-debug", "public-data", "local-tiny-fit", "formal")


def _json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _sha256(path: Path) -> str | None:
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def inspect(root: Path, mode: str = "formal", *, dataset_path: Path | None = None,
            acceptance_path: Path | None = None) -> dict:
    if mode not in MODES:
        raise ValueError(f"unknown training mode: {mode}")
    root = Path(root).resolve()
    dataset_path = Path(dataset_path).resolve() if dataset_path else root / "datasets/lerobot/local/piper_v1"
    if acceptance_path is not None:
        acceptance_path = Path(acceptance_path).resolve()
    elif mode in ("local-tiny-fit", "formal"):
        acceptance_path = dataset_path / "provenance/acceptance_manifest.json"
    else:
        acceptance_path = root / "datasets/manifests/piper_v1_acceptance.json"
    info = _json(dataset_path / "meta/info.json")
    provenance = _json(dataset_path / "provenance/conversion_manifest.json")
    acceptance = _json(acceptance_path)
    acceptance_hash_matches = (
        _sha256(acceptance_path) is not None
        and _sha256(acceptance_path) == provenance.get("acceptance_manifest_sha256")
        and provenance.get("acceptance_manifest") == "provenance/acceptance_manifest.json"
        and _sha256(dataset_path / "provenance/acceptance_manifest.json")
        == provenance.get("acceptance_manifest_sha256")
    )
    episodes = acceptance.get("episodes")
    if not isinstance(episodes, list):
        episodes = []
    valid = [row for row in episodes if is_verified_raw_command_episode(row)]
    registered = any(cfg.name == "pi05_piper_v1" and isinstance(cfg.data, PiperOpenPIDataConfig)
                     for cfg in openpi_config._CONFIGS)
    stats = _json(root / "assets/pi05_piper_v1/piper/norm_stats.json").get("norm_stats")
    stats_valid = isinstance(stats, dict) and all(
        key in stats and isinstance(stats[key], dict)
        and all(isinstance(stats[key].get(field), list) and len(stats[key][field]) == 7
                for field in ("mean", "std", "q01", "q99"))
        for key in ("state", "actions")
    )
    dataset_valid = isinstance(info.get("total_episodes"), int) and info["total_episodes"] > 0
    checks: dict[str, bool]
    if mode == "synthetic-debug":
        checks = {"synthetic_dataset": dataset_valid and provenance.get("synthetic_fixture") is True,
                  "explicit_synthetic_action_semantics": acceptance.get("action_semantics") == "synthetic_7d_test_only"}
    elif mode == "public-data":
        checks = {"public_dataset": dataset_valid and provenance.get("synthetic_fixture") is False,
                  **inspect_public_acceptance(acceptance, provenance)}
    else:
        minimum = 1 if mode == "local-tiny-fit" else 20
        checks = {
            "accepted_real_demonstrations": acceptance.get("schema") == "piper_real_demo_acceptance_v1"
            and len(valid) >= minimum and len(valid) == len(episodes),
            "piper_lerobot_dataset": dataset_valid and info["total_episodes"] >= minimum
            and provenance.get("synthetic_fixture") is False
            and provenance.get("command_target_provenance_verified") is True
            and provenance.get("action_target_source") == "observed_piper_can_command_frames",
            "dataset_identity": provenance.get("repo_id") == acceptance.get("repo_id")
            and bool(provenance.get("source_map_sha256"))
            and acceptance_hash_matches
            and len(episodes) == info.get("total_episodes")
            and len(provenance.get("source_episodes", [])) == info.get("total_episodes"),
            "action_semantics": has_raw_command_action_semantics(acceptance),
        }
        if mode == "formal":
            checks["piper_specific_norm_stats"] = stats_valid
            checks["official_training_entry_registered"] = registered
    return {
        "mode": mode,
        "status": "READY_FOR_SEPARATE_REVIEW" if all(checks.values()) else "BLOCKED",
        "checks": checks,
        "dataset_path": str(dataset_path),
        "acceptance_path": str(acceptance_path),
        "training_started": False,
        "hardware_control_allowed": False,
        "note": "Readiness is mode-specific; a one-episode fit is train-only with no validation and offline training never grants robot control.",
    }


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=MODES, default="formal")
    parser.add_argument("--dataset-path", type=Path)
    parser.add_argument("--acceptance-path", type=Path)
    args = parser.parse_args()
    report = inspect(Path(__file__).resolve().parents[2], args.mode,
                     dataset_path=args.dataset_path, acceptance_path=args.acceptance_path)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "READY_FOR_SEPARATE_REVIEW" else 1


if __name__ == "__main__":
    raise SystemExit(main())
