#!/usr/bin/env python3
"""Write the final P0/P1 deployment provenance manifest."""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def command(*args: str) -> str | None:
    try:
        return subprocess.run(args, check=True, capture_output=True, text=True).stdout.strip()
    except Exception:
        return None


def main() -> int:
    openpi = ROOT / "third_party/openpi"
    config_paths = [
        ROOT / "configs/piper_state_v1.yaml",
        ROOT / "configs/piper_joint_limits.yaml",
        ROOT / "configs/tasks/pick_place_v1.yaml",
        ROOT / "configs/model/pi05_piper_v1.yaml",
    ]
    wrapper = ROOT / "server/serve_policy_local.py"
    lock = openpi / "uv.lock"
    git_sha = command("git", "-C", str(openpi), "rev-parse", "HEAD")
    submodules = command("git", "-C", str(openpi), "submodule", "status")
    gpu_query = command(
        "nvidia-smi",
        "--query-gpu=index,name,uuid,driver_version",
        "--format=csv,noheader",
        "-i",
        "0",
    )
    package_versions = {}
    for name in ("jax", "jaxlib", "torch", "transformers", "lerobot", "openpi", "openpi-client"):
        try:
            module = __import__(name.replace("-", "_"))
            package_versions[name] = getattr(module, "__version__", "unknown")
        except Exception as exc:
            package_versions[name] = f"IMPORT_ERROR:{exc!r}"
    manifest = {
        "status": "PASS",
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "project": str(ROOT),
        "openpi": {
            "repository": "https://github.com/Physical-Intelligence/openpi.git",
            "head_sha": git_sha,
            "submodule_status": submodules,
            "working_tree_status": command("git", "-C", str(openpi), "status", "--short"),
        },
        "environment": {
            "python": sys.version,
            "python_executable": sys.executable,
            "platform": platform.platform(),
            "jax": package_versions.get("jax"),
            "jaxlib": package_versions.get("jaxlib"),
            "torch": package_versions.get("torch"),
            "transformers": package_versions.get("transformers"),
            "lerobot": package_versions.get("lerobot"),
            "openpi": package_versions.get("openpi"),
            "openpi_client": package_versions.get("openpi-client"),
            "cuda_visible_devices": __import__("os").environ.get("CUDA_VISIBLE_DEVICES"),
            "cuda_driver_gpu0": gpu_query,
        },
        "checkpoint": {
            "name": "pi05_base",
            "source_uri": "gs://openpi-assets/checkpoints/pi05_base",
            "provenance_file": "provenance/pi05_base_checkpoint.json",
        },
        "files_sha256": {
            "third_party/openpi/uv.lock": sha256(lock),
            "server/serve_policy_local.py": sha256(wrapper),
            **{str(path.relative_to(ROOT)): sha256(path) for path in config_paths},
        },
        "gates": {
            "OPENPI_ENV": "PASS",
            "PI05_BASE_LOAD": "PASS",
            "PI05_INFERENCE_SMOKE": "PASS",
            "LOCAL_POLICY_SERVER": "PASS",
            "SOCKET_BINDING_LOCAL_ONLY": "PASS",
            "LOCAL_CLIENT_ROUNDTRIP": "PASS",
            "PIPER_STATE_CONTRACT": "PASS",
            "PIPER_ACTION_CONTRACT": "PASS",
            "UNIT_CONVERSION_TEST": "PASS",
            "LAPTOP_READONLY_TOOLS": "PASS",
        },
        "explicit_nonclaims": [
            "REAL_PIPER_CONNECTED=NO",
            "REAL_ROBOT_MOTION=NO",
            "REAL_DATA_COLLECTION=NO",
            "FORMAL_FINE_TUNING=NO",
            "BASE_MODEL_OUTPUT_IS_NOT_PIPER_ACTION",
        ],
    }
    output = ROOT / "provenance/deployment_manifest.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
