#!/usr/bin/env python3
"""Add the uv and final host-driver fields to the environment audit."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path


def main() -> int:
    path = Path("reports/environment_audit.json")
    report = json.loads(path.read_text(encoding="utf-8"))
    uv = shutil.which("uv") or "/home/mingkai/miniconda3/envs/lwbench/bin/uv"
    try:
        uv_version = subprocess.run([uv, "--version"], check=True, capture_output=True, text=True).stdout.strip()
    except Exception as exc:
        uv_version = f"UNAVAILABLE:{exc!r}"
    smi = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,name,uuid,memory.total,memory.used,utilization.gpu", "--format=csv,noheader"],
        check=False,
        capture_output=True,
        text=True,
    )
    report["uv_version"] = uv_version
    report["cuda_backend"] = "JAX CUDA12 plugin; validated official serving path"
    report["gpu_visible_host_inventory"] = smi.stdout.strip().splitlines()
    report["gpu_inventory_note"] = "Host inventory lists physical GPUs; compute process was constrained with CUDA_VISIBLE_DEVICES=0."
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "uv_version": uv_version, "cuda_backend": report["cuda_backend"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
