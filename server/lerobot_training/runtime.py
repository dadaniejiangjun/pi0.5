"""Isolated, local-only runtime settings for official LeRobot training."""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
import os
import platform
import sys
from pathlib import Path

VERSION = "0.6.1"
COMMIT = "7e241bd630a3719a56157a497ce5d08f244784f1"

def configure(root: str | Path) -> dict[str, str]:
    root = Path(root).resolve()
    values = {"HF_HOME": str(root / "hf"), "HF_LEROBOT_HOME": str(root / "datasets"),
              "HF_HUB_OFFLINE": "1", "HF_DATASETS_OFFLINE": "1", "WANDB_MODE": "disabled",
              "HF_HUB_DISABLE_TELEMETRY": "1", "OMP_NUM_THREADS": "2", "MKL_NUM_THREADS": "2"}
    for k, v in values.items():
        os.environ[k] = v
    for name in ("hf", "datasets", "tmp"):
        (root / name).mkdir(parents=True, exist_ok=True)
    os.environ["TEMP"] = os.environ["TMP"] = str(root / "tmp")
    return values

def preflight() -> dict:
    import torch
    from lerobot.scripts.lerobot_train import train
    from lerobot.processor import PolicyProcessorPipeline
    packages = {p: importlib.metadata.version(p) for p in
                ("lerobot", "torch", "torchvision", "datasets", "accelerate", "transformers")}
    if packages["lerobot"] != VERSION:
        raise RuntimeError(f"requires pinned LeRobot {VERSION}")
    return {"status": "RUNTIME_READY", "python": sys.version, "executable": sys.executable,
            "platform": platform.platform(), "packages": packages, "upstream_commit": COMMIT,
            "cuda_available": torch.cuda.is_available(), "hardware_accessed": False,
            "trainer": train.__module__, "processor": PolicyProcessorPipeline.__module__}

def write_json(path: str | Path, value: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")

def file_hash(path: str | Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()
