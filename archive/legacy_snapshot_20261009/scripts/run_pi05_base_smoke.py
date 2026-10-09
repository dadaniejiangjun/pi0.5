#!/usr/bin/env python3
"""Official pi05_base + pi05_aloha load and inference smoke.

This intentionally exercises the official ALOHA policy contract only. It is
not a PiPER policy test and never connects to a robot.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import traceback

import numpy as np

from openpi.policies import aloha_policy
from openpi.policies import policy_config
from openpi.training import config as training_config


DEFAULT_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_base"


def _gpu_memory_mib() -> int | None:
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=memory.used",
                "--format=csv,noheader,nounits",
                "-i",
                "0",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        return int(result.stdout.strip().splitlines()[0])
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("reports/pi05_base_inference_smoke.json"))
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--inferences", type=int, default=3)
    args = parser.parse_args()

    report: dict = {
        "status": "RUNNING",
        "policy_scope": "official_pi05_aloha_base_load_only",
        "piper_policy_claim": False,
        "checkpoint": args.checkpoint,
        "config": "pi05_aloha",
        "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
        "openpi_data_home": os.environ.get("OPENPI_DATA_HOME"),
        "inference_count_requested": args.inferences,
        "load_seconds": None,
        "latencies_ms": [],
        "action_shapes": [],
        "nonfinite_counts": [],
        "observed_gpu_memory_mib": [],
    }

    try:
        load_start = time.perf_counter()
        policy = policy_config.create_trained_policy(
            training_config.get_config("pi05_aloha"),
            args.checkpoint,
        )
        report["load_seconds"] = time.perf_counter() - load_start
        report["metadata_keys"] = sorted(policy.metadata.keys())

        observation = aloha_policy.make_aloha_example()
        for index in range(args.inferences):
            start = time.perf_counter()
            result = policy.infer(observation)
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            actions = np.asarray(result["actions"])
            report["latencies_ms"].append(elapsed_ms)
            report["action_shapes"].append(list(actions.shape))
            report["nonfinite_counts"].append(int(np.size(actions) - np.isfinite(actions).sum()))
            report["observed_gpu_memory_mib"].append(_gpu_memory_mib())

        warm = report["latencies_ms"][1:]
        report["cold_latency_ms"] = report["latencies_ms"][0]
        report["warm_latency_ms"] = {
            "median": float(np.median(warm)) if warm else None,
            "p90": float(np.percentile(warm, 90)) if warm else None,
            "values": warm,
        }
        report["peak_observed_gpu_memory_mib"] = max(
            (x for x in report["observed_gpu_memory_mib"] if x is not None),
            default=None,
        )
        report["status"] = "PASS" if (
            args.inferences >= 3
            and all(count == 0 for count in report["nonfinite_counts"])
            and len(set(tuple(shape) for shape in report["action_shapes"])) == 1
        ) else "FAIL"
    except Exception as exc:
        report["status"] = "FAIL"
        report["error"] = repr(exc)
        report["traceback"] = traceback.format_exc()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"WROTE {args.output}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
