#!/usr/bin/env python3
"""Make the official smoke report's observed/qualified metadata explicit."""

from __future__ import annotations

import json
from pathlib import Path


def main() -> int:
    path = Path("reports/pi05_base_inference_smoke.json")
    report = json.loads(path.read_text(encoding="utf-8"))
    report["load_wall_time_seconds"] = report.get("load_seconds")
    report["device"] = "cuda:0"
    report["dtype"] = "bfloat16"
    report["observation_contract"] = {
        "prompt_accepted": True,
        "image_inputs_accepted": True,
        "state_accepted": True,
        "source": "official openpi.policies.aloha_policy.make_aloha_example",
        "semantic_scope": "official ALOHA sanity only",
    }
    report["jax_compile_time_ms"] = None
    report["first_inference_compile_plus_infer_ms"] = report.get("cold_latency_ms")
    report["jax_compile_time_note"] = (
        "The first inference includes JAX/XLA compilation and execution; this smoke did not isolate compilation from the first call."
    )
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("status", "device", "dtype", "first_inference_compile_plus_infer_ms", "jax_compile_time_ms")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
