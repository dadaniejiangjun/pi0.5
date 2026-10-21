"""Durable, isolated LoRA job using the project's official LeRobot launcher."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--assets", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--runtime", required=True, type=Path)
    parser.add_argument("--status", required=True, type=Path)
    parser.add_argument("--steps", type=int, default=30000)
    parser.add_argument("--save-freq", type=int, default=5000)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--gpu", type=int, default=0)
    args = parser.parse_args()
    if args.status.exists() or args.output.exists():
        raise FileExistsError("job output/status already exists; use a new run name")
    status = {"status": "STARTING", "pid": os.getpid(), "steps": args.steps,
              "checkpoint_interval": args.save_freq, "gpu": args.gpu,
              "dataset": str(args.dataset.resolve()), "output": str(args.output.resolve()),
              "method": "official PI05 PEFT LoRA", "lora_rank": 16,
              "hardware_accessed": False, "started_unix_s": time.time()}
    def save(**updates):
        status.update(updates)
        args.status.write_text(json.dumps(status, indent=2) + "\n")
    save()
    try:
        from .data import admit_dataset
        manifest = admit_dataset(args.dataset)
        usage = subprocess.check_output(["nvidia-smi", f"--id={args.gpu}",
            "--query-gpu=memory.used", "--format=csv,noheader,nounits"], text=True)
        if int(usage.strip()) > 1000:
            raise RuntimeError("selected GPU is occupied; no processes will be stopped")
        env = os.environ.copy()
        env.update(CUDA_VISIBLE_DEVICES=str(args.gpu), TOKENIZERS_PARALLELISM="false")
        cmd = [sys.executable, "-u", "-B", "-m", "server.lerobot_training.cli",
               "--runtime", str(args.runtime), "train", "--dataset", str(args.dataset),
               "--output", str(args.output), "--assets", str(args.assets),
               "--policy", "pi05", "--device", "cuda", "--lora",
               "--steps", str(args.steps), "--checkpoint-interval", str(args.save_freq),
               "--batch-size", str(args.batch_size)]
        save(status="TRAINER_LAUNCHED", argv=cmd, splits=manifest["splits"])
        result = subprocess.run(cmd, env=env, check=False)
        save(status="COMPLETE" if result.returncode == 0 else "FAILED", exit_code=result.returncode,
             finished_unix_s=time.time())
        return result.returncode
    except Exception as exc:
        save(status="FAILED", error=f"{type(exc).__name__}: {exc}")
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
