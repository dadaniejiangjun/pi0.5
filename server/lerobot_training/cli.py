"""Project entry point for the pinned official LeRobot training lifecycle."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
from .runtime import configure, preflight, write_json

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--runtime", default="cache/codex/lerobot_training_20261019/runtime")
    s = p.add_subparsers(dest="command", required=True)
    a = s.add_parser("preflight"); a.add_argument("--output")
    a = s.add_parser("prepare"); a.add_argument("--output", required=True)
    a.add_argument("--episodes", type=int, default=4); a.add_argument("--frames", type=int, default=32)
    a.add_argument("--source"); a.add_argument("--repo-id", default="local/piper_review_migration")
    a.add_argument("--route", choices=("migrate-review-copy", "proxy-from-feedback"))
    a.add_argument("--legacy-python", default=os.environ.get(
        "PIPER_LEGACY_PYTHON", r"E:\anaconda3\envs\pi05_lerobot_dataset\python.exe"))
    for name in ("train", "resume"):
        a = s.add_parser(name); a.add_argument("--dataset", required=True); a.add_argument("--output", required=True)
        a.add_argument("--steps", type=int, default=4); a.add_argument("--device", default="cpu")
        a.add_argument("--policy", choices=("act", "pi05"), default="act"); a.add_argument("--assets")
        a.add_argument("--software-only", action="store_true")
        a.add_argument("--lora", action="store_true")
        a.add_argument("--lora-rank", type=int, default=16)
        a.add_argument("--batch-size", type=int, default=1)
        a.add_argument("--checkpoint-interval", type=int, default=1,
                       help="PI05 checkpoint save interval; final step is always saved")
        if name == "resume": a.add_argument("--checkpoint", required=True)
    a = s.add_parser("predict"); a.add_argument("--dataset", required=True); a.add_argument("--output", required=True)
    a.add_argument("--checkpoint", required=True); a.add_argument("--software-only", action="store_true")
    a.add_argument("--policy", choices=("act", "pi05"), default="act"); a.add_argument("--device", default="cpu")
    a = s.add_parser("report"); a.add_argument("--checkpoint", required=True); a.add_argument("--output", required=True)
    args = p.parse_args()
    configure(args.runtime)
    if args.command == "preflight":
        result = preflight()
        if args.output: write_json(args.output, result)
    elif args.command == "prepare":
        from .data import prepare_synthetic, migrate_review_copy, prepare_proxy_from_feedback
        if args.route == "proxy-from-feedback":
            if not args.source:
                p.error("prepare --route proxy-from-feedback requires --source <v2.1 capture dataset>")
            result = prepare_proxy_from_feedback(args.source, args.output, repo_id=args.repo_id,
                                                 legacy_python=args.legacy_python)
        elif args.source:
            if args.route not in (None, "migrate-review-copy"):
                p.error("--route and --source are incompatible")
            result = migrate_review_copy(args.source, args.output, repo_id=args.repo_id)
        else:
            if args.route is not None:
                p.error("--route requires --source")
            result = {"manifest": str(prepare_synthetic(args.output, episodes=args.episodes, frames=args.frames))}
    elif args.command in ("train", "resume"):
        from .trainer import train_act
        if args.policy == "pi05":
            if not args.assets: p.error("pi05 requires --assets with verified local base assets")
            from .pi05_run import train_pi05
            result = train_pi05(args.dataset, args.output, args.assets, steps=args.steps, device=args.device,
                                software_only=args.software_only, resume_checkpoint=getattr(args, "checkpoint", None),
                                checkpoint_interval=args.checkpoint_interval,
                                lora=args.lora, lora_rank=args.lora_rank, batch_size=args.batch_size)
        else:
            result = train_act(args.dataset, args.output, steps=args.steps, device=args.device,
                               software_only=args.software_only, resume_checkpoint=getattr(args, "checkpoint", None))
    elif args.command == "predict":
        from .trainer import predict
        if args.policy == "pi05":
            from .pi05_run import predict_pi05
            result = predict_pi05(args.checkpoint, args.dataset, args.output,
                                  device=args.device, software_only=args.software_only)
        else:
            result = predict(args.checkpoint, args.dataset, args.output, software_only=args.software_only)
    else:
        from .trainer import validate_checkpoint
        result = {"checkpoint": str(Path(args.checkpoint).resolve()), "files": validate_checkpoint(args.checkpoint)}
        write_json(args.output, result)
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    main()
