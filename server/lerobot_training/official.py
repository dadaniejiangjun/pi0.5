"""Call the unmodified official trainer with project train-only statistics."""
import os
import json
import sys
import hashlib
import torch
from .processors import train_statistics

def main():
    torch.set_num_threads(int(os.environ.get("OMP_NUM_THREADS", "2")))
    import lerobot.scripts.lerobot_train as upstream
    original = upstream.make_train_eval_datasets
    root = os.environ["PIPER_TRAINING_DATA_ROOT"]
    software_only = os.environ.get("PIPER_SOFTWARE_ONLY") == "1"
    stats = train_statistics(root, software_only=software_only)
    def project_datasets(cfg):
        train, evaluation = original(cfg)
        from pathlib import Path
        manifest = json.loads((Path(root) / "project_training_manifest.json").read_text())
        if manifest.get("source_kind") == "verified_online_feedback_proxy":
            # Every H16 training window remains inside a verified continuous segment.
            cfg.policy.drop_n_last_frames = 15
        # Upstream metadata can aggregate held-out episodes. Replace only the
        # adapter result, without changing dataset files or training algorithm.
        train.meta.stats = stats
        if evaluation is not None:
            evaluation.meta.stats = stats
        return train, evaluation
    upstream.make_train_eval_datasets = project_datasets
    original_load = upstream.load_training_state
    def verified_load(*args, **kwargs):
        result = original_load(*args, **kwargs)
        from safetensors.torch import load_file
        from pathlib import Path
        cp = Path(args[0])
        saved = load_file(cp / "training_state/rng_state.safetensors")
        evidence = {"saved_step": result[0],
                    "torch_rng_restored_exact": torch.equal(torch.get_rng_state(), saved["torch_rng_state"])}
        target = os.environ.get("PIPER_RESUME_EVIDENCE")
        if target:
            Path(target).write_text(json.dumps(evidence, indent=2) + "\n")
        if not evidence["torch_rng_restored_exact"]:
            raise RuntimeError("saved Torch RNG was not restored")
        return result
    upstream.load_training_state = verified_load
    original_update = upstream.update_policy
    update_evidence = []
    update_number = 0
    evidence_stride = max(1, int(os.environ.get("PIPER_UPDATE_EVIDENCE_STRIDE", "1")))
    def witnessed_update(*args, **kwargs):
        nonlocal update_number
        update_number += 1
        policy = args[1] if len(args) > 1 else kwargs["policy"]
        selected = [(n, p) for n, p in policy.named_parameters() if p.requires_grad]
        if update_number == 1 and os.environ.get("PIPER_REQUIRE_LORA") == "1":
            if not selected or any("lora_" not in n for n, _ in selected):
                raise RuntimeError("LoRA run has unexpected trainable base parameters")
            from pathlib import Path
            Path(os.environ["PIPER_UPDATE_EVIDENCE"]).with_name("lora_parameters.json").write_text(
                json.dumps({"trainable_parameters": sum(p.numel() for _, p in selected),
                            "names": [n for n, _ in selected], "lora_only": True}, indent=2))
        sampled = update_number == 1 or update_number % evidence_stride == 0
        selected = selected[:4] + selected[-4:]
        def digest(p):
            return hashlib.sha256(p.detach().cpu().contiguous().view(torch.uint8).numpy().tobytes()).hexdigest()
        before = {n: digest(p) for n, p in selected} if sampled else {}
        result = original_update(*args, **kwargs)
        after = {n: digest(p) for n, p in selected} if sampled else {}
        metrics = result[0]
        path = os.environ.get("PIPER_UPDATE_EVIDENCE")
        if path:
            from pathlib import Path
            with Path(path).with_suffix(".jsonl").open("a") as stream:
                stream.write(json.dumps({"step_in_process": update_number,
                    "loss": float(metrics.loss.val), "grad_norm": float(metrics.grad_norm.val)}) + "\n")
        if not sampled:
            return result
        update_evidence.append({"step_in_process": update_number,
                                "loss": float(metrics.loss.val), "grad_norm": float(metrics.grad_norm.val),
                                "sampled_parameters_before": before, "sampled_parameters_after": after,
                                "changed_sampled_parameters": [n for n in before if before[n] != after[n]]})
        path = os.environ.get("PIPER_UPDATE_EVIDENCE")
        if path:
            from pathlib import Path
            Path(path).write_text(json.dumps(update_evidence, indent=2) + "\n")
        return result
    upstream.update_policy = witnessed_update
    original_save = upstream.save_checkpoint
    def witnessed_save(*args, **kwargs):
        result = original_save(*args, **kwargs)
        from .trainer import seal_checkpoint
        checkpoint = kwargs.get("checkpoint_dir", args[0] if args else None)
        seal_checkpoint(checkpoint, root)
        return result
    upstream.save_checkpoint = witnessed_save
    if os.name == "nt":
        # Windows unprivileged users cannot create directory symlinks. Keep
        # numbered upstream checkpoints untouched and expose a JSON pointer.
        def last_pointer(checkpoint):
            path = checkpoint.parent / "last.json"
            path.write_text(json.dumps({"checkpoint": checkpoint.name}) + "\n")
            return path
        upstream.update_last_checkpoint = last_pointer
    if any("pi05" in arg for arg in sys.argv) or os.environ.get("PIPER_POLICY_TYPE") == "pi05":
        from .pi05 import strict_pi05_loading
        from pathlib import Path
        with strict_pi05_loading() as evidence:
            upstream.train()
        path = Path(os.environ["PIPER_PI05_LOAD_EVIDENCE"])
        path.write_text(json.dumps(evidence, indent=2) + "\n")
    else:
        upstream.train()

if __name__ == "__main__":
    main()
