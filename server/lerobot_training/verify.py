"""Compare actual official checkpoint tensors for training and exact resume."""
from pathlib import Path
import json
from safetensors.torch import load_file
import torch
from .runtime import write_json, file_hash

def compare_resume(first, continuous, resumed, output):
    a, b, c = map(Path, (first, continuous, resumed))
    before = load_file(a / "pretrained_model/model.safetensors")
    after = load_file(b / "pretrained_model/model.safetensors")
    restored = load_file(c / "pretrained_model/model.safetensors")
    changed = [key for key in after if not torch.equal(before[key], after[key])]
    differences = [key for key in after if not torch.equal(after[key], restored[key])]
    optim = "training_state/optimizer_state.safetensors"
    opt_b, opt_c = load_file(b / optim), load_file(c / optim)
    opt_equal = set(opt_b) == set(opt_c) and all(torch.equal(opt_b[k], opt_c[k]) for k in opt_b)
    scheduler_equal = json.loads((b / "training_state/scheduler_state.json").read_text()) == json.loads((c / "training_state/scheduler_state.json").read_text())
    step_b = json.loads((b / "training_state/training_step.json").read_text())
    step_c = json.loads((c / "training_state/training_step.json").read_text())
    rng_b, rng_c = load_file(b / "training_state/rng_state.safetensors"), load_file(c / "training_state/rng_state.safetensors")
    rng_equal = {k: torch.equal(rng_b[k], rng_c[k]) for k in rng_b}
    result = {"status": "PASS_SOFTWARE_ONLY" if changed and not differences and opt_equal and scheduler_equal and step_b == step_c else "FAILED",
              "changed_tensor_count": len(changed), "resume_unequal_tensors": differences,
              "optimizer_exact": opt_equal, "scheduler_exact": scheduler_equal,
              "training_steps_equal": step_b == step_c, "final_step": step_b,
              "rng_tensor_equality": rng_equal,
              "note": "Torch final RNG differs after loader iterator reconstruction. This dropout-free test proves exact model/optimizer/scheduler continuation, not general stochastic trajectory equivalence.",
              "model_sha256": file_hash(b / "pretrained_model/model.safetensors"),
              "claim": "synthetic ACT short training; no real robot policy quality claim"}
    write_json(output, result)
    if result["status"] == "FAILED": raise ValueError(result)
    return result

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    for name in ("first", "continuous", "resumed", "output"): p.add_argument("--" + name, required=True)
    print(json.dumps(compare_resume(**vars(p.parse_args())), indent=2))
