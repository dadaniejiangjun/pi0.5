"""Bounded official PI05 training and offline prediction integration."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

from .data import MANIFEST, admit_dataset
from .pi05 import asset_manifest, config_overrides, load_pi05_strict, make_pi05_config
from .runtime import file_hash, write_json
from .trainer import seal_checkpoint, validate_checkpoint


def convert_cached_tokenizer(spm_file: str | Path, output: str | Path) -> dict:
    """Convert the existing official OpenPI SPM; never fetch gated HF assets.

    This is explicitly a local conversion, not a claim to have loaded Google's
    gated AutoTokenizer snapshot. Acceptance compares actual token IDs against
    the unchanged source SPM including PI05 prompt/state and padding behavior.
    """
    from types import SimpleNamespace
    import sentencepiece as spm
    from tokenizers.processors import TemplateProcessing
    from transformers import AutoTokenizer, PreTrainedTokenizerFast
    from transformers.convert_slow_tokenizer import GemmaConverter, GemmaSentencePieceExtractor

    source, output = Path(spm_file).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    sp = spm.SentencePieceProcessor(model_file=str(source))
    args = SimpleNamespace(vocab_file=str(source), pad_token=sp.id_to_piece(sp.pad_id()),
                           eos_token=sp.id_to_piece(sp.eos_id()), bos_token=sp.id_to_piece(sp.bos_id()))
    # Transformers 5.5's Gemma extractor still reads self.sp although its base
    # constructor now creates only self.proto. Supply the unchanged SPM through
    # a local subclass; never patch the installed transformers source.
    class CachedExtractor(GemmaSentencePieceExtractor):
        def __init__(self, model):
            super().__init__(model)
            self.sp = spm.SentencePieceProcessor(model_file=model)
    class CachedConverter(GemmaConverter):
        SpmExtractor = CachedExtractor
    tokenizer = CachedConverter(args).converted()
    tokenizer.post_processor = TemplateProcessing(single=f"{args.bos_token} $A",
        pair=f"{args.bos_token} $A $B", special_tokens=[(args.bos_token, sp.bos_id())])
    hf = PreTrainedTokenizerFast(tokenizer_object=tokenizer, pad_token=args.pad_token,
                                eos_token=args.eos_token, bos_token=args.bos_token,
                                unk_token=sp.id_to_piece(sp.unk_id()), padding_side="right")
    prompts = ["", "pick up the red block", "Task: pick up the block, State: 0 127 255;\nAction: ",
               "Task: move slowly, State: " + " ".join(map(str, range(32))) + ";\nAction: ",
               "  leading and trailing  ", "中文任务：拿起杯子", "\tline\nnext", "emoji 🦾 and 1.25"]
    for prompt in prompts:
        expected = sp.encode(prompt, add_bos=True)
        actual = hf.encode(prompt, add_special_tokens=True)
        if actual != expected:
            raise ValueError(f"Cached tokenizer conversion token mismatch for {prompt!r}")
    # Explicit special-token and batch padding checks, including empty prompt.
    if (hf.bos_token_id, hf.eos_token_id, hf.pad_token_id) != (sp.bos_id(), sp.eos_id(), sp.pad_id()):
        raise ValueError("Cached tokenizer special IDs differ")
    batched = hf(prompts, padding="max_length", max_length=200, truncation=True)
    for i, prompt in enumerate(prompts):
        expected = sp.encode(prompt, add_bos=True)[:200]
        if batched["input_ids"][i] != expected + [sp.pad_id()] * (200 - len(expected)):
            raise ValueError("Cached tokenizer padding differs")
        if batched["attention_mask"][i] != [1] * len(expected) + [0] * (200 - len(expected)):
            raise ValueError("Cached tokenizer attention mask differs")
    output.mkdir(parents=True)
    hf.save_pretrained(output)
    reloaded = AutoTokenizer.from_pretrained(output, local_files_only=True)
    for prompt in prompts:
        if reloaded.encode(prompt) != sp.encode(prompt, add_bos=True):
            raise ValueError("Saved cached tokenizer round-trip differs")
    report = {"status": "PASS_LOCAL_SPM_CONVERSION", "source_spm": str(source),
              "source_sha256": file_hash(source), "source_vocab_size": sp.vocab_size(),
              "converted_vocab_size": len(hf), "prompt_cases": len(prompts),
              "bos_id": hf.bos_token_id, "eos_id": hf.eos_token_id, "pad_id": hf.pad_token_id,
              "token_ids_equal": True, "padding_equal": True, "reload_equal": True,
              "claim": "existing official OpenPI cache converted locally, not gated HF snapshot"}
    write_json(output / "project_tokenizer_conversion.json", report)
    return report


def prepare_base(source: str | Path, output: str | Path, tokenizer: str | Path) -> dict:
    """Create an independent project config and processor view of public weights."""
    source, output, tokenizer = Path(source).resolve(), Path(output).resolve(), Path(tokenizer).resolve()
    manifest = asset_manifest(source, hash_weights=True)
    if output.exists():
        raise FileExistsError(output)
    if not (tokenizer / "tokenizer.json").is_file():
        raise ValueError("verified local AutoTokenizer assets missing")
    output.mkdir(parents=True)
    config = make_pi05_config("cuda")
    config.save_pretrained(output)
    # Same-filesystem hard link avoids duplicating immutable 14.5 GB base weights.
    os.link(source / "model.safetensors", output / "model.safetensors")
    for name in ("policy_preprocessor.json", "policy_postprocessor.json"):
        data = json.loads((source / name).read_text())
        for step in data["steps"]:
            if step["registry_name"] == "tokenizer_processor":
                step["config"]["tokenizer_name"] = str(tokenizer)
        write_json(output / name, data)
    manifest.update(project_base_view=str(output), tokenizer=str(tokenizer),
                    project_overrides=config_overrides("cuda"))
    write_json(output / "project_base_manifest.json", manifest)
    return manifest


def train_pi05(dataset: str | Path, output: str | Path, assets: str | Path, *,
               steps: int = 2, device: str = "cuda", software_only: bool = False,
               resume_checkpoint: str | Path | None = None,
               checkpoint_interval: int = 1, lora: bool = False,
               lora_rank: int = 16, batch_size: int = 1) -> dict:
    dataset, output, assets = Path(dataset).resolve(), Path(output).resolve(), Path(assets).resolve()
    manifest = admit_dataset(dataset, software_only=software_only)
    if checkpoint_interval < 1:
        raise ValueError("checkpoint_interval must be positive")
    if lora_rank < 1 or batch_size < 1:
        raise ValueError("LoRA rank and batch size must be positive")
    if output.exists():
        raise FileExistsError("use a new output directory to preserve evidence")
    asset_manifest(assets)
    cmd = [sys.executable, "-m", "server.lerobot_training.official"]
    if resume_checkpoint:
        cp = Path(resume_checkpoint).resolve()
        validate_checkpoint(cp, require_identity=True,
                            expected_manifest_sha256=file_hash(dataset / MANIFEST))
        prior = json.loads((cp / "training_state/training_step.json").read_text())["step"]
        if steps <= prior:
            raise ValueError("total steps must exceed saved step")
        cmd += [f"--config_path={cp / 'pretrained_model/train_config.json'}", "--resume=true"]
    else:
        cmd += [f"--policy.path={assets}"]
        cmd += [f"--policy.{key}={str(value).lower() if isinstance(value, bool) else value}"
                for key, value in config_overrides(device).items()]
        cmd += ["--use_policy_training_preset=false", "--optimizer.type=adamw",
                "--optimizer.lr=0.0001", "--scheduler.type=constant_with_warmup",
                "--scheduler.num_warmup_steps=1"]
        if lora:
            cmd += ["--peft.method_type=LORA", f"--peft.r={lora_rank}",
                    f"--peft.lora_alpha={2 * lora_rank}", "--policy.use_peft=false"]
    cmd += [f"--dataset.repo_id={manifest.get('repo_id', 'local/piper_synthetic_smoke')}",
            f"--dataset.root={dataset}",
            f"--dataset.episodes={json.dumps(manifest['splits']['train'])}",
            "--dataset.use_imagenet_stats=false", f"--output_dir={output}", f"--steps={steps}",
            f"--policy.device={device}", "--policy.push_to_hub=false", "--wandb.enable=false",
            "--save_checkpoint_to_hub=false", "--num_workers=0", f"--batch_size={batch_size}", "--seed=19",
            f"--save_freq={checkpoint_interval}", "--log_freq=10" if steps > 100 else "--log_freq=1",
            "--env_eval_freq=0", "--eval_steps=0"]
    logs = output.parent / (output.name + "_launch")
    logs.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(PIPER_TRAINING_DATA_ROOT=str(dataset), PIPER_SOFTWARE_ONLY="1" if software_only else "0",
               PIPER_POLICY_TYPE="pi05", PIPER_PI05_LOAD_EVIDENCE=str(logs / "strict_loads.json"),
               PIPER_RESUME_EVIDENCE=str(logs / "resume_state.json"),
               PIPER_UPDATE_EVIDENCE=str(logs / "updates.json"),
               TOKENIZERS_PARALLELISM="false")
    if steps > 100:
        env["PIPER_UPDATE_EVIDENCE_STRIDE"] = "100"
    if lora:
        env["PIPER_REQUIRE_LORA"] = "1"
    write_json(logs / "command.json", {"argv": cmd, "data_manifest_sha256": file_hash(dataset / MANIFEST),
                                      "software_only": software_only, "resume_from": str(resume_checkpoint)})
    with (logs / "stdout.log").open("w", encoding="utf-8") as stream:
        result = subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, env=env, check=False)
    report = {"status": "PASS" if result.returncode == 0 else "FAILED", "exit_code": result.returncode,
              "output": str(output), "log": str(logs / "stdout.log"), "software_only": software_only,
              "checkpoint": str(output / "checkpoints" / f"{steps:06d}")}
    if result.returncode == 0:
        for checkpoint in sorted(p for p in (output / "checkpoints").iterdir()
                                 if p.is_dir() and p.name.isdigit()):
            if not (checkpoint / "piper_checkpoint_identity.json").is_file():
                seal_checkpoint(checkpoint, dataset)
            validate_checkpoint(checkpoint, require_identity=True,
                                expected_manifest_sha256=file_hash(dataset / MANIFEST),
                                include_model_hash=False)
        report["checkpoint_inventory"] = validate_checkpoint(
            report["checkpoint"], require_identity=True,
            expected_manifest_sha256=file_hash(dataset / MANIFEST))
        loads = json.loads((logs / "strict_loads.json").read_text())
        if not loads:
            raise ValueError("official PI05 run produced no strict pretrained load evidence")
        report["strict_loads"] = loads
    write_json(logs / "result.json", report)
    if result.returncode:
        raise RuntimeError(f"official PI05 trainer failed; see {logs / 'stdout.log'}")
    return report


def load_checkpoint_policy(model_dir, config):
    """Strictly load base and optional LoRA adapter."""
    import torch
    model_dir = Path(model_dir)
    adapter_path = model_dir / "adapter_config.json"
    if adapter_path.is_file():
        from peft import PeftModel, get_peft_model_state_dict
        from safetensors.torch import load_file
        adapter_config = json.loads(adapter_path.read_text())
        base = Path(adapter_config["base_model_name_or_path"])
        policy, loaded = load_pi05_strict(base, config)
        policy = PeftModel.from_pretrained(policy, model_dir, is_trainable=False,
                                           local_files_only=True)
        actual = get_peft_model_state_dict(policy)
        expected = load_file(str(model_dir / "adapter_model.safetensors"))
        if set(actual) != set(expected) or any(not torch.equal(
                actual[key].cpu(), expected[key].to(dtype=actual[key].dtype)) for key in expected):
            raise ValueError("LoRA checkpoint adapter tensor readback differs")
        loaded.update(adapter_loaded_strict=True, adapter_tensors=len(expected),
                      adapter_sha256=file_hash(model_dir / "adapter_model.safetensors"))
    else:
        policy, loaded = load_pi05_strict(model_dir, config)
    return policy, loaded


def predict_pi05(checkpoint: str | Path, dataset: str | Path, output: str | Path,
                 *, device: str = "cuda", software_only: bool = False) -> dict:
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    from lerobot.utils.collate import lerobot_collate_fn

    cp, dataset = Path(checkpoint), Path(dataset)
    manifest = admit_dataset(dataset, software_only=software_only)
    validate_checkpoint(cp, require_identity=True,
                        expected_manifest_sha256=file_hash(dataset / MANIFEST))
    model_dir = cp / "pretrained_model"
    config = PI05Config.from_pretrained(model_dir, local_files_only=True)
    config.device = device
    policy, loaded = load_checkpoint_policy(model_dir, config)
    policy.eval()
    pre, post = make_pre_post_processors(config, pretrained_path=str(model_dir),
        preprocessor_overrides={"device_processor": {"device": device}})
    repo_id = manifest.get("repo_id", "local/piper_synthetic_smoke")
    ds = LeRobotDataset(repo_id, root=dataset, episodes=manifest["splits"]["validation"])
    batch = lerobot_collate_fn([ds[0]])
    with torch.no_grad():
        torch.manual_seed(19)
        policy.reset()
        actions = post(policy.predict_action_chunk(pre(batch)))
        torch.manual_seed(19)
        policy.reset()
        repeat = post(policy.predict_action_chunk(pre(batch)))
    if list(actions.shape) != [1, 16, 7] or not torch.isfinite(actions).all():
        raise ValueError("PI05 prediction shape/finite contract failed")
    if not torch.equal(actions, repeat):
        raise ValueError("seeded PI05 prediction/reset mismatch")
    report = {"status": "PASS_SOFTWARE_ONLY" if software_only else "PASS_OFFLINE", "strict_load": loaded,
              "shape": list(actions.shape), "finite": True, "seeded_reset_equal": True,
              "actions": actions.tolist(), "source_episode": int(ds[0]["episode_index"]),
              "claim": "real-base synthetic-data training facility; no robot success assessment"}
    write_json(output, report)
    return report
