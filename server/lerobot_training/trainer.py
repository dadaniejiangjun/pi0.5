"""Project admission and launch adapter; all optimization uses upstream trainer."""
from __future__ import annotations
import json
import os
import subprocess
import sys
from pathlib import Path
from .data import admit_dataset, MANIFEST
from .runtime import write_json, file_hash

CHECKPOINT_IDENTITY = "piper_checkpoint_identity.json"
CHECKPOINT_IDENTITY_SCHEMA = "piper_checkpoint_processor_identity_v1"


def _processor_identity_files(checkpoint: Path) -> dict[str, str]:
    """Hash the policy config and every persisted input/output processor file."""
    model = checkpoint / "pretrained_model"
    names = {"config.json", "train_config.json", "policy_preprocessor.json",
             "policy_postprocessor.json"}
    names.update(p.name for prefix in ("policy_preprocessor", "policy_postprocessor")
                 for p in model.glob(prefix + "*.safetensors"))
    hashes = {}
    for name in sorted(names):
        path = model / name
        if not path.is_file():
            raise ValueError(f"checkpoint processor identity file missing: {name}")
        hashes[f"pretrained_model/{name}"] = file_hash(path)
    if not any("policy_preprocessor" in name and name.endswith(".safetensors") for name in hashes):
        raise ValueError("checkpoint input processor statistics are missing")
    if not any("policy_postprocessor" in name and name.endswith(".safetensors") for name in hashes):
        raise ValueError("checkpoint output processor statistics are missing")
    return hashes


def _seal_checkpoint(checkpoint: Path, dataset_root: Path) -> dict:
    """Seal a checkpoint emitted by the current successful producer call."""
    checkpoint = checkpoint.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    identity_path = checkpoint / CHECKPOINT_IDENTITY
    if identity_path.exists():
        raise FileExistsError(f"checkpoint identity already exists: {identity_path}")
    identity = {
        "schema": CHECKPOINT_IDENTITY_SCHEMA,
        "processor_files": _processor_identity_files(checkpoint),
        "data_manifest_sha256": file_hash(dataset_root / MANIFEST),
        "processor_scope": ["config.json", "train_config.json", "policy_preprocessor.*",
                            "policy_postprocessor.*"],
    }
    write_json(identity_path, identity)
    return identity


def seal_checkpoint(checkpoint: str | Path, dataset_root: str | Path) -> dict:
    """Seal a checkpoint immediately after a successful current producer run."""
    return _seal_checkpoint(Path(checkpoint), Path(dataset_root))


def seal_legacy_checkpoint(checkpoint: str | Path, dataset_root: str | Path,
                           producer_command_json: str | Path,
                           producer_result_json: str | Path, *,
                           reconstructed_processor_files: dict[str, str] | None = None) -> dict:
    """Adopt one legacy output only when its producer evidence matches exactly.

    This is deliberately not an unverified migration switch. The command and
    successful result must identify the same output/checkpoint and dataset.
    Files absent from the producer inventory require hashes from an independent
    processor reconstruction using that dataset and the saved policy config.
    """
    cp = Path(checkpoint).resolve(strict=True)
    data = Path(dataset_root).resolve(strict=True)
    command_path, result_path = Path(producer_command_json), Path(producer_result_json)
    command = json.loads(command_path.read_text(encoding="utf-8"))
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result.get("status") != "PASS" or result.get("exit_code") != 0:
        raise ValueError("legacy checkpoint producer result is not successful")
    if Path(result.get("checkpoint", "")).resolve() != cp:
        raise ValueError("legacy evidence names a different checkpoint")
    output = cp.parent.parent.resolve()
    if Path(result.get("output", "")).resolve() != output:
        raise ValueError("legacy evidence names a different output directory")
    argv = command.get("argv", [])
    if f"--output_dir={output}" not in argv:
        raise ValueError("legacy producer command output does not match")
    if f"--dataset.root={data}" not in argv:
        raise ValueError("legacy producer command dataset does not match")
    expected_manifest = file_hash(data / MANIFEST)
    if command.get("data_manifest_sha256") != expected_manifest:
        raise ValueError("legacy producer command data manifest does not match")
    inventory = result.get("checkpoint_inventory")
    if not isinstance(inventory, dict):
        raise ValueError("legacy producer inventory is missing")
    current_files = _processor_identity_files(cp)
    reconstructed = reconstructed_processor_files or {}
    missing = []
    for name, digest in current_files.items():
        producer_digest = inventory.get(name)
        rebuilt_digest = reconstructed.get(name)
        if producer_digest is not None and producer_digest != digest:
            missing.append(name)
        elif producer_digest is None and rebuilt_digest != digest:
            missing.append(name)
        elif rebuilt_digest is not None and rebuilt_digest != digest:
            missing.append(name)
    if missing:
        raise ValueError(f"legacy processor/config files lack matching producer or reconstruction evidence: {missing}")
    return _seal_checkpoint(cp, data)

def train_act(dataset: str | Path, output: str | Path, *, steps: int = 4,
              device: str = "cpu", software_only: bool = False,
              resume_checkpoint: str | Path | None = None) -> dict:
    dataset, output = Path(dataset).resolve(), Path(output).resolve()
    manifest = admit_dataset(dataset, software_only=software_only)
    data_manifest_sha256 = file_hash(dataset / MANIFEST)
    if output.exists():
        raise FileExistsError("use a new output directory to preserve evidence")
    if steps < 1:
        raise ValueError("steps must be positive")
    cmd = [sys.executable, "-m", "server.lerobot_training.official"]
    if resume_checkpoint:
        cp = Path(resume_checkpoint).resolve()
        validate_checkpoint(cp, require_identity=True,
                            expected_manifest_sha256=data_manifest_sha256,
                            include_model_hash=False)
        prior = json.loads((cp / "training_state/training_step.json").read_text())["step"]
        if steps <= prior:
            raise ValueError("total steps must exceed saved step")
        cmd += [f"--config_path={cp / 'pretrained_model/train_config.json'}", "--resume=true"]
    else:
        cmd += ["--policy.type=act", "--policy.pretrained_backbone_weights=null",
                "--policy.chunk_size=16", "--policy.n_action_steps=16",
                "--policy.dim_model=32", "--policy.dim_feedforward=64", "--policy.n_heads=4",
                "--policy.n_encoder_layers=1", "--policy.n_decoder_layers=1",
                "--policy.use_vae=false", "--policy.dropout=0.0",
                "--use_policy_training_preset=false", "--optimizer.type=adamw", "--optimizer.lr=0.0001",
                "--scheduler.type=constant_with_warmup", "--scheduler.num_warmup_steps=1"]
    cmd += [f"--dataset.repo_id={manifest.get('repo_id', 'local/piper_synthetic_smoke')}",
            f"--dataset.root={dataset}", f"--dataset.episodes={json.dumps(manifest['splits']['train'])}",
            "--dataset.use_imagenet_stats=false", f"--output_dir={output}", f"--steps={steps}",
            f"--policy.device={device}", "--policy.push_to_hub=false", "--wandb.enable=false",
            "--save_checkpoint_to_hub=false", "--num_workers=0", "--batch_size=2", "--seed=19",
            "--save_freq=2", "--log_freq=1", "--env_eval_freq=0", "--eval_steps=0"]
    logs = output.parent / (output.name + "_launch")
    logs.mkdir(parents=True, exist_ok=False)
    write_json(logs / "command.json", {"argv": cmd, "data_manifest_sha256": file_hash(dataset / MANIFEST),
                                      "software_only": software_only, "resume_from": str(resume_checkpoint)})
    env = os.environ.copy()
    env["PIPER_TRAINING_DATA_ROOT"] = str(dataset)
    env["PIPER_SOFTWARE_ONLY"] = "1" if software_only else "0"
    env["PIPER_RESUME_EVIDENCE"] = str(logs / "resume_state.json")
    env["PIPER_UPDATE_EVIDENCE"] = str(logs / "parameter_updates.json")
    with (logs / "stdout.log").open("w", encoding="utf-8") as f:
        result = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, check=False)
    report = {"status": "PASS" if result.returncode == 0 else "FAILED", "exit_code": result.returncode,
              "output": str(output), "log": str(logs / "stdout.log"), "software_only": software_only,
              "checkpoint": str(output / "checkpoints" / f"{steps:06d}")}
    if result.returncode == 0:
        checkpoint_root = output / "checkpoints"
        checkpoints = sorted(p for p in checkpoint_root.iterdir()
                             if p.is_dir() and p.name.isdigit())
        if not checkpoints:
            raise ValueError("official trainer produced no numbered checkpoints")
        for checkpoint in checkpoints:
            if not (checkpoint / CHECKPOINT_IDENTITY).is_file():
                seal_checkpoint(checkpoint, dataset)
            validate_checkpoint(checkpoint, require_identity=True,
                                expected_manifest_sha256=data_manifest_sha256,
                                include_model_hash=False)
        report["checkpoint_inventory"] = validate_checkpoint(
            report["checkpoint"], require_identity=True,
            expected_manifest_sha256=data_manifest_sha256)
    write_json(logs / "result.json", report)
    if result.returncode:
        raise RuntimeError(f"official trainer failed; see {logs / 'stdout.log'}")
    return report

def validate_checkpoint(checkpoint: str | Path, *, require_identity: bool = False,
                        expected_manifest_sha256: str | None = None,
                        include_model_hash: bool = True) -> dict:
    cp = Path(checkpoint)
    required = ["pretrained_model/config.json", "pretrained_model/model.safetensors",
                "pretrained_model/train_config.json", "pretrained_model/policy_preprocessor.json",
                "pretrained_model/policy_postprocessor.json", "training_state/training_step.json",
                "training_state/optimizer_state.safetensors", "training_state/optimizer_param_groups.json",
                "training_state/scheduler_state.json", "training_state/rng_state.safetensors"]
    adapter_config = cp / "pretrained_model/adapter_config.json"
    if adapter_config.is_file():
        adapter = json.loads(adapter_config.read_text())
        if adapter.get("peft_type") != "LORA" or not adapter.get("base_model_name_or_path"):
            raise ValueError("LoRA checkpoint lacks valid base model binding")
        required.remove("pretrained_model/model.safetensors")
        required.extend(["pretrained_model/adapter_config.json", "pretrained_model/adapter_model.safetensors"])
    for name in required:
        if not (cp / name).is_file():
            raise ValueError(f"checkpoint missing {name}")
    identity_path = cp / CHECKPOINT_IDENTITY
    identity = None
    if require_identity:
        if not identity_path.is_file():
            raise ValueError("checkpoint is legacy/unsealed; exact producer evidence is required to adopt it")
        identity = json.loads(identity_path.read_text(encoding="utf-8"))
        if identity.get("schema") != CHECKPOINT_IDENTITY_SCHEMA:
            raise ValueError("checkpoint processor identity schema mismatch")
        actual_files = _processor_identity_files(cp)
        if identity.get("processor_files") != actual_files:
            raise ValueError("checkpoint processor/config identity mismatch")
        if expected_manifest_sha256 is None:
            raise ValueError("expected dataset manifest hash is required for checkpoint validation")
        if identity.get("data_manifest_sha256") != expected_manifest_sha256:
            raise ValueError("checkpoint processor statistics belong to a different dataset manifest")
    inventory = {name: file_hash(cp / name) for name in required
                 if include_model_hash or name != "pretrained_model/model.safetensors"}
    if identity is not None:
        inventory[CHECKPOINT_IDENTITY] = file_hash(identity_path)
    return inventory

def predict(checkpoint: str | Path, dataset: str | Path, output: str | Path,
            *, software_only: bool = False) -> dict:
    import torch
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.utils.collate import lerobot_collate_fn
    cp, dataset = Path(checkpoint), Path(dataset)
    m = admit_dataset(dataset, software_only=software_only)
    validate_checkpoint(cp, require_identity=True,
                        expected_manifest_sha256=file_hash(dataset / MANIFEST),
                        include_model_hash=False)
    policy = ACTPolicy.from_pretrained(cp / "pretrained_model")
    policy.eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(cp / "pretrained_model"))
    ds = LeRobotDataset(m.get("repo_id", "local/piper_synthetic_smoke"), root=dataset,
                        episodes=m["splits"]["validation"])
    batch = lerobot_collate_fn([ds[0]])
    with torch.no_grad():
        policy.reset()
        raw = policy.predict_action_chunk(pre(batch))
        actions = post(raw)
        policy.reset()
        repeat = post(policy.predict_action_chunk(pre(batch)))
    if actions.shape != (1, 16, 7) or not torch.isfinite(actions).all():
        raise ValueError("action shape or finite check failed")
    if not torch.equal(actions, repeat):
        raise ValueError("episode reset prediction changed")
    result = {"status": "PASS_SOFTWARE_ONLY" if software_only else "PASS_OFFLINE",
              "shape": list(actions.shape), "finite": True, "reset_equal": True,
              "actions": actions.tolist(), "source_episode": int(ds[0]["episode_index"]),
              "checkpoint_sha256": file_hash(cp / "pretrained_model/model.safetensors"),
              "claim": "offline prediction; no robot success assessment"}
    write_json(output, result)
    return result
