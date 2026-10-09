"""Loopback-only PiPER proxy policy server for prediction-only shadow tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from openpi.policies import policy_config
from openpi.serving.websocket_policy_server import WebsocketPolicyServer

from server.policy_server.artifact_identity import (
    files_sha256,
    resolve_managed_tree,
    sha256_file,
    sha256_text_file,
    tree_sha256,
)
from server.training.proxy_piper_tiny_fit import (
    DATA_ROOT,
    PROXY_DATASETS,
    inspect_dataset,
    make_config,
)
from server.policy_server.proxy_config import ProxyPolicyConfig


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "policy" / "pi05_proxy_shadow.json"
POLICY_PIPELINE_FILES = (
    "server/serve_proxy_piper_shadow.py",
    "server/policy_server/artifact_identity.py",
    "server/policy_server/proxy_config.py",
    "server/training/piper_openpi_config.py",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--port", type=int, help="bounded override of configured port")
    parser.add_argument("--num-steps", type=int,
                        help="bounded override of configured sampling steps")
    args = parser.parse_args()
    runtime = ProxyPolicyConfig.load(args.config)
    port = runtime.server_port if args.port is None else args.port
    num_steps = runtime.sampling_steps if args.num_steps is None else args.num_steps
    if not 1 <= port <= 65535 or not 1 <= num_steps <= 10:
        raise ValueError("invalid port or bounded sampling step count")
    variant = runtime.dataset_variant
    spec = PROXY_DATASETS[variant]
    if runtime.dataset_identity != spec.repo_id:
        raise ValueError("configured dataset does not match the isolated proxy variant")
    source = (
        inspect_dataset(
            variant,
            expected_dataset_tree_sha256=runtime.expected_dataset_tree_sha256,
            expected_readback_audit_sha256=runtime.expected_readback_audit_sha256,
        )
        if variant == "10fps" else inspect_dataset(variant)
    )
    config = make_config(exp_name=runtime.experiment_name, variant=variant)
    checkpoint = config.checkpoint_dir / str(runtime.checkpoint_step)
    if not checkpoint.is_dir():
        raise FileNotFoundError(checkpoint)
    normalization_stats = config.assets_dirs / runtime.dataset_identity / "norm_stats.json"
    dataset_root = resolve_managed_tree(DATA_ROOT, runtime.dataset_identity)
    if not normalization_stats.is_file():
        raise FileNotFoundError(normalization_stats)
    if (runtime.expected_normalization_sha256 is not None
            and sha256_file(normalization_stats) != runtime.expected_normalization_sha256):
        raise ValueError("normalization stats differ from the derived-state identity lock")
    openpi_commit = json.loads(
        (ROOT / "provenance" / "openpi_git.json").read_text(encoding="utf-8")
    )["head_sha"]
    if (runtime.expected_openpi_commit is not None
            and openpi_commit != runtime.expected_openpi_commit):
        raise ValueError("OpenPI submodule commit differs from the derived-state identity lock")
    identities = {
        "artifact_identity_schema": "piper_policy_artifact_identity_v1",
        "policy_config_sha256": sha256_text_file(args.config),
        "policy_pipeline_code_sha256": files_sha256(ROOT, POLICY_PIPELINE_FILES),
        "training_config_code_sha256": sha256_text_file(
            ROOT / "server" / "training" / "proxy_piper_tiny_fit.py"),
        "normalization_sha256": sha256_file(normalization_stats),
        "dataset_tree_sha256": tree_sha256(dataset_root),
        "openpi_commit": openpi_commit,
    }
    # Read the large checkpoint only after all small inputs and code pins pass.
    identities["checkpoint_sha256"] = tree_sha256(checkpoint)
    if (runtime.expected_checkpoint_sha256 is not None
            and identities["checkpoint_sha256"] != runtime.expected_checkpoint_sha256):
        raise ValueError("checkpoint differs from the derived-state identity lock")
    policy = policy_config.create_trained_policy(
        config, checkpoint, sample_kwargs={"num_steps": num_steps})
    server = WebsocketPolicyServer(
        policy, host=runtime.server_host, port=port,
        metadata={
            "project": "07_pi0.5", "config": config.name,
            "config_schema": runtime.schema,
            "checkpoint": checkpoint.relative_to(ROOT).as_posix(),
            "dataset": runtime.dataset_identity,
            "dataset_action_semantics": source["action_semantics"],
            "action_target_kind": source.get(
                "action_target_kind", "derived_future_observed_state_position_target"),
            "raw_command_supervision": source.get("raw_command_supervision", False),
            "normalization_identity": runtime.normalization_identity,
            **identities,
            "action_semantics": runtime.action_semantics,
            "action_horizon": runtime.action_horizon,
            "first_target_dt_s": runtime.first_target_dt_s,
            "sampling_steps": num_steps,
            "units_verified": False,
            "hardware_control_allowed": runtime.hardware_control_allowed,
            "prediction_only": True,
            "robot_control_ready": False, "control_enabled": False,
        },
    )
    print(f"PREDICTION_ONLY_LISTEN={runtime.server_host}:{port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
