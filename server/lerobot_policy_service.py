"""Prediction-only LeRobot PI05 adapter for the existing OpenPI MsgPack socket.

The adapter accepts the project's semantic PiPER observation and returns one
absolute-action preview chunk. It never creates a robot or transport client.
"""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from laptop.client.observation_schema import validate_piper_semantic_observation
from server.lerobot_training.data import MANIFEST
from server.lerobot_training.pi05 import ACTION_DIM, CAMERAS, HORIZON, load_pi05_strict
from server.lerobot_training.trainer import validate_checkpoint
from server.lerobot_training.runtime import file_hash

SCHEMA = "piper_lerobot_pi05_prediction_only_v1"
LOCK_SCHEMA = "piper_lerobot_pi05_prediction_only_lock_v1"
BASE_MODEL = "lerobot/pi05_base"
BASE_REVISION = "22c30e5b34f9e7ab738b1b34fc2fc5ecfe2d28f0"


def _hash_json(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def inspect_service_identity(checkpoint: str | Path, dataset: str | Path, *,
                             known_model_sha256: str | None = None) -> dict[str, Any]:
    """Check sealed checkpoint, its dataset binding, and processor identities."""
    cp = Path(checkpoint).resolve(strict=True)
    data = Path(dataset).resolve(strict=True)
    manifest_path = data / MANIFEST
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest_sha = file_hash(manifest_path)
    validate_checkpoint(cp, require_identity=True,
                        expected_manifest_sha256=manifest_sha,
                        include_model_hash=False)
    identity_path = cp / "piper_checkpoint_identity.json"
    checkpoint_identity = json.loads(identity_path.read_text(encoding="utf-8"))
    processor_files = checkpoint_identity["processor_files"]
    model_dir = cp / "pretrained_model"
    for relative, expected in processor_files.items():
        actual = file_hash(cp / relative)
        if actual != expected:
            raise ValueError(f"checkpoint processor identity mismatch: {relative}")
    normalization_files = {
        name: digest for name, digest in processor_files.items()
        if "normalizer_processor.safetensors" in name
    }
    if len(normalization_files) < 2:
        raise ValueError("checkpoint lacks sealed train-only input/output normalization state")
    weights_name = "adapter_model.safetensors" if (model_dir / "adapter_config.json").is_file() else "model.safetensors"
    model_hash = (file_hash(model_dir / weights_name) if known_model_sha256 is None
                  else known_model_sha256)
    if (not isinstance(model_hash, str) or len(model_hash) != 64
            or any(ch not in "0123456789abcdef" for ch in model_hash)):
        raise ValueError("known model SHA-256 must be 64 lowercase hexadecimal characters")
    return {
        "schema": SCHEMA,
        "backend": "lerobot_pi05",
        "config_schema": SCHEMA,
        "model_id": f"{BASE_MODEL}@{BASE_REVISION}",
        "base_model": BASE_MODEL,
        "base_revision": BASE_REVISION,
        "checkpoint": str(cp),
        "checkpoint_sha256": model_hash,
        "checkpoint_identity": str(identity_path),
        "checkpoint_identity_sha256": file_hash(identity_path),
        "dataset": manifest.get("repo_id", "local/piper_synthetic_smoke"),
        "dataset_manifest_sha256": manifest_sha,
        "source_kind": manifest.get("source_kind"),
        "training_eligible": manifest.get("training_eligible") is True,
        "software_only": manifest.get("source_kind") == "synthetic",
        "training_step": int(json.loads((cp / "training_state/training_step.json").read_text())["step"]),
        "processor_files_sha256": processor_files,
        "processor_identity_sha256": _hash_json(processor_files),
        "normalization_sha256": _hash_json(normalization_files),
        "action_semantics": "lerobot_pi05_absolute_action_chunk_preview_only",
        "action_horizon": HORIZON,
        "action_dimension": ACTION_DIM,
        "first_target_dt_s": 0.1,
        "prediction_only": True,
        "control_enabled": False,
        "robot_control_ready": False,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }


def write_identity_lock(checkpoint: str | Path, dataset: str | Path,
                        output: str | Path) -> dict[str, Any]:
    """Export the independently checked service identity for the client host."""
    target = Path(output)
    if target.exists():
        raise FileExistsError(target)
    metadata = inspect_service_identity(checkpoint, dataset)
    fields = (
        "config_schema", "backend", "action_semantics", "checkpoint",
        "checkpoint_sha256", "checkpoint_identity_sha256", "dataset",
        "dataset_manifest_sha256", "processor_identity_sha256",
        "normalization_sha256", "training_step", "base_model", "base_revision",
        "action_horizon", "action_dimension", "first_target_dt_s", "source_kind",
        "training_eligible", "software_only", "prediction_only", "control_enabled",
        "robot_control_ready", "hardware_control_allowed", "can_opened",
        "can_data_tx_calls",
    )
    lock = {"schema": LOCK_SCHEMA, **{field: metadata[field] for field in fields}}
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(lock, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return lock


class LeRobotPolicyService:
    """Strictly load a sealed PI05 checkpoint and produce preview chunks."""

    def __init__(self, checkpoint: str | Path, dataset: str | Path, *, device: str = "cuda",
                 policy: Any | None = None, preprocessor: Any | None = None,
                 postprocessor: Any | None = None, load_runtime: bool = True,
                 known_model_sha256: str | None = None):
        self.checkpoint = Path(checkpoint).resolve(strict=True)
        self.dataset = Path(dataset).resolve(strict=True)
        self.metadata = inspect_service_identity(
            self.checkpoint, self.dataset, known_model_sha256=known_model_sha256)
        self.device = device
        self.policy, self.preprocessor, self.postprocessor = policy, preprocessor, postprocessor
        if load_runtime:
            self._load_runtime()
        if self.policy is None or self.preprocessor is None or self.postprocessor is None:
            raise ValueError("policy and both checkpoint processors are required")
        self._session_id: str | None = None

    def _load_runtime(self) -> None:
        import torch
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.pi05.configuration_pi05 import PI05Config

        model_dir = self.checkpoint / "pretrained_model"
        config = PI05Config.from_pretrained(model_dir, local_files_only=True)
        config.device = self.device
        from server.lerobot_training.pi05_run import load_checkpoint_policy
        self.policy, evidence = load_checkpoint_policy(model_dir, config)
        if evidence.get("strict") is not True or evidence.get("matched_tensors") != 813:
            raise RuntimeError("PI05 checkpoint did not pass the expected strict 813-tensor load")
        self.policy.eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            config, pretrained_path=str(model_dir),
            preprocessor_overrides={"device_processor": {"device": self.device}},
        )
        self._torch = torch

    def reset_episode(self, session_id: str | None = None) -> None:
        """Reset policy episode state; a changed session cannot inherit a queue."""
        self.policy.reset()
        self._session_id = session_id

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        validate_piper_semantic_observation(observation)
        started = time.perf_counter_ns()
        session_id = observation.get("session_id")
        if session_id is not None and (not isinstance(session_id, str) or not session_id):
            raise ValueError("session_id must be a non-empty string when provided")
        if session_id != self._session_id:
            self.reset_episode(session_id)
        # Require the model's trained camera size. No resize or role substitution
        # occurs here; callers must provide the existing Base/Wrist RGB contract.
        for key in ("base_rgb", "wrist_rgb"):
            image = np.asarray(observation[key])
            if image.shape != (224, 224, 3) or image.dtype != np.uint8:
                raise ValueError(f"{key} must be 224x224 uint8 RGB for this checkpoint")
        if not np.isfinite(np.asarray(observation["state"])).all():
            raise ValueError("state must contain only finite values")
        if not observation["prompt"].strip():
            raise ValueError("prompt must be non-empty")
        request_id = observation.get("request_id")
        if request_id is not None and (not isinstance(request_id, str) or not request_id):
            raise ValueError("request_id must be a non-empty string when provided")

        torch = getattr(self, "_torch", None)
        if torch is None:
            import torch
        batch = {
            CAMERAS[0]: torch.from_numpy(np.array(observation["base_rgb"], copy=True, order="C"))
                .permute(2, 0, 1).unsqueeze(0),
            CAMERAS[1]: torch.from_numpy(np.array(observation["wrist_rgb"], copy=True, order="C"))
                .permute(2, 0, 1).unsqueeze(0),
            "observation.state": torch.from_numpy(
                np.array(observation["state"], dtype=np.float32, copy=True, order="C")
            )
                .unsqueeze(0),
            "task": [observation["prompt"]],
        }
        # predict_action_chunk has no cross-request action queue. Reset also clears
        # any policy-local episode cache and gives each request an independent seed.
        self.policy.reset()
        with torch.no_grad():
            action = self.postprocessor(self.policy.predict_action_chunk(self.preprocessor(batch)))
        if hasattr(action, "detach"):
            action = action.detach().to(dtype=torch.float32, device="cpu").numpy()
        actions = np.asarray(action, dtype=np.float32)
        if actions.shape == (1, HORIZON, ACTION_DIM):
            actions = actions[0]
        if actions.shape != (HORIZON, ACTION_DIM) or not np.isfinite(actions).all():
            raise ValueError("LeRobot PI05 result must be finite float32[16,7]")
        elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000
        result: dict[str, Any] = {
            "actions": actions,
            "backend": "lerobot_pi05",
            "model_id": self.metadata["model_id"],
            "checkpoint_sha256": self.metadata["checkpoint_sha256"],
            "processor_identity_sha256": self.metadata["processor_identity_sha256"],
            "server_timing": {"infer_ms": elapsed_ms},
            "prediction_only": True,
        }
        for key in ("session_id", "request_id", "observation_ns"):
            if key in observation:
                result[key] = observation[key]
        return result

    def close(self) -> None:
        if self.policy is not None:
            self.policy.reset()
        self._session_id = None


def create_websocket_server(checkpoint: str | Path, dataset: str | Path, *,
                            host: str = "127.0.0.1", port: int = 8024,
                            device: str = "cuda", known_model_sha256: str | None = None):
    """Create the upstream OpenPI-compatible MsgPack server; loopback only."""
    if host not in {"127.0.0.1", "localhost"}:
        raise ValueError("LeRobot policy service must bind to loopback")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port must be an integer in [1, 65535]")
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer
    service = LeRobotPolicyService(checkpoint, dataset, device=device,
                                   known_model_sha256=known_model_sha256)
    return WebsocketPolicyServer(service, host=host, port=port, metadata=service.metadata)


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8024)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--known-model-sha256",
                        help="reuse a previously verified checkpoint hash instead of rehashing model bytes")
    parser.add_argument("--write-identity-lock", type=Path,
                        help="export a checked lock and exit without loading weights or serving")
    args = parser.parse_args()
    if args.write_identity_lock:
        lock = write_identity_lock(args.checkpoint, args.dataset, args.write_identity_lock)
        print(json.dumps({"status": "PASS_IDENTITY_LOCK", "path": str(args.write_identity_lock),
                          "schema": lock["schema"]}, sort_keys=True))
        return 0
    server = create_websocket_server(args.checkpoint, args.dataset, host=args.host,
                                     port=args.port, device=args.device,
                                     known_model_sha256=args.known_model_sha256)
    print(f"LEROBOT_PREDICTION_ONLY_LISTEN={args.host}:{args.port}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
