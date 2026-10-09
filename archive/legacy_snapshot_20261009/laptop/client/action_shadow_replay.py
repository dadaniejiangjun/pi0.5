"""Replay verified, file-backed PiPER action episodes against a prediction-only server.

This tool has no camera, CAN, robot SDK, or SafetyBridge imports. It compares
server predictions with source-verified command targets and writes a local
report; it never turns a prediction into a robot command.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
import time

import cv2
import numpy as np

from laptop.recorder.piper_command_targets import ACTION_TARGET_SOURCE
from laptop.recorder.verify_source import verify_source


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_meter"
ACTION_SHADOW_SCHEMA = "pi05_piper_action_shadow_v1"
HASH_FIELDS = (
    "checkpoint_sha256", "policy_config_sha256", "policy_pipeline_code_sha256",
    "training_config_code_sha256", "normalization_sha256", "dataset_tree_sha256",
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


def validate_action_shadow_metadata(metadata: object) -> dict:
    """Require an identity-pinned PiPER action prediction service, never control."""
    if not isinstance(metadata, dict):
        raise ValueError("action shadow server metadata must be an object")
    expected = {
        "artifact_identity_schema": "piper_policy_artifact_identity_v1",
        "config_schema": ACTION_SHADOW_SCHEMA,
        "action_semantics": ACTION_SEMANTICS,
        "action_horizon": 16,
        "action_dimension": 7,
        "prediction_only": True,
        "hardware_control_allowed": False,
        "robot_control_ready": False,
        "control_enabled": False,
        "bind_address": "127.0.0.1",
        "checkpoint_identity_scope": "selected_numbered_openpi_checkpoint_tree",
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise PermissionError(f"action shadow metadata {key} violates prediction-only contract")
    if not isinstance(metadata.get("checkpoint"), str) or not metadata["checkpoint"]:
        raise ValueError("action shadow metadata lacks checkpoint identity")
    if not isinstance(metadata.get("dataset"), str) or not metadata["dataset"]:
        raise ValueError("action shadow metadata lacks dataset identity")
    if not isinstance(metadata.get("normalization_identity"), str) or not metadata["normalization_identity"]:
        raise ValueError("action shadow metadata lacks normalization identity")
    for key in HASH_FIELDS:
        if not isinstance(metadata.get(key), str) or not SHA256_RE.fullmatch(metadata[key]):
            raise ValueError(f"action shadow metadata {key} is not a lowercase SHA-256")
    if not isinstance(metadata.get("openpi_commit"), str) or not COMMIT_RE.fullmatch(metadata["openpi_commit"]):
        raise ValueError("action shadow metadata has an invalid OpenPI commit")
    return dict(metadata)


def _inside(root: Path, relative: object) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("aligned image path must be relative")
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError("aligned image path is missing or escapes the episode")
    return path


def _read_rgb(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"cannot read RGB source image: {path.name}")
    return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))


def _load_verified_episode(raw_path: Path, aligned_path: Path) -> tuple[dict, list[dict], dict]:
    raw_path, aligned_path = Path(raw_path).resolve(), Path(aligned_path).resolve()
    provenance = verify_source(raw_path, aligned_path)
    if (provenance.get("source_frame_extraction_verified") is not True
            or provenance.get("command_target_provenance_verified") is not True):
        raise PermissionError("episode lacks independently verified CAN-command target provenance")
    metadata = json.loads((aligned_path / "aligned_episode.json").read_text(encoding="utf-8"))
    if (metadata.get("schema") != "piper_aligned_episode_v1"
            or metadata.get("source") != "manual_teach"
            or metadata.get("action_target_source") != ACTION_TARGET_SOURCE
            or metadata.get("action_semantics") not in ("absolute_joint_position", ACTION_SEMANTICS)
            or metadata.get("action_target_provenance_verified") is not True
            or metadata.get("success") is not True):
        raise PermissionError("only a reviewed successful command-target episode can be replayed")
    rows = [json.loads(line) for line in (aligned_path / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
    expected_count = metadata.get("frame_count")
    if type(expected_count) is not int or len(rows) != expected_count or len(rows) < 1:
        raise ValueError("aligned frame count does not match its metadata")
    return metadata, rows, provenance


def _bounded_ws_client(host: str, port: int, timeout_s: float):
    """Create a one-connection OpenPI-protocol client with bounded recv waits."""
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("action shadow is loopback-only; use an SSH local tunnel for remote servers")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port must be in [1, 65535]")
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or not 0.1 <= timeout_s <= 60:
        raise ValueError("timeout must be in [0.1, 60] seconds")
    client_source = PROJECT_ROOT / "third_party/openpi/packages/openpi-client/src"
    if client_source.is_dir() and str(client_source) not in sys.path:
        sys.path.insert(0, str(client_source))
    import websockets.sync.client
    from openpi_client import msgpack_numpy

    connection = websockets.sync.client.connect(
        f"ws://{host}:{port}", compression=None, open_timeout=timeout_s,
        close_timeout=timeout_s, max_size=8 * 1024 * 1024, proxy=None,
    )
    try:
        metadata = msgpack_numpy.unpackb(connection.recv(timeout=timeout_s))
    except Exception:
        connection.close(timeout=timeout_s)
        raise

    class Client:
        def get_server_metadata(self) -> dict:
            return metadata

        def infer(self, observation: dict) -> dict:
            connection.send(msgpack_numpy.Packer().pack(observation))
            response = connection.recv(timeout=timeout_s)
            if isinstance(response, str):
                raise RuntimeError(f"action shadow server error: {response}")
            return msgpack_numpy.unpackb(response)

        def close(self) -> None:
            connection.close(timeout=timeout_s)

    return Client()


def replay(raw_path: Path, aligned_path: Path, client: object, *, max_samples: int = 20) -> dict:
    """Run bounded inference over verified frames and report target error only."""
    if type(max_samples) is not int or not 1 <= max_samples <= 100:
        raise ValueError("max_samples must be in [1, 100]")
    metadata, rows, provenance = _load_verified_episode(raw_path, aligned_path)
    server_metadata = validate_action_shadow_metadata(client.get_server_metadata())
    indices = sorted(set(np.linspace(0, len(rows) - 1, min(max_samples, len(rows)), dtype=int).tolist()))
    samples = []
    for index in indices:
        row = rows[index]
        if row.get("frame_index") != index:
            raise ValueError("aligned frame indices must be contiguous")
        state = np.asarray(row.get("state"), dtype=np.float32)
        target = np.asarray(row.get("action"), dtype=np.float64)
        if (state.shape != (7,) or target.shape != (7,)
                or not np.isfinite(state).all() or not np.isfinite(target).all()):
            raise ValueError("verified state and action target must be finite 7D vectors")
        observation = {
            "base_rgb": _read_rgb(_inside(Path(aligned_path), row.get("base_image"))),
            "wrist_rgb": _read_rgb(_inside(Path(aligned_path), row.get("wrist_image"))),
            "state": state,
            "prompt": metadata["task"],
        }
        started_ns = time.perf_counter_ns()
        response = client.infer(observation)
        finished_ns = time.perf_counter_ns()
        prediction = np.asarray(response.get("actions"), dtype=np.float64)
        if prediction.shape != (16, 7) or not np.isfinite(prediction).all():
            raise ValueError("PiPER action prediction must be finite with shape (16, 7)")
        error = prediction[0] - target
        samples.append({
            "frame_index": index,
            "policy_timestamp_ns": row["policy_timestamp_ns"],
            "action_target_timestamp_ns": row["action_target_timestamp_ns"],
            "prediction_round_trip_ms": (finished_ns - started_ns) / 1e6,
            "joint_mae_rad": float(np.mean(np.abs(error[:6]))),
            "gripper_abs_error_m_nominal": float(abs(error[6])),
            "predicted_first_action": prediction[0].tolist(),
            "command_target": target.tolist(),
        })
    return {
        "schema": "piper_action_shadow_replay_v1",
        "status": "PREDICTION_ONLY_REPLAY_COMPLETED",
        "raw_episode_id": metadata["raw_episode_id"],
        "aligned_episode_id": metadata["episode_id"],
        "action_target_source": ACTION_TARGET_SOURCE,
        "action_semantics": ACTION_SEMANTICS,
        "source_verification": provenance,
        "server_identity": {key: server_metadata[key] for key in (
            "checkpoint_sha256", "policy_config_sha256", "policy_pipeline_code_sha256",
            "training_config_code_sha256", "normalization_sha256", "dataset_tree_sha256",
            "openpi_commit")},
        "sample_count": len(samples),
        "samples": samples,
        "mean_joint_mae_rad": float(np.mean([item["joint_mae_rad"] for item in samples])),
        "mean_gripper_abs_error_m_nominal": float(np.mean(
            [item["gripper_abs_error_m_nominal"] for item in samples])),
        "prediction_only": True,
        "hardware_control_allowed": False,
        "camera_opened": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_episode", type=Path)
    parser.add_argument("aligned_episode", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8004)
    parser.add_argument("--max-samples", type=int, default=20)
    parser.add_argument("--timeout-s", type=float, default=15)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # Fail on source/data issues before making any network connection.
    _load_verified_episode(args.raw_episode, args.aligned_episode)
    client = _bounded_ws_client(args.host, args.port, args.timeout_s)
    try:
        report = replay(args.raw_episode, args.aligned_episode, client,
                        max_samples=args.max_samples)
    finally:
        client.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    print(json.dumps({key: value for key, value in report.items() if key != "samples"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
