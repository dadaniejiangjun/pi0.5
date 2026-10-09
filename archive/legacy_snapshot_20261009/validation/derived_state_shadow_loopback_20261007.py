"""Run a prediction-only WebSocket loopback on three saved LeRobot frames.

The server mode binds to 127.0.0.1. The client mode reads the already-prepared
dataset and never imports a robot, camera, SDK, or CAN transport.
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path


PROJECT = Path("/mnt/ssd/users/mingkai/07_pi0.5")
OVERLAY = PROJECT / "cache/codex/derived_shadow_20261007_r1"
SOURCE_ROOT = OVERLAY / "src"
RUNTIME_ROOT = PROJECT / "cache/codex/post_capture_train_20261007_r4/runtime_root"
DATASET = PROJECT / "cache/codex/prepare_overlay_20261007_r1/lerobot/local/piper_prepare_20261007_r1"
CHECKPOINT = RUNTIME_ROOT / "checkpoints/pi05_derived_state_piper_prepare_20261007_r1/piper_prepare_20261007_r1_pilot/19"
TRAINING = CHECKPOINT.parent / "training_provenance.json"
IDENTITY_LOCK = RUNTIME_ROOT / "validation/derived_state_shadow_identity_piper_prepare_20261007_r1.json"
REPORT = OVERLAY / "shadow_report.json"
HOST = "127.0.0.1"
PORT = 8021


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _install_import_paths() -> None:
    sys.path.insert(0, str(SOURCE_ROOT))
    sys.path.insert(1, str(RUNTIME_ROOT / "third_party/openpi/src"))


def serve() -> int:
    _install_import_paths()
    import server.training.proxy_multi_episode_pilot as pilot
    import server.serve_derived_state_multi_shadow as shadow_server

    pilot.ROOT = RUNTIME_ROOT
    shadow_server.ROOT = RUNTIME_ROOT
    sys.argv = [
        str(SOURCE_ROOT / "server/serve_derived_state_multi_shadow.py"),
        "--dataset-path", str(DATASET),
        "--checkpoint", str(CHECKPOINT),
        "--training-provenance", str(TRAINING),
        "--host", HOST,
        "--port", str(PORT),
        "--sampling-steps", "10",
    ]
    return shadow_server.main()


def _assert_server_identity(metadata: dict, training: dict, identity: dict) -> None:
    expected = {
        "artifact_identity_schema": identity["artifact_identity_schema"],
        "config_schema": identity["config_schema"],
        "dataset": identity["dataset"],
        "dataset_sha256": identity["dataset_sha256"],
        "action_semantics": identity["action_semantics"],
        "action_target_kind": identity["action_target_kind"],
        "action_horizon": identity["action_horizon"],
        "action_dimension": identity["action_dimension"],
        "checkpoint_tree_sha256": identity["checkpoint_tree_sha256"],
        "normalization_identity": identity["normalization_identity"],
        "normalization_sha256": identity["normalization_sha256"],
        "openpi_commit": identity["openpi_commit"],
        "training_provenance_sha256": identity["training_provenance_sha256"],
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise ValueError(f"server metadata identity mismatch: {key}")
    if _sha256(TRAINING) != identity["training_provenance_sha256"]:
        raise ValueError("training provenance hash differs from the pinned identity lock")
    if training.get("dataset_sha256") != identity["dataset_sha256"]:
        raise ValueError("training dataset differs from the pinned identity lock")
    if training.get("checkpoint_tree_sha256") != identity["checkpoint_tree_sha256"]:
        raise ValueError("training checkpoint differs from the pinned identity lock")
    if training.get("norm_stats_sha256") != identity["normalization_sha256"]:
        raise ValueError("training normalization differs from the pinned identity lock")
    if training.get("openpi_commit") != identity["openpi_commit"]:
        raise ValueError("training OpenPI commit differs from the pinned identity lock")
    for key in ("prediction_only",):
        if metadata.get(key) is not True:
            raise ValueError(f"server must assert {key}=true")
    for key in ("hardware_control_allowed", "robot_control_ready", "control_enabled", "can_opened"):
        if metadata.get(key) is not False:
            raise ValueError(f"server must assert {key}=false")
    if metadata.get("can_data_tx_calls") != 0 or metadata.get("raw_command_supervision") is not False:
        raise ValueError("server metadata must confirm prediction-only, zero-CAN operation")


def probe() -> dict:
    _install_import_paths()
    import numpy as np
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    training = _load_json(TRAINING)
    identity = _load_json(IDENTITY_LOCK)
    dataset = LeRobotDataset(training["repo_id"], root=DATASET)
    client = None
    samples = []
    try:
        client = WebsocketClientPolicy(host=HOST, port=PORT)
        metadata = client.get_server_metadata()
        _assert_server_identity(metadata, training, identity)

        for index in (0, 99, 198):
            row = dataset[index]

            def as_array(value):
                return value.numpy() if hasattr(value, "numpy") else np.asarray(value)

            observation = {
                "base_rgb": as_array(row["observation.images.image"]),
                "wrist_rgb": as_array(row["observation.images.wrist_image"]),
                "state": as_array(row["observation.state"]).astype(np.float32, copy=False),
                "prompt": row["task"],
            }
            if observation["state"].shape != (7,) or not np.isfinite(observation["state"]).all():
                raise ValueError(f"saved observation row {index} has invalid 7D state")
            if not isinstance(observation["prompt"], str) or not observation["prompt"]:
                raise ValueError(f"saved observation row {index} has no task text")

            start_ns = time.perf_counter_ns()
            response = client.infer(observation)
            end_ns = time.perf_counter_ns()
            actions = np.asarray(response["actions"], dtype=np.float32)
            if actions.shape != (16, 7) or not np.isfinite(actions).all():
                raise ValueError(f"row {index} returned invalid action shape or non-finite values")
            samples.append({
                "dataset_row_index": index,
                "inference_latency_ms": (end_ns - start_ns) / 1e6,
                "first_predicted_proxy": actions[0].tolist(),
                "action_shape": list(actions.shape),
            })
    finally:
        websocket = getattr(client, "_ws", None)
        close = getattr(websocket, "close", None)
        if callable(close):
            close()

    report = {
        "schema": "pi05_derived_state_shadow_loopback_v1",
        "status": "HISTORICAL_OBSERVATION_LOOPBACK_PASS_PREDICTION_ONLY",
        "input_kind": "three_saved_lerobot_observations",
        "dataset": training["repo_id"],
        "dataset_sha256": identity["dataset_sha256"],
        "checkpoint_tree_sha256": identity["checkpoint_tree_sha256"],
        "training_provenance_sha256": identity["training_provenance_sha256"],
        "action_semantics": identity["action_semantics"],
        "samples": samples,
        "held_out_evaluation": False,
        "generalization_claim": False,
        "hardware_success_claim": False,
        "prediction_only": True,
        "can_opened": False,
        "can_data_tx_calls": 0,
        "camera_opened": False,
        "robot_control_ready": False,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    with REPORT.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return report


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in {"server", "client"}:
        raise SystemExit("usage: derived_state_shadow_loopback_20261007.py {server|client}")
    if sys.argv[1] == "server":
        raise SystemExit(serve())
    probe()
