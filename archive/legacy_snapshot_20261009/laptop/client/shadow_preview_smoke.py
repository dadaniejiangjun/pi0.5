"""Send a recent passive PiPER preview to a prediction-only policy server.

No CAN, SDK interface, robot transport, or command sender is imported here.
This is a bounded nearline shadow smoke, not live robot control.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Callable

import numpy as np
from PIL import Image
from laptop.client.live_shadow import (
    DEFAULT_IDENTITY_LOCK,
    build_policy_prediction,
    load_artifact_identity_lock,
    validate_policy_metadata,
)
from laptop.client.network_latency_probe import _bounded_infer, _close_client
from laptop.client.observation_schema import PIPER_PROMPT, validate_piper_semantic_observation
from laptop.robot_client.policy_reject_only import audit_report
from laptop.runtime.config import RuntimeConfig
from laptop.runtime.modes import RuntimeMode
from laptop.recorder.verify_source import verify_source

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNTIME_CONFIG = PROJECT_ROOT / "configs/runtime/policy_reject_only.json"


def _client_factory(port: int) -> Callable[[], object]:
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    return lambda: WebsocketClientPolicy(host="127.0.0.1", port=port)


def run(
    raw: Path,
    preview: Path,
    *,
    port: int = 8003,
    max_samples: int = 3,
    timeout_s: float = 10.0,
    identity_lock_path: Path = DEFAULT_IDENTITY_LOCK,
    runtime_config_path: Path = DEFAULT_RUNTIME_CONFIG,
    client_factory: Callable[[], object] | None = None,
) -> dict:
    if not 1 <= port <= 65535 or not 1 <= max_samples <= 3:
        raise ValueError("invalid port")
    if not 0.25 <= timeout_s <= 60:
        raise ValueError("timeout must be between 0.25 and 60 seconds")
    runtime = RuntimeConfig.load(runtime_config_path)
    if (runtime.mode is not RuntimeMode.POLICY_REJECT_ONLY
            or runtime.can_tx_enabled or runtime.command_tx_enabled):
        raise PermissionError("preview replay requires POLICY_REJECT_ONLY with TX disabled")
    identity_lock = load_artifact_identity_lock(identity_lock_path)
    source = verify_source(raw, preview)
    if source["source_frame_extraction_verified"] is not True:
        raise ValueError("preview images were not verified against raw video")
    meta = json.loads((preview / "aligned_episode.json").read_text(encoding="utf-8"))
    if (meta.get("source") != "passive_wiring_smoke"
            or meta.get("raw_episode_id") != raw.name
            or meta.get("training_eligible") is not False):
        raise ValueError("not a passive, non-trainable shadow preview")
    rows = [json.loads(line) for line in (preview / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(rows) < 3:
        raise ValueError("at least three aligned frames required")
    factory = client_factory or _client_factory(port)
    client = None
    results = []
    audit_inputs = []
    try:
        client = factory()
        metadata = client.get_server_metadata()
        validate_policy_metadata(metadata, identity_lock)
        for index in sorted({0, len(rows) // 2, len(rows) - 1})[:max_samples]:
            row = rows[index]
            with Image.open(preview / row["base_image"]) as image:
                base = np.asarray(image.convert("RGB"))
            with Image.open(preview / row["wrist_image"]) as image:
                wrist = np.asarray(image.convert("RGB"))
            state = np.asarray(
                row["joint_rad_nominal_only"] + [row["gripper_raw"] * 1e-6],
                dtype=np.float32,
            )
            observation = {"base_rgb": base, "wrist_rgb": wrist,
                           "state": state, "prompt": PIPER_PROMPT}
            validate_piper_semantic_observation(observation)
            observation_ns = time.perf_counter_ns()
            result = _bounded_infer(client, observation, timeout_s)
            response_ns = time.perf_counter_ns()
            prediction = build_policy_prediction(
                result,
                metadata,
                request_id=f"recorded-preview-{index}-{observation_ns}",
                request_sequence=index,
                observation_ns=observation_ns,
                response_ns=response_ns,
                expected_identity_lock=identity_lock,
            )
            latency_ms = (response_ns - observation_ns) / 1e6
            results.append({
                "frame_index": index,
                "state": state.tolist(),
                "inference_latency_ms": latency_ms,
                "first_predicted_proxy": list(prediction.actions[0]),
            })
            audit_inputs.append({
                "request_id": prediction.request_id,
                "request_sequence": prediction.request_sequence,
                "observation_built_ns": observation_ns,
                "response_ns": response_ns,
                "state": state.tolist(),
                # This is a saved sample, not a live robot connection/status.
                "robot_status": {},
                "can_timestamps_ns": {},
                "target_dt_s": [prediction.target_dt_s[0]],
                "first_predicted_proxy": list(prediction.actions[0]),
                "checkpoint_identity": prediction.checkpoint_identity,
                "config_identity": prediction.config_identity,
                "normalization_identity": prediction.normalization_identity,
                "dataset_identity": prediction.dataset_identity,
            })
    finally:
        _close_client(client)

    audit = audit_report(
        {"predictions": audit_inputs, "units_hardware_verified": False}, runtime
    )
    required_reasons = {
        "robot_disconnected",
        "state_units_unverified",
        "next_observed_state_proxy_is_not_command",
        "proxy_to_command_mapping_unverified",
        "runtime_mode_forbids_can_tx",
    }
    if audit["decision_count"] != len(results) or audit["transport_send_attempts"] != 0:
        raise AssertionError("recorded-preview audit count or zero-send invariant failed")
    for decision in audit["decisions"]:
        if (decision["hardware_control_allowed"] is not False
                or decision["transport_send_attempted"] is not False
                or not required_reasons.issubset(set(decision["reasons"]))):
            raise AssertionError("recorded preview did not fail closed as required")
    return {
        "status": "RECORDED_PREVIEW_REJECTED_NO_TX",
        "input_kind": "source_verified_historical_dual_camera_preview",
        "raw_episode": raw.name,
        "preview": preview.name,
        "source_verification": source,
        "samples": results,
        "safety_audit": audit,
        "checkpoint": metadata.get("checkpoint"),
        "action_semantics": metadata["action_semantics"],
        "training_eligible": False,
        "units_hardware_verified": False,
        "camera_opened": False,
        "can_opened": False,
        "robot_command_sent": False,
        "control_enabled": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", required=True, type=Path)
    parser.add_argument("--preview", required=True, type=Path)
    parser.add_argument("--port", type=int, default=8003)
    parser.add_argument("--max-samples", type=int, default=3)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = run(args.raw, args.preview, port=args.port,
                 max_samples=args.max_samples, timeout_s=args.timeout_s)
    serialized = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as stream:
            stream.write(serialized)
    print(serialized, end="")


if __name__ == "__main__":
    main()
