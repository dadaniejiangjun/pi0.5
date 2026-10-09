"""Bounded synthetic-observation benchmark for the private OpenPI WebSocket path.

This diagnostic uses the OpenPI client protocol and validates the prediction-only
artifact lock. It never opens cameras or CAN and cannot issue robot commands.
"""

from __future__ import annotations

import argparse
from datetime import date
import hashlib
import json
import math
from pathlib import Path
import socket
import statistics
import time
from typing import Callable

import numpy as np

from laptop.client.live_shadow import (
    IDENTITY_HASH_FIELDS,
    load_artifact_identity_lock,
    segment_latency,
    validate_policy_metadata,
)
from laptop.client.observation_schema import make_piper_proxy_warmup_observation


def percentile_summary(values: list[float]) -> dict[str, float]:
    """Return latency extrema and tail percentiles after finite-value checks."""
    if not values or any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("latency samples must be a non-empty list of finite non-negative values")
    data = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(data)),
        "p50": float(np.percentile(data, 50)),
        "p90": float(np.percentile(data, 90)),
        "p95": float(np.percentile(data, 95)),
        "p99": float(np.percentile(data, 99)),
        "max": float(np.max(data)),
        "mean": float(statistics.fmean(values)),
    }


def _bounded_infer(client: object, observation: dict, timeout_s: float) -> dict:
    """Use the pinned OpenPI client's WebSocket with a bounded receive timeout.

    `WebsocketClientPolicy.infer()` doesn't expose a receive timeout. This uses
    its existing packer/connection so the payload stays byte-for-byte protocol
    compatible while applying `ClientConnection.recv(timeout=...)`.
    """
    from openpi_client import msgpack_numpy

    connection = client._ws
    connection.send(client._packer.pack(observation))
    response = connection.recv(timeout=timeout_s)
    if isinstance(response, str):
        raise RuntimeError("policy server returned an error frame")
    decoded = msgpack_numpy.unpackb(response)
    if not isinstance(decoded, dict):
        raise ValueError("policy response is not an object")
    actions = np.asarray(decoded.get("actions"))
    if actions.shape != (16, 7) or not np.isfinite(actions).all():
        raise ValueError("policy response actions must be finite 16x7")
    return decoded


def _close_client(client: object | None) -> None:
    if client is None:
        return
    try:
        client._ws.close()
    except Exception:
        pass


def _client_factory(host: str, port: int) -> Callable[[], object]:
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    return lambda: WebsocketClientPolicy(host=host, port=port)


def _check_listener(host: str, port: int, timeout_s: float) -> None:
    with socket.create_connection((host, port), timeout=timeout_s):
        pass


def _identity_summary(metadata: dict) -> dict:
    fields = (
        "artifact_identity_schema",
        "config_schema",
        "checkpoint",
        "dataset",
        "normalization_identity",
        "action_semantics",
        "openpi_commit",
        *IDENTITY_HASH_FIELDS,
        "action_horizon",
        "first_target_dt_s",
        "prediction_only",
        "control_enabled",
        "robot_control_ready",
        "hardware_control_allowed",
    )
    return {field: metadata.get(field) for field in fields}


def benchmark(
    *,
    host: str,
    port: int,
    requests: int,
    warmup: int,
    timeout_s: float,
    identity_lock_path: Path,
    transport_label: str,
    client_factory: Callable[[], object] | None = None,
) -> dict:
    """Measure warmed requests and a client-close/reconnect handshake.

    The reconnect check deliberately closes and reopens only the client socket;
    it doesn't interrupt the remote service or alter the server process.
    """
    if not host or not 1 <= port <= 65535:
        raise ValueError("valid host and TCP port are required")
    if not 1 <= requests <= 1000 or not 1 <= warmup <= 20:
        raise ValueError("requests must be 1..1000 and warmup must be 1..20")
    if not 0.25 <= timeout_s <= 60:
        raise ValueError("timeout must be between 0.25 and 60 seconds")

    lock = load_artifact_identity_lock(identity_lock_path)
    observation = make_piper_proxy_warmup_observation()
    prompt_hash = hashlib.sha256(observation["prompt"].encode("utf-8")).hexdigest()
    state_hash = hashlib.sha256(np.asarray(observation["state"]).tobytes()).hexdigest()
    image_shapes = {
        name: list(np.asarray(observation[name]).shape)
        for name in ("base_rgb", "wrist_rgb")
    }
    raw_image_bytes = sum(np.asarray(observation[name]).nbytes for name in image_shapes)
    factory = client_factory or _client_factory(host, port)
    client = None
    metadata = None
    samples: list[dict] = []
    failures: list[dict] = []
    request_count = requests + warmup

    try:
        _check_listener(host, port, min(timeout_s, 5.0))
        client = factory()
        metadata = client.get_server_metadata()
        validate_policy_metadata(metadata, lock)

        for index in range(request_count):
            started_ns = time.perf_counter_ns()
            try:
                result = _bounded_infer(client, observation, timeout_s)
                finished_ns = time.perf_counter_ns()
                roundtrip_ms = (finished_ns - started_ns) / 1e6
                segments = segment_latency(result, roundtrip_ms)
                if segments["client_roundtrip_ms"] > timeout_s * 1000:
                    raise TimeoutError("request exceeded configured wall-clock deadline")
                if index >= warmup:
                    samples.append({
                        "request_id": index - warmup,
                        "capture_monotonic_ns": None,
                        "send_time_ns": started_ns,
                        "receive_time_ns": finished_ns,
                        "prompt_sha256": prompt_hash,
                        "state_sha256": state_hash,
                        "timing": segments,
                    })
            except Exception as exc:
                failures.append({
                    "request_id": max(0, index - warmup),
                    "stage": "inference",
                    "error_type": type(exc).__name__,
                    "error": str(exc)[:240],
                })
                break
    except Exception as exc:
        failures.append({
            "request_id": 0,
            "stage": "connect_or_identity",
            "error_type": type(exc).__name__,
            "error": str(exc)[:240],
        })
    finally:
        _close_client(client)

    reconnect_ms = None
    reconnect_metadata_match = False
    reconnect_failure = None
    if not failures and len(samples) == requests:
        reconnect_started_ns = time.perf_counter_ns()
        reconnect_client = None
        try:
            _check_listener(host, port, min(timeout_s, 5.0))
            reconnect_client = factory()
            reconnect_metadata = reconnect_client.get_server_metadata()
            validate_policy_metadata(reconnect_metadata, lock)
            reconnect_metadata_match = reconnect_metadata == metadata
            if not reconnect_metadata_match:
                raise PermissionError("metadata changed after client reconnect")
            reconnect_ms = (time.perf_counter_ns() - reconnect_started_ns) / 1e6
        except Exception as exc:
            reconnect_failure = {
                "error_type": type(exc).__name__,
                "error": str(exc)[:240],
            }
        finally:
            _close_client(reconnect_client)

    timing_keys = (
        "model_sampling_ms",
        "server_non_model_ms",
        "client_server_residual_ms",
        "client_roundtrip_ms",
    )
    summaries = {
        key: percentile_summary([sample["timing"][key] for sample in samples])
        for key in timing_keys
    } if samples else {}
    success = (
        not failures
        and len(samples) == requests
        and reconnect_failure is None
        and reconnect_metadata_match
    )
    return {
        "schema": "piper_policy_network_latency_probe_v1",
        "date": date.today().isoformat(),
        "status": "PASS_SYNTHETIC_REQUEST_BENCHMARK" if success else "FAIL_OR_INCOMPLETE",
        "transport": {
            "label": transport_label,
            "endpoint": f"{host}:{port}",
            "persistent_network_selection_changed": False,
            "server_publicly_exposed": False,
        },
        "payload": {
            "kind": "synthetic_dual_rgb_state",
            "image_shapes_hwc_rgb": image_shapes,
            "raw_image_bytes": raw_image_bytes,
            "state_shape": list(np.asarray(observation["state"]).shape),
            "prompt_sha256": prompt_hash,
            "state_sha256": state_hash,
        },
        "artifact_identity": _identity_summary(metadata) if metadata else None,
        "requests": {
            "warmup": warmup,
            "requested_measured": requests,
            "measured_successfully": len(samples),
            "failures": failures,
            "configured_per_request_timeout_s": timeout_s,
        },
        "latency_ms": summaries,
        "reconnect": {
            "method": "client close followed by new WebSocket metadata handshake",
            "elapsed_ms": reconnect_ms,
            "metadata_identity_matches": reconnect_metadata_match,
            "failure": reconnect_failure,
            "remote_service_interrupted": False,
        },
        "p3_gate_assessment": {
            "synthetic_dual_rgb_state_1000_request_baseline": (
                "PASS" if success and requests == 1000 else "NOT_RUN_OR_FAILED"
            ),
            "persistent_network_option_selected": False,
            "network_drop_and_recovery_fault_injected": False,
            "all_payload_profiles_in_network_template_tested": False,
            "overall": "PARTIAL",
        },
        "samples": samples,
        "safety": {
            "synthetic_observation_only": True,
            "camera_opened": False,
            "can_opened": False,
            "robot_command_sent": False,
            "hardware_control_allowed": False,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--requests", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--transport-label", default="NET-A_TEMPORARY_SSH_TUNNEL")
    parser.add_argument("--identity-lock", type=Path, default=(
        Path(__file__).resolve().parents[2]
        / "configs" / "runtime" / "pi05_proxy_shadow_identity_lock.json"
    ))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = benchmark(
        host=args.host,
        port=args.port,
        requests=args.requests,
        warmup=args.warmup,
        timeout_s=args.timeout_s,
        identity_lock_path=args.identity_lock,
        transport_label=args.transport_label,
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "status": report["status"],
        "requests": report["requests"],
        "latency_ms": report["latency_ms"],
        "reconnect": report["reconnect"],
        "out": str(args.out),
    }, indent=2))
    if report["status"] != "PASS_SYNTHETIC_REQUEST_BENCHMARK":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
