"""Read-only ROS 2 shadow inference node. This module has no robot command path."""
from __future__ import annotations

from collections import OrderedDict
from concurrent.futures import Future, ThreadPoolExecutor
import os
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def _project_root() -> Path:
    default = Path(__file__).resolve().parents[4]
    return Path(os.environ.get("PI05_PROJECT_ROOT", str(default))).resolve()


class PolicyShadowNode(Node):
    """Consumes ROS observations and publishes validated proxy predictions only."""

    def __init__(self) -> None:
        super().__init__("pi05_policy_shadow")
        self.project_root = _project_root()
        if str(self.project_root) not in sys.path:
            sys.path.insert(0, str(self.project_root))
        client_source = Path(os.environ.get(
            "OPENPI_CLIENT_SOURCE",
            str(self.project_root / "third_party" / "openpi" / "packages" / "openpi-client" / "src"),
        )).resolve()
        runtime_deps = Path(os.environ.get(
            "OPENPI_RUNTIME_DEPS", "/home/mingkai/piper_ros_ws/runtime-deps"
        )).resolve()
        if runtime_deps.exists() and str(runtime_deps) not in sys.path:
            sys.path.insert(0, str(runtime_deps))
        if client_source.exists() and str(client_source) not in sys.path:
            sys.path.insert(0, str(client_source))

        self.declare_parameter("policy_host", "127.0.0.1")
        self.declare_parameter("policy_port", 8021)
        self.declare_parameter("policy_backend", "openpi")
        self.declare_parameter(
            "identity_lock_path",
            str(self.project_root / "validation" / "live5000_identity_20261008.json"),
        )
        self.declare_parameter("allow_fake_observations", False)
        self.declare_parameter("connect_timeout_s", 5.0)
        self.declare_parameter("request_timeout_s", 15.0)
        self.declare_parameter("inference_hz", 10.0)
        self.declare_parameter("completion_poll_ms", 5.0)
        self.declare_parameter("timing_log", "")
        self.declare_parameter("max_camera_pair_skew_ms", 100.0)
        self.policy_host = str(self.get_parameter("policy_host").value)
        self.policy_port = int(self.get_parameter("policy_port").value)
        self.policy_backend = str(self.get_parameter("policy_backend").value)
        if self.policy_backend not in ("openpi", "lerobot_pi05"):
            raise ValueError("policy_backend must be openpi or lerobot_pi05")
        self.identity_lock_path = Path(str(self.get_parameter("identity_lock_path").value))
        self.allow_fake = bool(self.get_parameter("allow_fake_observations").value)
        self.connect_timeout_s = float(self.get_parameter("connect_timeout_s").value)
        self.request_timeout_s = float(self.get_parameter("request_timeout_s").value)
        self.inference_hz = float(self.get_parameter("inference_hz").value)
        self.max_camera_pair_skew_ms = float(
            self.get_parameter("max_camera_pair_skew_ms").value
        )
        if not 1.0 <= self.inference_hz <= 30.0:
            raise ValueError("inference_hz must be within 1..30")
        if not 1.0 <= self.max_camera_pair_skew_ms <= 100.0:
            raise ValueError("max_camera_pair_skew_ms must be within 1..100")
        if not 1.0 <= self.connect_timeout_s <= 30.0:
            raise ValueError("connect_timeout_s must be within 1..30 seconds")
        if not 1.0 <= self.request_timeout_s <= 120.0:
            raise ValueError("request_timeout_s must be within 1..120 seconds")

        from laptop.client.policy_client import (
            load_artifact_identity_lock,
            prewarm_policy,
            validate_policy_metadata,
        )
        from pi05_piper_interfaces.msg import CameraFrameMetadata, PolicyChunk, StateSnapshot
        from .shadow_core import build_shadow_observation, validate_policy_actions
        from laptop.camera.pairing import select_latest_camera_pair
        from .openpi_websocket import OpenPiWebsocketPolicy

        self._load_artifact_identity_lock = load_artifact_identity_lock
        self._prewarm_policy = prewarm_policy
        self._validate_policy_metadata = validate_policy_metadata
        self._validate_policy_actions = validate_policy_actions
        self._build_shadow_observation = build_shadow_observation
        self._select_latest_camera_pair = select_latest_camera_pair
        self._client_type = OpenPiWebsocketPolicy
        self._identity_lock = self._load_artifact_identity_lock(self.identity_lock_path)

        self.state_sub = self.create_subscription(
            StateSnapshot, "/piper/state_snapshot", self._on_state, qos_profile_sensor_data
        )
        self.image_subs = {
            role: self.create_subscription(
                Image, f"/piper/cameras/{role}/image_raw",
                lambda message, selected_role=role: self._on_image(selected_role, message),
                qos_profile_sensor_data,
            )
            for role in ("base", "wrist")
        }
        self.metadata_sub = self.create_subscription(
            CameraFrameMetadata, "/piper/cameras/frame_metadata",
            self._on_metadata, qos_profile_sensor_data,
        )
        self.chunk_pub = self.create_publisher(PolicyChunk, "/pi05/policy_chunk", 10)
        self.status_pub = self.create_publisher(DiagnosticArray, "/pi05/policy/status", 10)

        self._state: Any | None = None
        self._images = {role: OrderedDict() for role in ("base", "wrist")}
        self._metadata = {role: OrderedDict() for role in ("base", "wrist")}
        self._last_sequences = {"state": 0, "base": 0, "wrist": 0}
        self._last_submitted_sequences: dict[str, int] | None = None
        self._last_status = "connecting to policy server"
        self._last_error = ""
        self._last_latency_ms: float | None = None
        self._last_observation_id = ""
        self._last_inference_ns = 0
        self._prediction_count = 0
        self._camera_pair_reject_count = 0
        self._last_camera_pair_skew_ms: float | None = None
        self._reject_counts: dict[str, int] = {}
        self._client = None
        self._metadata_cache: dict[str, Any] | None = None
        self._connect_future: Future | None = None
        self._inference_future: Future | None = None
        self._retry_after_ns = 0
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pi05-policy")
        self._connect_future = self._executor.submit(self._connect_and_prewarm)
        self._next_request_ns = 0
        poll_ms = float(self.get_parameter("completion_poll_ms").value)
        if not 1 <= poll_ms <= 1000:
            raise ValueError("completion_poll_ms outside 1..1000")
        self._timing_path = str(self.get_parameter("timing_log").value)
        self._timer = self.create_timer(poll_ms / 1000, self._on_timer)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "read-only shadow started; it publishes /pi05/policy_chunk only and never sends robot commands"
        )

    @staticmethod
    def _header_key(message: Any) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _remember(cache: OrderedDict, key: int, value: Any) -> None:
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > 8:
            cache.popitem(last=False)

    def _on_state(self, message: Any) -> None:
        if self._state is not None and message.session_id != self._state.session_id:
            for cache in (*self._images.values(), *self._metadata.values()):
                cache.clear()
            self._last_sequences = {"state": 0, "base": 0, "wrist": 0}
            self._last_submitted_sequences = None
            self._last_status = "gateway session changed; waiting for new camera frames"
        self._state = message

    def _on_image(self, role: str, message: Any) -> None:
        if message.encoding != "bgr8" or message.width <= 0 or message.height <= 0:
            self._reject_counts["malformed image header"] = self._reject_counts.get("malformed image header", 0) + 1
            return
        key = self._header_key(message)
        self._remember(self._images[role], key, message)

    def _on_metadata(self, message: Any) -> None:
        if message.role not in self._metadata:
            self._reject_counts["unknown camera role"] = self._reject_counts.get("unknown camera role", 0) + 1
            return
        key = self._header_key(message)
        self._remember(self._metadata[message.role], key, message)

    def _matched_candidates(self, role: str, *, after_sequence: int) -> list[tuple[Any, Any]]:
        common = self._images[role].keys() & self._metadata[role].keys()
        candidates = []
        for stamp_ns in common:
            image = self._images[role][stamp_ns]
            metadata = self._metadata[role][stamp_ns]
            if int(metadata.sequence) <= after_sequence:
                continue
            candidates.append((image, metadata))
        return candidates

    def _connect_and_prewarm(self):
        client = self._client_type(
            self.policy_host,
            self.policy_port,
            connect_timeout_s=self.connect_timeout_s,
            request_timeout_s=self.request_timeout_s,
        )
        try:
            metadata = client.get_server_metadata()
            self._validate_policy_metadata(
                metadata, self._identity_lock, backend=self.policy_backend
            )
            warmup = self._prewarm_policy(client)
            return client, metadata, warmup
        except Exception:
            client.close()
            raise

    def _run_inference(self, observation: dict, timing: dict, client: Any, metadata: dict):
        started_ns = time.perf_counter_ns()
        result = client.infer(observation)
        finished_ns = time.perf_counter_ns()
        actions = self._validate_policy_actions(
            result, metadata, self._identity_lock, backend=self.policy_backend,
            request=observation,
        )
        return {
            "actions": actions.tolist(),
            "timing": timing,
            "latency_ms": (finished_ns - started_ns) / 1e6,
            "response_local_monotonic_ns": finished_ns,
            "model_metadata": metadata,
            "server_timing": result.get("server_timing", {}),
            "policy_timing": result.get("policy_timing", {}),
            "rtc": result.get("rtc"),
        }

    def _prepare_request(self, observation: dict, timing: dict) -> dict:
        """Main-thread extension hook; plain inference leaves inputs unchanged."""
        if self.policy_backend == "lerobot_pi05":
            return {
                **observation,
                "session_id": timing["session_id"],
                "request_id": timing["observation_id"],
                "observation_ns": timing["observation_local_monotonic_ns"],
            }
        return observation

    def _accept_connect_result(self) -> None:
        if self._connect_future is None or not self._connect_future.done():
            return
        future = self._connect_future
        self._connect_future = None
        try:
            self._client, self._metadata_cache, warmup = future.result()
            self._last_status = "policy ready; synthetic prewarm passed"
            self._last_error = ""
            checkpoint_hash = self._metadata_cache.get(
                "checkpoint_tree_sha256", self._metadata_cache.get("checkpoint_sha256", "")
            )
            self.get_logger().info(
                "policy service identity verified: "
                f"endpoint={self.policy_host}:{self.policy_port} "
                f"config={self._metadata_cache.get('config_schema', '')} "
                f"dataset={self._metadata_cache.get('dataset', '')} "
                f"checkpoint={self._metadata_cache.get('checkpoint', '')} "
                f"checkpoint_sha256={checkpoint_hash} "
                f"openpi_commit={self._metadata_cache.get('openpi_commit', '')}; "
                f"synthetic prewarm latency={warmup['latency_ms']:.1f} ms"
            )
        except Exception as exc:
            self._last_status = "policy connection or identity check failed"
            self._last_error = f"{type(exc).__name__}: {exc}"
            self._retry_after_ns = time.perf_counter_ns() + 2_000_000_000
            self.get_logger().warning(self._last_error)

    def _publish_chunk(self, output: dict) -> None:
        from pi05_piper_interfaces.msg import PolicyChunk

        timing = output["timing"]
        metadata = output["model_metadata"]
        if self._state is None or self._state.session_id != timing["session_id"]:
            self._last_status = "discarded prediction from a previous gateway session"
            return
        chunk = PolicyChunk()
        chunk.header.stamp.sec, chunk.header.stamp.nanosec = divmod(
            timing["state_header_stamp_ns"], 1_000_000_000
        )
        chunk.header.frame_id = timing["source_clock_id"]
        chunk.model_id = f"{metadata['config_schema']}:{metadata['dataset']}"
        checkpoint_hash = metadata.get("checkpoint_tree_sha256", metadata.get("checkpoint_sha256", ""))
        chunk.checkpoint_id = f"{metadata['checkpoint']}#sha256:{checkpoint_hash}"
        chunk.observation_id = timing["observation_id"]
        chunk.action_semantics = metadata["action_semantics"]
        chunk.source_clock_id = timing["source_clock_id"]
        chunk.source_monotonic_ns = timing["source_monotonic_ns"]
        chunk.local_clock_id = timing["local_clock_id"]
        chunk.observation_local_monotonic_ns = timing["observation_local_monotonic_ns"]
        chunk.response_local_monotonic_ns = output["response_local_monotonic_ns"]
        chunk.target_local_monotonic_ns = [
            chunk.observation_local_monotonic_ns + (index + 1) * 100_000_000
            for index in range(16)
        ]
        chunk.inference_latency_ns = int(output["latency_ms"] * 1e6)
        chunk.rows = 16
        chunk.columns = 7
        chunk.target_position = [value for row in output["actions"] for value in row]
        chunk.prediction_only = True
        chunk.hardware_control_allowed = False
        chunk.fake_observation = bool(timing["fake"])
        chunk.hardware_connected = bool(timing["hardware_connected"])
        self.chunk_pub.publish(chunk)
        if self._timing_path:
            import json
            record = dict(response_ns=output["response_local_monotonic_ns"],
                          publish_ns=time.perf_counter_ns(),
                          observation_ns=chunk.observation_local_monotonic_ns,
                          roundtrip_ms=output["latency_ms"],
                          server_timing=output["server_timing"], policy_timing=output["policy_timing"])
            with open(self._timing_path, "a") as stream:
                stream.write(json.dumps(record) + "\n")
        self._prediction_count += 1
        self._last_status = "prediction-only preview published; command path remains absent"
        self._last_error = ""
        self._last_latency_ms = output["latency_ms"]
        self._last_observation_id = timing["observation_id"]
        self._last_inference_ns = output["response_local_monotonic_ns"]

    def _on_timer(self) -> None:
        self._accept_connect_result()
        if self._inference_future is not None and self._inference_future.done():
            future = self._inference_future
            self._inference_future = None
            try:
                self._publish_chunk(future.result())
            except Exception as exc:
                self._last_status = "policy inference failed; prediction discarded"
                self._last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().warning(self._last_error)
                if self._client is not None:
                    self._client.close()
                self._client = None
                self._metadata_cache = None
                self._retry_after_ns = time.perf_counter_ns() + 1_000_000_000

        if self._client is None:
            if (self._connect_future is None
                    and time.perf_counter_ns() >= self._retry_after_ns):
                self._last_status = "connecting to policy server"
                self._connect_future = self._executor.submit(self._connect_and_prewarm)
            return
        if self._inference_future is not None:
            return
        if time.perf_counter_ns() < self._next_request_ns:
            return
        if self._state is None:
            self._last_status = "waiting for state snapshot"
            return
        previous = self._last_submitted_sequences or {"base": 0, "wrist": 0}
        base_candidates = self._matched_candidates(
            "base", after_sequence=previous["base"]
        )
        wrist_candidates = self._matched_candidates(
            "wrist", after_sequence=previous["wrist"]
        )
        pair, closest_skew_ns = self._select_latest_camera_pair(
            base_candidates,
            wrist_candidates,
            max_skew_ns=round(self.max_camera_pair_skew_ms * 1_000_000),
        )
        if pair is None:
            self._camera_pair_reject_count += 1
            self._last_camera_pair_skew_ms = (
                closest_skew_ns / 1e6 if closest_skew_ns is not None else None
            )
            if not base_candidates or not wrist_candidates:
                self._last_status = "waiting for new Base/Wrist images with matched metadata"
            else:
                self._last_status = (
                    "waiting for Base/Wrist host-after-read timestamps within "
                    f"{self.max_camera_pair_skew_ms:g} ms"
                )
            self._last_error = "exposure synchronization is not provided by host-after-read timestamps"
            return
        base, wrist, pair_skew_ns = pair
        self._last_camera_pair_skew_ms = pair_skew_ns / 1e6
        self._last_error = ""
        if base is None or wrist is None:
            self._last_status = "waiting for matched image and frame metadata"
            return
        candidate_sequences = {
            "state": int(self._state.sequence),
            "base": int(base[1].sequence),
            "wrist": int(wrist[1].sequence),
        }
        if (self._last_submitted_sequences is not None
                and any(candidate_sequences[key] <= self._last_submitted_sequences[key]
                        for key in candidate_sequences)):
            self._last_status = "waiting for a new state and both new camera frames"
            return
        try:
            observation, timing = self._build_shadow_observation(
                state=self._state,
                base_image=base[0], base_metadata=base[1],
                wrist_image=wrist[0], wrist_metadata=wrist[1],
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_monotonic_ns=time.perf_counter_ns(),
                previous_sequences=self._last_sequences,
                allow_fake=self.allow_fake,
                max_camera_skew_ns=round(self.max_camera_pair_skew_ms * 1_000_000),
            )
        except Exception as exc:
            reason = str(exc)
            self._reject_counts[reason] = self._reject_counts.get(reason, 0) + 1
            self._last_status = "observation rejected"
            self._last_error = reason
            return
        self._last_sequences = dict(timing["sequence_key"])
        self._last_submitted_sequences = dict(timing["sequence_key"])
        self._next_request_ns = time.perf_counter_ns() + round(1e9 / self.inference_hz)
        try:
            observation = self._prepare_request(observation, timing)
        except ValueError as exc:
            self._last_status = "request extension rejected"
            self._last_error = str(exc)
            return
        self._inference_future = self._executor.submit(
            self._run_inference, observation, timing, self._client, self._metadata_cache
        )
        self._last_status = "inference in progress"
        self._last_error = ""

    def _publish_status(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "pi05/policy_shadow"
        status.hardware_id = self.policy_host
        if self._client is None:
            status.level = DiagnosticStatus.WARN
        elif self._last_error:
            status.level = DiagnosticStatus.ERROR
        else:
            status.level = DiagnosticStatus.OK
        status.message = self._last_status
        values = {
            "policy_port": str(self.policy_port),
            "policy_backend": self.policy_backend,
            "prediction_count": str(self._prediction_count),
            "identity_lock": str(self.identity_lock_path),
            "allow_fake_observations": str(self.allow_fake).lower(),
            "hardware_command_path": "absent",
            "action_semantics": (
                self._metadata_cache.get("action_semantics", "next_observed_state_proxy_nominal_units")
                if self._metadata_cache else "next_observed_state_proxy_nominal_units"
            ),
            "last_observation_id": self._last_observation_id,
            "last_error": self._last_error,
            "reject_reason_count": str(len(self._reject_counts)),
            "camera_pairing_semantics": (
                "linux_ros_callback_arrival_not_exposure_sync"
                if self._state is not None and self._state.source_clock_id == "linux_perf_counter_ns"
                else "host_after_read_software_pairing_not_exposure_sync"),
            "camera_pair_skew_limit_ms": f"{self.max_camera_pair_skew_ms:g}",
            "camera_pair_reject_count": str(self._camera_pair_reject_count),
        }
        if self._last_camera_pair_skew_ms is not None:
            values["last_camera_pair_skew_ms"] = f"{self._last_camera_pair_skew_ms:.3f}"
        if self._last_latency_ms is not None:
            values["last_roundtrip_ms"] = f"{self._last_latency_ms:.3f}"
        if self._metadata_cache:
            for field in ("checkpoint", "dataset", "config_schema", "openpi_commit"):
                values[field] = str(self._metadata_cache.get(field, ""))
            values["checkpoint_tree_sha256"] = str(
                self._metadata_cache.get(
                    "checkpoint_tree_sha256", self._metadata_cache.get("checkpoint_sha256", "")
                )
            )
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message.status = [status]
        self.status_pub.publish(message)

    def close(self) -> None:
        self._timer.cancel()
        self._status_timer.cancel()
        if self._client is not None:
            self._client.close()
        self._executor.shutdown(wait=False, cancel_futures=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
