"""Bounded, prediction/control-independent live LeRobot episode writer.

The module has no ROS import and does not import LeRobot until its worker is
asked to write a sample. ``submit`` never waits for disk, image encoding, or
the LeRobot worker. When its queue is full, it rejects the frame and the worker
later writes a fully masked timeline placeholder so later frames keep their
original 10 Hz row time.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import queue
import re
import shutil
import threading
import time
from typing import Any, Callable, Mapping

import numpy as np

from .validation import assess_episode


FPS = 10
ACTION_NAMES = ["joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad",
                "joint_5_rad", "joint_6_rad", "gripper_nominal_m"]
STATE_NAMES = ACTION_NAMES.copy()
ACTION_SEMANTICS_DEFAULT = "kernel_accepted_position_target_nominal_unverified"
EXPECTED_PROXY_SEMANTICS = "next_observed_state_proxy_nominal_units"
RUNTIME_VERSIONS = {
    "lerobot_version": "0.1.0",
    "lerobot_revision": "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5",
    "datasets_version": "3.6.0",
    "rerun_sdk_version": "0.26.2",
}

_LATENCY_BUCKETS = 48
_QUEUE_TREND_BUCKET_S = 10
_QUEUE_TREND_BUCKETS = 360
DEFAULT_FINISH_TIMEOUT_S = 180.0
MAX_FINISH_TIMEOUT_S = 300.0


def _new_latency_stat() -> dict[str, Any]:
    return {"count": 0, "total_ns": 0, "max_ns": 0,
            "histogram": [0] * _LATENCY_BUCKETS}


def _record_latency(metrics: dict[str, Any], name: str, elapsed_ns: int) -> None:
    elapsed_ns = max(0, int(elapsed_ns))
    stat = metrics.setdefault(name, _new_latency_stat())
    stat["count"] += 1
    stat["total_ns"] += elapsed_ns
    stat["max_ns"] = max(stat["max_ns"], elapsed_ns)
    bucket = min(_LATENCY_BUCKETS - 1, max(0, elapsed_ns.bit_length() - 11))
    stat["histogram"][bucket] += 1


def _latency_percentile_ms(stat: dict[str, Any], percentile: float) -> float | None:
    count = int(stat["count"])
    if count <= 0:
        return None
    target = max(1, math.ceil(count * percentile))
    cumulative = 0
    for index, bucket_count in enumerate(stat["histogram"]):
        cumulative += int(bucket_count)
        if cumulative >= target:
            return round((2 ** (index + 11)) / 1_000_000, 4)
    return round((2 ** (_LATENCY_BUCKETS + 10)) / 1_000_000, 4)


def _latency_snapshot(metrics: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, stat in metrics.items():
        count = int(stat["count"])
        result[name] = {
            "count": count,
            "total_s": round(int(stat["total_ns"]) / 1e9, 6),
            "mean_ms": round(int(stat["total_ns"]) / count / 1e6, 4) if count else None,
            "max_ms": round(int(stat["max_ns"]) / 1e6, 4),
            "p50_ms_est": _latency_percentile_ms(stat, 0.50),
            "p95_ms_est": _latency_percentile_ms(stat, 0.95),
            "p99_ms_est": _latency_percentile_ms(stat, 0.99),
        }
    return result


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, bytes):
        return {"encoding": "hex", "data": value.hex()}
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def _json_write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False,
                   default=_json_default) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _jsonl_write(path: Path, rows: list[dict[str, Any]], *, append: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a" if append else "w", encoding="utf-8", newline="\n") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"),
                                    allow_nan=False, default=_json_default))
            handle.write("\n")
        handle.flush()


def estimate_storage(
    frames: int,
    height: int,
    width: int,
    *,
    queue_capacity: int,
    diagnostic_bytes_per_frame: int = 2048,
    reserve_bytes: int = 512 * 1024 * 1024,
) -> dict[str, int]:
    """Estimate peak local storage from the observed image shape and row count.

    The estimate counts two uncompressed RGB image sets for temporary frame
    files and a conservative encoded-video allowance, plus Parquet and the
    measured/estimated JSONL sidecar size. Queue memory is reported separately
    because those arrays are resident RAM, not persisted files.
    """
    if min(frames, height, width, queue_capacity) < 0 or min(height, width) == 0:
        raise ValueError("frames, image dimensions, and queue capacity must be valid")
    pair_raw = int(height) * int(width) * 3 * 2
    raw_temporary = int(frames) * pair_raw
    encoded_video = int(frames) * pair_raw
    parquet = int(frames) * 512
    diagnostics = int(frames) * max(256, int(diagnostic_bytes_per_frame))
    disk_peak = raw_temporary + encoded_video + parquet + diagnostics
    margin = max(int(reserve_bytes), math.ceil(disk_peak * 0.10))
    queue_memory = int(queue_capacity) * pair_raw
    return {
        "frames": int(frames),
        "raw_temporary_bytes": raw_temporary,
        "encoded_video_estimate_bytes": encoded_video,
        "parquet_estimate_bytes": parquet,
        "diagnostic_estimate_bytes": diagnostics,
        "estimated_peak_disk_bytes": disk_peak,
        "reserve_margin_bytes": margin,
        "required_free_bytes": disk_peak + margin,
        "queue_memory_estimate_bytes": queue_memory,
    }


class LeRobotEpisodeWriter:
    """Write live samples to a pinned LeRobot 0.1.0 v2.1 dataset.

    ``submit`` performs a bounded queue admission check and a small, explicit
    copy of each image. All LeRobot calls, video encoding, pending markers,
    stats, and provenance files run on a daemon worker thread.
    """

    def __init__(
        self,
        root: str | Path,
        *,
        fps: int = FPS,
        queue_capacity: int = 20,
        cache_root: str | Path | None = None,
        startup_timeout_s: float = 30.0,
        operation_timeout_s: float = 30.0,
        finish_timeout_s: float = DEFAULT_FINISH_TIMEOUT_S,
        close_timeout_s: float = 10.0,
        reserve_bytes: int = 512 * 1024 * 1024,
        storage_check_interval_frames: int = 25,
        _dataset_factory: Any = None,
        _disk_usage: Any = None,
    ) -> None:
        constructor_started_ns = time.perf_counter_ns()
        if fps != FPS:
            raise ValueError(f"the live schema uses a nominal {FPS} Hz timeline")
        if type(queue_capacity) is not int or queue_capacity < 1:
            raise ValueError("queue_capacity must be a positive integer")
        timeouts = (startup_timeout_s, operation_timeout_s,
                    finish_timeout_s, close_timeout_s)
        if any(not math.isfinite(float(value)) or value <= 0 for value in timeouts):
            raise ValueError("timeouts must be positive")
        if finish_timeout_s > MAX_FINISH_TIMEOUT_S:
            raise ValueError(f"finish_timeout_s must not exceed {MAX_FINISH_TIMEOUT_S:g} seconds")
        if reserve_bytes < 0 or storage_check_interval_frames < 1:
            raise ValueError("reserve_bytes and storage_check_interval_frames are invalid")

        self.root = Path(root).expanduser().resolve()
        self.fps = fps
        self.queue_capacity = queue_capacity
        self.cache_root = (Path(cache_root).expanduser().resolve() if cache_root is not None
                           else self.root.parent / f".{self.root.name}_live_writer_cache")
        self.startup_timeout_s = float(startup_timeout_s)
        self.operation_timeout_s = float(operation_timeout_s)
        self.finish_timeout_s = float(finish_timeout_s)
        self.close_timeout_s = float(close_timeout_s)
        self.reserve_bytes = int(reserve_bytes)
        self.storage_check_interval_frames = int(storage_check_interval_frames)
        self._dataset_factory = _dataset_factory
        self._disk_usage = _disk_usage or shutil.disk_usage

        self._samples: queue.Queue[tuple[int, dict[str, Any], int]] = queue.Queue(maxsize=queue_capacity)
        # A single active episode allows at most one pending start command.
        # Keep spare capacity for future lifecycle commands without an unbounded queue.
        self._commands: queue.Queue[tuple[str, Any, threading.Event, dict[str, Any]]] = queue.Queue(maxsize=4)
        self._lock = threading.RLock()
        self._metrics_lock = threading.Lock()
        self._global_timings: dict[str, Any] = {}
        self._constructor_started_ns = constructor_started_ns
        self._worker_ready_ns: int | None = None
        self._runtime_cli: Any = None
        self._prewarmed_runtime: dict[str, Any] | None = None
        self._lerobot_dataset_type: Any = None
        self._worker_ready = threading.Event()
        self._stop_requested = threading.Event()
        self._current: dict[str, Any] | None = None
        self._accepting = False
        self._finish_request: dict[str, Any] | None = None
        self._closed = False
        self._worker_error: str | None = None
        self._writer_state = "WARMING_UP"
        self._last_storage_estimate: dict[str, int] | None = None
        self._last_free_bytes: int | None = None

        self._thread = threading.Thread(
            target=self._worker, name="lerobot-live-dataset-writer", daemon=True
        )
        self._thread.start()
        if not self._worker_ready.wait(self.startup_timeout_s):
            with self._lock:
                self._writer_state = "STARTUP_TIMEOUT"
                self._closed = True
            self._stop_requested.set()
            raise TimeoutError("live dataset writer worker did not start before timeout")
        self._worker_ready_ns = time.perf_counter_ns()
        with self._metrics_lock:
            _record_latency(self._global_timings, "writer_constructor_to_worker_ready",
                            self._worker_ready_ns - self._constructor_started_ns)
        with self._lock:
            if self._worker_error:
                raise RuntimeError(self._worker_error)
            self._writer_state = "IDLE"

    def start_episode(
        self,
        run_id: str,
        task: str,
        metadata: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError("run_id must be a non-empty string")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task must be a non-empty string")
        metadata = deepcopy(metadata or {})
        timeout = self.operation_timeout_s if timeout_s is None else float(timeout_s)
        if timeout <= 0:
            raise ValueError("timeout_s must be positive")

        with self._lock:
            if self._closed or not self._thread.is_alive():
                raise RuntimeError("writer is closed or its worker has exited")
            if self._worker_error:
                raise RuntimeError(f"writer worker failed: {self._worker_error}")
            if self._current is not None:
                raise RuntimeError("an episode is already active or finishing")
            context = {
                "run_id": run_id,
                "task": task,
                "metadata": metadata,
                "started_at_utc": _utc_now(),
                "events": [{"event": "episode_start", "run_id": run_id,
                            "task": task, "time_utc": _utc_now()}],
                "state": "STARTING",
                "cancel_start": False,
                "attempted_frames": 0,
                "accepted_frames": 0,
                "written_frames": 0,
                "placeholder_frames": 0,
                "dropped_ranges": [],
                "valid_actions": [],
                "episode_index": None,
                "dataset_row_start": None,
                "image_shape": None,
                "admission_image_shape": None,
                "diagnostic_bytes": 0,
                "last_storage_check_at": 0,
                "label": None,
                "pending_path": None,
                "stage_map_path": None,
                "stage_map_handle": None,
                "runtime": None,
                "repo_id": None,
                "dataset": None,
                "episode_started_monotonic_ns": time.perf_counter_ns(),
                "input_end_monotonic_ns": None,
                "drain_complete_monotonic_ns": None,
                "finish_started_monotonic_ns": None,
                "finish_completed_monotonic_ns": None,
                "timings": {},
                "queue_depth_peak": 0,
                "queue_depth_trend": {},
                "image_writer": None,
            }
            self._current = context
            self._accepting = False
            self._finish_request = None
            self._writer_state = "STARTING_EPISODE"
            done = threading.Event()
            result: dict[str, Any] = {}
            try:
                self._commands.put_nowait(("start", context, done, result))
            except queue.Full:
                self._current = None
                self._writer_state = "IDLE"
                raise RuntimeError("writer lifecycle command queue is full")

        if not done.wait(timeout):
            with self._lock:
                context["cancel_start"] = True
                self._accepting = False
                if self._current is context:
                    self._writer_state = "START_TIMEOUT"
            return {"status": "START_TIMEOUT", "run_id": run_id,
                    "worker_alive": self._thread.is_alive()}
        if result.get("error"):
            raise RuntimeError(result["error"])
        return result

    def submit(self, sample: Mapping[str, Any]) -> bool:
        """Attempt one non-blocking frame admission; return False on any loss."""
        submit_started_ns = time.perf_counter_ns()
        context: dict[str, Any] | None = None
        try:
            with self._lock:
                context = self._current
                if not isinstance(sample, Mapping):
                    if context is not None and self._accepting:
                        index = int(context["attempted_frames"])
                        context["attempted_frames"] += 1
                        self._record_drop(context, index, "invalid_sample_type")
                    return False
                if (context is None or not self._accepting or self._closed
                        or self._worker_error or not self._thread.is_alive()):
                    if context is not None and context.get("state") == "RECORDING":
                        self._record_drop(context, context["attempted_frames"], "writer_unavailable", sample)
                        context["attempted_frames"] += 1
                    return False

                index = int(context["attempted_frames"])
                context["attempted_frames"] += 1
                if sample.get("run_id") != context["run_id"] or sample.get("task") != context["task"]:
                    self._record_drop(context, index, "sample_episode_identity_mismatch", sample)
                    return False
                if self._samples.full():
                    self._record_drop(context, index, "writer_queue_full", sample)
                    return False
                normalize_started_ns = time.perf_counter_ns()
                try:
                    normalized = self._normalize_sample(sample, context)
                except Exception as exc:
                    self._record_drop(context, index, f"invalid_sample:{type(exc).__name__}", sample)
                    return False
                finally:
                    self._record_context_latency(
                        context, "submit_normalize_and_copy",
                        time.perf_counter_ns() - normalize_started_ns,
                    )
                enqueued_ns = time.perf_counter_ns()
                try:
                    self._samples.put_nowait((index, normalized, enqueued_ns))
                except queue.Full:
                    self._record_drop(context, index, "writer_queue_full", sample)
                    return False
                context["accepted_frames"] += 1
                self._record_queue_depth(context, self._samples.qsize(), enqueued_ns)
                self._writer_state = "RECORDING"
                return True
        finally:
            if context is not None:
                self._record_context_latency(context, "submit_call",
                                             time.perf_counter_ns() - submit_started_ns)

    def finish_episode(
        self,
        label: str | dict[str, Any] = "UNREVIEWED",
        *,
        events: list[dict[str, Any]] | None = None,
        timeout_s: float | None = None,
        progress_callback: Callable[[str], None] | None = None,
    ) -> dict[str, Any]:
        return self._request_finish("finish", label=label, reason=None, events=events,
                                    timeout_s=timeout_s,
                                    progress_callback=progress_callback)

    def abort_episode(
        self,
        reason: str,
        *,
        events: list[dict[str, Any]] | None = None,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        return self._request_finish("abort", label=None, reason=reason, events=events,
                                    timeout_s=timeout_s)

    def _request_finish(
        self,
        mode: str,
        *,
        label: str | dict[str, Any] | None,
        reason: str | None,
        events: list[dict[str, Any]] | None,
        timeout_s: float | None,
        progress_callback: Callable[[str], None] | None = None,
    ) -> dict[str, Any]:
        default_timeout = self.finish_timeout_s if mode == "finish" else self.operation_timeout_s
        timeout = default_timeout if timeout_s is None else float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout_s must be positive")
        if mode == "finish" and timeout > MAX_FINISH_TIMEOUT_S:
            raise ValueError(f"finish timeout must not exceed {MAX_FINISH_TIMEOUT_S:g} seconds")
        with self._lock:
            context = self._current
            if context is None:
                if self._worker_error:
                    return {"status": "FAILED_UNCOMMITTED", "writer_state": self._writer_state,
                            "error": self._worker_error, "worker_alive": self._thread.is_alive()}
                return {"status": "NO_ACTIVE_EPISODE", "writer_state": self._writer_state}
            if self._finish_request is not None:
                request = self._finish_request
            else:
                self._accepting = False
                context["input_end_monotonic_ns"] = time.perf_counter_ns()
                completion = threading.Event()
                request = {
                    "mode": mode,
                    "label": deepcopy(label),
                    "reason": reason,
                    "events": deepcopy(events or []),
                    "completion": completion,
                    "report": {},
                    "progress_callback": progress_callback,
                }
                self._finish_request = request
                context["state"] = "FINALIZING" if mode == "finish" else "ABORTING"
                self._writer_state = context["state"]

        if not request["completion"].wait(timeout):
            with self._lock:
                if request["completion"].is_set():
                    return dict(request["report"])
                if mode == "finish":
                    context["finish_timed_out"] = True
                    context["finish_timeout_s"] = timeout
                self._writer_state = "FINISH_TIMEOUT" if mode == "finish" else "ABORT_TIMEOUT"
            return {"status": self._writer_state, "run_id": context["run_id"],
                    "worker_alive": self._thread.is_alive(),
                    "accepted_frames": context["accepted_frames"],
                    "dropped_frames": self._dropped_count(context),
                    "partial": True,
                    "error": f"episode finalization exceeded its {timeout:g}s wait; pending data was preserved",
                    "pending_path": str(context["pending_path"])
                    if context.get("pending_path") else None}
        return dict(request["report"])

    def status(self) -> dict[str, Any]:
        with self._lock:
            context = self._current
            profile = self._profile_snapshot(context) if context else None
            return {
                "schema": "piper_live_lerobot_writer_status_v1",
                "state": self._writer_state,
                "active_run_id": context["run_id"] if context else None,
                "fps": self.fps,
                "queue_capacity": self.queue_capacity,
                "queue_depth": self._samples.qsize(),
                "attempted_frames": context["attempted_frames"] if context else 0,
                "accepted_frames": context["accepted_frames"] if context else 0,
                "written_frames": context["written_frames"] if context else 0,
                "placeholder_frames": context["placeholder_frames"] if context else 0,
                "dropped_frames": self._dropped_count(context) if context else 0,
                "dropped_ranges": deepcopy(context["dropped_ranges"]) if context else [],
                "partial": bool(context and self._dropped_count(context) > 0)
                          or self._worker_error is not None,
                "worker_error": self._worker_error,
                "worker_alive": self._thread.is_alive(),
                "initialization_timings": self._global_timing_snapshot(),
                "performance": profile,
                "storage_estimate": deepcopy(self._last_storage_estimate),
                "available_bytes": self._last_free_bytes,
                "close_requested": self._closed,
            }

    def close(self) -> dict[str, Any]:
        with self._lock:
            if self._closed:
                active = self._current is not None
            else:
                active = self._current is not None
        report: dict[str, Any] = {"status": "CLOSED"}
        if active:
            report = self.abort_episode("writer_close", timeout_s=self.close_timeout_s)
        with self._lock:
            self._closed = True
        self._stop_requested.set()
        self._thread.join(timeout=self.close_timeout_s)
        alive = self._thread.is_alive()
        with self._lock:
            if alive:
                self._writer_state = "CLOSE_TIMEOUT"
            elif self._writer_state not in {"FAILED", "CLOSE_TIMEOUT"}:
                self._writer_state = "CLOSED"
        return {
            **report,
            "close_status": "CLOSE_TIMEOUT" if alive else "CLOSED",
            "worker_alive": alive,
            "worker_error": self._worker_error,
        }

    def _record_context_latency(self, context: dict[str, Any], name: str,
                                elapsed_ns: int) -> None:
        with self._metrics_lock:
            _record_latency(context["timings"], name, elapsed_ns)

    def _record_global_latency(self, name: str, elapsed_ns: int) -> None:
        with self._metrics_lock:
            _record_latency(self._global_timings, name, elapsed_ns)

    def _record_queue_depth(self, context: dict[str, Any], depth: int,
                            at_monotonic_ns: int | None = None) -> None:
        at_ns = time.perf_counter_ns() if at_monotonic_ns is None else int(at_monotonic_ns)
        elapsed_ns = max(0, at_ns - int(context["episode_started_monotonic_ns"]))
        bucket_index = min(_QUEUE_TREND_BUCKETS, elapsed_ns // (_QUEUE_TREND_BUCKET_S * 1_000_000_000))
        with self._metrics_lock:
            context["queue_depth_peak"] = max(int(context["queue_depth_peak"]), int(depth))
            trend = context["queue_depth_trend"]
            bucket = trend.setdefault(int(bucket_index), {
                "bucket_start_s": int(bucket_index) * _QUEUE_TREND_BUCKET_S,
                "samples": 0, "first_depth": int(depth), "last_depth": int(depth),
                "max_depth": int(depth), "depth_sum": 0,
            })
            bucket["samples"] += 1
            bucket["last_depth"] = int(depth)
            bucket["max_depth"] = max(int(bucket["max_depth"]), int(depth))
            bucket["depth_sum"] += int(depth)

    def _profile_snapshot(self, context: dict[str, Any]) -> dict[str, Any]:
        with self._metrics_lock:
            trend = []
            for index in sorted(context["queue_depth_trend"]):
                item = context["queue_depth_trend"][index]
                trend.append({
                    "bucket_start_s": item["bucket_start_s"],
                    "overflow_bucket": index >= _QUEUE_TREND_BUCKETS,
                    "samples": item["samples"],
                    "first_depth": item["first_depth"],
                    "last_depth": item["last_depth"],
                    "max_depth": item["max_depth"],
                    "mean_depth": round(item["depth_sum"] / item["samples"], 3),
                    "depth_delta": item["last_depth"] - item["first_depth"],
                })
            timings = _latency_snapshot(context["timings"])
            started_ns = int(context["episode_started_monotonic_ns"])
            input_end_ns = context.get("input_end_monotonic_ns")
            drain_ns = context.get("drain_complete_monotonic_ns")
            finish_ns = context.get("finish_completed_monotonic_ns")
            return {
                "episode_elapsed_s": round((time.perf_counter_ns() - started_ns) / 1e9, 6),
                "input_end_elapsed_s": round((input_end_ns - started_ns) / 1e9, 6)
                                      if input_end_ns is not None else None,
                "input_to_drain_s": round((drain_ns - input_end_ns) / 1e9, 6)
                                    if drain_ns is not None and input_end_ns is not None else None,
                "input_to_finish_end_s": round((finish_ns - input_end_ns) / 1e9, 6)
                                         if finish_ns is not None and input_end_ns is not None else None,
                "queue_depth_peak": int(context["queue_depth_peak"]),
                "queue_depth_trend_bucket_s": _QUEUE_TREND_BUCKET_S,
                "queue_depth_trend": trend,
                "timings": timings,
                "dropped_ranges": deepcopy(context["dropped_ranges"]),
            }

    def _global_timing_snapshot(self) -> dict[str, dict[str, Any]]:
        with self._metrics_lock:
            return _latency_snapshot(self._global_timings)

    def _record_drop(self, context: dict[str, Any], index: int, reason: str,
                     sample: Mapping[str, Any] | None = None) -> None:
        dropped_at_ns = time.perf_counter_ns()
        elapsed_s = max(0.0, (dropped_at_ns - int(context["episode_started_monotonic_ns"])) / 1e9)
        ranges = context["dropped_ranges"]
        if ranges and ranges[-1]["end_frame_index"] == index - 1 and ranges[-1]["reason"] == reason:
            ranges[-1]["end_frame_index"] = index
            ranges[-1]["frames"] += 1
            ranges[-1]["last_dropped_monotonic_ns"] = dropped_at_ns
            ranges[-1]["last_dropped_elapsed_s"] = round(elapsed_s, 6)
            if sample is not None:
                ranges[-1]["last_source_map"] = self._source_map_summary(sample)
        else:
            ranges.append({
                "start_frame_index": index,
                "end_frame_index": index,
                "frames": 1,
                "reason": reason,
                "first_dropped_monotonic_ns": dropped_at_ns,
                "last_dropped_monotonic_ns": dropped_at_ns,
                "first_dropped_elapsed_s": round(elapsed_s, 6),
                "last_dropped_elapsed_s": round(elapsed_s, 6),
                "first_source_map": self._source_map_summary(sample) if sample is not None else None,
                "last_source_map": self._source_map_summary(sample) if sample is not None else None,
            })

    @staticmethod
    def _source_map_summary(sample: Mapping[str, Any]) -> Any:
        source_map = sample.get("source_map")
        if source_map is None:
            return None
        try:
            copied = deepcopy(source_map)
            encoded = json.dumps(copied, ensure_ascii=False, separators=(",", ":"),
                                 default=_json_default)
        except Exception:
            return {"serialization_error": True}
        if len(encoded) > 4096:
            return {"truncated": True, "serialized_bytes": len(encoded),
                    "prefix": encoded[:2048]}
        return copied

    @staticmethod
    def _normalize_sample(sample: Mapping[str, Any], context: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(sample, Mapping):
            raise TypeError("sample must be a mapping")
        images = sample.get("images")
        if not isinstance(images, Mapping) or "base" not in images or "wrist" not in images:
            raise ValueError("sample.images must contain base and wrist RGB images")

        state = np.asarray(sample.get("state"), dtype=np.float32).reshape(-1)
        if state.shape != (7,):
            raise ValueError("sample.state must contain seven values")
        state_valid = bool(sample.get("observation_valid", False)) and bool(np.isfinite(state).all())
        if not np.isfinite(state).all():
            state = np.nan_to_num(state, nan=0.0, posinf=0.0, neginf=0.0)

        camera_flags = sample.get("camera_valid")
        if isinstance(camera_flags, np.ndarray):
            camera_flags = camera_flags.tolist()
        if not isinstance(camera_flags, (list, tuple)) or len(camera_flags) != 2:
            raise ValueError("sample.camera_valid must contain base and wrist flags")
        camera_flags = [bool(value) for value in camera_flags]

        image_arrays: dict[str, np.ndarray] = {}
        image_shape: tuple[int, int, int] | None = None
        for role in ("base", "wrist"):
            image = np.asarray(images[role])
            if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"sample.images.{role} must be HWC RGB uint8")
            if image_shape is None:
                image_shape = tuple(int(v) for v in image.shape)
            elif image_shape != tuple(image.shape):
                raise ValueError("base and wrist image dimensions must match")
            image_arrays[role] = np.ascontiguousarray(image).copy()
        if (context["admission_image_shape"] is not None
                and image_shape != context["admission_image_shape"]):
            raise ValueError("image dimensions changed inside a LeRobot episode")
        if context["admission_image_shape"] is None:
            context["admission_image_shape"] = image_shape

        action = sample.get("action")
        action_valid = bool(sample.get("action_valid", False))
        if action is None:
            action_array = np.zeros((7,), dtype=np.float32)
            action_valid = False
        else:
            action_array = np.asarray(action, dtype=np.float32).reshape(-1)
            if action_array.shape != (7,):
                raise ValueError("sample.action must contain seven values")
            if not np.isfinite(action_array).all():
                action_valid = False
                action_array = np.nan_to_num(action_array, nan=0.0, posinf=0.0, neginf=0.0)
            action_array = action_array.copy()
        if not action_valid:
            action_array.fill(0.0)

        source_map = sample.get("source_map", {})
        if not isinstance(source_map, Mapping):
            raise ValueError("sample.source_map must be a mapping")
        source_map = deepcopy(dict(source_map))
        return {
            "run_id": str(sample["run_id"]),
            "task": str(sample["task"]),
            "images": image_arrays,
            "state": state.copy(),
            "action": action_array,
            "action_valid": action_valid,
            "observation_valid": state_valid and bool(sample.get("observation_valid", False)),
            "camera_valid": camera_flags,
            "source_map": source_map,
        }

    def _worker(self) -> None:
        try:
            if self._dataset_factory is None:
                stage_started_ns = time.perf_counter_ns()
                from gamepad_capture.tools.dataset import cli
                self._runtime_cli = cli
                self._record_global_latency("dataset_cli_import", time.perf_counter_ns() - stage_started_ns)

                stage_started_ns = time.perf_counter_ns()
                self._prewarmed_runtime = cli._check_runtime()
                self._record_global_latency("pinned_runtime_check", time.perf_counter_ns() - stage_started_ns)

                stage_started_ns = time.perf_counter_ns()
                cli._configure_offline_cache(self.cache_root)
                self._record_global_latency("offline_cache_config", time.perf_counter_ns() - stage_started_ns)

                stage_started_ns = time.perf_counter_ns()
                from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
                self._lerobot_dataset_type = LeRobotDataset
                self._record_global_latency("lerobot_dataset_import", time.perf_counter_ns() - stage_started_ns)
        except Exception as exc:
            with self._lock:
                self._worker_error = f"writer warmup failed: {type(exc).__name__}: {exc}"
                self._writer_state = "FAILED"
        self._worker_ready_ns = time.perf_counter_ns()
        self._worker_ready.set()
        if self._worker_error:
            return
        while True:
            try:
                while True:
                    command, payload, done, result = self._commands.get_nowait()
                    if command == "start":
                        try:
                            self._prepare_start(payload)
                            with self._lock:
                                if self._current is not payload:
                                    raise RuntimeError("episode start was superseded")
                                if payload["cancel_start"]:
                                    self._current = None
                                    self._writer_state = "IDLE"
                                    result["error"] = "start_episode timed out and was cancelled"
                                    continue
                                payload["state"] = "RECORDING"
                                self._accepting = True
                                self._writer_state = "RECORDING"
                            result.update({"status": "RECORDING", "run_id": payload["run_id"],
                                           "fps": self.fps, "queue_capacity": self.queue_capacity})
                        except Exception as exc:
                            result["error"] = f"start_episode failed: {type(exc).__name__}: {exc}"
                            with self._lock:
                                self._writer_state = "IDLE"
                                self._accepting = False
                                if self._current is payload:
                                    self._current = None
                        finally:
                            done.set()
                    elif command == "shutdown":
                        self._stop_requested.set()
                        done.set()
            except queue.Empty:
                pass

            try:
                item = self._samples.get(timeout=0.05)
            except queue.Empty:
                item = None
            if item is not None:
                frame_index, sample, enqueued_ns = item
                write_started_ns: int | None = None
                try:
                    context = self._current
                    if context is None:
                        continue
                    dequeued_ns = time.perf_counter_ns()
                    self._record_context_latency(context, "sample_queue_wait",
                                                 dequeued_ns - enqueued_ns)
                    self._record_queue_depth(context, self._samples.qsize(), dequeued_ns)
                    if self._worker_error:
                        self._record_drop(context, frame_index, "writer_failure")
                        continue
                    write_started_ns = time.perf_counter_ns()
                    self._write_sample(context, frame_index, sample)
                    self._record_context_latency(context, "write_sample_total",
                                                 time.perf_counter_ns() - write_started_ns)
                except Exception as exc:
                    context = self._current
                    if context is not None:
                        self._record_drop(context, frame_index, "writer_failure")
                        if write_started_ns is not None:
                            self._record_context_latency(context, "write_sample_total",
                                                         time.perf_counter_ns() - write_started_ns)
                    self._fail_worker(exc)
                finally:
                    self._samples.task_done()

            request = None
            with self._lock:
                if self._finish_request is not None and self._samples.empty():
                    request = self._finish_request
            if request is not None:
                context = self._current
                if context is not None:
                    try:
                        context["drain_complete_monotonic_ns"] = time.perf_counter_ns()
                        context["finish_started_monotonic_ns"] = context["drain_complete_monotonic_ns"]
                        self._notify_progress(request, "queue_drained")
                        request["report"].update(self._finish(context, request))
                    except Exception as exc:
                        self._fail_worker(exc, schedule_finalize=False)
                        pending_path = context.get("pending_path")
                        uncommitted = bool(pending_path is not None and pending_path.exists())
                        context["finish_completed_monotonic_ns"] = time.perf_counter_ns()
                        request["report"].update({
                            "status": "FAILED_UNCOMMITTED" if uncommitted else "FAILED",
                            "run_id": context["run_id"],
                            "accepted_frames": context["accepted_frames"],
                            "written_frames": context["written_frames"],
                            "frames": context["written_frames"],
                            "dropped_frames": self._dropped_count(context),
                            "partial": True,
                            "error": self._worker_error,
                            "pending_path": str(pending_path) if uncommitted else None,
                            "performance": self._profile_snapshot(context),
                        })
                    finally:
                        with self._lock:
                            context["state"] = request["report"].get("status", "FAILED")
                            self._current = None
                            self._accepting = False
                            self._finish_request = None
                            if self._worker_error:
                                self._writer_state = "FAILED"
                            else:
                                self._writer_state = context["state"]
                        request["completion"].set()

            if self._stop_requested.is_set() and self._samples.empty():
                with self._lock:
                    if self._current is None or self._worker_error:
                        break
        with self._lock:
            if self._writer_state not in {"FAILED", "CLOSE_TIMEOUT"}:
                self._writer_state = "CLOSED"

    def _prepare_start(self, context: dict[str, Any]) -> None:
        if not self.root.exists():
            return
        info_path = self.root / "meta" / "info.json"
        identity_path = self.root / "provenance" / "dataset_identity.json"
        pending_path = self.root / "provenance" / "pending_run.json"
        if not info_path.is_file() or not identity_path.is_file():
            raise FileExistsError(
                f"existing root is not a recognized live dataset; preserve it and use a new root: {self.root}"
            )
        identity = json.loads(identity_path.read_text(encoding="utf-8"))
        repo_id = identity.get("repo_id")
        if not isinstance(repo_id, str) or not repo_id:
            raise ValueError("dataset identity sidecar has no repo_id")
        context["repo_id"] = repo_id
        run_sources = self.root / "provenance" / "run_sources.jsonl"
        if pending_path.exists():
            pending = json.loads(pending_path.read_text(encoding="utf-8"))
            if (pending.get("schema") == "piper_live_dataset_pending_v1"
                    and pending.get("stage") == "episode_saved"
                    and pending.get("run_id") == context["run_id"]):
                self._recover_saved_pending(pending, identity)
                raise ValueError(
                    "recovered the matching saved pending episode and completed its sidecars; "
                    "the duplicate run_id is now complete and was not appended again"
                )
            raise RuntimeError(
                f"dataset has pending stage {pending.get('stage')!r}; preserve it and review "
                "the pending marker. Do not delete or overwrite partial data."
            )
        if run_sources.exists():
            for line in run_sources.read_text(encoding="utf-8").splitlines():
                if line and json.loads(line).get("run_id") == context["run_id"]:
                    raise ValueError(f"run_id {context['run_id']} is already complete; duplicate refused")

    def _recover_saved_pending(self, pending: dict[str, Any], identity: dict[str, Any]) -> None:
        stage_path = (self.root / str(pending.get("source_map_stage_path", ""))).resolve()
        try:
            stage_path.relative_to((self.root / "provenance").resolve())
        except ValueError as exc:
            raise ValueError("pending source-map path escapes provenance") from exc
        if not stage_path.is_file():
            raise FileNotFoundError("saved pending episode has no staged source-map sidecar")
        rows = [json.loads(line) for line in stage_path.read_text(encoding="utf-8").splitlines() if line]
        if len(rows) != int(pending.get("saved_episode_frames", -1)):
            raise RuntimeError("saved pending episode frame count does not match staged source map")
        from gamepad_capture.tools.dataset import cli
        from .validation import assess_episode

        dataset = cli._validate_existing_dataset(
            self.root, identity["repo_id"],
            cli._feature_schema(int(pending["height"]), int(pending["width"]))
        )
        episode_index = int(pending["episode_index"])
        if episode_index >= dataset.meta.total_episodes:
            raise RuntimeError("saved pending episode is not visible in dataset metadata")
        valid_actions = [row["action"] for row in rows if row.get("action_valid") is True]
        cli._save_masked_action_stats(self.root, episode_index, valid_actions)
        label = dict(pending.get("label") or {})
        metadata = dict(pending.get("metadata") or {})
        assessment = assess_episode(
            rows, label=label, metadata=metadata,
            dropped_frames=int(pending.get("dropped_frames", 0)),
            action_semantics=str(pending.get("action_semantics", ACTION_SEMANTICS_DEFAULT)),
        )
        self._commit_sidecars(pending, rows, label, metadata, assessment, identity["repo_id"])
        (self.root / "provenance" / "pending_run.json").unlink(missing_ok=True)
        stage_path.unlink(missing_ok=True)

    def _write_sample(self, context: dict[str, Any], frame_index: int,
                      sample: dict[str, Any]) -> None:
        self._ensure_dataset(context, sample)
        while context["written_frames"] < frame_index:
            self._add_placeholder(context, context["written_frames"])
        camera_valid = list(sample["camera_valid"])
        observation_valid = bool(sample["observation_valid"] and all(camera_valid))
        action_valid = bool(sample["action_valid"] and observation_valid)
        action = sample["action"] if action_valid else np.zeros((7,), dtype=np.float32)
        frame = {
            "observation.images.base": sample["images"]["base"],
            "observation.images.wrist": sample["images"]["wrist"],
            "observation.state": sample["state"],
            "action": action,
            "action_valid": np.asarray([action_valid], dtype=np.bool_),
            "observation_valid": np.asarray([observation_valid], dtype=np.bool_),
            "camera_valid": np.asarray([all(camera_valid)], dtype=np.bool_),
            "task": context["task"],
        }
        add_frame_started_ns = time.perf_counter_ns()
        context["dataset"].add_frame(frame)
        self._record_context_latency(context, "lerobot_add_frame",
                                     time.perf_counter_ns() - add_frame_started_ns)
        stage_started_ns = time.perf_counter_ns()
        source_row = self._source_row(context, frame_index, sample, action,
                                      action_valid, observation_valid, camera_valid)
        self._stage_source_row(context, source_row)
        self._record_context_latency(context, "source_map_stage",
                                     time.perf_counter_ns() - stage_started_ns)
        context["written_frames"] += 1
        context["diagnostic_bytes"] += len(json.dumps(source_row, separators=(",", ":"),
                                                     default=_json_default).encode("utf-8"))
        if action_valid:
            context["valid_actions"].append(action.astype(np.float32).tolist())
        if (context["written_frames"] - context["last_storage_check_at"]
                >= self.storage_check_interval_frames):
            self._check_storage(context)
            context["last_storage_check_at"] = context["written_frames"]

    def _ensure_dataset(self, context: dict[str, Any], sample: dict[str, Any]) -> None:
        if context["dataset"] is not None:
            return
        prepare_started_ns = time.perf_counter_ns()
        height, width = sample["images"]["base"].shape[:2]
        context["image_shape"] = (height, width, 3)
        self._check_storage(context, rows_hint=max(1, context["attempted_frames"]))

        if self._runtime_cli is not None:
            cli = self._runtime_cli
            context["runtime"] = deepcopy(self._prewarmed_runtime or RUNTIME_VERSIONS)
        else:
            import_started_ns = time.perf_counter_ns()
            from gamepad_capture.tools.dataset import cli
            self._record_context_latency(context, "dataset_cli_import",
                                         time.perf_counter_ns() - import_started_ns)
            runtime_started_ns = time.perf_counter_ns()
            context["runtime"] = cli._check_runtime()
            self._record_context_latency(context, "dataset_runtime_check",
                                         time.perf_counter_ns() - runtime_started_ns)
            cache_started_ns = time.perf_counter_ns()
            cli._configure_offline_cache(self.cache_root)
            self._record_context_latency(context, "dataset_cache_config",
                                         time.perf_counter_ns() - cache_started_ns)
        features = cli._feature_schema(height, width)
        dataset_create_started_ns = time.perf_counter_ns()
        if self._dataset_factory is not None:
            context["dataset"] = self._dataset_factory(self, context, features)
            context["repo_id"] = context.get("repo_id") or self._repo_id_for(context["run_id"])
            context["episode_index"] = int(getattr(context["dataset"].meta, "total_episodes", 0))
            context["dataset_row_start"] = int(getattr(context["dataset"].meta, "total_frames", 0))
        elif self.root.exists():
            if not (self.root / "meta" / "info.json").is_file():
                raise FileExistsError(f"existing output root is not a LeRobot dataset: {self.root}")
            identity_path = self.root / "provenance" / "dataset_identity.json"
            if not identity_path.is_file():
                raise FileExistsError(f"existing dataset has no identity sidecar: {self.root}")
            identity = json.loads(identity_path.read_text(encoding="utf-8"))
            repo_id = str(identity["repo_id"])
            dataset = cli._validate_existing_dataset(self.root, repo_id, features)
            context["dataset"] = dataset
            context["repo_id"] = repo_id
            context["episode_index"] = int(dataset.meta.total_episodes)
            context["dataset_row_start"] = int(dataset.meta.total_frames)
        else:
            self.root.parent.mkdir(parents=True, exist_ok=True)
            if self._lerobot_dataset_type is None:
                lerobot_import_started_ns = time.perf_counter_ns()
                from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
                dataset_type = LeRobotDataset
                self._record_context_latency(context, "lerobot_dataset_import",
                                             time.perf_counter_ns() - lerobot_import_started_ns)
            else:
                dataset_type = self._lerobot_dataset_type
            repo_id = self._repo_id_for(context["run_id"])
            dataset = dataset_type.create(
                repo_id=repo_id, root=self.root, fps=self.fps, robot_type="piper",
                features=features, use_videos=True, image_writer_processes=0,
                image_writer_threads=0,
            )
            context["dataset"] = dataset
            context["repo_id"] = repo_id
            context["episode_index"] = 0
            context["dataset_row_start"] = 0
            identity = {
                "schema": "piper_live_lerobot_dataset_identity_v1",
                "repo_id": repo_id,
                "format": "LeRobot v2.1",
                "fps": self.fps,
                "created_at_utc": _utc_now(),
            }
            _json_write(self.root / "provenance" / "dataset_identity.json", identity)
        self._record_context_latency(context, "dataset_open_or_create",
                                     time.perf_counter_ns() - dataset_create_started_ns)
        pending_started_ns = time.perf_counter_ns()
        self._create_pending(context, height, width)
        self._record_context_latency(context, "pending_sidecar_create",
                                     time.perf_counter_ns() - pending_started_ns)
        self._record_context_latency(context, "first_sample_dataset_prepare",
                                     time.perf_counter_ns() - prepare_started_ns)

    @staticmethod
    def _repo_id_for(run_id: str) -> str:
        sanitized = re.sub(r"[^A-Za-z0-9_-]+", "_", run_id).strip("_")[:80] or "run"
        return f"piper/live_{sanitized}"

    def _create_pending(self, context: dict[str, Any], height: int, width: int) -> None:
        provenance = self.root / "provenance"
        provenance.mkdir(parents=True, exist_ok=True)
        pending_path = provenance / "pending_run.json"
        if pending_path.exists():
            raise FileExistsError("another pending run marker exists; preserve it for recovery review")
        stage_dir = provenance / "staging"
        stage_dir.mkdir(parents=True, exist_ok=True)
        safe_run = re.sub(r"[^A-Za-z0-9_.-]+", "_", context["run_id"])
        stage_path = stage_dir / f"{safe_run}.source_map.pending.jsonl"
        if stage_path.exists():
            raise FileExistsError(f"pending source-map staging file already exists: {stage_path}")
        context["pending_path"] = pending_path
        context["stage_map_path"] = stage_path
        context["stage_map_handle"] = stage_path.open("x", encoding="utf-8", newline="\n")
        context["height"] = int(height)
        context["width"] = int(width)
        context["action_semantics"] = str(
            context["metadata"].get("action_semantics", ACTION_SEMANTICS_DEFAULT)
        )
        pending = self._pending_record(context, "recording")
        _json_write(pending_path, pending)

    def _add_placeholder(self, context: dict[str, Any], frame_index: int) -> None:
        shape = context["image_shape"]
        if shape is None:
            raise RuntimeError("cannot preserve the timeline before a valid image shape is known")
        height, width, _ = shape
        zeros = np.zeros((height, width, 3), dtype=np.uint8)
        state = np.zeros((7,), dtype=np.float32)
        action = np.zeros((7,), dtype=np.float32)
        context["dataset"].add_frame({
            "observation.images.base": zeros,
            "observation.images.wrist": zeros,
            "observation.state": state,
            "action": action,
            "action_valid": np.asarray([False], dtype=np.bool_),
            "observation_valid": np.asarray([False], dtype=np.bool_),
            "camera_valid": np.asarray([False], dtype=np.bool_),
            "task": context["task"],
        })
        source_row = {
            "schema": "piper_live_source_map_v1",
            "run_id": context["run_id"],
            "episode_index": context["episode_index"],
            "episode_frame_index": frame_index,
            "dataset_row_index": int(context["dataset_row_start"]) + frame_index,
            "dataset_timestamp_s": float(frame_index) / self.fps,
            "task": context["task"],
            "state": state.tolist(),
            "action": action.tolist(),
            "action_valid": False,
            "observation_valid": False,
            "camera_valid": [False, False],
            "dataset_camera_valid": False,
            "source_map": None,
            "writer_placeholder": True,
            "drop_reason": self._drop_reason_for(context, frame_index),
        }
        self._stage_source_row(context, source_row)
        context["written_frames"] += 1
        context["placeholder_frames"] += 1
        context["diagnostic_bytes"] += len(json.dumps(source_row, separators=(",", ":")).encode("utf-8"))

    @staticmethod
    def _drop_reason_for(context: dict[str, Any], frame_index: int) -> str:
        for item in context["dropped_ranges"]:
            if item["start_frame_index"] <= frame_index <= item["end_frame_index"]:
                return str(item["reason"])
        return "timeline_gap_without_admitted_sample"

    def _source_row(self, context: dict[str, Any], frame_index: int,
                    sample: dict[str, Any], action: np.ndarray, action_valid: bool,
                    observation_valid: bool, camera_valid: list[bool]) -> dict[str, Any]:
        return {
            "schema": "piper_live_source_map_v1",
            "run_id": context["run_id"],
            "episode_index": context["episode_index"],
            "episode_frame_index": frame_index,
            "dataset_row_index": int(context["dataset_row_start"]) + frame_index,
            "dataset_timestamp_s": float(frame_index) / self.fps,
            "task": context["task"],
            "state": sample["state"].astype(np.float32).tolist(),
            "action": action.astype(np.float32).tolist(),
            "action_valid": bool(action_valid),
            "observation_valid": bool(observation_valid),
            "camera_valid": list(camera_valid),
            "dataset_camera_valid": bool(all(camera_valid)),
            "action_semantics": context.get("action_semantics", ACTION_SEMANTICS_DEFAULT),
            "source_map": sample["source_map"],
            "writer_placeholder": False,
        }

    def _stage_source_row(self, context: dict[str, Any], row: dict[str, Any]) -> None:
        handle = context.get("stage_map_handle")
        if handle is None:
            raise RuntimeError("source-map staging is not open")
        handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"),
                               allow_nan=False, default=_json_default))
        handle.write("\n")
        handle.flush()

    def _check_storage(self, context: dict[str, Any], *, rows_hint: int | None = None) -> None:
        shape = context.get("image_shape")
        if shape is None:
            shape = (context["dataset"].meta.features["observation.images.base"]["shape"][1],
                     context["dataset"].meta.features["observation.images.base"]["shape"][2], 3)
        height, width, _ = shape
        projected = max(
            int(rows_hint or 0),
            int(context["metadata"].get("estimated_frames", 0) or 0),
            int(context["attempted_frames"] + self._samples.qsize()),
        )
        projected = max(projected, 1)
        diagnostic_bytes = max(
            2048,
            int(context["diagnostic_bytes"] / max(1, context["written_frames"]))
            if context["written_frames"] else 2048,
        )
        estimate = estimate_storage(
            projected, height, width, queue_capacity=self.queue_capacity,
            diagnostic_bytes_per_frame=diagnostic_bytes, reserve_bytes=self.reserve_bytes,
        )
        parent = self.root.parent
        parent.mkdir(parents=True, exist_ok=True)
        free = int(self._disk_usage(parent).free)
        self._last_storage_estimate = estimate
        self._last_free_bytes = free
        if free < estimate["required_free_bytes"]:
            raise OSError(
                "insufficient free space for the current live dataset forecast: "
                f"free={free}, required={estimate['required_free_bytes']}, "
                f"frames={projected}, image_shape={(height, width, 3)}"
            )

    def _finish(self, context: dict[str, Any], request: dict[str, Any]) -> dict[str, Any]:
        finish_started_ns = time.perf_counter_ns()
        context["finish_started_monotonic_ns"] = finish_started_ns
        mode = request["mode"]
        label = self._normalize_label(request["label"], mode, request.get("reason"))
        label["writer_closed_at_utc"] = _utc_now()
        events = list(context["events"]) + list(request.get("events") or [])
        if request.get("reason"):
            events.append({"event": "episode_abort", "reason": request["reason"],
                           "time_utc": _utc_now()})
        if self._worker_error:
            handle = context.get("stage_map_handle")
            if handle is not None:
                try:
                    handle.flush()
                    handle.close()
                except OSError:
                    pass
                context["stage_map_handle"] = None
            pending_path = context.get("pending_path")
            if pending_path is not None and pending_path.exists():
                try:
                    pending = self._pending_record(context, "failed_uncommitted")
                    pending["worker_error"] = self._worker_error
                    _json_write(pending_path, pending)
                except OSError:
                    pass
            context["finish_completed_monotonic_ns"] = time.perf_counter_ns()
            self._record_context_latency(context, "finish_total",
                                         context["finish_completed_monotonic_ns"] - finish_started_ns)
            return {"status": "FAILED_UNCOMMITTED", "run_id": context["run_id"],
                    "accepted_frames": context["accepted_frames"],
                    "written_frames": context["written_frames"],
                    "dropped_frames": self._dropped_count(context),
                    "partial": True, "error": self._worker_error,
                    "pending_path": str(pending_path) if pending_path else None,
                    "performance": self._profile_snapshot(context)}
        if context["dataset"] is None or context["written_frames"] == 0:
            context["finish_completed_monotonic_ns"] = time.perf_counter_ns()
            self._record_context_latency(context, "finish_total",
                                         context["finish_completed_monotonic_ns"] - finish_started_ns)
            return {"status": "ABORTED_NO_DATA" if mode == "abort" else "FAILED_NO_DATA",
                    "run_id": context["run_id"], "accepted_frames": context["accepted_frames"],
                    "dropped_frames": self._dropped_count(context),
                    "performance": self._profile_snapshot(context),
                    "partial": True, "reason": "no frame reached the LeRobot worker"}

        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="before_finalize")

        while context["written_frames"] < context["attempted_frames"]:
            self._add_placeholder(context, context["written_frames"])
        self._check_storage(context, rows_hint=context["written_frames"])
        handle = context.get("stage_map_handle")
        if handle is not None:
            flush_started_ns = time.perf_counter_ns()
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
            context["stage_map_handle"] = None
            self._record_context_latency(context, "source_map_flush_fsync_close",
                                         time.perf_counter_ns() - flush_started_ns)
        context["events"] = events
        context["label"] = label
        pending = self._pending_record(context, "committing")
        _json_write(context["pending_path"], pending)

        dataset = context["dataset"]
        image_writer = getattr(dataset, "image_writer", None)
        if image_writer is not None:
            image_flush_started_ns = time.perf_counter_ns()
            image_writer.wait_until_done()
            self._record_context_latency(context, "image_writer_flush",
                                         time.perf_counter_ns() - image_flush_started_ns)
        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="image_writer_flush",
                                              dataset_saved=False)
        original_encode = getattr(dataset, "encode_episode_videos", None)
        if callable(original_encode):
            def timed_encode(episode_index: int, *args: Any, **kwargs: Any) -> Any:
                encode_started_ns = time.perf_counter_ns()
                self._notify_progress(request, "video_encoding_started")
                try:
                    return original_encode(episode_index, *args, **kwargs)
                finally:
                    self._record_context_latency(context, "video_encode",
                                                 time.perf_counter_ns() - encode_started_ns)
                    self._notify_progress(request, "video_encoding_complete")
            dataset.encode_episode_videos = timed_encode
        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="before_episode_save",
                                              dataset_saved=False)
        self._notify_progress(request, "episode_save_started")
        save_started_ns = time.perf_counter_ns()
        context["dataset"].save_episode()
        self._record_context_latency(context, "save_episode_total",
                                     time.perf_counter_ns() - save_started_ns)
        context["dataset"].encode_episode_videos = original_encode
        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="episode_save",
                                              dataset_saved=True)
        self._notify_progress(request, "dataset_saved")
        pending = self._pending_record(context, "episode_saved")
        pending["saved_episode_frames"] = context["written_frames"]
        _json_write(context["pending_path"], pending)
        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="pending_update",
                                              dataset_saved=True)
        valid_actions = context["valid_actions"]
        from gamepad_capture.tools.dataset import cli
        stats_started_ns = time.perf_counter_ns()
        cli._save_masked_action_stats(self.root, int(context["episode_index"]), valid_actions)
        self._record_context_latency(context, "action_stats_write",
                                     time.perf_counter_ns() - stats_started_ns)

        stage_read_started_ns = time.perf_counter_ns()
        stage_rows = [json.loads(line) for line in
                      context["stage_map_path"].read_text(encoding="utf-8").splitlines() if line]
        self._record_context_latency(context, "source_map_readback",
                                     time.perf_counter_ns() - stage_read_started_ns)
        assessment_started_ns = time.perf_counter_ns()
        assessment = assess_episode(
            stage_rows, label=label, metadata=context["metadata"],
            dropped_frames=self._dropped_count(context),
            action_semantics=context.get("action_semantics", ACTION_SEMANTICS_DEFAULT),
        )
        self._record_context_latency(context, "episode_assessment",
                                     time.perf_counter_ns() - assessment_started_ns)
        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="episode_assessment",
                                              dataset_saved=True)
        sidecar_started_ns = time.perf_counter_ns()
        self._commit_sidecars(pending, stage_rows, label, context["metadata"], assessment,
                              context["repo_id"])
        if context.get("finish_timed_out"):
            return self._finish_timeout_report(context, request, phase="sidecar_commit",
                                              dataset_saved=True)
        context["pending_path"].unlink(missing_ok=True)
        context["stage_map_path"].unlink(missing_ok=True)
        self._record_context_latency(context, "sidecar_commit_and_cleanup",
                                     time.perf_counter_ns() - sidecar_started_ns)
        context["finish_completed_monotonic_ns"] = time.perf_counter_ns()
        self._record_context_latency(context, "finish_total",
                                     context["finish_completed_monotonic_ns"] - finish_started_ns)
        try:
            _jsonl_write(self.root / 'provenance' / 'timing_reports.jsonl', [{
                'schema': 'piper_episode_timing_v1', 'run_id': context['run_id'],
                'episode_index': context['episode_index'], 'frames': context['written_frames'],
                'initialization_timings': self._global_timing_snapshot(),
                'performance': self._profile_snapshot(context),
            }], append=True)
        except OSError as exc:
            import warnings
            warnings.warn(f"Episode committed; optional timing report failed: {exc}", RuntimeWarning)
        return {
            "status": "ABORTED_PARTIAL" if mode == "abort" else "SAVED_REVIEW_ONLY",
            "run_id": context["run_id"],
            "episode_index": context["episode_index"],
            "frames": context["written_frames"],
            "accepted_frames": context["accepted_frames"],
            "dropped_frames": self._dropped_count(context),
            "placeholder_frames": context["placeholder_frames"],
            "training_eligible": assessment["training_eligible"],
            "training_blockers": assessment["training_blockers"],
            "partial": (mode == "abort" or self._dropped_count(context) > 0
                        or context["placeholder_frames"] > 0
                        or len(context["valid_actions"]) < context["written_frames"]),
            "repo_id": context["repo_id"],
            "root": str(self.root),
            "readback_status": "NOT_RUN_BY_LIVE_WRITER",
            "performance": self._profile_snapshot(context),
        }

    @staticmethod
    def _notify_progress(request: dict[str, Any], phase: str) -> None:
        callback = request.get("progress_callback")
        if callback is None:
            return
        try:
            callback(phase)
        except Exception:
            # Progress output must never interrupt dataset finalization.
            pass

    def _finish_timeout_report(self, context: dict[str, Any], request: dict[str, Any], *,
                               phase: str, dataset_saved: bool = False) -> dict[str, Any]:
        pending_path = context.get("pending_path")
        timeout_s = float(context.get("finish_timeout_s", self.finish_timeout_s))
        if pending_path is not None:
            pending = self._pending_record(context, "failed_uncommitted")
            pending.update({
                "finish_timeout_s": timeout_s,
                "finish_timeout_phase": phase,
                "dataset_episode_saved_before_timeout": bool(dataset_saved),
            })
            _json_write(pending_path, pending)
        context["finish_completed_monotonic_ns"] = time.perf_counter_ns()
        finish_started_ns = int(context.get("finish_started_monotonic_ns")
                                or context["finish_completed_monotonic_ns"])
        self._record_context_latency(context, "finish_total",
                                     context["finish_completed_monotonic_ns"] - finish_started_ns)
        self._notify_progress(request, "finish_timed_out")
        return {
            "status": "FAILED_UNCOMMITTED",
            "run_id": context["run_id"],
            "accepted_frames": context["accepted_frames"],
            "written_frames": context["written_frames"],
            "dropped_frames": self._dropped_count(context),
            "partial": True,
            "error": f"finish timeout after {timeout_s:g}s at {phase}; pending data preserved",
            "pending_path": str(pending_path) if pending_path is not None else None,
            "performance": self._profile_snapshot(context),
        }

    @staticmethod
    def _normalize_label(label: str | dict[str, Any] | None, mode: str,
                         reason: str | None) -> dict[str, Any]:
        if isinstance(label, dict):
            result = deepcopy(label)
        else:
            result = {"demonstration_label": str(label or "UNREVIEWED")}
        result.setdefault("outcome", "unknown")
        result.setdefault("successful_complete_end", False)
        result.setdefault("complete_closeout_label_present", False)
        result.setdefault("robot_execution_verified", False)
        if mode == "abort":
            result.setdefault("demonstration_label", "aborted-demonstration")
            result["training_eligible"] = False
            result["abort_reason"] = reason
        return result

    def _pending_record(self, context: dict[str, Any], stage: str) -> dict[str, Any]:
        relative_stage = (context["stage_map_path"].relative_to(self.root).as_posix()
                          if context.get("stage_map_path") else None)
        return {
            "schema": "piper_live_dataset_pending_v1",
            "run_id": context["run_id"],
            "task": context["task"],
            "episode_index": context["episode_index"],
            "dataset_row_start": context["dataset_row_start"],
            "source_map_stage_path": relative_stage,
            "stage": stage,
            "started_at_utc": context["started_at_utc"],
            "updated_at_utc": _utc_now(),
            "fps": self.fps,
            "height": context.get("height"),
            "width": context.get("width"),
            "attempted_frames": context["attempted_frames"],
            "accepted_frames": context["accepted_frames"],
            "written_frames": context["written_frames"],
            "dropped_frames": self._dropped_count(context),
            "dropped_ranges": context["dropped_ranges"],
            "metadata": context["metadata"],
            "action_semantics": context.get("action_semantics", ACTION_SEMANTICS_DEFAULT),
            "label": context.get("label"),
            "events": context.get("events", []),
            "runtime": context.get("runtime"),
            "repo_id": context.get("repo_id"),
        }

    def _commit_sidecars(self, pending: dict[str, Any], rows: list[dict[str, Any]],
                         label: dict[str, Any], metadata: dict[str, Any],
                         assessment: dict[str, Any], repo_id: str) -> None:
        provenance = self.root / "provenance"
        run_id = str(pending["run_id"])
        episode_index = int(pending["episode_index"])
        runtime = pending.get("runtime") or RUNTIME_VERSIONS
        label_row = {
            "schema": "piper_live_demo_outcome_label_v1",
            "run_id": run_id,
            "episode_index": episode_index,
            **label,
            "training_eligible": assessment["training_eligible"],
            "acceptance": assessment,
        }
        source_row = {
            "schema": "piper_live_lerobot_run_source_v1",
            "run_id": run_id,
            "episode_index": episode_index,
            "task": pending["task"],
            "fps": self.fps,
            "dataset_repo_id": repo_id,
            "format": "LeRobot v2.1",
            "current_training_action_semantics": EXPECTED_PROXY_SEMANTICS,
            "runtime": runtime,
            "metadata": metadata,
            "action_semantics": pending.get("action_semantics", ACTION_SEMANTICS_DEFAULT),
            "action_units": metadata.get("action_units", "nominal_unverified"),
            "state_units": metadata.get("state_units", "nominal_unverified"),
            "camera_exposure_synchronization_verified": False,
            "frames": len(rows),
            "accepted_frames": int(pending["accepted_frames"]),
            "dropped_frames": int(pending["dropped_frames"]),
            "placeholder_frames": sum(bool(row.get("writer_placeholder")) for row in rows),
            "source_identity": metadata.get("source_identity"),
            "lifecycle_events": pending.get("events", []),
            "formal_training_eligible": assessment["training_eligible"],
            "training_blockers": assessment["training_blockers"],
        }

        from gamepad_capture.tools.dataset import cli
        cli._jsonl_upsert(
            provenance / "source_map.jsonl", rows,
            ("run_id", "episode_index", "episode_frame_index"),
        )
        cli._jsonl_upsert(
            provenance / "run_sources.jsonl", [source_row], ("run_id", "episode_index"),
        )
        cli._jsonl_upsert(
            provenance / "run_labels.jsonl", [label_row], ("run_id", "episode_index"),
        )
        cli._jsonl_upsert(
            provenance / "quality_reports.jsonl",
            [{"run_id": run_id, "episode_index": episode_index, **assessment}],
            ("run_id", "episode_index"),
        )
        file_slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", run_id)[:96] or "run"
        file_hash = hashlib.sha256(run_id.encode("utf-8")).hexdigest()[:10]
        _json_write(provenance / f"{file_slug}_{file_hash}_acceptance.json", assessment)
        event_rows = [
            {"event": "episode_saved", "time_utc": _utc_now(),
             "training_eligible": assessment["training_eligible"]},
            *pending.get("events", []),
        ]
        for event_index, event in enumerate(event_rows):
            if not isinstance(event, Mapping):
                event = {"event": "unstructured_event", "value": repr(event)}
            event_rows[event_index] = {
                **event,
                "run_id": run_id,
                "episode_index": episode_index,
                "event_index": event_index,
            }
        cli._jsonl_upsert(
            provenance / "lifecycle_events.jsonl", event_rows,
            ("run_id", "episode_index", "event_index"),
        )

    def _fail_worker(self, exc: Exception, *, schedule_finalize: bool = True) -> None:
        error = f"{type(exc).__name__}: {exc}"
        with self._lock:
            if self._worker_error is None:
                self._worker_error = error
            self._writer_state = "FAILED"
            self._accepting = False
            context = self._current
            if context is not None and schedule_finalize:
                context["state"] = "FAILED"
                if self._finish_request is None:
                    self._finish_request = {
                        "mode": "abort", "label": None,
                        "reason": "writer_failure", "events": [],
                        "completion": threading.Event(), "report": {},
                    }
            elif context is not None:
                handle = context.get("stage_map_handle")
                if handle is not None:
                    try:
                        handle.flush()
                        handle.close()
                    except OSError:
                        pass
                    context["stage_map_handle"] = None
                pending_path = context.get("pending_path")
                if pending_path is not None and pending_path.exists():
                    try:
                        pending = json.loads(pending_path.read_text(encoding="utf-8"))
                        if pending.get("stage") in {"recording", "committing"}:
                            pending["stage"] = "failed_uncommitted"
                            pending["updated_at_utc"] = _utc_now()
                            pending["worker_error"] = self._worker_error
                            _json_write(pending_path, pending)
                    except (OSError, ValueError, json.JSONDecodeError):
                        pass
        while True:
            try:
                item = self._samples.get_nowait()
            except queue.Empty:
                break
            else:
                if context is not None:
                    self._record_drop(context, item[0], "writer_failure")
                self._samples.task_done()

    @staticmethod
    def _dropped_count(context: dict[str, Any]) -> int:
        return sum(int(item["frames"]) for item in context["dropped_ranges"])
