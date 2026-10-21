"""Unified CLI for live LeRobot recording and read-only rosbag replay.

The live adapter is a WSL child process that subscribes to existing ROS topics
and publishes only the existing recording-lifecycle String. The Windows
process owns the pinned LeRobot writer and episode commands.
"""

from __future__ import annotations

import argparse
from collections import deque
import importlib.metadata
import json
import math
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import struct
import subprocess
import sys
import threading
import time
from typing import Any, Mapping
from .home_before_start import HomeBeforeStart

from .lifecycle import (
    CaptureCoordinator,
    CaptureError,
    DEFAULT_FINISH_TIMEOUT_S,
    MAX_FINISH_TIMEOUT_S,
)


MAX_MESSAGE_BYTES = 64 * 1024 * 1024
MSG_PACK_VERSION = "1.1.2"
LEROBOT_VERSION = "0.1.0"
LEROBOT_COMMIT = "0cf864870cf29f4738d3ade893e6fd13fbd7cdb5"
DATASETS_VERSION = "3.6.0"
RERUN_VERSION = "0.26.2"
DATASET_PYTHON_VERSION = "3.10.20"
DEFAULT_REPLAY_FINISH_TIMEOUT_S = 90.0
DEFAULT_TASK = "Pick up the black rod and place it on the green desk mat."
DEFAULT_OUTPUT_RELATIVE = Path("gamepad_capture/data/lerobot")


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _pace_replay_deadline(
    source_anchor_ns: int,
    host_anchor_ns: int,
    source_time_ns: int,
    rate: float,
    *,
    now_ns=time.perf_counter_ns,
    sleep_fn=time.sleep,
) -> tuple[float, int]:
    """Wait for an absolute replay deadline and return sleep seconds and lateness ns."""
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("absolute replay pacing requires a finite positive rate")
    deadline_ns = host_anchor_ns + int((source_time_ns - source_anchor_ns) / rate)
    before_ns = now_ns()
    remaining_ns = deadline_ns - before_ns
    slept_s = 0.0
    if remaining_ns > 0:
        slept_s = remaining_ns / 1e9
        sleep_fn(slept_s)
    after_ns = now_ns()
    return slept_s, max(0, after_ns - deadline_ns)


def _msgpack_path(root: Path) -> Path:
    return root / "cache" / "live_lerobot_deps"


def add_msgpack_dependency(root: Path) -> Any:
    """Load the approved project-local stdlib MessagePack dependency."""
    path = _msgpack_path(root)
    if not path.is_dir():
        raise RuntimeError(f"live stdio dependency path is missing: {path}")
    value = str(path)
    if value not in sys.path:
        sys.path.insert(0, value)
    try:
        import msgpack
    except ImportError as exc:
        raise RuntimeError(
            f"msgpack is unavailable under {path}; install the approved wheel there before live capture"
        ) from exc
    version = getattr(msgpack, "__version__", "unknown")
    if version != MSG_PACK_VERSION:
        raise RuntimeError(f"stdio msgpack version {version} does not match {MSG_PACK_VERSION}")
    return msgpack


def check_lerobot_runtime(root: Path, *, require_msgpack: bool) -> dict[str, Any]:
    """Check the existing locked Windows environment without installing packages."""
    from importlib.metadata import distribution, version

    actual = {
        "python_version": ".".join(map(str, sys.version_info[:3])),
        "python_executable": sys.executable,
        "lerobot_version": version("lerobot"),
        "datasets_version": version("datasets"),
        "rerun_sdk_version": version("rerun-sdk"),
    }
    expected = {
        "lerobot_version": LEROBOT_VERSION,
        "datasets_version": DATASETS_VERSION,
        "rerun_sdk_version": RERUN_VERSION,
    }
    mismatches = {key: (actual[key], value) for key, value in expected.items()
                  if actual[key] != value}
    direct_url = distribution("lerobot").read_text("direct_url.json")
    if not direct_url:
        raise RuntimeError("LeRobot installation has no direct_url provenance")
    try:
        actual["lerobot_revision"] = json.loads(direct_url).get("vcs_info", {}).get("commit_id")
    except (json.JSONDecodeError, AttributeError):
        actual["lerobot_revision"] = None
    if actual["lerobot_revision"] != LEROBOT_COMMIT:
        mismatches["lerobot_revision"] = (actual["lerobot_revision"], LEROBOT_COMMIT)
    if mismatches:
        raise RuntimeError(f"pinned LeRobot runtime mismatch: {mismatches}")
    import numpy as np
    actual["numpy_version"] = np.__version__
    if require_msgpack:
        msgpack = add_msgpack_dependency(root)
        actual["msgpack_version"] = msgpack.__version__
        actual["msgpack_path"] = str(_msgpack_path(root))
    return actual


class FramedStdioAdapter:
    """Bounded MessagePack client for the read-only WSL adapter subprocess."""

    def __init__(self, command: list[str], *, msgpack: Any,
                 emit_stderr=print, sample_queue_capacity: int = 24):
        self._msgpack = msgpack
        self._process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, bufsize=0,
        )
        if self._process.stdin is None or self._process.stdout is None or self._process.stderr is None:
            raise RuntimeError("could not create stdio pipes for the WSL ROS adapter")
        self._stdout = self._process.stdout
        self._stdin = self._process.stdin
        self._stderr = self._process.stderr
        self._messages: deque[dict[str, Any]] = deque()
        self._condition = threading.Condition()
        self._queued_samples = 0
        self._sample_queue_capacity = int(sample_queue_capacity)
        self._control_queue_capacity = 1024
        self._write_lock = threading.Lock()
        self._drop_lock = threading.Lock()
        self._closed = False
        self._fatal: str | None = None
        self.transport_dropped_samples = 0
        self.transport_coalesced_diagnostics = 0
        self._reader = threading.Thread(target=self._read_stdout, name="live-ros-stdio-reader", daemon=True)
        self._errors = threading.Thread(target=self._read_stderr, args=(self._stderr, emit_stderr),
                                        name="live-ros-stderr-reader", daemon=True)
        self._reader.start()
        self._errors.start()

    @staticmethod
    def _diagnostic_key(envelope):
        if envelope.get('type') == 'event':
            detail = envelope.get('payload', {})
            if not isinstance(detail, dict):
                return None
            if (envelope.get('event') == 'INPUT_STATUS'
                    and detail.get('event') == 'input_frame'
                    and detail.get('connected') is True):
                return ('input_frame', envelope.get('run_id'))
            if (envelope.get('event') == 'OPERATOR_EVENT'
                    and detail.get('status') in ('TARGET_PREVIEW', 'TARGET_RATE_PREVIEW')):
                return (detail['status'], envelope.get('run_id'))
        if (envelope.get('type') == 'status' and envelope.get('status') == 'PAIRING_STATUS'
                and not envelope.get('pairing', {}).get('episode_tail_complete')):
            return ('pairing_status', envelope.get('run_id'))
        return None

    @staticmethod
    def _read_exact(stream, size: int) -> bytes | None:
        chunks = bytearray()
        while len(chunks) < size:
            part = stream.read(size - len(chunks))
            if not part:
                return None if not chunks else bytes(chunks)
            chunks.extend(part)
        return bytes(chunks)

    def _read_stdout(self) -> None:
        try:
            while not self._closed:
                header = self._read_exact(self._stdout, 4)
                if header is None:
                    break
                if len(header) != 4:
                    raise RuntimeError("truncated MessagePack frame length")
                size = struct.unpack(">I", header)[0]
                if size <= 0 or size > MAX_MESSAGE_BYTES:
                    raise RuntimeError(f"MessagePack frame length {size} exceeds 64 MiB bounds")
                body = self._read_exact(self._stdout, size)
                if body is None or len(body) != size:
                    raise RuntimeError("truncated MessagePack frame body")
                envelope = self._msgpack.unpackb(body, raw=False, strict_map_key=False)
                if not isinstance(envelope, dict):
                    raise RuntimeError("adapter MessagePack envelope is not a map")
                is_sample = envelope.get("type") == "sample"
                with self._condition:
                    # Encoding/save can block the consumer for many seconds.
                    # Latest-value diagnostics must not fill the lifecycle queue.
                    # RB edges, disconnects, lifecycle and error messages remain
                    # lossless and ordered; sample overflow stays explicit.
                    key = self._diagnostic_key(envelope)
                    if key is not None:
                        for queued in self._messages:
                            if self._diagnostic_key(queued) == key:
                                self._messages.remove(queued)
                                self.transport_coalesced_diagnostics += 1
                                break
                    if is_sample and self._queued_samples >= self._sample_queue_capacity:
                        with self._drop_lock:
                            self.transport_dropped_samples += 1
                        continue
                    queued_controls = len(self._messages) - self._queued_samples
                    if not is_sample and queued_controls >= self._control_queue_capacity:
                        raise RuntimeError("adapter control-message buffer exceeded its bound")
                    self._messages.append(envelope)
                    if is_sample:
                        self._queued_samples += 1
                    self._condition.notify()
        except Exception as exc:
            with self._condition:
                self._fatal = f"stdio protocol failed: {type(exc).__name__}: {exc}"
        finally:
            with self._condition:
                if not self._closed and self._fatal is None:
                    code = self._process.poll()
                    self._fatal = f"WSL ROS adapter stdout closed (exit={code})"
                fatal = self._fatal or "WSL ROS adapter closed"
                self._messages.append({"type": "status", "status": "ERROR", "error": fatal})
                self._condition.notify_all()

    @staticmethod
    def _read_stderr(stream, emit) -> None:
        for raw_line in iter(stream.readline, b""):
            try:
                line = raw_line.decode("utf-8", errors="replace").rstrip()
            except Exception:
                line = "<unreadable WSL stderr line>"
            if line:
                emit(f"WSL: {line}")

    def send_command(self, command: str, *, run_id: str, task: str = "",
                     reason: str = "", tail_ns: int | None = None) -> None:
        if self._closed:
            raise RuntimeError("WSL ROS adapter is closed")
        value: dict[str, Any] = {"type": "command", "command": command,
                                 "run_id": run_id}
        if task:
            value["task"] = task
        if reason:
            value["reason"] = reason
        if tail_ns is not None:
            value["tail_ns"] = int(tail_ns)
        payload = self._msgpack.packb(value, use_bin_type=True)
        if len(payload) > MAX_MESSAGE_BYTES:
            raise RuntimeError("outbound MessagePack command exceeds 64 MiB")
        with self._write_lock:
            self._stdin.write(struct.pack(">I", len(payload)))
            self._stdin.write(payload)
            self._stdin.flush()

    def receive(self, timeout_s: float = 0.1) -> dict[str, Any] | None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            while not self._messages:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            message = self._messages.popleft()
            if message.get("type") == "sample":
                self._queued_samples -= 1
            return message

    def take_transport_dropped_samples(self) -> int:
        with self._drop_lock:
            count = self.transport_dropped_samples
            self.transport_dropped_samples = 0
            return count

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._stdin.close()
        except OSError:
            pass
        close_error: Exception | None = None
        try:
            self._process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                # This is the exact child created by this adapter instance.
                self._process.terminate()
            except subprocess.TimeoutExpired:
                pass
            except OSError as exc:
                close_error = exc
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    self._process.kill()
                    self._process.wait(timeout=1)
                except subprocess.TimeoutExpired as exc:
                    close_error = exc
                except OSError as exc:
                    close_error = exc
        finally:
            for stream in (self._stdout, self._stderr):
                try:
                    stream.close()
                except OSError:
                    pass
            self._reader.join(timeout=0.5)
            self._errors.join(timeout=0.5)
        if close_error is not None:
            raise RuntimeError(f"could not close recorder-owned WSL adapter child: {close_error}")


def _close_live_session(coordinator: CaptureCoordinator, return_code: int
                        ) -> tuple[dict[str, Any], int]:
    try:
        result = coordinator.close()
    except Exception as exc:
        result = {"status": "CLOSE_FAILED", "error": f"{type(exc).__name__}: {exc}"}
    if result.get("status") != "CLOSED":
        print(f"RECORDER_CLOSE: status={result.get('status')}; error={result.get('error')}",
              file=sys.stderr, flush=True)
        return result, return_code or 3
    return result, return_code


def _to_wsl_path(path: Path) -> str:
    resolved = str(path.resolve())
    match = re.fullmatch(r"([A-Za-z]):\\(.*)", resolved)
    if not match:
        raise ValueError(f"cannot map Windows project path to WSL: {resolved}")
    tail = match.group(2).replace("\\", "/")
    return f"/mnt/{match.group(1).lower()}/{tail}"


def build_wsl_adapter_command(root: Path, *, distro: str,
                              sensor_source: str = "windows",
                              domain_id: int | None = None,
                              site_confirmed: bool = False) -> list[str]:
    if sensor_source not in ("windows", "native"):
        raise ValueError("sensor source must be windows or native")
    wsl_root = _to_wsl_path(root)
    env_source = shlex.quote(f"{wsl_root}/gamepad_capture/scripts/gamepad_environment.sh")
    setup = [f"export PI05_GAMEPAD_SENSOR_SOURCE={shlex.quote(sensor_source)}",
             f"source {env_source} >&2"]
    setup.append("export RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
    if domain_id is not None:
        setup.append(f"export ROS_DOMAIN_ID={int(domain_id)}")
    setup.append("export ROS_LOCALHOST_ONLY=1")
    if site_confirmed:
        setup.append("export PI05_LIVE_SITE_CONFIRMED=1")
    args = ["--mode", "live", "--stdio"]
    if site_confirmed:
        args.append("--site-domain-confirmed")
    command = " ".join(shlex.quote(arg) for arg in args)
    setup.append(f"exec /usr/bin/python3 -m gamepad_capture.live_dataset.ros_adapter {command}")
    shell = " && ".join(setup)
    return ["wsl.exe", "-d", distro, "--cd", wsl_root, "--exec", "/bin/bash", "-lc", shell]


def _default_output(root: Path) -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return (root / DEFAULT_OUTPUT_RELATIVE / f"live_{stamp}_{os.getpid()}").resolve()


def _capture_preflight_parameter_errors(args: argparse.Namespace) -> list[str]:
    errors: list[str] = []
    if not isinstance(args.operator, str) or not args.operator.strip():
        errors.append("operator must be a non-empty string")
    if not isinstance(args.task, str) or not args.task.strip():
        errors.append("task must be a non-empty string")
    if type(args.queue_capacity) is not int or args.queue_capacity < 1:
        errors.append("queue_capacity must be a positive integer")
    for name, value in (("duration", args.duration),
                        ("startup_timeout", args.startup_timeout),
                        ("lifecycle_timeout", args.lifecycle_timeout)):
        if not math.isfinite(float(value)) or float(value) <= 0:
            errors.append(f"{name} must be finite and positive")
    if (not math.isfinite(float(args.lifecycle_tail))
            or not 0 <= float(args.lifecycle_tail) <= 10):
        errors.append("lifecycle_tail must be finite and in [0, 10] seconds")
    if args.continuous_episodes and not args.manual_lifecycle:
        errors.append("--continuous-episodes requires --manual-lifecycle")
    if args.max_capture_duration is not None:
        if not math.isfinite(float(args.max_capture_duration)) or args.max_capture_duration <= 0:
            errors.append("max_capture_duration must be finite and positive when set")
        if not args.manual_lifecycle:
            errors.append("--max-capture-duration requires --manual-lifecycle")
    try:
        _effective_finish_timeout(args.finish_timeout, live=True)
    except ValueError as exc:
        errors.append(str(exc))
    if args.domain_id not in (None, 231, 222):
        errors.append("live ROS domain must be 231 or 222")
    elif args.domain_id == 222 and not args.site_confirmed:
        errors.append("domain 222 requires --site-confirmed")
    return errors


def _inspect_path(path: Path, *, must_not_exist: bool) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    exists = resolved.exists()
    report: dict[str, Any] = {
        "requested_path": str(resolved),
        "exists": exists,
        "must_not_exist": must_not_exist,
        "writable_parent": False,
        "free_bytes": None,
    }
    if exists and must_not_exist:
        report["error"] = "destination already exists; choose a new output path"
        return report
    if exists and not resolved.is_dir():
        report["error"] = "path exists but is not a directory"
        return report
    candidate = resolved if exists else resolved.parent
    while not candidate.exists() and candidate != candidate.parent:
        candidate = candidate.parent
    if not candidate.is_dir():
        report["error"] = f"nearest existing parent is not a directory: {candidate}"
        return report
    report["nearest_existing_parent"] = str(candidate)
    report["writable_parent"] = os.access(candidate, os.W_OK)
    try:
        report["free_bytes"] = int(shutil.disk_usage(candidate).free)
    except OSError as exc:
        report["error"] = f"could not inspect free space: {exc}"
    if not report["writable_parent"]:
        report["error"] = f"nearest existing parent is not writable: {candidate}"
    return report


def _check_local_encoder_imports() -> dict[str, Any]:
    """Import the pinned local encoder without creating a dataset or opening devices."""
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    import av

    codec = av.codec.Codec("libsvtav1", "w")
    return {
        "lerobot_dataset_class_imported": LeRobotDataset.__name__ == "LeRobotDataset",
        "video_encoder": codec.name,
        "video_encoder_available": True,
        "encoder_invoked": False,
    }


def _check_offline_environment() -> tuple[dict[str, Any], list[str]]:
    values = {
        "HF_HOME": os.environ.get("HF_HOME"),
        "HF_DATASETS_CACHE": os.environ.get("HF_DATASETS_CACHE"),
        "HF_HUB_OFFLINE": os.environ.get("HF_HUB_OFFLINE"),
        "HF_DATASETS_OFFLINE": os.environ.get("HF_DATASETS_OFFLINE"),
        "TEMP": os.environ.get("TEMP"),
        "TMP": os.environ.get("TMP"),
    }
    errors: list[str] = []
    for key in ("HF_HOME", "HF_DATASETS_CACHE", "TEMP", "TMP"):
        value = values[key]
        if not value:
            errors.append(f"{key} must point to a task-isolated local directory")
            continue
        env_path = Path(value).expanduser().resolve()
        path_report = _inspect_path(env_path, must_not_exist=False)
        if not env_path.is_dir():
            path_report["error"] = "directory does not exist; create it before Python starts"
        values[f"{key}_path_ready"] = env_path.is_dir() and not bool(path_report.get("error"))
        values[f"{key}_path"] = str(env_path)
        if path_report.get("error"):
            errors.append(f"{key}: {path_report['error']}")
    for key in ("HF_HUB_OFFLINE", "HF_DATASETS_OFFLINE"):
        if str(values[key]).strip().lower() not in {"1", "true", "yes"}:
            errors.append(f"{key} must be enabled before Python starts")
    return values, errors


def _build_offline_preflight(args: argparse.Namespace, root: Path) -> dict[str, Any]:
    errors = _capture_preflight_parameter_errors(args)
    try:
        finish_timeout_effective: float | None = _effective_finish_timeout(
            args.finish_timeout, live=True)
    except ValueError:
        finish_timeout_effective = None
    report: dict[str, Any] = {
        "scope": "offline Windows runtime, local paths and capture parameters",
        "ros": "NOT_CHECKED",
        "scope_status": "UNKNOWN",
        "physical_devices_opened": False,
        "camera_source_started": False,
        "can_tx": 0,
        "errors": errors,
        "paths": {},
        "parameters": {
            "operator_set": bool(str(args.operator).strip()),
            "task_set": bool(str(args.task).strip()),
            "duration_s": args.duration,
            "manual_lifecycle": args.manual_lifecycle,
            "continuous_episodes": args.continuous_episodes,
            "max_capture_duration_s": args.max_capture_duration,
            "lifecycle_tail_s": args.lifecycle_tail,
            "queue_capacity": args.queue_capacity,
            "finish_timeout_s": finish_timeout_effective,
            "sensor_source": args.sensor_source,
            "ros_domain_id": args.domain_id,
            "site_confirmed": args.site_confirmed,
        },
    }
    required_files = [
        root / "gamepad_capture/live_dataset/cli.py",
        root / "gamepad_capture/live_dataset/writer.py",
        root / "gamepad_capture/scripts/gamepad_teleop.ps1",
        root / "cache/live_lerobot_deps/msgpack/__init__.py",
    ]
    missing_files = [str(path) for path in required_files if not path.is_file()]
    report["project_root"] = str(root)
    python_version = ".".join(map(str, sys.version_info[:3]))
    report["python_version"] = python_version
    if python_version != DATASET_PYTHON_VERSION:
        errors.append(
            f"Python {python_version} does not match locked dataset runtime "
            f"{DATASET_PYTHON_VERSION}"
        )
    report["project_files_ready"] = not missing_files
    report["missing_project_files"] = missing_files
    if missing_files:
        errors.append(f"required local project files are missing: {missing_files}")

    report["offline_environment"], environment_errors = _check_offline_environment()
    errors.extend(environment_errors)

    try:
        report["runtime"] = check_lerobot_runtime(root, require_msgpack=True)
    except (ImportError, RuntimeError, importlib.metadata.PackageNotFoundError, OSError) as exc:
        report["runtime_error"] = f"{type(exc).__name__}: {exc}"
        errors.append(report["runtime_error"])
    if "runtime_error" not in report:
        try:
            report["encoder_imports"] = _check_local_encoder_imports()
        except Exception as exc:
            report["encoder_import_error"] = f"{type(exc).__name__}: {exc}"
            errors.append(report["encoder_import_error"])

    if args.output is not None:
        report["paths"]["output"] = _inspect_path(args.output, must_not_exist=True)
    else:
        report["paths"]["default_output_parent"] = _inspect_path(
            root / DEFAULT_OUTPUT_RELATIVE, must_not_exist=False)
    if args.cache_root is not None:
        report["paths"]["cache_root"] = _inspect_path(args.cache_root, must_not_exist=False)
    for label, path_report in report["paths"].items():
        if path_report.get("error"):
            errors.append(f"{label}: {path_report['error']}")
    report["errors"] = errors
    report["status"] = "LOCAL_RUNTIME_READY" if not errors else "FAILED"
    return report


def _load_writer(root: Path, output: Path, *, queue_capacity: int,
                 cache_root: Path | None, finish_timeout_s: float):
    check_lerobot_runtime(root, require_msgpack=False)
    from .writer import LeRobotEpisodeWriter

    print("WRITER_WARMING_UP: pinned runtime, offline cache, LeRobot import", flush=True)
    writer = LeRobotEpisodeWriter(
        output, queue_capacity=queue_capacity, cache_root=cache_root,
        finish_timeout_s=finish_timeout_s,
    )
    print("WRITER_READY: bounded writer queue active", flush=True)
    return writer


def _edited_terminal_line(line: str) -> str:
    """Interpret raw terminal edits if the console has not consumed them."""
    text = []
    cursor = 0
    index = 0
    while index < len(line):
        char = line[index]
        if char in ('\b', '\x7f'):
            if cursor:
                cursor -= 1
                del text[cursor]
        elif char == '\x1b':
            match = re.match(r'\x1b\[(3~|D|C|H|F)', line[index:])
            if match is None:
                # Do not turn an unrecognized escape sequence into a command.
                return '<unsupported terminal editing sequence>'
            key = match.group(1)
            if key == '3~' and cursor < len(text):
                del text[cursor]
            elif key == 'D':
                cursor = max(0, cursor - 1)
            elif key == 'C':
                cursor = min(len(text), cursor + 1)
            elif key == 'H':
                cursor = 0
            elif key == 'F':
                cursor = len(text)
            index += len(match.group(0)) - 1
        elif char not in ('\r', '\n'):
            if ord(char) < 32:
                return '<unsupported terminal control character>'
            text.insert(cursor, char)
            cursor += 1
        index += 1
    return ''.join(text).strip()


def _terminal_reader(output: queue.Queue[str | None]) -> None:
    # A preceding native program can leave the shared Windows console with
    # echo disabled. Restore normal visible, editable line input when attached
    # to a console; redirected stdin keeps its existing behaviour.
    if os.name == 'nt' and sys.stdin.isatty():
        import ctypes
        import msvcrt
        from ctypes import wintypes
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.GetConsoleMode.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
        kernel.SetConsoleMode.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        handle = msvcrt.get_osfhandle(sys.stdin.fileno())
        mode = wintypes.DWORD()
        if kernel.GetConsoleMode(handle, ctypes.byref(mode)):
            kernel.SetConsoleMode(handle, mode.value | 0x0002 | 0x0004)
    while True:
        line = sys.stdin.readline()
        if line == "":
            output.put(None)
            return
        output.put(_edited_terminal_line(line))


def _lifecycle_terminal_command(value: str, state: str) -> str:
    """Normalize terminal input and let a blank line start a waiting capture."""
    command = value.strip().upper()
    if command == 'NEXT' and state in ('WAITING_FOR_START', 'WAITING_FOR_NEXT_START'):
        return 'START'
    if not command and state in ("WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
        return "START"
    return command


def _effective_finish_timeout(value: float | None, *, live: bool) -> float:
    default = DEFAULT_FINISH_TIMEOUT_S if live else DEFAULT_REPLAY_FINISH_TIMEOUT_S
    timeout = default if value is None else float(value)
    if (not math.isfinite(timeout) or timeout < 1
            or timeout > MAX_FINISH_TIMEOUT_S):
        raise ValueError(
            f"finish timeout must be finite and in [1, {MAX_FINISH_TIMEOUT_S:g}] seconds"
        )
    return timeout


def _run_live(args: argparse.Namespace, root: Path) -> int:
    finish_timeout_s = _effective_finish_timeout(args.finish_timeout, live=True)
    if args.domain_id not in (None, 231, 222):
        raise ValueError("live ROS domain must be 231, or 222 with --site-confirmed")
    if args.domain_id == 222 and not args.site_confirmed:
        raise ValueError("domain 222 requires --site-confirmed")
    if args.dry_run:
        command = build_wsl_adapter_command(root, distro=args.wsl_distro,
                                           sensor_source=args.sensor_source,
                                           domain_id=args.domain_id,
                                           site_confirmed=args.site_confirmed)
        print(json.dumps({"status": "DRY_RUN", "backend": "lerobot_live",
                          "python": sys.executable, "output": str(args.output),
                          "wsl_command": command,
                          "finish_timeout_s": finish_timeout_s,
                          "finish_timeout_max_s": MAX_FINISH_TIMEOUT_S,
                          "can_tx": 0, "camera_source_started": False,
                          "lerobot_imported": False, "ros_connected": False},
                         ensure_ascii=False, indent=2))
        return 0

    runtime = check_lerobot_runtime(root, require_msgpack=True)
    print(f"LEROBOT_READY: version={runtime['lerobot_version']}; "
          f"revision={runtime['lerobot_revision']}; "
          f"msgpack={runtime['msgpack_version']} (project-local)", flush=True)
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"live output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = _load_writer(root, output, queue_capacity=args.queue_capacity,
                          cache_root=args.cache_root,
                          finish_timeout_s=finish_timeout_s)
    try:
        command = build_wsl_adapter_command(root, distro=args.wsl_distro,
                                           sensor_source=args.sensor_source,
                                           domain_id=args.domain_id,
                                           site_confirmed=args.site_confirmed)
        adapter = FramedStdioAdapter(
            command, msgpack=add_msgpack_dependency(root),
            emit_stderr=lambda text: print(text, file=sys.stderr, flush=True),
        )
    except Exception:
        writer.close()
        raise

    coordinator = CaptureCoordinator(
        adapter, writer, operator=args.operator, task=args.task,
        manual_lifecycle=args.manual_lifecycle,
        continuous_episodes=args.continuous_episodes,
        duration_s=args.duration,
        max_capture_duration_s=args.max_capture_duration,
        tail_s=args.lifecycle_tail,
        allow_domain_222=args.domain_id == 222 and args.site_confirmed,
        lifecycle_timeout_s=args.lifecycle_timeout,
        finish_timeout_s=finish_timeout_s,
        online_next_state=True,
    )
    commands: queue.Queue[str | None] = queue.Queue()
    command_thread = threading.Thread(target=_terminal_reader, args=(commands,),
                                      name="live-capture-terminal", daemon=True)
    command_thread.start()
    started_at = time.monotonic()
    ready_deadline = started_at + args.startup_timeout
    automatic_start_sent = False
    command_eof = False
    terminal_command: str | None = None
    no_terminal_command = object()
    prompted_state: str | None = None
    home = None
    if args.home_before_start:
        home_command = ['wsl.exe', '-d', args.wsl_distro, '--exec', 'env',
                        f'ROS_DOMAIN_ID={args.domain_id}', '/bin/bash',
                        _to_wsl_path(root) + '/gamepad_capture/scripts/request_native_home.sh']
        if args.home_task_scope_id:
            home_command += ['--task-scope-id', args.home_task_scope_id]
        home = HomeBeforeStart(home_command, root / 'cache/live_lerobot/home_requests')
    try:
        while coordinator.state.state != "CLOSED":
            message = adapter.receive(timeout_s=0.05)
            if message is not None:
                coordinator.handle(message)
            dropped = adapter.take_transport_dropped_samples()
            if dropped:
                coordinator.record_transport_drops(dropped)
            if home is not None and home.busy:
                home_result = home.poll()
                if home_result is not None:
                    print('PRE_CAPTURE_HOME_RESULT: ' + json.dumps(home_result), flush=True)
                    if not home_result['ok']:
                        raise CaptureError('home failed; recording not started; see home log')
                    coordinator.command('START')
                    prompted_state = None

            if not coordinator.ready and time.monotonic() >= ready_deadline:
                raise CaptureError("live preflight timed out before ROS schema and camera identities were ready")

            if coordinator.ready and not args.manual_lifecycle and not automatic_start_sent:
                coordinator.start_episode()
                automatic_start_sent = True

            if args.manual_lifecycle and not command_eof:
                try:
                    terminal_command = commands.get_nowait()
                except queue.Empty:
                    terminal_command = no_terminal_command
                if terminal_command is None:
                    command_eof = True
                    if coordinator.state.state in ("STARTING", "CAPTURING"):
                        coordinator.request_stop("terminal_eof", over=True)
                        coordinator.state.partial_reasons.append("terminal_input_closed")
                    elif coordinator.state.state in ("WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
                        coordinator.state.state = "CLOSED"
                elif terminal_command is not no_terminal_command:
                    command = _lifecycle_terminal_command(
                        terminal_command, coordinator.state.state)
                    if not command:
                        continue
                    try:
                        if home is not None and home.busy:
                            print('HOME_IN_PROGRESS: wait for home completion; recording has not started.', flush=True)
                            continue
                        if command == 'START' and home is not None:
                            coordinator._require_ready()
                            if coordinator.state.state not in ('WAITING_FOR_START', 'WAITING_FOR_NEXT_START'):
                                raise CaptureError('START is only accepted while waiting for an episode')
                            home.start()
                            print('PRE_CAPTURE_HOME: returning to zero through the existing CAN owner; keep sticks neutral.', flush=True)
                            continue
                        result = coordinator.command(command)
                        if result == "IGNORED":
                            print(f"COMMAND_IGNORED: received {command!r}; use START, STOP, OVER, SAVE, RETAKE, SUCCESS, or FAILURE",
                                  flush=True)
                        else:
                            prompted_state = None
                    except CaptureError as exc:
                        print(f"COMMAND_REJECTED: {exc}", file=sys.stderr, flush=True)

            if command_eof and coordinator.state.state == "AWAITING_LABEL":
                coordinator._abort_active("terminal_input_closed_before_episode_label")
                break

            coordinator.tick()
            if coordinator.state.state == "CLOSED":
                break
            if coordinator.ready and not (home is not None and home.busy) and coordinator.state.state in (
                    "WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
                if prompted_state != coordinator.state.state:
                    if coordinator.state.state == "WAITING_FOR_START":
                        print("CAPTURE_READY: press ENTER or type START to begin recording; EOF closes the session.", flush=True)
                    else:
                        print("SEQUENCE_READY: 输入 NEXT 开始下一条，OVER 结束；复用当前录制器。", flush=True)
                    prompted_state = coordinator.state.state
            if message is None and not coordinator.ready and time.monotonic() >= ready_deadline:
                raise CaptureError("live preflight timed out")
    except KeyboardInterrupt:
        if coordinator.state.state in ("STARTING", "CAPTURING"):
            try:
                coordinator.request_stop("keyboard_interrupt", over=True)
            except CaptureError:
                pass
            coordinator.state.partial_reasons.append("keyboard_interrupt")
        elif coordinator.state.state in ("WAITING_FOR_START", "WAITING_FOR_NEXT_START"):
            coordinator.state.state = "CLOSED"
        if coordinator.state.state in ("STOP_REQUESTED", "TAIL_RECORDING"):
            deadline = time.monotonic() + max(1.0, args.lifecycle_tail + 2.0)
            while coordinator.state.state in ("STOP_REQUESTED", "TAIL_RECORDING") and time.monotonic() < deadline:
                try:
                    message = adapter.receive(timeout_s=0.05)
                    if message is not None:
                        coordinator.handle(message)
                except CaptureError as exc:
                    print(f"INTERRUPT_DRAIN_FAILED: {exc}", file=sys.stderr, flush=True)
                    break
        coordinator._abort_active("keyboard_interrupt_partial")
        return_code = 130
    except (CaptureError, OSError, RuntimeError, ValueError) as exc:
        print(f"LIVE_CAPTURE_FAILED: {exc}", file=sys.stderr, flush=True)
        coordinator._abort_active(f"live_capture_failed:{type(exc).__name__}")
        return_code = 2
    else:
        return_code = 0
    finally:
        if home is not None and home.close():
            print(f"HOME_REQUEST_STILL_PENDING: recorder exiting; existing home service may continue; log={home.path}; physical_stop_verified=false", flush=True)
        result, return_code = _close_live_session(coordinator, return_code)
    return return_code


def _run_replay(args: argparse.Namespace, root: Path) -> int:
    finish_timeout_s = _effective_finish_timeout(args.finish_timeout, live=False)
    profile_started_ns = time.perf_counter_ns()
    profile: dict[str, Any] = {
        "schema": "piper_live_writer_replay_profile_v1",
        "mode": "historical_review_replay",
        "stages_s": {},
        "result": "RUNNING",
        "ros_connected": False,
        "robot_motion_commanded": False,
        "training_eligible": False,
    }
    writer = None
    report: dict[str, Any] | None = None
    close_report: dict[str, Any] | None = None
    input_started_ns: int | None = None
    input_ended_ns: int | None = None

    def record_stage(name: str, started_ns: int) -> None:
        profile["stages_s"][name] = round((time.perf_counter_ns() - started_ns) / 1e9, 6)

    if args.domain_id is not None and args.domain_id != 231:
        raise ValueError("offline replay is ROS-disconnected and accepts only the documented domain 231 marker")
    if args.domain_id == 222:
        raise ValueError("offline replay must reject domain 222")
    if args.dry_run:
        print(json.dumps({"status": "DRY_RUN", "backend": "historical_bag_replay",
                          "bag": str(args.bag) if args.bag else None,
                          "manifest": str(args.manifest), "limit": args.limit,
                          "rate": args.rate, "ros_connected": False,
                          "finish_timeout_s": finish_timeout_s,
                          "can_tx": 0, "robot_motion_commanded": False,
                          "lerobot_imported": False}, ensure_ascii=False, indent=2))
        return 0
    if (not math.isfinite(args.rate) or args.rate < 0
            or (args.limit is not None and args.limit <= 0)):
        raise ValueError("replay rate/finish timeout must be finite and non-negative/positive; limit must be positive")
    if not args.manifest.is_file():
        raise FileNotFoundError(f"historical bag manifest not found: {args.manifest}")
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"replay output already exists: {output}")
    try:
        stage_started_ns = time.perf_counter_ns()
        from .pairing import replay_rosbag
        record_stage("pairing_module_import", stage_started_ns)

        stage_started_ns = time.perf_counter_ns()
        runtime = check_lerobot_runtime(root, require_msgpack=False)
        record_stage("pinned_runtime_check", stage_started_ns)
        profile["runtime"] = runtime

        stage_started_ns = time.perf_counter_ns()
        from .writer import LeRobotEpisodeWriter
        record_stage("writer_module_import", stage_started_ns)

        stage_started_ns = time.perf_counter_ns()
        print("WRITER_WARMING_UP: pinned runtime, offline cache, LeRobot import", flush=True)
        writer = LeRobotEpisodeWriter(output, queue_capacity=args.queue_capacity,
                                      cache_root=args.cache_root)
        record_stage("writer_construct_to_worker_ready", stage_started_ns)
        print("WRITER_READY: bounded writer queue active", flush=True)
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        source_run_id = str(manifest.get("run_id", ""))
        if not source_run_id:
            raise ValueError("historical manifest has no run_id")
        run_id = f"replay_{source_run_id}"
        task = str(manifest.get("task") or args.task)
        profile.update({"source_run_id": source_run_id, "run_id": run_id,
                        "queue_capacity": args.queue_capacity, "rate": args.rate,
                        "limit": args.limit,
                        "finish_timeout_s": finish_timeout_s})
        metadata = {"source_kind": "historical_review", "review_only": True,
                    "source_run_id": source_run_id,
                    "source_manifest": str(args.manifest.resolve()),
                    "scope_info": {"status": "UNKNOWN", "reason": "historical capture metadata"},
                    "action_semantics": "matched actual historical send/receipt evidence; replay only",
                    "physical_stop_verified": False,
                    "robot_motion_commanded_by_recorder": False,
                    "ros_domain_id": 231,
                    "ros_domain_meaning": "offline validation marker only; no ROS node was created",
                    "task_scope_id": "UNKNOWN"}
        stage_started_ns = time.perf_counter_ns()
        writer.start_episode(run_id, task, metadata)
        record_stage("episode_start", stage_started_ns)
        source_anchor_ns: int | None = None
        host_anchor_ns: int | None = None
        first_source_time_ns: int | None = None
        last_source_time_ns: int | None = None
        first_submit_ns: int | None = None
        last_submit_ns: int | None = None
        interarrival_total_ns = 0
        interarrival_max_ns = 0
        interarrival_count = 0
        pacing_sleep_s = 0.0
        pacing_lateness_total_ns = 0
        pacing_lateness_max_ns = 0
        pacing_late_over_1ms = 0
        count = 0
        dropped = 0
        started = time.monotonic()
        input_started_ns = time.perf_counter_ns()
        for sample in replay_rosbag(args.manifest, args.bag):
            source_map = sample.get("source_map", {})
            state_map = source_map.get("state", {}) if isinstance(source_map, Mapping) else {}
            event_time_ns = state_map.get("arrival_monotonic_ns")
            if type(event_time_ns) is int:
                first_source_time_ns = event_time_ns if first_source_time_ns is None else first_source_time_ns
                last_source_time_ns = event_time_ns
            if args.rate > 0 and type(event_time_ns) is int:
                if source_anchor_ns is None:
                    source_anchor_ns = event_time_ns
                    host_anchor_ns = time.perf_counter_ns()
                sleep_s, lateness_ns = _pace_replay_deadline(
                    source_anchor_ns, host_anchor_ns, event_time_ns, args.rate,
                )
                pacing_sleep_s += sleep_s
                pacing_lateness_total_ns += lateness_ns
                pacing_lateness_max_ns = max(pacing_lateness_max_ns, lateness_ns)
                if lateness_ns > 1_000_000:
                    pacing_late_over_1ms += 1
            sample = dict(sample)
            sample["run_id"] = run_id
            sample["task"] = task
            submit_ns = time.perf_counter_ns()
            if first_submit_ns is None:
                first_submit_ns = submit_ns
            if last_submit_ns is not None:
                interval_ns = submit_ns - last_submit_ns
                interarrival_total_ns += interval_ns
                interarrival_max_ns = max(interarrival_max_ns, interval_ns)
                interarrival_count += 1
            last_submit_ns = submit_ns
            if not writer.submit(sample):
                dropped += 1
            count += 1
            if count % 100 == 0:
                print(f"REPLAY_PROGRESS: rows={count}; rate={args.rate:g}x", flush=True)
            if args.limit is not None and count >= args.limit:
                break
        input_ended_ns = time.perf_counter_ns()
        profile["stages_s"]["input_stream"] = round((input_ended_ns - input_started_ns) / 1e9, 6)
        profile["input_rows"] = count
        profile["transport_dropped"] = dropped
        source_span_s = ((last_source_time_ns - first_source_time_ns) / 1e9
                         if first_source_time_ns is not None and last_source_time_ns is not None
                         else None)
        host_span_s = ((last_submit_ns - first_submit_ns) / 1e9
                       if first_submit_ns is not None and last_submit_ns is not None else None)
        profile["source_time_span_s"] = round(source_span_s, 6) if source_span_s is not None else None
        profile["source_rate_hz"] = (
            round((count - 1) / source_span_s, 4)
            if source_span_s and source_span_s > 0 else None
        )
        profile["host_submit_span_s"] = round(host_span_s, 6) if host_span_s is not None else None
        profile["host_submit_rate_hz"] = (
            round((count - 1) / host_span_s, 4)
            if host_span_s and host_span_s > 0 else None
        )
        profile["host_submit_interarrival_mean_ms"] = (
            round(interarrival_total_ns / interarrival_count / 1e6, 4)
            if interarrival_count else None
        )
        profile["host_submit_interarrival_max_ms"] = round(interarrival_max_ns / 1e6, 4)
        profile["pacing_sleep_s"] = round(pacing_sleep_s, 6)
        profile["pacing_lateness_total_s"] = round(pacing_lateness_total_ns / 1e9, 6)
        profile["pacing_lateness_max_ms"] = round(pacing_lateness_max_ns / 1e6, 4)
        profile["pacing_late_over_1ms_rows"] = pacing_late_over_1ms
        profile["pacing_mode"] = "absolute_source_deadline" if args.rate > 0 else "unpaced"
        if count == 0:
            raise RuntimeError("historical replay produced no samples; refusing an empty dataset episode")
        label = {"demonstration_label": "historical-review-replay",
                 "outcome": "unknown", "successful_complete_end": False,
                 "complete_closeout_label_present": False,
                 "historical_review_only": True, "training_eligible": False,
                 "physical_stop_verified": False}
        stage_started_ns = time.perf_counter_ns()
        report = writer.finish_episode(label=label, events=[{
            "owner": "offline_replay", "event": "HISTORICAL_REPLAY_COMPLETE",
            "source_run_id": source_run_id, "frames_submitted": count,
            "transport_dropped": dropped, "ros_connected": False,
            "robot_motion_commanded": False,
        }], timeout_s=finish_timeout_s)
        record_stage("input_end_to_finish_return", input_ended_ns)
        profile["writer_performance"] = report.get("performance")
        profile["finish_report"] = report
        profile["result"] = report.get("status", "UNKNOWN")
        elapsed = time.monotonic() - started
        print(f"REPLAY_QUALITY: status={report.get('status')}; source_run_id={source_run_id}; "
              f"rows={count}; dropped={dropped}; training_eligible=false; "
              f"ros_connected=false; robot_motion_commanded=false; elapsed_s={elapsed:.3f}",
              flush=True)
        return (0 if report.get("status") == "SAVED_REVIEW_ONLY" and dropped == 0
                and report.get("dropped_frames", 0) == 0 else 2)
    except Exception as exc:
        profile["result"] = "FAILED"
        profile["error"] = f"{type(exc).__name__}: {exc}"
        if writer is not None:
            writer.abort_episode("historical_replay_failed", events=[{
                "owner": "offline_replay", "event": "HISTORICAL_REPLAY_FAILED",
                "frames_submitted": profile.get("input_rows", 0),
                "transport_dropped": profile.get("transport_dropped", 0),
                "ros_connected": False, "robot_motion_commanded": False,
            }])
        raise
    finally:
        if writer is not None:
            stage_started_ns = time.perf_counter_ns()
            close_report = writer.close()
            record_stage("writer_close", stage_started_ns)
            profile["close_report"] = close_report
            profile["writer_initialization_timings"] = writer._global_timing_snapshot()
        if input_started_ns is not None:
            profile["input_end_wall_elapsed_s"] = round(
                ((input_ended_ns or time.perf_counter_ns()) - input_started_ns) / 1e9, 6
            )
        profile["process_run_s"] = round((time.perf_counter_ns() - profile_started_ns) / 1e9, 6)
        if args.profile_json is not None:
            profile_path = Path(args.profile_json).expanduser().resolve()
            profile_path.parent.mkdir(parents=True, exist_ok=True)
            profile_path.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n",
                                    encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("live", "replay", "preflight"))
    parser.add_argument("--project-root", type=Path, default=_project_root())
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--operator", default=os.environ.get("USERNAME", "unknown"))
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--duration", type=float, default=20.0,
                        help="legacy fixed-duration window; used when manual lifecycle is disabled")
    parser.add_argument("--manual-lifecycle", action="store_true")
    parser.add_argument("--continuous-episodes", action="store_true")
    parser.add_argument('--home-before-start', action='store_true',
                        help='request existing native home service before each manual START')
    parser.add_argument('--home-task-scope-id', default='')
    parser.add_argument("--max-capture-duration", type=float, default=None)
    parser.add_argument("--lifecycle-tail", type=float, default=1.0)
    parser.add_argument("--queue-capacity", type=int, default=20)
    parser.add_argument("--cache-root", type=Path, default=None)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--lifecycle-timeout", type=float, default=20.0,
                        help="bounded wait for lifecycle publish and stop-tail completion")
    parser.add_argument("--wsl-distro", default="Ubuntu-22.04")
    parser.add_argument("--sensor-source", choices=("windows", "native"), default="windows")
    parser.add_argument("--domain-id", type=int, default=None)
    parser.add_argument("--site-confirmed", action="store_true")
    parser.add_argument("--manifest", type=Path,
                        default=_project_root() / "gamepad_capture/data/manifests/20261017T113409Z_72321bfd.json")
    parser.add_argument("--bag", type=Path, default=None)
    parser.add_argument("--limit", type=int, default=None,
                        help="stop offline replay after this many paired rows")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="offline replay speed relative to source time; 0 runs as fast as possible")
    parser.add_argument(
        "--finish-timeout", type=float, default=None,
        help=("bounded episode finalize wait: live defaults to 180 seconds; replay defaults "
              "to 90 seconds; both are limited to 300 seconds"),
    )
    parser.add_argument("--profile-json", type=Path, default=None,
                        help="write bounded replay/writer phase timings and queue telemetry to this JSON path")
    parser.add_argument("--dry-run", action="store_true",
                        help="print arguments without importing ROS/LeRobot or connecting to ROS")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    root = args.project_root.resolve()
    if args.home_before_start and not (args.mode == 'live' and args.manual_lifecycle
            and args.sensor_source == 'native' and args.domain_id == 222 and args.site_confirmed):
        parser.error('--home-before-start requires live manual native domain 222 with site confirmation')
    if args.continuous_episodes and not args.manual_lifecycle:
        parser.error("--continuous-episodes requires --manual-lifecycle")
    if args.max_capture_duration is not None and not args.manual_lifecycle:
        parser.error("--max-capture-duration requires --manual-lifecycle")
    if args.mode == "preflight":
        if args.dry_run:
            print(json.dumps({"status": "DRY_RUN", "backend": "lerobot",
                              "python": sys.executable, "imports": False,
                              "ros_connected": False}, ensure_ascii=False, indent=2))
            return 0
        report = _build_offline_preflight(args, root)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["status"] == "LOCAL_RUNTIME_READY" else 2
    if args.output is None:
        args.output = _default_output(root)
    try:
        if args.mode == "live":
            return _run_live(args, root)
        return _run_replay(args, root)
    except (CaptureError, FileNotFoundError, FileExistsError, ImportError,
            RuntimeError, ValueError, OSError) as exc:
        print(f"LIVE_DATASET_FAILED: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
