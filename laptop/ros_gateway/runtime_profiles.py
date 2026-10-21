"""Load the shared, non-secret ROS deployment wiring profiles.

Profiles centralize connection and startup defaults only. Robot limits remain
owned by the existing PiPER control profile and SafetyBridge.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from pathlib import PurePosixPath
import subprocess
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = PROJECT_ROOT / "configs" / "ros" / "runtime_profiles.json"
PROFILE_SCHEMA = "pi05_ros_runtime_profiles_v1"


def _object_without_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _read_document(path: Path) -> tuple[dict[str, Any], str]:
    raw = path.read_bytes()
    document = json.loads(raw, object_pairs_hook=_object_without_duplicate_keys)
    if not isinstance(document, dict) or document.get("schema") != PROFILE_SCHEMA:
        raise ValueError(f"unsupported ROS deployment profile file: {path}")
    if not isinstance(document.get("profiles"), dict):
        raise ValueError("ROS deployment profile file has no profiles mapping")
    return document, hashlib.sha256(raw).hexdigest()


def _validate_port(name: str, value: Any) -> None:
    if value is not None and (type(value) is not int or not 1 <= value <= 65535):
        raise ValueError(f"{name} must be null or an integer TCP port in 1..65535")


def _validate_profile(name: str, profile: dict[str, Any]) -> None:
    if not isinstance(profile, dict):
        raise ValueError(f"profile {name!r} must be an object")
    domain = profile.get("ros_domain_id")
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError(f"profile {name!r} has an invalid ROS domain ID")
    gateway = profile.get("gateway")
    if not isinstance(gateway, dict):
        raise ValueError(f"profile {name!r} has no gateway mapping")
    port_keys = ("control_port", "base_image_port", "wrist_image_port")
    ports = [gateway.get(key) for key in port_keys]
    for key, value in zip(port_keys, ports):
        _validate_port(f"{name}.gateway.{key}", value)
    active_ports = [port for port in ports if port is not None]
    if len(active_ports) != len(set(active_ports)):
        raise ValueError(f"profile {name!r} assigns one TCP port to multiple streams")

    features = profile.get("features")
    if not isinstance(features, dict):
        raise ValueError(f"profile {name!r} has no feature gates")
    if name == "p2_control":
        if gateway["control_port"] != 18768 or gateway.get("camera_enabled") is not False:
            raise ValueError("p2_control must use the P2 control-only gateway")
        if profile.get("ros_domain_id") != 222:
            raise ValueError("p2_control must use the active deployment ROS domain 222")
        if any(features.get(key) is not False for key in
               ("camera", "policy_inference", "run_task_action")):
            raise ValueError("p2_control must remain camera-, model-, and task-action-free")
    elif name == "p3_p4_shadow":
        expected = (18765, 18766, 18767)
        if tuple(ports) != expected:
            raise ValueError("p3_p4_shadow gateway ports do not match the shared profile")
        if profile.get("ros_domain_id") != 222:
            raise ValueError("p3_p4_shadow must use ROS domain 222")
        if profile.get("rates", {}).get("image_publish_hz") != 30:
            raise ValueError("p3_p4_shadow image rate must match the current 30 Hz setting")
        if features.get("run_task_action") is not False:
            raise ValueError("p3_p4_shadow must not enable physical task control")
        policy = profile.get("policy", {})
        if (policy.get("host_resolution") != "wsl_default_route_gateway"
                or policy.get("port") != 18021
                or not policy.get("identity_lock_path")):
            raise ValueError("p3_p4_shadow must use the WSL-scoped model tunnel profile")
    elif name == "p4_shadow_only":
        if any(value is not None for value in ports):
            raise ValueError("p4_shadow_only must not start a gateway")
        if profile.get("ros_domain_id") != 222:
            raise ValueError("p4_shadow_only must use the live ROS domain 222")
        if features.get("policy_inference") is not True or any(
            features.get(key) is not False
            for key in ("camera", "run_task_action", "physical_task_control")
        ):
            raise ValueError("p4_shadow_only must remain prediction-only")
        policy = profile.get("policy", {})
        if (policy.get("host_resolution") != "wsl_default_route_gateway"
                or policy.get("port") != 18021
                or not policy.get("identity_lock_path")):
            raise ValueError("p4_shadow_only must use the WSL-scoped model tunnel profile")
    elif name in {"p6_camera_record", "p6_operator_demo"}:
        if any(value is not None for value in ports):
            raise ValueError(f"{name} must not start a gateway")
        expected_domain = 78 if name == "p6_camera_record" else 222
        if profile.get("ros_domain_id") != expected_domain:
            raise ValueError(f"{name} has an unexpected ROS domain")
        topics = profile.get("record_topics")
        if (not isinstance(topics, dict) or not topics
                or any(not isinstance(topic, str) or not topic.startswith("/")
                       or any(char.isspace() for char in topic)
                       for topic in topics.values())):
            raise ValueError(f"{name} record_topics must be named absolute ROS topic paths")


def load_runtime_profile(
    name: str,
    *,
    path: str | Path | None = None,
) -> tuple[dict[str, Any], str]:
    """Return one validated profile and the SHA-256 of its source file."""
    profile_path = Path(path) if path is not None else PROFILE_PATH
    document, digest = _read_document(profile_path)
    profile = document["profiles"].get(name)
    if profile is None:
        choices = ", ".join(sorted(document["profiles"]))
        raise ValueError(f"unknown ROS runtime profile {name!r}; choose one of: {choices}")
    _validate_profile(name, profile)
    return json.loads(json.dumps(profile)), digest


def load_expected_policy_identity(profile: dict[str, Any]) -> dict[str, Any] | None:
    """Read the configured model lock for display and startup consistency checks.

    This reports the expected identity only. The policy node still validates the
    live model-service metadata against the same lock after connecting.
    """
    policy = profile.get("policy", {})
    relative = policy.get("identity_lock_path")
    if relative is None:
        return None
    if not isinstance(relative, str) or not relative:
        raise ValueError("policy.identity_lock_path must be a relative project path")
    lock_relative = PurePosixPath(relative)
    if lock_relative.is_absolute() or ".." in lock_relative.parts:
        raise ValueError("policy.identity_lock_path must stay within the project")
    lock_path = PROJECT_ROOT.joinpath(*lock_relative.parts)
    raw = lock_path.read_bytes()
    lock = json.loads(raw, object_pairs_hook=_object_without_duplicate_keys)
    if not isinstance(lock, dict):
        raise ValueError("policy identity lock must be a JSON object")
    allowed_schemas = {
        "piper_proxy_artifact_lock_v1",
        "piper_derived_state_multi_shadow_identity_lock_v1",
    }
    if lock.get("schema") not in allowed_schemas:
        raise ValueError("policy identity lock has an unsupported schema")
    for field in ("checkpoint", "dataset", "config_schema", "openpi_commit"):
        if not isinstance(lock.get(field), str) or not lock[field].strip():
            raise ValueError(f"policy identity lock is missing {field}")
    if lock.get("hardware_control_allowed") is not False:
        raise ValueError("policy identity lock must keep hardware control disabled")
    if "prediction_only" in lock and lock["prediction_only"] is not True:
        raise ValueError("policy identity lock must be prediction-only")
    return {
        "identity_lock_relative_path": lock_relative.as_posix(),
        "identity_lock_sha256": hashlib.sha256(raw).hexdigest(),
        "schema": lock["schema"],
        "config_schema": lock["config_schema"],
        "checkpoint": lock["checkpoint"],
        "checkpoint_tree_sha256": lock.get(
            "checkpoint_tree_sha256", lock.get("checkpoint_sha256", "")
        ),
        "dataset": lock["dataset"],
        "openpi_commit": lock["openpi_commit"],
        "action_semantics": lock.get("action_semantics", ""),
        "hardware_control_allowed": False,
        "prediction_only": lock.get("prediction_only", True),
    }


def _environment_values(
    name: str,
    profile: dict[str, Any],
    digest: str,
    *,
    gateway_host: str | None = None,
    policy_host: str | None = None,
    default_workspace: str | None = None,
    wsl_distribution: str | None = None,
    wsl_project_root: str | None = None,
    policy_identity: dict[str, Any] | None = None,
) -> dict[str, str]:
    gateway = profile["gateway"]
    rates = profile.get("rates", {})
    policy = profile.get("policy", {})
    cameras = profile.get("cameras", {})
    values: dict[str, Any] = {
        "PI05_ROS_PROFILE": name,
        "PI05_ROS_PROFILE_SHA256": digest,
        "PI05_ROS_DOMAIN_ID": profile["ros_domain_id"],
        "PI05_ROS_DEFAULT_WORKSPACE": default_workspace,
        "PI05_WSL_DISTRIBUTION": wsl_distribution,
        "PI05_ROS_GATEWAY_HOST": gateway_host,
        "PI05_ROS_CONTROL_PORT": gateway.get("control_port"),
        "PI05_ROS_BASE_IMAGE_PORT": gateway.get("base_image_port"),
        "PI05_ROS_WRIST_IMAGE_PORT": gateway.get("wrist_image_port"),
        "PI05_ROS_CAMERA_CAPTURE_HZ": rates.get("camera_capture_hz"),
        "PI05_ROS_IMAGE_PUBLISH_HZ": rates.get("image_publish_hz"),
        "PI05_ROS_IMAGE_STREAM_HZ": gateway.get("image_stream_hz"),
        "PI05_ROS_POLICY_HZ": rates.get("policy_inference_hz"),
        "PI05_ROS_TARGET_HZ": rates.get("target_hz"),
        "PI05_ROS_TARGET_PERIOD_MS": (
            int(round(1000 / rates["target_hz"])) if rates.get("target_hz") else None
        ),
        "PI05_ROS_STREAM_DURATION_S": rates.get("stream_duration_s"),
        "PI05_POLICY_HOST": policy_host or policy.get("host"),
        "PI05_POLICY_PORT": policy.get("port"),
    }
    if policy_identity is not None:
        identity_root = PurePosixPath(wsl_project_root or "")
        values.update({
            "PI05_POLICY_IDENTITY_LOCK_PATH": str(
                identity_root / policy_identity["identity_lock_relative_path"]
            ),
            "PI05_POLICY_IDENTITY_LOCK_SHA256": policy_identity["identity_lock_sha256"],
            "PI05_POLICY_MODEL_CONFIG_SCHEMA": policy_identity["config_schema"],
            "PI05_POLICY_MODEL_CHECKPOINT": policy_identity["checkpoint"],
            "PI05_POLICY_MODEL_CHECKPOINT_SHA256": policy_identity["checkpoint_tree_sha256"],
            "PI05_POLICY_MODEL_DATASET": policy_identity["dataset"],
            "PI05_POLICY_MODEL_OPENPI_COMMIT": policy_identity["openpi_commit"],
            "PI05_POLICY_MODEL_ACTION_SEMANTICS": policy_identity["action_semantics"],
            "PI05_POLICY_MODEL_IDENTITY_SOURCE": "expected_identity_lock",
        })
    record_topics = profile.get("record_topics", {})
    if record_topics:
        values["PI05_ROS_RECORD_TOPICS"] = " ".join(record_topics.values())
    for role in ("base", "wrist"):
        camera = cameras.get(role, {})
        prefix = f"PI05_ROS_{role.upper()}_CAMERA"
        values[f"{prefix}_SOURCE"] = camera.get("source")
        values[f"{prefix}_NAME"] = camera.get("device_name")
        values[f"{prefix}_INDEX_HINT"] = camera.get("opencv_index_hint")

    # Keep unset ports explicit as zero so bridge launches do not accidentally
    # open camera streams in control-only or recording-only profiles.
    env: dict[str, str] = {}
    for key, value in values.items():
        if value is None and key.endswith("_PORT"):
            env[key] = "0"
        elif value is not None:
            env[key] = str(value)
    return env


def resolve_wsl_gateway_host(distribution: str = "Ubuntu-22.04") -> str:
    """Resolve the Windows-side address from the current WSL default route."""
    if os.name == "nt":
        command = [
            "wsl.exe", "-d", distribution, "--", "ip", "-4", "route", "show", "default",
        ]
    else:
        command = ["ip", "-4", "route", "show", "default"]
    try:
        result = subprocess.run(
            command, check=True, capture_output=True, text=True, timeout=8,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise RuntimeError(f"could not read the WSL default route: {error}") from error
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "default" and parts[1] == "via":
            candidate = parts[2]
            octets = candidate.split(".")
            if len(octets) == 4 and all(
                part.isdigit() and 0 <= int(part) <= 255 for part in octets
            ):
                return candidate
    raise RuntimeError("could not resolve the Windows gateway IP from the WSL default route")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--format", choices=("json", "env"), default="json")
    parser.add_argument("--distribution")
    parser.add_argument(
        "--resolve-host", action="store_true",
        help="resolve the current WSL-facing Windows IP from the default route",
    )
    args = parser.parse_args(argv)
    profile, digest = load_runtime_profile(args.profile)
    project, _ = _read_document(PROFILE_PATH)
    project_config = project.get("project", {})
    policy_identity = load_expected_policy_identity(profile)
    needs_gateway = any(
        profile.get("gateway", {}).get(key) is not None
        for key in ("control_port", "base_image_port", "wrist_image_port")
    )
    policy_needs_route = (
        profile.get("policy", {}).get("host_resolution")
        == "wsl_default_route_gateway"
    )
    resolved_host = (
        resolve_wsl_gateway_host(
            args.distribution or project_config.get("wsl_distribution", "Ubuntu-22.04")
        )
        if args.resolve_host and (needs_gateway or policy_needs_route) else None
    )
    policy_host = resolved_host if policy_needs_route else None
    if args.format == "env":
        for key, value in _environment_values(
            args.profile, profile, digest,
            gateway_host=resolved_host if needs_gateway else None,
            policy_host=policy_host,
            default_workspace=project_config.get("default_workspace"),
            wsl_distribution=project_config.get("wsl_distribution"),
            wsl_project_root=project_config.get("wsl_project_root"),
            policy_identity=policy_identity,
        ).items():
            if "\n" in value or "\r" in value or "=" in key:
                raise ValueError("runtime profile contains a value unsafe for environment export")
            print(f"{key}={value}")
    else:
        result = {
            "profile_name": args.profile,
            "profile_sha256": digest,
            "project": project_config,
            "profile": profile,
        }
        if needs_gateway and resolved_host is not None:
            result["gateway_host"] = resolved_host
        if policy_needs_route and resolved_host is not None:
            result["policy_host"] = resolved_host
        if policy_identity is not None:
            result["expected_policy_identity"] = policy_identity
            result["policy_identity_lock_wsl_path"] = str(
                PurePosixPath(project_config.get("wsl_project_root", ""))
                / policy_identity["identity_lock_relative_path"]
            )
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
