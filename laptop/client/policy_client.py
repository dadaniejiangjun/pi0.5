"""Shared identity validation/prewarm; no implicit connection or hardware access."""
from __future__ import annotations
import json
import math
import re
import time
from pathlib import Path
import numpy as np
from laptop.client.observation_schema import (
    make_piper_proxy_warmup_observation, validate_piper_semantic_observation,
)

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
ARTIFACT_IDENTITY_SCHEMA = "piper_policy_artifact_identity_v1"
ARTIFACT_LOCK_SCHEMA = "piper_proxy_artifact_lock_v1"
DERIVED_MULTI_ARTIFACT_SCHEMA = "piper_derived_state_multi_shadow_identity_v1"
DERIVED_MULTI_LOCK_SCHEMA = "piper_derived_state_multi_shadow_identity_lock_v1"
LEROBOT_PI05_LOCK_SCHEMA = "piper_lerobot_pi05_prediction_only_lock_v1"
LEROBOT_PI05_CONFIG_SCHEMA = "piper_lerobot_pi05_prediction_only_v1"
LEROBOT_PI05_BACKEND = "lerobot_pi05"
LEROBOT_PI05_ACTION_SEMANTICS = "lerobot_pi05_absolute_action_chunk_preview_only"
PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_IDENTITY_LOCK = (
    PROJECT_ROOT / "configs" / "runtime" / "pi05_proxy_shadow_identity_lock.json"
)
IDENTITY_HASH_FIELDS = (
    "checkpoint_sha256",
    "policy_config_sha256",
    "policy_pipeline_code_sha256",
    "training_config_code_sha256",
    "normalization_sha256",
    "dataset_tree_sha256",
)


def validate_artifact_identity_lock(lock: dict) -> dict:
    """Validate the laptop-side expected identity pins before shadow starts."""
    if isinstance(lock, dict) and lock.get("schema") == LEROBOT_PI05_LOCK_SCHEMA:
        required = {
            "schema", "config_schema", "backend", "action_semantics",
            "checkpoint", "checkpoint_sha256", "checkpoint_identity_sha256",
            "dataset", "dataset_manifest_sha256", "processor_identity_sha256",
            "normalization_sha256", "training_step", "base_model", "base_revision",
            "action_horizon", "action_dimension", "first_target_dt_s", "source_kind",
            "training_eligible", "software_only", "prediction_only", "control_enabled",
            "robot_control_ready", "hardware_control_allowed", "can_opened",
            "can_data_tx_calls",
        }
        if set(lock) != required:
            raise ValueError("LeRobot PI05 identity lock has missing or unexpected fields")
        _validate_lerobot_pi05_identity(lock, lock=True)
        return dict(lock)
    if isinstance(lock, dict) and lock.get("schema") == DERIVED_MULTI_LOCK_SCHEMA:
        required = {
            "schema", "artifact_identity_schema", "config_schema", "checkpoint",
            "checkpoint_tree_sha256", "dataset", "dataset_sha256",
            "normalization_identity", "normalization_sha256", "action_semantics",
            "action_target_kind", "action_horizon", "action_dimension",
            "first_target_dt_s", "hardware_control_allowed", "prediction_only",
            "openpi_commit", "training_provenance_sha256",
        }
        if set(lock) != required:
            raise ValueError("derived-state identity lock has missing or unexpected fields")
        if lock["artifact_identity_schema"] != DERIVED_MULTI_ARTIFACT_SCHEMA:
            raise ValueError("derived-state artifact identity schema mismatch")
        if lock["config_schema"] != "pi05_derived_state_multi_episode_shadow_v1":
            raise ValueError("derived-state lock config schema mismatch")
        if lock["action_semantics"] != "next_observed_state_proxy_nominal_units":
            raise ValueError("derived-state lock action semantics mismatch")
        if lock["action_target_kind"] != "derived_future_observed_state_position_target":
            raise ValueError("derived-state lock action target kind mismatch")
        if (lock["hardware_control_allowed"] is not False
                or lock["prediction_only"] is not True):
            raise PermissionError("derived-state identity lock cannot grant hardware control")
        if (lock["action_horizon"] != 16 or lock["action_dimension"] != 7
                or lock["first_target_dt_s"] != 0.1):
            raise ValueError("derived-state identity lock has a timing or action-shape mismatch")
        for field in ("checkpoint", "dataset", "normalization_identity"):
            if not isinstance(lock[field], str) or not lock[field].strip():
                raise ValueError(f"derived-state identity lock {field} is missing")
        if not isinstance(lock["openpi_commit"], str) or not _COMMIT_RE.fullmatch(lock["openpi_commit"]):
            raise ValueError("derived-state identity lock has invalid OpenPI commit")
        for field in ("checkpoint_tree_sha256", "dataset_sha256", "normalization_sha256",
                      "training_provenance_sha256"):
            if not isinstance(lock[field], str) or not _SHA256_RE.fullmatch(lock[field]):
                raise ValueError(f"derived-state identity lock {field} is not a lowercase SHA-256 digest")
        return dict(lock)

    required = {
        "schema",
        "artifact_identity_schema",
        "config_schema",
        "checkpoint",
        "dataset",
        "normalization_identity",
        "action_semantics",
        "hardware_control_allowed",
        "openpi_commit",
        *IDENTITY_HASH_FIELDS,
    }
    if not isinstance(lock, dict) or set(lock) != required:
        raise ValueError("artifact identity lock has missing or unexpected fields")
    if lock["schema"] != ARTIFACT_LOCK_SCHEMA:
        raise ValueError("unsupported artifact identity lock schema")
    if lock["artifact_identity_schema"] != ARTIFACT_IDENTITY_SCHEMA:
        raise ValueError("artifact identity schema mismatch")
    if lock["config_schema"] != "pi05_proxy_shadow_v1":
        raise ValueError("artifact lock config schema mismatch")
    if lock["action_semantics"] != "next_observed_state_proxy_nominal_units":
        raise ValueError("artifact lock action semantics mismatch")
    if lock["hardware_control_allowed"] is not False:
        raise PermissionError("artifact identity lock cannot grant hardware control")
    for field in ("checkpoint", "dataset", "normalization_identity"):
        if not isinstance(lock[field], str) or not lock[field].strip():
            raise ValueError(f"artifact identity lock {field} is missing")
    if not isinstance(lock["openpi_commit"], str) or not _COMMIT_RE.fullmatch(lock["openpi_commit"]):
        raise ValueError("artifact identity lock has invalid OpenPI commit")
    for field in IDENTITY_HASH_FIELDS:
        if not isinstance(lock[field], str) or not _SHA256_RE.fullmatch(lock[field]):
            raise ValueError(f"artifact identity lock {field} is not a lowercase SHA-256 digest")
    return dict(lock)


def load_artifact_identity_lock(path: Path) -> dict:
    """Load the expected content hashes from the local runtime lock file."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return validate_artifact_identity_lock(data)


def _validate_lerobot_pi05_identity(identity: dict, *, lock: bool) -> None:
    """Validate the separate synthetic LeRobot preview contract and its pins."""
    prefix = "LeRobot PI05 identity lock" if lock else "LeRobot PI05 policy metadata"
    expected = {
        "config_schema": LEROBOT_PI05_CONFIG_SCHEMA,
        "backend": LEROBOT_PI05_BACKEND,
        "action_semantics": LEROBOT_PI05_ACTION_SEMANTICS,
        "action_horizon": 16,
        "action_dimension": 7,
        "first_target_dt_s": 0.1,
        "source_kind": "synthetic",
        "training_eligible": False,
        "software_only": True,
        "prediction_only": True,
        "control_enabled": False,
        "robot_control_ready": False,
        "hardware_control_allowed": False,
        "can_opened": False,
        "can_data_tx_calls": 0,
    }
    for field, value in expected.items():
        if identity.get(field) != value:
            raise PermissionError(f"{prefix} {field} does not match prediction-only synthetic contract")
    for field in ("checkpoint", "dataset", "base_model", "base_revision"):
        if not isinstance(identity.get(field), str) or not identity[field].strip():
            raise ValueError(f"{prefix} {field} is missing")
    if identity["base_model"] != "lerobot/pi05_base":
        raise PermissionError(f"{prefix} base_model is not the supported PI05 base")
    if not _COMMIT_RE.fullmatch(identity["base_revision"]):
        raise ValueError(f"{prefix} base_revision must be a pinned commit SHA")
    step = identity.get("training_step")
    if isinstance(step, bool) or not isinstance(step, int) or step < 0:
        raise ValueError(f"{prefix} training_step must be a nonnegative integer")
    hash_fields = (
        "checkpoint_sha256", "checkpoint_identity_sha256", "dataset_manifest_sha256",
        "processor_identity_sha256", "normalization_sha256",
    )
    for field in hash_fields:
        if not isinstance(identity.get(field), str) or not _SHA256_RE.fullmatch(identity[field]):
            raise ValueError(f"{prefix} {field} is not a lowercase SHA-256 digest")


def validate_policy_metadata(metadata: dict, expected_identity_lock: dict | None = None,
                             *, backend: str = "openpi") -> None:
    """Require complete content identities before opening any hardware reads."""
    if not isinstance(metadata, dict):
        raise ValueError("policy server metadata must be an object")
    if backend == LEROBOT_PI05_BACKEND:
        _validate_lerobot_pi05_identity(metadata, lock=False)
        if expected_identity_lock is None:
            raise PermissionError("LeRobot PI05 shadow requires a local identity lock")
        lock = validate_artifact_identity_lock(expected_identity_lock)
        if lock.get("schema") != LEROBOT_PI05_LOCK_SCHEMA:
            raise PermissionError("LeRobot PI05 shadow requires its dedicated identity lock")
        compared = set(lock) - {"schema"}
        mismatched = sorted(field for field in compared if metadata.get(field) != lock[field])
        if mismatched:
            raise PermissionError("LeRobot PI05 service identity differs from local lock: "
                                  + ", ".join(mismatched))
        return
    if backend != "openpi":
        raise ValueError(f"unsupported policy backend: {backend}")
    if metadata.get("backend") == LEROBOT_PI05_BACKEND:
        raise PermissionError("LeRobot PI05 metadata requires explicit backend selection")
    if metadata.get("config_schema") == "pi05_derived_state_multi_episode_shadow_v1":
        expected_values = {
            "artifact_identity_schema": DERIVED_MULTI_ARTIFACT_SCHEMA,
            "prediction_only": True,
            "control_enabled": False,
            "robot_control_ready": False,
            "hardware_control_allowed": False,
            "can_opened": False,
            "can_data_tx_calls": 0,
            "action_semantics": "next_observed_state_proxy_nominal_units",
            "action_target_kind": "derived_future_observed_state_position_target",
            "action_horizon": 16,
            "action_dimension": 7,
            "first_target_dt_s": 0.1,
        }
        for field, expected in expected_values.items():
            if metadata.get(field) != expected:
                raise PermissionError(f"policy metadata {field} does not match derived-state shadow contract")
        if metadata.get("dataset_identity") != metadata.get("dataset"):
            raise ValueError("policy metadata dataset identity mismatch")
        for field in ("checkpoint", "dataset", "normalization_identity"):
            if not isinstance(metadata.get(field), str) or not metadata[field].strip():
                raise ValueError(f"policy metadata {field} is missing")
        if not isinstance(metadata.get("openpi_commit"), str) or not _COMMIT_RE.fullmatch(metadata["openpi_commit"]):
            raise ValueError("policy metadata has invalid OpenPI commit")
        for field in ("checkpoint_tree_sha256", "dataset_sha256", "normalization_sha256",
                      "training_provenance_sha256"):
            if not isinstance(metadata.get(field), str) or not _SHA256_RE.fullmatch(metadata[field]):
                raise ValueError(f"policy metadata {field} is not a lowercase SHA-256 digest")
        if expected_identity_lock is not None:
            lock = validate_artifact_identity_lock(expected_identity_lock)
            compared_fields = (
                "artifact_identity_schema", "config_schema", "checkpoint",
                "checkpoint_tree_sha256", "dataset", "dataset_sha256",
                "normalization_identity", "normalization_sha256", "action_semantics",
                "action_target_kind", "action_horizon", "action_dimension",
                "first_target_dt_s", "hardware_control_allowed", "prediction_only",
                "openpi_commit", "training_provenance_sha256",
            )
            mismatched = [field for field in compared_fields
                          if metadata.get(field) != lock.get(field)]
            if mismatched:
                raise PermissionError(
                    "policy artifact identity differs from local lock: "
                    + ", ".join(mismatched)
                )
        return

    expected_values = {
        "artifact_identity_schema": ARTIFACT_IDENTITY_SCHEMA,
        "prediction_only": True,
        "control_enabled": False,
        "robot_control_ready": False,
        "hardware_control_allowed": False,
        "action_semantics": "next_observed_state_proxy_nominal_units",
        "config_schema": "pi05_proxy_shadow_v1",
        "action_horizon": 16,
    }
    for field, expected in expected_values.items():
        if metadata.get(field) != expected:
            raise PermissionError(f"policy metadata {field} does not match proxy contract")
    first_target_dt_s = metadata.get("first_target_dt_s")
    if (isinstance(first_target_dt_s, bool)
            or not isinstance(first_target_dt_s, (int, float))
            or not math.isfinite(first_target_dt_s)
            or first_target_dt_s != 0.5):
        raise ValueError("policy metadata has invalid proxy target timing")
    for field in ("checkpoint", "normalization_identity", "dataset"):
        if not isinstance(metadata.get(field), str) or not metadata[field].strip():
            raise ValueError(f"policy metadata {field} is missing")
    for field in IDENTITY_HASH_FIELDS:
        if not isinstance(metadata.get(field), str) or not _SHA256_RE.fullmatch(metadata[field]):
            raise ValueError(f"policy metadata {field} is not a lowercase SHA-256 digest")
    if not isinstance(metadata.get("openpi_commit"), str) or not _COMMIT_RE.fullmatch(metadata["openpi_commit"]):
        raise ValueError("policy metadata has invalid OpenPI commit")
    if expected_identity_lock is not None:
        lock = validate_artifact_identity_lock(expected_identity_lock)
        compared_fields = (
            "artifact_identity_schema",
            "config_schema",
            "checkpoint",
            "dataset",
            "normalization_identity",
            "action_semantics",
            "hardware_control_allowed",
            "openpi_commit",
            *IDENTITY_HASH_FIELDS,
        )
        mismatched = [field for field in compared_fields
                      if metadata.get(field) != lock.get(field)]
        if mismatched:
            raise PermissionError(
                "policy artifact identity differs from local lock: "
                + ", ".join(mismatched)
            )


def prewarm_policy(client: object) -> dict:
    """Warm the remote model before opening cameras or CAN."""

    observation = make_piper_proxy_warmup_observation()
    validate_piper_semantic_observation(observation)
    started_ns = time.perf_counter_ns()
    result = client.infer(observation)
    finished_ns = time.perf_counter_ns()
    actions = np.asarray(result.get("actions"))
    if actions.shape != (16, 7) or not np.isfinite(actions).all():
        raise ValueError("invalid policy prewarm response")
    return {
        "status": "PASS",
        "hardware_observation_used": False,
        "latency_ms": (finished_ns - started_ns) / 1e6,
        "action_shape": list(actions.shape),
    }
