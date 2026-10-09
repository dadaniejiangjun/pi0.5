"""Strict loader for the prediction-only PiPER proxy server configuration."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re


EXPECTED_SCHEMA = "pi05_proxy_shadow_v1"
EXPECTED_SEMANTICS = "next_observed_state_proxy_nominal_units"
DERIVED_STATE_SCHEMA = "pi05_derived_state_10fps_shadow_v1"
DERIVED_STATE_SEMANTICS = "derived_future_observed_state_position_target_nominal_units"
DERIVED_STATE_DATASET = "local/cef166e2_proxy10fps_ds36"
_OPTIONAL_IDENTITY_FIELDS = {
    "expected_dataset_tree_sha256",
    "expected_readback_audit_sha256",
    "expected_checkpoint_sha256",
    "expected_normalization_sha256",
    "expected_openpi_commit",
}
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


@dataclass(frozen=True)
class ProxyPolicyConfig:
    schema: str
    experiment_name: str
    checkpoint_step: int
    comparison_experiment_name: str
    comparison_checkpoint_step: int
    dataset_identity: str
    normalization_identity: str
    action_semantics: str
    action_horizon: int
    action_dimension: int
    source_fps: float
    first_target_dt_s: float
    server_host: str
    server_port: int
    sampling_steps: int
    prediction_only: bool
    hardware_control_allowed: bool
    expected_dataset_tree_sha256: str | None = None
    expected_readback_audit_sha256: str | None = None
    expected_checkpoint_sha256: str | None = None
    expected_normalization_sha256: str | None = None
    expected_openpi_commit: str | None = None

    @classmethod
    def load(cls, path: Path) -> "ProxyPolicyConfig":
        data = json.loads(path.read_text(encoding="utf-8"))
        all_fields = set(cls.__dataclass_fields__)
        required = all_fields - _OPTIONAL_IDENTITY_FIELDS
        missing = sorted(required - set(data))
        extra = sorted(set(data) - all_fields)
        optional_present = set(data) & _OPTIONAL_IDENTITY_FIELDS
        if missing or extra or (optional_present and optional_present != _OPTIONAL_IDENTITY_FIELDS):
            raise ValueError(f"config keys mismatch: missing={missing}, extra={extra}")
        config = cls(**data)
        config.validate()
        return config

    def validate(self) -> None:
        if self.schema not in {EXPECTED_SCHEMA, DERIVED_STATE_SCHEMA}:
            raise ValueError(f"unsupported schema: {self.schema}")
        if not self.prediction_only:
            raise ValueError("proxy server must remain prediction-only")
        if self.hardware_control_allowed:
            raise ValueError("proxy server cannot grant hardware control")
        if not self.experiment_name or not self.dataset_identity:
            raise ValueError("experiment and dataset identities are required")
        if not self.normalization_identity:
            raise ValueError("normalization identity is required")
        if self.action_horizon != 16 or self.action_dimension != 7:
            raise ValueError("the frozen proxy contract is a 16 x 7 action chunk")
        if self.schema == EXPECTED_SCHEMA:
            if (self.action_semantics != EXPECTED_SEMANTICS
                    or self.source_fps != 2.0 or self.first_target_dt_s != 0.5
                    or self.dataset_identity != "local/pi05_proxy_cef166e2_2fps"
                    or any(getattr(self, field) is not None
                           for field in _OPTIONAL_IDENTITY_FIELDS)):
                raise ValueError("the legacy proxy contract is frozen at its 2 fps identity")
        else:
            if (self.action_semantics != DERIVED_STATE_SEMANTICS
                    or self.source_fps != 10.0 or self.first_target_dt_s != 0.1
                    or self.dataset_identity != DERIVED_STATE_DATASET):
                raise ValueError("derived-state shadow must use the isolated 10 fps dataset contract")
            for field in _OPTIONAL_IDENTITY_FIELDS:
                value = getattr(self, field)
                if field == "expected_openpi_commit":
                    if not isinstance(value, str) or not _COMMIT_RE.fullmatch(value):
                        raise ValueError("derived-state shadow requires a pinned expected_openpi_commit")
                    continue
                if not isinstance(value, str) or not _SHA256_RE.fullmatch(value):
                    raise ValueError(f"derived-state shadow requires a pinned {field}")
        if not 1 <= self.checkpoint_step <= 10:
            raise ValueError("checkpoint_step must be within the bounded tiny-fit range")
        if not 1 <= self.comparison_checkpoint_step <= 10:
            raise ValueError("comparison checkpoint step is out of range")
        if not 1 <= self.sampling_steps <= 10:
            raise ValueError("sampling_steps must be in [1, 10]")
        if self.server_host not in {"127.0.0.1", "localhost"}:
            raise ValueError("prediction-only server must bind to localhost")
        if not 1 <= self.server_port <= 65535:
            raise ValueError("server_port is invalid")

    @property
    def dataset_variant(self) -> str:
        """Select only the schema-bound dataset route, never by an arbitrary CLI override."""
        return "10fps" if self.schema == DERIVED_STATE_SCHEMA else "2fps"
