"""Strict runtime configuration for laptop receive, query, and shadow modes."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from laptop.runtime.modes import RuntimeMode, require_offline_mode


@dataclass(frozen=True)
class RuntimeConfig:
    schema: str
    mode: RuntimeMode
    policy_config: str
    can_tx_enabled: bool
    single_can_owner: str
    max_state_age_ms: int
    max_prediction_age_ms: int
    notes: str
    command_tx_enabled: bool = False
    supervised_position_tx_enabled: bool = False
    supervised_j1_commissioning_tx_enabled: bool = False
    supervised_manual_gamepad_tx_enabled: bool = False

    @classmethod
    def load(cls, path: Path) -> "RuntimeConfig":
        data = json.loads(path.read_text(encoding="utf-8"))
        allowed = set(cls.__dataclass_fields__)
        required = allowed - {
            "command_tx_enabled", "supervised_position_tx_enabled",
            "supervised_j1_commissioning_tx_enabled", "supervised_manual_gamepad_tx_enabled",
        }
        missing = sorted(required - set(data))
        extra = sorted(set(data) - allowed)
        if missing or extra:
            raise ValueError(f"runtime config keys mismatch: missing={missing}, extra={extra}")
        data.setdefault("command_tx_enabled", False)
        data.setdefault("supervised_position_tx_enabled", False)
        data.setdefault("supervised_j1_commissioning_tx_enabled", False)
        data.setdefault("supervised_manual_gamepad_tx_enabled", False)
        data["mode"] = RuntimeMode(data["mode"])
        config = cls(**data)
        config.validate()
        return config

    def validate(self) -> None:
        if self.schema != "piper_runtime_v1":
            raise ValueError("unsupported runtime schema")
        if (type(self.can_tx_enabled) is not bool
                or type(self.command_tx_enabled) is not bool
                or type(self.supervised_position_tx_enabled) is not bool
                or type(self.supervised_j1_commissioning_tx_enabled) is not bool):
            raise ValueError("CAN transmit fields must be booleans")
        if type(self.supervised_manual_gamepad_tx_enabled) is not bool:
            raise ValueError("manual gamepad transmit field must be a boolean")
        if self.mode is RuntimeMode.DIAGNOSTIC_QUERY:
            if (not self.can_tx_enabled or self.command_tx_enabled
                    or self.supervised_position_tx_enabled
                    or self.supervised_manual_gamepad_tx_enabled):
                raise PermissionError(
                    "diagnostic mode permits only the allowlisted query-frame channel"
                )
        elif self.mode is RuntimeMode.SUPERVISED_POLICY_STEP:
            if (not self.supervised_position_tx_enabled
                    or self.can_tx_enabled or self.command_tx_enabled
                    or self.supervised_j1_commissioning_tx_enabled
                    or self.supervised_manual_gamepad_tx_enabled):
                raise PermissionError(
                    "supervised mode requires only its dedicated position-TX gate"
                )
        elif self.mode is RuntimeMode.SUPERVISED_J1_COMMISSIONING:
            if (not self.supervised_j1_commissioning_tx_enabled
                    or self.can_tx_enabled or self.command_tx_enabled
                    or self.supervised_position_tx_enabled
                    or self.supervised_manual_gamepad_tx_enabled):
                raise PermissionError(
                    "J1 commissioning requires only its dedicated one-shot TX gate"
                )
        elif self.mode is RuntimeMode.SUPERVISED_MANUAL_GAMEPAD:
            if (not self.supervised_manual_gamepad_tx_enabled
                    or self.can_tx_enabled or self.command_tx_enabled
                    or self.supervised_position_tx_enabled
                    or self.supervised_j1_commissioning_tx_enabled):
                raise PermissionError(
                    "manual gamepad control requires only its dedicated operator TX gate"
                )
        else:
            require_offline_mode(self.mode)
            if (self.can_tx_enabled or self.command_tx_enabled
                    or self.supervised_position_tx_enabled
                    or self.supervised_j1_commissioning_tx_enabled
                    or self.supervised_manual_gamepad_tx_enabled):
                raise PermissionError("CAN transmit is disabled in this runtime mode")
        if self.single_can_owner != "safety-bridge":
            raise ValueError("SafetyBridge must be the sole declared CAN owner")
        if self.max_state_age_ms <= 0 or self.max_prediction_age_ms <= 0:
            raise ValueError("freshness limits must be positive")
        if not self.policy_config.strip():
            raise ValueError("policy_config is required")
