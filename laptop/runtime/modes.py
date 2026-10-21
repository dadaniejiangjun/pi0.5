"""Explicit runtime modes; the default is always non-transmitting shadow."""

from enum import Enum


class RuntimeMode(str, Enum):
    SHADOW = "SHADOW"
    DIAGNOSTIC_QUERY = "DIAGNOSTIC_QUERY"
    HIL_SINGLE_STEP = "HIL_SINGLE_STEP"
    FAKE_CAN_REHEARSAL = "FAKE_CAN_REHEARSAL"
    POLICY_REJECT_ONLY = "POLICY_REJECT_ONLY"
    SUPERVISED_POLICY_STEP = "SUPERVISED_POLICY_STEP"
    SUPERVISED_J1_COMMISSIONING = "SUPERVISED_J1_COMMISSIONING"
    SUPERVISED_MANUAL_GAMEPAD = "SUPERVISED_MANUAL_GAMEPAD"
    SUPERVISED_MODE_INITIALIZATION = "SUPERVISED_MODE_INITIALIZATION"
    AUTONOMOUS = "AUTONOMOUS"


DEFAULT_RUNTIME_MODE = RuntimeMode.SHADOW
OFFLINE_ONLY_MODES = frozenset(
    {RuntimeMode.SHADOW, RuntimeMode.POLICY_REJECT_ONLY}
)


def require_offline_mode(mode: RuntimeMode) -> None:
    if mode not in OFFLINE_ONLY_MODES:
        raise PermissionError(
            f"runtime mode {mode.value} is not implemented or authorized in this build"
        )
