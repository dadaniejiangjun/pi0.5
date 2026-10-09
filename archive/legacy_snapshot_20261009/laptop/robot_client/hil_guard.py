"""Fail-closed SafetyBridge audit gate for legacy HIL senders."""

from __future__ import annotations

import json

from laptop.safety.bridge import SafetyBridge


class LegacyHILBlockedError(PermissionError):
    """HIL rejection carrying the structured SafetyBridge audit event."""

    def __init__(self, message: str, audit_event: dict) -> None:
        self.audit_event = audit_event
        serialized = json.dumps(audit_event, sort_keys=True, separators=(",", ":"))
        super().__init__(f"{message}; audit_event={serialized}")


def reject_legacy_raw_can_tx(script_name: str) -> None:
    """Audit and block a legacy HIL entrypoint before any adapter access.

    SafetyBridge records the unavailable HIL mode and explicit blockers without
    constructing a transport. There is no override flag: a confirmation cannot
    authorize a direct raw-CAN call that bypasses the bridge.
    """
    if not isinstance(script_name, str) or not script_name.strip():
        raise ValueError("script_name is required")
    decision = SafetyBridge.reject_unimplemented_hil_entrypoint(script_name)
    raise LegacyHILBlockedError(
        f"{script_name}: direct gs_usb CAN TX is disabled; "
        "the SafetyBridge HIL_SINGLE_STEP send path is not implemented",
        decision.audit_event(),
    )
