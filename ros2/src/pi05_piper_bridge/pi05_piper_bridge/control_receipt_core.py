"""Validate gateway delivery receipts without ROS or hardware access."""


ADAPTER_ACCEPTED_DECISIONS = frozenset({
    "ADAPTER_ACCEPTED_POSITION_BATCH",
    "ADAPTER_ACCEPTED_MANUAL_RAW_BATCH",
    "ADAPTER_ACCEPTED_MANUAL_GAMEPAD_BATCH",
    "ADAPTER_ACCEPTED_P2_MANUAL_RAW_BATCH",
})


def valid_control_receipt(payload: dict) -> bool:
    """Accept known delivery states; adapter acceptance is not motion proof."""
    count = payload.get("can_tx_calls")
    accepted = payload.get("accepted")
    delivery = payload.get("delivery_status", "NOT_DISPATCHED")
    decision = payload.get("decision")
    if accepted is False and type(count) is int and count == 0:
        return True
    if accepted is False and count is None and delivery == "UNKNOWN":
        return True
    return (
        accepted is True
        and type(count) is int
        and count in (4, 5)
        and delivery == "ADAPTER_ACCEPTED"
        and isinstance(decision, str)
        and decision in ADAPTER_ACCEPTED_DECISIONS
    )
