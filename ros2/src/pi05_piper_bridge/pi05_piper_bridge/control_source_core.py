"""Pure validation helpers for ROS target-to-observation binding."""

from laptop.ros_gateway.control_rpc import validate_control_source_binding


def validate_bridge_control_source(
    *,
    recent_state_sources,
    source_session_id: str,
    source_state_sequence: int,
    active_session_id: str,
    latest_state_sequence: int,
    latest_state_monotonic_ns: int,
    latest_mapping_uncertainty_ns: int,
    latest_state_feedback_fresh: bool,
    now_monotonic_ns: int,
) -> None:
    """Validate a ROS request against the latest state snapshot, not a heartbeat.

    ``recent_state_sources`` contains same-session observation metadata retained by
    the bridge. Gateway status messages are intentionally absent: they report a
    separate, periodic health view and can arrive between a fresh snapshot and a
    target request. Freshness of both the requested observation and the latest
    actual state is still checked by the shared RPC validator.
    """
    requested = next(
        (item for item in reversed(recent_state_sources)
         if item[1] == source_state_sequence),
        None,
    )
    if requested is None:
        raise ValueError("requested source sequence is not in the same-session recent state cache")

    source_session, _sequence, source_ns, source_uncertainty_ns, source_fresh = requested
    validate_control_source_binding(
        source_session_id=source_session_id,
        source_state_sequence=source_state_sequence,
        active_session_id=active_session_id,
        latest_state_sequence=latest_state_sequence,
        latest_state_monotonic_ns=latest_state_monotonic_ns,
        mapping_uncertainty_ns=latest_mapping_uncertainty_ns,
        source_state_monotonic_ns=source_ns,
        source_mapping_uncertainty_ns=source_uncertainty_ns,
        source_feedback_fresh=(source_fresh is True and source_session == active_session_id),
        feedback_fresh=latest_state_feedback_fresh,
        now_monotonic_ns=now_monotonic_ns,
    )
