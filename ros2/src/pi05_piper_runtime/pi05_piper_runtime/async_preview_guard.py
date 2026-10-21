"""Shared fail-closed guards for proxy previews, with no hardware dependencies."""
import math
from .async_preview_core import PERIOD_NS, original_times

CLOCK = 'wsl_perf_counter_ns'
SEMANTICS = 'next_observed_state_proxy_nominal_units'


def state_reason(state, now):
    if (state is None or state.fake or not state.hardware_connected
            or not state.feedback_fresh or state.error_code or not state.session_id):
        return 'state_invalid'
    if state.source_local_clock_id != CLOCK:
        return 'state_clock'
    if not 0 <= now - state.source_local_monotonic_ns <= 120_000_000:
        return 'state_stale'
    return 'ok'


class ChunkGuard:
    """Pin identity for the lifetime of a preview run, including reconnects."""
    def __init__(self):
        self.identity = None
        self.session = None
        self.previous_obs = 0

    def accept(self, chunk, state, now):
        reason = state_reason(state, now)
        if reason != 'ok':
            return reason
        if (not chunk.prediction_only or chunk.hardware_control_allowed
                or chunk.fake_observation or not chunk.hardware_connected):
            return 'chunk_flags'
        if chunk.rows != 16 or chunk.columns != 7 or len(chunk.target_position) != 112:
            return 'chunk_shape'
        if not all(math.isfinite(x) for x in chunk.target_position):
            return 'chunk_nonfinite'
        if not chunk.observation_id.startswith(state.session_id + ':'):
            return 'chunk_session'
        if chunk.local_clock_id != CLOCK or chunk.action_semantics != SEMANTICS:
            return 'chunk_contract'
        if not 0 <= now - chunk.response_local_monotonic_ns <= 250_000_000:
            return 'response_stale'
        if not 0 <= now - chunk.observation_local_monotonic_ns <= 500_000_000:
            return 'observation_stale'
        if (not 0 <= chunk.inference_latency_ns <= 500_000_000
                or chunk.response_local_monotonic_ns < chunk.observation_local_monotonic_ns):
            return 'inference_stale'
        identity = (chunk.model_id, chunk.checkpoint_id)
        if not all(identity) or (self.identity is not None and identity != self.identity):
            return 'chunk_identity'
        if tuple(chunk.target_local_monotonic_ns) != original_times(chunk.observation_local_monotonic_ns):
            return 'chunk_schedule'
        previous = self.previous_obs if self.session == state.session_id else 0
        if chunk.observation_local_monotonic_ns <= previous:
            return 'chunk_order'
        self.identity = identity
        self.session = state.session_id
        self.previous_obs = chunk.observation_local_monotonic_ns
        return 'ok'


class PreviewGuard:
    """Reject late, duplicate, foreign-session, or unsafe received previews.

    This is a preview consumer, not an actuator gate or command conversion.
    """
    def __init__(self):
        self.identity = None
        self.last_target = 0
        self.seen = set()

    def accept(self, preview, state, now):
        reason = state_reason(state, now)
        if reason != 'ok':
            return reason
        if (not preview.prediction_only or preview.hardware_control_allowed
                or preview.emitted_to_hardware or preview.speed_percent != 0):
            return 'preview_flags'
        if preview.local_clock_id != CLOCK or preview.action_semantics != SEMANTICS:
            return 'preview_contract'
        if not preview.source_observation_id.startswith(state.session_id + ':'):
            return 'preview_session'
        if not 0 <= preview.row_index < 16 or len(preview.joint_position_rad) != 6:
            return 'preview_shape'
        if not all(math.isfinite(x) for x in (*preview.joint_position_rad, preview.gripper_nominal_m)):
            return 'preview_nonfinite'
        target = preview.target_local_monotonic_ns
        if preview.target_expiry_monotonic_ns != target + PERIOD_NS:
            return 'preview_schedule'
        if now < target:
            return 'preview_early'
        if now >= preview.target_expiry_monotonic_ns:
            return 'preview_expired'
        # The original row schedule identifies observation time. Reconstruct
        # publish time from its age and charge transport time to every budget.
        observation = target - (preview.row_index + 1) * PERIOD_NS
        sent = observation + preview.observation_age_ns
        transport = now - sent
        if transport < 0:
            return 'preview_future_publish'
        for name, age, budget in (
                ('state', preview.state_age_ns, 120_000_000),
                ('observation', preview.observation_age_ns, 500_000_000),
                ('prediction', preview.prediction_age_ns, 250_000_000),
                ('chunk', preview.chunk_age_ns, 250_000_000)):
            if age < 0 or age + transport > budget:
                return 'preview_' + name + '_stale'
        if not 0 <= preview.inference_latency_ns <= 500_000_000:
            return 'preview_inference_stale'
        identity = (preview.model_id, preview.checkpoint_id)
        if not all(identity) or (self.identity is not None and identity != self.identity):
            return 'preview_identity'
        key = (preview.source_observation_id, preview.row_index)
        if key in self.seen or target <= self.last_target:
            return 'preview_replay'
        # Bound memory: a strictly increasing target already blocks older replays.
        self.seen = {key}
        self.last_target = target
        self.identity = identity
        return 'ok'
