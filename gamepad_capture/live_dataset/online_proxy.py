"""One-observation buffer for measured next-state labels; no hardware access."""
from copy import deepcopy
import math

SEMANTICS = 'next_observed_state_proxy_nominal_units'


class OnlineNextState:
    def __init__(self):
        self.pending = None
        self.received = 0
        self.segment_id = 0

    def push(self, sample):
        current = dict(sample)
        current['source_map'] = deepcopy(sample['source_map'])
        previous = self.pending
        self.pending = current
        self.received += 1
        if previous is None:
            return None
        source = previous['source_map']['state']
        future = current['source_map']['state']
        start = source.get('source_local_monotonic_ns')
        candidates = previous['source_map'].pop('future_state_candidates', [])
        candidates = [candidate for candidate in candidates
                      if candidate.get('feedback_fresh') is True
                      and candidate.get('hardware_connected') is True
                      and candidate.get('fake') is False
                      and all(source.get(key) and source.get(key) == candidate.get(key)
                              for key in ('session_id', 'source_local_clock_id'))
                      and type(start) is int
                      and type(candidate.get('source_local_monotonic_ns')) is int
                      and 75_000_000 <= candidate['source_local_monotonic_ns'] - start <= 125_000_000]
        if candidates:
            future = min(candidates, key=lambda candidate: (
                abs(candidate['source_local_monotonic_ns'] - start - 100_000_000),
                candidate['source_local_monotonic_ns']))
        end = future.get('source_local_monotonic_ns')
        delta = end - start if type(start) is int and type(end) is int else None
        valid_time = delta is not None and 75_000_000 <= delta <= 125_000_000
        identity = all(source.get(key) and source.get(key) == future.get(key)
                       for key in ('session_id', 'source_local_clock_id'))
        values = list(future['values_float32'] if candidates else current['state'])
        finite = len(values) == 7 and all(math.isfinite(float(x)) for x in values)
        valid = bool(valid_time and identity and finite
                     and previous['observation_valid'] and all(previous['camera_valid'])
                     and (bool(candidates) or (current['observation_valid'] and all(current['camera_valid']))))
        previous['action'] = values if valid else [0.0] * 7
        previous['action_valid'] = valid
        # Preserve the original send/receipt evidence under its original key.
        previous['source_map']['proxy_target'] = {
            'semantics': SEMANTICS, 'action_label_valid': valid,
            'target_offset_ns': delta, 'expected_offset_ns': 100_000_000,
            'offset_tolerance_ns': 25_000_000,
            'state': deepcopy(future), 'values_float32': values,
            'invalid_reason': None if valid else 'future_state_time_identity_or_observation_invalid',
            'selection': 'nearest_measured_feedback_to_100ms' if candidates else 'next_observation_fallback',
            'trajectory_segment_id': self.segment_id,
            'continuity_break': not valid,
        }
        if not valid:
            self.segment_id += 1
        return previous

    def finish(self):
        """The pending final observation is target-only, never a copied action."""
        last = self.pending
        self.pending = None
        return {'source_observations': self.received, 'final_observation':
                deepcopy(last['source_map']['state']) if last else None,
                'final_observation_written_as_action_row': False}
