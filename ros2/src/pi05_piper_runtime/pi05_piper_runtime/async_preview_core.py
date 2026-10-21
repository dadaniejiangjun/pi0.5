"""Pure timing selection for proxy previews; no command or hardware access."""
PERIOD_NS = 100_000_000

def due_row(times, now, observation_id, consumed):
    for index, target in enumerate(times):
        if target <= now < target + PERIOD_NS and (observation_id, index) not in consumed:
            return index
    return None

def original_times(observation_ns, count=16):
    return tuple(observation_ns + (i+1)*PERIOD_NS for i in range(count))
