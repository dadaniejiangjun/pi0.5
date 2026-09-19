# Remote architecture

## Authority boundary

```text
Server: policy and training authority
Laptop: robot and safety authority
```

The server may receive an observation and return a policy response. It may not
enable the robot, reset it, stop it, access CAN, or bypass the laptop safety
bridge. This remains true even if the laptop is running a ROS or SDK wrapper.

## Components

### Laptop

- `laptop/camera`: identifies and captures the base and wrist cameras;
- `laptop/recorder`: writes raw episodes locally using one monotonic clock;
- `laptop/robot_client`: constructs observations and records telemetry;
- `laptop/piper`: read-only feedback and later validated command adapter;
- `laptop/safety`: final `PiperSafetyBridge` veto and watchdog;
- `laptop/diagnostics`: hardware, camera and network probes.

### Server

- `server/policy_server`: official OpenPI WebSocket service plus project health
  and metadata checks;
- `server/training`: norm stats, transforms, tiny overfit and fine-tuning;
- `server/dataset_tools`: checksum verification, raw validation and LeRobot
  conversion;
- `datasets/raw`: master canonical raw episodes;
- `datasets/lerobot`: derived training dataset;
- `checkpoints`: server-only frozen weights;
- `reports`: audit, evaluation and latency evidence.

## Message flow

### Request

The laptop constructs:

```json
{
  "episode_id": "episode_000001",
  "request_id": "uuid-or-monotonic-sequence",
  "capture_monotonic_ns": 0,
  "observation": {
    "base_rgb": "uint8 model-sized frame",
    "wrist_rgb": "uint8 model-sized frame",
    "state": "STATE_7, units frozen by audit",
    "prompt": "canonical task instruction"
  },
  "prompt_hash": "sha256",
  "state_hash": "sha256"
}
```

The exact keys are frozen only after the official OpenPI transform audit. The
project adapter may repack this schema to the official client keys, but it must
record the mapping and its hash.

### Response

The laptop accepts only a response that contains:

- the matching `request_id`;
- the frozen checkpoint ID and model-config hash;
- finite actions with the expected shape;
- server inference timing;
- no newer observation that makes this response stale.

Any failure produces a reject event and no new robot command.

## Process and failure domains

The recorder, policy client, piper feedback adapter and safety bridge are
separate failure domains. The safety bridge must not be killed by a policy
server OOM or a recorder disk-full condition. The robot-facing process must
fail closed when the server or camera disappears.

The server policy process is separate from training. A training OOM or cache
cleanup must not restart or mutate the live policy checkpoint. A checkpoint is
immutable after its hash is published into rollout metadata.

## Startup order for a later phase

1. Verify hardware and camera audit files are current.
2. Verify server checkpoint/config/norm metadata hashes.
3. Start the private transport (NET-A/B/C, user-selected).
4. Start the policy server and project health endpoint.
5. Laptop checks health and metadata compatibility.
6. Run dummy, then real-observation shadow requests.
7. Only a later execution phase may instantiate a command-capable piper path.

This order is a plan only; it is not executed in the current turn.
