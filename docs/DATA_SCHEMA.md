# Data schema

## Raw episode

```text
episode_000001/
  metadata.json
  instruction.txt
  robot.parquet | robot.npz
  base.mp4
  wrist.mp4
  events.jsonl
  manifest.json
```

### `metadata.json`

Required fields:

```json
{
  "episode_id": "episode_000001",
  "robot_model": "AgileX PiPER",
  "piper_firmware": "verified string",
  "sdk_name": "verified string",
  "sdk_version": "verified string",
  "protocol_version": "verified string",
  "camera_serials": {"base": "...", "wrist": "..."},
  "camera_fps": {"base": 0, "wrist": 0},
  "task": "Pick up the block and place it in the tray.",
  "operator": "operator id",
  "start_monotonic_ns": 0,
  "end_monotonic_ns": 0,
  "success": null,
  "notes": ""
}
```

Values are placeholders until P2/P4. No old project hardware value is copied.

### Robot rows

Each row contains:

| Field | Meaning |
|---|---|
| `episode_id` | immutable episode identity |
| `frame_id` | monotonic row index |
| `monotonic_ns` | laptop time authority |
| `base_capture_ts` | base frame capture time |
| `wrist_capture_ts` | wrist frame capture time |
| `joint_sample_ts` | joint feedback sample time |
| `gripper_sample_ts` | gripper feedback sample time |
| `q1..q6` | six joint values, units frozen by audit |
| `gripper` | gripper value, unit frozen by audit |
| `tcp_pose` | optional diagnostic only |
| `joint_velocity` | optional diagnostic only |
| `arm_status` | raw status/event code |
| `teach_status` | raw teach status/event code |

## Events

`events.jsonl` records episode start/stop, operator teach entry/exit, pause,
continue, invalid/dropout, gripper target changes, gripper actual changes,
manual notes, checksum completion and upload state. Each event uses the laptop
monotonic clock and an explicit source.

## Manifest

`manifest.json` records schema version, every file's size and SHA256, frame
counts, video metadata, timestamp monotonicity, dropout counts, skew
statistics, validation status and upload status. `READY_FOR_UPLOAD` requires
all required files closed and checksummed.

## LeRobot mapping

The target derived dataset uses the current LeRobot convention, subject to the
pinned version audit:

```text
observation.images.image        ← base RGB
observation.images.wrist_image  ← wrist RGB
observation.state                ← STATE_7
action                          ← ACTION_7
task                            ← canonical language instruction
```

The raw dataset remains the source of truth. A LeRobot v3 conversion should
preserve metadata/stats/tasks/episode boundaries and store tabular state/action
in Parquet with camera videos in MP4. Exact version and path templates are
recorded during P7 rather than assumed from memory.

## Split and normalization

- split at episode level only;
- recommended initial train/validation ratio is 80/20;
- test is a future frozen holdout;
- no demonstration frames may cross splits;
- compute PiPER-specific state and action statistics;
- record min/max/mean/std, q01/q99, percentiles and outliers;
- do not silently reuse DROID statistics.

## Model-input audit

For every derived frame, retain enough provenance to answer:

```text
which raw base/wrist frame → which model input → which state sample → which action target?
```

The resize/pad policy, input dtype, channel order, camera mapping and padding
mask are versioned and hashed. This mapping is tested before training.
