---
Document status: REFERENCE
Current authority: docs/ACTION_SEMANTICS.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../ACTION_SEMANTICS.md)。

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
  "task": "Pick up the black rod and place it on the green desk mat.",
  "operator": "operator id",
  "start_monotonic_ns": 0,
  "end_monotonic_ns": 0,
  "success": null,
  "notes": ""
}
```

The task label is the user-selected black-rod-to-green-mat task. Hardware and
operator values remain placeholders until P2/P4. Selecting a task does not
authorize robot motion or episode collection.

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

The current recorder writes `base.mp4`, `wrist.mp4`, seven timestamped channel
JSONL files, `events.jsonl`, and a checksummed `manifest.json`. The file-only
`laptop.recorder.align_raw` command bridges that raw layout to the converter:

```text
raw MP4 + base/wrist/joint_12/joint_34/joint_56/gripper/status JSONL
  -> piper_aligned_preview_v1 (extracted PNG + aligned source references)
  -> human review of units, task result, and action targets
  -> piper_aligned_episode_v1 (frames.jsonl + aligned_episode.json + PNG)
  -> convert_piper_aligned.py -> LeRobot dataset pending P7 audit
```

Preview rows include `policy_timestamp_ns`, per-camera timestamps, raw source
frame IDs, per-channel feedback timestamps, nominal six-joint radians, and
the unchanged gripper raw value. They deliberately omit canonical `state`
and `action`. Host arrival timestamps do not establish camera exposure sync.

For reviewed conversion input, `aligned_episode.json` records the raw
manifest SHA-256, `frame_map_sha256`, the separate review and action-file
SHA-256 values, verified `joint_degrees_per_raw` and
`gripper_meters_per_raw`, action offset, task outcome, and all seven QC flags.
For production real-data conversion, the review must name
`action_target_source: "observed_piper_can_command_frames"`, the hash of raw
`command_can_raw.jsonl`, nominal joint/gripper wire scales, the target-time
offset, and `zero_order_hold_latest_complete_absolute_target` semantics.
`align_raw` derives all action rows from the raw `0x155`–`0x157` and `0x159`
frames; it does not trust a separately typed action vector. The separate
`piper_alignment_review_v1` JSON contains those source hashes, `success: true`,
the unit scales and offset, plus each of
`visual_review_passed`, `camera_sync_verified`, `resampling_verified`,
`joint_units_verified`, `gripper_units_verified`,
`gripper_action_verified`, and `action_target_verified` set to true.
Both files stay outside the immutable raw episode. Reviewed alignment derives
and stores `actions.jsonl`; a later run independently recomputes every action
from the hash-bound raw CAN log. A failed or unreviewed episode remains a
preview and cannot enter the production converter. The passive listener cannot
prove CAN direction or coexist safely with the OEM application on one adapter;
those must be checked separately. If no command frames were captured, do not
reconstruct a command target from feedback.

`laptop.recorder.verify_source` recomputes the raw alignment map, checks
every referenced CAN timestamp and converted state value, and compares each
derived PNG pixel-for-pixel with the selected decoded MP4 frame. The
converter requires this for real reviewed episodes and records the result
in its conversion manifest. `server.dataset_tools.audit_training_data`
then creates a deterministic episode-level split and train-only 7D
statistics. Its audit binds hashes of the dataset and statistics, and can
bind a checkpoint only when that checkpoint carries matching provenance.
These file checks do not establish camera exposure synchronization or
automatically set `training_eligible=true`.

For real reviewed episodes, conversion also writes
`provenance/acceptance_manifest.json` from the checksum-bound human review and
sample-level verifier result. It records task success, command-target source,
camera/gripper/unit review flags, and source hashes; the conversion manifest
pins its SHA-256. `check_piper_readiness` requires both the in-dataset sidecar
and any supplied acceptance copy to match this hash. It does not infer success
or turn the dataset's `training_eligible` flag on.

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

The [OpenPI UR5 example](https://github.com/Physical-Intelligence/openpi/blob/main/examples/ur5/README.md)
shows how a robot-specific adapter maps base and wrist images, state, and task
text into policy inputs. The [LeRobot Dataset v3 guide](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx)
documents multi-camera video, tabular signals, task metadata, and episode
boundaries. These references do not establish that PiPER collection is ready.

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
