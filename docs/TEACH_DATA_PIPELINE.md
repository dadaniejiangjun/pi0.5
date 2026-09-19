# Teach-mode multimodal data pipeline

## Collection authority

The operator enters PiPER native Drag Teach / Teach Mode through the official
current-hardware procedure. The first recorder does not assume the SDK can enter
teach mode programmatically. It passively records the robot state, gripper,
cameras, instruction and events while the operator performs the demonstration.

The PiPER-internal teaching trajectory is retained as replay/reference evidence
when available. It is not the only training source because π0.5 requires a
time-aligned vision + language + state → action record.

## Laptop-first episode lifecycle

```text
CREATE episode.tmp
  → start monotonic clock
  → open base/wrist video writers
  → open state and event streams
  → record until operator ends episode
  → close all writers
  → validate monotonicity, counts, timestamps and finite values
  → SHA256 every file
  → write manifest
  → READY_FOR_UPLOAD
  → background encrypted upload
```

Network availability is not a precondition for a demonstration. A transfer
failure leaves the closed local episode intact. A remote episode is accepted
only after checksum verification and atomic rename; an existing episode ID is
never overwritten.

## Required raw contents

```text
episode_000001/
  metadata.json
  instruction.txt
  robot.parquet        # or robot.npz, with explicit schema/version
  base.mp4
  wrist.mp4
  events.jsonl
  manifest.json
```

Each state row contains episode/frame identifiers and separate capture/sample
timestamps for base RGB, wrist RGB, joints and gripper. All timestamps are
derived from the laptop monotonic clock. Remote wall-clock time is telemetry,
never alignment authority.

## Gripper hard gate

The first experiment must establish whether PiPER drag teaching can coexist
with a software or official gripper command. Record both target and actual
gripper state. If that cannot be done reliably, the episode is not
training-eligible even if the video looks successful.

## Collection stages

### P0/P1/P2 — no collection

Freeze source and audit hardware facts first.

### P4/P5 — five episodes

The only purpose is recorder validation. Check dual video, state, gripper,
timestamps, instruction, events, checksums and camera-state skew. Do not train.

### P6 — twenty to fifty demonstrations

Collect successful and naturally varied trajectories. Do not repeatedly replay
one path and call it diversity. Keep success/failure labels and operator notes.

### P7 onward — fifty to one hundred

Scale only after raw and conversion gates pass. Episode density in a controlled
workspace matters more than a headline count. Future 100/200/500+ targets are
decisions driven by held-out performance, not a first-turn commitment.

## Upload and conversion boundaries

```text
Laptop spool
  → SHA verified server incoming
  → canonical validated raw
  → LeRobot v3 dataset
  → PiPER transform/norm audit
```

Raw files are immutable inputs. The conversion script writes a new derived leaf
and a manifest containing source episode IDs, source hashes, code commit,
schema version, camera mapping, split assignment and statistics hash.
