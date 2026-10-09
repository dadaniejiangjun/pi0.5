---
Document status: REFERENCE
Current authority: docs/DATA_AND_TRAINING.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：当前实验使用 future-state proxy 转换；下文 command-label 转换是另一条有不同监督条件的路线，不能将其要求套成所有训练均被阻塞。 当前领域主文档见 [authority](../../docs/DATA_AND_TRAINING.md)。

# Dataset tools module

`laptop.recorder.align_raw` now creates a reproducible, checksum-bound aligned
preview from saved raw video and feedback. An independent successful-demo and
action-target review can produce `piper_aligned_episode_v1`, which
`convert_piper_aligned.py` verifies and converts to a local LeRobot dataset.
Both stages preserve source references and refuse to overwrite existing
outputs. For a real reviewed episode, the converter now replays every CAN
reference and compares each extracted PNG with its decoded source MP4 frame.
It sets `source_frame_extraction_verified=true` only after those checks
pass. It independently rebuilds every action target from hash-bound raw CAN
command frames (`0x155`–`0x157`, `0x159`) and stores the per-frame CAN source
references. Synthetic fixtures retain `false`. Conversion still reports
`training_eligible=false`; frame/action provenance alone is not training
approval. A sidecar action file without corresponding raw CAN target evidence
is not accepted for real-data conversion.
The real conversion also derives a separate `provenance/acceptance_manifest.json`
from the checksum-bound human review and verified raw source, then pins its hash
in the conversion manifest. Training readiness checks that this sidecar and any
supplied copy match; the converter never decides success on its own.

Raw data is never overwritten or discarded by conversion.

For OpenPI-bound `convert_next_state_proxy.py` outputs, use the isolated
environment pinned by the OpenPI lock: `datasets==3.6.0` and LeRobot revision
`0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`. The converter checks both before
creating output and records the runtime versions in `provenance/`.
Datasets created with another `datasets` version may have incompatible Parquet
features even when conversion succeeds; retain them as legacy artifacts and do
not pass them to the pinned OpenPI training stack.

Use `audit_training_data.py build --dataset <LeRobot directory> --output
<unused audit directory>` to split by complete episode and calculate
seven-dimensional state/action statistics from training episodes only.
One episode is a train-only tiny-fit split with no invented validation set.
Run `verify --output <audit directory>` to rehash the dataset and stats.
To bind a checkpoint later, its `training_provenance.json` must name the
exact `dataset_sha256` and `norm_stats_sha256` from the audit. Then use
`bind --output <audit directory> --checkpoint <checkpoint directory>`.
This checks recorded provenance and file integrity; it cannot prove that
training actually used those files. Neither audit tool starts training.
