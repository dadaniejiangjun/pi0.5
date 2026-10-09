---
Document status: REFERENCE
Current authority: docs/DATA_AND_TRAINING.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/DATA_AND_TRAINING.md)。

# Recorder

The local raw-episode writer now records per-channel monotonic timestamps,
events, the black-rod/green-mat instruction, SHA-256 checksums, and a manifest.
The synthetic smoke test also writes two short MP4 files. It exercises the file
pipeline only; it does not connect to cameras or PiPER.

Run the offline smoke test from the repository root in the dedicated Conda
environment:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.recorder.synthetic_smoke
```

The resulting episode stays under the ignored `datasets/raw/` directory.
`validate_episode()` intentionally marks it **INCOMPLETE** for training: the
samples are synthetic, the state-to-action target has not been defined or
validated, and the gripper command target is absent. An MP4 file and readback
alone never establish a usable demonstration. The writer refuses an existing
episode ID and never overwrites a prior capture.

The process-supervised `passive_capture.py` entry point wires dual-camera
video and ACK-only PiPER feedback into one local episode. It is a ten-second P4 wiring
smoke, not a task demonstration. It requires explicit approval through
`--passive-capture-approved`; without the flag it opens neither cameras nor CAN
and creates no episode. With approval, run it only after confirming that no
other program owns CAN and that the operator controls all robot movement.
First list the present Windows camera IDs, then confirm which live OpenCV
image is Base and which is Wrist. Reconfirm this mapping after any replug:

```powershell
Get-PnpDevice -Class Camera -PresentOnly |
  Select-Object FriendlyName,Status,InstanceId
```

Pass the two exact `InstanceId` values with the visually confirmed OpenCV
indices:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.recorder.passive_capture `
  --base-index 0 --wrist-index 1 --duration 10 `
  --base-device-id '<Base InstanceId>' --wrist-device-id '<Wrist InstanceId>' `
  --roles-visually-confirmed --passive-capture-approved
```

The PnP IDs establish which devices are present, but Windows does not prove
the OpenCV-index-to-ID mapping. The operator's visual confirmation is recorded
in `camera_identity.json` and `camera_report.json`. If a camera
`read()` or driver cleanup blocks, the CLI terminates its disposable
worker after the duration plus a startup/cleanup allowance. It writes an
out-of-episode marker under `datasets/raw/_capture_runs/`.
Only a marker with `complete=true` and a manifest hash records a
finished raw capture; this never means training acceptance. A forced worker
termination can leave a partial episode, which must not be used. The direct
`capture()` helper is not process-supervised; use the CLI for hardware.

The standalone CAN feedback probe keeps its five-second limit. Only this
explicitly approved combined capture can run for up to ten seconds. The cameras
open first and wait for the CAN receiver to start before capture timing begins;
this startup change is currently verified only with simulated devices. The program
never enters teach mode or sends CAN data frames. It retains raw
CAN payloads, separate camera-frame and feedback timestamps, two MP4 videos,
events, and checksums under the ignored `datasets/raw/` directory. An earlier
real-device ten-second passive episode was recorded, but its startup preceded
complete CAN state coverage and visual review did not show task success. It is
**not** a demonstration. The revised synchronized startup has not yet been
retested with the real PiPER. All passive output remains ineligible for training because raw joint feedback
is not yet a validated action target, and the gripper teaching target is
unknown. Visual playback, camera-to-state alignment, firmware and unit
confirmation, gripper teaching, and a 10–20-second task demonstration remain
separate P2/P4 gates. Never send a model prediction to CAN from this tool.

For a command-target collection attempt, `manual_teach_capture.py` listens in
hardware **listen-only** mode for raw PiPER command IDs `0x155`–`0x157` and
`0x159`. When the OEM controller owns one candleLight adapter, the recorder
must use a second, physically separate supported adapter on the same CAN bus.
It selects that listener by exact USB bus/address; it never opens the OEM-owned
adapter. The recorder only receives frames; it does not send CAN or infer
motion from feedback. First enumerate USB identities (this does not set bitrate
or start CAN), identify the dedicated passive adapter, and then run one bounded
demonstration with the same camera identity checks and an unused raw destination:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.diagnostics.list_candlelight_adapters
$CanBus = Read-Host 'USB bus of the dedicated passive listener from the list above'
$CanAddress = Read-Host 'USB address of that listener from the list above'
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.recorder.manual_teach_capture `
  --duration 20 --lead-in 5 --base-index 0 --wrist-index 1 `
  --can-adapter-bus $CanBus --can-adapter-address $CanAddress `
  --base-device-id '<Base InstanceId>' --wrist-device-id '<Wrist InstanceId>' `
  --roles-visually-confirmed --manual-capture-approved
```

The CAN frame IDs and raw payloads are preserved with laptop monotonic receive
timestamps, but direction is not independently proven by this logger. The
exact device must be a separate listener on the same physical bus, support the
hardware listen-only flag, and see the command frames while the OEM software
controls the robot. If no separate adapter or usable OEM command-log export is
available, this raw command-target path is not ready; do not try to share one
USB-CAN adapter or reconstruct targets from joint feedback. An empty command
log is not a target dataset.

The raw episode intentionally ends with `success=None`. The recorder cannot
judge the task. Record the observed result separately, without changing the
checksum-verified raw episode:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.recorder.label_episode `
  datasets/raw/<episode_id> --failure `
  --notes 'Gripper did not hold the black rod' --operator '<operator name>'
```

Use `--success` only after visual review confirms the task. The
outcome file is created once under the ignored `datasets/manifests/outcomes/`,
binds to the raw manifest SHA-256, and cannot be overwritten by this command.
A result label does not create a validated action target or unlock training.

To re-audit an existing episode without connecting to any device or changing
the raw files, run:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.recorder.audit_episode `
  datasets/raw/<episode_id>
```

The exit code is nonzero for an incomplete episode. Host-arrival timestamp
alignment is diagnostic only; it does not prove synchronized camera exposures.

## Align a saved episode

Run the file-only alignment tool from the repository root with the dedicated
Conda interpreter. It verifies the raw manifest, selects a common camera and
feedback time window, extracts the selected MP4 frames, and saves source
references for every derived frame. It never opens a camera or CAN device.

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.recorder.align_raw `
  datasets/raw/<episode_id> `
  --output datasets/manifests/generated/<preview_id> --fps 10
```

This default output has schema `piper_aligned_preview_v1`. It includes the six
joint values in nominal radians for inspection and the untouched raw gripper
value. It has no canonical seven-dimensional state or action target, so it is
not accepted by the LeRobot converter. The report's `frame_map_sha256` binds
a later review to the exact selected source samples.

To create a converter input from observed CAN targets, prepare a review JSON
bound to the raw manifest and preview frame-map hashes. Set
`action_target_source` to `observed_piper_can_command_frames`, include the raw
`command_can_raw.jsonl` SHA-256, nominal wire scales, nonnegative target-time
offset, and explicit `zero_order_hold_latest_complete_absolute_target`
semantics. All seven review flags must pass. `align_raw` recomputes each target
from the raw command stream; do not supply manually typed targets for this
production route. Its output remains `training_eligible=false` until the
independent converter and dataset audit pass. Failed episodes remain failures.
See `docs/DATA_SCHEMA.md` for the exact fields.

Run `python -m laptop.recorder.verify_source <raw episode> <aligned episode>`
to recompute every selected source reference, state conversion, and decoded
video frame. A reviewed output also keeps checksum-bound copies of
`review.json` and `actions.jsonl`, which the verifier checks against
the aligned actions. This is a file-provenance check, not proof of synchronized
camera exposure, correct human labels, or task success.
