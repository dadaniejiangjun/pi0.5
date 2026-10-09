---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

# Laptop diagnostic script specifications

Diagnostic implementations now exist under `laptop/diagnostics/`; this page
is the original output/permission specification, not a current progress report.
Current reusable camera/CAN checks are documented in [RUNBOOK](../RUNBOOK.md).
Probe evidence and its verification scope must be read from dated reports.
Read-only probes must not be confused with separately authorized reset/enable/motion tools.

## `probe_piper.py`

Inputs: optional CAN device and SDK configuration supplied by the operator.

Outputs: `reports/laptop_piper_hardware_audit.json` containing:

- firmware version;
- SDK/package version and protocol version;
- CAN device and baudrate;
- joint feedback frequency and timestamp behavior;
- gripper type, units, limits and feedback;
- supported control modes;
- teach-mode support and whether programmatic entry is reliable;
- real joint ranges and current DH/frame convention;
- read-only evidence commands, source docs and timestamps.

Safety: do not call enable, reset, stop, motion, teach-entry or gripper-control
methods. A future command-capable probe requires a separate user-approved
hardware procedure, not a flag added casually to this script.

## `probe_cameras.py`

Outputs: `reports/laptop_camera_audit.json` containing, for each base/wrist
candidate:

- USB identifier, manufacturer, model and serial;
- connection topology;
- supported RGB resolutions, formats and FPS;
- intrinsics/distortion availability;
- selected raw capture mode;
- mount description and frame name;
- frame counter/dropout checks;
- raw-to-model resize/pad mapping evidence.

The probe does not guess the model of the user's “大白摄像头”. If the wrist is
a D435, it explicitly records RGB stream, intrinsics, mount and camera frame;
otherwise it records the unknown state for official documentation lookup.

## `probe_network.py`

Outputs: `reports/network_latency_report.json` after the user selects NET-A,
NET-B or NET-C. It runs 1000 requests per payload for dummy, one RGB, dual RGB,
and dual RGB + STATE_7. It records RTT median/p90/p95/p99, failures and
reconnect time. It must never send a robot command; the server endpoint is a
policy/diagnostic endpoint only.

## Common requirements

- use laptop monotonic time for local measurements;
- include project version and script hash;
- emit JSON plus a human-readable summary;
- preserve raw probe output;
- distinguish `NOT_RUN`, `PASS`, `FAIL` and `UNKNOWN`;
- do not overwrite an existing audit report;
- record the exact hardware/docs source used for each value;
- fail closed when a dependency or device is missing.
