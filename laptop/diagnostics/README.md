---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/RUNBOOK.md)。

# Laptop diagnostics

Diagnostics are opt-in and bounded; importing a module never opens CAN or
controls the robot. Each CLI that performs a live read requires an explicit
approval flag. Getter-query utilities are CAN data transmitters and are
separately gated; they are not equivalent to passive receive-only probes.

`probe_piper_feedback_ack_only.py` now accepts `--feedback-log` to exclusively
create a JSONL file with each decoded CAN feedback frame, raw payload, and
laptop monotonic arrival timestamp. Its existing five-second bound and explicit
`--feedback-read-approved` guard remain. The adapter emits hardware ACKs but
the script contains no CAN data-frame transmission or robot-control call.
Only run it when the operator has confirmed that no other program owns the
adapter and has approved this specific read. The output belongs in ignored
`cache/`, never Git. `audit_feedback_log.py` analyzes that file offline; a
continuity result does not certify manual-motion correlation or gripper units.

`list_candlelight_adapters.py` only enumerates supported USB bus/address
identities. It does not set bitrate or start a CAN interface. The manual
command-target recorder uses an exact identity for a second passive adapter
and requires hardware `LISTEN_ONLY`; do not select the adapter owned by the
OEM application or assume USB ownership can be shared.

The normal ACK-only, PiPER feedback, motor-feedback, and listen-only probes
also receive through `SafetyBridge` and the shared
`GsUsbPiperCanTransport`. The motor-feedback probe is blocked unless you pass
its explicit approval flag. Listen-only mode requires confirmation that an
independent peer can acknowledge bus traffic; it cannot transmit getter
requests. These wrappers use `POLICY_REJECT_ONLY`, so they never authorize
command frames.

The firmware, joint-parameter, and gripper-parameter query tools now share
`CanDiagnosticSession` and route through `SafetyBridge` plus the common
`PiperCanTransport`. The separate `DIAGNOSTIC_QUERY` runtime permits only exact
allowlisted getter requests, only after fresh status/joint/gripper feedback,
and only with an explicit per-invocation CLI approval flag and bounded query
budget. These are CAN data TX frames even though they do not command motion.
`CommandProposal` TX remains rejected in every mode. If a USB write fails or
its result is ambiguous, the transport closes and the tool must not retry.

The offline tests use `FakeCanTransport`; they do not enumerate a USB adapter,
read live feedback, or transmit diagnostic queries. The current local report
still records that the 2026-10-06 passive probe saw no PiPER feedback, so this
refactor is not evidence that a live diagnostic query is currently possible.

`probe_dual_cameras.py` now runs its camera work in a disposable
process. The command requires the two present Windows camera
`InstanceId` values and `--roles-visually-confirmed` in
addition to distinct OpenCV indices. It records both the indices and PnP IDs
in the report, but the index-to-ID binding still depends on an operator
checking the actual images. Recheck after replugging either camera. A blocked
driver read can exceed the nominal capture duration; the parent process
terminates the worker after the duration plus a startup/cleanup allowance and
reports a timeout instead of claiming a completed test. The direct `probe()`
function is not process-supervised. None of this validates real-device
termination behavior without a hardware test.
