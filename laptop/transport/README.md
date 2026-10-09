---
Document status: REFERENCE
Current authority: docs/SAFETY_AND_CONTROL.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/SAFETY_AND_CONTROL.md)。

# PiPER CAN transport

This package defines the single CAN ownership boundary for PiPER runtime and
offline tests. `SafetyBridge` claims one transport instance and owns its
receive lifecycle.

## Implementations

`PiperCanTransport` is the shared interface. `GsUsbPiperCanTransport` opens
exactly one candleLight adapter at 1 Mbps in either `NORMAL_ACKING` or
`LISTEN_ONLY` mode, validates the requested adapter flags, and timestamps
received frames with the laptop monotonic clock. Its command `send()` method
always raises. A separate
`send_diagnostic_query()` method accepts only a validated `DiagnosticCanQuery`
from a fixed getter allowlist (firmware `0x4AF`, joint parameters `0x472`, and
the `0x477/0x04` gripper-parameter getter). It cannot accept arbitrary CAN
frames or command proposals.

The dedicated `send_supervised_position()` path accepts only a typed absolute
position proposal encoded as the fixed four-frame batch `0x155`, `0x156`,
`0x157`, `0x159`, plus a one-use operator authorization bound to that exact
proposal and valid for at most 250 ms. It is available only in
`NORMAL_ACKING` mode and also requires the caller's fresh feedback snapshot and
explicit `SafetyLimits`; the transport rechecks state sequence, freshness,
joint/gripper bounds, and per-step deltas before writing. `LISTEN_ONLY` and the
generic `send()` path still reject position commands. Successful writes return the exact frame bytes and the
monotonic time when the local USB adapter accepted each write. This timestamp
does not prove the robot executed the target. If any write is false, times out,
or fails after an earlier frame, the adapter closes and the transport latches
delivery-unknown; it never retries that batch or reopens that transport
instance. The caller must stop and verify read-only feedback before creating a
new session.

The 13 site-gate values and operator approval are caller-supplied attestations;
the transport cannot inspect physical limits, unit calibration, emergency-stop
readiness, or the truth of those attestations. This code path has only been
tested with in-memory adapters. It is not approval to send on a robot and is
not yet connected to the policy runtime or SafetyBridge.

Getter requests are still CAN data transmissions. They are enabled only in
`DIAGNOSTIC_QUERY`, after explicit operator approval and fresh arm/joint/gripper
feedback through SafetyBridge, with an explicit bounded query budget. They do
not authorize mode changes, enable, reset, target, gripper-control, HIL, or
policy-originated command traffic. A failed or unacknowledged USB write is
treated as delivery-unknown and must not be retried automatically.
`LISTEN_ONLY` mode never ACKs the bus and rejects all diagnostic getter TX.

Manual command logging can select a separate passive candleLight receiver by
exact USB bus/address when the OEM application owns another adapter. Selection
never falls back to another device if the requested identity is missing. The
USB identity listing helper does not set CAN bitrate or start the controller;
the command logger requires hardware-verified-at-open `LISTEN_ONLY` mode and
fails closed when unsupported.

`FakeCanTransport` implements ownership and frame-read behavior without USB
or CAN hardware. Generic sends are still denied by default; its explicit
recorded-position option writes only to an in-memory receipt and cannot touch
the robot. Tests enqueue `CanFrame` objects and can inject disconnects without
touching the robot.

## Runtime ownership

The live-shadow CAN reader, passive PiPER probes, generic ACK-only probe,
listen-only probe, and allowlisted getter-query tools use the shared transport
through SafetyBridge. The bridge closes the receive path before it releases
ownership. Legacy HIL scripts remain blocked before USB access until a
command-capable SafetyBridge path passes the real-hardware gates.

## Verification boundary

Unit tests verify ownership, frame delivery, disconnect handling, command-send
veto, the query whitelist, freshness preflight, and delivery-unknown behavior.
They do not verify firmware, units, physical limits, safe-hold, watchdog timing,
or adapter behavior on the connected robot. Do not treat an offline transport
test as hardware qualification.
