---
Document status: REFERENCE
Current authority: docs/ARCHITECTURE.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：这是文档收口前的2026-10-08审计快照；代码结论保留，文档authority调整以 DOCUMENTATION_CLEANUP_REPORT 为准，不重写当时证据。 当前领域主文档见 [authority](../ARCHITECTURE.md)。

# AgileX/PiPER source audit

Status: `P0/P2 REQUIRED / HARDWARE NOT VERIFIED`

Use current official material only:

- https://github.com/agilexrobotics/piper_sdk
- https://github.com/agilexrobotics/pyAgxArm
- the exact official `piper_ros` or `agx_arm_ros` package identified for the
  current robot;
- the user's AgileX Yuque documentation, when supplied, as the hardware
  operation authority.

The official SDK source contains firmware-sensitive interface details. Those
details are audit cues, not facts about this PiPER. P2 must record the actual
firmware, SDK, protocol, units, limits, modes, teach behavior, feedback rate,
DH/frame convention and safe-hold behavior before any autonomous execution.

No value from an old PiPER project is a substitute for this audit.
