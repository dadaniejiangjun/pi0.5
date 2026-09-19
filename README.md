# 07_pi0.5 — Real-world π0.5 + AgileX PiPER VLA

This is an independent project for a real AgileX PiPER language-conditioned
manipulation baseline.

The physical control boundary is deliberate:

```text
Laptop: cameras + PiPER authority + PiperSafetyBridge
    ⇅ private/encrypted transport
GPU server: OpenPI policy inference + training
```

The server never owns CAN, robot enable/reset/stop, or low-level control.
`06_maniskill_tactile` and `03_gripper2` are read-only references only. This
project does not use ManiSkill, PiperPegInsertion, or their task/dataset/policy.

## Current status

```text
P0 = PASS
P1 = PASS
P2 = READY
autonomous_execution_authorized = false
```

This status is reconciled from the completed P0/P1 evidence in
`reports/P0_P1_DEPLOYMENT_REPORT.md`,
`provenance/deployment_manifest.json`, and
`docs/PI05_PIPER_MASTER_PLAN.md`. It is metadata reconciliation only;
no experiment, download, inference, training, benchmark, server startup,
or CAN access is performed by a status update.

The current safety nonclaims remain:

```text
REAL_PIPER_CONNECTED = NO
REAL_ROBOT_MOTION = NO
REAL_DATA_COLLECTION = NO
FORMAL_FINE_TUNING = NO
```

The single machine-readable authority for phase status is
`docs/PI05_PIPER_PHASE_GATES.json`. After any phase completion, update
that file together with this README current-status block and
`provenance/deployment_manifest.json`.
Start with:

- [Master plan](docs/PI05_PIPER_MASTER_PLAN.md)
- [Phase gates](docs/PHASE_GATES.md)
- [User decisions](docs/USER_DECISIONS.md)
- [OpenPI provenance](docs/OPENPI_PROVENANCE.md)
- [Real-robot safety plan](docs/REAL_ROBOT_SAFETY_PLAN.md)

The unique machine-readable authority for phase status is
`docs/PI05_PIPER_PHASE_GATES.json`. A technical check, a shadow prediction,
and a real-robot success are separate claims.
`autonomous_execution_authorized` remains `false` until the safety-gated
project process explicitly changes it.
