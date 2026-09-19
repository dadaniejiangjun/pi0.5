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

`PLAN_ONLY / NO_HARDWARE_EXECUTION`

This initial scaffold contains the project contract, phase gates, provenance
records, data schema, network options, safety plan, and laptop diagnostic
specifications. It intentionally does not clone OpenPI, download weights,
install dependencies, connect CAN, collect data, train, or move the robot.

Start with:

- [Master plan](docs/PI05_PIPER_MASTER_PLAN.md)
- [Phase gates](docs/PHASE_GATES.md)
- [User decisions](docs/USER_DECISIONS.md)
- [OpenPI provenance](docs/OPENPI_PROVENANCE.md)
- [Real-robot safety plan](docs/REAL_ROBOT_SAFETY_PLAN.md)

The only valid path to autonomous execution is through the gates in
`docs/PI05_PIPER_PHASE_GATES.json`. A technical check, a shadow prediction,
and a real-robot success are separate claims.
