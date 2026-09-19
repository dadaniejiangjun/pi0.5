# Real-robot safety plan

Status: `HARD-GATED / NOT-READY-FOR-AUTONOMOUS-EXECUTION`

## Control authority

Only the laptop can reach PiPER CAN/ROS/SDK. The server cannot enable, reset,
stop or command the robot. The laptop `PiperSafetyBridge` has final veto over
every model action.

## SafetyBridge input/output

```text
raw model response
  → response identity/freshness checks
  → finite-value/shape checks
  → state freshness and arm-status checks
  → joint/gripper hard limits
  → software soft limits
  → per-step delta / velocity / acceleration limits
  → workspace sanity
  → mode/teach/watchdog checks
  → SAFE_ACTION or REJECT
```

No raw model action is sent directly to a controller. Every accept and reject
has a reason code and the preceding observation/action hashes.

## Mandatory rejection conditions

- NaN, Inf, wrong dimensionality or malformed serialization;
- missing or stale camera frame;
- missing or stale joint/gripper feedback;
- wrong `request_id`, checkpoint hash or model-config hash;
- late response after a newer observation;
- timeout, disconnect, server exception or heartbeat failure;
- joint hard/soft limit violation;
- excessive per-step delta, velocity or acceleration;
- gripper limit violation;
- unknown or unsafe robot/teach/control status;
- any unverified unit or frame convention.

## Watchdog and safe hold

The watchdog stops generation of new commands on server unavailability, excess
RTT, response timeout, laptop process failure or model exception. The robot
must enter a safe hold behavior that is validated on the actual PiPER firmware.
The project does not guess whether hold means a position hold, controller
disable, stop service, or another hardware-specific mode. Automatic reset,
power-off and `stop_srv` remain prohibited until separately audited.

If the laptop crashes, the robot-side/laptop-side watchdog path must still
fail safe. This requirement is tested before any action gate.

## Execution ladder

| Gate | Operation | Required conditions |
|---|---|---|
| G1 | dry-run display | no command path; predicted vs current q only |
| G2 | one action | operator, E-stop, very low speed, fresh state, all checks |
| G3 | five actions | G2 repeatable; no stale/reject ambiguity |
| G4 | ten actions | latency and margins recorded |
| G5 | full episode | prior gates plus task-level stop/recovery evidence |

Each gate has an explicit abort procedure. Passing a software unit test is not
passing a physical safety gate.

## Required rollout log

Each real rollout retains:

- episode and request IDs;
- OpenPI commit and checkpoint SHA;
- PiPER firmware/SDK/protocol versions;
- robot and camera config hashes/serials;
- prompt and prompt hash;
- network option;
- all observations, raw actions, safe actions and reject reasons;
- current robot states, latency percentiles and watchdog events;
- base/wrist video and operator outcome.

## Absolute stop conditions

Autonomous execution is prohibited while any of the following is unknown or
failed: PiPER firmware, units, camera-state synchronization, gripper
recording, network reliability, SafetyBridge, stale-response protection,
PiPER/OpenPI transform equivalence, or normalization contract.
