# π0.5 + AgileX PiPER master plan

Status: `PLAN_ONLY / NO_AUTONOMOUS_EXECUTION`

This plan creates a reproducible route to the first research milestone:

```text
real base RGB + real wrist RGB + real PiPER state + language
    → π0.5
    → action chunk
    → laptop PiperSafetyBridge
    → real PiPER
```

The first milestone is a single-object, language-conditioned pick-and-place
task. The project does not begin with multi-object tasks, tactile sensing,
ManiSkill, PiperPegInsertion, or a remote server controlling CAN.

## 1. Non-negotiable architecture

```text
                         private encrypted link
     ┌──────────────────────────────────────────────────────┐
     │                                                      │
┌────▼─────┐       observation request       ┌──────────────▼─────────┐
│ Laptop   │ ──────────────────────────────► │ GPU/data server         │
│          │                                 │                          │
│ cameras  │ ◄──────── action response ───── │ OpenPI policy server     │
│ recorder │                                 │ π0.5 / checkpoint        │
│ piper    │                                 │ training / datasets      │
│ safety   │                                 │ no CAN / no robot enable │
└────┬─────┘                                 └──────────────────────────┘
     │
     │ final veto, validated command only
┌────▼─────┐
│ PiPER    │  CAN / ROS / SDK is reachable only from the laptop
└──────────┘
```

The server is the policy/training authority. The laptop is the robot
authority. A server response is never a robot command until the laptop has
validated it. A network failure must prevent a new command from being issued,
and must not cause execution of an old action chunk.

## 2. Scope and exclusions

In scope:

- official `Physical-Intelligence/openpi` source and π0.5 server/client;
- an operator-manual PiPER drag-teach workflow;
- laptop-first dual-camera/state recording;
- raw episode integrity and encrypted upload;
- LeRobot conversion with episode-level splits;
- PiPER-specific transforms, normalization, and action contract;
- shadow inference and a guarded real execution ladder.

Out of scope for phase 1:

- ManiSkill or any dependency on `06_maniskill_tactile` tasks, datasets or
  policies;
- edits to `06_maniskill_tactile` or `03_gripper2`;
- tactile sensing;
- public WebSocket exposure;
- automatic teach-mode entry before current firmware evidence exists;
- multi-GPU training before measured single-GPU feasibility;
- full action-chunk execution before one-step execution is accepted.

## 3. Responsibility split

| Concern | Laptop | GPU/data server |
|---|---|---|
| CAN / PiPER SDK / ROS | Owns and isolates | Never accesses |
| Camera capture | Owns | Receives model-sized frames only for inference |
| Time authority | Monotonic clock for all sensor records | Logs request/receive timing, never sensor alignment |
| Raw episode | Writes and preserves local spool | Verifies and stores a second master copy |
| Policy inference | Calls client, validates response | Loads and runs OpenPI |
| Safety | Final veto, watchdog, safe hold | No veto override and no robot command |
| Training | No checkpoints | Owns OpenPI, datasets, norm stats, checkpoints |
| Weights | Not synchronized by default | Server-only |

## 4. Phase sequence

| Phase | Purpose | Main output | Real motion? |
|---|---|---|---|
| P0 | Official docs + architecture audit | pinned provenance and unresolved contract list | No |
| P1 | OpenPI server bring-up | dummy inference evidence | No |
| P2 | Laptop PiPER/camera/network probe | hardware audit reports | No |
| P3 | Laptop ↔ server network | latency and reconnect report | No |
| P4 | Teach-mode multimodal recorder | one locally valid raw episode | Operator-controlled only |
| P5 | Five-demo recorder validation | recorder gate report | No autonomous motion |
| P6 | Twenty to fifty demonstrations | labeled, diverse raw set | Manual teach only |
| P7 | LeRobot conversion + data audit | validated dataset and manifest | No |
| P8 | PiPER transforms + tiny overfit | round-trip and tiny-fit evidence | No |
| P9 | Pilot fine-tune | early/mid/final offline checkpoints | No |
| P10 | Formal fine-tune | frozen checkpoint/config/norm bundle | No |
| P11 | Real shadow inference | predictions and safety telemetry | No |
| P12 | Single-action guarded execution | one accepted action evidence | Yes, one action |
| P13 | Full single-object pick-place | first real milestone evidence | Yes, gated |
| P14 | Language-grounded multi-object tasks | later milestone evidence | Yes, gated |

No later phase is implied by completion of an earlier phase. In particular,
P1 inference success is not robot success, and P11 shadow success is not
authorization for P12.

## 5. OpenPI source and model plan

The only allowed OpenPI source is the official repository. P0 must clone it
with submodules and freeze the exact local commit, submodule SHAs, dependency
lock, inspected source-file hashes, and checkpoint identity in
`provenance/openpi_contract.json`.

The current official source snapshot distinguishes a `pi05_base` checkpoint /
asset identity from a `pi05_droid` DROID configuration. This project does not
turn that observation into a final user decision. CKPT-A (`pi05_base`) is the
recommended initial candidate for a new embodiment; CKPT-B (`pi05_droid`) is a
secondary comparison only.

OpenPI's DROID transform is a reference for the shape of a custom transform,
not a PiPER contract. The PiPER implementation must define project-local:

- `PiperInputs`: base/wrist image keys, state ordering, units and prompt;
- `PiperOutputs`: action extraction and 7D unpadding;
- `PiperDataConfig`: raw/LeRobot repacking, action semantics and norm stats;
- `PiperTrainConfig`: frozen source/model/config/seed and output paths.

The offline transform and inference transform must share one contract. The
required test is:

```text
PiPER state 7D → model state representation
PiPER action 7D → model action padding/normalization
model action → unpadding/denormalization → PiPER action 7D
```

The round trip must preserve ordering, units, masks and finite values. A DROID
norm-stat file or DROID action convention is not accepted as a PiPER default.

## 6. Observation and action contract

Planning candidate, pending hardware and user decisions:

```text
STATE_7 = [q1, q2, q3, q4, q5, q6, gripper]
ACTION_7 = [q1, q2, q3, q4, q5, q6, gripper]
images = [base_rgb, wrist_rgb]
prompt = episode task instruction
execution_horizon = 1
```

The recommended action choice is ACTION-A, absolute joint position plus
gripper. It remains unfrozen until teach replay, real controller, units,
limits, and OpenPI transform audits pass. ACTION-B and ACTION-C remain
documented alternatives; the plan does not silently select one.

Raw camera streams remain at native or audited capture resolution. The model
input is generated on the laptop using the audited official OpenPI resize/pad
and uint8 path, typically 224×224. The raw frame and model-input mapping must
be retained for later audit.

## 7. Data path

### Offline training path

```text
manual drag teach on PiPER
  → laptop local episode
  → close video/state files
  → checksum + integrity report
  → READY_FOR_UPLOAD
  → encrypted transfer
  → SHA256 verify + atomic rename
  → canonical validated raw
  → LeRobot dataset
  → PiPER norm stats + transforms
  → π0.5 fine-tuning
```

The raw episode is never discarded after conversion. The laptop copy is not
automatically deleted after upload. Existing episode IDs are never overwritten.

### Online inference path

```text
laptop capture at monotonic timestamp
  → preprocess to model input
  → request_id + hashes + telemetry
  → private WebSocket path
  → server policy inference
  → checkpoint/config metadata
  → laptop stale/finite/limit/health checks
  → one safe action or reject
```

The initial execution horizon is exactly one action. EH2/EH4 are later
experiments, not defaults inherited from an old diffusion-policy project.

## 8. Teach-mode plan

Phase 1 assumes the operator enters native Drag Teach / Teach Mode using the
official hardware procedure. The recorder does not assume programmatic teach
entry. It records joint state, gripper state, dual RGB, monotonic timestamps,
teach-status events and the task instruction.

The gripper path is a separate hard gate. It must establish whether drag teach
can coexist with a software or official gripper command, and must record target
and actual gripper state. Video-only inference of gripper state is forbidden.

Data collection is staged:

- P0: no collection;
- P4/P5: recorder validation with 5 episodes, no training;
- P6: 20–50 successful and naturally varied demonstrations;
- P7 onward: 50–100 only after quality gates pass, then scale by evidence.

Failure labels, notes, and success labels are retained in metadata. Failed or
invalid episodes do not silently enter the training set.

## 9. Network plan

The official OpenPI remote pattern is WebSocket-based with a default candidate
port of 8000. This project never exposes a raw policy WebSocket publicly.

Three options are specified, not selected:

- NET-A: SSH local port forwarding for bring-up;
- NET-B: Tailscale/private VPN for recurring experiments;
- NET-C: self-managed WireGuard for independently managed infrastructure.

Recommendation: NET-A for P0/P1 and NET-B only after a stable private-network
need is demonstrated. The final choice is `USER_DECISION_REQUIRED`.

Each request records `episode_id`, `request_id`, capture/send/receive
monotonic times, prompt hash and state hash. Each response records actions,
server inference time, checkpoint ID and model config hash. P3 measures median,
p90, p95, p99 and reconnect behavior for dummy, one-RGB, dual-RGB and
dual-RGB+state payloads.

## 10. Safety and execution ladder

`PiperSafetyBridge` must reject any action with:

- NaN/Inf or malformed shape;
- stale request/response or response for the wrong `request_id`;
- timeout, disconnect, checkpoint mismatch or metadata mismatch;
- stale camera or joint feedback;
- joint hard/soft limit violation;
- excessive per-step delta, velocity or acceleration;
- gripper limit violation;
- workspace sanity failure;
- unsafe arm/teach/control status.

The bridge owns the final veto. Network watchdog failure stops new commands and
enters a previously validated PiPER safe-hold behavior. The exact hold command
is not guessed; it is established in a controlled hardware audit. Automatic
reset, power-off and `stop_srv` are not added by assumption.

Execution ladder:

```text
G1 dry-run display
G2 one action, very low speed
G3 five actions
G4 ten actions
G5 full single-object episode
```

Every step requires an operator, ready E-stop, fresh feedback, accepted safety
checks and a rollback/stop procedure that has already been validated.

## 11. Experiment design

Primary hypothesis:

> A PiPER-specific π0.5 transform, normalization and absolute-joint action
> contract can predict sensible one-step targets from dual RGB, STATE_7 and a
> fixed language instruction on held-out PiPER episodes without leaking episode
> frames or relying on DROID semantics.

Minimal decision sequence:

1. one-episode tiny overfit, 100–500 updates;
2. ten-episode overfit with held-out frames and image/language ablations;
3. 20–50 demonstration pilot with early/mid/final checkpoints;
4. formal training only after data, transform, network and safety gates pass.

Required metrics include training/validation action error, trivial-baseline
gap, finite-value rate, action continuity, state/action range coverage,
camera-state skew, inference and end-to-end latency, rejection rate, and
shadow joint-limit margin. Real success rate is measured only in P13/P14.

Stop rules are explicit: a failed transform round trip, tiny overfit, held-out
sanity check, normalization audit, stale-action test, or safety gate stops the
next expensive phase. Hyperparameters are not changed to hide a data or
contract failure.

## 12. Latency budget

The following components are measured separately, with median/p95/p99:

```text
camera capture
→ preprocessing
→ uplink
→ server inference
→ downlink
→ PiperSafetyBridge
→ robot command
```

The aggregate is `OBS_TO_COMMAND_LATENCY`. The control frequency is selected
from measured capture, feedback, controller and safety behavior; 50 Hz is not
assumed.

## 13. Risk register

| Risk | Consequence | Mitigation / stop |
|---|---|---|
| Firmware/DH convention unknown | wrong state/action geometry | P2 audit; no autonomous execution |
| Gripper not observable during teach | invalid training labels | separate gripper gate; no training-ready demos |
| Camera/state skew or dropout | temporal contract failure | monotonic timestamps, skew metrics, reject episode |
| DROID semantics reused | false transfer claim | local PiPER transforms and stats only |
| OpenPI moving branch | irreproducible result | local commit/submodule/lock freeze |
| Network loss or stale response | unsafe command | horizon 1, request IDs, watchdog, reject |
| Server OOM or checkpoint drift | service instability/wrong model | measured GPU audit, metadata hash, health gate |
| Overly broad data collection | poor coverage and wasted time | 5 → 20–50 → 50–100 staged collection |
| Laptop crash | loss of robot authority or episode | local spool, watchdog, no old-chunk replay |
| Public policy exposure | unauthorized access | tunnel/private VPN; public raw WebSocket prohibited |

## 14. Hard stops

Any unresolved item below blocks autonomous real execution:

- unknown PiPER firmware;
- unknown action units;
- camera/state synchronization failure;
- gripper recording unavailable;
- unreliable network for the selected online mode;
- missing `PiperSafetyBridge`;
- missing stale-action protection;
- PiPER transform mismatch;
- normalization mismatch.

The next executable phase after this plan is P0: official source and
architecture audit. P0 itself remains read-only with respect to robot hardware
and does not download large π0.5 weights.
