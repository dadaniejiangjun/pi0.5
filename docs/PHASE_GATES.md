# Phase gates

Each gate has a status, evidence path and stop condition. `PASS` means the
named evidence exists and was checked; it does not grant later permissions.

| ID | Gate | Required evidence | Stop if |
|---|---|---|---|
| P0 | official docs + architecture audit | OpenPI commit/submodules/config inspection; server/laptop contract; user decision list | source or authority boundary remains ambiguous |
| P1 | server OpenPI bring-up | pinned environment; dummy observation; output shape; VRAM/load/compile/inference timing; finite output | import/load/inference mismatch or large weight operation not approved |
| P2 | laptop hardware/camera probe | firmware, SDK, protocol, CAN, modes, units, limits, camera serial/model/FPS/intrinsics | firmware, units, teach status, camera identity or control mode unknown |
| P3 | network bring-up | selected NET-A/B/C; 1000-request latency/reconnect report; metadata handshake | public exposure, unreliable path, stale response or unsafe timeout behavior |
| P4 | recorder implementation | local-first closed episode; dual video/state/gripper/instruction/events; checksums | any missing modality, non-monotonic timestamps or unavailable gripper |
| P5 | five-demo validation | 5 episodes; frame/FPS/dropout/skew/integrity report | recorder cannot pass all five without manual repair |
| P6 | 20–50 demonstrations | natural variation; success/failure labels; operator notes; no replay-as-diversity | inadequate coverage or labels/units still uncertain |
| P7 | LeRobot conversion/data audit | source-to-derived manifest; episode split; schema; stats; raw preserved | split leakage, schema mismatch or missing normalization fields |
| P8 | transforms/tiny overfit | 7D round trip; online/offline equivalence; 1-episode 100–500-update fit | transform mismatch, no loss descent, trivial baseline not beaten |
| P9 | pilot fine-tune | 20–50 demos; early/mid/final checkpoints; offline eval and failure slices | data-path or offline result is not credible |
| P10 | formal fine-tune | frozen commit/config/dataset/norm/checkpoint provenance; measured GPU use | any provenance or normalization drift |
| P11 | real shadow inference | no robot commands; predictions/current q/margins/latency/video overlay | stale/unsafe predictions or metadata mismatch |
| P12 | single action | G1/G2 safety evidence; low speed; operator/E-stop; one fresh action | any reject path fails closed or physical behavior is unvalidated |
| P13 | full single-object task | G3–G5 ladder; task success and failure evidence | task success depends on unlogged intervention or unsafe margins |
| P14 | multi-object language | frozen P13 baseline; color/target language dataset and held-out evaluation | grounding claim lacks target-controlled data |

## Universal hard stops

Unknown PiPER firmware, unknown units, camera/state synchronization failure,
unavailable gripper recording, unreliable network, missing SafetyBridge, missing
stale-action protection, PiPER/OpenPI transform mismatch, or normalization
mismatch blocks autonomous execution regardless of phase number.

## Evidence discipline

Reports are written before a server process or simulator is shut down. Every
result includes command, environment, commit, config, seed, input manifest and
output path. A technical pass is not labeled as physical success.
