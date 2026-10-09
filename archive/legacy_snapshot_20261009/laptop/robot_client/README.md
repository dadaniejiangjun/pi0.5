---
Document status: REFERENCE
Current authority: docs/SAFETY_AND_CONTROL.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/SAFETY_AND_CONTROL.md)。

# Offline robot-client replay

## Supervised mode initialization

`supervised_mode_initialization.py` provides a separate reset, standby, and
CAN/MOVE J initialization path. Reset can release holding torque and cause
the arm to fall. Confirm a supported physical zero pose before using it.
The entrypoint checks fresh joint/motor feedback, nominal zero within 1 degree
on all axes, and stable feedback before each write. This tolerance is a software
precondition, not calibration evidence. Each write is followed by feedback;
missing standby confirmation blocks CAN setting. Unknown delivery stops without
retry. The three frames use the official SDK encoding and a 5% speed setting.

Use `--preview` for a device-free frame preview. Live initialization requires
`--initialize-can-mode --operator-authorized-reset --zero-pose-supported
--physical-estop-ready --motion-envelope-cleared`. It does not enable motors
or send position targets. If feedback already reports CAN control, it sends
no frames. Check motor enable feedback afterward; CAN mode alone is insufficient.

## Offline replay

`offline_replay.py` consumes JSONL events with `snapshot`, `response`,
and laptop-monotonic `now_ns` fields. It passes each row to the pure safety
validator and prints acceptance or rejection reasons. `FakeRobotTransport`
provides recorded rows and raises `PermissionError` on any send attempt.
The replay client never calls `send`.

Run it in the dedicated Conda environment with a reviewed offline policy
JSON and a local event file:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.robot_client.offline_replay `
  --events <events.jsonl> --policy <offline-policy.json>
```

The policy JSON must contain the fields of `SafetyPolicy` in
`laptop/safety/offline_guard.py`. Test fixtures can supply bounds, but they
do not prove the real robot's limits. This tool imports no camera, network
policy client, PiPER SDK, or CAN transport. A passing replay does not enable
robot control.

## HIL entrypoint gate

Legacy supervised HIL scripts call the shared `hil_guard` before importing or
opening a CAN adapter. The guard requests a structured rejection event from
`SafetyBridge` and raises `LegacyHILBlockedError`; the event records that the
HIL mode and live gates are unavailable, with `can_tx_attempted=false`. This
does not mean that a command-capable policy HIL bridge has been implemented.
The policy and legacy HIL paths remain blocked unless their own gates pass.

## Fixed J1 commissioning step

`supervised_j1_commissioning.py` is a separate one-step calibration route. It
accepts a selected +0.100-degree or +1.000-degree J1 target from the measured current pose,
holds J2-J6 at their measured positions, and does not command the gripper. A
zero pose is not required. It requires fresh bridge-received feedback, all six
joints within conservative official SDK nominal bounds, `CAN_CTRL`, all six
motors already enabled, at least 300 ms of feedback with no joint drifting more
than 0.050 degree, three explicit onsite checks, and a typed confirmation
immediately before writing. These software bounds are not physical hard stops.
The 0.100-degree route does not configure mode. The 1.000-degree route prefixes
the joint frames with the fixed CAN/MOVE_J/5% mode setting. Neither enables motors.

The command is not part of policy rollout and does not establish that policy
units or mapping are correct. Do not run it until the arm is physically clear,
the low-speed setting and E-stop are confirmed onsite, and a human explicitly
authorizes this exact one-step test. Keep a hand at the physical E-stop. Closing
USB does not stop the arm; if feedback or motion is unexpected, use the physical
E-stop, inspect feedback, and do not retry an ambiguous write.

The live invocation is deliberately separate from tests:

```powershell
E:\anaconda3\envs\pi05_laptop\python.exe -B -m laptop.robot_client.supervised_j1_commissioning `
  --commissioning-j1-plus-0.1deg --physical-estop-ready `
  --low-speed-confirmed --motion-envelope-cleared
```

The operator must then inspect the printed current/target values and type the
exact confirmation phrase shown by the program.

## Reusable CAN tool

`python -B -m laptop.robot_client.piper_tool` reuses the existing bridge and
transport through subcommands: `status`, `motor-status`, `mode-init`,
`enable-j6`, `step-j1`, `step-joint`, and `step-gripper`. Each invocation requires a new `--output` report
path. An existing report is refused before device access. Reports include UTC
start/end times; sending operations retain their existing monotonic write
timestamps and feedback checks. Exceptions are logged without automatic retry.
The tool opens no cameras. No arbitrary CAN payload sender is exposed.

Read-only examples (normal receive mode acknowledges the bus):

```powershell
python -B -m laptop.robot_client.piper_tool status --read-approved --output validation/status_new.json
python -B -m laptop.robot_client.piper_tool motor-status --read-approved --output validation/motors_new.json
```

Run these from the project root with the `pi05_laptop` Python environment.
`mode-init` requires `--operator-authorized-reset --zero-pose-supported
--physical-estop-ready --motion-envelope-cleared`; `enable-j6` uses
`--operator-authorized-enable` with the same site checks. The reset operation
can remove holding torque. These flags must reflect actual operator approval
and current site conditions.

For a separately authorized single J1 test, select `step-j1 --degrees 0.1`
or `step-j1 --degrees 1`, add `--physical-estop-ready --low-speed-confirmed
--motion-envelope-cleared --output <new-report.json>`, and enter the exact
confirmation phrase after inspecting the target. The step is relative to the
current measured pose, not an absolute angle. Re-running sends another step;
prior one-step authorization does not authorize repeated invocations.

## Persistent Python communication API

Use `PiperSession` from `laptop.robot_client.piper_session` for collection and
control applications that need one persistent CAN connection. The class owns
the connection lifecycle and writes one exclusive JSONL log for the session.
Call it from one thread. Receiving is synchronous: call `poll()` regularly.

```python
from laptop.robot_client.piper_session import PiperSession

with PiperSession("validation/session_new.jsonl") as robot:
    robot.poll(0.6)
    feedback = robot.state()
    print(feedback)
```

The default session receives only. `state()` reports missing or stale channels,
per-channel monotonic timestamps, joint channel skew, control mode, error code,
six joint positions, gripper position/effort/status, and all motor enable states.
Values remain nominal SDK units; a cached position is not automatically fresh.

For control, pass the already configured `SafetyBridge` as `bridge=`. Its mode,
limits, and TX configuration determine which operations are available. You can
call `send_position(snapshot=..., batch=..., authorization=...)` with the
existing `RobotStateSnapshot`, `PositionCommandBatch`, and
`SupervisedPositionAuthorization` types. This supports six joint and gripper
absolute targets and preserves the existing site, units, and proposal checks.
`send_j1()` accepts the existing commissioning types; `commissioning_snapshot()`
captures the bridge-owned commissioning feedback.

Other methods are `send_query(query, operator_approved=...)`,
`set_mode_stage(stage, **site)`, `enable_j6(**site)`, `watchdog_tick()`, and
`wait_joint_target(target_raw, timeout_s=..., tolerance_raw=...)`.
Target waiting receives fresh feedback and checks six joint targets; it does
not resend commands or verify a gripper target. Every RX frame is logged with
its source timestamp and raw payload. TX requests, receipts, failures, and
target-check results are also logged. Adapter acceptance remains distinct from
feedback matching. Close is idempotent and does not claim a physical stop.

This API currently covers the implemented position-control protocol. Cartesian
control, arbitrary CAN payloads, general motor enable/disable, and an autonomous
policy loop are not implemented here. Mode initialization and position control
use separate configured sessions because the existing bridge modes are mutually
exclusive. The persistent API has offline coverage; no new live motion was
performed to test it.

## Joint and gripper commissioning extensions

For one separately approved remaining-axis test, use `piper_tool step-joint
--axis 2 --physical-estop-ready --low-speed-confirmed --motion-envelope-cleared
--output validation/j2_new.json`. Select an axis from 2 through 6. J3 uses −1°;
the other selected axes use +1°. Each target is relative to fresh measured
feedback, with the other axes held. The tool requires the printed exact scope
phrase. Historical J1 class and mode names remain for compatibility; the
validated batch explicitly binds `selected_axis` and its one permitted step.
The default J1 interface retains its previous restrictions.

`supervised_gripper_commissioning.execute(...)` reuses `PiperSession` to send
one gripper-only opening and log raw RX/TX data. `opening_mm=1` selects nominal
0.1 N·m effort; `opening_mm=5` selects nominal 0.3 N·m effort and a different
confirmation phrase. Neither sets a zero point or sends joint targets. These
are separate operator authorization scopes. No retry is performed. The current
software target envelope is 0–70 mm; it is not a measured physical travel limit.
`PiperSession.open_gripper_step()` exposes the same typed, bounded path for
applications; its default is the 1 mm scope.

October 7 live evidence: each remaining joint responded to its selected command.
J2's transient aggregate flag remains unresolved despite stable later feedback
and an operator report of normal motion. The first 1 mm gripper attempt produced
no measured position change. Therefore complete gripper mapping and autonomous
policy deployment are still pending. A subsequently approved +5 mm / 0.3 N·m
test changed gripper feedback from nominal 0.8 mm to 5.4 mm for a 5.8 mm target.
This establishes position response while retaining a 0.4 mm target error.
The initial 1 mm attempt may have settled later; do not describe the gripper as
completely unresponsive. Use `piper_tool step-gripper --opening-mm 1` or `5`
with the same site flags, a new output path, and its exact confirmation phrase
only for a newly authorized attempt. A separate raw CAN JSONL is saved beside
the report. See the current project plan and local
validation reports for exact values and authorization history.

## Phase A.5 retained legacy registry

当前操作入口见 [RUNBOOK](../../docs/RUNBOOK.md)。以下不是当前机器人运行入口，正常执行首先由 legacy guard 拒绝；仅为历史行为回归和来源保留。不得通过绕过 guard 使用它们。

| File | 保留原因 |
|---|---|
| `supervised_j1_step_trial.py` | Observed 多帧聚合、preflight 被其他 legacy 和测试依赖 |
| `supervised_j1_one_degree_trial.py` | 独立起始姿态与授权回归 |
| `supervised_j5_probe.py` | 历史起始状态、授权回归 |
| `supervised_joint_replay.py` | legacy guard 与历史回放复现；没有完整等价替代 |
| `supervised_restore_gripper.py` | 历史夹爪目标与授权回归 |
| `supervised_restore_start.py` | checked_feedback、回放规划和 envelope 回归及跨 legacy 依赖 |
| `supervised_smooth_joint_replay.py` | legacy guard 与历史平滑回放复现；没有完整等价替代 |

单帧 feedback decoder 不替代这些历史聚合/校验职责。归档结论见 [A.5 report](../../reports/CODE_CLEANUP_PHASE_A5_REPORT.md)。
