# 2026-10-21 work summary

This page summarizes the current gamepad capture and LeRobot training work.
It separates verified software and training results from the field checks that
remain open. Raw demonstrations, camera recordings, and CAN logs stay local.

## Completed software work

- **Gamepad control:** Cartesian and joint teleoperation now use the revised
  stick mappings and target smoothing. A/B/X/Y actions are disabled. START
  changes control mode, LB selects a speed tier, LT/RT control the gripper,
  and RB ends the current recording.
- **Control and homing handoff:** the home flow pauses the executor and drains
  in-flight requests before using the existing HomeArm service. After a
  successful home, the executor waits for a new target. Unknown delivery or a
  real control fault still stops execution.
- **Recording workflow:** one recorder process can handle multiple episodes.
  START/NEXT requests the existing home flow before capture; RB/STOP ends an
  episode; the operator enters SUCCESS or FAILURE; NEXT waits for save
  completion. `-SingleEpisode` remains available, and `-Review` opts into
  contact sheets. Capture still requires complete ROS and camera preflight.
- **CAN mode before recording:** HomeArm now checks for CAN_CTRL/MOVE_J before
  returning to zero. It reuses the single existing CAN owner. It does not
  enable motors, clear faults, reset the arm, or change encoder zero.
- **Training labels:** live samples use measured feedback nearest to
  `t+100 ms`, within the accepted 75–125 ms interval. The label is
  `next_observed_state_proxy_nominal_units`; it is not the CAN command target.
  The final observation has no future action row. Command evidence is retained
  separately, and invalid labels remain masked.
- **Recorder input reliability:** repeated diagnostics are coalesced while
  critical lifecycle, stop, disconnect, and error events remain queued. The
  terminal label editor supports Backspace, Delete, and cursor movement.
- **Training run:** the 10-episode dataset view retained 5,684 rows in 21
  continuous segments, with 4,599 training rows and 1,085 validation rows.
  The LeRobot 0.6.1 PI05 LoRA run completed 30,000 steps and saved checkpoints
  from 5,000 through 30,000 steps.

## Verification and limits

Offline verification recorded for this work includes 19 lifecycle/stdio checks,
23 pairing checks with one skipped, 29 home-handoff checks, 24 terminal-edit
checks, and 18 CAN-mode transition checks. The recorder workflow and synthetic
LeRobot write/read path also passed their focused checks. These results verify
software paths and data contracts; they do not establish task success on the
robot.

The LoRA run has not yet passed a held-out task evaluation or a physical robot
trial. Pairing records trajectory segment boundaries, but the training input
path must continue to prevent action chunks from crossing invalid gaps.

## Current field blockers

- The latest recorder output reached `ROS_READY` but rejected capture because
  Base and Wrist camera identities were missing. Camera publishers were then
  restarted and both streams were observed, but a subsequent `CAPTURE_READY`
  has not been recorded here. Treat capture readiness as unconfirmed until the
  recorder reports it.
- Physical recovery after the reported collision is not verified by camera
  readiness or software preflight. Do not treat either as proof that the arm is
  ready to move or collect demonstrations.
- Recheck the current robot status and fault recovery path before any motion.
  The recording startup path deliberately does not enable or reset the arm.

## Next steps

1. Confirm Base and Wrist identities in the active recorder and wait for
   `CAPTURE_READY`.
2. Recheck the arm with the existing read-only status path. Resolve the joint
   fault through a verified recovery procedure before enabling motion.
3. After the robot is cleared, perform a short supervised control check, then
   record and review a complete demonstration.
4. Evaluate the saved LoRA checkpoint on the held-out episodes before planning
   a low-speed physical trial.
