# Laptop setup plan

The laptop is the robot authority and the only machine permitted to access
PiPER CAN/ROS/SDK. This document is a future setup plan, not a command to
install or connect hardware in the planning turn.

## Planned components

```text
laptop/
  robot_client/  observation construction, request IDs, telemetry
  recorder/      local-first raw episode writer
  camera/        camera discovery/capture and model preprocessing
  piper/         feedback adapter and later command adapter
  safety/        PiperSafetyBridge and watchdog
  diagnostics/   read-only probes and reports
```

The laptop needs only the lightweight `openpi-client`, validated camera driver,
PiPER driver/SDK, recorder and safety components. It does not need training
weights or the server training environment.

## Bring-up order

1. Run `probe_piper.py` in read-only audit mode and save firmware/SDK/protocol,
   units, ranges, modes, feedback frequency and teach support.
2. Run `probe_cameras.py`; identify both cameras by USB identifier, model and
   serial, then verify native streams and intrinsics.
3. Run camera/state timestamp and dropout checks without robot commands.
4. Install/verify the official OpenPI client pinned to the server source
   contract.
5. Connect to the selected private network mode with dummy data only.
6. Shadow real observations and verify response metadata; robot execution is a
   later gate.

## Local-first retention

The laptop keeps closed raw episodes after successful upload and SHA
verification. Deletion/retention is a user decision. The laptop never receives
checkpoints unless a future explicit local-inference decision changes scope.
