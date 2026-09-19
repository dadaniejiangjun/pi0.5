# PiPER gripper during teach-mode audit (P0/P1)

Status: `G_C_OFFICIAL = UNKNOWN`; `USER_HARDWARE_TEST_REQUIRED`.

The official V2 interface documents `GripperCtrl` and documents teach-mode
feedback, but the reviewed material does not explicitly establish that an
independent gripper control call is supported and safe while the arm is in
Teach Mode. It is therefore not marked supported or unsupported by inference.

The primary future choice remains the official PiPER teach/gripper interface.
The external operator-input fallback is not selected now; it is conditional on
a future hardware audit proving the primary interface unavailable. No gripper
call or real hardware test was performed in P0/P1.

Source: https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD
