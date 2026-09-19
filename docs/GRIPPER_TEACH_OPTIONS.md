# Gripper teach options

Status: `HARD-GATE / USER_DECISION_REQUIRED_AFTER_HARDWARE_AUDIT`

The gripper is not inferred from video after the fact. A training-eligible
episode must contain synchronized gripper target and actual state, with units
and limits sourced from the current PiPER audit.

| ID | Input | Advantages | Risks | Decision status |
|---|---|---|---|---|
| G-A | laptop keyboard key | fastest bring-up | awkward while dragging; key focus and operator timing | pending |
| G-B | USB foot pedal/external button | hands remain on arm; explicit events | extra hardware and debouncing audit | pending |
| G-C | official PiPER teach control | hardware-native if supported | support and API semantics may be firmware-specific | pending |

## Audit questions

1. Can drag teaching remain active while software or official controls operate
   the gripper?
2. Can target and actual gripper values be read at the same timestamp domain?
3. What are the current firmware's units, limits, control modes and safe hold
   behavior?
4. Can an open/close event be recorded even if the arm is manually dragged?
5. What action should the SafetyBridge issue when gripper feedback is stale?

Until these questions are answered on the actual robot, P4 may specify the
recorder but may not declare training-ready episodes.
