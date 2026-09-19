# PiPER teach-mode audit (P0/P1)

Status: `TEACH_ENTRY = UNKNOWN` for programmatic entry.

The official AgileX `piper_sdk` V2 interface documents `GetArmStatus()` feedback
with `ctrl_mode=0x02` meaning teaching mode. The same interface documents
`ModeCtrl` only with `ctrl_mode=0x00` (standby) and `0x01` (CAN command control);
it does not provide evidence in the reviewed interface page for a safe SDK
method that enters teach mode programmatically. Therefore feedback support is
not treated as entry support.

Default future plan: an operator manually enters Teach Mode, then a separately
approved read-only audit verifies the observed state. No CAN call, teach entry,
or robot connection was attempted in P0/P1.

Sources:

- https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD
