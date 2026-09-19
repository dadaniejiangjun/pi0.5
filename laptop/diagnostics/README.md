# Laptop diagnostics

The Python files in this directory are safe specification stubs for the first
planning turn. They emit their contract and a `NOT_RUN` result; they do not
open CAN, move the arm, enable/disable the robot, or guess camera identity.

Later implementation must preserve that default and add separately reviewed,
read-only probes for the actual laptop hardware.
