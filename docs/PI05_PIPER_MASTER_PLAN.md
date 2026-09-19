# π0.5 + AgileX PiPER master plan

The original plan history is preserved byte-for-byte in
[`PI05_PIPER_MASTER_PLAN_HISTORY.md`](PI05_PIPER_MASTER_PLAN_HISTORY.md) and
[`PI05_PIPER_MASTER_PLAN_ORIGINAL.md`](PI05_PIPER_MASTER_PLAN_ORIGINAL.md).
This file records the P0/P1 execution status without rewriting that history.

## Deployment execution log — 2026-09-19

### P0 official OpenPI deployment

- Official source only: `https://github.com/Physical-Intelligence/openpi.git`.
- Frozen OpenPI commit: `215abfb217dbac7d5f1273282331b9b1866c0479`.
- Submodules were cloned recursively and recorded in `provenance/openpi_git.json`.
- Project-local Python 3.11.16 uv environment: `.venv/`.
- Project-local cache roots: `cache/huggingface`, `cache/openpi`, `cache/uv`,
  `cache/xdg`.
- RLDS dependency group was not installed.
- GPU0 selected; GPU3 remained untouched while its existing Qwen/SGLang
  workload continued.
- `pi05_base` was downloaded into the project-local OpenPI cache. Its 29-file,
  11.59 GiB inventory and SHA-256 values are in
  `provenance/pi05_base_checkpoint.json`.
- Official `pi05_aloha + pi05_base` load/inference smoke passed 3/3 finite
  responses with action shape `[50, 14]`. This is an ALOHA base sanity, not a
  PiPER policy result.

### P1 local policy server and contract bootstrap

- `server/serve_policy_local.py` wraps the official server class and hard-codes
  `127.0.0.1:8000`; no public WebSocket binding is allowed.
- `/healthz`, metadata, 100 sequential official-client requests, and clean
  shutdown passed. Binding evidence is in `validation/socket_binding.txt`.
- Server performance baseline passed with 25 requests and 24 warm requests;
  see `reports/pi05_server_performance.json`.
- `PIPER_STATE_V1` is frozen semantically as
  `[q1,q2,q3,q4,q5,q6,gripper]` with radian/meter canonical units.
- `PIPER_ACTION_V1` is frozen as absolute joint position with the same 7D
  ordering; no delta or end-effector action was introduced.
- Pure conversion, padding/unpadding, image preprocessing, and dtype/shape
  tests passed. No SDK, CAN, camera, or robot call was made.
- Official AgileX nominal limits are pinned to the reviewed `piper_sdk` commit
  in `configs/piper_joint_limits.yaml` and are labelled non-confirmed firmware
  limits.
- Teach-mode programmatic entry remains `UNKNOWN`; gripper-during-teach is
  `UNKNOWN` and requires a future user hardware audit.
- Laptop package, SSH tunnel instructions, camera/network probes, and static
  read-only probe audit are ready for P2. They were not run against a laptop.

### Frozen nonclaims

```text
REAL_PIPER_CONNECTED = NO
REAL_ROBOT_MOTION = NO
REAL_DATA_COLLECTION = NO
FORMAL_FINE_TUNING = NO
Tailscale = NOT_INSTALLED
P2_LAPTOP_HARDWARE_AUDIT = NOT_EXECUTED
```

The next allowed phase is P2 laptop hardware audit. It must not be inferred
from these server-side passes, and this execution log does not authorize CAN,
teach episodes, real demonstrations, or autonomous motion.
