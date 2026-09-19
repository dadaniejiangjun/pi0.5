# P0 + P1 deployment report

Date: 2026-09-19
Project: `/mnt/ssd/users/mingkai/07_pi0.5`

## Gate summary

| Gate | Result | Evidence |
|---|---|---|
| `OPENPI_ENV` | `PASS` (Torch warning retained) | `reports/environment_audit.json` |
| `PI05_BASE_LOAD` | `PASS` | `reports/pi05_base_inference_smoke.json` |
| `PI05_INFERENCE_SMOKE` | `PASS` | 3/3 finite, shape `[50,14]` |
| `LOCAL_POLICY_SERVER` | `PASS` | `reports/local_policy_server_smoke.json` |
| `SOCKET_BINDING_LOCAL_ONLY` | `PASS` | `validation/socket_binding.txt` |
| `LOCAL_CLIENT_ROUNDTRIP` | `PASS` | `reports/local_client_roundtrip.json`, 100/100 |
| `PIPER_STATE_CONTRACT` | `PASS` | `configs/piper_state_v1.yaml` and synthetic tests |
| `PIPER_ACTION_CONTRACT` | `PASS` | `configs/model/pi05_piper_v1.yaml` and padding tests |
| `UNIT_CONVERSION_TEST` | `PASS` | `validation/piper_unit_roundtrip.json` |
| `LAPTOP_READONLY_TOOLS` | `PASS` | `validation/laptop_probe_static_audit.json` |

## Required answers

1. **OpenPI exact commit:** `215abfb217dbac7d5f1273282331b9b1866c0479`.
2. **Environment successful:** Yes for the validated official JAX path;
   Torch reports an explicit `sm_120` compatibility warning and is not claimed
   as the serving backend.
3. **`pi05_base` downloaded:** Yes, project-local cache; 29 files, about
   11.59 GiB, SHA-256 inventory recorded.
4. **Official model load:** Yes.
5. **Official inference:** Yes, 3/3 finite ALOHA sanity responses.
6. **Warm inference latency:** Official smoke warm median `61.76 ms`; server
   baseline median `59.70 ms`, p90 `60.38 ms`.
7. **Peak VRAM:** `8881 MiB` observed on GPU0 during the server baseline.
8. **Local policy server:** Yes; health, metadata, inference, and clean stop
   passed.
9. **Localhost-only:** Yes, observed `127.0.0.1:8000`; no public bind.
10. **100-request roundtrip:** `100/100` valid responses, no failures.
11. **`PIPER_STATE_V1` frozen:** Yes, semantic 7D
    `[q1,q2,q3,q4,q5,q6,gripper]`, rad/rad/rad/rad/rad/rad/meter.
12. **`PIPER_ACTION_V1` frozen:** Yes, absolute joint position, 7D; no delta
    or EE-delta semantics.
13. **Unit conversions tested:** Yes, 10,000 continuous random samples per
    joint/gripper conversion; errors stayed within half a quantization unit.
14. **Teach-mode programmatic entry:** `UNKNOWN`; official feedback indicates
    teach mode, but reviewed API evidence does not establish an entry method.
15. **Official gripper-during-teach:** `UNKNOWN`; user hardware test required.
16. **Laptop package ready:** Yes; installable lightweight package and
    read-only tools prepared. Laptop execution is not claimed.
17. **Any real robot accessed:** No.

## Explicit safety status

```text
REAL_ROBOT_ACCESSED = NO
TEST_REAL_MOTION = NO
REAL_PIPER_CONNECTED = NO
REAL_DATA_COLLECTION = NO
FORMAL_FINE_TUNING = NO
REMOTE_TUNNEL = READY_FOR_LAPTOP_TEST
READY_FOR_LAPTOP_P2 = YES
```

The official base action is intentionally retained as ALOHA output and is not
converted into `PiperOutputs`. No claim is made that π0.5 zero-shot controls
PiPER or that the PiPER contract is training-ready.

## Source references

- [Official OpenPI repository](https://github.com/Physical-Intelligence/openpi)
- [Official OpenPI remote inference documentation](https://github.com/Physical-Intelligence/openpi/blob/main/docs/remote_inference.md)
- [Official AgileX PiPER SDK](https://github.com/agilexrobotics/piper_sdk)
- [Pinned AgileX V2 interface source](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/asserts/V2/INTERFACE_V2.MD)
