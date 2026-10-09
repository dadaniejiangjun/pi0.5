---
Document status: HISTORICAL
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

> **HISTORICAL**：以下正文保留当时计划、决策或假设，不描述当前部署。pending/未实现/未选择等状态仅限原日期，不能覆盖新的运行证据。 当前领域主文档见 [authority](../CURRENT_STATUS.md)。

# Initial π0.5 checkpoint decision

Status: `USER_DECISION_REQUIRED / RECOMMENDATION_ONLY`

## Candidates

| ID | Candidate | Intended role | Recommendation | Blocking concern |
|---|---|---|---|---|
| CKPT-A | `pi05_base` | General π0.5 base checkpoint/assets | Recommended for a new PiPER embodiment | Exact checkpoint path, license/access, input/action assumptions and local commit must be verified |
| CKPT-B | `pi05_droid` | DROID-configured π0.5 checkpoint | Secondary comparison only | DROID camera/state/action semantics may not match PiPER |

## Planned decision rule

Use CKPT-A as the initial candidate only after:

1. P0 confirms the official checkpoint identity and source commit;
2. P1 loads the checkpoint with a dummy observation without NaN/Inf;
3. the PiPER transform contract is explicit;
4. the PiPER-specific normalization stats are available or the model path
   explicitly documents how they are obtained;
5. the output is inspected offline before any robot-facing path exists.

CKPT-B may be used as a diagnostic comparison, but a DROID pass is not a PiPER
deployment result. The user must make the final selection in `USER_DECISIONS.md`.
