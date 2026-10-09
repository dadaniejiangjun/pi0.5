---
Document status: REFERENCE
Current authority: docs/ARCHITECTURE.md
Last reviewed: 2026-10-08
---

# PiPER helper package

此目录主要保存 unit-conversion 等 helper，实际 adapter/session 已在
[PiperSession](../robot_client/piper_session.py) 实现。架构 authority 是
[ARCHITECTURE](../../docs/ARCHITECTURE.md)。

当前通过 `robot_client → safety/runtime → transport` 完成受监督 typed TX 与反馈。
不要因本目录缺少完整 driver 就写“PiPER adapter 尚未实现”。
单位与未来状态代理合同见 [ACTION_SEMANTICS](../../docs/ACTION_SEMANTICS.md)，
生效检查见 [SAFETY_AND_CONTROL](../../docs/SAFETY_AND_CONTROL.md)。
