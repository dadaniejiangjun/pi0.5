---
Document status: REFERENCE
Current authority: docs/SAFETY_AND_CONTROL.md
Last reviewed: 2026-10-08
---

# Safety implementation reference

SafetyBridge 已实现且被当前 bounded trial 调用，不再是仅规划的组件。
安全/control authority 是 [SAFETY_AND_CONTROL](../../docs/SAFETY_AND_CONTROL.md)。

当前路径使用 commissioning 模式、snapshot 和 typed batch，配合 transport 发送
受监督目标。Formal supervised 使用另一套 P12证据/verified mapping和授权，不能
由 commissioning 成功自动宣告正式gate通过。

`offline_guard` 的速度/加速度检查有实现，但没有接入当前 bounded loop。
独立连续watchdog尚未实现；失联/进程崩溃后的物理safe-hold和硬件停止尚未验证。
禁止将服务的 `prediction_only=true` 或退出进程解释为整个机器人不会动/已经停稳。
