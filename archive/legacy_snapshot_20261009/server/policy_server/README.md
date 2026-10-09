---
Document status: REFERENCE
Current authority: docs/ARCHITECTURE.md
Last reviewed: 2026-10-08
---

# Policy server support reference

此目录提供服务相关支持/契约，当前实际启动入口是
`server.training.serve_trained_demonstrations`，不是待实现的 policy server。
架构 authority 是 [ARCHITECTURE](../../docs/ARCHITECTURE.md)，操作见
[RUNBOOK](../../docs/RUNBOOK.md)。

当前 wrapper 读取 COMPLETE training report、审计 dataset/norm/checkpoint 和
OpenPI 来源，生成 identity lock，调用 multi-episode derived-state 服务。
服务只做观察到预测，绑定 loopback；客户端可另经 typed Bridge/CAN 执行受监督目标。

`prediction_only=true` / `hardware_control_allowed=false` 只表达服务没有机器人
控制权限。不要据此把已记录的 model→CAN 实验改写为“系统没有发送链路”。
旧proxy/base服务保留历史复现用途，不作为当前默认启动入口。
