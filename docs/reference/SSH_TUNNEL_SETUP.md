---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

# SSH tunnel setup reference

当前具名 profile、端口和完整操作唯一见 [RUNBOOK](../RUNBOOK.md)。本页只保留方法说明。

服务绑定服务器 loopback，Windows SSH `-L` 将选定本地端口转发到服务端口。
使用 `ExitOnForwardFailure=yes` 检查绑定失败；同时检查服务identity和一次shadow请求，
SSH连接成功不等于模型已经可响应。不要将裸 WebSocket 直接暴露公网。

HISTORICAL：旧 8000/8002 是早期 base/proxy profile 例子；不能拿这些端口替代
RUNBOOK 的 1000/5000-step comparison profiles。切换模型时使用对应 training report
和新生成的 service lock，不只更改端口或文件名。
