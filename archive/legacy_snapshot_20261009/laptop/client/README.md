---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

# Laptop model client reference

当前职责是构造实时 observation、核对服务identity、请求预测，并按明确选择的
shadow或受监督模式执行。操作命令唯一见 [RUNBOOK](../../docs/RUNBOOK.md)。

## Current entrypoints

以下两个可复用入口按不同权限运行。

| Module | 行为 |
|---|---|
| `live_model_preview` | 自动解析已确认相机角色，复用live_shadow worker，只读预测与限幅预览 |
| `bounded_model_trial` | 受监督commissioning，选定proxy行→bounded target→Bridge/CAN→反馈检查；可真实运动 |

当前具名1000/5000 profiles在RUNBOOK维护，不按旧README中的8000/8002选择模型。
这些旧端口仅为 HISTORICAL profile；底层模块默认端口不代表选定部署参数。
模型shape/单位见 [ACTION_SEMANTICS](../../docs/ACTION_SEMANTICS.md)。

## Retained support

旧shadow、loopback、网络探针和helper保留测试/复现价值。
当前主循环还没有连续scheduler、完整chunk消费或平滑。
`live_model_preview` 使用 `live_shadow._worker` 的私有helper，是后续整理候选；
本轮没有重构或改变import关系。
