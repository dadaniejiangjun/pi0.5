---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

# Documentation index

以下七份文档分别负责一个当前领域；本索引和根 README 不独立维护项目状态。

| Topic | Current authority |
|---|---|
| 状态 | [CURRENT_STATUS](CURRENT_STATUS.md) |
| 实际架构 | [ARCHITECTURE](ARCHITECTURE.md) |
| 输入输出 / action 语义 | [ACTION_SEMANTICS](ACTION_SEMANTICS.md) |
| 生效安全机制与控制 | [SAFETY_AND_CONTROL](SAFETY_AND_CONTROL.md) |
| 数据与训练 | [DATA_AND_TRAINING](DATA_AND_TRAINING.md) |
| 操作命令 / profiles | [RUNBOOK](RUNBOOK.md) |
| 正式 phase 标准 | [PHASE_GATES](PHASE_GATES.md) |

## Reference and history

规格、审计和计划各有不同职责，不能用历史状态覆盖新的执行证据。

- [PROJECT_LAYOUT](reference/PROJECT_LAYOUT.md)：实际目录地图，未来结构单独注明。
- [PROJECT_CODEBASE_AUDIT](reference/PROJECT_CODEBASE_AUDIT.md)：2026-10-08代码与证据审计快照。
- [REDUNDANCY_CANDIDATES](reference/REDUNDANCY_CANDIDATES.md)：审计时候选清单，不是删除许可。
- [CURRENT_PROJECT_PLAN](reference/CURRENT_PROJECT_PLAN.md)：保留的路线图与按日期记录的执行历史。
- [history](history/README.md)：旧计划、规格、报告与归档候选索引。
- [decisions](decisions/README.md)：决策来源及尚需人工决策的边界。
- [DOCUMENTATION_CLEANUP_REPORT](reference/DOCUMENTATION_CLEANUP_REPORT.md)：本轮整理与一致性检查。

`CURRENT` 是各领域主文档；`REFERENCE` 是技术参考；`HISTORICAL` 是日期限定记录；
`TEMPLATE` 不表示当前部署；`DEPRECATED` 保留兼容/来源价值但不作为当前入口。
机器执行身份仍由 config、report、provenance 与 identity 决定。

## Documentation folders

根目录保留7份CURRENT主文档与本导航。

- [reference](reference/README.md)：技术参考、审计、路线图与整理记录。
- [history](history/README.md)：历史计划、弃用副本和历史阶段JSON。
- [decisions](decisions/README.md)：决策来源索引。

当前实际源码入口与依赖见 [ACTIVE_CODE_INDEX](ACTIVE_CODE_INDEX.md)。代码保存与本地运行产物的边界见 [GITHUB_SOURCE_SNAPSHOT](reference/GITHUB_SOURCE_SNAPSHOT.md)。
