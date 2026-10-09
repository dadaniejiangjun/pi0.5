---
Document status: CURRENT
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

# PiPER × π0.5 real-robot deployment

本项目使用 π0.5 控制 AgileX PiPER，重点是实际机器人 VLA 部署。
hydrogel tactile / VTLA 是后续扩展方向，当前阶段聚焦视觉、状态与语言条件的拾放。

## Project goal

让机械臂根据相机、反馈与文本完成黑棒到绿色桌垫的拾放，并保留数据、训练、
推理、命令和实机结果的可追溯证据。

## Current status

当前状态唯一以 [CURRENT_STATUS](docs/CURRENT_STATUS.md) 为准，以下只作入口摘要。

| 能力 | 已核对范围 |
|---|---|
| Data acquisition / 10 Hz proxy dataset | VERIFIED：3 条成功示教、595 帧，operator-assisted |
| π0.5 1000 / 5000-step training | VERIFIED：已完成；另有 50K 实验的运行快照，见状态页 |
| WebSocket inference | VERIFIED |
| Model → Bridge → CAN → feedback | VERIFIED / SUPERVISED：受监督实验闭环 |
| Continuous scheduler | NOT IMPLEMENTED |
| Physical safe-hold on failure | NOT VERIFIED |
| Autonomous pick-place success | NOT VERIFIED |

## Current architecture

当前职责分工如下，详细调用链见 [ARCHITECTURE](docs/ARCHITECTURE.md)。

```text
Windows Laptop (RGB / feedback)
  → SSH / WebSocket → Linux GPU Server / π0.5
  → action prediction → Windows Laptop / SafetyBridge
  → CAN → PiPER → feedback
```

## Documentation

下列文档分别负责一个当前领域；其他文档是指南、规格或历史证据。

| Topic | Authority |
|---|---|
| Status | [CURRENT_STATUS](docs/CURRENT_STATUS.md) |
| Architecture | [ARCHITECTURE](docs/ARCHITECTURE.md) |
| Action semantics | [ACTION_SEMANTICS](docs/ACTION_SEMANTICS.md) |
| Safety / control | [SAFETY_AND_CONTROL](docs/SAFETY_AND_CONTROL.md) |
| Data / training | [DATA_AND_TRAINING](docs/DATA_AND_TRAINING.md) |
| Operational commands | [RUNBOOK](docs/RUNBOOK.md) |
| Formal phase criteria | [PHASE_GATES](docs/PHASE_GATES.md) |

目录见 [PROJECT_LAYOUT](docs/reference/PROJECT_LAYOUT.md)，历史索引见
[history](docs/history/README.md)，整理结果见
[DOCUMENTATION_CLEANUP_REPORT](docs/reference/DOCUMENTATION_CLEANUP_REPORT.md)。

## Important safety notice

明确标为 HIL / supervised 的入口可能发送真实机器人命令。
`prediction_only=true` 描述推理服务，不能解释为整个系统不会运动。
现场授权、检查与停止边界见 [SAFETY_AND_CONTROL](docs/SAFETY_AND_CONTROL.md)。

## Repository status

本地源码、配置及 validation/runtime reports 是项目证据主源。
GitHub 与本地不完全同步；HEAD 不是未提交源码的完整身份。
历史报告、初始契约与旧计划保留作 provenance，不代表当前运行状态。
本轮只整理文档；不修改、迁移或启动机器人/训练代码。

当前链路代码清单见 [ACTIVE_CODE_INDEX](docs/ACTIVE_CODE_INDEX.md)。历史实现与测试保存在 [archive](archive/README.md)，不参与默认测试发现。
