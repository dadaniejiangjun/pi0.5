---
Document status: REFERENCE
Current authority: docs/ARCHITECTURE.md
Last reviewed: 2026-10-09
---

# Project layout

本文件映射当前真实目录职责，架构唯一以 [ARCHITECTURE](../ARCHITECTURE.md) 为准。
CURRENT/MIXED/HISTORICAL/GENERATED 是目录用途分类，不是测试通过状态。

## Current layout

目录内可能同时包含当前入口、历史 helper 与生成物，不能按目录名整体删除。

| 目录 | 类别 | 实际职责 |
|---|---|---|
| `laptop/camera` | CURRENT | DirectShow设备枚举、双路采集、公共相机源 |
| `laptop/client` | MIXED | live preview / bounded model trial；当前观察、policy helper 与实时聚合；旧网络/实验入口已归档 |
| `laptop/robot_client` | MIXED | PiperSession、piper_tool、commissioning、回零；当前 bounded / commissioning 支持；7 个旧 sender 已归档 |
| `laptop/safety` | CURRENT | SafetyBridge、typed 授权与状态检查；部分 guard 不在当前 bounded loop 生效 |
| `laptop/runtime` | CURRENT | snapshot、batch、正式/commissioning 类型及运行期证据 |
| `laptop/transport` | CURRENT | gs_usb / candleLight transport、协议编码、typed CAN TX |
| `laptop/recorder` | CURRENT | 可复用人工示教采集、会话和结果标注；反馈监督而非原厂命令监督 |
| `laptop/piper` | MIXED | unit conversion 等 helper；实际 session 在 robot_client |
| `laptop/diagnostics` | MIXED | 只读/受批准查询、OEM采集与历史探针 |
| `server/training` | MIXED | train/evaluate/serve_demonstrations、loss工具；当前 multi-episode 底层依赖；旧训练分支已归档 |
| `server/policy_server` | MIXED | artifact identity 支持；实际服务封装在 training |
| `server/dataset_tools` | MIXED | raw对齐、future-state proxy和LeRobot转换；旧 command-label/采样比较实验已归档 |
| `configs` | MIXED | 机器契约、模板、模型初始意图；并非每个配置都描述当前部署 |
| `docs` | CURRENT | 六份当前领域文档、formal gates及导航 |
| `docs/reference` | MIXED | 规格、审计、路线图、整理报告与参考索引 |
| `docs/history` | HISTORICAL | 10份历史计划/决策文档、弃用重复副本、历史阶段JSON及索引 |
| `reports` | GENERATED | 按日期的诊断/部署/来源报告，保留，不当作实时状态 |
| `validation` | GENERATED | 本地实验日志、identity、训练/评估及实机试跑证据；全部顶层历史 runner 已归档；运行证据保留 |
| `tests` | MIXED | contracts / laptop / server 等测试与历史回归；Phase A.5 仅运行纯软件回归 |
| `scripts` | MIXED | 审计/验证/报告工具；全部历史脚本已归档；当前正式工具使用 package 入口 |
| `archive` | HISTORICAL | 审核后的历史源码；active code 不得依赖；见 [manifest](../../archive/ARCHIVE_MANIFEST.md) |
| `datasets` | GENERATED | 本地 raw / manifests / 转换数据，真实采集默认不发布 |
| `cache` / `checkpoints` | GENERATED | 运行副本、缓存、模型产物，本地/服务器保留 |
| `third_party/openpi` | CURRENT | 官方 Physical-Intelligence/openpi pinned submodule，不能替换成fork |
| `provenance` | MIXED | 官方pin与来源事实；早期planning快照原样保留 |

本地 root 和服务器隔离 `runtime_root` 不是自动同步的同一 checkout。
机器路径/profile唯一见 [RUNBOOK](../RUNBOOK.md)，来源 pin 见
[OPENPI_PROVENANCE](OPENPI_PROVENANCE.md)。

## Proposed future structure

下面仅是 PROPOSED，**NOT IMPLEMENTED**，Phase A.5 仅归档四份历史脚本，没有进行整体目录迁移。

```text
src/       reusable library
apps/      explicit operational entrypoints
tools/     offline audits and conversions
tests/     regression and contract tests
configs/   versioned machine contracts
docs/      current authorities and history
archive/   reviewed historical source / deprecated scripts
```

任何迁移需先检查 imports、tests/helper 和报告引用。旧 direct sender 不因当前
入口被阻断就自动可删除。归档候选见 [history](../history/README.md)。

当前源码逐文件清单见 [ACTIVE_CODE_INDEX](../ACTIVE_CODE_INDEX.md)。本轮整组归档历史实现和历史测试；前次 A.5 保留结论是较早快照。
