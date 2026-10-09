---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

# Historical documentation and artifact index

旧资料保存当时的假设、选择和结果，原则是 immutable whenever possible。
2026-10-08文档收口后，经用户授权，将10份HISTORICAL文档统一迁入本目录，
保留原正文并修复相对链接；不移动源码、实验报告或provenance。
当前状态唯一见 [CURRENT_STATUS](../CURRENT_STATUS.md)。

## Historical plans and decisions

以下10份文件标为 HISTORICAL；pending/未开始/未接通等表述仅限原日期。

| Document | 保留价值 |
|---|---|
| [NETWORK_OPTIONS](NETWORK_OPTIONS.md) | SSH/VPN初始备选与待选择状态 |
| [USER_DECISIONS](USER_DECISIONS.md) | 初始用户决策清单 |
| [CHECKPOINT_INIT_DECISION](CHECKPOINT_INIT_DECISION.md) | base权重选择的旧待定记录 |
| [REAL_ROBOT_SAFETY_PLAN](REAL_ROBOT_SAFETY_PLAN.md) | 原安全规划，不代表当前生效机制 |
| [REMOTE_ARCHITECTURE](REMOTE_ARCHITECTURE.md) | 初始远程架构设想 |
| [LAPTOP_SETUP_PLAN](LAPTOP_SETUP_PLAN.md) | 初始笔记本环境重建计划 |
| [SERVER_SETUP_PLAN](SERVER_SETUP_PLAN.md) | 初始服务器环境计划 |
| [2026-10-02 diagnosis week plan](project-diagnosis-week-plan-2026-10-02.md) | 当时诊断和周任务 |
| [PI05_PIPER_MASTER_PLAN](PI05_PIPER_MASTER_PLAN.md) | 初始计划整理版 |
| [PI05_PIPER_MASTER_PLAN_HISTORY](PI05_PIPER_MASTER_PLAN_HISTORY.md) | 初始计划全文主副本 |

[CURRENT_PROJECT_PLAN](../reference/CURRENT_PROJECT_PLAN.md) 是 REFERENCE：保留路线图、
按日期的执行历史和既有链接。本轮没有删正文或直接替换成新状态文档。
以后可单独收缩成roadmap，但不能丢失实验来历。

## Deprecated files and completed archival

Phase A.5 已将两份历史 Python 补丁归档，原字节不变；历史文档继续留在 docs/history。

| File | Current location | 当前处理 |
|---|---|---|
| [MASTER_PLAN_ORIGINAL](PI05_PIPER_MASTER_PLAN_ORIGINAL.md) | `docs/history/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | 重复副本保留；整理版链接改到HISTORY |
| [add_final_gate_aliases.py](../../archive/deprecated_scripts/add_final_gate_aliases.py) | `archive/deprecated_scripts/add_final_gate_aliases.py` | 一次性旧manifest补丁，不作为当前gate工具 |
| [annotate_pi05_smoke_metadata.py](../../archive/deprecated_scripts/annotate_pi05_smoke_metadata.py) | `archive/deprecated_scripts/annotate_pi05_smoke_metadata.py` | 固定旧smoke注释补丁，不作为当前服务身份工具 |

2026-10-09 当前链路收口：7 个旧 legacy sender 已随仅验证旧实现的测试整组归档，见 [snapshot](../../archive/legacy_snapshot_20261009/README.md)。前次 A.5 的保留判断是较早快照。
本轮归档来源与复现限制见 [Archive Manifest](../../archive/ARCHIVE_MANIFEST.md)。后续候选需重新审查依赖。

## Dated reports and machine artifacts

下面产物按其日期和实验身份读取，保留原字节，不加文档metadata或改写历史状态。

- `reports/`：P0/P1部署、官方配置、示教/夹爪/环境审计等日期报告。
- `validation/`：旧2fps/单episode、20-step、1000/5000训练/评估、OEM、相机及真机trial记录。
- `provenance/`：初始planning与后续actual pin分别保留。`openpi_contract.json` 是初始
  pending快照，实际pin来自 `openpi_git.json` 和实验provenance。
- `docs/history/PI05_PIPER_PHASE_GATES.json`：历史机器阶段记录；不因实验自动改PASS。
- raw/source maps/outcomes/dataset/norm/checkpoint/identity：整体保持来源链。

保留产物不等于它仍是active部署，最新解释在六份领域authority中。
如果以后移动，先修复所有引用；归档保留期限和旧实验支持范围需要人工决策。

## Additional historical artifacts

本次继续按用户授权迁入以下两项，保留内容与阶段状态。

- [弃用初始计划重复副本](PI05_PIPER_MASTER_PLAN_ORIGINAL.md)。
- [历史阶段JSON](PI05_PIPER_PHASE_GATES.json)：只修正文档路径，不修改gate或权限。

表中proposed archive位置仅是旧建议；实际文档现统一保留于docs/history。
