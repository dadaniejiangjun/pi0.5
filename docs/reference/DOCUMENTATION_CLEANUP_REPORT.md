---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

# Documentation cleanup report

本轮按用户附件执行文档收口，依据本地源码、配置、validation、既有代码审计，
并只读核对服务器报告/进程。服务器快照时间为2026-10-08 13:17 UTC。
没有重构Python、移动/删除文件、改变CAN/SafetyBridge/训练逻辑，也没有启动机器人、
训练、policy server或HIL。没有提交或推送Git。

## Created

新增6份当前核心文档，另加history、decisions索引及本报告，共9份Markdown。

- `docs/CURRENT_STATUS.md`
- `docs/ARCHITECTURE.md`
- `docs/ACTION_SEMANTICS.md`
- `docs/SAFETY_AND_CONTROL.md`
- `docs/DATA_AND_TRAINING.md`
- `docs/RUNBOOK.md`
- `docs/history/README.md`
- `docs/decisions/README.md`
- `docs/DOCUMENTATION_CLEANUP_REPORT.md`

## Updated

41份既有Markdown更新正文或身份说明；其中13份重写/修正入口与导航，其余28份
主要加metadata和范围说明。两份YAML仅加6行注释，不改变原配置数据。

正文/导航更新：

- `README.md`
- `docs/README.md`
- `docs/PROJECT_LAYOUT.md`
- `docs/DIAGNOSTIC_SCRIPT_SPEC.md`
- `docs/OPENPI_PROVENANCE.md`
- `docs/SSH_TUNNEL_SETUP.md`
- `laptop/README_FIRST_CONNECTION.md`
- `laptop/piper/README.md`
- `scripts/README.md`
- `server/training/README.md`
- `laptop/safety/README.md`
- `server/policy_server/README.md`
- `laptop/client/README.md`

Metadata/范围说明及小修正：

- `docs/AGILEX_SOURCE_AUDIT.md`
- `docs/CHECKPOINT_INIT_DECISION.md`
- `docs/CURRENT_PROJECT_PLAN.md`
- `docs/DATA_SCHEMA.md`
- `docs/GRIPPER_TEACH_OPTIONS.md`
- `docs/LAPTOP_SETUP_PLAN.md`
- `docs/NETWORK_OPTIONS.md`
- `docs/PHASE_GATES.md`
- `docs/PI05_PIPER_MASTER_PLAN.md`
- `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md`
- `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md`
- `docs/PROJECT_CODEBASE_AUDIT.md`
- `docs/REAL_ROBOT_SAFETY_PLAN.md`
- `docs/REDUNDANCY_CANDIDATES.md`
- `docs/REMOTE_ARCHITECTURE.md`
- `docs/SERVER_SETUP_PLAN.md`
- `docs/TEACH_DATA_PIPELINE.md`
- `docs/USER_DECISIONS.md`
- `docs/project-diagnosis-week-plan-2026-10-02.md`
- `laptop/camera/README.md`
- `laptop/diagnostics/OEM_CAN_CAPTURE.md`
- `laptop/diagnostics/README.md`
- `laptop/recorder/README.md`
- `laptop/recorder/USAGE.md`
- `laptop/robot_client/README.md`
- `laptop/transport/README.md`
- `server/dataset_tools/README.md`
- `server/training/TRAINING_USAGE.md`

`CURRENT_PROJECT_PLAN`原正文保留，仅在顶部说明它现在负责roadmap和日期限定历史；
`PHASE_GATES`增加“Experimental success does not automatically satisfy a formal
phase gate.”，没有把任何gate自动改PASS。MASTER_PLAN整理版链接改到HISTORY主副本；
recorder/USAGE末尾不再声称新示教待开始。审计报告保留其当时结论，只加快照说明。

### Audited stale documents

审计列出的14项已全部处理；UPDATE_CURRENT指保留当前导航/参考职责，不让它独立
竞争状态authority。REFERENCE标记用于技术指南，CURRENT仅用于领域authority/首页。

| File | Action |
|---|---|
| `README.md` | UPDATE_CURRENT |
| `docs/PROJECT_LAYOUT.md` | UPDATE_CURRENT |
| `docs/DIAGNOSTIC_SCRIPT_SPEC.md` | UPDATE_CURRENT / REFERENCE |
| `docs/NETWORK_OPTIONS.md` | HISTORICAL |
| `docs/USER_DECISIONS.md` | HISTORICAL |
| `docs/CHECKPOINT_INIT_DECISION.md` | HISTORICAL |
| `docs/OPENPI_PROVENANCE.md` | UPDATE_CURRENT / REFERENCE |
| `docs/SSH_TUNNEL_SETUP.md` | UPDATE_CURRENT / REFERENCE |
| `laptop/README_FIRST_CONNECTION.md` | UPDATE_CURRENT / REFERENCE |
| `laptop/piper/README.md` | UPDATE_CURRENT / REFERENCE |
| `scripts/README.md` | UPDATE_CURRENT / REFERENCE |
| `server/training/README.md` | UPDATE_CURRENT / REFERENCE |
| `configs/project_contract.yaml` | TEMPLATE |
| `configs/network_template.yaml` | TEMPLATE |

## Marked historical

10份Markdown不再描述当前部署，旧pending/未开始/计划机制只按当时日期读取。

- `docs/CHECKPOINT_INIT_DECISION.md`
- `docs/LAPTOP_SETUP_PLAN.md`
- `docs/NETWORK_OPTIONS.md`
- `docs/PI05_PIPER_MASTER_PLAN.md`
- `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md`
- `docs/REAL_ROBOT_SAFETY_PLAN.md`
- `docs/REMOTE_ARCHITECTURE.md`
- `docs/SERVER_SETUP_PLAN.md`
- `docs/USER_DECISIONS.md`
- `docs/project-diagnosis-week-plan-2026-10-02.md`

历史报告、provenance和机器JSON没有改写。它们原有的旧状态按历史身份保留；
`openpi_contract.json`初始pending不覆盖`openpi_git.json`实际pin。
CURRENT_PROJECT_PLAN是REFERENCE而非HISTORICAL整份冻结，因为仍保存roadmap。

## Marked template

2份配置在顶部显式说明初始模板不是active deployment。

- `configs/project_contract.yaml`
- `configs/network_template.yaml`

PLAN_ONLY、selected=null、候选端口等字段原样保留，以保持契约/测试引用兼容。
真正部署参数来自argv、report和identity；本轮没有新增或假称已有统一机器profile。

## Deprecated

共登记3项：1份Markdown直接标DEPRECATED；2个Python脚本在文档索引登记，源码未改。

| File | Proposed archive location |
|---|---|
| `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | `archive/historical_docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` |
| `scripts/add_final_gate_aliases.py` | `archive/deprecated_scripts/add_final_gate_aliases.py` |
| `scripts/annotate_pi05_smoke_metadata.py` | `archive/deprecated_scripts/annotate_pi05_smoke_metadata.py` |

本轮未归档。其他legacy supervised脚本仍有test/helper依赖，全部保留。

## Consistency checks

静态核对涵盖根README、六份领域文档、PHASE_GATES、重要模块指南、历史/决策索引。

| Check | Result / limitation |
|---|---|
| 当前状态authority | 仅CURRENT_STATUS；其他入口摘要/记录均链接主文档 |
| 当前架构authority | 仅ARCHITECTURE；未来结构明确PROPOSED / NOT IMPLEMENTED |
| 过时关键词 | PLAN_ONLY/NOT_STARTED、旧单episode/39frames/2fps、8000/8002仅保留于历史/模板/规格或引用历史的范围 |
| 四类错误声明 | 当前摘要不再声称training未开始、只有1episode/39帧、model不能发CAN、SafetyBridge不存在 |
| Gripper source check | 实际bounded runtime无夹爪单步上限；修正早期1mm说法，1mm仍用于反馈变化/误差检查 |
| CLI source check | 按源码parser检查相机check、CAN status/motor-status、preview、bounded trial、训练/服务入口；未执行这些设备命令 |
| 本地Markdown文件链接/显式锚点 | 50份Markdown / 287个本地链接：缺失文件0、显式锚点错误0；不验证公网链接或服务可达性 |
| Source/config baseline | 271个文件复核；仅2份YAML新增注释，其他269个原字节不变，无新增源码/配置文件 |
| YAML preservation | 去掉6行新增注释后，两份原始文件SHA256与基线相同；本机无PyYAML，未运行解析器比较，字节相同确认底层数据未变 |
| Runtime verification | 本轮只有读报告/进程；不把进程存在视为新推理成功或新实机任务成功 |

最终CLI AST核对：6个入口模块，所列参数缺失0；未import或执行硬件模块。

## Remaining conflicts

已整理的当前领域文档中未发现互相矛盾的current-state descriptions。
仍存在以下明确记录的未知、实现差异和未来事项，不通过猜测消除。

1. CLI/worker/helper的joint-step默认值不同，RUNBOOK要求显式参数；本轮不改代码。
2. 机器配置模板和历史报告仍含旧状态，已注明身份，不把它们作为实时telemetry。
3. 50K训练为时点RUNNING快照，最终checkpoint/loss/任务效果尚未确认；状态会随时间变化。
4. 当前服务有进程，但本轮不做infer请求；服务是否此刻响应不能由进程快照保证。
5. 夹爪机械行程、physical safe-hold、held-out泛化、自主抓取均NOT VERIFIED；scheduler/watchdog未实现。
6. root HEAD不能代表完整未提交实现，服务器runtime副本按实验provenance识别，不能假定自动同步。

NEEDS HUMAN DECISION：后续连续控制/现场标定的验收与授权范围；新增示教和held-out
场景；旧实验支持范围、保留期限与未来归档目录。没有凭本轮文档改动授予新动作权限。

## Current documentation authorities

以下七个领域各保留一个人类可读authority；机器实际行为由代码/config/manifest决定。

| Topic | Authority |
|---|---|
| Status | [CURRENT_STATUS](../CURRENT_STATUS.md) |
| Architecture | [ARCHITECTURE](../ARCHITECTURE.md) |
| Action | [ACTION_SEMANTICS](../ACTION_SEMANTICS.md) |
| Safety | [SAFETY_AND_CONTROL](../SAFETY_AND_CONTROL.md) |
| Training | [DATA_AND_TRAINING](../DATA_AND_TRAINING.md) |
| Operations | [RUNBOOK](../RUNBOOK.md) |
| Phase gates | [PHASE_GATES](../PHASE_GATES.md) |

## Recommended next step

下一阶段先冻结机器可读action/deployment身份，再单独设计连续scheduler、平滑和
watchdog的实现与验证，补齐夹爪标定和held-out任务评估。随后才审查helper抽取、
目录迁移和历史归档。本轮到文档交付为止，不自动继续重构或启动实验。

## Historical document migration (2026-10-08 follow-up)

用户随后授权集中存放历史文档。本节更新文件位置；上方清单是迁移前文档收口的
日期快照，并不表示历史文档仍在旧目录。10份正文保留，只修复相对链接。

| Previous path | Current path |
|---|---|
| `docs/CHECKPOINT_INIT_DECISION.md` | [docs/history/CHECKPOINT_INIT_DECISION.md](../history/CHECKPOINT_INIT_DECISION.md) |
| `docs/LAPTOP_SETUP_PLAN.md` | [docs/history/LAPTOP_SETUP_PLAN.md](../history/LAPTOP_SETUP_PLAN.md) |
| `docs/NETWORK_OPTIONS.md` | [docs/history/NETWORK_OPTIONS.md](../history/NETWORK_OPTIONS.md) |
| `docs/PI05_PIPER_MASTER_PLAN.md` | [docs/history/PI05_PIPER_MASTER_PLAN.md](../history/PI05_PIPER_MASTER_PLAN.md) |
| `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md` | [docs/history/PI05_PIPER_MASTER_PLAN_HISTORY.md](../history/PI05_PIPER_MASTER_PLAN_HISTORY.md) |
| `docs/project-diagnosis-week-plan-2026-10-02.md` | [docs/history/project-diagnosis-week-plan-2026-10-02.md](../history/project-diagnosis-week-plan-2026-10-02.md) |
| `docs/REAL_ROBOT_SAFETY_PLAN.md` | [docs/history/REAL_ROBOT_SAFETY_PLAN.md](../history/REAL_ROBOT_SAFETY_PLAN.md) |
| `docs/REMOTE_ARCHITECTURE.md` | [docs/history/REMOTE_ARCHITECTURE.md](../history/REMOTE_ARCHITECTURE.md) |
| `docs/SERVER_SETUP_PLAN.md` | [docs/history/SERVER_SETUP_PLAN.md](../history/SERVER_SETUP_PLAN.md) |
| `docs/USER_DECISIONS.md` | [docs/history/USER_DECISIONS.md](../history/USER_DECISIONS.md) |

CURRENT_PROJECT_PLAN、当前核心文档、弃用重复副本和Python源码保持原位置。
PI05_PIPER_PHASE_GATES.json仅修正REMOTE_ARCHITECTURE文档引用路径，
gate/status/权限全部不变；未改配置、历史运行报告或provenance。

迁移后静态复查：297个本地Markdown链接，缺失0；history下10份HISTORICAL文档。

## Reference and remaining history migration (2026-10-08)

用户授权将参考资料与剩余历史文件集中存放。旧清单按原日期保留；
实际新路径如下，移动前验证目标均在workspace内且不覆盖已有文件。

| Previous path | Current path |
|---|---|
| `docs/AGILEX_SOURCE_AUDIT.md` | [docs/reference/AGILEX_SOURCE_AUDIT.md](AGILEX_SOURCE_AUDIT.md) |
| `docs/CURRENT_PROJECT_PLAN.md` | [docs/reference/CURRENT_PROJECT_PLAN.md](CURRENT_PROJECT_PLAN.md) |
| `docs/DATA_SCHEMA.md` | [docs/reference/DATA_SCHEMA.md](DATA_SCHEMA.md) |
| `docs/DIAGNOSTIC_SCRIPT_SPEC.md` | [docs/reference/DIAGNOSTIC_SCRIPT_SPEC.md](DIAGNOSTIC_SCRIPT_SPEC.md) |
| `docs/DOCUMENTATION_CLEANUP_REPORT.md` | [docs/reference/DOCUMENTATION_CLEANUP_REPORT.md](DOCUMENTATION_CLEANUP_REPORT.md) |
| `docs/GRIPPER_TEACH_OPTIONS.md` | [docs/reference/GRIPPER_TEACH_OPTIONS.md](GRIPPER_TEACH_OPTIONS.md) |
| `docs/OPENPI_PROVENANCE.md` | [docs/reference/OPENPI_PROVENANCE.md](OPENPI_PROVENANCE.md) |
| `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | [docs/history/PI05_PIPER_MASTER_PLAN_ORIGINAL.md](../history/PI05_PIPER_MASTER_PLAN_ORIGINAL.md) |
| `docs/PI05_PIPER_PHASE_GATES.json` | [docs/history/PI05_PIPER_PHASE_GATES.json](../history/PI05_PIPER_PHASE_GATES.json) |
| `docs/PROJECT_CODEBASE_AUDIT.md` | [docs/reference/PROJECT_CODEBASE_AUDIT.md](PROJECT_CODEBASE_AUDIT.md) |
| `docs/PROJECT_LAYOUT.md` | [docs/reference/PROJECT_LAYOUT.md](PROJECT_LAYOUT.md) |
| `docs/REDUNDANCY_CANDIDATES.md` | [docs/reference/REDUNDANCY_CANDIDATES.md](REDUNDANCY_CANDIDATES.md) |
| `docs/SSH_TUNNEL_SETUP.md` | [docs/reference/SSH_TUNNEL_SETUP.md](SSH_TUNNEL_SETUP.md) |
| `docs/TEACH_DATA_PIPELINE.md` | [docs/reference/TEACH_DATA_PIPELINE.md](TEACH_DATA_PIPELINE.md) |

新增reference/README索引；修复Markdown相对链接及历史JSON文档引用。
契约测试只把DATA_SCHEMA存在性检查路径改为docs/reference/DATA_SCHEMA.md，
不修改测试逻辑或机器人/训练代码。不启动设备、训练或服务。

本轮迁移后检查：329个本地Markdown链接缺失0；历史JSON文档引用缺失0；
契约测试的新文档路径存在。未启动硬件或运行时。
