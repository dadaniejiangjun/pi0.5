---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：这是文档收口前的2026-10-08审计快照；代码结论保留，文档authority调整以 DOCUMENTATION_CLEANUP_REPORT 为准，不重写当时证据。 当前领域主文档见 [authority](../CURRENT_STATUS.md)。

# Redundancy Candidates

2026-10-08只读审计。完整调用链和分类见[主审计](PROJECT_CODEBASE_AUDIT.md)。
HIGH表示条件式归档判断明确，仍要保留证据、先修引用；本轮没有cleanup。

## Safe / High Confidence

以下3项可在下一次批准后归档，不能直接删除。

| Path | 用途/状态证据 | 替代依据 | 仍被引用 | 独特功能 | 归档风险 | 建议 | Confidence |
|---|---|---|---|---|---|---|---|
| `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | 另一份初始计划全文 | PI05_PIPER_MASTER_PLAN_HISTORY.md字节相同 | docs/PI05_PIPER_MASTER_PLAN.md | 无独特字节；名称/链接有兼容价值 | 低到中：MASTER_PLAN第9行仍链接 | 未来archive/historical_docs；先改引用，留一份全文 | HIGH |
| `scripts/add_final_gate_aliases.py` | 固定把初始manifest写PASS/YES的一次性补丁 | 现有deployment_manifest成品及当前运行report | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 历史补丁生成过程 | 中：重放会强行写gate PASS；保留源码 | 未来archive/deprecated_scripts，不再作当前状态工具 | HIGH |
| `scripts/annotate_pi05_smoke_metadata.py` | 对固定ALOHA base smoke加元数据的一次性补丁 | 现有smoke成品和当前服务自动metadata/provenance | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 旧报告注释的生成过程 | 低到中：历史重现可能需要 | 未来archive/deprecated_scripts，不删除 | HIGH |

## Needs Manual Review

下列有复现用途或归属不明。

| 候选 | 用途/替代/独特价值 | 风险与处理 | Confidence |
|---|---|---|---|
| validation/camera_base_retry_runner_20261007_r1.py、camera_index_scan_runner_20261007_r1.py | 相机排障；dual_camera/source是当前通用入口，旧backend/index对照仍独特 | 固定实验路径/报告引用需确认，未来old_camera_diagnostics | MEDIUM |
| validation/derived_state_shadow_loopback_20261007.py | 旧loopback验证；serve wrapper覆盖常规任务但非旧identity全替代 | 先确认旧模型对比需求 | MEDIUM |
| validation/inspect_cando_function.py | OEM函数检查；capture_oem_can不完全替代分析 | 第三方版本和参数来源需保留 | MEDIUM |
| pytest临时目录/缓存 | 生成物，旧失败日志可能有用 | 部分不可读，先确认无活动任务 | MEDIUM |
| world_land_sea*、knowledge_landsea_experiment、artifacts/knowledge_world_map、testfoo.txt | 非PiPER主链，用途归属未建立 | UNKNOWN不等于冗余，由用户决定 | LOW |

## Stale Documents

以下14项需要更新状态或主入口；被契约校验读取的配置不能整份移走。

| Path | 用途/状态证据 | 替代依据 | 仍被引用 | 独特功能 | 归档风险 | 建议 | Confidence |
|---|---|---|---|---|---|---|---|
| `README.md` | 当前摘要仍偏prediction-only/旧单示教 | CURRENT_PROJECT_PLAN最新记录+task_trial5000 | docs/CURRENT_PROJECT_PLAN.md; docs/DATA_SCHEMA.md; docs/PROJECT_LAYOUT.md | 项目根导航/背景 | 高：根入口不能移走 | 保留并更新；旧快照可归档 | HIGH |
| `docs/PROJECT_LAYOUT.md` | 部分模块仍按planned/相机无IO描述 | 实际camera/session/Bridge/worker | docs/CURRENT_PROJECT_PLAN.md; docs/README.md; README.md | 目录职责地图 | 中：旧链接仍使用 | 保留更新 | HIGH |
| `docs/DIAGNOSTIC_SCRIPT_SPEC.md` | 诊断脚本规格仍描述stub/待实现 | query_*/probe_*实际实现 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 原始验收要求 | 中：仍是设计依据 | 保留规格，去掉当前状态误读 | HIGH |
| `docs/NETWORK_OPTIONS.md` | 网络决策待选 | 现用SSH/loopback8020/8021 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | VPN备选说明 | 中：备选仍有用 | 保留备选，记录实际方案 | HIGH |
| `docs/USER_DECISIONS.md` | 原始决策pending | plan+训练/运行provenance | docs/CHECKPOINT_INIT_DECISION.md; docs/PI05_PIPER_PHASE_GATES.json | 决策来历 | 中：不可丢决策历史 | 分离历史和最新 | HIGH |
| `docs/CHECKPOINT_INIT_DECISION.md` | 权重选择待定 | 实际pi05_base training provenance | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 选择理由 | 中：旧状态不是旧理由失效 | 保留标历史状态 | HIGH |
| `docs/OPENPI_PROVENANCE.md` | LOCAL_PIN_PENDING | provenance/gitlink/server HEAD一致pin | docs/PI05_PIPER_PHASE_GATES.json | 上游来源说明 | 高：来源必须保留 | 更新pin状态 | HIGH |
| `docs/SSH_TUNNEL_SETUP.md` | 8000旧部署例子 | 当前服务argv8020/8021 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | SSH方法仍有效 | 中：历史base smoke仍8000 | 区分不同服务例子 | HIGH |
| `laptop/README_FIRST_CONNECTION.md` | 旧P2 handoff/8000入口 | 当前可复用capture/train/serve入口 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 首次配置顺序 | 中：用户可能按旧命令执行 | 保留更新 | HIGH |
| `laptop/piper/README.md` | adapter规划描述 | PiperSession/Bridge/transport | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | unit_conversion说明 | 中：单位模块仍import | 链接真实adapter | HIGH |
| `scripts/README.md` | 目录仍plan-only/planned | 12个实际Python工具及迁移地图 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 工具索引 | 低：指南过短过旧 | 保留重写索引 | HIGH |
| `server/training/README.md` | 以单episode旧tiny-fit为主要入口 | TRAINING_USAGE/train_demonstrations/multi pilot | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 历史实验操作方法 | 中：旧实验仍复现 | 更新主入口，旧流程标历史 | HIGH |
| `configs/project_contract.yaml` | PLAN_ONLY；D1/D2/D5未选择 | 实际factory/report/argv | tests/contracts/test_piper_contract.py | 权限/时间/存储硬约束 | 高：contract validators读取 | 保留契约，更新状态；不整份归档 | HIGH |
| `configs/network_template.yaml` | selected=null/USER_DECISION_REQUIRED | SSH与loopback实际部署 | docs/CURRENT_PROJECT_PLAN.md; docs/PI05_PIPER_PHASE_GATES.json; laptop/client/network_latency_probe.py | 可复用多方案模板 | 中：不是当前部署profile | 保留模板并新增当前profile | HIGH |

## Historical Experiment Artifacts

历史记录默认保留，特别是tests/helper和当前identity引用。

| Path | 用途/状态证据 | 替代依据 | 仍被引用 | 独特功能 | 归档风险 | 建议 | Confidence |
|---|---|---|---|---|---|---|---|
| `docs/PI05_PIPER_MASTER_PLAN.md` | 整理后的初始计划 | CURRENT_PROJECT_PLAN | docs/CURRENT_PROJECT_PLAN.md | 原阶段划分和历史链接 | 中：索引仍引用两副本 | 保留历史标识 | HIGH |
| `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md` | 初始计划全文 | 当前进度由CURRENT_PROJECT_PLAN补充 | docs/PI05_PIPER_MASTER_PLAN.md | 原始全文 | 高：至少保留一个历史全文 | 保留为历史原文主副本 | HIGH |
| `docs/LAPTOP_SETUP_PLAN.md` | 初装环境计划 | 现在requirements/Conda路径/plan | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 机器重建步骤 | 中：重建可能要复用 | 保留；未来historical_docs | MEDIUM |
| `docs/SERVER_SETUP_PLAN.md` | 初装服务器计划 | 实际runtime_root/provenance | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 环境重建步骤 | 中：不可仅按日期删除 | 保留；未来historical_docs | MEDIUM |
| `docs/project-diagnosis-week-plan-2026-10-02.md` | 2026-10-02周计划 | 当前plan最新条目 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 当时问题与决策 | 中：历史诊断记录 | 保留；未来historical_docs | HIGH |
| `configs/model/pi05_piper_v1.yaml` | 明示initialization intent only | 实际piper_openpi_config factory/training report | docs/CURRENT_PROJECT_PLAN.md; server/piper_policy/piper_policy_config.py; server/training/check_piper_readiness.py；共8处，详主报告 | formal绝对位置初始意图 | 高：piper_train_entry/contract/tests引用 | 保留；不作当前训练状态 | HIGH |
| `reports/P0_P1_DEPLOYMENT_REPORT.md` | 2026-09-19初装验收 | 最新validation补充，不覆盖历史 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 初装测量/结果 | 高：provenance证据 | 保留不可变报告 | HIGH |
| `reports/pi05_official_config_audit.md` | 官方ALOHA/base配置审计 | 当前PiPER factory/identity补充 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | ALOHA sanity与PiPER差别 | 高：来源和环境证据 | 保留 | HIGH |
| `reports/piper_gripper_teach_audit.md` | P0/P1 teaching夹爪文档审计 | OEM/CAN_CTRL实测补充 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 未证明teach-mode独立夹爪支持 | 中：CAN_CTRL成功不等价teaching支持 | 保留历史上下文 | HIGH |
| `reports/piper_teach_mode_audit.md` | P0/P1程序进入teaching文档审计 | 后续mode-init补充 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 进入示教与CAN_CTRL切换区别 | 中：不能互相冒充 | 保留 | HIGH |
| `laptop/robot_client/supervised_j1_step_trial.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | docs/CURRENT_PROJECT_PLAN.md; laptop/robot_client/supervised_j1_one_degree_trial.py; laptop/robot_client/supervised_j5_probe.py；共11处，详主报告 | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_j1_one_degree_trial.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_j1_one_degree_trial.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_j5_probe.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_j5_probe.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_joint_replay.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | tests/laptop/test_legacy_hil_guard.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_restore_gripper.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | docs/CURRENT_PROJECT_PLAN.md; tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_restore_gripper.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_restore_start.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | laptop/robot_client/supervised_j5_probe.py; laptop/robot_client/supervised_joint_replay.py; laptop/robot_client/supervised_restore_gripper.py；共6处，详主报告 | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_smooth_joint_replay.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | docs/CURRENT_PROJECT_PLAN.md; tests/laptop/test_legacy_hil_guard.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |

cef166e2 2fps/10fps与旧step1/9、ALOHA smoke、1000/5000训练报告、loss、真机试跑日志有对照价值。
若以后归档old_training_experiments，需整体保留raw/source_map/outcomes/norm/checkpoint/provenance/hash链。
当前3条示教及595样本dataset不属可丢弃冗余。

## Explicitly Keep

下列有当前调用、权限、测试或来源用途。

- PiperSession、SafetyBridge、transport/codecs、所有typed授权与guard。
- 7个legacy sender及guard tests，直到共享helper和引用迁移完成；当前入口阻断但不是随便删除理由。
- unit_conversion、replay_preflight、formal P12/verified mapping、site profile模板。
- bounded_zero_return与supervised_near_zero_home：起始范围/夹爪行为不同。
- 可复用双相机、record_demonstrations、train/evaluate/serve wrappers与multi config/contract。
- 当前raw/outcomes/source_map/dataset/norm/checkpoints/training provenance/identity。
- CURRENT_PROJECT_PLAN最新记录、PHASE_GATES、provenance/.gitmodules/官方OpenPI pin。
- 仍被tests引用的历史proxy/public/ALOHA adapter和服务；先决定支持范围再迁移。

下一阶段先修状态/引用和source manifest，再逐项批准归档；本清单不授权cleanup。
