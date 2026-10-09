# Phase A.5 Repository Slimming Report

日期：2026-10-09。以本地工作树为证据；本轮没有连接服务器、设备或模型服务。

## 1. Summary

归档 4 个 Python 文件；active tree Python 162 → 158，scripts 12 → 10，validation 顶层 runner 4 → 2。34 个重点候选中 4 个归档、30 个保留。没有永久删除源码。7 个 legacy sender 全部 KEEP；其历史聚合/校验和回归仍有依赖。

Behavior Difference：NONE OBSERVED（纯软件测试范围）。运行/训练/action/dataset/SafetyBridge/transport 的源码与配置内容未改；归档路径变化是预期 repository change，旧补丁路径不再是入口。

## 2. Dependency Re-Audit

Phase A 抽出单位转换、单帧解码、observation builder、相机 worker 与 policy helper；并没有把旧 Observed/preflight/checked_feedback/waypoint/envelope 迁到新 authority。单帧解码不等于多帧聚合与安全门，不能将旧测试改到不等价的 decoder。

扫描使用 rg 文件/引用搜索、git grep（Python/PowerShell/shell/config）、Python AST import/call-site 搜索及文档索引；检查 scripts 全部 12 个 Python、validation 顶层 4 个 Python、7 个 legacy sender 与 11 个旧 policy/smoke 候选。脚本目录没有 shell/PowerShell 文件。保存完整候选关系与 imports：

- [dependency audit](CODE_CLEANUP_PHASE_A5_DEPENDENCY_AUDIT.md)
- [dependency JSON](CODE_CLEANUP_PHASE_A5_DEPENDENCIES.json)

current_callers 包括 legacy → legacy，不等于正式 runtime 的依赖。CLI 没有 import caller 不等于 dead code。phase_a_before.json 是原实现 oracle，文本命中不算 active caller。历史审计的旧路径文字保留为 dated snapshot，不把历史提议当成当前入口。部分旧 validation 临时测试目录读取被权限拒绝；未删除或修改。这不影响本轮 4 个顶层源码及正式包扫描；这些目录不具备归档依据。

### Baseline

- Branch：`main
`
- HEAD：`99ab21c9ebbe1048c66c54880af1b9a7e2a12382
`
- 初始工作树及权限警告：[baseline JSON](CODE_CLEANUP_PHASE_A5_BASELINE.json)。已有 modified/untracked 文件均保留；未 reset/clean/commit/push。
- 归档前：228 tests + 23 subtests PASS；[before log](phase_a5_tests_before.log)。
- 281 个原源码/配置哈希全部未变（4 个源码按新路径比对）：[static checks](CODE_CLEANUP_PHASE_A5_STATIC_CHECKS.json)。

## 3. Archived Files

| Original | New Path | Reason | Replacement | Confidence |
|---|---|---|---|---|
| `scripts/add_final_gate_aliases.py` | [archive/deprecated_scripts/add_final_gate_aliases.py](../archive/deprecated_scripts/add_final_gate_aliases.py) | 无 active/test/config/正式入口调用；当前索引已切换 | dated deployment manifest + CURRENT_STATUS + formal PHASE_GATES (do not replay PASS patch) | HIGH |
| `scripts/annotate_pi05_smoke_metadata.py` | [archive/deprecated_scripts/annotate_pi05_smoke_metadata.py](../archive/deprecated_scripts/annotate_pi05_smoke_metadata.py) | 无 active/test/config/正式入口调用；当前索引已切换 | existing immutable smoke + current service artifact metadata | HIGH |
| `validation/camera_base_retry_runner_20261007_r1.py` | [archive/old_diagnostics/camera_base_retry_runner_20261007_r1.py](../archive/old_diagnostics/camera_base_retry_runner_20261007_r1.py) | 无 active/test/config/正式入口调用；当前索引已切换 | laptop.camera.dual_camera / camera.source (historical backend comparison preserved) | HIGH |
| `validation/camera_index_scan_runner_20261007_r1.py` | [archive/old_diagnostics/camera_index_scan_runner_20261007_r1.py](../archive/old_diagnostics/camera_index_scan_runner_20261007_r1.py) | 无 active/test/config/正式入口调用；当前索引已切换 | laptop.camera.dual_camera + diagnostics.probe_cameras | HIGH |

归档准入 A–I：4 个候选均不被正式 runtime/training/SafetyBridge/transport import，不被正式 entrypoint 调用；无待迁 active helper；测试无依赖；当前文档只登记为历史；有明确当前职责替代；归档后相关纯测试通过。camera retry 的旧 backend 对比细节仍保留在原字节中，不声称 dual_camera 复现全部历史比较。

两份 tracked script 使用 git mv（R100）；两份 untracked runner 使用验证过范围的文件 rename。只有这两项 rename 暂存，未暂存已有修改。逐文件 SHA256、Git HEAD、日期、Last Known Role、CONDITIONAL 复现限制见 [manifest](../archive/ARCHIVE_MANIFEST.md)。

## 4. Legacy HIL Review

| File | Decision | Active runtime import? | Dependencies / reason | Can execute motion normally? |
|---|---|---|---|---|
| `supervised_j1_step_trial.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | Observed/_read_for/preflight：其余 legacy 与独立状态/零位/竞争控制器回归依赖 | NO：execute 第一条语句 reject；保留不可达 send 源码 |
| `supervised_j1_one_degree_trial.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | 只有 legacy guard 与独立 prior-step/preflight/授权测试；公共 decoder 不替代这些 | NO：execute 第一条语句 reject；保留不可达 send 源码 |
| `supervised_j5_probe.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | legacy guard、EXPECTED_START_RAW/授权回归；仍引用 Observed/checked_feedback | NO：execute 第一条语句 reject；保留不可达 send 源码 |
| `supervised_joint_replay.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | guard test 与历史 replay 复现；调用旧多帧/恢复 helper，无完整等价 replacement | NO：execute 第一条语句 reject；保留不可达 send 源码 |
| `supervised_restore_gripper.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | legacy guard、TARGET_GRIPPER_RAW/授权回归；仍引用旧状态 helper | NO：execute 第一条语句 reject；保留不可达 send 源码 |
| `supervised_restore_start.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | checked_feedback 被其他 legacy 使用；recorded_start_raw/plan_waypoints/within_segment 独立回归 | NO：execute 第一条语句 reject；保留不可达 send 源码 |
| `supervised_smooth_joint_replay.py` | KEEP / LEGACY_REFERENCED | 无正式 runtime import；部分有 legacy cross-import | guard test 与历史 smooth replay 复现；无完整等价 replacement | NO：execute 第一条语句 reject；保留不可达 send 源码 |

Typed Bridge/transport 覆盖当前受控发送职责，但不等价于旧起始状态、历史编码、echo、freshness 和轨迹规划回归。仅拆 guard fixture 仍不能解除其余依赖；本轮没有复制 helper 或削弱测试来强行归档。正常 execute 在 adapter 前拒绝，7 个 send 仅在绕过 guard 后可能可达；guard 未改。

## 5. scripts/ Cleanup

| Path | Classification | Tool role | Decision |
|---|---|---|---|
| `scripts/add_final_gate_aliases.py` | ARCHIVE_READY | DEPRECATED / ONE_OFF_TOOL | ARCHIVED |
| `scripts/annotate_pi05_smoke_metadata.py` | ARCHIVE_READY | DEPRECATED / ONE_OFF_TOOL | ARCHIVED |
| `scripts/audit_laptop_probe.py` | ACTIVE_TOOL | DIAGNOSTIC_TOOL | KEEP；独立工具用途，无完整替代 |
| `scripts/refresh_environment_audit.py` | ACTIVE_TOOL | PROVENANCE_TOOL | KEEP；独立工具用途，无完整替代 |
| `scripts/run_local_policy_server_smoke.py` | ACTIVE_TOOL | DIAGNOSTIC_TOOL / historical reproduction | KEEP；独立工具用途，无完整替代 |
| `scripts/run_pi05_base_smoke.py` | ACTIVE_TOOL | DIAGNOSTIC_TOOL / base-ALOHA comparison | KEEP；独立工具用途，无完整替代 |
| `scripts/run_server_performance.py` | ACTIVE_TOOL | DIAGNOSTIC_TOOL | KEEP；独立工具用途，无完整替代 |
| `scripts/validate_piper_continuous.py` | ACTIVE_TOOL | DIAGNOSTIC_TOOL / contract validation | KEEP；独立工具用途，无完整替代 |
| `scripts/validate_piper_contract.py` | ACTIVE_TOOL | DIAGNOSTIC_TOOL / contract validation | KEEP；独立工具用途，无完整替代 |
| `scripts/write_checkpoint_provenance.py` | ACTIVE_TOOL | PROVENANCE_TOOL | KEEP；独立工具用途，无完整替代 |
| `scripts/write_deployment_manifest.py` | ACTIVE_TOOL | PROVENANCE_TOOL | KEEP；独立工具用途，无完整替代 |
| `scripts/write_socket_binding_report.py` | ACTIVE_TOOL | PROVENANCE_TOOL / diagnostic binding report | KEEP；独立工具用途，无完整替代 |

没有为目录美观搬迁其余 10 个工具；没有启动 smoke、性能测量或任何 writer。scripts README 已区分日常工具与归档补丁。

## 6. validation/ Cleanup

| Path | Classification | Decision |
|---|---|---|
| `validation/camera_base_retry_runner_20261007_r1.py` | ARCHIVE_READY | ARCHIVED；dated camera retry |
| `validation/camera_index_scan_runner_20261007_r1.py` | ARCHIVE_READY | ARCHIVED；dated camera scan |
| `validation/derived_state_shadow_loopback_20261007.py` | LEGACY_REFERENCED | KEEP；旧 checkpoint/身份/loopback 比较复现独有，无等价替代 |
| `validation/inspect_cando_function.py` | UNKNOWN | KEEP / HUMAN REVIEW；顶层 Frida attach，OEM disassembly 独有，没有完整替代；禁止 import/run |

原 camera JSON/output 目录、3 条成功示教、595-frame dataset、outcomes/source_map/norm provenance、1000/5000 训练/评估报告、checkpoint/deployment identity、最新 CAN trial 日志/视频全保留；未搬移 result evidence、raw dataset 或 checkpoint。runner 过期不等于结果过期。

## 7. Historical Policy / Config Cleanup

| Candidate | Classification | Decision |
|---|---|---|
| `laptop/client/action_shadow_replay.py` | TEST_SUPPORT | KEEP；已有代码/测试依赖 |
| `laptop/client/offline_latency_replay.py` | UNKNOWN | KEEP；无已核验完整 replacement，保留历史复现入口 |
| `laptop/client/policy_client_smoke.py` | ACTIVE_ENTRYPOINT | KEEP；已有代码/测试依赖 |
| `laptop/client/public_piper_transport_smoke.py` | UNKNOWN | KEEP；无已核验完整 replacement，保留历史复现入口 |
| `laptop/client/shadow_preview_smoke.py` | TEST_SUPPORT | KEEP；已有代码/测试依赖 |
| `laptop/client/synthetic_policy_reject_smoke.py` | TEST_SUPPORT | KEEP；已有代码/测试依赖 |
| `server/serve_derived_state_multi_shadow.py` | ACTIVE_ENTRYPOINT | KEEP；已有代码/测试依赖 |
| `server/serve_piper_action_shadow.py` | UNKNOWN | KEEP；无已核验完整 replacement，保留历史复现入口 |
| `server/serve_policy_local.py` | ACTIVE_ENTRYPOINT | KEEP；已有代码/测试依赖 |
| `server/serve_proxy_piper_shadow.py` | TEST_SUPPORT | KEEP；已有代码/测试依赖 |
| `server/serve_public_piper_smoke.py` | UNKNOWN | KEEP；无已核验完整 replacement，保留历史复现入口 |

proxy/2fps identity lock 仍被 policy_client/network probe/tests 引用，policy 配置仍被 shadow/reject-only/runtime 与 proxy 服务引用；旧数据规格也是训练/比较契约。其余模板/initialization intent 没有完整弃用证明，全部保留。configs 没有移动或改写。

`PI05_PIPER_MASTER_PLAN_ORIGINAL.md` 已在 `docs/history/`，本轮确认保留；没有重复搬移历史文档。旧实验 identity、OpenPI commit、config、report 和来源入口均保留。

## 8. Compatibility Wrappers Removed

0。live_shadow 中 Phase A compatibility reexports 共 14 个（policy_client 13 个 + camera worker alias 1 个，按顶层 ImportFrom imported-name 数计）；仍有调用/测试依赖，例如 camera alias identity 回归、旧 live_shadow API/identity 测试及网络 probe。live_shadow 本身不是空 shim。未增加 forwarding wrapper、sys.path hack 或 duplicate helper。

## 9. Generated Files / .gitignore

追加 `.pytest_cache/`。原 __pycache__/pyc ignore 保留；git ls-files 未发现已跟踪的 pyc/__pycache__/pytest cache，因此没有 untrack、删除或递归清理。权限不明的 validation 临时目录保留，未用 git clean。

## 10. Active Import Check

AST 扫描 laptop/server/scripts/tests/configs：archive import = 0、parse error = 0。active laptop/server/scripts 的 literal import_module/__import__(archive...) 命中 = 0；没有 active → archive 依赖。归档入口不被 import/执行。静态扫描不声称证明任意计算出来的动态 import；本轮 active 源码哈希未变。

## 11. CAN TX Audit

Before 11；After 11。相同路径、行号、receiver；7 个 guard 后 legacy send + 4 个 typed transport 后端 send。没有新增 sender，没有移除 active TX。

| Path | Line | Receiver |
|---|---|---|
| `laptop/robot_client/supervised_j1_one_degree_trial.py` | 73 | `device.send` |
| `laptop/robot_client/supervised_j1_step_trial.py` | 119 | `device.send` |
| `laptop/robot_client/supervised_j5_probe.py` | 59 | `device.send` |
| `laptop/robot_client/supervised_joint_replay.py` | 121 | `device.send` |
| `laptop/robot_client/supervised_restore_gripper.py` | 79 | `device.send` |
| `laptop/robot_client/supervised_restore_start.py` | 139 | `device.send` |
| `laptop/robot_client/supervised_smooth_joint_replay.py` | 231 | `device.send` |
| `laptop/transport/gs_usb_piper_can.py` | 245 | `self._device.send` |
| `laptop/transport/gs_usb_piper_can.py` | 255 | `self._device.send` |
| `laptop/transport/gs_usb_piper_can.py` | 264 | `self._device.send` |
| `laptop/transport/gs_usb_piper_can.py` | 274 | `self._device.send` |

发送 ID、encoding、authorization、SafetyBridge、PiperSession command behavior、mode init、gripper commissioning、bounded strategy 均未改。

## 12. Tests

归档前与归档后均 **228 tests + 23 subtests PASS，FAIL 0**。包括 Phase A 原 202 + 23 基线和 26 项 legacy 相关回归。测试入口复用已有 offline runner，拒绝真实 VideoCapture、CAN adapter open、Websocket client construction。静态 guard 测试确认 send 都在 execute 内，且第一条语句拒绝；没有设备连接、CAN TX、真机 motion、训练、服务启动。

- [before log](phase_a5_tests_before.log)
- [after log](phase_a5_tests_after.log)
- [exact file list](phase_a5_test_files.json)

| Test file | Tested behavior | Result |
|---|---|---|
| `tests/contracts/test_piper_units.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_live_shadow.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_piper_session.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_piper_feedback_ack_only_guard.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_bounded_policy_target.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_bounded_model_trial.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_safety_bridge_runtime.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_offline_safety_bridge.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_phase_a_pure_core.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_position_bridge.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_policy_session.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_shadow_preview_smoke.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_legacy_hil_guard.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_j1_step_trial.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_j1_one_degree_trial.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_j5_probe.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_restore_gripper.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |
| `tests/laptop/test_supervised_restore_start.py` | units / feedback / observation / identity / fake transport / legacy regression（按模块职责） | PASS |

额外核验：281 原源码/配置的内容哈希一致；4 archive 源码哈希一致；git diff --check PASS（已有 LF/CRLF 提示，不是 diff error）；git staged R100 仅两份归档补丁。

## 13. Behavior Difference

NONE OBSERVED。源码、配置和测试原内容未改；没有调整阈值、限位、CAN频率、checkpoint、训练参数、OpenPI transform、action horizon/order。仅路径归档、导航/registry 与 cache ignore 有意变化。本轮没有硬件行为证据，不能由纯测试声称实机行为验证。

## 14. Repository Simplification Metrics

口径：active source = laptop/server/scripts 递归 Python（排除 __pycache__）+ validation 顶层 Python；包含 __init__，不含 tests/third_party/reports/archive。archive 仅计 Python。compatibility = Phase A live_shadow 顶层 reexports。duplicate helper = Phase A 已收敛 pure-core 范围已知重复实现数量，不是全库 dead-code 证明。

| Metric | Before | After |
|---|---|---|
| active_python | 162 | 158 |
| scripts_python | 12 | 10 |
| validation_top_python | 4 | 2 |
| legacy_senders | 7 | 7 |
| archive_python | 0 | 4 |
| phase_a_compatibility_reexports | 14 | 14 |
| known Phase A pure-core duplicate implementations | 0 | 0 |

本轮未追求数字下降而改动 legacy/test support。archive 增加 README、manifest；报告/audit 文件不计 product source。

## 15. Items Not Archived

主要保留：7 个 HIL sender 的历史聚合/安全回归；validation OEM Frida 分析与 derived-state loopback；policy/proxy/base-smoke 的测试、比较、来源和正式服务依赖；10 个独立 scripts 工具；历史配置。没有明确 replacement 或无法确认独特功能消失的候选标 UNKNOWN/LEGACY_REFERENCED。没有归档后再让 active code import archive。

本轮操作文件：4 个 source rename（内容不变）；archive README/manifest；scripts README；docs/history README 已归档 registry/链接；docs/reference PROJECT_LAYOUT；robot_client README legacy registry；.gitignore；本轮 reports。已有用户修改没有覆盖。

## 16. Recommended Next Phase

建议经用户确认后进入 Phase B 的设计/审计：明确控制循环 scheduler、smoothing、continuous session/watchdog 与相机异步化的目标和验收；先区分行为变更与纯提取。机械标定仍是独立工作。7 个 legacy 的测试拆分需专门证明历史安全语义等价，不能用单帧 decoder 强替。**本轮没有启动 Phase B。**

### Acceptance

依赖复核、4 个高置信归档、active→archive=0、manifest、legacy逐项结论、TX不增加、pure tests PASS 均完成。无测试需要改引用；保留依赖原helper的测试以避免改变回归语义。未新增 duplicate helper，未运行硬件/训练，未改数据与模型。
