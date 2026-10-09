# Current-chain repository cleanup report

日期：2026-10-09。本轮按用户追加指令，收口到实际通信、相机、数采、转换、训练、server、inference 与受监督控制链路；允许将只验证历史实现的测试一起归档。前次 Phase A.5 因历史测试保留的判断被本轮整组归档取代。

## Results

| 项目 | Before | After |
|---|---|---|
| Active product Python（laptop/server/scripts + validation 顶层；不含 pycache/tests） | 158 | 93 |
| 当前测试模块（不含 initializer） | 93 | 60 |
| scripts Python | 10 | 0 |
| validation 顶层 Python runner | 2 | 0 |
| 活跃旧 HIL sender | 7 | 0 |
| Active direct CAN TX locations | 11 | 4 |
| 全库含归档 direct CAN TX locations | 11 | 11 |

归档 98 个完整 Python 文件：33 个历史测试 + 63 个旧实现（含 1 个空 package initializer）+ 2 个 runner。另外新增 1 个历史测试片段模块，保留从 mixed identity test 移出的 1 个旧 proxy source 检查。归档 Python 新文件合计 99；另保存 6 份改写前的目录 README。没有永久删除源码、数据或运行证据。

归档位置：[legacy_snapshot_20261009](../archive/legacy_snapshot_20261009/README.md)。完整原路径/新路径/日期/HEAD/哈希/替代与条件见 [manifest](../archive/ARCHIVE_MANIFEST.md)。

## Current boundary

以 [RUNBOOK](../docs/RUNBOOK.md) 和实际源码为准。根入口与传递依赖保存在 [inventory](ACTIVE_CHAIN_CLEANUP_INVENTORY.json)，当前逐文件索引见 [ACTIVE_CODE_INDEX](../docs/ACTIVE_CODE_INDEX.md)。保留：

- dual_camera、worker、设备身份与相机/通信诊断。
- piper_tool、PiperSession、feedback；当前 mode/enable/axis/gripper commissioning、回零与夹爪操作。
- record_demonstrations、capture_session、passive capture 底层、episode、标注/质量/时间对齐与来源。
- convert_next_state_proxy、post_capture_pipeline。
- train/evaluate/serve_demonstrations、loss 汇总；实际底层 proxy_multi_episode_*、piper_openpi_config、serve_derived_state_multi_shadow。
- live_model_preview、bounded_model_trial、policy identity、observation；仍被当前入口使用的 live_shadow 聚合和必要 helper。
- 现有 SafetyBridge、typed transport、runtime/control contracts 和它们的 fake/纯软件回归。

没有根据 proxy/supervised 名字删当前实际依赖。所有通信/相机 diagnostics 和既有控制 authority 保留为链路维护工具，不因为没有主入口入站 import 就归档。配置文件、当前数据/训练/身份/实机证据仍原位。

## Archived groups

| Group | Treatment |
|---|---|
| 7 legacy raw HIL senders + hil_guard | 与历史测试和 helper 一起归档；未改变 guard 内容 |
| 旧 formal-policy/session、SDK preview、reject-only/offline replay | 不在实际 bounded 模型链路中，与专属测试一起归档 |
| 旧 public-data / ALOHA / 单条 proxy 训练与 shadow / latency 实验 | 实验入口和专属测试一起归档 |
| 旧 command-target converter / 2fps sampling comparison / synthetic smoke | 与专属实验测试一起归档；当前 proxy 转换与 provenance 不变 |
| 10 个 scripts 工具 | 历史 base smoke / 环境补丁 / 固定报告 writer 归档，当前操作使用正式 package 入口 |
| 剩下的 loopback 与 Frida runner | 历史复现源码归档，原结果/视频/CAN 日志不移动 |

## Dependency and import audit

使用 AST/import + module/call-string 搜索、当前入口与目录指南检查；额外 rg 检查测试中的硬编码 .py 路径，发现并拆出 mixed identity test 的旧 proxy source 检查。没有让当前模块 import archive，也没有添加 archive 路径 shim。

[Static checks](ACTIVE_CHAIN_CLEANUP_STATIC_CHECKS.json)：active → archive import = 0；当前已扫描 imports 没有指向本轮移走的模块；98 个归档 Python 哈希与归档前一致。静态分析不声称覆盖任意计算出的动态 import；当前所有生产 Python 内容均保持原字节。

## Source and test edits

生产 Python 修改：0。保留源码 content hash 与本轮基线一致。测试修改只有两处：

1. train_demonstrations 的旧 upper-bound 测试：`10001` → `50001`，与当前已存在 `1..50000` 约束一致，防止旧测试落入实际训练分支。没有修改训练上限。
2. artifact identity 测试：仅移走检查已归档 `serve_proxy_piper_shadow.py` imports 的函数；其余当前身份测试原样保留。历史函数文本哈希已登记。

文档变化：6 个目录 README 保存旧版本后收口为当前入口；更新 active index/layout/历史 registry/README 导航与实际文件链接。技术状态 CURRENT_STATUS 未改。新增 pytest.ini，仅从 tests/ 自动发现，archive/third_party/cache/证据目录排除。

## Tests and issues encountered

归档前（校正旧训练断言后）：**533 passed, 3 skipped, 61 subtests passed**。归档后最终：**532 passed, 3 skipped, 61 subtests passed**。少 1 项为明确归档的历史 proxy import 检查，不是跳过当前失败测试。

- [Verified before log](active_chain_tests_before_verified.log)
- [Final after log](active_chain_tests_after_final.log)
- [Guarded runner](active_chain_pure_test_runner.py)

保留 60 个测试模块，覆盖当前单位/观察/反馈、CAN encoding/typed transport、Bridge、相机/采集、来源/转换、训练包装器、服务契约/metadata 与身份。fake adapter / fake policy / 录制视频可用；真实 VideoCapture、USB find、真实 adapter、policy WebSocket construction 和 pilot training execute 被测试 runner 拒绝。线程/进程测试仅使用 fake transport 或纯 heartbeat/timeout。没有硬件、服务启动或训练。

3 项 skipped 来源：两项 LeRobot 缺失条件测试、一项 Windows symlink 创建不可用；不将其标 PASS。初次默认临时目录不可写导致 setup/fixture 失败，日志保留；改用新建 cache/cleanup_tests 临时目录并提升执行权限复跑成功。归档后第一次复跑发现 mixed identity test 的旧路径依赖，随后单独归档历史函数并全部复跑。错误日志未删除，没有放宽生产阈值。

## CAN authority

当前只剩 `laptop/transport/gs_usb_piper_can.py` 的 4 个实际 device.send（245/255/264/274 行）；7 个旧 send 仅作为源码保留在 archive。没有新增 sender、发送 ID、编码、授权接口。SafetyBridge/transport/PiperSession/bounded 模型行为未改变。

## Git and preservation

HEAD：`99ab21c9ebbe1048c66c54880af1b9a7e2a12382`；branch：`main`。工作树初始 modified/untracked 与源哈希见 [baseline](ACTIVE_CHAIN_CLEANUP_BASELINE.json)。19 个 tracked 文件使用 git mv，其他使用经过路径范围检查的 rename；没有 reset/clean/commit/push，没有覆盖未提交源码内容。

本轮 archive 是源码/测试历史，不是独立可运行工作树。旧 import 和 ROOT/cwd 保留；历史复现须在隔离 checkout 按 manifest 恢复旧路径和依赖，不能在当前仓库直接调用 archived 硬件脚本。默认测试不发现 archive。训练报告、dataset manifests、outcomes、source_map、norm/checkpoint provenance、deployment identity、CAN/video trial evidence 全部保留原位。

## Behavior and next scope

Current runtime behavior：NONE CHANGED。有意改变的是历史入口位置、测试发现范围和目录导航；这不等于实机行为验收。未进入 Phase B、未进行调度/平滑/频率/阈值/限位/动作/训练语义修改。
