---
Document status: CURRENT
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-09
---

# Current project status

本文件是当前项目状态的唯一人类可读 authority。源码、机器可读报告与 identity
决定实际行为；本页汇总其证据，不替代运行配置。历史结论不能自动升级为当前验证。

| Identity | 当前值 / 证据范围 |
|---|---|
| Last updated | 2026-10-08；服务器进程和报告检查于 13:17 UTC（21:17 Singapore） |
| Git commit | `99ab21c9ebbe1048c66c54880af1b9a7e2a12382`；本地有大量未提交实现，HEAD 不能代表完整运行源码 |
| OpenPI commit | `215abfb217dbac7d5f1273282331b9b1866c0479`；gitlink、provenance 和服务器 HEAD 一致 |
| Active dataset | `local/piper_new3_20261007_r1`；3 条成功示教、595 帧、10 fps |
| Active checkpoint | 实机比较使用 5000-step 的 `4999`；1000-step 的 `999` 保留对照。完整路径由 training report / identity 指定 |
| Active server profile | `piper-5000-comparison` 与 `piper-1000-comparison`；核查时两者有服务进程，当前轮未请求推理 |
| Active ports | 以上两个具名 profile 的端口、GPU、启动方式统一见 [RUNBOOK](RUNBOOK.md) |
| Active task | Pick up the black rod and place it on the green desk mat. |
| Current development stage | 受监督实验闭环；自主拾放成功尚未验证 |

## Verified

以下事实有本地源码及既有运行报告支持。VERIFIED 仅限表中行为。

| 项目 | 证据 |
|---|---|
| 双相机与关节、夹爪反馈采集 | `record_demonstrations` / `dual_camera`；采集会话及原始数据质量报告 |
| 3 条成功示教、595 个样本、10 fps | corrected dataset/source map；成功标签由现场操作者提供，属于 operator-assisted 示教 |
| 1000-step training | [训练报告](../validation/train1000_report_20261008.json)：COMPLETE，checkpoint `999` |
| 5000-step training | [训练报告](../validation/train5000_report_20261008.json)：COMPLETE，checkpoint `4999` |
| WebSocket inference | 既有 shadow / bounded trial 中记录有效 `[16,7]` 输出与 identity 检查 |
| Model → bounded target → SafetyBridge → CAN → feedback | `bounded_model_trial`、`PiperSession`、typed transport 与实机 trial 记录；不是单纯 prediction-only 链路 |

另一个 50K 训练实验 `piper_new3_50k_ckpt20k_20261008_r1` 在核查时为 RUNNING，
日志约到 38,576 step，GPU1；训练报告为服务器运行目录中的
`train50000_ckpt20k_report_20261008_r1.json`。它不是当前实机比较模型，最终 loss、
最终 checkpoint 和任务效果均尚未确认。进程快照不保证之后仍运行。

## Partial

PARTIAL 表示闭环动作存在，但完整任务验收不成立。

| 实验 | 已有结果 | 未满足 |
|---|---|---|
| 1000-step long trial | 既有记录中 48 次目标记录、47 次关节反馈匹配 | 中途停止；没有抓取成功证据 |
| 5000-step long trial | 既有一轮 24 次更新、24 次反馈匹配 | 稳定窗口检测导致停止；没有抓取成功证据 |
| Supervised policy control | 现场看护下模型目标进入 Bridge/CAN | 尚非自主任务运行时；正式 phase gates 不自动 PASS |

详细试跑版本、停止原因和证据路径见[审计快照](reference/PROJECT_CODEBASE_AUDIT.md)
及[保留的执行历史](reference/CURRENT_PROJECT_PLAN.md)。各轮结果不能拼接成一次成功任务。

## Not implemented

当前主要执行路径没有以下连续控制组件。

- Continuous action scheduler。
- Proper action chunk consumption：没有按 10 Hz 连续消费整个 16 行 chunk。
- 连续轨迹插值、平滑。
- 独立持续运行的 watchdog。
- 自主任务运行时与自动任务成功判断。

## Not verified

以下项目不能用训练完成、反馈匹配或进程存在来替代验证。

- 自主完成黑棒 → 绿色桌垫的拾放。
- 网络断开、进程崩溃后的物理 safe-hold；硬件停止确认。
- 夹爪实际 jaw opening 与名义单位的最终机械标定。
- Held-out 任务泛化；当前数据均为训练数据。
- 50K 最终结果及任何新 checkpoint 的实机效果。

## Current blockers

阻塞完整任务的重点是执行与验证，不是“没有模型”或“没有 CAN 发送链路”。

1. Action scheduling / timing：逐步等待、重开 CAN 和推理时间与训练 10 Hz 语义不同。
2. Proxy → physical command calibration：未来反馈代理目标与真正命令、夹爪机械行程仍有差距。
3. Continuous-control safety semantics：过期检查存在，独立连续 watchdog、失联保持未验证。
4. Camera acquisition stability：双路已可用，但窗口内新鲜图像和持续稳定性会触发停止。
5. Failure / disconnect behavior：停止发送不等于机械臂已物理安全停止。

## Next three engineering tasks

这些是后续建议，本次文档整理没有实施代码或启动实验。

1. 冻结 TrainingAction / ExecutionTarget / RobotCommand 的版本与单位映射，补齐夹爪标定。
2. 设计并验证连续 action 调度、平滑及独立 watchdog；先离线/只读，再受监督实机验证。
3. 采集更多独立成功示教和 held-out 任务，用任务成功率与失败视频比较 checkpoint。

正式阶段标准唯一见 [PHASE_GATES](PHASE_GATES.md)。操作唯一见 [RUNBOOK](RUNBOOK.md)。

## Repository maintenance — 2026-10-09

本地当前链路代码已经收口：product Python 158 → 93，保留 60 个当前测试模块；98 个完整旧 Python 文件及 1 个历史测试片段归档。532 项纯测试、61 个子测试通过，3 项条件跳过。控制、训练、模型、数据语义没有因清理改变；未新增实机结果。当前代码索引见 [ACTIVE_CODE_INDEX](ACTIVE_CODE_INDEX.md)，GitHub 源码保存范围见 [source snapshot](reference/GITHUB_SOURCE_SNAPSHOT.md)。

上方 Git commit 是实机审计当时的基线，后续源码整理提交以 `git log` 为准；上方服务/50K 状态仍是 2026-10-08 快照，未在本轮重新核验服务器。
