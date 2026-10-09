---
Document status: CURRENT
Current authority: docs/DATA_AND_TRAINING.md
Last reviewed: 2026-10-08
---

# Data and training

本文件是数据链、训练和评估边界 authority。具体命令由 [RUNBOOK](RUNBOOK.md)
维护，实时状态与结果由 [CURRENT_STATUS](CURRENT_STATUS.md) 汇总。

## Current dataset

当前训练数据由三个独立的成功示教组成，成功标签来自现场操作者。

| 项目 | 当前数据 |
|---|---|
| Dataset | `local/piper_new3_20261007_r1` |
| Episodes / frames | 3 / 595 |
| Rate | 10 fps |
| Task | Pick up the black rod and place it on the green desk mat. |
| Collection | Operator-assisted；人工扶动机械臂完成任务，反馈与双路图像记录 |
| Action supervision | Future observed state proxy，名义单位；没有原厂发送端命令监督 |
| Split | Train-only，`train:0:3`；没有 held-out validation |

手动拖动的成功示教可用于此 proxy 路线，不要求另一只 leader arm。
失败示教不是增加成功行为多样性的等价数据；保存失败标签用于分析，当前成功
模仿训练集不默认纳入失败动作。不能复制同一条轨迹冒充五条独立示教。

## Current data chain

下面列出复用程序与数据产物之间的关系；不同模块名表示实际处理环节。

```text
record_demonstrations → capture_session → passive_capture
  → label_episode → verify_source → align_raw
  → next_state_proxy → convert_next_state_proxy
  → LeRobot → normalization
  → train_demonstrations → evaluate_demonstrations
  → serve_trained_demonstrations
```

采集保留 raw CAN/视频/时间戳、session manifest 和 outcome。
完整状态窗口之外的边缘图像需要有可追溯的对齐处理，不能伪造反馈或偷偷改原始文件。
转换产物/source map 追踪来源；训练 report 绑定数据、norm、代码和 checkpoint。
服务入口检查 COMPLETE report 和这些身份后生成新的客户端 lock。

## Training and evaluation evidence

已完成实验的报告是数值来源，以下指标不能解释成任务成功率。

| 实验 | 完成步数 | 最后一步 loss | 最后 100 步均值 | 来源 |
|---|---:|---:|---:|---|
| 1000-step | 1000 | 0.0347 | 0.022951 | [loss summary](../validation/train1000_loss_summary_20261008.json) |
| 5000-step | 5000 | 0.0034 | 0.014172 | [loss summary](../validation/train5000_loss_20261008.json) |

早期 corrected tiny-fit 是 **20 steps**，不是 20K；其同训练数据评估关节
MAE 约 0.364068 rad、夹爪约 0.02759 m。1000-step 的既有相同 15 行训练数据
评估 MAE 约 0.0478098 rad、夹爪约 0.00234621 m。这些是名义单位误差，
不是独立任务测试，也不代表成功率或泛化能力。

50K 运行状态只见 [CURRENT_STATUS](CURRENT_STATUS.md)；未完成前不写最终 loss。
新训练默认从 base 初始化，不把独立 1000/5000/50K 实验误写成恢复同一 optimizer。

## Identity and reproducibility

机器身份由产物决定，文档不复制维护多份 hash/checkpoint 路径。

- 原始数据及 source map/outcomes 是监督来源证据。
- Training report / training provenance 指定实际 dataset、norm、代码与 checkpoint。
- Service identity lock 绑定 checkpoint tree、dataset、norm、语义和 OpenPI commit。
- 端口与 GPU 是具名部署 profile；它们不改变训练数据语义。
- 当前运行副本在服务器 `runtime_root`；项目根 HEAD 与未提交源码不能混为一体。

## Historical dataset experiments

以下为 HISTORICAL，保留复现价值，不是当前训练集。

- 旧单 episode、39 frames、2 fps 数据和旧 step-1/step-9 实验。
- 后续旧单 episode 10 fps proxy / tiny-fit。
- ALOHA/base smoke；它不代表 PiPER 任务验收。

来源、报告与旧计划索引见 [history](history/README.md)。
下一阶段需要更多独立成功示教、held-out 场景及现场任务结果，而不仅是更低训练 loss。
