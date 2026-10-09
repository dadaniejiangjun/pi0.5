---
Document status: REFERENCE
Current authority: docs/DATA_AND_TRAINING.md
Last reviewed: 2026-10-08
---

# Demonstration training reference

当前数据训练 authority 是 [DATA_AND_TRAINING](../../docs/DATA_AND_TRAINING.md)，
操作 authority 是 [RUNBOOK](../../docs/RUNBOOK.md)。本目录已有可复用训练封装。

## Current entrypoints

当前多示教 proxy 路线使用以下模块，参数参考 [TRAINING_USAGE](TRAINING_USAGE.md)。

| Module | 职责 |
|---|---|
| `train_demonstrations` | 审计数据/norm、调用官方OpenPI、输出训练来源及checkpoint report |
| `evaluate_demonstrations` | 对 COMPLETE report 指定的checkpoint作离线预测 |
| `serve_trained_demonstrations` | 从 COMPLETE report 启动loopback服务并生成identity lock |
| `summarize_training_loss` | 解析loss日志、汇总与曲线 |

当前三条示教的1000/5000实验已完成，完整结果仅在状态页维护；同训练数据MAE和
loss不是任务成功率。本程序不访问机器人。服务prediction_only仅描述服务器职责，
不代表客户端HIL不会发送CAN目标。

## Historical / alternative workflows

HISTORICAL：旧单episode/短tiny-fit和ALOHA smoke保留作复现，不能当成当前全部进展。
正式 command-label 训练/数据 readiness 路线有不同的监督要求；future-state proxy
不是原厂command。不要因旧formal入口尚未满足条件就写“所有训练未开始”。
