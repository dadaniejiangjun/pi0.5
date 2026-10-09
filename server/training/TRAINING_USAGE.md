---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/RUNBOOK.md)。

# 可复用示教训练入口

在服务器项目的隔离运行目录执行 `server.training.train_demonstrations`。
它复用已有数据审计、归一化哈希校验、官方 OpenPI 训练和 checkpoint 来源记录。
默认 batch 为 1，使用已验证的单 GPU 配置；本程序不访问机器人。

## 启动训练

先选定空闲 GPU，再运行以下命令。路径和实验名按当前数据修改：

```bash
python -B -m server.training.train_demonstrations \
  --dataset-path /path/to/audited/dataset \
  --base-params /path/to/pi05_base/params \
  --expected-dataset-sha256 DATASET_SHA256 \
  --steps 5000 --min-episodes 3 \
  --experiment piper_new3_5000step_r1 --gpu-index 2 \
  --report /path/to/new/train_report.json \
  --background --train
```

`--steps`支持1至50000步；`--checkpoint-interval`设置周期保存间隔，
`--keep-period`指定需保留的周期检查点（建议与保存间隔相同）。例如50K训练
在20K和40K附近保存并保留中间checkpoint，完成时另保存最终checkpoint：

```bash
--steps 50000 --checkpoint-interval 20000 --keep-period 20000
```

`--background`创建独立后台进程，退出终端后继续运行。
不加`--background`则前台运行，便于调试。每次使用新的实验名和报告路径，
已有checkpoint、报告或日志不会被覆盖。

## 查看结果

训练生成三个文件：

- `train_report.launch.json`：后台PID与日志路径。
- `train_report.log`：逐步loss和训练日志。
- `train_report.json`：RUNNING、COMPLETE或FAILED，完成时包含checkpoint信息。

用`tail -f train_report.log`查看训练；完成以报告的COMPLETE为准，不能只看进程
退出。单步loss会波动，同时检查最后100步平均值和趋势。小数据训练loss降低
不代表泛化或抓取成功，还需离线预测检查和受监督实测。

## 后续训练

更换步数、数据集路径、数据哈希和实验名即可复用。当前入口每个新实验从基座
初始化，不支持在同一目录直接断点续训；不要把独立10000步实验称为从5000步
续训。若需要保留optimizer进行续训，应明确增加并验证resume功能。

每次新实验都从Pi0.5基座初始化；周期checkpoint用于阶段比较/恢复材料，
此训练封装当前不会自动从20K checkpoint恢复继续训练。

## 保存后的离线预测测试

训练报告为COMPLETE后，使用同一个评估入口：

```bash
python -B -m server.training.evaluate_demonstrations \
  --training-report /path/to/train_report.json \
  --output /path/to/new/evaluation_report.json --gpu-index 0
```

默认每条示教取5帧，报告预测、目标和误差；这些是训练数据上的检查。
先核对GPU空闲，不和占满显存的训练进程挤在同一张卡上。
1000步实验从基座独立运行，最终保存序号999；与5000步使用不同实验名。

## 汇总loss与保存曲线

```bash
python -B -m server.training.summarize_training_loss \
  --log /path/to/train_report.log --output-prefix /path/to/new/loss_summary --plot
```

生成JSON、CSV与PNG。去掉`--plot`时仅需Python标准库；绘图需要已有Matplotlib
环境，本机可使用`E:\anaconda3\python.exe`。输出路径已存在时拒绝覆盖。
