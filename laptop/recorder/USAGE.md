---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：以下是技术参考或有日期范围的规格/证据，不独立维护当前部署状态。 当前领域主文档见 [authority](../../docs/RUNBOOK.md)。

# 可复用示教采集程序

统一入口是 `laptop/recorder/record_demonstrations.py`。复用已有首次采集链路：
`capture_session` → `passive_capture` → 双路相机读取和 CAN 反馈监听。
无需另一台主臂。程序只记录，不切模式、不使能、不发送运动或夹爪目标。

## 录一条，再补四条

在项目根目录的普通 PowerShell 运行：

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B -m laptop.recorder.record_demonstrations --record --episodes 1
```

默认每条 20 秒、开始前等待 5 秒，任务为拾起黑棒并放到绿色桌垫。
按 Enter 准备，看到 `CAPTURE_START` 才开始示教。录制结束后输入 `s` 表示
任务成功，`f` 表示失败或未完成，`q` 表示不标注并停止。备注写明人工辅助等情况。
每条重新布置物体；五条指五次真实录制，复制一条不算五条。

第一条通过采集完整性检查且确认成功后，用同一命令补四条：

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B -m laptop.recorder.record_demonstrations --record --episodes 4
```

`--episodes` 指尝试次数，失败不计成功条数，应另外补录。完整性检查失败会停止
本次会话并保留部分文件；不要删除失败证据，也不要把它计为成功数据。

## 更换任务和录制时间

任务文字、时长和操作员都可以配置：

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B -m laptop.recorder.record_demonstrations --record --episodes 5 --duration 45 --lead-in 5 --task 'Place the toy blocks in the tray.' --operator 'researcher'
```

时长支持 10–60 秒，条数支持 1–50。每条单独准备和确认结果。
`--help` 查看全部参数；不加 `--record` 不打开设备。

## 相机与文件

当前已确认 Base 为 `ACER HD User Facing`，Wrist 为 `Dabai DC1`。
每次按名称解析当前 DSHOW 序号，不依赖固定 index 0/1，不扫描 Broadcast。
更换硬件后先人工确认视角，再设置 `--base-name`、`--wrist-name`。
关闭其他正在使用摄像头或 CAN 适配器的程序。
Codex 调用须在受限沙箱外执行；普通 PowerShell 无需管理员权限。

默认输出路径相对项目根目录固定：

- `datasets/raw/<episode_id>/`：双路 MP4、帧时间戳、CAN 原始反馈、任务、元数据和哈希清单。
- `datasets/manifests/capture_sessions/<session_id>/`：会话、各条结果和完整性检查。
- `datasets/manifests/outcomes/`：与原始数据哈希绑定的成功/失败标注。

程序创建新 ID，保留原始数据。时间戳为主机读取后的单调时钟，并非曝光同步。
这是反馈轨迹示教；后续用未来实测状态构建训练目标，不能称为原厂命令标签。
录制完成也不等于训练数据已转换，或模型已可安全控制机械臂。
实际采集与训练进展见 [CURRENT_STATUS](../../docs/CURRENT_STATUS.md)；本使用指南不维护实验状态。
