---
Document status: CURRENT
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-08
---

# Runtime runbook

本文件是操作命令 authority。命令基于现有可复用入口；本轮仅编写，没有执行。
当前状态见 [CURRENT_STATUS](CURRENT_STATUS.md)，停止条件见
[SAFETY_AND_CONTROL](SAFETY_AND_CONTROL.md)。使用新文件名保留每次运行证据。

## Prerequisites

先确认解释器和已有环境。不要因为终端未激活而重装依赖。

| 主机 | 已用环境 / 依赖 |
|---|---|
| Windows laptop | `E:\anaconda3\envs\pi05_laptop\python.exe`；OpenCV、PyUSB、gs_usb、piper_sdk、openpi-client |
| Linux server | `/mnt/ssd/users/mingkai/07_pi0.5/.venv/bin/python`；官方 OpenPI、JAX、已完成 checkpoint |
| SSH host | `rtx6000_Mingkai`，使用已配置的 SSH 凭据，不把凭据写入项目 |

在 Windows PowerShell 建立公共变量。以下 profile 表只在本页维护部署选择。

```powershell
Set-Location 'E:\01_NUS\02_5003\12_pi0.5'
$Py = 'E:\anaconda3\envs\pi05_laptop\python.exe'
$RunId = Get-Date -Format 'yyyyMMdd_HHmmss'
```

| Named deployment profile | Training report（RUN_ROOT 下） | Checkpoint step | GPU | Port |
|---|---|---|---:|---:|
| `piper-1000-comparison` | `train1000_report.json` | report 指定的 `999` | 0 | 8020 |
| `piper-5000-comparison` | `train5000_report.json` | report 指定的 `4999` | 2 | 8021 |

服务器公共路径为：

```bash
RUN_ROOT=/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/new_demo_tinyfit_20261007_r1
PYTHON=/mnt/ssd/users/mingkai/07_pi0.5/.venv/bin/python
```

核查时两个 service 进程存在，GPU1 上另有 50K 训练。**不要重复启动或按旧 PID
杀进程。** 先检查现有服务的 argv、报告、GPU 和端口，再决定复用或由操作者停止。
这张表是人类可读 profile，运行时参数、training report 和 identity lock 才控制程序；
尚未新增统一机器可读 deployment profile，不能假定已有这样的配置文件。

## Preflight

先检查身份与通信。只读 preview 不会发送机器人位置目标，但会打开设备。

1. 关闭占用相机的应用；确认 Base/Wrist 画面与设备角色。
2. 确认 CAN 适配器未被上位机/其他程序占用，线缆与电源正常。
3. 检查反馈完整、新鲜、模式及六轴使能；未知/异常时不执行运动入口。
4. 检查 SSH、选定 profile 的监听服务和实际训练报告。
5. 使用该服务生成的 identity lock；不靠重命名旧 lock 切换模型。

已确认相机为 Base=`ACER HD User Facing`、Wrist=`Dabai DC1`，使用 DirectShow。
历史成功映射为 index 0/1；程序按当前枚举解析，必须目视确认角色，不能认为 index
永远固定。相机 check 是采集可用性检查，不证明 exposure 同步。

```powershell
& $Py -B -m laptop.camera.dual_camera check --duration 5 --output-dir "validation/camera_check_$RunId"
& $Py -B -m laptop.robot_client.piper_tool status --read-approved --duration 3 --output "validation/status_$RunId.json"
& $Py -B -m laptop.robot_client.piper_tool motor-status --read-approved --duration 3 --output "validation/motors_$RunId.json"
```

查询程序的具体 CAN 行为由实现决定，不能把所有查询都等同于物理总线无 TX。
如果处于示教模式、使能缺失或反馈故障，按 [piper_tool 指南](../laptop/robot_client/README.md)
处理；**不要自动复位/使能**。模式复位需要零位、支撑和明确授权，可能失去保持力。

## Start server

当前入口是 `server.training.serve_trained_demonstrations`。
只有 selected report 为 COMPLETE 才可启动；checkpoint 来自报告，不手工复制路径。
以下是在没有同 profile 服务占用端口/GPU时，由操作者执行的 5000-step 例子。

```bash
cd "$RUN_ROOT/runtime_root"
TRAIN_REPORT="$RUN_ROOT/train5000_report.json"
GPU=2
PORT=8021
LOCK="$RUN_ROOT/live5000_$(date -u +%Y%m%dT%H%M%SZ).json"
"$PYTHON" -B -m server.training.serve_trained_demonstrations \
  --training-report "$TRAIN_REPORT" --identity-lock "$LOCK" \
  --gpu-index "$GPU" --port "$PORT"
```

1000-step 对照按 profile 表改 report/GPU/port。使用唯一 lock 文件名，已有 lock
会触发 FileExistsError；不要覆盖。服务完成模型初始化后写 lock，再开始服务；
只有进程存在或文件存在不足以证明当前能响应，仍需 shadow 请求核对。
可选 `--background` 会返回子进程 PID 和日志路径，需保留，不使用通配 kill。

把**实际这次服务的 lock**复制到本地。先设 `$ServerLock` 为服务器实际 lock 路径。

```powershell
$Port = 8021
$ServerLock = '/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/new_demo_tinyfit_20261007_r1/live5000_identity_restore_20261008.json'
$LocalLock = "validation/service_identity_$RunId.json"
scp "rtx6000_Mingkai:$ServerLock" $LocalLock
```

上例 lock 名是本次核查的恢复服务 argv；若重启服务，必须换成新生成的 `$LOCK`。
客户端还会检查 lock 与服务元数据一致，不把文档中的文件名当作永久身份。

## Start SSH tunnel

本地和服务端端口由选定 profile 决定，另开 PowerShell 保持 tunnel 运行。

```powershell
$Port = 8021
ssh -N -o ExitOnForwardFailure=yes -L "${Port}:127.0.0.1:${Port}" rtx6000_Mingkai
```

HISTORICAL：旧 8000/8002（以及其他早期端口）是旧服务 profile 例子，不是当前
5000-step 的默认入口。服务绑定 loopback；不将裸 WebSocket 暴露公网。

## Shadow mode

先做只读模型接入检查；这个入口复用相机角色解析和 live_shadow worker。

```powershell
& $Py -B -m laptop.client.live_model_preview --read-only `
  --identity-lock $LocalLock --port $Port --duration 15 `
  --output "validation/shadow_$RunId.json"
```

检查输出状态、identity、有效 `[16,7]` prediction、图像和反馈新鲜度。
Shadow 成功只能证明观察到预测链路，不是 CAN 跟踪或任务成功。

## Supervised model trial

**THIS CAN MOVE THE ROBOT.** 当前入口是 `bounded_model_trial` 的受监督
commissioning 路径；它不能冒充已验收的 formal supervised 或 autonomous runtime。

实际执行前确认上位机断开、场地清空、手离开、急停/断电可用、现场看护以及本次
参数范围的明确授权。以下复现既有长循环参数；文档命令本身不提供新的运动授权。
无需为了试跑把其他停止条件全部去掉。

```powershell
& $Py -B -m laptop.client.bounded_model_trial `
  --identity-lock $LocalLock --port $Port --output "validation/task_$RunId.json" `
  --long-loop --duration 120 --joint-step-deg 60 --joint-tolerance-deg 0.5 `
  --site-cleared-long-loop-authorized
```

这是关节每步最多 60°、5%速度、最长120 s / 最多120更新的
例子；**没有累计角度上限**，关节范围、时效、跟踪和状态检查仍在。
60°是上限，不是每步必走60°。当前夹爪没有单步变化上限，保留名义范围与反馈误差
检查；这与早期1 mm目标限幅不同。目标等待和启动检查会使更新频率远低于10 Hz。
不要把完成循环的状态当作任务成功；保存视频、每步目标、反馈和停止原因。
短试跑可用 `--updates 1` 与 `--site-cleared-three-updates-authorized`，明确指定
单步角度、容差和 profile；短入口最多3次。

## Data collection and training

示教采集使用可复用 recorder，不发送机械臂运动命令。先独立采一条并核验，再补
独立成功示教；不是复制扩增成五条。

```powershell
& $Py -B -m laptop.recorder.record_demonstrations --record --episodes 1 --duration 20 --lead-in 5
```

按交互提示开始，每条结束后如实标注成功/失败/退出。更多参数见
[recorder 使用指南](../laptop/recorder/USAGE.md)。

训练使用 `server.training.train_demonstrations`；评估使用
`server.training.evaluate_demonstrations`；loss 汇总使用
`server.training.summarize_training_loss`。完整参数和新实验命名见
[TRAINING_USAGE](../server/training/TRAINING_USAGE.md)。这些属于显式新操作，不随
服务启动、shadow 或文档整理自动触发。数据到训练边界见
[DATA_AND_TRAINING](DATA_AND_TRAINING.md)。

## Stop / failure procedure

当前异常路径停止后续发送、记录 `STOPPED_NO_RETRY` 或 parent timeout，并保存
已产生的日志/视频；没有自动重试/自动回零。停止程序不保证物理停止。

1. 观察异常运动时由现场操作者使用已确认的急停或电源措施。
2. 退出客户端后查看 trial report、progress、逐步 CAN 日志与视频；不要直接重跑。
3. 只读确认机器人反馈、模式与状态，再决定恢复步骤。
4. 推理服务前台 Ctrl+C / 停 tunnel 只影响网络或进程，不证明机械臂保持或已停止。

Network/process failure physical safe-hold：**NOT VERIFIED**。
硬件停止确认：**NOT VERIFIED**。本轮没有新增或测试故障处理机制。
