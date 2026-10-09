---
Document status: CURRENT
Current authority: docs/ACTION_SEMANTICS.md
Last reviewed: 2026-10-08
---

# π0.5 PiPER action semantics

本文件是当前 action 语义 authority。训练代理、执行目标和 CAN 命令是三个概念；
当前运行解释不会改变原始数据的监督来源。

## Observation

当前观察接口是以下四个字段。

| 字段 | 类型 | 语义 |
|---|---|---|
| `base_rgb` | uint8 HWC RGB | Base 相机图像 |
| `wrist_rgb` | uint8 HWC RGB | Wrist 相机图像 |
| `state` | float32[7] | 前六维为 PiPER 六轴位置，第七维为夹爪反馈位置 |
| `prompt` | string | 任务指令 |

图像先由 OpenCV BGR 转为 RGB。OpenPI 输入映射为 `base_0_rgb` 与
`left_wrist_0_rgb`；缺失的 right wrist 使用零图及 false mask。
相机身份依设备名称/PnP 与人工角色确认，OpenCV index 本身不是稳定身份。

## Training action

当前名称为 `next_observed_state_proxy_nominal_units`；target kind 为
`derived_future_observed_state_position_target`，即“未来实测机器人状态的代理目标”。

它不是 velocity、joint delta、torque、raw CAN command，也不是 original
teleoperation command。反馈记录不能标成原厂示教发送端的命令标签。

## Units

当前数值转换如下。这里的 nominal 明确保留现场机械标定范围的限制。

| 分量 | 单位 | 原始反馈到训练数值 |
|---|---|---|
| Joint 1–6 | nominal rad | `raw × π / 180000`，raw 是 0.001° 数值 |
| Gripper | nominal meter | `raw × 1e-6` |

夹爪名义 meter 不能直接等同于已经完整机械标定的实际 jaw opening。
执行侧关节值按 `round(rad × 180000 / π)` 回到协议 raw，夹爪按
`round(meter × 1e6)` 回到协议 raw；协议值与实际行程关系仍需现场校准。

## Temporal semantics

当前数据与模型输出的时间语义如下。

| 项目 | 值 |
|---|---|
| Dataset rate | 10 fps |
| First future target | 约 observation 后 +100 ms |
| Action horizon | 16，约 +0.1 到 +1.6 s |
| Internal action dimension | 32，七维数据经 padding |
| `PiperOpenPIOutputs` | 取前七维 |
| 对外输出 | `actions.shape == [16, 7]` |

采集时间戳是 host arrival 的单调时钟时间，不是相机 exposure，也不是硬件同步证明。

## Current execution interpretation

`bounded_model_trial` 根据本次推理经过时间，以约 100 ms 为间隔选一行 proxy，
将其解释成 **bounded absolute position target**。关节相对当前反馈做单步限幅，
并裁剪到名义关节范围。夹爪当前没有单步限幅：有效名义范围是0–0.1 m，
超出范围（包括负值）保持当前反馈，不直接裁剪成闭合/最大打开命令。
非有限/错误形状输出拒绝。

当前代码没有按 10 Hz 消费整个 chunk、连续插值或完整闭环轨迹调度。
参数和停止条件见 [SAFETY_AND_CONTROL](SAFETY_AND_CONTROL.md)。

## Known semantic gap

`training proxy ≠ OEM robot command`。三层概念必须分别追踪。

| 概念 | 当前来源 | 边界 |
|---|---|---|
| TrainingAction | 后续实测状态 | 不是已发送命令 |
| ExecutionTarget | proxy 加执行侧限幅/裁剪 | 本次计划发送的绝对位置 |
| RobotCommand | 协议编码与 typed CAN TX | 已构造/适配器接收的帧；反馈才提供跟踪证据 |

后续可建立版本化 config/schema；本轮没有新增机器合同或修改语义。
数据来源、转换和评估边界见 [DATA_AND_TRAINING](DATA_AND_TRAINING.md)。
