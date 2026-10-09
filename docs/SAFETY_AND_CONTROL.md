---
Document status: CURRENT
Current authority: docs/SAFETY_AND_CONTROL.md
Last reviewed: 2026-10-08
---

# Safety and control

本文件区分当前实际生效的检查、未接入的实现与尚未验证的物理行为。
检查来源是 bounded runtime、PiperSession、SafetyBridge 和 typed CAN transport。
本页不扩大任何现场授权；操作见 [RUNBOOK](RUNBOOK.md)。

## Active safety mechanisms

当前 `bounded_model_trial` 使用 commissioning 路径，而不是加载正式 P12 site profile。
以下表格描述源码中的实际边界，不保证所有现场风险都已覆盖。

| 检查 / 行为 | 当前参数与停止条件 |
|---|---|
| Service identity | checkpoint/dataset/norm/provenance/语义与 identity lock 不匹配时拒绝 |
| 明确执行授权 | 短试跑或长循环需要对应 `--site-cleared-...-authorized` 标志；标志不能代替真实现场确认 |
| Heartbeat | 发送前 freshness检查约250 ms；这是调用时检查，不是独立连续watchdog |
| Typed Bridge / TX | SafetyBridge 状态、commissioning snapshot 与 typed batch；普通 send 拒绝控制帧，typed 路径可发送 |
| Feedback freshness | 约 120 ms 上限；关节组合 skew 约 20 ms；缺失/过期拒绝 |
| Mode / status | CAN_CTRL=1，正常状态、关节运动模式、无错误、六轴使能；故障/模式改变停止 |
| 启动/每步稳定窗口 | 约 300 ms，关节变化 ≤0.05°；历史覆盖需满足检查，采样间隙上限约 120 ms |
| Camera startup | 最多等待 3 s；双路不就绪停止 |
| Camera source | host arrival age ≤250 ms，双路 skew ≤100 ms；每步最多等 2 s，超时不发送 |
| Inference/source age | ≤250 ms；推理期间机器人关节变化 >0.05°或夹爪 >1 mm 停止 |
| 输出结构 | `[16,7]` 且全部有限；无效输出拒绝 |
| Joint target bounds | J1 ±150°；J2 0–180°；J3 −170–0°；J4 ±100°；J5 ±70°；J6 ±120°，执行侧裁剪 |
| Joint step | CLI 可选 1/5/10/20/60°，默认 60°；相对每步当前反馈，不是累计范围 |
| Gripper | 当前反馈必须在名义 0–100 mm；模型输出超出此范围（包括负值）保持当前值；当前 bounded runtime **没有夹爪单步变化上限**，机械行程仍未完整标定 |
| Speed | 当前实验固定 5% |
| Target tracking | 等待最多 5 s；关节容差 CLI 可选 0.1° / 0.5°，默认 0.5°；不匹配停止、不重试 |
| Post-write feedback | 复查新鲜反馈、错误、模式与六轴使能；夹爪误差 >1 mm 停止 |
| Long loop | 最长 120 s、最多 120 次更新；没有累计角度限制；单步和关节范围仍生效 |
| Step lifecycle | 发送后 poll 0.35 s，释放 CAN 后等待 0.5 s；下一步重开、poll 0.65 s 并重新检查 |
| Parent timeout | 长循环 `duration + 35 s`，短试跑 40 s；进程超时中断不是物理停止证明 |

`TIME_BOUNDED_TRIAL_COMPLETE` 只表示循环完成，不能当作拾放任务成功。
当前 worker 调用 `limit_target` 时未传 gripper_step_m，构造
`BoundedTrialBatch` 时明确传 gripper_step_limit_raw=None；类型本身的1000默认值
不在这条运行路径生效。1 mm仍是推理期间变化/到位核对阈值，不是目标单步上限。
transport 的每实例 batch 预算和循环总次数是不同边界；每步新建 transport
会重置该实例计数，因此不能把它描述为全程最多三次。

稳定性阈值限制的是发目标前窗口，不是所有允许的目标运动。
CLI、worker 和 helper 的默认单步数值并不统一：分别为 60°、5°、1°。
运行时必须明确传 `--joint-step-deg`，避免把某个函数默认值当作部署合同。

## Implemented but not active in current runtime

以下实现有源码/测试用途，但不能声称它们已保护当前 bounded loop。

| 实现 | 当前边界 |
|---|---|
| `offline_guard` velocity / acceleration checks | IMPLEMENTED BUT NOT ACTIVE：bounded 主循环未调用它 |
| Formal P12 evidence / verified mapping / site profile | 另一条正式受监督路径；当前 commissioning 不等同于正式 gate 验收 |
| Legacy direct replay/recovery senders | 入口 fail-closed guard 阻断；部分 helper/test 仍依赖，不能随意删除 |

## Not implemented / not verified

运行异常通常停止后续发送，但尚不能证明机械臂执行了安全保持动作。

| 项目 | 状态 |
|---|---|
| Independent continuous watchdog | NOT IMPLEMENTED |
| Network loss physical safe-hold | NOT VERIFIED |
| Process crash physical safe-hold | NOT VERIFIED |
| Hardware stop confirmation | NOT VERIFIED |
| Continuous scheduler / smoothing | NOT IMPLEMENTED |

**kill process ≠ verified robot safe stop**。推理线程退出等待、USB/相机阻塞等
也可能影响实际退出时刻，不能把 120 s 描述为所有故障情形的硬件停止保证。

## Control modes

模式名称表达不同职责与验收边界。

| 模式 | 行为与状态 |
|---|---|
| SHADOW | 读相机、读反馈、请求预测并记录；不发送机器人位置目标 |
| COMMISSIONING / SUPERVISED | 当前 bounded 实验；现场授权后通过 typed Bridge/CAN 发送受限目标 |
| FORMAL SUPERVISED | 需要正式证据/映射/授权的另一条路径；实验记录不自动满足 formal gate |
| AUTONOMOUS | NOT IMPLEMENTED：没有已验收的自主任务运行时 |

查询、复位、模式切换、使能和位置运动不是同一权限。复位可能失去保持力；
不能在未知姿态直接复位，也不能由文档例子推导“本轮已授权运动”。
正式验收标准见 [PHASE_GATES](PHASE_GATES.md)。
