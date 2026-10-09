---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：保留路线图和按日期追加的执行历史；原有当前状态描述按其日期阅读。新的唯一人类可读状态为 CURRENT_STATUS，本轮不删除或替换计划正文。 当前领域主文档见 [authority](../CURRENT_STATUS.md)。

# PiPER × π0.5 current project plan / 当前项目计划

This document is the current implementation plan for the `07_pi0.5` project.
It records the verified state, target architecture, safety boundaries, and
execution order as of October 7, 2026. The Chinese sections are normative;
the English section is an execution summary for collaborators and agents.

> **Warning:** This plan does not authorize robot motion. Live commands require
> a separate supervised session with verified mapping, bounded targets, a
> working stop, and explicit operator authorization.

## 2026-10-08 50K训练与20K周期检查点

用户要求复用现有训练程序，一次运行50K优化步，在20K训练点保存checkpoint。
已调整 `server.training.train_demonstrations`：上限扩至50K，增加
`--checkpoint-interval` 与 `--keep-period`；`proxy_multi_episode_pilot`
将两项写入OpenPI配置及训练来源记录。OpenPI官方脚本按`step % save_interval`
周期保存，`keep_period=20000`让20K和40K的检查点不被最新checkpoint淘汰；
50K末另保存最终checkpoint。此为新实验，从pi05_base初始化，不接续5K optimizer。

本次实验 `piper_new3_50k_ckpt20k_20261008_r1` 已在空闲GPU1启动，PID2919322，
50K步、保存间隔20K、保留周期20K、显存比例0.90、batch1，使用相同3条成功示教
和数据哈希957953ede1bbeba09b265a984034ebc088f494a5d9cbe72d020dbeb39782f862。
初始报告 `cache/codex/new_demo_tinyfit_20261007_r1/train50000_ckpt20k_report_20261008_r1.json`，
日志同路径`.log`，启动记录`.launch.json`。训练离线运行，不访问CAN。
已有1000步服务GPU0与5000步服务GPU2保持运行；GPU3上其他项目任务未触碰。
复用参数详见 `server/training/TRAINING_USAGE.md`。

## 2026-10-07 快速部署执行决策（后续执行以本节为准）

### π0.5 chunk与horizon调研决策（2026-10-08）

当前训练make_unregistered_config设pi05=True、action_dim=32、action_horizon=16；
服务输出裁为16×7，身份锁同样固定16。数据10Hz，首动作标签为t+0.1秒，
连续16行覆盖t+0.1至t+1.6秒。服务flow采样10次，与训练5000更新无关。
当前执行每次只选一行，index=floor(推理源年龄/100ms)，实测多为1或2；
没有n_action_steps动作队列，执行长度实际为1目标，重规划频率约0.36Hz。
内部32维是padding，不是32个机械臂动作，也不是chunk长度。

官方当前main：OpenPI Pi0Config默认H50；pi05_droid H15、pi05_libero H10。
LeRobot PI05Config默认chunk_size50、n_action_steps50、num_inference_steps10。
OpenPI DROID示例open_loop_horizon8、控制15Hz，但当前示例断言输出10×8，
与pi05_droid H15配置并非完全一致，不能整段照抄或误称全部π0.5默认8。
RTC执行重叠horizon与普通一次执行K不同，不应机械套同名参数。
参考链接：
- https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/config.py
- https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/models/pi0_config.py
- https://github.com/Physical-Intelligence/openpi/blob/main/examples/droid/main.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/pi05/configuration_pi05.py
- https://huggingface.co/docs/lerobot/main/rtc

下一实现建议（尚未实现/未硬件验证）：保留当前H16及5000步checkpoint，
按训练10Hz时间轴使用动作队列，每轮执行K4–6（起点5）后用新观察重规划；
异步预测与执行重叠，按测得延迟约150–200ms对齐/剔除已过期前缀，
在剩余队列耗尽前生成下一块，适配速度/加速度及5%实际跟踪能力。
不能逐个100ms目标等待静止再发；连续控制健康检测与静止标定检查分开。
预测H更长不等于执行K更长：H50在10Hz覆盖5秒，全部盲执行风险与漂移增加；
更长H的计算开销和远期误差可能增加，需数据与重训验证，不能仅改服务常量。
RTC需适配当前OpenPI服务；LeRobot训练型RTC要求专门训练的checkpoint，
当前模型未做该训练，不能直接开启trained模式。先基础异步队列，再RTC。
先评估连续目标频率、间隔抖动、动作块交接和实际任务成功，再决定H扩展；
3条示教数据不足仍限制泛化，不能只靠调horizon或增加训练步数补偿。

### 连续动作执行改造安排（2026-10-08，建议，尚未实现）

目标是把已有16×7预测按时间连续执行，使有效轨迹目标达到10Hz；
模型重规划先约2Hz。当前0.360Hz来自5000步r2日志，不是实时硬件状态。
本节仅记录方案，不启动CAN、相机、推理服务或运动。

本轮源码核对发现，当前入口仍复用单步标定接口：每轮打开CAN、poll0.65s、
等待300ms静止、推理期间要求关节变化不超过0.05°、发送后等待到位最多5s、
poll0.35s、关闭CAN、gc、sleep0.5s。固定等待至少1.5s，检查本身的计算成本
尚未单独计时；不能称所有剩余时间都是检查运算。底层每连接最多3个
BoundedTrialBatch、预测有效期250ms，并且每次重发CAN/MOVE_J/5%模式帧。
因此仅修改K或在现有函数外围加队列，无法获得长时连续控制。

实施顺序如下，复用PiperSession、现有SDK编码、相机身份解析和服务：

1. 保留单步诊断入口；新增连续轨迹入口和明确的流式会话契约。
   启动时检查身份、模式、使能、运动范围、反馈和相机，再保持一个CAN owner。
   单一owner同时负责RX及TX，避免多个线程竞争PyUSB或并发发控制帧。
   处理自身TX echo需利用可核验的适配器元数据及已发帧记录；不能把全部控制帧
   当回显丢弃，证据不足的冲突继续报告。流式次数预算由会话时长及更新预算定义，
   不在循环中重开连接绕过3次限制；模式设置放启动阶段，按协议核验保持要求。
2. 增加带时间的动作队列。每项保存source_observation_ns、request_id、
   action_index、target_time_ns、绝对六轴和夹爪目标。训练首目标是t+0.1s，
   第i项时间t+(i+1)*0.1s；按本机单调时间对齐，丢弃已过期前缀。
   H保持16；首版每约5个轨迹步切换新预测，推理请求约2Hz、最多1个在途。
   提前量先按300–400ms设计，后以端到端P95/P99加裕量更新；不是只用GPU耗时。
   切换时替换未承诺的未来段，保留推理期间仍会执行的前缀；禁止无条件FIFO追加
   所有16项形成陈旧长队列，禁止重复执行已过期目标。
3. CAN控制循环按绝对deadline每100ms调度，缓存健康检查后发送当时目标。
   不调用逐帧静止等待、wait_joint_target、固定poll和sleep；持续RX刷新反馈。
   健康检查继续逐tick执行：故障、模式、使能、新鲜度、限位、watchdog等。
   推理期间正常运动不再按0.05°漂移拒绝，改为轨迹一致性与跟踪误差监测。
   250ms响应延迟与动作项有效期分开；队列动作并不在请求后250ms全部过期，
   但必须有明确执行时间、最长开环时间和过期策略，不能循环刷新老预测身份。
   队列欠载停止推进轨迹，不外推；停止发目标不等于实体急停或电机停止，
   保持/停机行为需沿用并核验控制器实际响应。
4. 相机常驻读取最新帧，推理线程读取一致的图像与状态快照；视频编码和日志
   放有界后台写入队列，避免on_frame编码与逐帧flush阻塞相机读线程。
   当前腕部相机间歇慢读是待定位问题，不能假定改线程必然恢复；分别测读取、
   编码、写盘耗时。未取得新鲜观察时不继续补充预测，可执行的旧段仍受截止约束。
5. 将大单步角度限幅改为基于真实dt的速度/加速度约束和块交接检查。
   60°/100ms相当于600°/s的单步差分，不能当作连续速度设定。
   夹爪继续沿用已取消的单步1mm限幅决定；发送后1mm误差检查改为连续跟踪
   的滞后/持续误差评估，避免正常开合过程即被拒绝。物理行程与SDK名义单位
   尚未独立标定，阈值由实测确定。5%速度下若跟不上，保持10Hz发送但将参考
   轨迹放慢；简单把数据每行改为200ms会改变训练时间语义，必须显式记录重定时。
6. 首版使用时间对齐及有限过渡完成基础异步队列；若块边界仍有跳变，适配
   RTC guided：新推理接收旧块剩余动作和预计延迟，生成时约束新块前缀衔接。
   后处理插值/加权融合不等于RTC。现有普通flow checkpoint可研究guided适配，
   当前未实现；trained模式需专门RTC训练，不能直接开启。

建议参数：H16，K先5（比较4–6），参考轨迹10Hz，重规划约2Hz，flow采样10；
可先5Hz调度排查，再升10Hz，但必须注明轨迹重定时。20–50Hz插值发送只作为
后续可选实验，须核对固件/控制模式持续位置目标行为及Windows/USB实际时序；
插值不会增加模型决策频率或自动提高抓取成功率。

建议验收而非已测结果：先用录制观察和假transport核对时间轴、迟到响应、
块交接、回显及欠载行为；之后实测连续60s，整组六轴+夹爪目标更新9–11Hz，
P95间隔80–120ms、最大正常间隔不超过200ms，记录所有欠载/停顿。
另记录真实关节跟踪误差、块边界速度突变、双相机新鲜度、推理P95/P99、
CAN接收/发送及编码写盘耗时。模式故障或输入过期引发的正确停止不算吞吐通过。
频率验收和完整抓取成功分别报告；现有3条训练示教仍限制任务能力。

参考：LeRobot async https://huggingface.co/docs/lerobot/async 与
RTC https://huggingface.co/docs/lerobot/main/rtc 。

### 取消模型夹爪单步1mm限幅（2026-10-08，最新授权）

用户明确要求取消动作大小表中的“名义最多1mm”限制。
limit_target默认不限制夹爪单次变化，合法模型位置目标原样使用；
bounded_model_trial显式传入gripper_step_limit_raw=None，底层同步放行。
底层批次默认仍1000raw，旧诊断调用须显式选择取消；归零夹爪仍保持。
SDK名义目标范围0–100000raw、非有限数拒绝、非法预测保持当前位置、
故障及新鲜反馈检查仍保留。推理期间夹爪漂移1mm与发送后跟踪误差1mm
是不同检查，未因这次单步限幅取消而删除；下一实机必须区分这些阈值。
相关离线测试22项通过，含70mm名义目标编码及非法范围拒绝；本轮无运动。
物理夹爪尺度仍未完成独立标定，不把名义100mm当真实可执行行程。

### 实机目标更新频率核验（2026-10-08）

根据已保存adapter_accepted_frames单调时间戳离线计算，不连接硬件。
5000步60°r2：24次更新，首末跨度63.829秒，目标频率0.360Hz；
间隔平均2.775秒，范围1.985–4.015秒；已记录推理平均153.684ms，
范围122.257–203.814ms。1000步无累计限幅r1：48次，0.413Hz，
平均间隔2.419秒、推理133.865ms。报告
`validation/measured_policy_update_rate_20261008.json`。
这是整组关节+夹爪目标更新频率，不是CAN帧数、机械臂内部伺服频率或相机fps。
当前infer放入线程只为网络等待期间继续接收CAN；控制执行仍等待future，
没有预测与轨迹执行重叠。16×7预测每次只执行index1或2的一行，剩余未排队。
每步重开CAN并poll0.65s，到位阻塞、再poll0.35s、再sleep0.5s，
加上300ms稳定预检及相机等待，构成逐步停顿。当前并非连续异步动作控制。
下一实现方向：长期单一CAN owner、独立预测线程和动作队列、按训练10Hz
时间轴消费动作块、重叠预测、块间插值及速度/加速度约束；连续运动健康检查
与单步标定稳定预检分开，不逐帧要求静止到位。需处理自身TX echo，不能
直接忽略所有竞争帧。5%低速需与轨迹时间轴匹配，10Hz是待验证目标。
本节记录建议，尚未实现或实测异步控制；未声称RTC可直接接入现有checkpoint。

### 60°单步与0.5°容差试跑（2026-10-08，最新设置）

用户再次授权后先实景只读预热通过，r2重跑，完成24次更新，
24次到位匹配，随后关节稳定窗口未通过而停止。
本轮未跑满120秒。视频与summary.json在
`validation/task_trial5000_60deg_20261008_r2`，完整报告加.json。
不应把此次中断归为推理超时，当前reason为关节稳定窗口检查。


用户要求放宽到位容差、单步上限60°，随后归零重跑。
模型CLI默认单步60°、到位容差0.5°、等待5秒，可显式恢复较小限额。
底层批次默认仍1°，上限可显式60°，SDK位置限位不变；无累计角度限幅。
控制相关测试68项通过，含60°边界、越界拒绝、SDK限位保留。
归零原0.1°分段容差导致J6偏差0.172°停止；同步分段容差0.5°后，
从新反馈继续5段归零成功。最终零位仍检查±0.1°，并非零点标定。
随后5000步试跑发送1次目标，反馈通过0.5°检查；下一次推理/source
超过250ms时效，在发送下一条目标前停止，未跑满120秒。
报告 `validation/task_trial5000_60deg_20261008_r1.json`，视频及CAN同名目录。
本次中断原因已不是到位容差。未放宽推理源时效或稳定窗口阈值。

复用试跑命令（现场授权后，输出路径须唯一）：

```powershell
E:/anaconda3/envs/pi05_laptop/python.exe -B -m laptop.client.bounded_model_trial --identity-lock validation/live5000_restart_r2_identity_20261008.json --output validation/task_trial5000_60deg_20261008_r1.json --port 8021 --long-loop --duration 120 --joint-step-deg 60 --joint-tolerance-deg 0.5 --site-cleared-long-loop-authorized
```

### 5000步服务重启与抓取预检（2026-10-08）

再确认零位、CAN_CTRL、MOVE_J及六轴使能后，按用户授权执行r3长程
抓取尝试。同样120秒/5%速度/20°单步/无累计限幅，实际发送5次，
4次反馈匹配，随后到位检查失败停止，未自动重试。
详细摘要 `validation/task_trial5000_restart_long_20261008_r3/summary.json`；
双路视频解码通过，未跑满120秒。当前反复中断来自严格的到位判断，
不能视为完整模型抓取能力测试。


用户通过上位机回零后再试r2：零位、六轴使能及稳定预检通过，
5000步模型实际发送1次更新（5帧适配器接受），随后2秒跟踪检查未通过，
STOPPED_NO_RETRY。详细目标和反馈偏差在同名目录summary.json，
未继续发送或自动重试，未完成120秒抓取。


用户告知服务停止，已核验旧PID2858500不存在，重新启动服务：
GPU2、PID2915476、127.0.0.1:8021，checkpoint4999哈希一致。
新身份锁 `validation/live5000_restart_r2_identity_20261008.json`。
CAN只读确认CAN_CTRL/MOVE_J、error_code=0，8021隧道连接正常。
按用户授权尝试120秒抓取，20°单步、5%速度、无累计限制。
实时稳定窗口等待3秒后仍未通过，发送0次位置目标，STOPPED_NO_RETRY。
服务已开启，但不能把本次预检称为抓取运动成功或完整任务测试。
报告 `validation/task_trial5000_restart_long_20261008_r1.json` 与同名CAN日志。

### 5000步完整长循环再次尝试（2026-10-08）

用户要求完整长程。新增新鲜图像等待最多2秒，等待期间只收CAN，
恢复后重新读取机器人状态；图像250ms年龄及100ms两路偏差检查不放宽。
新增下一步开始前最多3秒只读等待稳定窗口；其他错误立即停止，
不发送重复位置目标。预测期间保持250ms源时效，未使用过期预测。
相关测试16项通过，含只读稳定等待及电机错误立即退出。
先3段归零完成，r5发送1次并反馈匹配，下一步稳定窗口失败停止。
r6未发送目标，3秒只读等待后仍稳定窗口失败。最后400ms原始反馈：
J6范围0–0.329°，其他轴不变，均在SDK位置范围内。
因此本轮实际反馈不稳定，不是累计限幅；未完成120秒，未完成抓取。
不应声称5000步模型完整任务已测试。汇总
`validation/task_trial5000_long_20261008_attempts_r5_r6.json`。
先定位J6近零小目标反馈波动，不能仅删除稳定检查后继续运动。

### 5000步模型实际运动测试（2026-10-08，最新）

用户手动归零后继续抓取。恢复8021隧道；实测六轴零位、CAN_CTRL，
J1–J5使能、J6未使能。按既有授权使用supervised_enable_j6单次0x471
0602000000000000，使能六轴确认，位置无变化；enable_j6_before5000报告留存。
r2无位置发送（使能不足），r3无位置发送（推理/source超过250ms）。
实景只读预热live5000_preview_r2通过后，r4开始实际运动。
与1000步同配置：5000步checkpoint4999，5%速度，每步最多20°，
最长120秒，无累计角度范围，夹爪名义每步1mm。
实际约20秒、8次更新全部关节反馈匹配，最大实际单次变化8.261°。
因相机画面过期停止发送；视频时间戳证实wrist最大帧间隔1002.882ms，
base最大50.722ms。无自动运动重试，结束黑棒仍在桌面，未完成抓取。
base612帧、wrist542帧完整解码；报告和视频目录
`validation/task_trial5000_long20deg_nocumulative_20261008_r4`（报告加.json）。
本轮长度不一致且相机中断，不能据此判断5000步比1000步更好或更差。
后续先处理wrist采集停顿，再做等时长任务比较，不删除图像过期检查。

### 5000步对比中途归零请求（2026-10-08）
用户确认机械臂电源开启。随后再次只读检查，3秒内PiPER状态/关节帧均为0，
对所有CAN帧的3秒检查也为0；USB适配器可打开，CAN总线当前无可见帧。
原因未定位。归零程序未发帧。5000步服务仍在GPU2、8021供后续比较；
5000步实机比较尚未获得可记录的进度结果，试跑子进程已被用户的重新归零
请求中止。需要重新插接USB-CAN并确认机械臂侧CAN接头及上位机状态，
只读反馈恢复后再归零。


已启动5000步checkpoint4999独立服务，GPU2 PID2858500，端口8021。
身份锁live5000_identity_20261008.json，与训练报告checkpoint哈希一致。
明确授权90°起始范围后，先18段归零成功，再只读预测通过，无CAN发送。
随后相同设置启动5000步实机试跑：20°单步、5%速度、120秒、无累计限幅。
用户中途要求重新归零；已终止该次发送子进程37980，父进程退出。
本次WORKER_ERROR是人为终止产生，不是模型推理错误，视频为未验收部分文件。
重新归零预检反馈不完整，0次运动发送；随后3秒只读查询全部主要反馈帧为0，
因此尚未归零，禁止把默认零状态/默认STANDBY当实测结果。
报告zero_interrupt5000_20261008_r1.json、status_after_interrupt5000_20261008_r1.json。
待现场确认电源、CAN连接及上位机占用，恢复反馈后继续归零；不自动重启试跑。

### 移除长循环累计角度限制（2026-10-08，最新决策）

本次实际执行：先12段归零，随后无累计角度限制运行约117秒。
发送48次更新，47次到位确认；第48次J6偏差0.114°超过0.1°容差，
在2秒检查结束后停止发送，控制器error_code=0。J2最终85.639°，
证实原累计60°限制未生效。实际最大单次目标变化15.551°（上限20°）。
base3510帧、wrist3483帧完整解码，结束画面黑棒仍在桌面，任务未成功。
未自动重试；停止发送不等价于硬件急停。完整报告与同名视频目录已保存。


用户明确说明20°为单次目标更新上限，并授权取消累计角度限制。
长循环已移除相对起点60°限制，报告cumulative_joint_limit_deg=null。
每轴每次最多20°，实际值由模型预测确定；仍受SDK关节位置范围约束。
保留5%速度、最长120秒、最多120次更新、反馈与相机新鲜度、模式/故障、
发送结果检查及夹爪名义1mm限幅。短测试累计范围保持原行为。
未将“其他不必要限制”解释为删除硬件限位、故障或断联检查。
本次先调用bounded_zero_return，再运行同一模型长循环；报告路径：
`validation/zero_before20deg_nocumulative_20261008_r1.json` 与
`validation/task_trial1000_long20deg_nocumulative_20261008_r1.json`。
相关限幅、发送和归零测试17项通过；此前完整相关测试67项通过。

### 归零后20°上限试跑（2026-10-08，最新授权）

本次试跑已结束：约25秒、9次更新全部反馈匹配；配置上限20°，实际
最大单次目标变化15.137°。第10次拟定目标超出起始姿态累计60°边界，
发送前停止；J2最后反馈58.871°。这是软件试跑累计边界，不是官方
关节限位或硬件故障，不应混淆。未跑满120秒、未完成抓取，黑棒仍在桌面。
报告 `validation/task_trial1000_long20deg_20261008_r1.json`，同名目录
summary.json及双路视频完整解码（base744帧、wrist734帧）。


用户明确授权先归零，再以每步20°上限重跑；沿用最长120秒和速度5%。
新增可复用 `laptop.robot_client.bounded_zero_return`：每段最多5°，
反馈匹配后继续，夹爪保持；只接受六轴起点均在±60°内，不做零点标定、
复位或夹爪关闭。每段到位最多等待5秒，无发送重试；报告路径须唯一。
本次归零6段完成，六轴反馈零位±0.1°，报告
`validation/zero_before20deg_20261008_r1.json`。这表示位置回零，非重新标定。
模型入口新增显式 `--joint-step-deg 20`；默认仍5°，底层默认仍1°。
最长120秒、累计相对起点最多60°、夹爪每步名义1mm及所有SDK限位保留。
20°相关控制测试65项、归零分段测试2项通过。当前服务仍1000步模型。

复用归零命令：

```powershell
E:/anaconda3/envs/pi05_laptop/python.exe -B -m laptop.robot_client.bounded_zero_return --output validation/zero_before20deg_20261008_r1.json --site-cleared-zero-authorized
```

复用长循环命令：

```powershell
E:/anaconda3/envs/pi05_laptop/python.exe -B -m laptop.client.bounded_model_trial --identity-lock validation/live1000_identity_20261008.json --output validation/task_trial1000_long20deg_20261008_r1.json --long-loop --duration 120 --joint-step-deg 20 --site-cleared-long-loop-authorized
```

### 10°、两分钟试跑配置（2026-10-08，最新授权）

已执行本次实机试跑，实际约47秒，发送20次目标更新，前19次反馈匹配。
第20次未在2秒跟踪检查内确认目标，STOPPED_NO_RETRY，未自动重试。
最后反馈完整、新鲜，控制器error_code=0；不等价于机械臂已硬停止。
实际最大目标变化10°，base1425帧、wrist1412帧视频均完整解码。
未达到两分钟，未确认任务成功。报告
`validation/task_trial1000_long10deg_20261008_r1.json`，视频及summary.json
在同名目录。后续须核对低速下到位时间与最后偏差，不应继续增加步长。


用户明确要求每步10°并运行两分钟。入口支持 `--joint-step-deg 10`
与 `--duration 120`，长循环最多120次更新，累计相对起始姿态仍最多60°。
速度仍5%，夹爪每步名义最多1mm，SDK限位、反馈稳定及过期检查保留。
10°须显式配置；底层批次默认1°，CLI默认每步5°。长循环默认时长120秒。
父进程超时预算为运动时长加35秒，监督器硬上限180秒；硬超时停止发送
不等价于硬件急停。关节步长、帧编码与相关反馈测试63项通过。
前轮最后300ms原始反馈确认J4漂移0.110°，其余轴该窗口漂移0，
因此稳定窗口失败有实际反馈证据，不能简单视为无用安全门。
此次复用命令：

```powershell
E:/anaconda3/envs/pi05_laptop/python.exe -B -m laptop.client.bounded_model_trial --identity-lock validation/live1000_identity_20261008.json --output validation/task_trial1000_long10deg_20261008_r1.json --long-loop --duration 120 --joint-step-deg 10 --site-cleared-long-loop-authorized
```

### 5°上限连续实机试跑结果（2026-10-08）

用户澄清要求连续长程运动，已复用bounded_model_trial，以1000步模型、
5%速度、每步最多5°、最长60秒运行，夹爪仍每步名义最多1mm。
约54秒后停止，完成24次更新、24次关节反馈匹配、120帧适配器接受。
实际最大目标变化达到5°，不是1°测试。下一次预检未通过300ms稳定窗口
（同时校验SDK范围与最多0.050°漂移），程序STOPPED_NO_RETRY，无自动重试。
该组合报错尚未拆分具体轴和具体条件，不能仅凭文本断言是哪轴抖动。
模型六轴目标方向反转次数为[10,15,9,14,7,14]，扩大步长未消除目标反转。
base1642帧、wrist1623帧完整解码；结束画面黑棒仍在桌面，抓取未完成。
报告 `validation/task_trial1000_long5deg_20261008_r1.json`；视频、时间戳和
summary.json在同名目录。程序已停止发送，不视为断电或停机保持的证明。
后续应先定位目标反转与稳定窗口失败，再决定轨迹平滑或模型调整，
不能把继续增大步长当作已经证实有效的修复。

### 每步关节上限调整为5°（2026-10-08，最新配置）

随后按用户授权执行一次1000步模型实机更新，速度5%、上限5°。
六轴目标变化为[+3.541,-1.501,-0.955,-0.627,+4.936,-2.729]°，
反馈全部匹配，最终error_code=0；夹爪预测为负数，按规则保持。
已停止发送，未执行连续抓取；此单步结果不证明抖动消失。
报告 `validation/task_trial1000_5deg_20261008_r1.json`，双路视频及
逐帧时间戳保存在同名目录。


用户报告上一轮机械臂抖动，要求将1°上限改为5°。已完成代码修改，
本轮未发送CAN或执行新运动。增大位置步长不能证明会消除抖动。
可复用入口 bounded_model_trial 新增 `--joint-step-deg 5`（默认5），
可用 `--joint-step-deg 1`恢复原限制；预测限幅和底层发送校验使用同一值。
底层批次默认仍为1°，调用者须显式携带5°限额，超过5°直接拒绝。
长循环累计60°边界、最长60秒、速度5%、夹爪名义1mm、SDK限位和
过期/异常停止规则保持。短循环累计边界为更新次数×单步上限。
包含5°精确编码、5.001°拒绝和夹爪限幅的相关测试61项通过。
下一轮实机需先检查已记录视频和命令方向，定位抖动；不要把5°当作
消除抖动的结论。此前31次反馈匹配只说明达到各次目标，不说明轨迹平滑。

### 1000步模型60秒长循环入口（2026-10-08）

实机试跑已执行：约60秒，31次目标更新全部获得关节反馈匹配，
155帧被适配器接受。base视频1818帧、wrist视频1801帧均完整解码。
结束画面黑棒仍在桌面，未完成抓取放置，不应记为成功任务。
结束状态无报告的控制故障；程序已停止发送，未追加自动试跑。
结果摘要 `validation/task_trial1000_long_20261008_r1/summary.json`。

用户明确授权并现场看护。复用 `laptop.client.bounded_model_trial`，
新增 `--long-loop --duration 60 --site-cleared-long-loop-authorized`。
默认短测试仍最多3次、累计3°；长测试最多60次、最长60秒，累计最多60°，
同时始终受官方关节限位约束。速度5%，每步每轴最多1°，夹爪名义最多1mm。
反馈过期、异常、跟踪失败或发送结果不明即停止，不自动重试。
双路摄像头复用 live_shadow 的统一读取函数，并增加可选录制回调；
MJPG AVI按30fps编码，逐帧JSONL记录实际主机到达时间，不能视为曝光同步。
每次更新保留独立CAN收发日志，视频目录中progress.json保留过程结果。
相关控制、限幅、CAN和USB测试59项通过。实测报告：
`validation/task_trial1000_long_20261008_r1.json`，视频及时间戳在同名目录。
运行仍使用1000步checkpoint999的身份锁；5000步模型未替换该服务。
此入口的“循环完成”不等于抓取成功，须结合视频和现场结果记录。

复用命令（仅在获得该次现场运动授权并确认现场条件后运行，输出须唯一）：

```powershell
E:/anaconda3/envs/pi05_laptop/python.exe -B -m laptop.client.bounded_model_trial --identity-lock validation/live1000_identity_20261008.json --output validation/task_trial1000_long_20261008_r1.json --long-loop --duration 60 --site-cleared-long-loop-authorized
```

### 5000步训练完成（2026-10-08）

后台检查核验COMPLETE，独立训练5000次更新，保存checkpoint4999。服务器
完成UTC2026-10-07T16:09:24（新加坡2026-10-08 00:09:24），未自动启动10000步。
最后一步loss0.0034；最后100步平均0.014172，最初100步平均0.047029。
1000步最后100步平均0.022951，仅比较训练loss，不代表抓取成功或泛化。
报告 `validation/train5000_report_20261008.json`、完整日志
`validation/train5000_completed_20261008.log`，loss汇总、CSV和曲线分别为
`validation/train5000_loss_20261008.json`、`.csv`、`.png`。
checkpoint来源哈希7e8cbb5f74a785e224dbc230cd9ab980eb32650c221ee1d63cb5653c3dcc1adc。
该模型尚未做离线预测或实机试跑，当前8020服务仍为1000步模型，不可混淆。
复用loss分析入口 `server.training.summarize_training_loss`；绘图使用本机已有
E:/anaconda3/python.exe的Matplotlib环境，未安装或修改训练环境依赖。
后台检查5000工作完成，暂停自动化5000，本轮没有连接CAN或发送动作。
用户已授权完整任务长循环，但程序扩展与验收尚未执行；该目标仍待后续推进。

### 1000步训练与离线对比（2026-10-08，已完成）

完整日志确认1000步，最后loss0.0347，最后100步平均0.022951；最初100步平均
0.047029。日志 `validation/train1000_completed_20261008.log`，loss汇总
`validation/train1000_loss_summary_20261008.json`。
复用evaluate_demonstrations，checkpoint999来源校验通过，在同一数据、相同15行
做离线预测。关节平均绝对误差20步20.8596°，1000步
2.7393°；夹爪名义误差20步27.5935mm，
1000步2.3462mm。非留出集，不宣称泛化或完整任务成功。
报告 `validation/eval1000_report_20261008.json` 和
`validation/eval20_vs1000_20261008.json`。本次自动检查未访问CAN或发送动作。
1000步检查工作已完成，暂停自动化1000-checkpoint；5000步训练与其检查继续。

### 1000步checkpoint实机测试（2026-10-08，已完成3次更新）

1000步训练已COMPLETE，保存999，完成报告
`validation/train1000_report_20261008.json`；服务器服务PID2821626、GPU0、回环8020，
由正式复用入口serve_trained_demonstrations启动，identity锁
`validation/live1000_identity_20261008.json`。没有用20步模型替代。
实时三次预测通过，延迟108–152ms。用户明确现场看护后执行5%速度、3次更新，
每次关节≤1°、夹爪名义≤1mm，共15帧被适配器接受，三次关节反馈匹配。
最终六轴反馈约[6.563,6.883,-4.812,-3.373,1.654,2.862]°；夹爪raw2500。
用户观察“机械臂有轻微的变化”，与限幅试跑范围一致。
报告 `validation/task_trial1000_20261008_r1.json`，用户观察及身份绑定汇总
`validation/task_trial1000_outcome_20261008.json`。执行后Base照片显示黑棒仍在
原位置，未完成拾放，不能把控制反馈匹配称为抓取成功。没有追加动作。
后续比较离线误差及更长任务动作轨迹后再安排试验；GPU2的5000步训练保留。

### 1000步实机测试授权与等待（2026-10-08）

用户要求1000步真机测试，并明确确认任务物体和相机准备、上位机断开、现场
清空、手离开、急停/电源可用及现场看护；范围5%速度、最多3次更新、每次关节
≤1°和夹爪名义≤1mm。已检查Base图和CAN_CTRL只读反馈，报告
`validation/task_scene_1000_preflight_20261008_r1/report.json` 与
`validation/site_1000_preflight_20261008_r1.json`。本轮尚未发送动作。
1000步优化已结束，但服务器999临时checkpoint仍写盘，train1000_report状态
RUNNING；不能加载半成品，也不能换回20步模型冒充1000步测试。
新增正式服务入口 `server.training.serve_trained_demonstrations`，只接受COMPLETE
训练报告，核验模型和数据后导出客户端identity lock，支持后台启动。
保存完成后接入此服务、实时输出检查，然后执行上述已授权范围的试跑。

### 并行1000步checkpoint测试（2026-10-07，已启动）

用户要求在空闲GPU单开1000步保存checkpoint测试，保留5000步训练。
重新检查发现GPU0空闲，GPU1已由其他用户的新训练占用；因此使用GPU0。
复用 `server.training.train_demonstrations`，新建 `piper_new3_1000step_r1`，
同一三条成功示教595样本、相同数据及统计量哈希，从基座独立训练1000步。
GPU0、batch1、显存池95%，PID2809074。运行根仍为
`/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/new_demo_tinyfit_20261007_r1`，
报告 `train1000_report.json`、日志 `train1000_report.log`。
最终checkpoint序号应为999（1000次更新），以实际COMPLETE报告为准，尚未完成。
GPU2的5000步PID2800586不停止，也不自动追加10000步。

已新增可复用离线评估入口 `server.training.evaluate_demonstrations`，读取COMPLETE
训练报告，核验checkpoint/data/stats来源，每条默认5帧，只预测、不访问机器人。
例：在runtime_root运行
`python -B -m server.training.evaluate_demonstrations
--training-report ../train1000_report.json --output ../eval1000_report.json --gpu-index 0`。
训练完成后比较1000步与20步结果，再决定实时接入测试。自动化
“检查1000步checkpoint并测试”（id1000-checkpoint）每5分钟检查，正常运行静默，
完成后离线评估并报告；不自动执行机械臂任务，完成或失败后暂停检查。

### 三条成功示教5000步训练（2026-10-07，后台运行）

用户先要求5000再10000，随后明确改为先只跑5000看loss；本轮不自动跑10000。
训练从基座新建实验 `piper_new3_5000step_r1`，复用三条成功示教595样本与已绑定
归一化，不是在20步checkpoint上恢复optimizer。GPU2、batch1、XLA显存池95%。
后台PID2800586，启动UTC2026-10-07T15:35:04；日志已验证至少到step407，loss0.0058，
尚未完成，也不据单步loss宣称收敛。
服务器运行根 `/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/new_demo_tinyfit_20261007_r1`：
`train5000_report.json`、`train5000_report.log`、`train5000_report.launch.json`。
训练完成后记录最后一步与最后100步平均loss，再决定后续训练。

正式复用入口 `server.training.train_demonstrations`：步数1..10000、数据路径与
哈希、实验名、GPU、前台/后台、日志、失败报告与来源绑定；指南
`server/training/TRAINING_USAGE.md`。12项针对测试通过。旧tiny-fit默认100步上限
保持，新入口显式传allow_long_training。本入口不访问CAN，也不自动启动实机。
可通过其parser/run调用，或CLI运行，后续修改参数即可复用。

GPU资源：已核实并停止本项目GPU2试跑推理PID2776446、GPU1历史推理PID1441963；
GPU1显存已降至31MiB。GPU0是另一用户user的活跃训练，保留；GPU3不在释放范围。
GPU2现用于5000步训练。旧8020推理服务已停止，训练期间不要运行机器人试跑。
已设置本聊天每10分钟检查训练的后台自动化“检查三条示教5000步训练”（id5000），
正常运行且状态无变化时静默；完成下载日志、汇总loss及更新plan后通知，失败报告
原因，结束后暂停检查。训练本身由服务器独立后台进程运行。

### 恢复场景后的任务试跑（2026-10-07，已执行）

用户确认场景恢复并要求执行。查看r2双相机快照，Base可见机械臂、黑棒和绿色
垫子；Wrist仍朝向夹爪前方。执行现有 bounded_model_trial 首段3次更新，5%速度，
关节每次≤1°、夹爪名义≤1mm。三次关节反馈均匹配，15帧被适配器接受。
同进程三次USB重开成功，显式资源释放修复首次得到实机多步验证。
最终关节反馈 [5.745,5.858,-5.761,-5.843,1.923,5.865]°，夹爪raw2100。
报告 `validation/task_trial_20261007_r1.json`；结果和画面绑定汇总
`validation/task_trial_outcome_20261007.json`。
执行后Base图 `validation/task_scene_after_segment1_20261007/base_snapshot.jpg`
显示夹爪仍在黑棒上方，物体仍在原处，没有抓取或放置成功。本次只是短段任务
试跑，不标记为完整任务成功。没有继续扩大步幅或追加运动。
下一步检查训练与现场视角、动作分布，并增加本次checkpoint训练步数后重新
预测检查；完整拾放能力尚未得到证明。

### 完整抓取请求的现场检查（2026-10-07）

用户授权尝试抓取，现场看护电源开关。使用双相机snapshot和实时只读预测检查，
报告 `validation/task_scene_preflight_20261007_r1/report.json` 与
`validation/task_model_preflight_20261007_r1.json`。模式CAN_CTRL、状态正常、error0，
三次实时预测通过。但查看真实图片：Base朝墙面/天花板，Wrist朝夹爪及杂物桌面，
两路均未见黑棒或绿色垫子。因此没有发送完整抓取动作；等待用户恢复示教场景
和相机视角。用户任务试跑授权保留，场景恢复后先重新检查画面再继续。
模型第一行J2目标约52–56°，夹爪名义目标三次约31.7、2.7、66.1mm；
这些只能作为模型输出，不能视为已验证抓取计划或直接大幅下发。

### 模型小幅实机试跑（2026-10-07，已执行）

用户明确要求执行并授权现场清空、急停可用、5%速度、最多3次更新，六轴每次
不超过1°、夹爪名义每次不超过1mm。已完成3次模型驱动小幅更新，共15帧被
适配器接受，三次六轴反馈均在目标±0.1°内。最终六轴反馈约
[2.849, 2.952, -2.832, -2.842, -0.982, 2.960]°；夹爪末次目标raw1300、
反馈raw900，存在位置反馈变化，不能等同于真实物理开口毫米标定。
汇总：`validation/new_demo_model_trial_summary_20261007.json`。
实际发送分别保存在r3、r4、r5报告；r1/r2发送前停止。
r3/r4各执行1次后重开USB失败，先只读复查恢复，分别只继续剩余2次、1次，
总计严格3次。已增加显式PyUSB资源释放；修复后的多步同进程重开仍待实测，
不能称为已通过。最后r5单次成功，不追加任何动作。

复用程序 `laptop.client.bounded_model_trial`，明确实验入口，不伪填完整P12验收。
使用既有双相机、PiperSession、SDK范围、稳定反馈与碰撞检测；模型网络推理
在独立线程执行，CAN owner持续接收，保留120ms反馈新鲜度要求。
未来动作行按推理耗时选择，预测来源保持原值，并另外记录限幅结果；总来源
年龄250ms，最大3次，每次独立preflight，失败停止，不自动重发。
固定帧为CAN/MOVE_J/5% + 0x155–0x157 + 0x159；夹爪effort0、enable1、zero0，
来自已核对OEM组合，不设置夹爪零点，不复位、不使能电机。
默认发送路径仍拒绝，新增typed bounded trial仅接受明确批准的小幅实验。
相关验证40项通过，USB transport与trial补查24项通过（包括USB资源释放测试）。

复用命令（必须在现场准备并明确授权对应次数后运行；不要现在重复）：
`python -B -m laptop.client.bounded_model_trial
--identity-lock validation/new_demo_live_identity_20261007.json
--output validation/NEW_UNIQUE_TRIAL.json --updates 1
--site-cleared-three-updates-authorized`。
`--updates`支持1..3，默认3；服务端口默认8020，经SSH隧道连接服务器本次step19。
本次只证明小幅反馈闭环，没有完成抓黑棒放绿垫，也没有验证连续长程控制。
下一步核对现场动作是否平稳，扩充成功示教并训练更多步，再扩大实机试验范围。

### 新模型现场接入（2026-10-07）

已启动本次 step19 的服务器服务（GPU2、回环8020、PID2776446），本机通过
SSH 隧道访问。复用双路相机名称解析和只读 CAN 客户端，连续取得3份现场预测，
报告 `validation/new_demo_live_preview_20261007_r2.json`；无 CAN TX。
修复只读客户端诊断锁存影响接收的问题：诊断使用独立 FakeCanTransport bridge，
真实接收 owner 保持独立，控制发送门未改变；24项测试及20子测试通过。
可复用入口 `laptop.client.live_model_preview`，参数示例：
`--read-only --identity-lock validation/new_demo_live_identity_20261007.json
--output validation/NEW_UNIQUE_REPORT.json --duration 15`（默认端口8020）。
新增 `laptop.robot_client.bounded_policy_target.limit_target`：各轴每次最多1°、
夹爪名义每次最多1mm；关节目标遵守SDK范围，非法夹爪预测保持，拒绝非有限值。
4项针对测试通过。此函数只生成候选目标，尚未连接实机发送入口。
带限幅候选报告 `validation/new_demo_bounded_preview_20261007.json`。
用户明确批准现场清空、急停可用、5%速度、最多3次更新及上述步幅。
但实时读到 TEACHING_MODE / STOP_RECORDING / error0，不能直接执行控制。
当前J5约9.7°，没有零位支撑确认，禁止套用含复位的模式初始化。
用户随后确认零位准备及复位切模式授权。复用 piper_tool mode-init，实测已经
CAN_CTRL、六轴原始反馈全0、六电机全使能，返回 ALREADY_CAN_CTRL_NO_WRITES；
因此没有复位、没有控制帧。报告 validation/new_model_mode_init_20261007_r1.json。
零位后实时预测报告 validation/new_demo_live_at_zero_20261007.json。
此段为试跑前状态；随后已完成上述3次小幅实机更新。不能将模板中的物理夹爪
尺度、watchdog、断联保持等未核验字段伪填为已验证。

### 可复用示教采集入口（2026-10-07 更新）

新示教服务器 tiny-fit（2026-10-07，已完成离线半程）：复用采集、转换和训练程序，
三条完整可对齐成功示教生成 595 样本，10 fps。服务器 GPU 2、batch 1 训练
20 steps，保存 `piper_new3_20step_r2/19`；随后抽取每条五帧，共15帧预测。
首次显存分配不足，调整 XLA 显存池后完成；报告接口缺少统计量哈希已修复，
复用本次新训练 checkpoint 完成评估，没有重复训练。评估来源标识的旧循环变量
已修复，并按 source_map 修正报告，预测数值和误差不变。
关节平均绝对误差名义 0.36407 rad（约20.86°），夹爪名义 0.02759 m；
这只是训练数据上的链路验通，20步不足以证明任务学习，尚无留出集或实机成功。
预测出现负夹爪值，不能直接下发。五条完整成功示教目标尚未完成。
本地报告 `validation/new_demo_tinyfit_20261007_corrected.json`、训练日志
`validation/new_demo_tinyfit_20261007_train.log`。服务器隔离运行目录：
`/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/new_demo_tinyfit_20261007_r1/runtime_root`。
下一步补两条完整成功示教，复用相同程序扩充数据，再训练并检查预测；之后先
接入 prediction-only 服务验证本地请求，再进行有明确动作范围授权的实机短环。
本轮未连接 CAN、未发送机器人动作。

四条补采核验（2026-10-07）：session `a055d8c05c4b48559f8c51a8b093ae8f`
完整录四条，操作员标3成功1失败，原始哈希、视频全解码、标注绑定均复查。
第1条成功 `d4c544a9...` 腕部最大采样间隔239.6401ms，严格50ms对齐失败，
暂不加入完整episode训练集；第2条 `bca3e52f...` 操作员标失败，虽可对齐也排除。
第3条 `0cd272a0...` 200对齐帧/199proxy，第4条 `992b4ed3...` 199帧/198proxy
转换通过。连同首次辅助成功198proxy，目前4条成功标注、3条完整可对齐成功，
合计595proxy样本。需补1条成功以达到5条标注；另需处理d4c腕部空档，不能
把缺帧伪造为同步观测，也不自动扩大时间容差。此处为训练前审计记录；随后训练完成，仍未访问机器人。
完整报告 `validation/new_four_demo_audit_20261007.json`。

首条结果确认：用户确认 episode `6a5a627971ee4347a0d3533fd7ce2921` 成功，
手扶机械臂，夹爪抓黑棒并放到绿色垫子。已写入外部哈希绑定 outcome：
`datasets/manifests/outcomes/6a5a627971ee4347a0d3533fd7ce2921.json`，明确标记
物理辅助示教，不算自主成功。已用复用程序生成198条10fps未来状态proxy，
输出 `datasets/derived/6a5a627971ee4347a0d3533fd7ce2921_success_proxy`。
过渡审计显示98条有至少一轴超过首步1°限制，最大J5单步16.49°；
夹爪10条超过名义1mm首步限制。原目标保留，不自动放宽实机限幅，
可用于离线tiny-fit，不能直接执行原示教速度。五条目标现完成1条，还需4条。

首条新示教恢复（2026-10-07）：episode `6a5a627971ee4347a0d3533fd7ce2921`
录到 Base603/Wrist564 帧与19.993秒完整状态窗。旧检查把两路各2个首尾边界帧
当作整条失败；已改为边界告警，仍保留哈希、完整视频解码、非空通道、
单调时间和公共窗检查；公共窗内相机不足2帧仍失败。反馈示教派生对齐只选
公共完整状态窗内相机帧，原始视频与清单不修改。复查 FILE_INTEGRITY_PASS，
生成199帧10fps preview；成功结果后由用户确认并独立标注（见上段）。
证据 `validation/first_demo_reaudit_20261007.json`、
`validation/first_demo_alignment_20261007.json`。继续使用同一可复用采集入口。

夹爪 OEM 参数实机测试（2026-10-07）：用户明确授权继续，并说明上位机行程
设置 70 mm、实际可执行约 0–10 mm。设备只读 0x47E 返回最大行程配置 100 mm；
这三者不一致，保留为未解决标定问题，不将 raw/1000 当作已验证物理开度。
首轮因 raw99500 超过旧70mm假定而在发送前停止（0运动帧），随后按读回配置
核对反馈范围，并在用户明确同意后执行目标 raw54015、effort=0、code=1、
set_zero=0 的有界重复发送。19 帧后目标反馈匹配，最终 raw54400，误差385，
六轴反馈全0、六轴使能、无错误；发送窗口最多5秒，本次连同末尾观察约0.797秒。
状态 `OEM_GRIPPER_TARGET_FEEDBACK_MATCH`。这证明该参数组合可以驱动反馈跟踪，
不证明真实毫米标定，也不能单独归因于力度或发送频率。
可复用测试入口 `laptop.robot_client.supervised_oem_gripper_test`；证据
`validation/site_oem_gripper_test_20261007_r2.json` 及其 CAN 原始日志。
参数查询证据 `validation/site_oem_test_gripper_parameters_20261007_r1.json`；
相关离线测试41项通过。尚未接入采集前自动回位/闭合，不得自行重复运动测试。

官方上位机发送观察（2026-10-07）：`laptop.diagnostics.capture_oem_can`
记录窗口固定 5 秒，支持开始前 10 秒倒计时。首轮 candle 入口记录为空，
用户确认夹爪有动作；第二轮发现实际走 `cando.dll/cando_frame_send`。
只读反汇编证明第二参数为发送至 USB 的 24 字节缓冲区。
第三轮实际抓到 34 条 0x159、5 条 0x477，接口返回值均为 1。
夹爪目标 raw 从 42336 增至 54015 再降至 0，effort 全为 0、code=1、
set_zero=0。相邻夹爪调用间隔中位数 13.2068 ms，范围 0.4617–624.8886 ms，
不是固定 5 ms 周期。因此此前“低力度是主要原因”的解释未经证实；也不能把
effort=0 自动解释为无驱动力或据此直接增力。需对照现场输入/实际动作，再
设计与上位机参数一致的有时长上限控制测试。当前证据是发送 API 参数，
不是机器人执行反馈；候选字段布局仍保留与 UI 交叉核对标记。
证据：`validation/oem_gripper_capture_20261007_r3/report.json`、
`validation/oem_gripper_capture_analysis_20261007_r3.json`。
观察钩子临时修改进程内存并增加开销，但不改参数、不主动发控制帧，已卸载。
使用见 `laptop/diagnostics/OEM_CAN_CAPTURE.md`。

夹爪最大开度（2026-10-07 20:32 SGT）：用户授权执行目标 raw=70000、
effort=100（名义 70 mm / 0.1 N·m），已单次发出 0x159 `0001117000640100`。
六轴维持零位、无错误；发送前夹爪反馈 700（前次闭合后继续变化），3 秒后 1000，
随后独立只读复查为 1200。尚未达到最大开度，不能报告完成，未重发或增加力度。
证据：`validation/site_gripper_max_open_20261007_r1.json` 与
`validation/site_gripper_max_open_observe_20261007_r2.json`；相关离线测试 11 项通过。

现场回零（2026-10-07）：用户明确授权六轴回 0° 和空夹爪闭合并确认现场清空。
`supervised_near_zero_home` 以 5% 速度发出近零回位，最终六轴原始反馈均为 0，
CAN_CTRL、六轴使能、无错误。第一次连接后续闭合预检被自身回位帧回读拦截，
已改为关闭回位连接、独立连接重新预检，保留冲突检测而不清除标记。
独立单次夹爪闭合命令（目标 0、名义 0.1 N·m、无零点标定）已发送，
反馈仍为 5400；不能报告夹爪闭合成功，没有重试或加力。
证据：`validation/site_home_and_close_20261007_r1.can.jsonl` 与
`validation/site_close_at_zero_20261007_r1.json`。尚未接入采集前自动回位。

后续示教统一调用 `laptop/recorder/record_demonstrations.py`，使用指南见
`laptop/recorder/USAGE.md`。先真实录制一条并检查，再真实补录四条，禁止把
复制同一条算为五条。默认 20 秒、5 秒开始等待，每条人工确认成功/失败。
复用已有双视频与 CAN 反馈采集；按已确认的 ACER Base / Dabai Wrist 名称
解析当前 DSHOW index 并向底层传递；任务写入原始文件与会话清单。
采集不发送控制帧、不切模式。用户本轮明确要求先完成程序，稍后开始示教，
因此不得将本轮程序验证写成新成功示教。命令在普通 PowerShell 或 Codex
沙箱外运行：`python -B -m laptop.recorder.record_demonstrations --record --episodes 1`。
本轮相关离线测试 19 项通过，CLI 帮助入口已检查；尚未执行新的现场示教。

目标分为两个可验收的里程碑：先让 π0.5 的 7D 位置目标经 PiPER 适配器完成
受监督的短时实机控制，再评估“拾起黑棒并放到绿色桌垫”的任务成功率。
下列安排根据本地文件和已完成的只读核对制定；旧示教复用得到的 20-step
prediction-only checkpoint 和历史帧回环已完成，但不表示新示教训练、已开启控制
或已取得实机动作授权。本节更新了下文历史 P12/P13 描述的
**执行顺序**：原厂命令帧只对“原厂命令监督”路线必需，不是明确标注为
`derived_future_observed_state_position_target` 的路线的训练前提。
第二台主臂也不是该路线的前提。
现场配置中的 `command_target_source_verified` 应核对所选目标语义及其可追溯来源；
它是操作员声明，程序只检查字段完整性，不会替人核验引用内容。本路线可引用哈希绑定的
未来实测状态 proxy，不要求原厂 CAN 命令帧。独立的
`policy_to_command_mapping_verified` 仍须由现场物理验证通过。

### 当前基线和真正缺口

- 本地成功 episode `cef166e2cc6240b397db241a916ece3a`、结果 sidecar 和
  10 fps/199-transition LeRobot 数据已存在；服务器保留旧 step-9 checkpoint，另有
  用该旧 episode 在隔离缓存新训的 20-step tiny-fit checkpoint。
  2026-10-07 只读运行 `select_successful_episodes(..., min_episodes=1)` 得到
  1 条成功、1 条失败、7 条未标注。旧成功示教有人手扶持夹爪，因此只用于
  旧模型复用和新流水线的离线验通，不作为自主抓取能力的证据。
- `capture_session.py`、`post_capture_pipeline.py`、多 episode 训练预检和
  `serve_derived_state_multi_shadow.py` 已在本地实现。旧示教的 20-step checkpoint
  和历史帧回环已完成；新采集的成功 episode、以新示教训练的 checkpoint，以及
  使用现场新观测的 shadow 仍不存在。服务器工作树有未提交改动；新增文件继续只放
  隔离缓存，不能覆盖 checkout；选卡前要读当前 GPU 状态。
- 当前笔记本的只读设备清点已看到 3 台已启动摄像头（ACER、Dabai DC1、USB Camera）
  和一只 `1d50:606f` candleLight 适配器。WMI `Get-PnpDevice` 被系统拒绝访问，代码已
  增加只读 `pnputil /enum-devices /class Camera /connected` 回退；但第三台摄像头出现后，
  操作员本轮确认 OpenCV index `1` 是 Wrist、index `0` 预计是 Base；这与此前签名会话
  `0=ACER/Base、1=Dabai/Wrist` 一致，且两个 PnP ID 仍存在。当前 index `0` 无法打开，
  index `3` 仅返回近黑画面；当前 index 到设备 ID 的对应关系仍需在恢复视频流后复核。
- 控制侧已有 `policy_sdk_preview.py` 的 7D→PiPER SDK/CAN 编码预览和历史
  关节受监督回放代码；真实通用 `GsUsbPiperCanTransport.send()` 仍拒绝运动命令。
  低层四帧位置发送原语现已通过专用 `SafetyBridge` 入口接入，并由单独配置项
  显式开启；没有修改通用 `send()`。新的策略单步函数可完成预测到批次并要求独立
  授权后发送。独立 `supervised_live_step.py` 已接入现场配置、模型身份、相机绑定、
  CAN 控制模式检查和推理前的一批操作员授权；只有显式 `--supervised-position-step`
  与口令确认后才尝试一个 4 帧批次。CAN 必须已处于 `CAN_CTRL(0x1)` 且机械臂状态正常；
  程序不会自行切换控制模式。默认 shadow 仍是只读。
  六轴方向/尺度、夹爪反馈零点和命令映射尚未现场验证，空白 site profile 也尚未完成，
  因此本轮没有运行该入口或发送运动帧。
- 新采后入口默认 20 steps、最多 100 steps；这一级只验通数据和训练链路。
  旧 step-9 checkpoint 的同轨迹五帧关节 MAE 为名义 `0.29106 rad`；新训 20-step
  checkpoint 的同轨迹五帧关节 MAE 为名义 `0.25373 rad`、夹爪绝对误差为名义
  `0.01326 m`。两者都是同一条旧 episode 上的离线预测结果，不能作为泛化或抓取成功
  证据，也不能直接发送未限幅的预测。

### 2026-10-07 实际执行进展

- **最新 20:05：双路摄像头统一复用入口已实现并实测，正常读取方式须保留。**
  用户要求把正常读取方式写入 plan 和记忆，后续图像/视频采集使用统一程序。
  新入口 `python -B -m laptop.camera.dual_camera {check,snapshot,video,frames,all}`，
  每次传新的 `--output-dir`，DSHOW 固定后端，按 ACER HD User Facing / Dabai DC1
  名称解析当前 Base/Wrist 序号（当前 0/1），缺失/重名/显式序号不匹配则拒绝，
  不盲扫 Broadcast 虚拟摄像头。Python 提供 capture/on_frame 逐帧回调及
  supervised_capture 受监督进程入口。两路独立读取线程，记录每帧真实
  perf_counter_ns 到达时间、帧序号、形状与 PNG/视频索引；BGR 数组由调用方按需转 RGB。
  图片按固定时间槽选帧，记录实际时间，不伪造严格 100 ms 时序。视频全量解码核对帧数。
  CLI 有进度超时、目录独占创建和中断保留机制；不访问 CAN、不自动结束其他应用。
  正常命令必须在 Codex 沙箱外执行，即 exec_command 使用
  `sandbox_permissions="require_escalated"`；这不是要求用户普通终端必须以 Windows
  管理员身份运行。普通沙箱对 ACER 的 0x80070005 不能直接解释为硬件故障。
  项目 Python 仍为 `E:\anaconda3\envs\pi05_laptop\python.exe`。
  公共 source.open_camera 已接入旧双路 probe/recorder 路径和 live_shadow 的摄像头
  循环，避免各处另写后端选择；历史 episode schema 未改。
  离线定向测试 **36 passed、20 subtests passed**。统一脚本先做 3 秒 all 实测，
  双视频回读 92/83 帧；最终版本另做 2 秒 all 实测，视频回读 62/53 帧与写入一致，
  保存 21/18 张 PNG、两张末帧 JPG、两份 JSONL，CAPTURE_PASS，零读取失败。
  证据 `validation/dual_camera_reusable_live_20261007_r{1,2}/report.json`。
  使用说明在 `laptop/camera/README.md`。未验收硬件曝光同步、相机/CAN 对齐或模型控制。

- **最新 19:53：Base/Wrist 双路初调通过，此前 ACER 打不开由执行沙箱访问限制导致。**
  用户确认目标摄像头名称 ACER HD User Facing，并报告 Windows“相机”应用有画面。
  当前 DirectShow 枚举 0=ACER、1=Dabai、2=Broadcast 虚拟设备。在普通沙箱内，
  按 ACER 确切设备 moniker 绑定滤镜返回 `0x80070005 E_ACCESSDENIED`；
  在 `require_escalated` 沙箱外执行中，同一设备滤镜绑定和 OpenCV index 0 读取均成功。
  因此纠正此前主要按 Broadcast 占用/驱动故障排查的判断；不需要根据这次证据换驱动。
  相机设备 restart 被 OS 拒绝，实际没有重启；Broadcast 主应用/UI 已结束，
  专用容器未能结束，但在该状态下双路仍通过。
  沙箱外 DSHOW 双路 5.031 秒：Base 153 帧、30.41 fps；Wrist 142 帧、28.23 fps；
  两路均 640×480，读取失败 0，有效图像，采样非近黑。到达时间差中位 13.38 ms、
  P95 59.36 ms、最大 292.03 ms；这是主机读回时间，不是硬件曝光同步，
  后续数据采集仍需处理启动期/偏差大的帧并核对与 CAN 的同步。
  Base 静帧可见机械臂、黑棒及桌面，绿色桌垫只露出左侧部分；Wrist 可见夹爪与桌面。
  初调通过不代表最终任务视野/视觉动作映射已验收。未打开 CAN、未发送运动、未录像。
  报告：`validation/camera_dual_unsandboxed_20261007_r1.json`；汇总：
  `validation/camera_initial_debug_summary_20261007_r1.json`；Base 预览：
  `validation/camera_acer_unsandboxed_20261007_r1.jpg`。
  后续在 Codex 调用真实相机采集时必须使用沙箱外工具权限；不能把沙箱的拒绝访问
  报错直接当作硬件故障或现场相机配置失败。下一步固定任务视野，再进行相机/CAN
  同步采集与现场新观测的模型 shadow；本轮没有模型实机动作。

- **最新相机初调 19:43–19:48：Wrist 有效，Base 设备角色待现场确认。**
  用户恢复相机调试，并在关闭 Broadcast 后要求重试 Base，随后指出 Base 可能不是 index 0。
  本轮未访问 CAN。Windows 当前物理摄像头清点为 ACER HD User Facing 与 Dabai DC1，
  均为 OK。index 1（此前用户确认 Wrist）单独实测 640×480、83 帧、约 29.985 fps，
  静帧可见夹爪和桌面；证据 `validation/camera_wrist_probe_20261007_r1.json` 与预览 JPG。
  index 0 经 DSHOW/MSMF 重试均无法打开。Broadcast 正常关闭窗口不可用，随后结束其
  主应用/UI 进程；两个 NvBroadcast.Container 进程因 OS 拒绝访问未能结束，
  不能声称已排除全部后台占用。扫描 index 2 时触发 Broadcast 虚拟滤镜重新启动程序，
  因此停止盲扫，改用仅 BindToStorage 的 DirectShow 名称枚举，不构建视频滤镜。
  新增 `laptop/diagnostics/list_directshow_cameras.ps1`，当前 DirectShow 枚举顺序：
  0=ACER HD User Facing，1=Dabai DC1，2=Camera (NVIDIA Broadcast，虚拟设备，无 USB 路径)。
  证据 `validation/camera_directshow_inventory_20261007_r1.json`。当前未发现第三台外接
  物理摄像头。已询问用户 Base 实际是内置 ACER 还是另一外接相机；未自动改相机角色配置。
  单帧/设备枚举不是双路并发、观测同步或模型视觉部署验收。相机序号只能作为当前
  后端的临时顺序，后续复用须结合稳定设备身份及画面确认。

- **最新 19:38：六轴及夹爪均已有小幅命令→反馈位置变化，精确标定与模型短环尚未完成。**
  用户追加明确授权单次夹爪 +5 mm / 0.3 N·m，现场仍清空。新鲜起始反馈为 raw=800
  （名义 0.8 mm），所以这次实际命令目标为 raw=5800（5.8 mm），发送
  `0x159 / 000016a8012c0100`，1.5 秒后反馈 raw=5400（5.4 mm），变化 4.6 mm。
  19:38:34 起另做 3 秒只读复查仍为 5400，六轴位置为
  `[929,976,-949,935,984,983]`，CAN_CTRL/MOVE_J/NORMAL、错误码 0。
  这证明夹爪命令有位置响应，但误差 0.4 mm 超过当前严格 0.05 mm 阈值。
  前一次 1 mm 测试的 1.5 秒窗口未观察到位置变化；后来起始反馈已为 0.8 mm，
  因而不能继续表述为“夹爪完全不响应”。不能仅凭两次同时改变目标/力度的试验归因于死区或力度。
  5 mm 报告与全过程 CAN 日志：`validation/site_gripper_open_5mm_20261007_r1.json`
  及 `.jsonl`；复查：`validation/site_after_gripper_5mm_readonly_20261007_r1.json` 及 `.jsonl`。
  已生成哈希绑定的七执行器响应汇总
  `validation/site_seven_actuator_response_20261007_r1.json`，保留严格失败结果、
  J2 瞬时标记及用户现场说明。J1 的小误差按用户要求不再阻塞后续，但不篡改历史结果。
  `piper_tool` 已增加 step-joint 与 step-gripper；所有已确认单次运动范围均已执行完，
  本轮不自动重复、回零或追加动作。定向回归 **91 passed**，差异空白检查通过。
  本阶段可记录“小幅发送与反馈响应核验完成”，不能记录“物理单位精密标定完成”。
  模型受监督单步与 3 步短环尚未运行：用户仍要求跳过摄像头，现有现场 profile
  仍是 DRAFT，机械包络/真实 watchdog/断联保持等证据尚未补全。
  不以临时把模板字段设为 true 的方式通过这些条件；下一步继续整理现场配置和可复用
  程序接线，恢复视觉输入后再执行模型单步/短环。

- **2026-10-07 19:29–19:35 逐轴实测已执行，夹爪位置响应尚未确认。**
  用户明确要求忽略 J1 精度误差作为后续阻塞；保留该误差原始记录，不修改旧报告或
  声称精确角度标定已完成。用户确认急停/净空/夹爪无物体、手已离开，批准以下范围：
  5% 速度，J2/J4/J5/J6 各单次 +1°，J3 单次 −1°，夹爪单次打开 1 mm。
  已执行五次单轴批次，每次四帧（固定模式帧+三个关节帧），其余轴保持实测位置，
  不发夹爪帧、不自动回零。最终名义变化：J2 +0.976°、J3 −0.949°、
  J4 +0.935°、J5 +0.984°、J6 +0.983°；所有最终反馈均 NORMAL/错误码 0/六轴使能。
  J5/J6 通过现有 ±0.050° 阈值；J3/J4 未通过严格精度阈值，但方向和小幅位置响应存在。
  J2 曾出现历史“越界或其他轴移动”聚合标记，当前报告不能还原瞬时轴和峰值；
  因此先停止发送，3 秒只读复查六轴每轴最小值=最大值，位置为
  `[929,976,0,0,0,0]`。用户现场确认无抖动/反向/碰撞/其他轴异常后才继续 J3。
  未将 J2 历史标记改成通过。J3–J6 新报告保存了全过程关节样本和逐轴最小/最大值。
  逐轴报告为 `validation/site_j{2..6}_one_degree_20261007_r1.json`；
  J2 复查为 `validation/site_after_j2_readonly_20261007_r1.json` 及 JSONL。
  夹爪单次发送 `0x159 / 000003e800640100`（名义目标 1 mm、力度 0.1 N·m），
  USB 接受；最终位置 raw=0，力度 raw=108，状态 raw=64，六轴保持不变。
  报告 `validation/site_gripper_open_1mm_20261007_r1.json` 及全过程原始 CAN JSONL。
  该结果不证明夹爪命令到位置映射通过；不自动增加力度或重发。
  已准备另一次夹爪 +5 mm / 0.3 N·m 固定范围及独立口令，因超过已确认的单次
  +1 mm 范围，已向用户请求新的范围授权，目前未执行。
  通信 API 新增 `open_gripper_step`；通用工具新增 `step-joint --axis {2..6}`。
  保留旧 J1 类型/模式名称以复用现有授权和 transport，但 batch 内明确绑定 selected_axis，
  仅允许 J3 −1°、J2/J4/J5/J6 +1°，不接受任意六轴向量，J1 原限制不变。
  控制相关定向测试 **89 passed**；本轮未开相机、未改服务器、未执行模型单步或短环。
  下一步先取得夹爪位置响应；之后仍须按实际证据补全现场配置，不得把单轴小幅反馈
  直接当作机械限位、断联保持、watchdog 或视觉策略部署验收。

- **通信复用方式修正：后续采用持续连接的 Python API，而非只依赖五个 CLI 子命令。**
  用户要求完整可复用的指令收发与状态检测程序。新增
  `laptop/robot_client/piper_session.py::PiperSession`，统一 connect/poll/state、
  六轴加夹爪 typed position batch、诊断查询、模式阶段、J6 使能、J1 批次、
  watchdog 检查、目标反馈等待及 close；一个会话持有一个 bridge 和一份 JSONL。
  状态包含缺失/过期通道、逐通道时间戳、关节时间差、模式/错误、六轴/夹爪/使能。
  RX 原始帧、TX 请求/接受回执/异常与目标检查均落盘；目标等待不重发。
  默认只接收；控制由调用方传入现有已配置 bridge、typed batch 和授权。
  模式初始化与位置控制沿用互斥 bridge 模式，分阶段建立会话。
  下一步采集/控制接入以此接口为基础；旧 CLI 目前仍保留原调用路径，尚未迁移。
  本轮未连接实机、未增加运动授权；该 API 的六轴/夹爪发送不代表现场映射已验收。
  尚未实现通用电机使能/失能、笛卡尔控制或自主策略循环；不得把这些记为已完成。

- **最新：六轴已使能，J1 单次 1° 发送取得反馈变化；统一 CAN 工具已实现。**
  用户授权 J6 使能后，发送一次 `0x471 / 0602000000000000`，读回六轴均使能，
  六轴位置保持 raw=0。随后授权的 +0.100° 三帧测试发送成功，但反馈保持 0。
  用户再授权一次 +1° 测试：发送固定 CAN/MOVE_J/5% 配置帧及三帧关节目标，
  J1 目标 raw=1000，读回 raw=929；19:19 新加坡时间另做 3 秒只读复查仍为 929，
  J2–J6 为 0，CAN_CTRL/MOVE_J/NORMAL、错误码 0。名义反馈变化为 0.929°，
  与目标误差 0.071°，超过当前 0.050° 验收阈值，因此保留未匹配状态，不自动重发。
  1°测试同时新增模式配置，不能把此前 0.1°未移动归因于某个单一因素。
  证据：`validation/site_j6_enable_20261007_r1.json`、
  `validation/site_j1_commissioning_20261007_r1.json`、
  `validation/site_j1_one_degree_20261007_r1.json`、
  `validation/site_after_j1_one_degree_readonly_20261007_r1.json` 及其原始 JSONL。
  新增 `laptop/robot_client/piper_tool.py`，统一 status、motor-status、mode-init、
  enable-j6、step-j1 五个子命令，复用现有 bridge/transport，不复制发送逻辑。
  每次必须提供新报告路径，先独占创建再访问设备；异常落盘且不重试。
  J1 仅允许 +0.1° 或 +1°，保留现场条件、口令及一次发送约束。
  既有控制测试 72 项通过；新增工具测试 6 项通过（默认 pytest 临时目录权限失败，
  改用项目内独立临时目录后通过）。统一工具未另做实机发送。
  下一步验证 J1 跟踪精度，再按授权范围扩展其他轴与夹爪映射；当前不宣称六轴尺度、
  夹爪映射或策略部署完成。摄像头仍按用户要求跳过；本轮未改服务器。

- **最新现场状态已是 CAN_CTRL，但 J6 未使能。**用户授权按官方流程复位/切模式，
  随后要求等待，最后确认已放到零位；支撑已就位且示教按钮灯熄灭。
  2026-10-07 18:59（新加坡时间）3 秒只读反馈实测六轴 raw 全为 0，控制模式
  CAN_CTRL(0x1)、MOVE_J(0x1)、NORMAL、错误码 0，五类反馈各约 600 帧。
  因模式已满足，未执行复位或再次切模式。随后 3 秒低速电机反馈每轴均取得 60 帧，
  J1–J5 已使能，J6 未使能且期间一致。因此 CAN 模式条件已通过，全六轴使能条件未通过；
  本轮仍无 CAN 数据发送，也没有执行 J1 步进。切换至 CAN/零位的来源未记录，不能把它
  归因于本轮新增代码。证据见
  [`零位和模式反馈`](../../validation/site_zero_pose_check_20261007_r1.json)、
  [`零位电机使能检查`](../../validation/site_zero_motor_enable_check_20261007_r1.json)。
  新增独立 `supervised_mode_initialization.py`，通过专用 bridge/transport 路径只允许
  一次 reset→standby→CAN/MOVE_J/5% 设置；要求支撑确认、零位附近的稳定新反馈，
  每一步发送后回读，待机未确认不得发 CAN 设置，发送不明则关闭且不重试。
  若初读已是 CAN，则不发送任何帧。该入口不发送使能或位置目标，尚未在实机运行。
  新入口与 J1/策略授权/live 的组合离线测试为 **52 passed**，5 个改动模块编译通过，
  三条帧均与本机 SDK 0.6.2 编码器逐字节一致。下一步先恢复并读回 J6 使能；
  J1 +0.100° 仍需要对应运动授权。详见
  [`模式初始化准备报告`](../../validation/mode_initialization_preparation_20261007_r1.json)。

- **现场只读两步检查已执行，控制模式仍未通过。**用户授权这两步，并确认其他 CAN
  程序已停止占用，急停、低速设置和净空可以；这些现场条件属于操作者声明。
  提权只读进程检查未找到命令行匹配本项目/PiPER/CAN 的 Python 进程，PnP 识别到
  一只 candleLight 主设备；这只是占用核查的辅助证据。
  2026-10-07 10:36 UTC 的 5 秒只读反馈取得五类消息各约 1,000 帧，解析错误为 0，
  状态 NORMAL、错误码 0，但模式仍为 TEACHING_MODE(0x2)，teach_status 为
  STOP_RECORDING(0x2)。随后 3 秒电机反馈确认六轴全部已使能且期间一致。
  两次检查均未发送 CAN 数据帧，未改变模式、使能或关节目标，未打开相机。
  证据见 [`现场反馈`](../../validation/site_feedback_check_20261007_r1.json) 和
  [`六轴使能反馈`](../../validation/site_motor_enable_check_20261007_r1.json)。
  厂商 2024 年快速手册 2.2 节要求退出拖动示教后先待机、再切 CAN，且切换前处于零位；
  官方 SDK demo 还提示示教后需要 reset。因此不能把下述“J1 无需先回零”的测试条件
  推广为“可以从当前示教姿态直接切 CAN”。需先确认按钮灯状态及适用固件的切换流程；
  本轮未执行 reset、模式切换或运动。手册来源：
  [PiPER quick start manual](https://static.generation-robots.com/media/agilex-piper-user-manual.pdf)。
  用户随后确认示教按钮灯熄灭，且其上位机没有控制模式切换入口。因此撤销本计划第 2 步
  必须通过现有上位机切模式的假设，改为准备独立的官方 SDK 模式初始化路径。
  2026-10-07 官方材料复核：ModeCtrl（等价 MotionCtrl_2）通过 0x151 设置
  ctrl_mode=0x00 待机或 0x01 CAN；move_mode=0x01 为 MOVE J，is_mit_mode=0x00
  为位置速度模式。官方 piper_ctrl_reset.py 明确要求示教/MIT 转位置速度控制时执行
  MotionCtrl_1(0x02,0,0)，即 ResetPiper；接口文档说明它会立即失力下落，不能当作
  无运动副作用的模式设置。厂商官方社区亦解释初次退出示教需要初始化，特别涉及
  J2/J3/J5 的安全位置和必要的人工辅助。上述研究没有执行任何模式设置或复位。
  本地 2026-10-06 固件读取记录为 S-V1.9-1，SDK 为 0.6.2；尚未找到该固件专用的
  免复位切换依据。Windows/candleLight 路径需适配当前 transport，不能直接照抄
  Linux can0 的 ConnectPort demo。后续初始化必须现场确认零位/支撑和复位影响，
  完成反馈读回后才进入既有 J1 commissioning。
  官方来源：[V2 接口](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD)、
  [reset 示例](https://github.com/agilexrobotics/piper_sdk/blob/master/piper_sdk/demo/V2/piper_ctrl_reset.py)、
  [厂商示教切换说明](https://bbs.agilex.ai/d/46)。

- **跳过相机后，独立的 J1 小步 commissioning 软件路径已实现并通过内存 CAN 回归。**
  `SUPERVISED_J1_COMMISSIONING` 与策略位置发送是两个互斥模式；新入口
  `laptop/robot_client/supervised_j1_commissioning.py` 只允许 J1 从当前实测值增加
  `0.100°`、J2–J6 保持当前值、不发夹爪帧。按 SDK 名义换算，最近只读反馈 J1=`88.151°`，
  该固定步的预期目标为 `88.251°`；因此撤销先前不必要的全六轴近零起始限制，无需先回零。快照必须由
  SafetyBridge 从自己实际接收的 CAN 反馈生成，并满足：关节在保守的官方 SDK 标称角度范围内、
  全部反馈不超过 120 ms、六轴反馈通道时差不超过 20 ms、300 ms 内各轴总漂移不超过
  `0.050°`、状态健康、没有监听到其它控制帧、`CAN_CTRL(0x1)` 且六个电机已使能。SDK
  标称范围是软件包络，不是机械硬限位；低速、急停可用和运动空间清空仍需现场确认。操作者还须
  在程序显示目标后输入一次性确认口令。程序不切控制模式、不使能电机、不回零、不控制夹爪；
  写帧结果不明时锁止且不重试。策略路径与旧 legacy HIL 拦截保持原样。
  更新后，commissioning、受监督 transport/bridge、legacy J1 拦截、runtime 配置、关节预览/恢复、
  SDK/夹爪/策略和离线 shadow 合并回归 **191 passed、46 个子测试通过**；改动 Python 文件通过
  `py_compile`。均为离线或内存适配器测试；本轮没有打开相机、连接真实 CAN、运行 live 入口或
  移动机械臂。USB 关闭不是机械臂停止；若发送后反馈异常，仍由现场操作员使用实体急停。
  最近一次真实反馈模式是
  `TEACHING_MODE(0x2)`，仍不满足 `CAN_CTRL(0x1)`，因此这不是实机验证，也没有验证策略映射。
  详情见
  [`J1 commissioning 内存回归`](../../validation/j1_commissioning_bridge_20261007.json)。

- **J1 实机前复核仍未放行。**2026-10-07 定向回归复跑为 **19 passed**。
  `pnputil` 只读枚举到一只 `1d50:606f` candleLight 主设备及其接口；按进程名筛查
  没看到明显的 PiPER/CAN 进程，但这两项都不能证明适配器当前独占。没有打开 CAN、
  没有发送帧，也没有运动。现场模式、电机使能、急停、低速、净空和适配器归属仍需
  操作者核实，并明确授权一次 J1 `+0.100°`。详情见
  [`J1 实机前复核`](../../validation/j1_preflight_20261007.json)。

### 摄像头暂缓后的执行顺序

本阶段先不处理相机。继续做与视觉无关的 PiPER 控制映射验证，顺序如下：

1. 确认 candleLight 适配器当前没有被原厂上位机或其他程序占用，再做短时只读
   状态检查。检查只接收反馈，不发 CAN 数据帧；若无法确认适配器归属，就先不打开它。
2. 单独完成示教到 `CAN_CTRL(0x1)` 的官方 SDK 初始化并读回模式和六轴使能。
   用户当前上位机没有模式入口；按上述官方研究准备 Windows/candleLight 适配，先
   核实零位/支撑及 reset 的失力影响，再取得对应操作授权。J1 commissioning 自身
   不切模式或使能。2026-10-07 本轮只读实测仍为 `TEACHING_MODE(0x2)`，六轴已使能。
3. 操作者确认实体急停、低速和运动空间后，单独运行 J1 `+0.100°` commissioning
   命令（[完整命令和步骤](../../laptop/robot_client/README.md#fixed-j1-commissioning-step)）。
   该步骤不需要相机；目标必须先在程序输出中核对，再输入一次性确认短语。
4. 只在反馈确认 J1 实际方向和变化量后，继续校准其余关节和夹爪的单位/方向；再把
   这些实测映射用于 π0.5 的位置目标适配。单个 J1 步进不代表策略控制已验证。

本阶段不启动策略实机 rollout。相机恢复后再完成实时 observation 和短时 shadow；这
两项不阻塞上述控制侧标定，但仍是 π0.5 视觉策略实机部署的必要输入。

策略入口范围复核：[`supervised_live_step.py`](../../archive/legacy_snapshot_20261009/laptop/client/supervised_live_step.py)
每次运行只调用一次 `client.infer`、发送一个 4 帧位置批次，再等发送后反馈并退出；
`duration_s` 是 worker 超时上限，不表示闭环重复控制。现增加显式
`--supervised-short-loop` 路径，最多 3 个批次、10 秒授权窗口；每批仍使用独立的
250 ms 提案令牌。下一批要求完整 Base/Wrist 新图像和五路 CAN 反馈时间均晚于上批最后
一帧写入，并有匹配写回执和健康状态。整段相对初始实测状态的关节/夹爪位移仍不得超出
原有单步限幅。`--supervised-position-step` 原入口保持一次推理/一次批次；默认不带模式
标志时不打开设备。
内存适配器端到端回归完成两种场景：连续新传感器数据时发送 3 个批次（12 帧）；发送
首批后停止刷新反馈时只接受首批 4 帧并停止，不进入第二步。覆盖 J1、短环授权、策略
位置发送和 live worker 的定向组合为 **38 passed**；Python 编译通过；无模式标志运行确认
状态为 `BLOCKED_EXPLICIT_SUPERVISED_STEP_FLAG_REQUIRED`、相机/CAN 均未打开、发送次数为 0。
这些是内存/离线检查，不是实机闭环结果。没有连接真实 CAN、打开真实相机、改变服务器
工作树或移动机械臂。
现场 J1 标定与关节/夹爪映射仍未完成；site profile schema 目前只表示单步现场就绪，
项目里也没有完整的实机 ready profile。下一步先在明确授权后完成 J1 `+0.100°`，再逐项
核验命令映射并通过受监督单步实测；之后才运行 3 步短环。
报告见 [`短环报告`](../../validation/supervised_short_loop_grant_20261007.json)。

本轮非相机核验：USB 只读枚举仍识别到唯一的 `1d50:606f` 适配器（bus 2/address 16）；
没有打开 CAN，故适配器当前是否被其他程序占用、机械臂当前模式和电机状态均未刷新。
合并覆盖 J1 commissioning、位置帧与超时行为、SDK/夹爪编码预览、受监督单步、只读
shadow、代理目标转换、readback 和采后合同的离线回归为 **228 passed、3 skipped、
46 个子测试通过**。新增跨层检查确认：历史全开反馈
`101800` 按名义 SDK 单位为 `101.8 mm`，超过配置的 `100 mm` 时会被命令提案层拒绝，
不会静默裁剪或放行；实物开口和单位仍未校准。没有重训、改动服务器工作树或移动机械臂。
记录见 [`非相机部署复核`](../../validation/non_camera_deployment_followup_20261007.json)。
另对现有成功 episode 做了逐行 100 ms 状态到目标增量审计：199 行中 111 行至少一轴
超过当前 `1°` 单步软件上限，13 行夹爪变化超过 `1 mm`，最大分别为 J5 `16.982°`
和夹爪 `24.2 mm`。所以这条旧 episode 仍适合验证转换/训练链路，但不适合直接按当前
单步限幅回放。新采 1 条和 5 条示教时，采后先跑同类增量审计；不可据此放宽限幅。
数据和单位边界见 [`状态目标增量审计`](../../validation/proxy_target_step_envelope_audit_20261007.json)。
为让后续 1 条与 5 条新示教自动得到同一检查，`next_state_proxy` 现在会把逐轴最大变化、
超当前 `1°/1 mm` 单步软件上限的行数及是否全量落在该包络写入 `proxy_manifest`；它不改
目标值，也不把结果升级为实机命令。独立转换、readback 和采后合同定向回归曾为
**36 passed、3 skipped**；合并回归结果见 [`非相机部署复核`](../../validation/non_camera_deployment_followup_20261007.json)。

- 本机重新校验结果标签后，仍是 1 条成功、1 条失败、7 条未标注。复用已有
  `cef166e2_preview_10fps`，当前 `next_state_proxy` 代码重新生成 199 条目标；
  每条目标间隔 100 ms，来源 manifest、preview、标签均通过哈希核验。输出在
  [`本地派生目标清单`](../../datasets/manifests/derived_state_pilot/piper_local_reuse_20261007_r1/proxy_manifest.json)，
  目标语义仍是下一时刻实测状态，名义单位，不能当作已验证的 PiPER 命令。
- **服务器离线 `prepare` 与全量回读已完成。**用户授权后，将 13 个依赖源码文件
  放入服务器项目内的隔离缓存目录
  `/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/prepare_overlay_20261007_r1/src`；
  上传归档 SHA-256 为
  `21e56bc223feb7f58ea415e100a9cd1a747d418f6981c9137ed8bcc1d26633d8`。没有覆盖服务器
  checkout，也没有上传数据、权重或凭据。必须从该 `src` 目录启动模块，确保同名
  Python 包优先加载隔离副本。
- 本次真实数据转换选中成功 episode `cef166e2cc6240b397db241a916ece3a`，得到
  1 episode、199 frames、10 fps。LeRobot 状态、动作、顺序和双路图像全量回读为
  `PASS_READBACK`；dataset SHA-256 为
  `f026b993e50abdbb573cc1d2ee93facfc070a8e14b1fe39f64969fee5ac05b36`，source-map
  SHA-256 为 `699cfd2fd7989f5d0d600ee76360e91a0bb59e179329024e13b4afea0d7ed111`。
  报告在服务器项目 `datasets/manifests/derived_state_pilot/piper_prepare_20261007_r1/`
  和缓存目录 `cache/codex/prepare_overlay_20261007_r1/prepare_report.json`。
  这只是下一时刻实测状态位置目标；`formal_training_eligible=false`、
  `hardware_control_allowed=false`、`can_opened=false`、CAN TX 调用为 0；没有训练。
- 转换器原先要求至少 2 条示教，与 tiny-fit 允许 1 条的入口冲突；现已修正为接受
  1 条有效示教、拒绝空列表。上述真实数据运行确认单条转换和全量回读路径可用；
  旧 step-9 checkpoint 未重训。
- **旧示教复用的 20-step tiny-fit 与离线评估已完成。**服务器隔离运行
  `post_capture_train_20261007_r4`，使用上述 1 episode/199-frame 数据训练 20 steps，
  新 checkpoint 位于其隔离 `runtime_root` 的 step `19`；checkpoint tree SHA-256 为
  `21750d663f54f23b1b25bd35b1677eba19b4ac8f00bb72bdd98551c36021e14f`，train-only
  normalization SHA-256 为 `d51ad915215b2735fbd12bb41169795bf0970bdab44d54df086eaed26847f35a`。
  同一 episode 的 5 行评估（`0,49,99,148,198`）得到名义关节 MAE
  `0.253734 rad`、夹爪绝对误差 `0.013263 m`；无 holdout、泛化或实机成功声明。完整报告：
  [`20-step tiny-fit 报告`](../../validation/post_capture_tinyfit_20261007.json)。旧 step-9
  checkpoint 没有重训。
- **历史观测 WebSocket 回环已通过。**将 shadow 服务和 runner 放在新的隔离缓存
  `cache/codex/derived_shadow_20261007_r1/`，服务只绑定 `127.0.0.1:8021`；核对
  dataset/checkpoint/training provenance identity 后，将同一 LeRobot 数据的第 `0,99,198`
  行各发送一次，均返回有限的 `16×7` 预测。三次推理延迟分别为 `6578.88 ms`、
  `87.72 ms`、`85.86 ms`（首帧含冷启动）。报告：
  [`历史帧回环报告`](../../validation/derived_state_shadow_loopback_20261007.json)。服务进程
  已停止、端口已关闭；CAN TX 为 `0`，未打开相机或 CAN，服务器 checkout 的 dirty 条目数
  运行前后均为 `58`。这是服务器同机的历史数据回环，不含笔记本网络、实时相机或机械臂；
  冷启动耗时远超首个 100 ms 目标间隔，后续 live runtime 必须丢弃过期目标，不能沿用
  这次热启动耗时当作端到端延迟。
- **为新 checkpoint 准备了本机身份锁并修复摄像头枚举回退。**从 20-step 训练报告导出
  [`派生状态 shadow identity lock`](../../validation/derived_state_shadow_identity_lock_20261007.json)，
  本机 `live_shadow.load_artifact_identity_lock()` 校验通过；该锁仍明确
  `hardware_control_allowed=false`。Windows WMI 摄像头查询实测因
  `HRESULT 0x80041003` 被拒绝，新增 `pnputil` 只读回退后枚举到 3 台 `Status=OK` 摄像头；
  角色绑定现对比大小写不敏感的稳定 Instance ID，仍要求操作员确认当前视角。定向回归
  **32 passed、20 subtests passed**。最新设备清点报告：
  [`camera / USB inventory`](../../validation/camera_identity_inventory_20261007.json)。本次只枚举
  PnP 与 USB；该清点没有打开相机流、启动 CAN 或发帧。由于 OpenCV 序号尚待确认，未启动
  新 checkpoint 的现场传感器 shadow。
- **本轮相机角色得到部分确认，但双路读取仍失败。**操作员确认 index `1` 是 Wrist、
  index `0` 预计是 Base。它与此前签名会话记录的 `0=ACER/Base、1=Dabai/Wrist` 一致，
  且这两个稳定 PnP ID 仍在当前清单中。最新单路扫描中，index `1` 在 `DSHOW` 下读到
  可见桌面画面；index `0` 在 `DSHOW/MSMF/ANY` 下均打不开；index `2` 同样打不开；
  index `3` 虽能读帧，但画面近黑（均值 `0.004`）。三个双路配对仍全部 `OPEN_FAILED`，
  因而还不能采集完整 observation 或启动 live shadow。画面接触表在忽略目录
  [`camera preview`](../../cache/codex/camera_role_preview_20261007_r1/contact_sheet.png)，
  元数据与历史设备绑定对照见 [`相机流访问复核`](../../validation/camera_stream_access_recheck_20261007.json)。
  没有保留临时视频、启动 CAN 或发送帧。
- 为排除 Base 设备仅仅换了较高 OpenCV 序号，本轮在隔离进程中只读扫描 DSHOW index
  `4–8`；五个序号均为 `OPEN_FAILED`，没有保存图像。结果见
  [`高序号相机扫描`](../../validation/camera_high_index_scan_20261007.json)。PnPUtil 仍报告
  ACER 相机为 `Started`、无设备问题；同时观察到 NVIDIA Broadcast 进程在运行，但没有
  停止进程或改动驱动。进一步只读检查 Broadcast 日志发现其最近记录的输入序号 `0` 是
  `ACER HD User Facing`，但当时预览不可见；虚拟相机过滤器日志也记录 Python 曾打开输出并
  拷贝 `1920×1080` 像素，未记录实际源相机名。证据整理见
  [`Broadcast 输入复核`](../../validation/nvidia_broadcast_camera_input_recheck_20261007.json)。
- Windows DirectShow 只读枚举得到顺序 `0=ACER HD User Facing`、`1=Dabai DC1`、
  `2=USB Camera`、`3=Camera (NVIDIA Broadcast)`；前三个 moniker 与 PnP Instance ID
  匹配。结果见[`DirectShow 相机枚举`](../../validation/camera_directshow_enumeration_20261007.json)。
  这排除了“Base 被挪到高序号”的猜测，但枚举不等于流可打开：此前 OpenCV index `0`
  仍失败。日志表明 Broadcast 配置到 ACER，可能在虚拟相机活跃时占用它，但当前占用状态和
  OpenCV 失败原因都未证实。下一步需在操作者允许暂时退出 Broadcast 后重试 ACER index `0`；
  若仍失败，再查系统相机权限和驱动。恢复 Base 后复测双路读取，再启动短时只读 shadow。
- 对 ACER index `0` 试了 DSHOW `MJPG 1920×1080`、`MJPG 640×480`、`YUY2 640×480`，
  以及 MSMF `1920×1080@30`、`640×480@30`；均 `OPEN_FAILED`，没有保存画面。结果分别见
  [`ACER 格式探测`](../../validation/camera_acer_format_probe_20261007.json) 和
  [`ACER MSMF 探测`](../../validation/camera_acer_msmf_probe_20261007.json)。Windows
  `Camera_FrameServer` 日志记录 Edge 在 16:10:56 曾启动并正常关闭 ACER 流（`hr=0`）；
  这表明当时有客户端成功初始化过该设备，不证明当前仍占用，也没定位到具体标签页。
- 为判断是否可暂用 Broadcast 输出替代 Base，做了 index `3` 虚拟相机 + index `1` 腕部相机的
  5 秒无录像并发测试。`DSHOW` 探针超时被终止，`MSMF` 打不开；`ANY` 虽读到双路帧，但
  index `3` 仅约 `0.97 fps` 且 5/5 帧近黑，index `1` 为 `27.79 fps`；到帧时差 P95
  `90.30 ms`、最大 `109.36 ms`。报告将 `camera_usable_for_observation` 记为 false；未开
  CAN、未存视频或发送机器人命令。结果见[`Broadcast + Wrist 并发探测`](../../validation/camera_broadcast_wrist_stream_20261007.json)。
- Broadcast 虚拟流探针退出、Broadcast 应用仍运行后，再测 ACER index `0` + Wrist index `1`：
  DSHOW/ANY 下 ACER 均打不开而 Wrist 可打开，MSMF 下两路均打不开。未存视频、未开 CAN。
  这表明仅释放虚拟输出过滤器不足以恢复 ACER；完整退出 Broadcast 后重测仍待操作者确认。
  结果见[`ACER + Wrist 复测`](../../validation/camera_acer_wrist_recheck_after_broadcast_probe_20261007.json)。
- 操作者随后确认 OpenCV index `1` 是 Wrist，index `0` 应为 Base。该确认用于标注相机角色；
  它没有证明 index `0` 当前能打开或验证其设备身份。ACER/Base 流仍待恢复，且暂时退出
  NVIDIA Broadcast 的请求尚未获批，因此没有关闭应用。
- 在不退出 Broadcast 的情况下，把 index `2`（PnP 名称 `USB Camera`）与 Wrist index `1`
  做了 2 秒无录像探测：DSHOW/ANY 均能打开，但 index `2` 仅约 `0.99 fps` 且采样画面
  判为近黑；index `1` 约 `25.78 fps`、有图像信号。MSMF 下两路均打不开。故 index `2`
  不能作为当前可用的 Base 替代源。报告 [`index 2 + Wrist 候选复测`](../../validation/camera_index2_wrist_candidate_20261007.json)
  记录 `video_saved=false`、`can_opened=false`、`robot_accessed=false`。
- 夹爪只读证据复核：操作员报告的闭合/全开反馈分别为原始值 `200`/`101800`，差值
  `101600`；SDK 0.6.2 的名义换算为 `0.2`/`101.8 mm`、跨度 `101.6 mm`。`0x477`
  getter 读到的配置行程为 `100 mm`。这组数据没有配对测量实际夹爪间距，不能证明物理单位、
  行程或命令映射；详情见[`夹爪端点对比`](../../validation/lab_gripper_endpoint_feedback_comparison_20261006.json)
  与[`夹爪参数读取`](../../validation/lab_gripper_parameter_read_20261006_170212.json)。
  代码中独立 `gripper_command_preview.py` 的恢复入口仅预览至 `70 mm`；策略单步预览则由
  site profile 限制至最多 `100 mm`，两者是不同入口的软件边界，不能作为硬件行程证据。
  本轮没有扩大任一限值或发送夹爪命令。首次使用系统 Anaconda 时缺少 `python-can` 和
  `piper_sdk`，两项 SDK 编码器测试未运行；改用 README 指定的 `pi05_laptop` 环境后，夹爪
  编码、官方 SDK 字节对照、策略预览、命令提案和 site profile 定向测试为
  **21 passed、11 个子测试通过**。此结果验证软件编码和拒绝逻辑，不验证实机单位或映射。
- 继续用 README 指定的 `pi05_laptop` 环境复核受监督控制和 shadow 软件路径：10 个定向
  测试文件共 **132 passed、23 个子测试通过**，覆盖 SDK 帧编码、位置提案范围检查、
  SafetyBridge、单步 preflight 和 prediction-only shadow。超时故障由内存适配器模拟：
  第二帧结果不明后 transport 锁止、关闭适配器且拒绝重试。全程为 fake/in-memory 软件测试，
  没有打开相机或真实 CAN；因此仍未验证真实 USB 超时行为、现场单位/限位或实机 shadow。
- 进程级停发再验：`test_process_exit_watchdog.py` 与 `test_bounded_process.py` 为
  **4 passed**。worker 被终止、异常退出或超过时限时，软件会锁存 stale-heartbeat/fault，
  不尝试发送；测试只用 Fake transport 和子进程，证明软件 watchdog 路径，不证明实机断联
  safe-hold 或物理 watchdog。
- 修正单步入口的发送后结果：此前四帧均被适配器接受、但 750 ms 内没有完整新反馈时，
  报告仍显示普通的适配器接受状态。现在区分“适配器接受但未观察到新反馈”与“已观察到新反馈”；
  前者明确要求停止、禁止直接重试并先读回反馈。两种状态都不宣称机械臂已运动或到达目标。
  单步、授权会话、位置提案/transport 和进程 watchdog 定向回归共 **34 passed**；
  全部为离线/fake 测试，没有打开相机或真实 CAN，也没有验证物理动作结果。详见
  [`发送后反馈状态回归`](../../validation/supervised_post_send_feedback_20261007.json)。
- 重审 P12 的目标来源项：现有成功 episode 的 proxy manifest、原始 manifest、对齐
  metadata、时间行和成功标签五项 SHA-256 全部匹配；199 条样本均为 10 fps，目标间隔
  均为 100 ms。由此，所选 `derived_future_observed_state_position_target` 路线的
  **数据来源语义已验证**，该哈希报告可满足 `command_target_source_verified` 的路线来源要求。
  这不是原厂命令帧，也不验证单位或策略到 PiPER 的物理映射。核验只读 JSON/JSONL，未读图像/视频。
  详见[`P12 派生目标来源审计`](../../validation/p12_proxy_target_source_audit_20261007.json)。
  下一步是不依赖相机的 J1 小幅标定：现有一次性入口仍被 legacy HIL guard 拦截；需先接入
  独立、严格限于单关节参考步的 SafetyBridge commissioning 路径并通过 Fake 测试，策略路径的
  完整 P12 门槛保持不变。现场测试仍需确认低速与急停可用，并对那次具体动作取得授权。
- 按用户指示跳过摄像头后，继续核对实机反馈总线。硬件 `LISTEN_ONLY` 接收 5 秒只看到
  `0x2A1`（30,999 帧），四组关节/夹爪反馈均为 0；改回项目既有的 `normal_receive_ack_only`
  模式（会 ACK 总线帧，但不发送 CAN 数据）后，同一 candleLight 收到五类反馈，计数为
  `0x2A1:1068`、`0x2A5:1002`、`0x2A6:1001`、`0x2A7:1002`、`0x2A8:1001`，解码错误和
  短 USB 包均为 0。每类约 200 Hz；最新通道到达时间相差最多 `3.836 ms`，单通道最大
  帧间隔不超过 `12.283 ms`。时间戳是主机读取到帧后的单调时钟，不是总线硬件时间或相机曝光时间。
  本次状态为 `TEACHING_MODE(0x2)`、机械臂 `NORMAL(0x0)`、错误码 0，夹爪反馈原始值 `5600`；
  `0x155`–`0x159` 命令 ID 均未出现。`can_data_tx_calls=0`、没有构造控制 SDK 或命令机械臂。
  汇总与时间审计见[`listen-only 反馈`](../../validation/piper_feedback_listen_only_recheck_20261007.json)、
  [`ACK-only 反馈`](../../validation/piper_feedback_ack_only_timestamped_20261007.json)和
  [`反馈时序审计`](../../validation/piper_feedback_timing_audit_20261007.json)；逐帧日志仅存于
  Git 忽略的 `datasets/raw/_readonly_shadow_ackonly_20261007_can/feedback.jsonl`。这验证了
  CAN 反馈读取和新鲜度，不等同于带图像的策略 shadow；当前不是 `CAN_CTRL(0x1)`，故本轮不做单步发送。
- 为确认新采数据后的最快离线验通入口，单独回归
  [`post_capture_pipeline` 一条成功示教测试](../../tests/server/test_post_capture_pipeline.py)：
  **2 passed、7 deselected**，覆盖一条哈希绑定的成功标签可以进入 `prepare_dataset`，以及
  一条成功 episode 可传入 tiny-fit 数据转换器。实际 `capture_session` 仍要求 Base/Wrist
  双路可用、明确的 `--operator-demo-capture-approved` 和操作员逐条判定成功；本轮没有启动
  采集或训练。当前 Base 流不可用，故新一条示教仍不能开始。
- 受监督位置适配器当前代码的定向离线回归为 **24 passed、3 个子测试通过**，覆盖位置
  四帧编码、SafetyBridge 单次授权、策略单步转换及提案校验；测试使用内存适配器，未打开
  真实 CAN。另做 candleLight USB 身份枚举，当前找到一只 `1d50:606f`（bus `2`、address
  `16`）；只枚举 USB，未启动 CAN、设 bitrate 或发送数据。以上验证了适配器代码和 USB
  可见性，不等于实机写帧或运动验收。
- 随后按用户要求做了两次最多 5 秒的 ACK-only 接收；适配器以 1 Mbps 正常接收并 ACK
  总线帧，没有发送 CAN 数据帧。一次收到 `11,670` 帧、错误帧 `0`；全 ID 直方图未见
  `0x155`、`0x156`、`0x157` 或 `0x159`，报告见
  [`ACK-only 总线复核`](../../validation/can_ack_only_recheck_20261007.json)。官方 `piper_sdk 0.6.2`
  V2 解析器对状态、六轴和夹爪五类反馈均解码成功，解码错误和短 USB 包均为 `0`。最新
  状态为 `TEACHING_MODE(0x2)`、`STOP_RECORDING(0x2)`、机械臂 `NORMAL(0x0)`、错误码 `0`；
  CAN 数据 TX、SDK 接口构造及运动均为 `0`。解码报告见
  [`PiPER ACK-only 反馈解码`](../../validation/piper_feedback_ack_only_recheck_20261007.json)。反馈
  原始量纲与夹爪映射仍未验证；该状态不是 live camera shadow，也不满足单步控制所需的
  `CAN_CTRL(0x1)`。
- 首条示教的单 episode PowerShell 命令模板已加入关键路径第 3 项；本机实际运行
  不含 `--operator-demo-capture-approved` 的 CLI 预检，返回
  `BLOCKED_APPROVAL_REQUIRED`，并明确 `camera_started=false`、`can_started=false`、
  `can_data_tx_calls=0`。这只验证默认入口会停在设备访问之前，没有开始采集。
- 新增 [`派生目标到命令提案的显式转换`](../../archive/legacy_snapshot_20261009/laptop/robot_client/verified_policy_command.py)：
  只有全部 `P12GateEvidence` 项通过、当前状态与预测仍新鲜、且调用方提供
 `SafetyLimits` 后才创建绝对位置 `CommandProposal`；还会再次检查六轴和夹爪的位置范围
 与单步增量。它不发 CAN。证据字段是现场人员声明，本函数不核验现场记录；共享
 transport 仍拒绝通用运动帧。
  预览、Fake rehearsal、单条转换门槛及采后入口测试为 **29 项通过、2 项跳过、9 个
  子测试通过**；跳过项是需要本机 LeRobot 的转换/回读集成测试。
- 本轮补上“预览限幅”与调用方位置范围之间的二次核对；命令提案、预览及 SafetyBridge
  回归测试 **86 项通过、5 个子测试通过**。这只验证软件拒绝/提案逻辑，不是现场单位、
  限位或 CAN 发送验证。
- 新增低层 `send_supervised_position()`：只接受与位置提案字节完全匹配的
  `0x155/0x156/0x157/0x159` 四帧、同一反馈序号、最新状态、显式 `SafetyLimits`、
  完整 P12 声明和最多 250 ms 的一次性操作员批准；成功回执记录实际写入帧和本机
  单调时间。任一帧写入失败或结果不明会关闭适配器并锁住该 transport，不会自动重试；
  通用 `send()` 仍拒绝。P12 字段仍是调用方声明，不能代替现场验收；本轮随后将
  该原语接入专用 SafetyBridge 单步入口，但连续策略运行循环仍未接通。本次仅
  Fake/内存适配器测试，未连接 CAN。
  整个 `tests/laptop` 离线套件为 **319 passed, 72 subtests passed**；初次运行中 2 项
  因系统 pytest 临时目录权限无法初始化，改用仓库内隔离 basetemp 后全量通过。
- 专用运行模式现为 `SUPERVISED_POLICY_STEP`，配置项
  `supervised_position_tx_enabled` 默认关闭；它要求专用真实 transport，并保持
  通用 `can_tx_enabled`、`command_tx_enabled` 为关闭。`SafetyBridge` 在发送前检查
  新鲜反馈、命令提案和一次性授权；发送结果不明时锁定 bridge。Fake 内存适配器
  验证了成功、拒绝和写入不明后的锁止行为。
- 新增 `supervised_policy_step.py`，将 16×7 预测的首个未来有效行依次转换为 SDK
  预览、经过 P12 与位置限幅检查的命令提案、以及严格匹配提案的四帧位置批次；
  发送是另一个显式函数，要求操作员一次性授权和专用 SafetyBridge。预测响应超过
  250 ms、请求序号与状态不匹配、状态过期或 P12 不完整时拒绝。该函数未接入默认
  `POLICY_REJECT_ONLY` shadow 循环，也未连接真实 CAN。新增集成测试以内存适配器
  覆盖完整单步；`tests/laptop` 与 `tests/contracts` 回归为
  **373 passed, 76 subtests passed**。
- 用户授权后，已将共 22 个文件（适配器源码、对应测试、Python 包入口和计划快照）
  上传到服务器项目内的隔离目录
  `/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/supervised_adapter_overlay_20261007_r1/src`；
  服务器端逐文件 SHA-256 校验通过。该目录不覆盖服务器 checkout；本次未在服务器
  运行测试、启动训练或打开 CAN。
- 本轮单步策略接线后，已将共 32 个最新版源码、导入依赖、定向测试和计划快照
  上传到新的隔离目录
  `/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/supervised_adapter_overlay_20261007_r2/src`；
  逐文件 SHA-256 校验通过。没有覆盖服务器 checkout，也没有在服务器运行测试或
  连接 CAN。
- `live_shadow` 现在用最旧 CAN 通道时间标记反馈快照的采集时刻、用最新通道时间标记
  到达时刻；单位验证仍默认为 false。主流程复用该快照进行 prediction-only 判定，
  不会把观察构造时间误当作硬件反馈时间。新增时间戳测试后完整回归为
  **373 passed, 76 subtests passed**。用户授权后，最新版源码、导入依赖、测试和
  计划快照共 40 个文件已上传到服务器独立 r3 overlay，逐文件 SHA-256 校验通过；
  没有覆盖服务器 checkout。服务器 `.venv` 在该隔离副本上运行了单步、SafetyBridge
  和 RuntimeConfig Fake/内存测试：**14 passed**。测试未打开 CAN、启动训练或发送命令。
- 检查批准时序后发现：首个策略目标通常在观测后 100 ms 生效，命令提案到该时刻
  就失效；要求操作者看完目标再批准会来不及。新增
  `supervised_policy_session.py`：先展示并确认任务、checkpoint/config/normalization/
  dataset 身份及完整安全限幅，只授予一批且 10 秒后失效；推理后再检查身份、当前
  状态、P12 和同一限幅，并即时生成原有最多 250 ms 的精确提案授权。单批 Fake
  端到端与拒绝路径测试已通过。**此段记录的是当时仅有单批 grant 的状态**；当前
  live-worker 短环接线与验证见文首 2026-10-07 执行进展。它不能替代现场 P12 验收，
  也未授权实机运动。
- 新授权路径定向测试为 **15 passed**。当前本机 Python 环境中，`tests/laptop` 有
  **327 passed、5 项因缺少 `python-can` 未能运行**；除依赖 `openpi_client` 的
  `tests/contracts/test_piper_contract.py` 外，contracts 为 **41 passed**。完整回归
  尚未完成依赖环境核对；本轮没有连接 CAN、训练或运动。
- 新增 [`supervised_site_profile.py`](../../laptop/robot_client/supervised_site_profile.py)
  与空白 commissioning 模板。它要求现场限位、13 项 P12 声明和每项证据引用齐全，
  并拒绝超出官方 SDK 名义关节范围的配置；不会验证引用内容真假、开启 TX 或更改
  shadow 默认。配置还要求明确的夹爪力度；夹爪命令上限为当前项目 100 mm，首次单步
  限幅最多每轴 1°、夹爪 1 mm。它们是软件上限，不是机械行程实测值。现有
  `piper_joint_limits.yaml` 的 SDK/控制器读数不能直接充当现场配置。
- 新增 [`supervised_live_step.py`](../../archive/legacy_snapshot_20261009/laptop/client/supervised_live_step.py)，将 site
  profile、已锁定模型、相机身份、CAN 状态、推理前授权和现有单步发送接成一个独立
  worker；授权通过可 pickle 的一次性 ticket 转入 spawn 子进程。控制模式不是
  `CAN_CTRL(0x1)`、机械臂状态异常、profile 不完整、目标越界、授权过期时均在发送前拒绝。
  进程超时或写入结果不明会报告“交付未知、禁止重试”，而不会自动重发。
  现场配置解析、力度绑定、授权 ticket、单步转换和 runner 拒绝路径定向测试为
  **34 passed**；没有连接 CAN、训练或移动机械臂。新增入口尚未在真实设备上运行；site
  profile 未填、物理方向/尺度/速度和 safe-hold 仍待现场验收。
  详情见 [`supervised live-step validation`](../../validation/supervised_live_step_20261007.json)。
- 用户授权后，已从 r3 建立独立 r4 源码快照，含 42 个源文件和校验清单：
  `/mnt/ssd/users/mingkai/07_pi0.5/cache/codex/supervised_adapter_overlay_20261007_r4/`。
  `SHA256SUMS` 对 42 个服务器源文件逐项校验通过，且与本地逐文件哈希一致。服务器
  `.venv` 对授权、单步发送、transport
  和 runtime config 定向回归为 **35 passed, 1 deselected**；未选中项需要缺少的
  `python-can`。服务器 checkout 状态与上传前相同；未运行训练或打开 CAN。
- 本轮使用已有 `pi05_laptop` 环境复核控制与采集边界：适配器、SafetyBridge、策略单步、
  gripper 编码预览、命令转换、SDK 预览和采集预检共 **37 passed、11 subtests passed**。
  Fake/内存测试覆盖精确四帧、一次性授权、反馈新鲜度和限幅、USB 写入不明后关闭并禁止重试；
  相机身份、采集预检和 `live_shadow` 客户端另有 **32 passed、20 subtests passed**。
  新 checkpoint 的 identity lock 本轮重新加载校验为 `PASS`，并保持
  `hardware_control_allowed=false`。
  本轮没有打开相机、启动 CAN、发送帧、训练或移动机械臂。报告：
  [`受监督适配器离线复核`](../../validation/supervised_adapter_offline_recheck_20261007.json)。
  这仍不验证真实 USB-CAN 发送、机械臂单位/方向/尺度、夹爪零点/映射或现场 watchdog。

### 最短关键路径与验收

以下工时是在机械臂、相机和一张服务器 GPU 可用时的工程估计，不是成功率
或完成日期承诺。第 2、3 项与第 1 项并行推进；不要等待录满 50 条才开发控制适配器。

1. **已完成（2026-10-07）：复用旧数据，验证离线入口。**将本地新增模块放入
   服务器项目缓存下的隔离目录，保留服务器原工作树及固定 OpenPI/LeRobot
   版本；对旧成功 episode 与结果 sidecar 运行 `prepare`，核对 10 fps 图像、
   状态、目标、来源哈希和全量回读。验收通过：成功 episode 被选择，199 帧
   可完整回读，服务器原 checkout 未被覆盖。已有 step-9 checkpoint 未重训。
   这一步不代表新模型训练或任务能力验证。
2. **进行中：受监督 PiPER 位置适配器。**低层 transport、专用
   `SUPERVISED_POLICY_STEP` 配置门和 SafetyBridge 单批次发送入口均已实现；Fake
   测试覆盖成功发送、授权/模式拒绝和写入不明后的锁止；新的策略单步函数已接通
    typed prediction→`CommandProposal`→`PositionCommandBatch`→SafetyBridge。新增的
    `supervised_policy_session.py` 与独立 `supervised_live_step.py` 已接通；运行入口要求
    显式开关、完整 profile、相机身份、模型身份锁和逐次口令确认，只执行一批。Fake 路径
    已测，真实入口没有运行。下一步是在现场核对 status=CAN_CTRL、逐轴小幅参考目标、
    夹爪力度/方向、急停和断联行为，并把真实证据写入 profile；在这些条件完成前不得
    调用该发送入口。默认 shadow 仍拒绝命令。
    P12 仍有现场未验证项；
   不得把本地布尔声明
   视作现场验收。完成软件接线后再现场
   逐轴确认六轴和夹爪方向、单位、限位、低速、急停、watchdog 及断联保持。验收仍是：
   失败输入不发运动帧；USB 结果不明时停止且不重发；夹爪负数反馈不静默变成负行程命令。
3. **第 0–1 天：采集新示教（现场先 1 条，再累计 5 条成功）。**使用
   [`capture_session.py`](../../laptop/recorder/capture_session.py) 的现有
   45 秒/5 秒倒计时入口，保持 Base/Wrist 相机角色、黑棒、绿垫和任务文本
   一致；逐条标成功/失败。先确认拖动示教时采集器能独占当前 CAN
   适配器。若原厂上位机必须同时占用唯一适配器，先解决设备占用（例如
   独立监听器），不要让两个程序共用同一 USB-CAN。验收：至少 1 条新
   成功 episode 的双视频、六轴、夹爪、时间戳和结果标签完整；5 条成功
   是多示教 pilot 目标，不是首次 tiny-fit 的硬门槛。相机角色目视确认后，
   首条从仓库根目录执行以下命令；将占位值换成已确认的序号和设备 ID。操作员
   按提示启动录制、完成示教并单独标注结果。

   ```powershell
   & 'E:\anaconda3\envs\pi05_laptop\python.exe' -m laptop.recorder.capture_session `
     --episodes 1 --duration 45 --lead-in 5 `
     --base-index <Base序号> --wrist-index <Wrist序号> `
     --base-device-id '<Base设备ID>' --wrist-device-id '<Wrist设备ID>' `
     --roles-visually-confirmed --operator-demo-capture-approved
   ```
4. **进行中：新示教 tiny-fit 与只读 shadow（旧示教复用基线已完成）。**
   旧 episode 已完成 20-step tiny-fit 和三行历史帧回环；这只验证同一条旧示教上的
   离线训练及服务链路。采到至少 1 条新的成功示教后，再在隔离服务器目录运行
   [`post_capture_pipeline.py`](../../server/training/post_capture_pipeline.py) 的
   `prepare → train`，默认 20 steps；核对 train-only 统计量、
   checkpoint、数据哈希和逐帧回读，再启动
   [`serve_derived_state_multi_shadow.py`](../../server/serve_derived_state_multi_shadow.py)
   与笔记本 [`live_shadow.py`](../../laptop/client/live_shadow.py)。验收：
   真实观测得到有限的 16×7 预测，身份一致，CAN TX 为 0。20–100 steps
   的结果仍只叫链路验通，不宣称任务训练完成。
5. **第 1–2 天：受监督实机映射与短闭环（仅在现场、按具体动作授权）。**
   先以人为指定的小幅参考目标逐轴确认 J1–J6 的方向和尺度，夹爪另行
   确认开合、零点及命令范围；检查现场急停、低速、单步限幅、反馈跟踪和
   断联后停止发送。原厂上位机与策略控制不能同时占用同一适配器。
   参考目标通过后，才测试一次限幅的模型目标，再做 3 次、每次 10–20 秒
   的短闭环。历史 shadow 往返延迟约 124–168 ms，超过首个 100 ms
   目标间隔，运行时必须跳过过期目标并记录实际发送/反馈时间。
   验收：无越界或失联后继续发送，六轴及夹爪实测方向正确，保存目标、
   实测轨迹、中止原因和视频。此项验收代表工程部署跑通，不代表任务成功。
6. **短闭环后：提高任务成功率。**按 [LeRobot 真机数据建议](https://huggingface.co/docs/lerobot/il_robots)
   扩到约 50 条训练示教、5 个起点各约 10 条，并另留实机测试起点。
   新建正式微调配置，解除 pilot 专用的 100-step 上限，按阶段保存
   checkpoint 并检查独立 episode；不要盲目照搬文章的 40k steps。
   在至少 5 次未参与训练的实机任务尝试中报告成功次数和失败类型，
   再决定是否继续采集或训练。

本路线保留最小运行保护（位置/单步限制、最新反馈、超时停发、现场急停），
但不再要求先获得原厂 `0x155–0x159` 命令帧才能进行**派生位置目标**的
离线训练或设计受监督适配器。历史章节中“先补原厂命令目标才能推进”的
表述只适用于原厂命令监督；历史 `P12/P13`、`hardware_control_allowed=false`
和旧验证报告保留其当时的证据含义。实际机器人动作仍需现场对具体动作
明确授权，本节本身不授权 CAN TX。

## 2026-10-07 离线采后训练与 shadow 接线

### 目标与 SO-101 流程对照

按 LeRobot 的 SO-101 路线保留“示教/记录 → 训练 → 评估 → 部署”的顺序；
本项目沿用固定版本 OpenPI、LeRobot 数据格式和 PiPER 自己的状态映射，不安装
要求更高 LeRobot 版本的 PiPER 插件，也不把 SO-101 电机驱动当作 PiPER 驱动。
[SO-101 官方教程](https://github.com/huggingface/lerobot/blob/main/docs/source/so101.mdx)、
[LeRobot 部署流程](https://github.com/huggingface/lerobot/blob/main/docs/source/cheat-sheet.mdx)。

### 本轮离线实现

新增 [`post_capture_pipeline.py`](../../server/training/post_capture_pipeline.py)，
在服务器收到至少 1 条原始 episode 和结果 sidecar 后，可用一条 `run` 命令完成成功标签筛选、
10 fps 对齐、来源核验、未来实测状态目标、LeRobot 转换/全量回读、train-only
统计、单 GPU 20-step 默认 tiny-fit、同训练集 prediction 评估，并生成 loopback
服务命令及新格式的笔记本 identity lock。失败 episode 继续保留并排除出成功训练集；
所有 action 明确是 `derived_future_observed_state_position_target`，不是 PiPER 原厂
命令。采样器现在保留有符号夹爪反馈；单位仍是 nominal、未完成实机校准。

新多 episode 服务现提供 16×7 / 0.1 秒时序和数据、checkpoint、normalization、
training provenance、OpenPI commit 身份；笔记本 [`live_shadow.py`](../../laptop/client/live_shadow.py)
能验证该身份并将预测记入 `POLICY_REJECT_ONLY`。这只接通 read-only shadow，
不接 CAN TX。由于真示教/新 checkpoint 尚不存在，本轮没有执行实际模型训练、checkpoint
加载、WebSocket 服务或实机 shadow。

定向测试：`48 passed, 2 skipped, 20 subtests passed`；未连接相机/CAN，未运行训练。
现已把新链路的默认门槛改为 1 条经校验的成功示教，和项目既有 local-tiny-fit
一致；5 条及以上是更好的多示教试验规模，不是第一次跑通训练/预测链路的硬门槛。
上传这 1 条 episode 及 outcome label 后即可启动离线 tiny-fit。注意：本轮只读检查发现
GPU checkout 缺少这些新入口，且 converter 与本机 API 不同；远端有未提交/未跟踪文件，
因此没有覆盖或同步。下次采集前需先做一次保留远端现有改动的代码同步；这是服务器版本
同步事项，不是新增的实机安全门槛。
真实 PiPER 控制仍不能由数据集单独解锁，因为 `GsUsbPiperCanTransport.send()` 仍拒绝
运动帧，P12 受监督单步仍需一次简短现场确认和明确动作授权。

本轮补强：1 条成功 episode 的选择、失败 episode 排除、checkpoint 身份校验和
prediction-only shadow 已有定向测试；真实训练和真实 shadow 没有运行。证据见
[`2026-10-07 离线就绪报告`](../../validation/offline_readiness_20261007.json)。

## 2026-10-07 多示教训练后的 prediction-only 部署准备

### 开源路线核对

LeRobot 官方仍按 Teleoperate → Record → Train → Deploy 组织数据与部署；PiPER
社区仓库 `AgRoboticsResearch/lerobot_robot_piper` 展示了 PiPER 的 LeRobot 适配、
关节方向符号映射、弧度选项、夹爪开关和 `lerobot-record` 示例。它可用于对照
接口设计，但不能证明本机 PiPER 的方向/单位已校准。该插件 README 要求 LeRobot
≥0.4.0，而服务器现有兼容环境记录为 `lerobot==0.1.0`；因此本项目不安装该插件、
不切换 LeRobot/OpenPI 栈，只借鉴“明确关节映射并按 episode 记录”的做法。
[PiPER 社区插件](https://github.com/AgRoboticsResearch/lerobot_robot_piper)、
[LeRobot 记录/训练/部署流程](https://github.com/huggingface/lerobot/blob/main/docs/source/index.mdx)、
[OpenPI 自定义 LeRobot 微调配置](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/config.py)。

### 部署缺口及本轮修复

只读复核确认服务器现有 10 fps 数据集、旧 step-9 tiny-fit checkpoint 和 `.venv`
仍存在；服务器主 checkout 有用户未提交修改，故未覆盖或同步代码。旧
`server/serve_proxy_piper_shadow.py` 固定引用单集 2/10 fps 登记配置，不能直接加载
`proxy_multi_episode_pilot.py` 生成的新多 episode checkpoint。现新增独立入口
[`serve_derived_state_multi_shadow.py`](../../server/serve_derived_state_multi_shadow.py)：
启动前重新核验训练 provenance 中记录的成功 episode 数、LeRobot 数据集哈希、训练来源、checkpoint
tree 哈希、归一化统计量和 pinned OpenPI commit；服务仅绑定 loopback，metadata 明确
`prediction_only=true`、`hardware_control_allowed=false`、CAN 未打开且 TX=0。该入口
尚未在服务器项目目录部署，也未加载真实新 checkpoint。

本轮本机 Conda 相关测试为 `9 passed`，覆盖 loopback/采样步数边界、数据/权重/统计量/
OpenPI 身份不匹配拒绝，以及 Fake server 只收到 prediction-only 元数据。服务器主
checkout仍保持未修改；服务器 GPU 快照为 GPU 0/2 空闲、GPU 1/3 已占用，旧 8003
服务进程存在，训练没有运行。该快照不保证次日资源状态。
同一 Fake-only 测试也在服务器 `.venv` 的 `/tmp/pi05-multishadow-20261007`
隔离副本通过（`6 passed`）；临时副本不在项目 checkout 内，未读取真实新 checkpoint，
未启动 WebSocket 服务或训练。

### 当前状态与明早交付边界

当前仍无本轮新采集的真实成功示教，所以不能现在生成对应 LeRobot 数据、统计量、
新 checkpoint 或模型预测结果。已完成的软件准备是：一次 5×45 秒反馈-only 采集会话、
逐条成功/失败 sidecar、既有 10 fps 对齐/派生目标/LeRobot 转换和多 episode 训练预检，
以及新的多 episode prediction-only 服务入口。此前 5 条合成 episode 的转换、视频/逐帧
回读与预检通过服务器端 `13 passed`；它不是机器人数据或训练证据。

拿到真实示教后的最快安全链路是：只对成功标注的 episode 做 10 fps 对齐和未来实测状态
派生目标 → pinned LeRobot 转换及全量回读/hash 核验 → train-only 统计 → 单 GPU、最多
100 step 的隔离 tiny-fit → 同训练 episode prediction-only 评估 → loopback shadow 服务
并回放保存的观测。失败 episode 保留作失败分析，不混入成功训练集。所有 action 仍是
`derived_future_observed_state_position_target`，不是原厂 CAN 命令；shadow 服务不能
驱动机械臂。真实实机动作仍需 P12 所列单位、限位、低速、急停、watchdog、断联保持全部
通过，并对每个具体动作另行同意。

13 项矩阵仍为 8 项完成、3 项部分完成、2 项未开始；第 10 项旧单集实验完成，采后
tiny-fit 路线已接通、待一条新真实成功示教执行，第 11 项旧 2 fps live shadow 完成但
新 checkpoint shadow 尚未运行；
第 12、13 项仍锁定。详细机器可读证据追加在
[`derived-state reuse audit`](../../validation/derived_state_tiny_fit_reuse_20261006.json)。

## 2026-10-06 开源路线核对与 derived-state tiny-fit 复用

### 开源项目判断

优先沿用项目已经固定的官方 OpenPI 子模块，不切换模型栈：OpenPI 官方把
小型自定义示教转为 LeRobot 后微调；完整 DROID 规模则单独使用 RLDS。LeRobot
的 Pi0.5 入口本身接收 `observation.state` 与 `action`，但字段形状并不能证明
`action` 是真实机器人命令。[OpenPI DROID/自定义数据流程](https://github.com/Physical-Intelligence/openpi/blob/main/examples/droid/README_train.md)、
[LeRobot Pi0.5 数据约定](https://github.com/huggingface/lerobot/blob/main/docs/source/pi05.mdx)。
OpenVLA 也支持自定义数据，但需单独接入其 RLDS/PyTorch 数据转换和模型栈；它不会
补出这条示教缺失的 CAN 命令目标，因此当前切换没有收益。[OpenVLA 自定义数据与微调](https://github.com/openvla/openvla)。

### 示教、数据和 checkpoint 核验

Episode `cef166e2cc6240b397db241a916ece3a` 可以继续复用作**离线派生状态目标**，
不能升级为真实命令监督。其成功标签是任务结果；原始 CAN 日志共 20,013 行，只有
`0x2A1/0x2A5/0x2A6/0x2A7/0x2A8` 反馈 ID，不含 `0x155–0x157` 或 `0x159`；
`metadata.action_target_present=false`。manifest SHA-256 为
`794111b8c523364321ae181f2970b9f7768b0fb81a489b59f05da5daea7e98c6`，反馈日志 SHA-256 为
`c20e7eceb262b0a227a22dcca024d2cdb59a5a4f076d2d7d1cedfa0c0ce814bd`。

准确动作语义定为 **derived/future-observed-state position target**：时刻 `t` 的双相机
与 7D 状态是 observation，`action` 是同一轨迹 `t+Δ` 时实际读到的 6 关节＋夹爪反馈。
LeRobot 里历史语义 token 仍保留为 `next_observed_state_proxy_nominal_units`，不改写
已校验数据；新审计与独立 runtime config 另标
`derived_future_observed_state_position_target_nominal_units`。关节 radian 与夹爪 meter
都只是名义单位、尚未实机确认。手在部分操作画面中扶住抓夹，也限制了对示教动作因果性的解释。

本机 LeRobot 数据树和服务器副本一致：10 fps、199 transitions、1 个 train-only episode，
dataset SHA-256 `9e0912029ec9d2f4c3e256ad1911416a75bc0bb461e0edb5f992e10585e7b8df`，
readback/source audit SHA-256 `7e5f7cf73feb5be7d7e6966598e0b59b69ef68d416e9c5bf62d07150cbcc6b0d`；
双视频帧提取、199 行 observation/target 对应和来源帧回读均通过。2 fps 的 39-transition
来源审计副本仍保留，其 dataset/audit hashes 分别为
`9bb32bca43661e72a0878db9234a28ca13bb7a8f8eacb83437d18be63359d666` 与
`ac73d19d4164a4eb1766d41c707f3baf0c1c1cf4cf86b30703d8ed451e2d17bb`。

10 fps checkpoint `.../cef166e2_10fps_ten_step/9`（10 updates，step 9）当前 tree SHA-256
为 `852c3393ca6e8804d32ea79f743394cc0be522c08c5a05c7f82cd232351cf7d6`；train-only 统计量
SHA-256 为 `d51ad915215b2735fbd12bb41169795bf0970bdab44d54df086eaed26847f35a`；OpenPI
子模块仍是 pinned commit `215abfb217dbac7d5f1273282331b9b1866c0479`。这次重新对当前
checkpoint 做了 5 帧离线评估：关节 MAE `0.29106 rad`、夹爪绝对误差 `0.01303 m`
（名义值）。这与历史记录的 `0.29108 rad / 0.01301 m` 一致到报告精度，但只是同一条训练
episode 上的预测 sanity check，不是泛化、控制精度或任务成功证据；详细身份记录见
[`derived-state reuse audit`](../../validation/derived_state_tiny_fit_reuse_20261006.json)。

### 接线与边界

现有 `proxy_piper_tiny_fit` 和 LeRobot/OpenPI transform 已能读这类 7D 数据，且与正式
`piper_v1` raw-command acceptance 路径分离，无需重转数据或重训。本轮补充了精确
target-kind/time-relation 元数据、拒绝把 derived state 当作 raw command 的纯函数门禁，
以及单独 hash-pinned 的
[`10 fps derived-state shadow config`](../../configs/policy/pi05_derived_state_10fps_shadow.json)。
该配置同时固定数据树、readback audit、train-only 统计量、当前 checkpoint 和 OpenPI
commit；训练、评估、shadow 配置都保持 `hardware_control_allowed=false`。本地训练/评估代码哈希与
历史 10 fps 报告吻合；只读检查发现服务器当前 `proxy_piper_tiny_fit.py` 与 evaluator 是
较旧版本，且与当时报告代码哈希不同。因此这轮没有声称当前服务器已部署新 shadow 接线；
新 config 尚未在服务器启动或经 WebSocket 回放，现存 2 fps live shadow 不改动。

第 12 项受监督实机单步仍未开始：速度单位、实际限速、机械硬限位、低速、急停、
物理 watchdog 和断联 safe-hold 均未通过。缺少原厂命令帧只阻断“真实命令监督”路线，
不阻断明确标注的未来实测状态离线 proxy 路线；proxy 模型仍不得直接当作硬件命令。
第 13 项依赖第 12 项。CAN TX 仍关闭；本轮没有打开相机/CAN、启动训练或发送机器人命令。证据见
[`P12 阶段报告`](../../validation/p12_offline_preflight_timing_20261006.json)。

## 2026-10-06 离线多示教路线续推

用户指出的服务器端资源已直接核对：既有 10 fps LeRobot 数据、step-9 π0.5
tiny-fit checkpoint 和 pinned OpenPI/LeRobot 环境都在服务器；旧数据身份与同轨迹
五帧预测评估已经有报告，**本轮没有重训或重复评估**。只读 SSH 看到项目预测服务进程
`server/serve_proxy_piper_shadow.py --port 8003` 仍在；GPU 0/2 当时空闲、GPU 1/3
正在占用。本轮没有检查 WebSocket 响应，也没有停止/重启服务。服务器 checkout
有大量既有未提交改动，因此没有把本地新代码覆盖到其工作树。

为下一批离线实验新增了 feedback-only `operator_demonstration` 会话
([`capture_session.py`](../../laptop/recorder/capture_session.py))：每条最长 45 秒，
开录前 5 秒提示，逐条记录独立成功/失败结果；失败条目保留但不会进入成功训练集。该路线
以相邻 10 fps 时刻的实测状态构造 `derived_future_observed_state_position_target`，
不是原厂 CAN 命令。新增多 episode LeRobot 来源/回读合同与隔离 tiny-fit/eval 入口：至少
5 条不同且成功标注的示教，训练/统计仍为 train-only，单 GPU、最多 100 steps，模型保持
prediction-only、`hardware_control_allowed=false`，不提供 CAN TX 路径；对应入口为
[`multi-episode converter`](../../server/dataset_tools/convert_next_state_proxy.py)、
[`offline pilot`](../../server/training/proxy_multi_episode_pilot.py) 与
[`data contract`](../../server/training/proxy_multi_episode_contract.py)。单位仍是名义值，
未完成实机校准。

本地专用 Conda 的相关测试为 `28 passed, 2 skipped`（本地没有 pinned LeRobot）。随后将
源码放进服务器 `/tmp` 隔离目录，在现有 `lerobot==0.1.0`、`datasets==3.6.0` 环境用五条
合成 episode 实际完成 LeRobot 转换、每帧/双视频解码回读和训练预检：`13 passed`。该
集成修复了 pending 输出目录名被误写入 readback 审计的问题。服务器项目 checkout 未改，
新 converter/pilot 尚未部署到项目目录；没有 5 条新真实示教，因此本轮没有真实数据转换、
统计或训练。测试证明软件集成路径能跑通，不证明真实示教质量或模型泛化。

**当前唯一需要的现场操作**：在相机设备仍对应上次列举的身份，且你已目视确认 OpenCV
index 0=笔记本 Base、index 1=Dabai Wrist 后，运行一次下方采集命令。脚本每条 episode
都等你按 Enter，再以 5 秒倒计时提示 `CAPTURE_START`；随后由你手动示教黑棒放绿垫，并
立即标记成功或失败。它只开双相机和 CAN 反馈接收，不进入示教、不发送 CAN 数据帧，也不
让模型控制机械臂。数据收齐后再部署新代码到服务器、用 pinned LeRobot 转换/回读、计算
统计、做小样本训练与 prediction-only 评估。

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -m laptop.recorder.capture_session `
  --base-index 0 --wrist-index 1 `
  --base-device-id 'USB\VID_0408&PID_4035&MI_00\6&239FB1A8&0&0000' `
  --wrist-device-id 'USB\VID_2BC5&PID_0557&MI_00\7&3164C1CD&0&0000' `
  --roles-visually-confirmed --episodes 5 --duration 45 --lead-in 5 `
  --operator-demo-capture-approved
```

## 2026-10-06 P12 acceleration-unit source reconciliation

The earlier audit treated the getter and setter scales as an unresolved
documentation conflict. The installed `piper_sdk` 0.6.2 sources and the
official interface/changelog distinguish the directions: acceleration
feedback uses `0.001 rad/s²` per raw unit, while setter arguments use
`0.01 rad/s²` per input unit. All six saved feedback values are raw `500`,
which is nominally `0.5 rad/s²` under the getter scale. The source hashes and
links are recorded in the existing
[P12 stage report](../../validation/p12_offline_preflight_timing_20261006.json).

This resolves the getter/setter scale confusion only. The disputed speed unit,
physical acceleration response, mechanical limits, low-speed setting, stop and
disconnect behavior, action-target source, and policy-to-command mapping still
block P12. No CAN, camera, or robot command was used in this audit; only the
P12 report JSON and installed SDK file hashes were checked. The validation
covered JSON parsing and source hashes, not robot behavior.

The offline single-step preflight now labels its result
`fake_can_offline_only`. Even with all 13 Fake test-fixture gates set to
`true`, it reports physical-gate verification and robot-command readiness as
false, with CAN TX disabled. The focused test reports 12 passed and 2
subtests passed; see the same P12 report for the result and updated source
hashes.

The 13-item matrix remains 8 complete, 3 partial, and 2 not started. Item 7
remains partial; item 12 remains not started, and CAN TX remains disabled. The
next actionable gate is still a traceable, timestamped joint/gripper command
target source; the existing successful episode contains feedback only.

## 2026-10-06 P12 单步预检审计哈希补强

单步预检此前分别返回 SafetyBridge 决策、P12 门槛和操作员批准，但没有一个
摘要把这些输入绑定在一起。现在 `preflight_audit` 将状态快照、完整命令提案、
全部门槛值、人工批准、阻塞原因、SafetyBridge 决策哈希和 Fake transport
发送计数纳入规范 JSON 的 SHA-256；审计事件可以独立复算。专测
`tests/laptop/test_single_step_dry_run.py` 为 `12 passed`、2 个子测试通过。

这只补强离线证据完整性。门槛仍是调用方提供的布尔记录，不构成现场证明；入口
仍只接受精确 `FakeCanTransport`，`CAN TX` 仍关闭，没有重跑硬件或 shadow。

### P12 实机前置门槛逐项核对

下表按 `P12GateEvidence` 的 13 个字段对照已保存的本机记录。`部分` 表示只读
数据存在，但含义或物理行为仍未确认；没有把历史读数升级为当前实机许可。

| 门槛 | 当前证据与状态 |
|---|---|
| 固件与协议 | 部分：读到固件 `S-V1.9-1`、SDK `0.6.2`；仅确认固件查询/反馈协议，不证明控制命令语义。[固件查询](../../validation/lab_firmware_query_20261006_170233.json) |
| 六轴状态单位 | 未验证：有原始反馈和名义转换，缺少以真实姿态/单位对照的校准证据。[关节参数读取](../../validation/lab_joint_parameter_read_20261006_170250.json) |
| 夹爪状态/命令单位 | 未验证：读到夹爪参数与反馈，但报告明确记录硬件单位未验证，配置行程不是物理量程证明。[夹爪参数读取](../../validation/lab_gripper_parameter_read_20261006_170212.json) |
| 控制器限值 | 部分：六轴角度、速度和加速度参数已读；反馈加速度 raw 500 按 getter 文档名义换算为 0.5 rad/s²，但没有验证实际加速度响应。[关节参数读取](../../validation/lab_joint_parameter_read_20261006_170250.json) |
| 机械硬限位 | 未验证：没有机械端点测量或经确认的硬件证据。 |
| 速度/加速度单位 | 部分：加速度 getter 反馈单位为 0.001 rad/s²，setter 输入为 0.01 rad/s²，尺度区别已由本机 SDK 0.6.2 源码和官方 changelog 核对；速度单位争议仍未解决，物理动态响应也未实测。[P12 单位审计](../../validation/p12_offline_preflight_timing_20261006.json)、[官方接口](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD)、[官方 changelog](https://github.com/agilexrobotics/piper_sdk/blob/master/CHANGELOG%28ZH%29.MD)、[速度单位 issue](https://github.com/agilexrobotics/piper_sdk/issues/122) |
| 低速设置 | 未验证：没有记录当前控制器低速设置及对应的实测速度。 |
| 操作员与急停 | 未验证：当前 P12 blocker 报告没有本次急停可用性证据。[P12 门槛报告](../../validation/p12_offline_preflight_timing_20261006.json) |
| 物理 watchdog | 未验证：没有实机超时后的观察结果。 |
| 断联 safe-hold | 未验证：没有断联后的实机保持行为证据。 |
| 相机/状态同步 | 未验证：shadow 记录了相机与 CAN 到达时间差，但没有相机曝光时间戳与机器人状态的同步证据。[最新 shadow 复核](../../validation/lab_live_shadow_signed_gripper_recheck_20261006_1840.json) |
| 动作目标来源 | **所选派生目标路线已验证**：原始 manifest、成功标签、对齐 metadata/时间行与 proxy targets 五项哈希匹配；199 条样本严格相隔 100 ms。[P12 派生目标来源审计](../../validation/p12_proxy_target_source_audit_20261007.json)。原厂 CAN 命令标签仍未找到，但不是该路线前提；proxy 也不是原厂命令，不证明单位或实机映射。 |
| 策略到命令映射 | 未验证：SDK 0.6.2 编码预览与官方编码器字节一致，仍没有实测方向、尺度或机械响应；`POLICY_REJECT_ONLY` 继续拒绝执行 proxy 目标。[最新 shadow 复核](../../validation/lab_live_shadow_signed_gripper_recheck_20261006_1840.json) |

后续数据按已选路线执行：取得 1 条新的成功 operator demonstration，验证来源和
10 fps 对齐，生成明确标注的未来状态派生目标，跑 20-step tiny-fit；首条验通后扩至
5 条。无需等待原厂命令日志。实际策略单步仍须独立完成关节/夹爪单位、方向和物理映射、
限位、低速、急停、watchdog、断联保持等现场验收，并取得该次运动的明确授权；本表和
预检哈希均不授权 CAN TX。

## 2026-10-06 全量 raw CAN 与文本 sidecar 动作目标来源审计

本轮离线扫描了 `datasets/raw` 下所有含 `can_id` 字段的 JSONL，共 8 个文件、
110,076 条记录。记录只包含 `0x2A1`、`0x2A5`、`0x2A6`、`0x2A7` 和 `0x2A8`
状态/反馈帧；没有关节命令 `0x155–0x157` 或夹爪命令 `0x159`。这 8 份 CAN
日志本身不能提供可追溯的命令目标，不能据此生成 operator-verified action。
随后扫描了 `datasets/raw` 下的 `.json`、`.jsonl` 和 `.txt` 文本旁车：9 份
`metadata.json` 都声明 `action_target_present=false`，全文没有
`operator_verified_target` 或命令目标字段。8 份 CAN 日志和 9 份 metadata 的
SHA-256 均已复核；成功 episode 的独立来源审计仍为
`action_target_verified=false`。逐文件行数和哈希见
[`全量 raw CAN 目标来源盘点`](../../validation/action_target_raw_corpus_inventory_20261006.json)。

本次没有查看图像/视频内容，也没有原厂软件导出可供核对，因此结论仅限于本地
CAN 日志和文本旁车；不从画面或状态变化反推动作命令。

最短补齐路线仍是获取带时间戳的原厂命令导出；若软件没有导出能力，就要在下一次
示教前选定并接入一个能记录真实命令目标的采集来源。旧轨迹继续只用于状态预测和
数据通路实验，失败标签保持不变。本次只读本地文件，没有访问 CAN、相机或服务器。

## 2026-10-06 18:40 夹爪负值反馈纠错与 live shadow 复验

5 秒只读 CAN 检查收到全部五类状态、关节和夹爪反馈，夹爪 `0x2A8` 收到
1,006 帧，解码错误为 0。当前夹爪原始值是 `-100`。整合 shadow 的两处
输入校验此前都要求夹爪原始值非负，因此把已收到并解析的反馈错误标为缺失。
现已改为校验严格整数但保留符号；负值作为观测值保留，不代表物理限位或控制
命令。此前操作员标记为“完全张开”的历史原始值为 `101800`，本轮为 `-100`；
由于同一物理姿态未在本轮独立核实，这两值暂不作为校准对照，也不据此判断硬件故障。
证据和复现链见
[`夹爪有符号反馈复核`](../../validation/lab_gripper_signed_feedback_recheck_20261006.json)。

修复后 `tests/laptop/test_live_shadow.py` 为 `21 passed`、15 个子测试通过。
第 11 项现场复验结束后，本机阶段全套测试为 `363 passed`、4 skipped、
65 个子测试通过；4 项跳过均因 LeRobot 只装在服务器/隔离数据环境或 Windows
不支持测试所需的 symlink。
整合 shadow 在 908 ms 内进入 `READY`，五个反馈通道各解码 235–238 帧、
解码失败为 0，并完成 3 次 prediction-only inference。相机最大 skew 为
23.96 ms，最旧传感器年龄最大 30.15 ms，往返延迟为 124–168 ms。
程序配置上限为 15 秒，但达到代码规定的 3 次预测上限后在 1.75 秒结束；
这不是 15 秒耐久测试。所有预测仍被 `POLICY_REJECT_ONLY` 拒绝，原因包括
单位未验证、proxy 不是命令、动作映射未验证和运行模式禁止 CAN TX。
CAN 数据帧发送与 SafetyBridge 发送尝试均为 0；本次结果见
[`有符号夹爪反馈修复后的 shadow`](../../validation/lab_live_shadow_signed_gripper_recheck_20261006_1840.json)。

现场状态读数是 `TEACHING_MODE(0x2)`、`NORMAL`、`STOP_RECORDING`、错误码 0。
这是观察值，不证明 safe-hold 或控制器状态可用于动作。固件、速度/加速度单位、
机构硬限位、watchdog、断联 safe-hold、可追溯命令目标来源和 policy-to-command
映射仍未通过，因此 P12/P13 保持锁定，未发送使能、回零、复位或运动命令。

## 2026-10-06 18:26 时序契约与 P12 离线预检

本轮新增版本化 `TimingContract`：预测延迟、预测首目标时间、状态年龄、提案
有效期与 watchdog 使用笔记本单调时钟，并写入 SafetyBridge 审计及输入哈希。
P12 单步入口现可通过同一 SafetyBridge 做离线预检，但只接受精确的
`FakeCanTransport` 和显式 `CommandProposal`；next-state proxy 不能转换成命令，
即使测试夹具把全部 P12 门槛设为通过，CAN TX 仍保持关闭。实现和测试证据见
[`P12 离线预检与时序验证`](../../validation/p12_offline_preflight_timing_20261006.json)。

专用 Conda 环境中的 `tests/contracts` 与 `tests/laptop` 为 `319 passed`。
同轮真实输入 shadow 的新复测结果为 `PARTIAL`：两相机角色与 policy 预热通过，
但 2 秒启动窗口没有收到 gripper CAN 反馈，因此没有执行 policy inference；CAN
数据发送和 SafetyBridge 发送尝试均为 0。临时 SSH 隧道已关闭。旧 live-shadow
通过结果仍是历史证据，但不能替代本轮新时序门的现场复验。

当前仍不具备实机单步条件：控制速度/加速度单位冲突、机械硬限位、经核验的低速
设置、急停准备、断联 safe-hold、相机与状态的曝光时间同步、夹爪物理单位映射，以及
可追溯动作目标仍未全部解决。成功 episode 的原始 CAN 日志没有关节/夹爪命令目标；
缺少 gripper 反馈时也不启动本轮预测。不得发送使能、回零、重置或运动命令。

## 2026-10-06 18:12 重启后的服务器合成拒绝复验

只读检查确认项目 proxy 服务仍监听 `127.0.0.1:8003`（PID `1441963`），占用
GPU UUID `GPU-7f906a98-2416-20bb-9198-3563ac6fa96a`，显存读数为 `73,588 MiB`。
另外两个 GPU 进程在另一张卡上，未触碰。通过临时本地 SSH 隧道运行合成输入，
返回有限 `16×7` proxy；`POLICY_REJECT_ONLY` 因合成/断开状态、单位未知、
proxy 非命令等理由拒绝。`camera_opened=false`、`can_opened=false`、
transport send 与 CAN TX 均为 0。结果见
[`重启后服务器合成拒绝复验`](../../validation/server_post_restart_synthetic_reject_20261006.json)。
临时隧道已关闭，端口 `18083` 已确认释放。这是服务器可用性与拒绝边界复验，
不是实时相机/CAN shadow，也不是实机控制验收。

同轮专用 Conda 环境测试结果为 `326 passed, 3 skipped, 60 subtests passed`；
3 项跳过是本机未安装固定版 LeRobot 数据集包，不涉及本次服务器合成请求。

## 2026-10-06 18:05 重启后的只读 CAN 反馈复核

Codex 重启后通过 `SafetyBridge` 重新执行了 5 秒反馈接收检查。仅被动读取
`0x2A1`、`0x2A5–0x2A8`，状态、六轴和夹爪通道分别收到 1,007–1,010 帧；
解码错误和短 USB 包均为 0。报告见
[`重启后被动反馈复核`](../../validation/lab_passive_feedback_recheck_post_restart_20261006.json)。

本次状态为 `STANDBY`、`NORMAL`、示教关闭、错误码 0。操作员此前报告夹爪
完全张开，最新反馈原始值为 `101800`，与已有的完全张开端点读数一致；这不代表
物理夹口宽度已测量。六轴反馈继续变化并有新时间戳。程序处于
`POLICY_REJECT_ONLY`，没有构造机器人 SDK 接口，没有发送 CAN 数据帧或运动命令。

此检查没有重新查询固件/参数，也没有验证单位、机械硬限位、速度/加速度尺度、
watchdog 或 safe-hold；因此不能解锁 HIL/P12。夹爪端点差异仍按小幅观察记录，
不作为严重硬件故障结论。

## 2026-10-06 17:54 成功示教的动作来源审计

已逐条检查 episode `cef166e2cc6240b397db241a916ece3a` 的原始 CAN 日志。
日志只有 `0x2A1`、`0x2A5–0x2A8` 五种状态/反馈帧，共 20,013 帧；没有
关节命令 `0x155–0x157` 或夹爪命令 `0x159`。采集器本身只订阅这五个反馈 ID，
并将 `action_target_present` 固定为 false。成功结果标签为 true，但
`action_target_verified` 和 `training_eligible` 均为 false。

因此这条成功示教可以继续支持已完成的 next-state proxy 预测实验，但它不含可追溯
到原始动作命令的训练目标；正式 LeRobot 转换器要求 `operator_verified_target`，
服务器 `local-tiny-fit` 继续保持阻止。本次没有篡改标签、创建正式数据集或启动训练。
审计和哈希见[`成功示教动作来源审计`](../../validation/action_target_source_audit_20261006.json)。

要解除数据门槛，需获得一条可核验的目标来源：原厂软件导出的关节/夹爪目标及时间戳，
或由操作者选择一种能同时记录人工示教目标与反馈的重新采集方式。如果决定把未来
实测关节状态作为离线训练目标，也必须明确标为独立的 next-state action 语义；这不会
自动授权 CAN 控制。

## 2026-10-06 17:48 单卡推理服务收敛

服务器当时有两个同一 proxy 的 loopback 服务：8002 元数据缺少 checkpoint、
policy、dataset、normalization 和 OpenPI identity hashes；8003 的身份完整，且
与本机 identity lock 一致。两者分别占用一张 GPU。确认 8002 没有已建立客户端后，
只对旧 PID 发 `SIGTERM` 并确认退出；没有 force-kill，也没有触碰 8003。现在仅
项目 policy 服务 8003 保留，独占一张 GPU；服务器上的其他 GPU 进程未更改。

通过临时 SSH loopback 隧道运行现有 synthetic reject-only smoke：8003 返回有限
`16×7` proxy，SafetyBridge 因 synthetic 来源、缺少实机状态和 proxy 非命令而拒绝；
相机/CAN 均未打开，CAN TX 与 transport send 尝试为 0。隧道已关闭。证据见
[`单卡服务收敛和合成复验`](../../validation/server_service_consolidation_20261006.json)
和[`8003 合成拒绝复验`](../../validation/server_current_survivor_8003_synthetic_recheck_20261006.json)。

## 2026-10-06 21:10 离线动作数据链路收尾与 13 项状态更新

本快照汇总实施顺序中的 13 项工作；它是当前状态索引，不覆盖下方的日期化证据。
本轮为下一条实机示教补齐离线动作目标链路：被动记录 `0x155–0x157`、`0x159`
原始帧及单调时钟；解析完整 7D 绝对目标；将 `align_raw`、逐帧来源重算、LeRobot
转换和训练审计接成一条哈希校验链。转换器从 checksum-bound 人工 review 派生 acceptance
sidecar，并把其哈希写入 conversion manifest。新增的文件回放客户端只读已核验的双相机帧/状态，向本地预测服务请求动作并与
记录目标比较；它没有相机、CAN、PiPER SDK 或 SafetyBridge 导入。针对性测试为 `131 passed`、
1 skipped、15 个子测试通过。训练配置修复为 train/eval/serve
共享相同的归一化目录，并把选中的 OpenPI step checkpoint、checkpoint 内统计量、数据集和
统计量来源互相绑定；该 OpenPI/LeRobot 集成测试须在服务器专用环境运行，本机 Conda 缺少
这些包，所以本轮只做了代码审查和语法校验。
这些是软件准备与 Fake/offline 验证，不表示已经收到新的真实命令目标、转换出真实 LeRobot
数据集、启动 π0.5 训练或通过第 12 项。采集器现在可按 USB bus/address 精确选择第二只
candleLight 适配器，并强制这只监听器使用硬件 listen-only；USB 身份枚举本身不会设置速率
或启动 CAN。仍需现场确认有独立适配器、其 listen-only 能力及确实能看到原厂上位机的目标帧。
不支持、也不假设原厂上位机与 recorder 共用同一 USB-CAN 适配器。
全量 raw CAN 审计进一步确认 8 份本地日志都只有反馈帧，未发现命令目标；见
[`全量 raw CAN 目标来源盘点`](../../validation/action_target_raw_corpus_inventory_20261006.json)。
时序契约与 SafetyBridge 预检之前的 `tests/contracts`、`tests/laptop` 结果为
`319 passed`，见
[`P12 离线预检与时序验证`](../../validation/p12_offline_preflight_timing_20261006.json)。
服务器 checkout `ac0e8b7` 的 `tests` 为 `61 passed`，但远端工作树仍有用户改动；
没有同步、重启服务或执行训练。该旧证据与此前 `local-tiny-fit` readiness 检查见
[`服务器测试与单集训练门槛`](../../validation/server_current_local_tiny_fit_gate_20261006.json)。
本轮没有打开摄像头或 CAN，没有发送 CAN 数据帧、启动训练或执行 Git 写操作。
随后只读枚举到一只 candleLight（bus 2/address 16）；Windows 相机 PnP 查询返回
“拒绝访问”。因没有第二只独立 listen-only 监听器，采集未启动、相机未打开；详见
[`P12 预检报告`](../../validation/p12_offline_preflight_timing_20261006.json)中的
`manual_command_capture_preflight_attempt`。13 项矩阵保持 8 完成、3 部分、2 未开始。

| # | 当前状态 | 已有证据 / 仍需门槛 |
|---:|---|---|
| 1 | 完成（离线契约） | 独立 prediction/command proposal 与版本化单调时钟 timing contract；真实目标来源/控制时序仍未冻结。 |
| 2 | 完成（离线配置） | proxy 配置与 PiPER 动作配置/身份隔离；动作服务仅 prediction-only。 |
| 3 | 完成（离线接口） | 独占 CAN transport 抽象与 Fake transport 已测试；真实命令发送实现仍禁用。 |
| 4 | 完成（离线拒绝桥） | RuntimeMode、状态机、哈希审计及时序契约已测试；不代表 HIL 可执行。 |
| 5 | 完成（软件故障注入） | 非有限值、错单位、过期/乱序、响应超时、目标过期、限位、断连、网络、USB 与进程故障有拒绝测试。 |
| 6 | 完成（拒绝式运行） | `POLICY_REJECT_ONLY` 保持 CAN TX 为 0。 |
| 7 | 部分完成（只读实机） | 已读固件、六轴参数、夹爪参数和反馈；加速度 getter/setter 尺度已澄清，但速度单位、物理响应、夹爪映射、机械硬限位、低速、急停、物理 watchdog 和断联 safe-hold 仍未通过。历史夹爪端点不能视为单位校准。逐门槛证据见上表和 P12 单位审计。 |
| 8 | 部分完成（仅离线） | 单关节和夹爪提案经过同一 SafetyBridge；Fake-only 单步 rehearsal 只写内存，并明确声明 offline-only、不声明实机 readiness。仍没有 command-capable HIL sender；实机单步未验收。 |
| 9 | 部分完成（软件） | 离线 watchdog/fail-closed 与多故障路径已有测试；物理 watchdog、safe-hold 及经 HIL 的平滑轨迹未验证。 |
| 10 | 完成（既有单集实验）；本机采后 tiny-fit 管线待同步至服务器并待真实数据执行 | 2/10 fps 来源审计、10 fps 10-step tiny-fit 与同集复评已完成并核对 hashes，本轮未重跑。5 条合成 episode 转换/回读/预检通过。本机新增一条命令完成成功 episode 筛选、对齐、派生目标、pinned LeRobot 转换/回读、train-only stats、单 GPU 20-step 默认 tiny-fit 和同集评估；默认 1 条成功示教即可跑通，5 条以上更适合多示教 pilot。服务器远端文件尚未同步，真实训练未运行。所有 action 是未来实测状态派生目标，不是 PiPER 原厂命令。 |
| 11 | 完成（旧 2 fps prediction-only live shadow）；新 checkpoint shadow 接线完成、待训练产物 | 旧路线已有 3 次实时预测及 camera/CAN 身份、时间、延迟和 reject reason 核验；CAN TX 为 0。新路线的服务 metadata、loopback identity lock 和笔记本 `live_shadow.py` 校验逻辑已接通并测试；支持一条成功示教的 prediction-only checkpoint，但本轮无新 checkpoint，未启动真实服务或运行新模型 shadow。 |
| 12 | 未开始（安全门锁定） | 既有/待新增的派生状态模型仅可 prediction-only/shadow，不能直接当作已验证 PiPER 命令；8 份旧 CAN 日志仍只有反馈帧。受监督实机单步需要先完成 P12 单位、限位、低速、急停、watchdog、断联保持，并获得对具体动作的逐次同意；Fake/离线训练不计为 P12。 |
| 13 | 未开始（依赖第 12 项） | 只有受监督单步可重复验收后，才评估短闭环 rollout。 |

当前矩阵仍为 8 项完成、3 项部分完成、2 项未开始。这里保留两条不能混淆的监督语义：
原厂命令监督必须捕获真实命令帧，反馈日志不能反推；最简离线派生路线不声称是原厂命令，
而把时刻 `t+Δ` 的反馈明示为 proxy position target，因此无需为这条离线路线先找到
`0x155–0x159`。新 operator-demo recorder 只读相机/反馈、不进示教模式、不发 CAN 数据帧；
转换前必须通过成功标签、时间/帧来源、LeRobot 解码回读与 hash 核验。模型仍只能做
prediction-only/shadow，P12/P13 的实机控制门槛不因此改变。

正式 raw-command `local-tiny-fit` readiness 仍为 `BLOCKED`：成功标注的 episode
`cef166e2cc6240b397db241a916ece3a` 的 `action_target_verified=false`，正式 acceptance
文件和 `datasets/lerobot/local/piper_v1` 均不存在。这与服务器已经完成的单集
derived-state 10 fps tiny-fit 是两条不同路线：后者可作同集预测 sanity check，但不能
静默升级为原厂命令监督、绝对控制验证或实机成功。

官方 SDK 文档复核没有解除这些硬件门槛：接口页在 `GetArmGripperMsgs` 的字段
说明中写 `0.001 mm`，但同页返回摘要又写 `0.001°`；0.4.3 release note 表示
夹爪值已改为行程、单位 mm，且本机安装的 0.6.2 源码按 `0.001 mm/raw` 标注，
因此可把它作为名义换算，不能替代实测夹口宽度。速度查询注释与 issue #122
提到的协议尺度存在 10 倍冲突；同一接口页的加速度 getter 写 `0.001 rad/s²`、
setter 写 `0.01 rad/s²`。该页没有说明通讯超时 watchdog 或断联 safe-hold；它明确
写明 `ResetPiper` 会失去动力并下坠，因此不得将 reset 当作安全停止方式。来源及
判断记录见[`官方 SDK 单位与停止语义审计`](../../validation/official_sdk_semantics_audit_20261006.json)。

## 2026-10-06 17:26 真实 observation shadow 复测

本次在 15 秒上限内重新运行双相机与 CAN 被动输入，并验证更新后的启动就绪门。
程序在三次预测后结束，总运行约 2.24 秒；未发送 CAN 控制帧。

本机 `8002` 首次连接到缺少 artifact identity 字段的旧服务，客户端在打开相机和
CAN 前正确拒绝。随后确认服务器 `8003` metadata 与本机 identity lock 一致，使用
临时 SSH loopback 隧道完成复测；隧道已关闭。完整报告见
[`启动就绪修复后的 live shadow`](../../validation/lab_live_shadow_startup_fix_20261006_172633.json)
和[旧服务身份拒绝记录](../../validation/lab_live_shadow_startup_fix_20261006_1719.json)。

启动门在 `1,397.5 ms` 后取得两路首帧和五类 CAN 反馈，`startup_readiness=READY`，
启动期与运行期的拒收计数均为 0。三次 policy round-trip 为 `148.75–159.33 ms`，
相机 skew 最大 `10.81 ms`，最旧传感数据年龄最大 `25.64 ms`。每次 prediction
仍由 `POLICY_REJECT_ONLY` 拒绝，原因包括单位未核验、proxy 不是命令、映射未验证和
运行模式禁止 CAN TX；`can_data_tx_calls=0`、`safety_bridge_send_attempts=0`。
第 11 项的身份、时间、延迟和拒绝原因已在本次真实输入 shadow 中复核；这不是
实机动作测试或策略成功证明。

## 2026-10-06 17:12–17:17 夹爪反馈端点复核

本节记录操作员确认的夹爪闭合与完全张开状态，以及两次被动 CAN 读取。
读取只接收反馈帧，没有发送 CAN 数据帧，也没有操作机械臂关节。

| 操作员观察状态 | 原始行程字段 | 读取帧数 | 解码错误 |
|---|---:|---:|---:|
| 闭合 | 200 | 1,007 | 0 |
| 完全张开 | 101,800 | 1,003 | 0 |

`piper_sdk 0.6.2` 的安装源码将反馈字段标为 `0.001 mm`/raw unit。按该
名义比例换算，两个端点约为 `0.2 mm` 和 `101.8 mm`，差值为
`101.6 mm`。同日 `0x477` 参数读取报告记录的配置最大行程为 `100 mm`；
因此，端点差按 SDK 名义比例比配置行程大 `1.6 mm`，约为 `1.6%`。这是
小幅差异，不视为严重硬件问题。该换算尚未用物理夹口间距测量验证，控制配置
继续保留 `min=null`、`max=null`；这不影响只读反馈、离线训练或
prediction-only shadow，在用于控制目标或限位前再确认比例和零点即可。

两份原始摘要、SDK 字段注释哈希和参数读取来源已固化在
[`夹爪反馈端点对比`](../../validation/lab_gripper_endpoint_feedback_comparison_20261006.json)。
这证明 CAN 反馈随操作员观察到的开合状态显著变化，但不证明机器人能独立
抓持物体，也不解除 SafetyBridge 的拒绝控制状态。后续动作标定时再记录物理
夹口间距与反馈对应点；watchdog 和 safe-hold 是独立实机控制门槛。

## 2026-10-06 实验室最新复测

本节优先于同日较早的相机和 CAN 快照，记录 17:00–17:05
（Asia/Singapore）的当前现场复测。它证明只读数据链路可用，不代表
单位、物理限位、safe-hold 或实机控制已验收。

| 角色 | 设备 | OpenCV index | 实测帧率 | 读帧失败 |
|---|---|---:|---:|---:|
| Base | ACER HD User Facing | 0 | 30.18 FPS | 0 |
| Wrist | Dabai DC1 | 1 | 29.19 FPS | 0 |

相机测试持续 10 秒，两路均有有效图像信号。新 shadow 使用同一设备身份
并得到 3 次预测；其启动轮询记录了 12 次缺少首帧和 5 次 CAN 快照尚未
齐备的拒收，随后均正常取得新鲜输入。`sensor_errors` 为空。这些启动期
不完整快照不等于相机驱动读帧失败。测试没有保存视频。详见
[双相机复测](../../validation/lab_camera_preflight_20261006_170029.json)和
[现场 shadow](../../validation/lab_live_shadow_20261006_170525.json)。

USB-CAN 适配器为唯一枚举的 `1d50:606f`。5 秒只读反馈收到五类状态、
关节和夹爪反馈，每类约 1,008 帧；解码错误为 0，数据帧发送数为 0。
状态为 `STANDBY`、`NORMAL`、示教关闭、错误码 0。固件单次查询得到
`S-V1.9-1`。报告分别见
[被动反馈](../../validation/lab_can_readonly_preflight_20261006_170113.json)
和[固件查询](../../validation/lab_firmware_query_20261006_170233.json)。

经已批准的只读 getter，六轴角度/速度与加速度各读取一次，共发送 12 个
`0x472` 查询帧，没有设置参数或运动。控制器报告的角度范围与此前上位机
截图一致；这不是机构硬限位证明。六轴速度原始值均为 300，SDK 查询文档
换算为 0.3 rad/s，而上位机显示 3.0 rad/s；加速度原始值均为 500，查询
文档与 setter/界面分别对应 0.5 和 5.0 rad/s²。单位冲突仍未解决，不能
作为动作限速依据。详见
[本次六轴参数读取](../../validation/lab_joint_parameter_read_20261006_170250.json)。

夹爪单次 `0x477` getter 返回行程比例 100%、配置行程 100 mm、示教摩擦
1；当前反馈原始行程值为 42700。该参数是控制器配置，不证明实际 jaw
间距或反馈单位。详见
[本次夹爪参数读取](../../validation/lab_gripper_parameter_read_20261006_170212.json)。

现场 shadow 使用固定 identity lock，三次推理往返为约 148–166 ms，最大
传感器数据年龄约 22.6 ms，CAN 数据发送为 0。SafetyBridge 因
`state_units_unverified`、代理状态不是动作命令、映射未验证及
`POLICY_REJECT_ONLY` 模式而拒绝所有预测；模型仍是同一 episode 的
`next_observed_state_proxy_nominal_units` tiny-fit，不是 PiPER 正式策略。
SSH 隧道已关闭。不得据此开放实机控制。

下一项现场门槛是由操作员核实夹爪实际间距与反馈值的对应关系，并在确认
方式后评估 watchdog/safe-hold；在单位、命令映射和这些硬件行为通过前，
P12 单步动作与 P13 rollout 均保持锁定。

## English executive summary

The project has verified the OpenPI deployment, one assisted real PiPER
demonstration, a 39-frame 2 fps proxy dataset, a source-audited 199-frame
10 fps proxy dataset, bounded 2 fps and 10 fps tiny-fit checkpoints, and
WebSocket shadow inference. The 10 fps checkpoint is a same-episode
prediction-only data-path experiment, not a formally trained PiPER policy.
The project has not authorized a model-to-CAN control loop.

- Keep the GPU server responsible for training and inference only.
- Keep the Windows laptop responsible for cameras, PiPER feedback, safety, and
  all robot authority.
- Treat `next_observed_state_proxy_nominal_units` as a prediction-only state
  reference, never as a verified CAN command.
- Make one laptop SafetyBridge process the exclusive USB-CAN owner for both
  feedback and commands.
- Keep `action[0]` as the only initial candidate. Do not introduce temporal
  ensembling until per-action timing and closed-loop behavior are verified.
- Validate watchdog, disconnect, stale-response, unit, limit, and safe-hold
  behavior before any multi-joint or policy-originated command.
- Preserve the existing P0-P14 phase gates. Do not create a second phase
  numbering system.
- Refactor incrementally. Preserve the laptop/server boundary and keep the
  existing supervised scripts as auditable references during migration.
- Keep tests grouped by laptop, data pipeline, server, and shared contracts;
  see [project layout](PROJECT_LAYOUT.md).

## 文档权威与适用范围

本文件是后续开发顺序和目标架构的当前计划入口。它不替代不可变的运行证据，
也不修改实机授权门槛。

发生冲突时，按以下顺序解释项目状态：

1. 原始 manifest、validation 报告和 checkpoint provenance 证明已经发生的
   事实。
2. [阶段门槛](../PHASE_GATES.md)和[实机安全计划](../history/REAL_ROBOT_SAFETY_PLAN.md)
   决定当前允许执行的操作。
3. 本文件决定后续工作的优先级、架构边界和实施顺序。
4. YAML 配置表示准备如何运行，不能覆盖验证证据或安全门槛。
5. 设备和进程的实时状态必须通过现场检查获得，不能从旧报告推断。

早期 P0/P1 部署记录保留在
[Master plan](../history/PI05_PIPER_MASTER_PLAN.md)。原始计划与历史版本继续保留，
但不作为当前执行优先级的来源。

## 当前已验证状态

本节只记录有代码、报告、数据或现场检查支持的结论。技术通过、离线预测和
实机成功是三种不同的声明。

### 软件与服务器

OpenPI 官方子模块固定在提交
`215abfb217dbac7d5f1273282331b9b1866c0479`。服务器使用项目内 `.venv` 和
JAX；它不能访问 PiPER CAN，也没有机器人控制权限。

- `pi05_base` 官方 ALOHA smoke、localhost policy server 和客户端往返测试
  已通过。
- SSH 别名 `rtx6000_Mingkai` 可连接服务器 `workstation`。
- October 5, 2026 的现场复测启动了 GPU0 上的 proxy policy server；远端
  `127.0.0.1:8002` 通过 SSH 隧道提供 prediction-only 推理。该运行状态是
  临时快照，不能从文档推断服务在未来仍然存活。
- `8000` 保留给早期官方 `pi05_base`/ALOHA smoke；PiPER proxy shadow
  代码默认使用 `8002`。
- 部署前的 2026-10-06 只读复核确认远端 OpenPI 环境可导入，注册的 pi0.5 配置包括
  `pi05_aloha`、`pi05_libero` 等；`pi05_base` 是 checkpoint 身份而不是该 checkout
  中的配置名。现有 proxy 服务仍监听 `127.0.0.1:8002`，元数据指向
  `cef166e2_two_step/1`，并标记 `prediction_only=true`、控制关闭。当时一次合成输入
  推理返回有限的 `16×7` 输出，client round-trip 为 `75.06 ms`。该结果不含真实
  机器人观测，也不代表真机延迟或控制能力。当时服务器 checkout 为 `ac0e8b7`，
  工作树原本有未提交改动；该快照没有做批量同步。完整记录见
  [`server_openpi_environment_recheck_20261006.json`](../../validation/server_openpi_environment_recheck_20261006.json)。

同日先使用服务器 `.venv` 在单张 GPU 上评估已有 2 fps 数据和两个 checkpoint。
数据有 39 帧、1 个 train-only episode；step 1 的同 episode 关节 MAE 为
`0.306816 rad`、夹爪名义误差为 `0.011631 m`，step 9 分别为 `0.306400 rad` 和
`0.011588 m`。两者几乎没有差别；这不是 held-out 泛化测试。该次评估时服务器
checkout 为 `ac0e8b7` 且保持 dirty，本机 10 fps 数据集尚未上传。之后用户明确授权
上传派生数据并在服务器工作；完成的 10 fps 统计、tiny-fit 与同 episode 评估见下文
及 [`proxy_10fps_tiny_fit_20261006.json`](../../validation/proxy_10fps_tiny_fit_20261006.json)。
2 fps 评估原始结果见
[`proxy_2fps_server_eval_20261006.json`](../../validation/proxy_2fps_server_eval_20261006.json)。

### 本地采集与硬件

Windows 专用 Conda 环境包含 Python 3.10、OpenCV、`openpi-client`、
`piper_sdk`、PyUSB、`gs_usb` 和 Conda 内的 `libusb-1.0.dll`。项目不使用
ROS2 或 EtherCAT。

- 已确认的 Base 角色是 `ACER HD User Facing`，上次绑定为 OpenCV index
  `0`。
- 已确认的 Wrist 角色是 `Dabai DC1`，上次绑定为 OpenCV index `1`。
- 相机 index 不是稳定身份。每次重新插接后必须结合 Windows InstanceId
  和现场画面重新确认角色。
- candleLight USB-CAN 的 VID:PID 是 `1D50:606F`，CAN 比特率为
  `1,000,000 bit/s`。
- October 5, 2026 的现场复测中，Base、Dabai 和 candleLight 均显示 `OK`。
  操作员画面核验确认 OpenCV index `0` 是 Base、index `1` 是 Wrist。5 秒
  并发测试分别得到 152 和 142 帧，读取失败均为 0。
- 同次 CAN ACK-only 测试收到全部五类必需反馈，每类约 1,000 帧；状态为
  `STANDBY(0x0)`、`NORMAL(0x0)`、示教关闭和错误码 `0`。程序发送的 CAN
  数据帧数为 0。
- 经操作员明确批准后，程序使用 Conda 中的 `piper_sdk 0.6.2` 参考协议，
  通过 candleLight 适配器对 J1–J6 各查询角度/速度和加速度一次，共发送
  12 个 `0x472` 参数读取帧；全部收到对应的 `0x473` 和 `0x47C` 回复。
  查询耗时约 `0.017 s`，未构造 SDK 控制接口。
- CAN 角度范围与操作员提供的上位机截图六轴全部一致：J1 `±150°`、J2
  `0…180°`、J3 `−170…0°`、J4 `±100°`、J5 `±70°`、J6 `±170°`。
  所有速度回复的原始值均为 `300`；按 SDK 查询回复文档的 `0.001 rad/s`
  单位换算为 `0.3 rad/s`，与界面 `3.0 rad/s` 相差十倍。
- 所有关节的加速度回复原始值均为 `500`。SDK 查询回复说明写作
  `0.001 rad/s²`，但 SDK 设置接口说明写作 `0.01 rad/s²`；因此不能仅凭
  这个字段断定实机值是 `0.5` 还是 `5.0 rad/s²`，目前保留原始值并标记
  单位冲突。
- J6 的 CAN 配置为 `±170°`，而官方 SDK 名义 `JointCtrl` 范围为 `±120°`。
  这确认了控制器配置，不证明机构可安全运行到 `±170°`；任何软件运动限制
  仍采用两者交集，不扩大至 SDK 名义范围之外。
- 同一短查询期间还收到了状态、三组关节和夹爪反馈：状态为
  `CAN_CTRL(0x1)`、`NORMAL(0x0)`、示教停止、错误码 `0`；六轴名义反馈为
  `[-89.703, 22.210, -62.037, -0.572, 50.628, -6.198]°`；夹爪原始反馈为
  行程字段 `52100`、力度字段 `100`、状态码 `0x40`。CAN 回读未确认这些夹爪
  数值的物理单位。
- 查询共发出 12 个 `0x472` 参数读取帧；没有发送模式、使能、回零、目标位置、
  夹爪控制、参数设置或复位帧，没有命令机械臂运动。
- 原始字段、回复 payload、CAN 收发统计和现场状态快照保存在
  [CAN 参数回读报告](../../validation/can_joint_parameters_20261005.json)。
- 操作员提供的上位机截图显示 J1–J6 角度范围分别为 `±150°`、`0…180°`、
  `−170…0°`、`±100°`、`±70°`、`±170°`，界面最大速度均为 `3 rad/s`，
  最大加速度均为 `5 rad/s²`。这些值记录为界面显示配置，不等同于已验证的
  机械硬限位。CAN 角度与界面全部一致，但六轴速度均相差十倍，加速度字段
  单位仍有 SDK 文档冲突；J6 的 CAN/界面范围 `±170°` 也大于官方名义
  `±120°`。较早反馈日志的名义 J2/J3 范围
  还分别越过界面范围约 `1.96°` 和 `1.82°`。单位、零位或控制分支未核对前，
  这些差异保持未解决，不扩大任何软件安全限位。逐关节截图读数和差异见
  [`joint_limit_software_display_20261005.json`](../../validation/joint_limit_software_display_20261005.json)。
- 2026 年 10 月 5 日完成 20 秒 ACK-only 反馈变化采集。20,016 条日志无
  缺失 ID、重复/倒退时间戳、100 ms 以上间隔或格式错误；五类反馈中位更新
  间隔约 5 ms，最大间隔约 22 ms。六关节及夹爪数值均有变化；全过程维持
  `STANDBY`、`NORMAL`、示教关闭、错误码 0，采集程序发送 0 个 CAN 数据帧。
  分通道范围和日志哈希记录在
  [`feedback_change_20261005.json`](../../validation/feedback_change_20261005.json)。
- 该采集没有人工事件时间标记；关节名义角度单位仍未由硬件独立确认，夹爪
  单位也未确认。因此它证明反馈字段会变化和接收链路连续，不作为控制单位
  标定或具体手动动作时间对应关系的证据。
- 2026 年 10 月 6 日本机仍枚举到两台相机和 candleLight USB-CAN，但一次新的
  5.048 秒 ACK-only 读取在五类必需反馈 ID 上都收到 0 帧。程序发送 0 个 CAN
  数据帧、未保存逐帧日志；当前解析器显示的零关节/夹爪值只是无帧时的默认值，
  不能当作实机状态。固件、角度/加速度参数和夹爪反馈均未在本次读取中核验。
  证据见 [`can_passive_feedback_20261006.json`](../../validation/can_passive_feedback_20261006.json)。
  这不能单独区分 CAN 线未接、机械臂未通电或其他链路问题；在反馈帧恢复前不发送
  `0x472`/`0x4AF` 查询，也不继续任何 HIL 操作。
- 随后的本机 `pnputil` 枚举仍显示 ACER、Dabai DC1 和 candleLight (`1D50:606F`)
  设备为 Started；同日新的 5.007 秒 SafetyBridge `POLICY_REJECT_ONLY` 被动读取中，
  五类 PiPER feedback ID 仍全部为 0 帧，`can_data_tx_calls=0`，没有构造 SDK 控制
  接口或命令机械臂运动。无帧时解析器保留的零值仍只是默认值。此次设备枚举不能证明
  CAN 端接线或机械臂供电，固件/单位/夹爪/safe-hold 均未核验；证据见
  [`can_passive_feedback_recheck_20261006.json`](../../validation/can_passive_feedback_recheck_20261006.json)。

以上设备状态只是日期化快照，不能作为下一次实验的连接证明。

### 数据、训练与评估

真实 episode `cef166e2cc6240b397db241a916ece3a` 已完成来源和画面核验。
操作员报告夹爪成功将黑棒放到绿垫，但手部遮挡和辅助意味着它不能证明机器人
独立承载物体。

- 40 个对齐样本和 80 个视频帧通过来源检查。
- 4,003 个状态样本均为 `TEACHING_MODE(0x2)`、`NORMAL(0x0)` 和错误码
  `0`。
- 双相机 host-arrival skew 中位数为 `11.2817 ms`，最大值为
  `36.8974 ms`；这不证明曝光同步。
- 当前 proxy 数据身份是 `local/pi05_proxy_cef166e2_2fps`。
- 数据只有 1 个 episode、39 帧、2 fps，并且仅有 train split。
- action 语义是 `next_observed_state_proxy_nominal_units`。
- 对原始成功 episode 的完整性审计发现完整状态时间窗外各有 2 帧相机帧，且
  没有已验证动作目标；这使原始 episode 整体保持 `INCOMPLETE` 和不可正式训练。
  已选取的 2 fps preview 单独完成 40 个样本、80 张视频帧的逐样本来源核验。
  现有 39 帧 proxy 转换预检通过，保持 train-only 和 prediction-only。
- 另有 199 帧、10 fps 的 `local/cef166e2_proxy10fps_ds36` 派生候选数据集。
  新增的服务器入口校验数据身份、固定版 LeRobot 运行时、train-only split、
  逐行 source-map/readback 关联、Parquet 与双路视频哈希；stats/train/eval 还要求
  外部传入本机生成的数据树和 readback 审计 SHA-256。其 config、stats 和 checkpoint
  命名与 2 fps 分离，2 fps 仍为默认。当前本机输入校验已通过 7 项故障注入测试，
  真实候选的来源审计通过。用户随后明确授权传输该派生数据集（未传 raw capture）；
  服务器端 train-only normalization、10-step tiny-fit 和同 episode prediction-only
  评估均已完成。10 fps 结果见
  [`proxy_10fps_tiny_fit_20261006.json`](../../validation/proxy_10fps_tiny_fit_20261006.json)。
  本机预检 hash 记录见
  [`proxy_10fps_training_preflight_20261006.json`](../../validation/proxy_10fps_training_preflight_20261006.json)。
- `cef166e2_two_step/1` 和 `cef166e2_ten_step/9` 均存在。
- 10-step 在相同训练 episode 上仅有极小误差改善，不能证明泛化或实机
  可控性。
- 正式 PiPER policy 训练、独立验证集和正式 fine-tuning 均未完成。

### 控制与安全

当前在线链路止于 prediction-only shadow。模型预测没有接入 CAN。

- `laptop/client/live_shadow.py` 能读取双相机和 CAN 反馈、构建 observation、
  调用服务器、把 prediction 送入 reject-only SafetyBridge，并记录决定。
- 此前的现场 shadow 在打开相机和 CAN 前使用合成 observation 预热模型；随后三次
  真实 observation 推理延迟为 `138.07 ms`、`130.62 ms` 和 `129.67 ms`，没有
  prediction 因 observation 过期而拒绝。这与本轮新服务的合成推理不是同一测试。
- `laptop/safety/offline_guard.py` 是纯函数离线检查器，没有 CAN 或机器人
  API。
- 多个 supervised 脚本包含受限 CAN 发送路径，但它们没有连接 π0.5，也
  不是统一 SafetyBridge。
- 当前没有完整的 ActionChunkBroker、在线 SafetyBridge、经过实机验证的
  watchdog，或经过固件确认的 safe-hold。
- `hardware_control_allowed=false` 和 `robot_command_sent=false` 仍是当前
  proxy policy 的强制边界。
- proxy server 为 checkpoint 树、policy 配置、serving/data-transform 源码包、
  tiny-fit 源码、normalization 文件和 LeRobot 数据树计算内容 SHA-256，并记录固定
  OpenPI commit。shadow client 在打开相机或 CAN 前把完整 metadata 与本机 identity
  lock 逐项比较；缺锁、字段缺失或身份不符都会 fail-closed。
- 2026-10-06 已将服务入口和 artifact identity helper 部署到服务器现有 dirty
  checkout，保留原有未提交文件，并保留可恢复备份。新 prediction-only 服务运行于
  空闲 GPU1 的 `127.0.0.1:8003`；原 GPU0 `127.0.0.1:8002` 服务未重启或修改。
  经本机 Conda 客户端和 SSH 隧道发送合成 observation，metadata 与本机锁匹配，
  返回有限的 `16×7` 输出，首次推理约 `7.50 s`。随后 10 次 warm synthetic 请求的
  round-trip p50/p95 为 `108.30/184.24 ms`，模型计时 p50 为 `57.45 ms`；最大
  round-trip 为 `242.41 ms`。相机、CAN 均未打开，也没有发送机器人命令。这些是
  合成 observation 的 server/client timing，不是真机端到端延迟。
- 2026-10-06 又通过临时 SSH loopback 隧道对 GPU1/8003 运行 P3 合成网络探测：
  10 次预热后 1000/1000 个双 RGB + 7D state 请求成功，artifact identity 与本机
  lock 相符；client round-trip p50/p95/p99 为 `108.20/116.88/124.30 ms`，客户端
  关闭后重新握手为 `68.42 ms`。本次使用零值合成图像/状态，没有打开相机或 CAN，
  未改变 `configs/network_template.yaml` 中的长期网络选择；该 RTT residual 包含
  序列化、SSH tunnel、服务器排队和调度，不能解释为纯网络延迟。记录见
  [`network_latency_ssh_tunnel_20261006.json`](../../validation/network_latency_ssh_tunnel_20261006.json)。
  这只通过 1000 请求合成基线；P3 仍未完全关闭，长期 NET 方案尚未选择，也未做
  隧道中断后的实测恢复或 network template 的全部 payload profile。
- 2026-10-06 又通过临时 SSH loopback 隧道对现有 GPU1/8003 prediction-only 服务
  完成 50 个合成请求复测。50/50 请求成功，重连后 metadata 与本机 artifact lock
  一致；client round-trip p50/p95/p99 为 `107.84/124.02/126.61 ms`，模型计时 p50
  为 `57.36 ms`，重新握手为 `120.66 ms`。测试后临时隧道已关闭，两个既有
  localhost-only 服务仍在监听。该短测不替代 1000 请求基线，也不是纯网络或实机
  延迟测量。离线故障测试增加了推理超时、无效响应和重连身份变化场景，聚焦套件
  9 项及 server-independent 套件 228 项均通过。证据见
  [`network_latency_ssh_tunnel_recheck_20261006.json`](../../validation/network_latency_ssh_tunnel_recheck_20261006.json)、
  [`network_latency_probe_unit_tests_20261006.json`](../../validation/network_latency_probe_unit_tests_20261006.json)
  和 [`offline_test_execution_network_recheck_20261006.json`](../../validation/offline_test_execution_network_recheck_20261006.json)。
  P3 仍为部分完成：注入故障不代替真实隧道中断/恢复测试，长期网络方案与所有
  payload profile 仍未验证。

## 目标系统架构

目标架构把机器学习域和控制域分开。服务器只返回预测；笔记本解释、验证并
最终否决任何可能到达 CAN 的提案。

```text
               ML domain / 机器学习域

Base + Wrist RGB       PiPER feedback snapshot
        \                       /
         └──── ObservationAssembler ────┐
                                        ↓
                                  PolicyClient
                                        ↓
                              SSH tunnel + WebSocket
                                        ↓
                                  π0.5 server
                                        ↓
                               PolicyPrediction
                         [NEXT_OBSERVED_STATE_PROXY]

               Hard semantic boundary / 硬语义边界
                                        ↓
                              ProxyReferenceValidator
                                        ↓
                              StateReferenceController
                                        ↓
                                CommandProposal

               Control domain / 控制域
                                        ↓
                              TrajectoryGenerator
                                        ↓
                              PiperSafetyBridge
                          exclusive CAN RX/TX owner
                               /                \
                  state snapshots              CAN commands
                             ↑                       ↓
                         CAN feedback ← PiPER controller
```

SafetyBridge 必须成为唯一打开 USB-CAN 的进程。它负责接收反馈、发布只读状态
快照、验证 CommandProposal，并在获得明确授权时发送受限命令。相机、policy
client、recorder 和 HIL 工具不得各自打开同一适配器。

SafetyBridge 与 policy runtime 最终使用 Windows localhost IPC 通信。实现时
采用版本化、长度前缀的 msgpack 消息，并只绑定 `127.0.0.1`；禁止使用 pickle
作为跨进程协议。协议至少包含状态快照、命令提案、安全决定和 heartbeat。

## 动作与时间契约

动作契约必须先阻止错误解释，再允许经过验证的转换。增加类型名称本身不能把
proxy 变成硬件命令。

### PolicyPrediction

`PolicyPrediction` 表示模型输出，不表示执行授权。计划接口至少包含以下字段：

```text
request_id
request_sequence
observation_timestamp_ns
received_timestamp_ns
checkpoint_hash
model_config_hash
normalization_hash
dataset_identity
action_semantics
actions[H, 7]
target_dt_s[H]
joint_unit
gripper_unit
prediction_only
```

当前实例必须设置：

```text
action_semantics = NEXT_OBSERVED_STATE_PROXY
target_dt_s[0] = approximately 0.5
prediction_only = true
hardware_units_verified = false
```

`target_dt_s[i]` is an offset from `observation_monotonic_ns`, not from the
server response or a future execution time. State, response, proposal creation,
and proposal expiry timestamps use the same laptop monotonic clock. Never
compare server wall-clock time with laptop monotonic time.

任何代码不得把这一对象直接交给 CAN transport。

### CommandProposal

`CommandProposal` 是控制域唯一接受的候选输入。只有经过验证的 reference
controller 才能创建它。

```text
proposal_id
source_prediction_id
q_target_rad[6]
gripper_target_m
valid_from_ns
valid_until_ns
action_semantics = ABSOLUTE_JOINT_POSITION_COMMAND
max_velocity
max_acceleration
runtime_mode
```

在硬件单位、目标时间和状态跟踪行为通过实机验收前，
`ProxyReferenceValidator` 必须拒绝生成 `CommandProposal`。

### 当前 2 fps 限制

当前第一步 proxy target 约对应 observation 后 `0.5 s` 的状态。它不能作为
20–50 Hz 的即时位置命令。EMA 或插值只能改变平滑程度，不能修复错误的时间
语义。

初期继续只评估 `action[0]`。在逐 action 时间戳、开放环误差和闭环替换规则
通过验证前，不启用 action chunk buffer、temporal ensemble 或大于 1 的执行
horizon。

后续必须比较两条路径：

1. 保留 500 ms reference，仅用于受监督、限时的状态参考实验。
2. 从原始视频和 CAN 重新构造更高频率的离线数据，并重新训练独立 checkpoint。

两条路径都不能绕过动作语义和硬件单位验收。

## 运行模式

运行模式必须是显式状态机，而不是散落的布尔值。进程启动和任何异常恢复均
回到 `SHADOW`。

- `SHADOW`：允许相机、CAN RX 和 policy prediction；代码级禁止 CAN TX。
- `HIL_SINGLE_STEP`：不连接 policy；只允许一个由操作员明确授权的受限
  command。
- `POLICY_REJECT_ONLY`：完整 policy proposal 进入 SafetyBridge，但所有
  CAN TX 强制拒绝。
- `SUPERVISED_POLICY_STEP`：一次只处理一个新鲜 proposal；操作员逐次授权，
  SafetyBridge 保留最终否决权。
- `AUTONOMOUS`：当前不存在；只有对应 P0–P14 门槛全部通过后才能实现。

暂不定义 `ARMED_HOLD`。真实 PiPER 的 hold、停止刷新、失能和恢复语义尚未
经过固件验证，不能用名称代替安全证据。

## SafetyBridge 责任

SafetyBridge 是控制域的最终权限边界。通过软件单元测试不等于通过实机安全
门槛。

SafetyBridge 必须执行以下检查：

- request ID、sequence、checkpoint、config 和 normalization 身份；
- observation、response、camera、CAN feedback 和 heartbeat 新鲜度；
- shape、NaN、Inf、单位和 action semantics；
- 真实 joint/gripper hard limit 和软件 soft limit；
- 单步 delta、velocity、acceleration 和目标有效期；
- PiPER control mode、health、error 和 motor enable 状态；
- 断网、服务器异常、USB 断开、相机丢失和进程退出；
- 独占 CAN 所有权和未知命令帧检测；
- 每次 ACCEPT/REJECT 的原因、输入哈希和时间戳记录。

独立 Windows 进程只能隔离软件故障，不能替代硬件急停。进入 P12 前必须在
真实固件上验证无新命令、进程崩溃、USB 断开和急停后的实际行为。

## 数据与训练路线

当前 proxy tiny-fit 只证明数据和训练代码能够连通。后续不得通过增加训练步数
把单 episode 记忆解释成策略能力。

正式路线保持以下边界：

- 保留当前 2 fps proxy checkpoint，作为 prediction-only 基线。
- 保留失败 episode 的失败标签，只用于数据通路和错误分析。
- 使用原始数据重新采样时，保留 source frame、CAN 引用、时间戳和哈希。
- 在相机曝光同步无法证明时，继续标明 host-arrival timestamp 限制。
- 在获得独立 action target 前，不把下一实测状态标为正式控制标签。
- 先完成 episode 级 train/validation 划分，再只用 train split 计算统计量。
- 先用 5 条轨迹验证 recorder 可靠性，再扩展 20–50 条 pilot 数据。
- 正式 checkpoint 必须绑定 OpenPI commit、数据 manifest、split、norm stats、
  config 和 checkpoint hash。

正式 [PiPER model config](../../configs/model/pi05_piper_v1.yaml) 保持
`training: NOT_STARTED` 是正确的，因为已完成的是 proxy 实验，不是正式
PiPER policy。后续为 proxy 新建独立 policy 配置，不修改正式配置来冒充进度。

## 配置、证据与状态

项目必须把期望配置、不可变证据和瞬时运行状态分开。任何一类都不能替代另外
两类。

后续逐步建立以下结构：

```text
configs/hardware/piper_lab.yaml       desired hardware binding
configs/policy/pi05_proxy_shadow.json desired checkpoint and semantics
configs/runtime/shadow.json           desired runtime mode and endpoints

validation/                           immutable verification evidence
datasets/**/manifest.json             immutable source/data identity
checkpoint provenance                 immutable model identity

status/current_status.json            generated live snapshot
docs/CURRENT_STATUS.md                generated human-readable summary
```

未核验的 firmware、单位、limit 和 safe-hold 字段必须保留为 `null` 或
`UNVERIFIED`，并触发 fail-closed。不得为了让配置完整而填写推测值。

相机配置同时记录 OpenCV index、Windows InstanceId、friendly name 和角色确认
时间。运行时发现 InstanceId 或画面角色不一致时，必须停止而不是自动交换。

服务器入口不得继续硬编码 `cef166e2_two_step/1`。未来 proxy 配置必须明确
checkpoint 路径和哈希；由于 10-step 没有显著优势，迁移配置时不得自动把它
提升为默认策略。

## 渐进式代码整理

当前代码已经按 `laptop/` 与 `server/` 分开，并将测试按子系统分组；但
笔记本端仍有多个脚本密集目录，且 `laptop/camera/` 目前只有说明文档，实时
相机逻辑仍在 `client/` 和 `diagnostics/`。因此下方结构是目标，不是当前实际
目录。详细的现状、模块职责和入口边界见[项目与测试布局](PROJECT_LAYOUT.md)。

不进行全仓库大搬迁。重构必须与行为测试同步，并保留 laptop/server 主机边界。

目标目录逐步收敛为以下职责；只在对应模块有可运行回归测试后再迁移代码：

```text
laptop/
  camera/
  observation/
  policy_client/
  control/
  transport/
  runtime/
  tools/hil/

server/
  policy_server/
  training/
  dataset_tools/

tests/
  contracts/
  data/
  laptop/
  server/
```

入口文件只负责加载配置和启动 runtime；核心逻辑移入可测试模块。优先从
`laptop/client/` 抽出 observation 与 policy-client 职责，再将相机采集实现迁入
`laptop/camera/`。每次迁移只处理一个职责，先跑相关测试，再跑完整本地套件。

现有 `supervised_j1_step_trial.py`、`supervised_smooth_joint_replay.py`、
`supervised_restore_gripper.py` 及其他 `supervised_*.py` 在真实 SafetyBridge
HIL 路径具备测试证据前保持原位，不做改名、删除或归档。它们是审计入口，不是
当前推荐的模型控制入口。

测试目录按其负责的子系统分组；unit、integration、shadow 和 HIL 是验证与授权层，
不是另一套必须创建的目录树。风险层次如下：

- unit：纯函数，不打开相机、CAN、网络或模型；
- integration：recorded data、fake policy 和 FakeCanTransport；
- shadow：真实相机、CAN RX 和服务器，但 CAN TX 为零；
- HIL：真实 CAN TX，必须有操作员、急停和明确授权。

## 实施顺序

后续开发必须按以下顺序推进。每一步通过后仍不能自动获得下一步的实机权限。

1. 定义 proxy prediction、command proposal 和 timing contract。
2. 新增独立 proxy policy 配置，并移除 checkpoint 硬编码。
3. 抽象唯一 `PiperCanTransport` 和 `FakeCanTransport`。
4. 实现 SafetyBridge 离线状态机、RuntimeMode 和审计事件。
5. 注入 NaN、错单位、过期、乱序、超限、断网、USB 断开和进程退出故障。
6. 接通 `POLICY_REJECT_ONLY`，保持 CAN TX 强制关闭。
7. 重新连接实机，核验 firmware、单位、真实 limit、夹爪和 safe-hold。
8. 让单关节和夹爪 HIL 测试统一经过 SafetyBridge。
9. 验证 watchdog 和 fail-closed 后，再执行多关节平滑轨迹。
10. 构造更高频率离线数据实验，并保留 2 fps 基线和全部来源证据。
11. 重新运行真实 shadow，核验身份、时间、延迟和 reject reason。
12. 在 P12 条件下执行一条受监督 policy proposal。
13. 只有单步路径可重复通过后，才计划短 closed-loop rollout。

这一顺序继续使用现有 P0–P14 阶段编号。不得创建第二套 Phase 0–8 来表示相同
工作。

## 验收与停止条件

每个技术成果必须与它实际证明的范围匹配。以下条件任一存在时，policy-originated
CAN TX 必须保持禁止。

- proxy 与真实 command 的映射未验证；
- target timing 或 response deadline 未冻结；
- firmware、关节单位、夹爪单位或真实 limit 未确认；
- SafetyBridge 不是唯一 CAN 所有者；
- stale、乱序、断连或 watchdog 故障不能 fail-closed；
- safe-hold 和急停恢复行为未在真实固件上确认；
- 相机身份、状态新鲜度或同步检查失败；
- checkpoint、config、normalization 或 dataset 身份不匹配；
- 操作员和物理急停不在现场；
- 上一个 P0–P14 门槛缺少对应证据。

通过一次模型推理、一次 tiny-fit、一次 shadow、一个单关节动作或一条人工辅助
成功轨迹，都不能单独解除以上停止条件。

## 离线实施状态 — October 6, 2026

当前代码已经完成实施顺序第 1–6 步的离线部分。这个结论只表示类型、配置、
拒绝路径和测试已经接通，不表示实机控制已经获得授权。

- `PolicyPrediction` 和 `CommandProposal` 现在是不同的不可变类型。代码明确
  拒绝把 next-observed-state proxy 隐式转换为 PiPER 命令。
- 独立 proxy 配置固定使用 `cef166e2_two_step/1`、2 fps、16 × 7 action
  chunk、`prediction_only=true` 和 `hardware_control_allowed=false`。服务器
  入口从该配置加载 checkpoint、端口、时间语义和数据身份，不再硬编码
  checkpoint step。
- `PiperCanTransport` 定义统一的独占所有权接口，`FakeCanTransport` 用于离线
  验证。live shadow 通过 SafetyBridge 接收 CAN 帧；传输层普通 `send()` 永远拒绝
  命令提案。单独的 `DIAGNOSTIC_QUERY` 仅允许经 SafetyBridge 的精确 getter 请求，
  它们属于 CAN data TX，但不能控制机械臂。safe-hold 或硬件限位验证仍未实现。
  Fake transport 可注入 USB 断开和 transport 进程退出，并由 SafetyBridge fail-closed。
- `SafetyBridge` 默认使用 `SHADOW` 和 `POLICY_REJECT_ONLY`；另有隔离的
  `DIAGNOSTIC_QUERY` 查询模式。HIL/策略控制模式仍在构造阶段拒绝；影子和
  reject-only 模式不发 CAN 帧，查询模式也不能调用通用命令发送路径。
- `single_step_dry_run.py` 现在把单步命令提案交给 reject-only SafetyBridge，
  并输出标准审计事件；关节和夹爪提案共用同一拒绝路径，只接受精确的
  FakeCanTransport；Fake transport 和真实 CAN 发送次数保持为 0。它是
  离线合同演练，不是 HIL 通过。7 个现存 `supervised_*` 原始 CAN 发送入口现在
  即使收到确认标志也会在打开适配器前 fail-closed；其拒绝事件现通过 SafetyBridge
  生成并随异常携带。这个共享 veto 没有 CAN transport，也不是 command-capable
  HIL bridge，因此计划第 8 项的物理单关节/夹爪 HIL 尚未完成，不能把离线 rehearsal
  当作单步实机验收。
- SafetyBridge 生命周期显式为 `READY → FAULT_LATCHED → CLOSED`。状态、时间戳、
  重放顺序、heartbeat 或 proposal 校验异常会锁存故障；后续 heartbeat 不能解除，
  关闭后不能再接受 heartbeat。当前只能在诊断原因后重建桥接实例恢复。
- `SafetyBridge.watchdog_tick()` 现在用独立的 `max_heartbeat_age_ns` 检查
  heartbeat，输出带输入哈希的 audit event，并对缺失、过期、未来时间戳及无效时钟
  锁存故障。该 tick 由调用方调度；目前没有实时线程、CAN 发送或机器人 hold，不能
  作为实机 watchdog/safe-hold 验收，也不解除 HIL 禁止状态。
- 夹爪软件限值必须是有限数，时间新鲜度窗口必须是正整数；NaN/Inf 不能利用
  浮点比较绕过过期或夹爪 delta 检查。提案创建时间早于来源状态到达时间时会拒绝。
- SafetyBridge 的 prediction 和 command-proposal 决定现在都提供标准化审计事件，
  带单调时钟决定时间和输入 SHA-256；`live_shadow.py` 将该事件写入每条
  prediction 记录。收到时间晚于当前单调时钟的 prediction 会以
  `prediction_received_from_future` 拒绝，而不再被误判为新鲜数据。
- `POLICY_REJECT_ONLY` 回放器对缺字段、非字典样本、错误根对象和错误 predictions
  容器都返回同一审计 schema，包含单调决定时间、原始输入哈希和明确拒绝原因；
  非有限 JSON 数值会先规范化再哈希，不会破坏拒绝日志。
- 故障注入覆盖 NaN、错误 shape、未确认单位、过期状态、乱序/重放、断连、
  网络超时/拒绝连接/连接重置、USB 断开、transport 进程退出、worker
  超时/崩溃、异常状态、缺失/过期 heartbeat、关节/夹爪越限和未知 IPC
  消息。较早一次 2026-10-06 笔记本专用 Conda 的 server-independent suite 共通过
  196 项、跳过 3 项、48 个 subtests；测试排除了需要 OpenPI/LeRobot 的五个项目模块
和 upstream `third_party/openpi`。SafetyBridge、reject-only 回放、live shadow、离线
guard、桥接协议和 control contract 六个聚焦模块单独通过 68 项及 3 个 subtests。
完整运行与排除范围见
[`offline_test_execution_20261006.json`](../../validation/offline_test_execution_20261006.json)。
- 本机服务和 shadow client 现在绑定六项内容哈希（checkpoint、policy config、
  serving/data-transform pipeline、远端 tiny-fit config source、normalization、
  dataset tree）及固定 OpenPI commit。源码和
  文本配置哈希先统一 CRLF/LF，checkpoint 哈希只在服务启动时计算一次。客户端在
  打开相机/CAN 前，将服务 metadata 与本机 identity lock 逐项匹配；缺锁或身份不符
  会 fail-closed。该次 identity-lock/layout 验证的 server-independent suite 通过
  207 项、跳过 4 项和 63 个
  subtests；18 项聚焦测试、1 项平台限制跳过，另有 15 个 subtests 通过。此次测试
  没有访问硬件。报告见
  [`offline_test_execution_project_layout_20261006.json`](../../validation/offline_test_execution_project_layout_20261006.json)。
- `live_shadow.py` 将结构化 proxy prediction 送入 reject-only SafetyBridge，
  并记录拒绝原因。该接线已在重新连接后的相机和 CAN 上运行；3 个 prediction
  全部按设计拒绝控制，SafetyBridge transport send attempts 和 CAN 数据帧
发送数均为 0。逐条拒绝原因保存在原始 shadow 和 SafetyBridge audit 中，
其哈希由现场验证报告引用；每条都包含 `state_units_unverified`、
`next_observed_state_proxy_is_not_command`、`proxy_to_command_mapping_unverified`
和 `runtime_mode_forbids_can_tx`。

同一成功标注的 raw episode 已生成 2 fps 基线和 10 fps candidate，并由可重跑的
采样率审计逐条验证两路原始视频帧、CAN/关节/夹爪引用、原始文件哈希、outcome
标签和 proxy 目标哈希。基线为 40 个对齐帧/39 个 next-state 样本；10 fps
candidate 为 200 个对齐帧/199 个样本。该对比不做插值、不训练模型、不改变现有
2 fps policy 配置，也不授予真机控制资格。

验证记录见离线
[`policy_reject_only_offline_20261005.json`](../../validation/policy_reject_only_offline_20261005.json)
和现场
[`live_shadow_recheck_20261005.json`](../../validation/live_shadow_recheck_20261005.json)，
逐条实机 shadow 与 SafetyBridge reject reason 见本地原始日志
`logs/hardware_recheck_20261005/live_shadow_prewarmed.json` 和
`logs/hardware_recheck_20261005/policy_reject_only_prewarmed_audit.json`；2/10 fps
数据来源对比见
[`proxy_sampling_rate_comparison_20261005.json`](../../validation/proxy_sampling_rate_comparison_20261005.json)。
以及本次完整六轴参数
[`can_joint_parameters_20261005.json`](../../validation/can_joint_parameters_20261005.json)。
早期 J2/J3-only
[`j2_j3_limit_query_20261005.json`](../../validation/j2_j3_limit_query_20261005.json)
作为历史记录保留，已由完整六轴回读取代。笔记本专用 Conda 环境中与本机
可用依赖相符的离线测试通过 165 项，1 项跳过，52 个 subtests 通过；这次运行
排除了 5 个需要 `openpi` 或 `lerobot` 的模块。聚焦故障集另通过 34 项和
3 个 subtests。2 fps 与 10 fps proxy 分别通过转换器只读预检，得到 39 和
199 个样本。完整运行证据见
[`offline_test_execution_20261005.json`](../../validation/offline_test_execution_20261005.json)。

服务器 `.venv`（Python 3.11.16、`datasets==3.6.0`、固定 LeRobot revision）可导入
`openpi`、`lerobot`，并检测到 4 张 RTX PRO 6000。服务器 HEAD `ac0e8b7` 与笔记本
HEAD `99ab21c` 不同，服务器工作树原本已有改动；本轮未覆盖或同步远端代码。
复核发现服务器已有 `pi05_base`、2 fps proxy 数据集，以及两步/十步 prediction-only
checkpoint。该服务器数据的 proxy manifest/target SHA-256 与本机一致，Parquet 中
state/action 数值哈希也与新本机转换结果一致；但 Parquet 和编码后视频文件字节不
同，不能称为同一份转换产物。服务器当前没有本机 10 fps 数据集。

本轮在一张当时空闲的 GPU 上重新评估已有两步和十步 checkpoint，使用同一训练
episode 的 5 个帧索引 `[0, 10, 20, 30, 38]`。两步 checkpoint 的关节 MAE 为
`0.30670 rad`、夹爪名义误差为 `0.011637 m`；十步 checkpoint 分别为
`0.30623 rad`、`0.011589 m`。差异很小，且这是同 episode 抽样，不是泛化测试，
更不是实机任务成功或控制安全证据。评估没有训练或发送机器人命令，详见
[`server_proxy_prediction_recheck_20261006.json`](../../validation/server_proxy_prediction_recheck_20261006.json)。

## Next steps / 下一步

本轮已部署并验证独立 prediction-only 服务：GPU1 上的 `127.0.0.1:8003` 返回
符合本机 artifact lock 的合成预测，GPU0 上既有 `127.0.0.1:8002` 服务保持不变。
服务器随后在物理 GPU2 上完成一次 10-step 10 fps proxy tiny-fit，checkpoint 位于
`checkpoints/pi05_piper_next_state_proxy_10fps_tiny_fit/cef166e2_10fps_ten_step/9`；
另以同一 episode 的 5 个帧索引做 prediction-only 评估。该实验只验证数据/训练/预测
通路，不是泛化、正式 PiPER 微调或控制验证。完整结果见
[`proxy_10fps_tiny_fit_20261006.json`](../../validation/proxy_10fps_tiny_fit_20261006.json)。
完整 server-independent suite 最新结果为 224 项通过、4 项跳过和 63 个 subtests；
复跑记录见 [`offline_test_execution_network_20261006.json`](../../validation/offline_test_execution_network_20261006.json)。
服务器 `.venv` 在不同且已有未提交改动的 checkout 上此前通过 52 项。

2 fps 与 10 fps 数据已在隔离环境 `pi05_lerobot_dataset` 中按 OpenPI 固定版本重建，
新目录分别为 `cef166e2_proxy2fps_ds36` 和 `cef166e2_proxy10fps_ds36`。运行时锁为
`datasets==3.6.0`、LeRobot revision `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`；
转换器现在会在版本不符时于写文件前拒绝，并将版本写入转换 provenance。两份数据
分别有 39/199 条 Parquet 记录；base/wrist 两路视频都逐帧解码通过，PTS 单调，
每路为 39/199 帧、640×480。每一帧都按来源索引与原始预览图核对；LeRobot API
也成功读取了每份数据的首尾样本。Parquet state/action、索引、逐行来源映射均通过
审计，详细哈希保存在数据目录的 `provenance/`。旧目录
`cef166e2_proxy2fps`、`cef166e2_proxy10fps` 是用 `datasets==4.8.5` 生成的，仍作为
历史产物保留，但不兼容 OpenPI 锁定的 `datasets==3.6.0`，不应继续送入训练。

两份新数据的动作语义仍是 `next_observed_state_proxy_nominal_units`，均为单个
train-only episode，且 `formal_training_eligible=false`、
`hardware_control_allowed=false`。服务器物理 GPU2 上已计算 10 fps train-only
normalization，SHA-256 为 `d51ad915215b2735fbd12bb41169795bf0970bdab44d54df086eaed26847f35a`；
并完成 10 个优化 step（保存 step 9）及 5 帧同 episode prediction-only 评估。
评估平均关节 MAE 为 `0.291078 rad`，夹爪名义绝对误差为 `0.013012 m`。它没有
held-out episode，不能证明泛化或真机可控。用户明确授权后，服务器接收了派生的
10 fps LeRobot 数据与来源证据；原始本机 CAN/相机 capture 未上传。数据树与
readback-audit 哈希见预检报告，完整训练、统计和评估记录见结果报告。新的环境没有
修改原 Conda 环境。

SafetyBridge 审计与锁存状态机的离线聚焦测试通过 68 项和 3 个 subtests；未来
接收时间戳、非有限安全限值、早于源状态的提案和格式错误回放均 fail-closed，
拒绝事件绑定输入哈希。这只改善离线可追溯性和故障锁存，不改变拒绝式运行模式，
不开放 CAN TX。详见
[`safety_bridge_state_machine_20261006.json`](../../validation/safety_bridge_state_machine_20261006.json)。

本轮又为 SafetyBridge 增加独立 heartbeat-age watchdog tick。失效会产生哈希审计事件
并锁存桥接；恢复 heartbeat 不会清除故障，FakeCanTransport 的发送次数始终为 0。
相关回归测试 62 项通过和 3 个 subtests；完整离线 suite 为 218 项通过、4 项跳过、
63 个 subtests。该验证不覆盖线程调度抖动、进程崩溃后的硬件行为、CAN TX 或
safe-hold。记录见
[`offline_test_execution_watchdog_20261006.json`](../../validation/offline_test_execution_watchdog_20261006.json)。

本轮补充了真实子进程崩溃注入：子进程发布一次 heartbeat 后由父进程强制终止；在
超过 heartbeat-age 窗口后调用 watchdog tick，SafetyBridge 锁存 `FAULT_LATCHED`
并拒绝恢复，FakeCanTransport 的发送次数仍为 0。新用例与 SafetyBridge 回归共 60 项
通过；本机可运行全套为 328 项通过、4 项跳过及 63 个 subtests。该测试验证的是
“heartbeat 停更后由调用方 tick 检出”的离线语义；独立调度器、操作系统级进程监控、
真实机器人 watchdog 和 safe-hold 仍未实现或验证。证据见
[`process_exit_watchdog_20261006.json`](../../validation/process_exit_watchdog_20261006.json)。

数据集的 `local/pi05_proxy_cef166e2_2fps` 是指向 `datasets/incoming` 内部目录的
符号链接；服务先解析并限制目标仍位于配置数据根目录内，再计算 dataset tree hash。
checkpoint 哈希仍拒绝符号链接，不因该数据布局而放宽。

本机 identity lock 位于
[`pi05_proxy_shadow_identity_lock.json`](../../configs/runtime/pi05_proxy_shadow_identity_lock.json)，
绑定 checkpoint、policy config、serving pipeline、现有远端 tiny-fit 源码、stats、
2 fps 数据树和 OpenPI commit。10 fps tiny-fit 临时使用扩展脚本，实验后已从标准
`proxy_piper_tiny_fit.py` / `proxy_piper_eval.py` 路径恢复实验前的精确版本，以保持
2 fps artifact identity 不变；原有 server dirty checkout 文件未被清理。10 fps 训练与
评估脚本哈希记录在结果报告中。
部署的 serving 源码与本机 lock 哈希匹配。新服务启动时校验 checkpoint 树哈希一次，
后续请求复用启动时的 metadata，不逐次重哈希，也不复制 checkpoint。
服务器部署与合成推理证据见
[`server_proxy_deployment_20261006.json`](../../validation/server_proxy_deployment_20261006.json)。

2026 年 10 月 6 日的后续离线复核将 SafetyBridge 的当前单调时钟入口收紧为
正整数校验；布尔值、非正值、非有限值和字符串时钟均只产生拒绝审计，不会触发
发送。SafetyBridge 现在依赖统一 `PiperCanTransport` 协议，而非写死 Fake 实现。
聚焦回归通过 85 项和 18 个 subtests；本机 Conda 可运行套件通过 273 项、跳过 4 项，
服务器 OpenPI/LeRobot 环境全套测试通过 61 项。测试未访问 CAN、未启动训练、未重启
推理服务。细节见
[`safety_bridge_clock_validation_20261006.json`](../../validation/safety_bridge_clock_validation_20261006.json)。

同日又把 live-shadow 与被动 PiPER feedback 的 CAN 接收统一到
`GsUsbPiperCanTransport`：SafetyBridge 独占 claim/start/read/stop/release 生命周期；
candleLight 普通命令 `send()` 固定抛出 `PermissionError`。getter 请求只由
SafetyBridge 的独立 `DIAGNOSTIC_QUERY` 通道提供，并且使用精确白名单，不能表达
机械臂动作。Fake 与真实传输共用同一接口。离线测试模拟适配器扫描、
receive-mode flags、帧读取、USB 断开和发送拒绝；本机 Conda 可运行测试集通过 286 项、跳过
4 项、63 个 subtests，现有 53 个测试模块仍按子系统分组。没有连接 CAN 或运行硬件
测试，所以这不证明真实适配器行为。详见
[`transport_unification_validation_20261006.json`](../../validation/transport_unification_validation_20261006.json)。

本轮又把固件、关节参数和夹爪参数查询脚本迁移到 `CanDiagnosticSession`，由
SafetyBridge 持有同一个 `PiperCanTransport`，并要求五类新鲜 PiPER 反馈、无臂状态
错误、显式 CLI 批准及每次会话的查询预算。白名单只包括官方 firmware/joint/gripper
getter；所有 command proposal 路径仍拒绝。USB 写失败按 delivery-unknown 处理并关闭
适配器，不自动重试。Fake 传输测试与脚本静态隔离检查证明的是本地代码行为，不是
CAN 当前可读性，也不是已执行查询。本机聚焦套件通过 119 项（含 12 个 subtests），
本机可运行全套通过 317 项、跳过 4 项及 63 个 subtests；
未连接或访问真实 CAN。2026-10-06 最近一次无帧被动读取仍然有效，恢复实时反馈前不运行
getter 查询。报告见 [`diagnostic_query_bridge_validation_20261006.json`](../../validation/diagnostic_query_bridge_validation_20261006.json)。

同一轮继续将 7 个旧 HIL 入口的 fail-closed 决策统一为 SafetyBridge 产生的带哈希
审计事件。每个脚本仍在枚举 USB 或调用 CAN 前抛出异常；事件显式记录
`can_tx_attempted=false`、实机门槛未核验和 HIL 发送路径未实现。该变化只统一拒绝与
审计，不创建或解锁 HIL command sender。对应离线测试覆盖全部 7 个入口；本机全套
回归仍为 317 项通过、4 项跳过和 63 个 subtests；没有接入实机。细节见
[`hil_entrypoint_bridge_gate_20261006.json`](../../validation/hil_entrypoint_bridge_gate_20261006.json)。

随后将普通 ACK-only、PiPER 反馈、motor-feedback 和 listen-only 被动探针也统一到
`SafetyBridge` 与唯一 `PiperCanTransport` 所有权边界。普通 ACKing 与精确 listen-only
接收模式明确区分；listen-only 不能发 getter 查询。诊断目录静态扫描不再发现直接
调用 `GsUsb.scan`、设备 `send/read`、`GsUsbFrame` 或 USB claim 的代码。Fake 传输聚焦
回归为 100 项及 3 个 subtests；本机可运行全套为 327 项通过、4 项跳过及 63 个
subtests。上述全部是离线软件验证，没有枚举适配器、读取实时 CAN、发送诊断帧或
执行服务器操作；最新现场被动读取仍是五类反馈 ID 全部为 0 帧，因此 getter 与 HIL
仍保持停止。证据见
[`passive_can_probe_safety_bridge_20261006.json`](../../validation/passive_can_probe_safety_bridge_20261006.json)。

获准后对服务器 `ac0e8b7` 做了只读复核：原有 dirty working tree 保留，8002/8003
两个 prediction-only 服务进程仍在监听；服务器 `.venv` 全套测试再次通过 61 项
（5.30 秒）。本次没有上传、训练、重启服务或哈希多 GB checkpoint。复核记录见
[`server_authorized_offline_recheck_20261006.json`](../../validation/server_authorized_offline_recheck_20261006.json)。

本轮再次只读连接服务器：checkout 仍为 `ac0e8b7` 且工作树 dirty；
`127.0.0.1:8002` 与 `127.0.0.1:8003` 仍有服务监听，服务器测试套件 61 项通过。
本轮没有写服务器文件、运行训练、请求模型推理或重启服务。监听和测试结果不等于
真实观测 shadow；本机 CAN 五类反馈仍为 0 帧。对应证据一并记录在
[`process_exit_watchdog_20261006.json`](../../validation/process_exit_watchdog_20261006.json)。

随后经临时 SSH loopback 隧道对现有 8003 服务发送 1 次预热和 1 次计时的合成
双 RGB + 7D state 请求，并完成关闭客户端后的 metadata 重连校验。artifact lock 在
初连及重连后均匹配，计时请求返回有限的 `16×7` 输出；单次 RTT 为 `77.9945 ms`，
模型采样计时为 `57.8322 ms`。该输入由全零合成图像和状态构成，不能代表实机延迟或
任务表现；请求未打开相机/CAN，也没有发送机器人命令。临时隧道端口已确认关闭，
服务未重启。原始测量见
[`server_current_synthetic_request_20261006.json`](../../validation/server_current_synthetic_request_20261006.json)。

2026 年 10 月 6 日，用户说明当前人在实验室外，PiPER 已断电；因此当前无法收到
CAN 反馈是预期的现场条件，本轮不再尝试 CAN，也不把这个状态当作新的通信故障诊断。
这项说明只描述当前断电状态，不追溯改变之前日期化被动读取报告的事实。

同日本机仍枚举到 `ACER HD User Facing` 与 `Dabai DC1` 两台相机。无录像双流检查发现
OpenCV index `0` 在 `DSHOW`、`MSMF` 和自动后端下均未打开；index `1` 可打开。随后用
`DSHOW` 对 index `1` 与 `2` 做 5 秒并发读取，两个 index 均各取得 5 帧，读取失败为 0，
最近帧 host-arrival skew 最大 `0.40 ms`，但有效帧率只有约 `1 FPS`。本次没有保存视频，
也没有通过画面确认 index 与 Base/Wrist 的对应关系；它只证明这两个索引当时能取帧，
不能验收相机身份、采集帧率或示教数据质量。记录见
[`camera_stream_backend_recheck_20261006.json`](../../validation/camera_stream_backend_recheck_20261006.json)
和 [`camera_stream_index_recheck_20261006.json`](../../validation/camera_stream_index_recheck_20261006.json)。

本轮新增了 loopback-only 的 `synthetic_policy_reject_smoke.py`，把合成双 RGB + 7D 状态
经当前服务器 `8003` 预测后直接送入 `POLICY_REJECT_ONLY` SafetyBridge。服务器 metadata
与本机 artifact identity lock 一致，返回有限 `16×7` 输出；桥接因 proxy 不是命令、无真实
机器人状态且硬件单位未核验而 fail-closed，审计事件带输入哈希，Fake CAN 发送尝试和
CAN 数据 TX 均为 0。该链路验证的是服务器到拒绝边界，不是实机 shadow 或控制。
临时 SSH 隧道已关闭。结果见
[`synthetic_policy_reject_smoke_20261006.json`](../../validation/synthetic_policy_reject_smoke_20261006.json)。

随后本机专用 Conda 环境中 `tests/contracts`、`tests/laptop` 和 `tests/data` 共通过
309 项、跳过 3 项，另有 58 个 subtests；服务器项目 `.venv` 中 `pytest tests` 通过
61 项。笔记本环境刻意不包含服务端的 OpenPI/LeRobot 训练依赖，所以服务端测试在服务器
环境运行。以上都没有打开 CAN 或启动训练。

本轮又把 `shadow_preview_smoke.py` 扩展为有界 historical-preview 回放：逐样本重算
source/frame-map 哈希，检查 18 张导出帧与原视频一致，并在调用服务器前校验 artifact
identity lock。2026 年 10 月 6 日从 `passive_wiring_smoke` preview 选择第 0、4、8 帧，
经当前 `8003` 返回三个有限 `16×7` 预测；端到端耗时分别为 `182.42 ms`、`153.49 ms`、
`155.69 ms`。三个结果都经 `POLICY_REJECT_ONLY` SafetyBridge 审计，因离线来源、没有
当前机器人连接/状态、硬件单位未验证且 proxy 不是命令而拒绝；Fake CAN 发送尝试为 0。
本次没有打开相机/CAN，历史 preview 仍为不可训练数据，也不等于实时 shadow。临时 SSH
隧道已关闭。证据见
[`recorded_preview_reject_replay_20261006.json`](../../validation/recorded_preview_reject_replay_20261006.json)。

同日追加的相机属性读取显示 index `1` 与 `2` 的 driver-reported FPS 都是 `0`，实际
host-read 速率约 `1 FPS`；分辨率为 `640×480`。由于该次没有验证这两个索引对应哪台
物理相机，这只是当次 OpenCV DirectShow 索引源的读帧速率，不是 Base/Wrist 的硬件帧率
或稳定性能结论。详见
[`camera_stream_properties_20261006.json`](../../validation/camera_stream_properties_20261006.json)。

随后重新检查历史映射的 OpenCV index `0/1`：`DSHOW` 与自动后端下 index `0` 未打开、
index `1` 打开；`MSMF` 下两者均未打开。因配对未能建立，本次没有测出这组相机的吞吐率。
这与 10 月 5 日经画面确认的 index `0/1` 双路约 `30 FPS` 不可直接比较，表明当前索引或
采集源映射已变化或不可用；不能据此断言相机硬件只有 `1 FPS`。后续必须重新把稳定的
Windows 设备身份与实际画面绑定，再测单路和双路 host-read 速率。该检查没有保存视频，
也没有访问 CAN 或机械臂。结果见
[`camera_stream_backend_recheck_20261006.json`](../../validation/camera_stream_backend_recheck_20261006.json)。

后续按以下顺序推进：

1. 服务器端 10 fps 派生数据上传、train-only normalization、单 GPU 10-step tiny-fit
   和同 episode prediction-only 评估已经完成。保留该实验仅用于验证数据/训练通路，
   不将其称为泛化或正式 PiPER 微调；2 fps policy、checkpoint、stats 和身份锁保持
   不变。服务器 checkout 与笔记本 HEAD 不同且原有工作树 dirty，后续仍禁止批量同步
   或清理无关改动。
2. 用户当前说明 PiPER 因人不在现场而断电，所以本轮不要求处理 CAN。用户回到实验室
   并给机械臂通电、接好 CAN 后，先重新做只读反馈检查；只有五类反馈持续更新时才查询
   固件及参数。2026 年 10 月 5 日的历史回读虽确认六轴角度限位，但速度缩放仍有十倍
   差异、加速度单位仍冲突、夹爪物理单位与 safe-hold 未确认；这些查询不证明机械
   硬限位，也不扩大运动安全范围。`laptop/diagnostics/query_piper_readiness_snapshot.py`
   将固件、六轴角度/速度/加速度及夹爪配置读取串成一个有界入口；它要求显式
   `--send-documented-getter-queries` 授权、每个阶段各自通过新鲜反馈预检，最多发送
   14 个固定 getter 帧。遇到异常或响应不完整时立即停止，不重试；它不发送模式、使能、
   回零、运动或夹爪控制帧，也不验证 safe-hold 或物理硬限位。
3. 这些字段形成证据后，再完成独立动作目标审查、正式转换/统计量和 SafetyBridge
   单步验收。在此之前 `PiperCanTransport.send()` 只能保持拒绝式实现，不开放
   CAN 命令发送或 `HIL_SINGLE_STEP`，也不发送 policy-originated CAN 帧。

2026 年 10 月 6 日新增的离线门禁要求 `live_shadow.py` 在打开相机或 CAN 前，取得
当前 Windows camera `InstanceId` 并要求操作者显式确认 Base/Wrist 画面角色；启动报告
会记录该会话绑定。PnP 存在性本身不能把设备 ID 自动映射到 OpenCV index，因此拔插或
重启后仍须重新目视确认。该门禁不会修复或推断当前相机帧率。门禁新增后，笔记本
`tests/contracts`、`tests/laptop`、`tests/data` 共通过 312 项、跳过 3 项和 58 个
subtests；服务器现有 checkout 的 61 项测试也已重新通过。当前 GPU1/8003 再次返回
合成 `16×7` proxy，`POLICY_REJECT_ONLY` 按预期拒绝，CAN TX 为 0；这不是实机 shadow。
证据见 [`non_can_readiness_recheck_20261006.json`](../../validation/non_can_readiness_recheck_20261006.json)
和 [`server_current_synthetic_reject_followup_20261006.json`](../../validation/server_current_synthetic_reject_followup_20261006.json)。

## 2026-10-06 非 CAN 链路复核

本节记录本轮复核的最新状态；它补充而不改写此前的日期化验证快照。

- Windows 专用 Conda 环境的完整本地离线套件通过 345 项、跳过 4 项，另有
  63 个 subtests。当前测试共 59 个模块，按 laptop、data、server 和 contracts
  分组；上游 OpenPI 与 5 个要求服务器 OpenPI/LeRobot 环境或外部 fixture 的
  模块不在本地运行范围内。
- RTX6000 服务器 checkout `ac0e8b7` 的 `.venv` 全套 `pytest tests` 通过 61 项，
  其中新的 `tests/server` 分组通过 9 项。`8002` 和 `8003` 仍在 loopback 监听；
  本轮通过临时隧道向 `8003` 发送合成输入，获得
  有限 `16×7` 预测，随后由 `POLICY_REJECT_ONLY` 拒绝，CAN 数据发送为 0。
  隧道已关闭；没有启动训练或重启服务。远端工作树仍为 dirty，未修改源码。
- 当前相机报告中的 OpenCV index `1/2` 各约 `0.99 FPS`，且画面几乎全黑。
  提权后的只读 PnP 清单显示 ACER 与 Dabai 均为 `OK` 且 `ProblemCode=0`，
  两者使用 `usbvideo`；另有 `USB Camera` 为 `Unknown`。未发现 PnP 错误，
  但清单不建立 OpenCV 索引到设备的映射，也不证明图像流可用。
  10 月 5 日经画面确认的 Base/Wrist 约 `30 FPS` 历史记录仍有效；当前结果
  不能证明相机硬件只有 `1 FPS`，也不接受为录制或 shadow 输入。
- NVIDIA Broadcast 正在运行，DirectShow 注册了其虚拟相机源。临时停止
  Broadcast 主程序和 UI 后，index `0/1` 仍无法组成双流，index `1/2` 的
  有界读取超时；应用已恢复。A/B 结果没有证明 Broadcast 是根因，当前 camera
  pipeline 仍不可用。记录见
  [`camera_broadcast_ab_diagnostic_20261006.json`](../../validation/camera_broadcast_ab_diagnostic_20261006.json)。
- 完整证据见
  [`non_can_readiness_supplement_20261006.json`](../../validation/non_can_readiness_supplement_20261006.json)。

下一步仍需先从实际可见画面重新绑定 OpenCV 索引与相机身份，
并复测实际双路帧率。PiPER 仍须等现场重新通电连接后，按只读反馈、固件和
单位/限位/夹爪/safe-hold 门槛继续；当前 server prediction-only 通过不等于
实机 shadow、受监督控制或 closed-loop rollout 已通过。
