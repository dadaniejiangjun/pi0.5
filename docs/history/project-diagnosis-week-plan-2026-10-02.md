---
Document status: HISTORICAL
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

> **HISTORICAL**：以下正文保留当时计划、决策或假设，不描述当前部署。pending/未实现/未选择等状态仅限原日期，不能覆盖新的运行证据。 当前领域主文档见 [authority](../CURRENT_STATUS.md)。

# PiPER × π0.5：代码诊断与维修等待期工作计划

审查日期：2026-10-02。范围：笔记本当前工作树、通过 SSH 读取的服务器工作树、
现存原始录制、纯离线测试。没有连接相机或 CAN，没有运行训练或实机动作。
本报告存放在项目 docs 目录；不包含原始影像、密钥或硬件日志。

更新：同日已实现 `laptop/recorder/align_raw.py`，补上本报告第 1 项指出的
raw → aligned 缺口。合成数据已经走通视频/反馈 → 对齐 → LeRobot 读回；
现有失败轨迹生成了 200 帧本地对齐预览，仍不可用于训练。
以下诊断段落保留修复前的发现，其他缺口和七天计划仍待执行。

## 结论

项目已拥有可运行的软件部件，但尚未形成完整的数据到执行链路。
最主要缺口是原始录制到对齐训练样本、按实验用途区分的训练入口、以及实际
SafetyBridge。等待硬件维修的一周可以完成这些软件工作的大部分，并建立可复现的
离线实验。物理关节方向、夹爪映射、相机曝光同步和硬件停机行为仍须修复后验证。

## 本轮验证

- 笔记本 pi05_laptop Conda 环境：30/30 离线测试通过。
- 服务器现有隔离 .venv，禁用 GPU、开启离线模式：22/22 测试通过。
  覆盖 OpenPI 数据转换、锁定入口、合成 LeRobot 写入/读回、世界模型时间/姿态契约。
  LeRobot 编码过程出现 ResourceWarning，未导致测试失败。
- OpenPI 子模块仍为 215abfb217dbac7d5f1273282331b9b1866c0479。
- 笔记本主仓库 HEAD：99ab21c9ebbe1048c66c54880af1b9a7e2a12382。
- 服务器主仓库 HEAD：ac0e8b731168d07dbd417c0c43a3b09253aabb71。
- 双端 convert_piper_aligned.py、piper_openpi_config.py、piper_train_entry.py
  的 SHA-256 一致，但两端都有大量未提交代码，世界模型文件主要在服务器。
- 没有安装或升级依赖。后续新增依赖按用户要求只进入专用 Conda 环境。

测试通过说明现有断言通过，不能推出真实数据已能训练或机器人可执行。

## 实际代码发现

### 1. 原始数据与转换器之间存在缺失步骤

修复状态：已补上可执行对齐器和相关测试。现有失败轨迹仅生成预览；
正式转换仍需单独确认关节/夹爪单位、任务结果及动作目标。

录制器保存 MP4、各通道 JSONL、事件和 manifest；转换器从
aligned_episode.json、frames.jsonl 和 RGB 图片开始。项目中只有测试 fixture
生成后者，没有实际 raw → aligned 的抽帧、重采样、动作标签构造工具。

证据：laptop/recorder/episode.py；
server/dataset_tools/convert_piper_aligned.py:89；
tests/server/test_piper_aligned_converter.py:21。

建议：补全这个中间步骤，保存每一帧的原始来源、对齐策略、裁剪边界、动作时间偏移。
可将未来的示教关节位置作为明确声明的模仿学习代理目标进行离线试验，但必须标明
它来自反馈而非操作者指令；其时间定义与真实控制器响应以后单独验证。
夹爪单位/目标未知时保持缺失，不能用任意常数伪造完整 7D 示教。

### 2. 训练入口按设计永久拒绝执行，门槛也未区分实验类型

piper_train_entry.py:49 的 execute_training() 无条件抛 PermissionError。
check_piper_readiness.py:38 要求至少 20 条成功示教，因而没有单轨迹离线过拟合入口。
这是过去“接通但锁住”的实现结果，继续推进必须修改入口，而不是重复采集后重试。

建议：分别定义 synthetic/debug、public-data、local-tiny-fit 和正式训练模式。
每个模式记录数据身份与动作语义；离线训练权限和硬件控制权限独立。
一条质量合格轨迹可用于 tiny-fit，20–50 条属于后续 pilot 的采集目标。
失败轨迹保留失败标签，可用于数据通路/失败分析，不能标成成功示教。

更新（2026-10-02）：训练入口现按上述四种模式分别检查数据身份、
动作语义和条数。单条合格成功示教只可进入离线 tiny-fit；正式训练仍
要求至少 20 条。执行需要显式开关、基础权重和阶段化归一化统计，
不授予硬件控制权限。当前失败轨迹仍被拒绝。服务器 OpenPI 环境的
运行时验证尚未完成，不能把此代码修改等同于已开始训练。

### 3. 实机安全桥尚未实现

更新（2026-10-02）：已加入离线纯函数校验、
`FakeRobotTransport` 和无发送 JSONL 回放客户端，并通过故障注入测试。
这不是已接入真机的 `PiperSafetyBridge`。夹爪实机限位、
急停/保持/超时语义及物理动作验收仍须维修后现场核实。

原诊断时，laptop/safety/ 和 laptop/robot_client/ 只有 README。
laptop/client/public_piper_transport_smoke.py:12 的 offline_veto 是始终拒绝的
smoke 辅助函数，不检查完整的状态新鲜度、响应顺序、限位、速度、加速度和 watchdog。

建议：本周实现纯函数验证、FakeRobotTransport 和无发送的回放客户端。
通过丢包、乱序、超时、NaN、单位错误、超限等故障注入验证拒绝行为。
真实急停/保持动作语义仍留待维修后确认。

### 4. 数据来源记录已存在，但尚未验证到每个原始样本

更新（2026-10-02）：已实现逐样本文件来源复验：从原始 JSONL 重新生成
来源映射、校验数值单位转换，并逐帧比较原 MP4 解码结果与导出 PNG。
已有失败实机轨迹的 200 条对齐样本及 400 帧图像通过只读复验，
仍标记为不可训练。另有 episode 级划分、训练集专用统计量及
数据/统计量/检查点来源声明哈希绑定工具；合成端到端测试通过。
相机曝光同步、人工动作语义、真实训练运行与物理成功仍未由文件哈希证明。

转换器校验原始文件哈希、引用字段格式和时间差；它不逐个核对引用时间戳是否
出现在原始 CAN 日志中，也未核实导出 RGB 就是指定原始视频帧。
代码明确输出 source_frame_extraction_verified=false、training_eligible=false。
这是尚待实现的来源验证，不应被“存在 provenance 文件”掩盖。

证据：convert_piper_aligned.py:162、:238、:271。
建议把原始帧提取、数值转换和引用检查放到可重跑的工具中，另加 episode 级划分与
仅使用训练集计算统计量的流程，并校验数据/统计量/检查点哈希对应关系。

### 5. 录制的限时退出与身份记录需补强

probe_dual_cameras.py:117 的 read() 在工作线程阻塞，:144 使用无超时 join；
passive_capture.py:110 同样无超时等待。设备驱动若卡住，“20 秒”不保证按时退出。
这属于静态可见风险，本轮没有人为挂起驱动复现。

相机报告记录 index，而非稳定设备身份；拔插后的角色映射需要再次核对。
示教入口以 success=None 结束，缺少录后追加任务结果的正式流程。
建议增加可终止的采集工作进程、收尾标记、设备身份、独立结果标注文件。
原始已校验文件保持不变。

### 6. 公开数据 smoke 与本机动作契约必须明确隔离

validation/public_piper_smoke.json 记录 294 episodes / 61,709 frames，
统计量仅取 128 帧，训练 2 steps；数据使用 degree + binary gripper。
本机契约是 radian + meter。相同的 7D shape 并不意味着可直接互换。
两步 loss 下降只说明训练代码运行；合成输入首次关节变化 44.6126°不能解释为
实机关节误差，更不能由此推断某个关节损坏。

建议完整的数据卡/单位/夹爪定义核对后，单独做公开数据离线 tiny-fit 与留出集评估。
新检查点保持 prediction-only，直到本机映射与实机安全验收完成。

### 7. 世界模型已有预测器试验，最新状态是复现失败后暂停

服务器 reports/OFFLINE_REPLAY_WM_MVP.md 的最新追加记录说明：
V-JEPA 2 冻结特征 GPU 推理、仿真状态/动作与视觉预测器都做过试验。
最新 94 条测试 episode 的单种子结果为：

| 方法 | 位置 RMSE | 姿态误差 |
|---|---:|---:|
| B：状态与动作 | 57.62 mm | 2.289° |
| C：视觉、同样状态与动作 | 43.56 mm | 2.900° |

增加视觉在该次试验改善位置但恶化总体姿态。早期 inline pilot 未保留完整执行协议，
后续 runner 未复现历史结果；最新状态 STOPPED_REPRODUCTION_MISMATCH。
source 数据是预装 peg 的仿真插入，不包含真实触觉、失败 episode 或真实手指抓取。
所有上述数值是本轮读到的已有报告，本轮仅重跑相关单元测试。

建议保存现在的执行协议、特征缓存、训练检查点及重载评估入口，在新实验 ID 下
建立可复现基线，不再要求追平无法充分追溯的历史数字。
优先比较信息一致的 B/C；GT 姿态 persistence 只作特权参考。

## 七天工作安排

每天约 4–6 小时，主要依靠已有数据；维修和配件到货不作为前六天软件工作的依赖。
下面是待执行计划，不代表本轮已经实现。

| 天 | 工作 | 当天交付与验收 |
|---|---|---|
| 1 | 整理两端版本、环境和实验身份；保留全部未提交改动；固定公开/本机/仿真三类数据语义 | 一份代码与环境快照、统一状态表、可重复运行的离线测试命令；无覆盖或自动 push |
| 2 | 实现 raw → aligned：抽帧、共同时间窗裁剪、通道重采样、来源引用；添加结果标注 | 真实失败轨迹可回放和绘曲线，5 个边界帧有明确处置；合成轨迹验证已知偏移与缺失帧；不把未知夹爪变成有效标签 |
| 3 | 分离离线实验/正式训练/实机执行权限；实现 episode 划分与 train-only 统计量绑定 | 合成完整输入经过 converter → LeRobot → OpenPI；篡改来源、NaN、统计量错配、split 重叠均被拒绝 |
| 4 | 用公开数据固定小子集做 100–500 update tiny-fit，再做小规模 episode 留出评估；单 GPU、先检查资源 | 保存 config/seed/split/norm/checkpoint 和重载结果；按关节/夹爪报告误差，与持有当前状态基线比较；若 tiny-fit 不下降先排查，停止加量 |
| 5 | 实现 SafetyBridge、FakeRobotTransport 和无发送的观测回放 | 一条命令串起已有观测 → 策略 → 校验 → 记录；错误单位、旧响应、断网、NaN、越限全部拒绝；无 CAN 发送路径 |
| 6 | 给世界模型新建可复现基线，缓存冻结特征、保存模型与样本列表；小预算复跑 B/C | 同配置重复/重载结果在预设容差内；分别报告位置和姿态；仍无改善则保存负结果，不扩模型 |
| 7 | 进行离线交付演练；为 VTLA 添加可选触觉字段与 validity mask 的接口设计；准备维修后测试表 | 复现实验清单、离线演示和明确的硬件待验证项；缺失触觉显式标缺失，不能作为真实零值或触觉收益证据 |

第 1–5 天优先，世界模型工作如超时顺延，不阻塞主线。
本周完成标准：现有数据能够回放、转换、训练和预测；预测经过可测试的安全桥；
关键实验可从保存的配置和检查点复现。实机操作成功与 VTLA 触觉收益不在本周离线验收范围。

## 硬件简短建议

线缆已断且夹爪不响应，保持断电并保存断口/接头/原接线位置照片，待专业人员核对
线序、供电和驱动。缺少烧痕不能确认或排除电气损坏。不要通过反复使能、归零、
随机互换关节线来定位故障。

单关节反向先记录关节编号，利用旧录像比较物理方向、角度数值与模型显示方向。
修复后在厂家确认的流程下复查。不要先在代码里乘 -1 或重新写零点。
官方 SDK 的 ResetPiper 会立即失能并可能使机械臂下落，不能当成普通软件刷新。
官方参考：https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD

## 本轮可重复验证命令

笔记本在项目根目录执行以下纯离线测试，测试中的相机/CAN 调用已由 mock 或默认拒绝保护。

```powershell
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B -m unittest tests.test_piper_contract tests.test_piper_units tests.test_can_ack_only_guard tests.test_can_listen_only_guard tests.test_piper_feedback_ack_only_guard tests.test_dual_camera_probe tests.test_episode_recorder tests.test_feedback_log_audit tests.test_passive_capture -v
& 'E:\anaconda3\envs\pi05_laptop\python.exe' -B -m laptop.recorder.audit_episode datasets/raw/a6201f55139a44e6ae513d453349150b
```

第二条本轮返回 INCOMPLETE：Base 604 帧、Wrist 600 帧，分别有 2/3 帧超出完整状态窗，
以及 validated action target absent。完整状态窗约 19.9907 秒；双相机主机到达时间
偏差中位数约 7.564 ms。这些时间指标不证明曝光同步。用户反馈任务失败，但原始
events.jsonl 中 success 仍为 null，后续应通过独立标注文件保存用户反馈。

服务器在 /mnt/ssd/users/mingkai/07_pi0.5 使用已有环境执行；本轮没有更改依赖。

```bash
env CUDA_VISIBLE_DEVICES=-1 JAX_PLATFORMS=cpu HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 .venv/bin/python -B -m unittest tests.test_piper_openpi_offline tests.test_piper_train_entry tests.test_piper_aligned_converter tests.test_offline_replay_wm_mvp -v
```
