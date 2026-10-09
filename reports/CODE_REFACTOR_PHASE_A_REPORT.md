# Code Refactor Phase A Report

2026-10-08。Pure-Core Extraction / Behavior-Preserving Refactor。
结论：本阶段软件验收完成；未启动硬件、训练、服务，未自动进入Phase B。
文档authority体系和目录保持原样。本报告按docs-writer规范组织证据，不再重构文档。

## 1. Scope

局部修改10份现有产品源码，新增3个公共模块；新增1份golden测试模块与2份静态fixture。
另有1份仅离线测试runner。产品源码总计涉及13份，既有用户修改以当前文件为基线保留。

- 单位authority仍为 `laptop/piper/unit_conversion.py`，增加保持原算式的标量API。
- 单帧feedback content与SDK已解析内容projection放到 `robot_client/feedback.py`。
- observation builder放到现有 `client/observation_schema.py`，不创建平行schema。
- 相机worker公开到 `camera/worker.py`，旧名称保留alias。
- identity验证/prewarm移到 `client/policy_client.py`，live_shadow保留兼容导出。

## 2. Explicit non-goals

没有更改control、scheduler、safety semantics、训练、模型/数据语义和硬件行为。

未改action horizon/dim、state/image/prompt顺序、RGB语义、CAN ID/encoding、授权、限位、
60°默认值、250ms等时效检查、稳定性检查、等待到位、重开CAN、sleep、频率、checkpoint
选择。未访问相机/USB/CAN，未执行mode/enable/replay/restore/HIL，未启动训练或生成
项目dataset，未改checkpoint或OpenPI upstream，未reset/clean/delete/archive用户文件。

## 3. Baseline

基线来自当前工作树，不是仅从HEAD恢复旧文件。

- Branch：`main`。
- HEAD：`99ab21c9ebbe1048c66c54880af1b9a7e2a12382`。
- 完整modified/untracked清单、git diff/stat/name、271个source/config哈希和send AST
  保存在 [BASELINE](CODE_REFACTOR_PHASE_A_BASELINE.md)。
- Git对部分旧validation pytest缓存报Permission denied，保留警告，未删除/修权限。
- 修改前安全测试：132 PASS，23 subtests PASS，见 [before log](phase_a_tests_before.log)。
- 当前旧实现快照在 `tests/fixtures/phase_a_before.json`；测试只编译需要的纯函数，
  不调用旧worker/HIL。修改前静态inventory见 [INVENTORY](CODE_REFACTOR_PHASE_A_INVENTORY.md)。

## 4. Unit conversion consolidation

从active runtime和纯数据/诊断辅助调用移走11处内联算式，保持旧运算顺序、round和类型。

| Old location | Expressions removed | New authority |
|---|---:|---|
| live_shadow.select_observation | 2 | joint_raw_to_nominal_rad / gripper_raw_to_nominal_m |
| bounded_model_trial.worker | 6 | joint raw→rad两处、gripper raw→m两处、rad→raw及m→raw |
| probe_piper_feedback_ack_only | 1 | joint_raw_to_nominal_rad，保留round(...,6) |
| align_raw | 1 | joint_raw_to_nominal_rad |
| verify_source | 1 | joint_raw_to_nominal_rad |

新函数在同一module：`joint_raw_to_nominal_rad`、`nominal_rad_to_joint_raw`、
`gripper_raw_to_nominal_m`、`gripper_raw_to_nominal_m_divided`、`nominal_m_to_gripper_raw`。

现有NumPy SDK API没有改，因为它们有可配置scale、np.rint、int64及不同异常/溢出处理。
实时Python scalar的round保留Python ties-to-even。gripper乘1e-6与除1e6保留两个明确
变体，不把浮点运算顺序差异当成可以强制统一。nominal meter仍不是机械jaw标定。

剩余位置逐项分类见 [REMAINING_CONVERSIONS](CODE_REFACTOR_PHASE_A_REMAINING_CONVERSIONS.md)
及 [STATIC_CHECKS](CODE_REFACTOR_PHASE_A_STATIC_CHECKS.json)。
`next_state_proxy`与command-target scale是INTENTIONAL语义常量；旧shadow preview为
HISTORICAL；协议/safety raw解析为PROTOCOL_REQUIRED；测试显式expectation保留。
时间单位换算也标INTENTIONAL，不误报成机器人单位重复。本次扫描没有剩余的
active realtime手写相同关节/gripper转换；不追求数字字符串为零。

## 5. Feedback decode consolidation

共收敛3个调用位置：session的raw内容解析进入公共decoder；shadow与probe的2份
SDK字段projection合并为1份。不是把3种不同accumulator强行合成同一运行状态。

| Old implementation | New authority | Preserved boundary |
|---|---|---|
| PiperSession status/joint/gripper内容解析 | decode_feedback_frame | session保留_latest、freshness、mode/enable/时间与snapshot聚合 |
| live_shadow SDK message projection | sdk_feedback_snapshot | 官方DecodeMessage及其累积语义、诊断计数、锁不变 |
| feedback probe SDK message projection | sdk_feedback_snapshot(gripper_details=True) | enum字符串、gripper effort/status日志、passive capture结构不变 |

新 `DecodedPiperFeedback` 仅表示单帧ID、arrival时间与内容，不代表完整六轴snapshot。
passive_capture本来组装已解码日志，未修改。SafetyBridge里的授权相关解析未动。
没有替换官方SDK parser，没有将frame filtering/线程/IO/对齐/freshness塞进decoder。

Golden fixture来自既有 `datasets/raw/0cd272a037f0489bbebaad76f45287c3/feedback_can_raw.jsonl`，
安全抽取5类反馈帧到 `tests/fixtures/phase_a_recorded_feedback.json`。所有raw count精确相等；
另测signed32/signed16边界、状态错误字段、unknown ID与官方SDK累积projection。

## 6. Observation consolidation

shadow、bounded和warmup共3个builder调用收敛到现有schema的 `build_piper_observation`。

| Old | New | Behavior |
|---|---|---|
| select_observation手工dict | builder | BGR→RGB仍由caller各转换一次；随后保留原validator |
| bounded worker手工dict | builder | float32[7]；不新增原先不存在的validation gate |
| proxy warmup手工dict | builder | 224×224零RGB、零float32状态与原prompt |

builder只打包already-RGB、state与prompt，不做时效、skew、clipping、device IO。
验证authority仍是 `validate_piper_semantic_observation`，未改变其拒绝规则。
用np.array_equal比较旧/新图像和状态，比较keys、dtype、shape、prompt与RGB通道。
server evaluation builder及OpenPI mapping/transforms保持原样，避免改变训练dtype/adapter。
其余historical/synthetic/ALOHA观察不是当前PiPER runtime builder的等价副本，保留。

## 7. Camera worker

完成公开worker，`camera.worker.camera_loop`的函数体AST与原_camera_loop精确相同。

- bounded_model_trial改用公共worker，不再import live_shadow任何helper。
- formal supervised_live_step改用公共camera worker；其CAN worker依赖仍保留原路径。
- live_shadow保留 `_camera_loop = camera_loop`兼容导出，原shadow入口仍正常。
- CAP_DSHOW/open_camera、resolution/FPS行为、perf_counter_ns时间源、buffer替换、
  callback、锁、异常信息、stop与release全部保持。

用fake opener、固定时钟与deterministic RGB对比旧/新成功和失败路径。
没有异步video writer/flush，没有改变camera启动时间或录制时序。

## 8. Policy client / identity

原先identity/prewarm只有一份定义但由多个入口借用live_shadow；现在移到公共模块，
保持函数体与常量，live_shadow继续兼容导出。bounded与live_model_preview直接使用公共API。

WebsocketClientPolicy constructor/host/port/协议/序列化/SSH假设/timeout：**NO CHANGE**。
latency smoke构造和warmup场景不同，不强制合并。服务prediction_only定义未改变。
validator/prewarm迁移通过旧函数oracle逐字段metadata/lock变异接受拒绝及异常信息比较，
覆盖旧proxy与derived multi identity，且只有fake client，不建立网络连接。

## 9. Regression tests

测试runner在真正硬件边界显式拦截cv2.VideoCapture、真实_GsUsbReceiveAdapter.open_read_only
和WebsocketClientPolicy构造。所有实际发送测试仅使用fake/in-memory transport。

| Test file | Result | Evidence scope |
|---|---|---|
| tests/contracts/test_piper_units.py | PASS | SDK NumPy 10000值roundtrip/scale |
| tests/laptop/test_phase_a_pure_core.py | 57 PASS | old/new精确算式、整数、CAN fixtures、完整snapshot、观察、identity、prewarm、fake camera、worker AST |
| tests/laptop/test_live_shadow.py | PASS | pure metadata/timing/observation与mocked启动边界 |
| tests/laptop/test_piper_session.py | PASS | injected fake bridge，反馈/日志/错误/不重试 |
| tests/laptop/test_piper_feedback_ack_only_guard.py | PASS | fake transport/read guard/raw日志，不开USB |
| tests/laptop/test_bounded_policy_target.py | PASS | 原clipping/步长与invalid处理 |
| tests/laptop/test_bounded_model_trial.py | PASS | in-memory exact TX frames，授权、故障与step范围 |
| tests/laptop/test_safety_bridge_runtime.py | PASS | fake transport授权/时效与拒绝 |
| tests/laptop/test_offline_safety_bridge.py | PASS | pure离线guard/contract |
| tests/laptop/test_supervised_position_bridge.py | PASS | MemoryPositionAdapter，formal授权与encoding |
| tests/laptop/test_supervised_policy_session.py | PASS | MemoryPositionAdapter，formal grant/helper |
| tests/laptop/test_shadow_preview_smoke.py | PASS | fake socket/policy/静态dataset fixture读取 |

最终合计 **202 PASS，23 subtests PASS**，见 [final log](phase_a_tests_final.log)。

首轮修改后出现一次1ms实时poll测试波动：期望11帧只读到3帧。
poll方法AST没有改变；未放宽生产阈值，单独复查4/4通过，最终全组也通过。
原失败日志保留 [initial after](phase_a_tests_after_initial.log)，复查日志
[session recheck](phase_a_session_recheck.log)。新snapshot golden使用固定时钟，避免这类窗口依赖。

## 10. CAN TX authority check

发送调用按AST接收者与源文件比对，新增pure module不含send。

- 全源码`.send(...)` callsite：23 → 23（包含IPC与fake测试，不全是CAN）。
- actual device.send callsite：11 → 11。
- active真实backend仍为一个gs_usb adapter，4个device.send位置；7个legacy direct位置
  仍guard阻断，源码未改，未归档。
- PiperSession poll/send_bounded_model_trial/wait_joint_target/commissioning_snapshot
  的方法AST不变；SafetyBridge与所有transport原字节未变。
- 没有新增direct sender，也没有平行Safety/transport。

所有actual direct send位置前后相同如下。

| Path | Line | Receiver | Status |
|---|---:|---|---|
| `laptop/robot_client/supervised_j1_one_degree_trial.py` | 73 | `device.send` | legacy guarded |
| `laptop/robot_client/supervised_j1_step_trial.py` | 119 | `device.send` | legacy guarded |
| `laptop/robot_client/supervised_j5_probe.py` | 59 | `device.send` | legacy guarded |
| `laptop/robot_client/supervised_joint_replay.py` | 121 | `device.send` | legacy guarded |
| `laptop/robot_client/supervised_restore_gripper.py` | 79 | `device.send` | legacy guarded |
| `laptop/robot_client/supervised_restore_start.py` | 139 | `device.send` | legacy guarded |
| `laptop/robot_client/supervised_smooth_joint_replay.py` | 231 | `device.send` | legacy guarded |
| `laptop/transport/gs_usb_piper_can.py` | 245 | `self._device.send` | active typed backend |
| `laptop/transport/gs_usb_piper_can.py` | 255 | `self._device.send` | active typed backend |
| `laptop/transport/gs_usb_piper_can.py` | 264 | `self._device.send` | active typed backend |
| `laptop/transport/gs_usb_piper_can.py` | 274 | `self._device.send` | active typed backend |

完整23个callsite与前后行号均保存在STATIC_CHECKS JSON。
0x150/151/155/156/157/159/471定义、encoder与typed transport均在未修改的源码中。

## 11. Behavior differences

在测试范围内，数值、异常类型/信息、payload内容、typed proposal/command整数、
观察结构/通道/状态、identity接受拒绝与worker控制逻辑差异：**NONE OBSERVED**。

预期变化仅是函数定义所在模块、源码行号和helper import路径；旧import兼容导出保留。
这不是对所有未知输入或真实硬件性能的证明。抽离改变少量Python调用成本，未测实机，
未据此改变任何阈值。SafetyBridge的physical hold能力仍未验证，本阶段不宣称新增安全性。

## 12. Remaining technical debt

以下全部记录为FOLLOW-UP ISSUE，本轮未修复。

- scheduler、proper chunk consumption、smoothing/temporal ensemble、continuous session。
- independent continuous watchdog与物理safe-hold验证。
- synchronous video writer/flush造成camera loop阻塞、双路稳定性。
- gripper机械行程标定；nominal feedback≠command supervision。
- CLI/worker/helper默认步长差异。
- latency smoke timeout与通用client边界。
- live_model_preview仍使用live_shadow._worker；formal入口仍使用_can_loop。
- 原1ms wall-clock session test易受机器调度影响；未更改测试或生产poll阈值。
- 原training/provenance会绑定source hashes；部署新源码须生成/核对独立运行身份，
  不篡改历史报告或沿用旧哈希伪装成同一运行版本。本轮没有上传或更新服务器runtime。

## 13. Files changed

以下逐文件列出产品源码与测试；所有产品function-level变动都局限于纯helper抽离。

| Path | Reason | Behavior change? |
|---|---|---|
| `laptop/client/bounded_model_trial.py` | public helper imports；转换与观察调用切换，worker AST展开等价 | NO (verified scope) |
| `laptop/client/live_model_preview.py` | identity loader改用公共module；_worker不动 | NO (verified scope) |
| `laptop/client/live_shadow.py` | camera/policy兼容alias、builder与SDK字段projection | NO (verified scope) |
| `laptop/client/observation_schema.py` | 同一schema内新增builder，warmup复用 | NO (verified scope) |
| `laptop/client/supervised_live_step.py` | public camera worker，其他私有CAN helper保留 | NO (verified scope) |
| `laptop/diagnostics/probe_piper_feedback_ack_only.py` | SDK字段projection与原算式helper | NO (verified scope) |
| `laptop/piper/unit_conversion.py` | 新增精确scalar变体；原NumPy API保留 | NO (verified scope) |
| `laptop/recorder/align_raw.py` | 原joint算式helper | NO (verified scope) |
| `laptop/recorder/verify_source.py` | 原joint算式helper | NO (verified scope) |
| `laptop/robot_client/piper_session.py` | 纯内容decoder；freshness/aggregate/IO不动 | NO (verified scope) |
| `laptop/camera/worker.py` | 新增公共worker，旧函数体不改 | NO (verified scope) |
| `laptop/client/policy_client.py` | 新增identity/prewarm authority，函数体移出 | NO (verified scope) |
| `laptop/robot_client/feedback.py` | 新增纯单帧内容和SDK projection | NO (verified scope) |

| `tests/laptop/test_phase_a_pure_core.py` | 新增57项golden/static测试 | NO runtime behavior |
| `tests/fixtures/phase_a_before.json` | 开始时源码oracle | NO |
| `tests/fixtures/phase_a_recorded_feedback.json` | 既有5类CAN反馈静态样本 | NO |
| `reports/phase_a_pure_test_runner.py` | 阻断实际device/network，仅运行explicit测试清单 | NO production behavior |

报告及log文件仅新增，不覆盖已有产物。未修改其他未提交文件、config、server、legacy
sender、SafetyBridge、transport、control/contracts或checkpoint。

## 14. Recommended Phase B

建议可进入独立审批的Phase B设计与离线验证：连续action调度/时间语义、chunk消费、
平滑及watchdog。需要单独定义验收与后续实机授权，不能借Phase A新增控制策略。
**本阶段停止，等待用户确认，不自动进入Phase B。**

## Acceptance checklist

验收仅限本轮软件证据，硬件禁止项均未执行。

- [x] unit conversion authority明确；active runtime复用精确helper。
- [x] feedback decoder与aggregation边界明确。
- [x] observation builder与原validation authority明确。
- [x] private camera helper跨入口依赖移除；旧alias保留。
- [x] observation/action semantics、bounded worker控制AST不变。
- [x] SafetyBridge/transport与训练、config原字节不变。
- [x] direct CAN sender/backend数量不增加。
- [x] 已审查pure/fake regression最终202项、23子测试通过。
- [x] 未运行硬件、训练，未改checkpoint，未生成项目dataset。
- [x] 未大规模目录迁移，legacy源码全部保留。
- [x] baseline、inventory、golden、静态扫描和最终报告完成。
