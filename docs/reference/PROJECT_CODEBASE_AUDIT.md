---
Document status: REFERENCE
Current authority: docs/CURRENT_STATUS.md
Last reviewed: 2026-10-08
---

> **REFERENCE**：这是文档收口前的2026-10-08审计快照；代码结论保留，文档authority调整以 DOCUMENTATION_CLEANUP_REPORT 为准，不重写当时证据。 当前领域主文档见 [authority](../CURRENT_STATUS.md)。

# PiPER π0.5 Project Codebase Audit

本报告恢复实际架构、核对数据和控制语义，并提供以后整理仓库的证据清单。
审计日期：2026-10-08，Asia/Singapore。以本地工作树为主要证据，服务器只读补证。
本轮仅创建本报告与 REDUNDANCY_CANDIDATES.md；不改变程序、配置、Git或机器人状态。

## 1. Executive Summary

当前项目已经拥有可采集、可训练、可推理、可执行受监督动作的实验链路，尚无自主抓取成功证据。

- 附件中的“闭环尚未接通”已过时：bounded_model_trial.worker已把实时模型输出经Bridge和CAN发送给PiPER，并读回反馈。
- 这是受监督实验闭环，不能称为抓取部署完成；运行记录task_success_claim=false，最新5000步长试跑提前停止。
- 当前数据是3条人工辅助成功示教、595个10fps样本；1000步和5000步训练都完成。39帧/2fps/单episode是历史实验。
- 输出仍是后续实测位置的代理目标，名义radian×6 + meter，不是原厂示教命令标签、速度、增量或力矩。
- 当前执行使用commissioning/BoundedTrialBatch专用通路；正式P12/CommandProposal通路另存，验收口径不同。
- 按独立控制实现模块计，有10个条件式控制路径：8个位置/夹爪目标路径、2个模式/使能路径；另有7个旧raw TX实现被入口guard阻断。
- 新路径实际设备发送集中在gs_usb_piper_can后端；对象/线程owner没有形成跨进程设备锁。
- 长循环逐步等待到位和静止、重开CAN，只消费16帧中的一帧，不是持续10Hz轨迹控制器。
- 0.05°/300ms稳定性与0.5°到位容差并存，能导致“到位但下一步停”；同步写视频和flush会拖慢相机读取。
- 抽查5个关键训练/服务文件与服务器runtime_root哈希一致，不能据此声称全仓同步。
- README和多个初始状态文档滞后；实际进度由plan最新条目与不可变运行报告共同证明。
- 可开始状态文档、入口登记和来源整理；不适合批量搬移控制代码或清除历史证据。

## 2. Repository Scope and Audit Method

审计区分逐文件静态检查、重点代码审阅、既有实测记录和本轮实时只读证据。
第23节给出确切文件数、分类数和逐文件清单，核心文件计数限定为CORE_ACTIVE。

| 范围 | 核验与限制 |
|---|---|
| 本地 | ACCESSIBLE：E:/01_NUS/02_5003/12_pi0.5；laptop/server/scripts/configs/docs/tests/provenance/reports及第一方补充文本 |
| 服务器 | ACCESSIBLE：SSH rtx6000_Mingkai，/mnt/ssd/users/mingkai/07_pi0.5及cache/codex/new_demo_tinyfit_20261007_r1/runtime_root |
| 方法 | 文件清单、UTF-8读取、AST/import/call、文本引用、字节SHA-256、Git status/history/gitlink、JSON报告、只读SSH进程/元信息 |
| Git | main，HEAD 99ab21c9ebbe1048c66c54880af1b9a7e2a12382；origin github.com/dadaniejiangjun/pi0.5.git；105个tracked path |
| 审计前状态 | 22个modified、2个deleted、2277个untracked文件；部分pytest临时目录permission warning；这些改动均先于审计 |
| OpenPI | gitlink、provenance与服务器实际HEAD为215abfb217dbac7d5f1273282331b9b1866c0479；官方Physical-Intelligence/openpi submodule |
| 执行边界 | 未运行pytest或项目CLI；未连接CAN、发送TX、采集、请求模型推理、训练、启停服务或进行Git mutation |
| 逐文件范围 | 第一方源码/配置/文档、validation JSON和一次性脚本；第三方、venv/cache不作为第一方源码逐文件审计 |
| 大生成物 | raw/incoming/lerobot媒体、AVI/PNG/CSV/CAN JSONL按证据族保留；不逐帧展开，不全量重算checkpoint参数树哈希 |
| 证据等级 | 源码证明实现；report证明对应历史运行；本轮进程证明启动。任何一层都不独自证明现在任务成功 |

CLI无入站import仍可有效。mtime仅辅助；Git只有bootstrap历史，不能用它证明功能迁移。
完整文件清单不是声称每份大型运行报告都逐字段人工审读；关键路径、契约、发送点与异常分支重点审阅。

服务器本轮进程：1000服务PID2821626，GPU0/8020；5000服务PID2914575，GPU2/8021。
进程存在不代表已重新请求并验证服务响应，更不代表机械臂处于可运动状态。
两套checkpoint目录实际存在；dataset/norm/checkpoint树哈希引用训练report，未全量重新计算。
五个当前代码文件本地和服务器runtime_root SHA相同：train_demonstrations、proxy_multi_episode_pilot、
piper_openpi_config、serve_trained_demonstrations、serve_derived_state_multi_shadow。
服务器主目录与runtime_root不是同一个代码目录；审计不能把本机缺weights当成总体阻塞。

## 3. Current System Architecture

主链由Windows硬件/执行端、SSH网络通道及Linux OpenPI训练/推理端组成，没有ROS2运行主链。

```text
Windows laptop                             Linux GPU server
base+wrist UVC -> DSHOW/OpenCV              training report + dataset + norm
 -> camera/source -> _camera_loop            -> provenance/hash
candleLight -> GsUsb/PyUSB -> PiperSession    -> serve_trained_demonstrations
  | raw joints/gripper/status/motor flags     -> serve_derived_state_multi_shadow
  +-- observation RGB/state[7]/prompt --------> OpenPI inputs/normalization
                     SSH/WebSocket             -> Pi05 [16,32]
  <------------- proxy positions [16,7] ------- output slice [:7]
  -> bounded_policy_target.limit_target
  -> BoundedTrialBatch + commissioning snapshot
  -> SafetyBridge.send_bounded_model_trial
  -> GsUsbPiperCanTransport typed send
  -> 0x151 + 0x155/156/157 + 0x159 -> PiPER
  -> fresh feedback / arrival / next-step preflight

Separate formal branch:
PolicyPrediction -> preview + P12GateEvidence -> CommandProposal
 -> PositionCommandBatch -> supervised_position Bridge

Offline data branch:
passive capture -> label -> source/hash -> align -> next_state_proxy
 -> LeRobot -> norm -> OpenPI train -> same-training-episode eval
```

live_shadow自身无目标TX。bounded_model_trial复用其私有相机、预热和身份函数，
另外构造observation/控制循环。offline_guard不是当前长循环的在线保护器。

## 4. Current Verified Runtime Pipeline

VERIFIED指既有实测记录与实现对应，本轮没有重新连接硬件。
PARTIAL指只有部分行为得到验证；NOT VERIFIED指实现存在而证据不足。

| 能力 | 状态 | 证据与边界 |
|---|---|---|
| 双相机+PiPER反馈录制 | VERIFIED | 成功capture及3条训练来源；base=ACER HD User Facing，wrist=Dabai DC1；索引随枚举变化 |
| RGB/state/prompt→服务器预测 | VERIFIED | shadow与bounded trial记录；metadata/identity校验；16×7有限输出 |
| 模型→限幅→Bridge→CAN→反馈 | VERIFIED | task_trial1000_long20deg_nocumulative_20261008_r1.json与task_trial5000_60deg_20261008_r2.json有TX receipts及feedback match |
| 1000长试跑 | PARTIAL | 48个step记录、47个match，最后停止；不是抓取成功 |
| 5000长试跑 | PARTIAL | r2有24次更新/关节match，随后历史稳定性拒绝；base2048帧/wrist110帧 |
| 持续轨迹/chunk调度/平滑 | NOT IMPLEMENTED | active worker每轮用一行，等到位并重开CAN；legacy smooth replay被guard阻断 |
| 正式P12现场映射和safe-hold验收 | NOT VERIFIED | site profile template DRAFT；commissioning动作不是正式gate自动通过 |
| 自主抓黑棒并放绿垫 | NOT VERIFIED | task_success_claim=false；既有现场/视频说明未证明成功，本轮无新实测 |
| 曝光级同步/机械单位标定 | NOT VERIFIED | host arrival不是曝光同步；gripper机械尺度未建立 |

缺失闭环应写成持续调度、统一标定映射、停止/断联行为验收和独立任务评估，
不能继续写“SafetyBridge/CAN transport完全未实现”。

## 5. Training / Dataset Pipeline

当前可复用链录制独立示教并人工标注成功，不把一条复制成五条。

```text
record_demonstrations -> capture_session -> passive_capture
 -> raw videos + CAN/feedback + monotonic timestamps
 -> episode quality + label_episode
 -> verify_source + align_raw (default nearest host-arrival max skew 50ms)
 -> next_state_proxy (future observed state, +100ms)
 -> convert_next_state_proxy.convert_many -> LeRobot + source_map/hash
 -> proxy_multi_episode_contract + pilot.prepare
 -> official compute_norm_stats + norm provenance
 -> train_demonstrations -> pilot.execute -> official OpenPI train
 -> checkpoint + training_provenance
 -> evaluate_demonstrations -> pilot.evaluate (same training episode rows)
 -> serve_trained_demonstrations -> identity lock + WebSocket
```

post_capture_pipeline.prepare可复用；其train默认20步，上限100步。
长训练由train_demonstrations调用同一pilot core，当前本地和服务器允许1..50000步，
默认5000/min-episodes=3/GPU2/memory-fraction=0.95。
pilot.execute默认min-episodes=5，短pilot100步上限；调用方明确改变参数，不能混成一个默认标准。

| 当前数据/训练 | 实际依据 |
|---|---|
| Dataset | 服务器meta/info.json：local/piper_new3_20261007_r1，3 episodes，595 frames，10fps，train=0:3 |
| raw episodes | 0cd272a037f0489bbebaad76f45287c3；6a5a627971ee4347a0d3533fd7ce2921；992b4ed369c847bcad42f348470b791b；eval source rows可追溯 |
| dataset SHA（report） | 957953ede1bbeba09b265a984034ebc088f494a5d9cbe72d020dbeb39782f862 |
| norm SHA（report） | fd958229586a7affc87f6f8c9b7f8231de1bace4d1a808ad3062332c893bdc6f |
| 1000 steps | COMPLETE，piper_new3_1000step_r1，checkpoint .../999目录存在 |
| 5000 steps | COMPLETE，piper_new3_5000step_r1，checkpoint .../4999目录存在 |
| 5000 loss | validation/train5000_loss_20261008.json：5000 logged steps，最后0.0034，最后100步均值0.014172；仅训练loss |
| 1000 offline eval | server eval1000_report.json：同训练集15行，joint MAE名义0.0478098rad，gripper0.00234621m；没有held-out |
| 20步历史 | new_demo_tinyfit_20261007_corrected.json：同训练行joint MAE0.364068rad；不是20K训练 |
| 2fps历史 | 本地cef166e2_proxy2fps*/meta/info.json：1 episode/39frames；10fps历史版199frames |

pilot.evaluate每episode最多取5行，对预测第一行和对应future-state标签比较。
先对每行六轴绝对误差平均，再跨行平均；夹爪独立统计。它不是任务成功率或轨迹唯一正确性的判据。
成功轨迹可以多解，MAE不能独自判抓取失败；全train-only的低loss也不证明泛化。
失败episode可保留评估/研究用途；现有成功模仿prepare链筛选success标签，
不带reward/advantage或恢复目标直接混入失败动作，不自动提高泛化。

## 6. Core Code Map

表中区分复用core、当前入口、formal辅助与HIL工具；Caller表示源码或运行记录中的调用方。

| Path | Platform | Layer | Role | Caller | Dependencies | Status | Importance |
|---|---|---|---|---|---|---|---|
| `laptop/camera/source.py` | Windows | IO | open_camera共享DSHOW工厂 | live_shadow._camera_loop、diagnostics | OpenCV/UVC | CORE_ACTIVE | Critical |
| `laptop/camera/dual_camera.py` | Windows | IO/identity | inventory/resolve_roles/capture双路设备名绑定 | record_demonstrations、bounded_model_trial、CLI | source/PowerShell设备枚举 | ENTRYPOINT_ACTIVE | High |
| `laptop/diagnostics/camera_identity.py` | Windows | Identity | 设备身份及角色核验 | recorder/diagnostics | 设备名/PnP | CORE_ACTIVE | Critical |
| `laptop/client/observation_schema.py` | Both | Observation | 7D和图像/prompt schema、warmup | live_shadow/bounded_model_trial/eval | NumPy | CORE_ACTIVE | Critical |
| `laptop/client/live_shadow.py` | Windows | Runtime | select_observation、_can_loop、_camera_loop、metadata/prewarm | CLI/live_model_preview/bounded private helpers | camera/Bridge/OpenPI client | ENTRYPOINT_ACTIVE | High |
| `laptop/client/bounded_model_trial.py` | Windows | HIL runtime | worker真实模型单步及120s监督循环 | CLI/当前trial reports | PiperSession/identity/limit_target | HIL_TOOL_ACTIVE | Critical |
| `laptop/robot_client/bounded_policy_target.py` | Both | Action | limit_target把proxy解释为有界名义目标 | bounded_model_trial | replay_preflight SDK bounds | CORE_ACTIVE | Critical |
| `laptop/robot_client/piper_session.py` | Windows | Robot IO | poll/state/typed send与TX/RX日志 | piper_tool/supervised/bounded tools | SafetyBridge/transport | CORE_ACTIVE | Critical |
| `laptop/robot_client/replay_preflight.py` | Both | Limits | SDK名义范围、replay数据检查 | bounded_policy_target/preview/tests | unit_conversion | CORE_ACTIVE | Critical |
| `laptop/robot_client/verified_policy_command.py` | Both | Formal mapping | P12 verified preview→CommandProposal | supervised_policy_step | contracts/preview | CORE_ACTIVE | Critical |
| `laptop/robot_client/supervised_policy_step.py` | Windows | Formal HIL | prepare/send单次正式授权目标 | supervised_live_step/tests | P12GateEvidence/PositionCommandBatch | HIL_TOOL_ACTIVE | Critical |
| `laptop/robot_client/hil_guard.py` | Windows | Legacy gate | reject_legacy_raw_can_tx在设备打开前阻断 | 7个legacy execute/tests | Bridge reject | CORE_ACTIVE | Critical |
| `laptop/piper/unit_conversion.py` | Both | Units | SDK名义单位双向换算 | position_commands/preview/contract tools | 纯数学 | CORE_ACTIVE | Critical |
| `laptop/control/contracts.py` | Both | Contract | TimingContract/Snapshot/Prediction/Proposal | Bridge/formal/shadow | dataclass | CORE_ACTIVE | Critical |
| `laptop/runtime/modes.py` | Windows | State | RuntimeMode/BridgeState枚举 | Bridge/runtime/HIL | 枚举 | CORE_ACTIVE | Critical |
| `laptop/runtime/config.py` | Windows | Config | JSON加载及mode/TX/identity校验 | shadow/reject-only/tests | config文件；bounded未加载 | CORE_ACTIVE | Critical |
| `laptop/runtime/bridge_protocol.py` | Both | Protocol | bridge/policy元数据和时间契约 | shadow/tests | contracts | CORE_ACTIVE | Critical |
| `laptop/safety/bridge.py` | Windows | Safety | SafetyBridge状态机、freshness、授权及typed TX | PiperSession/shadow/HIL | contracts/codecs/transport | CORE_ACTIVE | Critical |
| `laptop/transport/can_transport.py` | Windows | Transport interface | CanFrame/抽象transport/fake接口 | Bridge/diagnostics/tests | 无真实device.send | CORE_ACTIVE | Critical |
| `laptop/transport/gs_usb_piper_can.py` | Windows | CAN backend | candleLight RX/typed TX/owner/unknown-delivery fault | SafetyBridge | gs_usb/PyUSB/libusb | CORE_ACTIVE | Critical |
| `laptop/transport/position_commands.py` | Windows | Codec/limits | SDK编码、snapshot/history/授权验证 | Bridge/transport/HIL | units/contracts | CORE_ACTIVE | Critical |
| `laptop/transport/bounded_trial.py` | Windows | Trial contract | BoundedTrialBatch的范围/TTL/4帧目标 | bounded_trial/zero_return/transport | 0x155-157/159 | CORE_ACTIVE | Critical |
| `laptop/transport/diagnostic_queries.py` | Windows | Query codec | getter strict allowlist | CanDiagnosticSession/Bridge | 0x4AF/472/477 | CORE_ACTIVE | Critical |
| `laptop/transport/mode_initialization.py` | Windows | Mode codec | 模式/reset/J6 enable typed frames | mode-init/enable-j6/transport | 0x150/151/471 | CORE_ACTIVE | Critical |
| `laptop/transport/gripper_commissioning.py` | Windows | Gripper codec | 单步/开闭端点/OEM限定帧 | Bridge/gripper HIL | 0x159/time/range | CORE_ACTIVE | Critical |
| `laptop/diagnostics/bounded_process.py` | Windows | Process bound | spawn worker父超时终止 | camera/bounded trials/zero return | multiprocessing，非急停 | CORE_ACTIVE | Critical |
| `laptop/recorder/record_demonstrations.py` | Windows | Capture entry | 可复用手动示教/倒计时/identity入口 | 操作员CLI | capture_session/camera inventory | ENTRYPOINT_ACTIVE | High |
| `laptop/recorder/capture_session.py` | Windows | Capture | 独立episode/session和success标签 | record_demonstrations | passive_capture/episode/label | CORE_ACTIVE | Critical |
| `laptop/recorder/passive_capture.py` | Windows | Capture IO | 相机与CAN RX原始数据记录 | capture_session/manual_teach_capture | OpenCV/RX/episode | CORE_ACTIVE | Critical |
| `laptop/recorder/episode.py` | Both | Raw contract | 完成/质量检查、边界帧判定 | capture/audit/pipeline | 视频/timestamps/manifest | CORE_ACTIVE | Critical |
| `laptop/recorder/label_episode.py` | Both | Labels | 人工success标签及来源验证 | capture_session/post_capture | outcomes/raw hash | CORE_ACTIVE | Critical |
| `laptop/recorder/align_raw.py` | Both | Alignment | host-arrival近邻对齐 | post_capture/CLI | video decode/source | CORE_ACTIVE | Critical |
| `laptop/recorder/verify_source.py` | Both | Provenance | 原始文件/hash/source关系审核 | align/proxy/CLI | manifest/SHA | CORE_ACTIVE | Critical |
| `laptop/recorder/next_state_proxy.py` | Both | Targets | future-state proxy build | post_capture/converter | alignment/future timestamp | CORE_ACTIVE | Critical |
| `server/dataset_tools/convert_next_state_proxy.py` | Linux | Dataset | convert_many多episode LeRobot/source_map | post_capture/CLI | LeRobot/proxy contract | CORE_ACTIVE | Critical |
| `server/training/proxy_multi_episode_contract.py` | Both | Data contract | 成功多episode、10fps/action/source校验 | pilot/service/prepare | info/source_map | CORE_ACTIVE | Critical |
| `server/training/post_capture_pipeline.py` | Both/LinuxGPU | Orchestration | prepare + 默认20步/上限100步pilot | CLI/20step report | align/convert/pilot | CORE_ACTIVE | Critical |
| `server/training/proxy_multi_episode_pilot.py` | LinuxGPU | Training core | build_config/execute/evaluate/stats/provenance | train/eval/serve wrappers | official OpenPI/LeRobot | CORE_ACTIVE | Critical |
| `server/training/piper_openpi_config.py` | LinuxGPU | Model adapter | PiperOpenPIInputs/Outputs/DataConfig | pilot.build_config/service | OpenPI transforms/Pi05 | CORE_ACTIVE | Critical |
| `server/training/train_demonstrations.py` | LinuxGPU | Training entry | foreground/background GPU/steps/report | 1000/5000CLI | pilot/subprocess | ENTRYPOINT_ACTIVE | High |
| `server/training/evaluate_demonstrations.py` | LinuxGPU | Eval entry | COMPLETE report→same-data预测评估 | CLI/历史automation | pilot.evaluate | ENTRYPOINT_ACTIVE | High |
| `server/training/serve_trained_demonstrations.py` | LinuxGPU | Service entry | report→policy/identity/background服务 | 当前两套服务PID | multi shadow/OpenPI | ENTRYPOINT_ACTIVE | High |
| `server/serve_derived_state_multi_shadow.py` | LinuxGPU | Serving core | inspect_artifacts/create_server/hash/loopback限制 | serve wrapper | policy_config/WebsocketPolicyServer | CORE_ACTIVE | Critical |
| `server/policy_server/artifact_identity.py` | Both | Identity | artifact hash/identity支持 | server/client契约 | JSON/SHA | CORE_ACTIVE | Critical |

非Python关键资产包括：piper_state_v1.yaml（STATE_7/名义单位）、piper_joint_limits.yaml（官方/读回范围证据）、
site_profile.template.json（formal gate模板）、list_directshow_cameras.ps1（DSHOW枚举）、requirements.txt（Laptop环境）、
CURRENT_PROJECT_PLAN.md（最新进度）、PHASE_GATES.md（正式标准）、provenance/openpi_git.json/.gitmodules（上游pin）。
它们是契约、指南或来源文件，不能只审计Python。

## 7. Core Documentation Map

下面逐份覆盖主要源码/文档目录与根部的第一方Markdown（生成的validation说明在第23节逐文件列出）；历史报告在其时间范围内有效，不因现在进度变化而失去证据价值。

| Path | 用途 | Classification | Freshness |
|---|---|---|---|
| `docs/AGILEX_SOURCE_AUDIT.md` | AgileX/PiPER source audit | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/CHECKPOINT_INIT_DECISION.md` | Initial π0.5 checkpoint decision | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `docs/CURRENT_PROJECT_PLAN.md` | PiPER × π0.5 current project plan / 当前项目计划 | SUPPORT_ACTIVE | 最新条目+report可用；旧段落不可覆盖新证据 |
| `docs/DATA_SCHEMA.md` | Data schema | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/DIAGNOSTIC_SCRIPT_SPEC.md` | Laptop diagnostic script specifications | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `docs/GRIPPER_TEACH_OPTIONS.md` | Gripper teach options | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/LAPTOP_SETUP_PLAN.md` | Laptop setup plan | HISTORICAL | 历史；见15节 |
| `docs/NETWORK_OPTIONS.md` | Network options and diagnostics | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `docs/OPENPI_PROVENANCE.md` | OpenPI provenance and contract | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `docs/PHASE_GATES.md` | Phase gates | SUPPORT_ACTIVE | 正式规范；commissioning实验不自动通过 |
| `docs/PI05_PIPER_MASTER_PLAN.md` | π0.5 + AgileX PiPER master plan | HISTORICAL | 历史；见15节 |
| `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md` | π0.5 + AgileX PiPER master plan | HISTORICAL | 历史；见15节 |
| `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | π0.5 + AgileX PiPER master plan | DUPLICATE | 与HISTORY字节相同，但被索引链接 |
| `docs/project-diagnosis-week-plan-2026-10-02.md` | PiPER × π0.5：代码诊断与维修等待期工作计划 | HISTORICAL | 历史；见15节 |
| `docs/PROJECT_LAYOUT.md` | Project and test layout | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `docs/README.md` | How to use the project documentation | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/REAL_ROBOT_SAFETY_PLAN.md` | Real-robot safety plan | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/REMOTE_ARCHITECTURE.md` | Remote architecture | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/SERVER_SETUP_PLAN.md` | GPU/data-server setup plan | HISTORICAL | 历史；见15节 |
| `docs/SSH_TUNNEL_SETUP.md` | SSH tunnel for the localhost-only policy server | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `docs/TEACH_DATA_PIPELINE.md` | Teach-mode multimodal data pipeline | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `docs/USER_DECISIONS.md` | User decisions required | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `laptop/camera/README.md` | Reusable Base/Wrist capture | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/client/README.md` | Laptop policy client | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/diagnostics/OEM_CAN_CAPTURE.md` | 官方上位机 5 秒命令记录 | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/diagnostics/README.md` | Laptop diagnostics | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/piper/README.md` | PiPER module | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `laptop/README_FIRST_CONNECTION.md` | First laptop connection | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `laptop/recorder/README.md` | Recorder | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/recorder/USAGE.md` | 可复用示教采集程序 | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/robot_client/README.md` | Offline robot-client replay | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/safety/README.md` | Offline safety validation | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `laptop/transport/README.md` | PiPER CAN transport | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `server/dataset_tools/README.md` | Dataset tools module | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `server/policy_server/README.md` | Policy server module | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `server/training/README.md` | Training module | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `server/training/TRAINING_USAGE.md` | 可复用示教训练入口 | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `scripts/README.md` | Project scripts | STALE_DOCUMENT | 旧状态滞后；见14节 |
| `tests/README.md` | Test organization | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `reports/P0_P1_DEPLOYMENT_REPORT.md` | P0 + P1 deployment report | HISTORICAL | 历史；见15节 |
| `reports/pi05_official_config_audit.md` | Official π0.5 configuration audit | HISTORICAL | 历史；见15节 |
| `reports/piper_gripper_teach_audit.md` | PiPER gripper during teach-mode audit (P0/P1) | HISTORICAL | 历史；见15节 |
| `reports/piper_teach_mode_audit.md` | PiPER teach-mode audit (P0/P1) | HISTORICAL | 历史；见15节 |
| `reports/README.md` | Reports | SUPPORT_ACTIVE | 保留用途；操作参数以parser/运行report为准 |
| `README.md` | 07_pi0.5 — Real-world π0.5 + AgileX PiPER VLA | STALE_DOCUMENT | 根导航/规范/manifest说明；状态以当前报告为准 |
| `AGENTS.md` | 07_pi0.5 project rules | SUPPORT_ACTIVE | 根导航/规范/manifest说明；状态以当前报告为准 |
| `datasets/manifests/README.md` | Dataset manifests | SUPPORT_ACTIVE | 根导航/规范/manifest说明；状态以当前报告为准 |

不存在独立CURRENT_STATUS.md或ARCHITECTURE.md主文件，实际用plan、REMOTE_ARCHITECTURE和源码恢复。
UTF-8读取未发现解码异常；部分文档已有乱码/替代字符样式，需区分历史字符污染与终端显示，
不能因PowerShell显示问题就断言文件必为GBK。本轮未修改编码。

## 8. Configuration Map

真实加载对象和启动参数决定运行状态，初始合同/模板不会自动驱动当前bounded worker。

| Path | 实际角色/消费者边界 | Status |
|---|---|---|
| `configs/camera_template.yaml` | 角色/标定模板；多字段null，当前inventory按名称枚举，不从这里加载index | SUPPORT_ACTIVE |
| `configs/dataset_template.yaml` | 数据字段、存储、采样规划/契约引用 | SUPPORT_ACTIVE |
| `configs/model/pi05_piper_v1.yaml` | 明示initialization intent only；NOT_STARTED/NOT_AVAILABLE不表示当前没训练 | HISTORICAL |
| `configs/network_template.yaml` | selected_option=null滞后；当前loopback8020/8021来自CLI | STALE_DOCUMENT |
| `configs/piper_joint_limits.yaml` | SDK名义范围+controller读回，J6读回±170°但SDK软件±120°；gripper min/max null | SUPPORT_ACTIVE |
| `configs/piper_state_v1.yaml` | STATE_7布局/名义单位；旧P2状态不作为最新现场进度 | SUPPORT_ACTIVE |
| `configs/policy/pi05_derived_state_10fps_shadow.json` | cef166e2 10fps step9 / port8010旧identity与hash；历史实验仍使用 | SUPPORT_ACTIVE |
| `configs/policy/pi05_proxy_shadow.json` | 历史2fps proxy服务；不是当前new3模型 | SUPPORT_ACTIVE |
| `configs/project_contract.yaml` | 权限/时间/存储约束有效；PLAN_ONLY、待选状态落后 | STALE_DOCUMENT |
| `configs/robot_template.yaml` | 身份、固件、接口现场模板；不替代fresh feedback | SUPPORT_ACTIVE |
| `configs/runtime/diagnostic_query.json` | 非运动getter mode及权限 | SUPPORT_ACTIVE |
| `configs/runtime/pi05_proxy_shadow_identity_lock.json` | 历史proxy固定identity；当前new3另有identity | SUPPORT_ACTIVE |
| `configs/runtime/policy_reject_only.json` | 拒绝policy控制模式 | SUPPORT_ACTIVE |
| `configs/runtime/shadow.json` | SHADOW can_tx=false；notes全build拒绝TX过宽，只有本mode保证 | SUPPORT_ACTIVE |
| `configs/runtime/supervised_site_profile.template.json` | formalP12 DRAFT/null/false模板；当前bounded worker不加载 | SUPPORT_ACTIVE |
| `configs/tasks/pick_place_v1.yaml` | 黑棒/绿垫任务；prompt也在observation_schema硬编码 | SUPPORT_ACTIVE |

真实训练配置由make_unregistered_config构造，不是直接训练model YAML。
identity锁由serve wrapper基于COMPLETE training report生成。RuntimeConfig.load已实现，
但bounded worker直接创建Bridge，不能凭shadow.json TX=false断言整个仓库无运动。

## 9. Entry Points

wrapper与实现分开登记，无入站import的CLI仍可能是有效独立工具。

| 类别 | 入口 | 实际复用/边界 |
|---|---|---|
| Laptop capture | laptop.recorder.record_demonstrations | capture_session/passive_capture，只记录图像/反馈 |
| Laptop cameras | laptop.camera.dual_camera | inventory/resolve_roles/capture，共享source打开 |
| Laptop shadow | laptop.client.live_shadow；live_model_preview wrapper | 身份/预热/RX/预测，不发目标 |
| Laptop model HIL | laptop.client.bounded_model_trial | 显式授权，最长120s/120updates，当前默认单步60° |
| Dataset | server.training.post_capture_pipeline prepare；convert_next_state_proxy CLI | label/source/hash→10fps多episode代理目标 |
| Training | server.training.train_demonstrations | 当前1000/5000使用；foreground/background/GPU/steps/report |
| Evaluation | server.training.evaluate_demonstrations | COMPLETE report→同训练episode评估 |
| Server | server.training.serve_trained_demonstrations | multi shadow core，当前8020/8021，prediction-only服务器 |
| CAN tool | laptop.robot_client.piper_tool | status/mode/enable/axis/gripper wrapper，非单一全部控制实现 |
| Home | bounded_zero_return / supervised_near_zero_home | 分段/近零、夹爪处理不同，不等价 |
| Diagnostic | query_* / CanDiagnosticSession / probe_* | getter有TX，normal ACK也不同于listen-only |
| Historical/synthetic | serve_policy_local、serve_proxy_piper_shadow、public_*、synthetic_* | 保留smoke/契约/比较，不是当前5000任务主服务 |

```text
bounded_model_trial
 -> live_shadow private helpers + dual_camera.inventory + recorder.camera_identity
 -> bounded_policy_target -> replay_preflight bounds
 -> PiperSession -> SafetyBridge -> typed codec -> GsUsb/PyUSB
 -> WebsocketClientPolicy -> SSH loopback -> serve_trained_demonstrations
 -> multi_shadow -> pilot.build_config -> piper_openpi_config

record_demonstrations -> capture_session -> passive_capture -> episode/label
post_capture_pipeline -> align_raw/verify_source/next_state_proxy/convert_many
train/evaluate wrappers -> proxy_multi_episode_pilot -> official OpenPI/LeRobot
```

sys.path修改、动态import官方脚本和_CONFIGS_DICT临时注册是现有适配方式；
本轮没有改写上游，后续package化也必须保留官方pin。

## 10. Action Semantics Audit

模型预测“后续实测状态代理目标”；执行程序另外把它解释为新定义的绝对位置命令。

| 层 | 语义/shape | 源码/证据 |
|---|---|---|
| raw | 手拖/操作的反馈，raw_can_command_supervision=false | capture_mode=feedback_only_operator_demonstration |
| label | derived_future_observed_state_position_target / next_observed_state_proxy_nominal_units | next_state_proxy/contract/training_provenance |
| Observation | base/wrist uint8 RGB HWC，state float32[7]，prompt string | live_shadow.select_observation；bounded worker |
| units | raw joint×π/180000→名义rad；gripper/1e6→名义m | worker第98/120/122行；机械标尺未验证 |
| model inputs | base→base_0；wrist→left_wrist；missing right=zeros/mask=false | PiperOpenPIInputs.__call__ |
| model outputs | internal action_dim32/horizon16，output[:7]→16×7 | make_unregistered_config/PiperOpenPIOutputs |
| Time | 10fps，首未来目标0.1s，16行后续位置 | contract/identity first_target_dt_s |
| execution | limit_target裁剪一行proxy，round到SDK整数→BoundedTrialBatch | worker第120–128行 |
| formal mapping | command_from_proxy拒绝盲转；P12 verified preview才有CommandProposal | contracts/verified_policy_command/supervised_policy_step |

训练和服务没有把反馈标签改成原厂command，语义保存正确。
但bounded worker确实用prediction_only/control_enabled=false服务的结果发真实目标；
这些flag只描述服务器无直接控制权，不能被界面读成整个实验无TX。
limit_target的hardware_control_allowed=false也只表示helper不授予权限，调用者仍会发送。
需独立execution grant，保存proxy→command解释、单位、映射验收和实际授权范围。

训练反馈存在负夹爪/近零边界越界，执行会hold或clip，使训练label与可执行目标不同。
bounded名义0..100mm、旧commissioning0..70mm、OEM54.015mm目标与实际夹爪间距未建立统一标尺。
GUI行程不能替代机械开口测量。

index=floor((now-started)/100ms)只按这次推理耗时选行，而每轮还包含到位、poll、重连和sleep。
250ms source限制使正常主要选0..2；index>=16通常被250ms限制先覆盖。
其余chunk没有持续消费；这才是action scheduler的实际缺口。

## 11. Robot Command Paths

按独立提出控制目标/阶段的实现模块计数，wrapper、Bridge和后端不重复计数。
条件式可调用不意味着每条路径已经现场成功；本轮均未执行。

| File | Can Send Robot Command? | IDs / APIs | Safety Gate | Should remain? |
|---|---|---|---|---|
| client/bounded_model_trial.py | 是，当前实验 | 0x151/155–157/159；send_bounded_model_trial | CLI授权/fresh/stable/batch/TTL | 保留，完善调度 |
| robot_client/bounded_zero_return.py | 是，分段归零 | 同上；夹爪保持目标也发送 | 起始范围/现场授权/逐段检查 | 保留 |
| robot_client/supervised_j1_commissioning.py | 是，单轴共用实现 | send_supervised_j1_commissioning_step；151可选/155–157 | 固定范围/confirm/模式稳定性 | 保留；axis wrapper不另计 |
| robot_client/supervised_gripper_commissioning.py | 是，单步夹爪 | open_gripper_step→159 | +1mm/.1Nm或+5mm/.3Nm明确授权 | 保留 |
| robot_client/supervised_gripper_max_open.py | 是，夹爪端点 | send_gripper_opening_once(open_to_max)→159 | 名义70mm端点/fresh/site | 保留诊断 |
| robot_client/supervised_near_zero_home.py | 是，近零/闭夹爪 | send_j1+gripper，151/155–157/159 | 近零姿态/夹爪源范围 | 与分段归零不等价 |
| robot_client/supervised_oem_gripper_test.py | 是，OEM端点 | send_oem_gripper_test_frame→159 | 固定54015/最多5s/fresh/审批 | 保留成功对照 |
| robot_client/supervised_policy_step.py | 条件式formal helper | send_prepared→send_supervised_position_batch | 全P12/verified preview/one-use授权 | 保留；非长worker路径 |
| robot_client/supervised_mode_initialization.py | 模式/reset可失力 | 150/151 typed frame | 零位支撑/现场授权/阶段 | 保留，非getter |
| robot_client/supervised_enable_j6.py | 使能改变保持状态 | 471 typed enable | 模式/零位/现场/读回 | 保留，非只读 |

表中client/robot_client均在laptop下。合计10个条件式控制实现路径，8个位置/夹爪、2个模式/使能。这个数是实现模块口径；Bridge只有7类机器人命令API，并不是10个独立物理CAN后端。
axis_commissioning、supervised_live_step、supervised_policy_session、piper_tool是包装/调用层，不重复计数。

| 发送层/查询 | 实际影响 | 门与边界 |
|---|---|---|
| PiperSession | typed query/mode/enable/joint/gripper/bounded委托 | 线程owner、TX/RX日志、Bridge |
| SafetyBridge | 7类robot-command API + diagnostic query | formal和commissioning分别检查 |
| gs_usb_piper_can | 真正self._device.send后端 | typed校验、owner、generic send拒绝、unknown delivery fault |
| codecs | 构造帧，无设备本身 | 类型/编码不是授权 |
| query_piper_firmware/joint_limits/all_joint_parameters/gripper_parameters | 4个getter CLI家族；4AF/472/477 TX，回应4AF/473/47C/47E | allowlist/CanDiagnosticSession；readiness_snapshot只是聚合 |
| normal ACK receive | 可产生ACK，无关节/夹爪目标 | ACK不等于物理listen-only |
| capture_oem_can | hook外部EXE已经发送的帧 | 自身不构造目标；外部上位机不纳入仓库Bridge计数 |
| server | 无机器人CAN目标路径 | model/训练权限，不拥有enable/reset权限 |

旧raw TX实现7个：supervised_j1_step_trial、supervised_j1_one_degree_trial、supervised_j5_probe、
supervised_joint_replay、supervised_restore_gripper、supervised_restore_start、supervised_smooth_joint_replay。
路径均为laptop/robot_client/<name>.py。execute先调用reject_legacy_raw_can_tx，
在设备访问前抛LegacyHILBlockedError，无override；不是额外7个当前可执行运动入口。
部分helper互相引用、tests验证guard，暂不可整体移走。

当前新发送已经集中在Bridge→transport，不应写成“全部绕过Bridge”。
但多帧CAN发送不是原子事务，失败可能已有部分目标送达；“拒绝”不证明完全无运动。

## 12. Duplicate Logic

相同职责才算重复；相似文件名或字段不代表可互相替换。

| 领域 | 定位 | 具体建议/边界 |
|---|---|---|
| CAN parser | PiperSession.state、live_shadow._can_loop、passive_capture解析 | 抽纯decode；记录/RX调度职责保留不同 |
| 单位 | unit_conversion；shadow、bounded worker、next_state_proxy手写π/180000、1e-6 | 共用名义换算，先证明数值不变 |
| 限位 | replay_preflight、position_commands、YAML、Bridge、offline_guard | 区分SDK/controller/机械/arrival/stability，不能只统一一个数字 |
| Camera | source.open_camera已复用；_camera_loop私有跨入口复用；早期probe独立扫描 | 改公开IO模块；解码视频VideoCapture不算设备打开重复 |
| Client | shadow/bounded/smoke/latency各建WebsocketClientPolicy | 共用建连/identity/warmup，保留不同fixture |
| Observation | schema、select_observation、bounded dict、server eval dict | 当前bounded没有调用完整shadow sensor-skew；共享packing/validation |
| Model mapping | server/piper_policy/*与training/piper_openpi_config | 前者旧契约/tests，后者当前32→7训练适配；先决策后归档 |
| Checkpoint/port | legacy configs/services/current report/argv/plan | named deployment profile，不能全局端口替换 |
| Flags | prediction_only/hardware_control_allowed/control_enabled | 区分server预测权限和Laptop execution grant |
| Defaults | worker step5/main60/helper1；episode min1/3/5；pilot100/long50000 | 每次记录effective config，不能依赖模糊默认 |
| 字节重复 | MASTER_PLAN_HISTORY与ORIGINAL | SHA完全相同（65b296d9711fe4203c7262ecd214b4f9b6f6502f8dd1a48329c3825c85b8b508）但均被索引链接；条件归档一份 |

## 13. Hard-Coded Values

将协议常量、机器设置、实验参数和安全验收数字分开；以下为重要运行值完整领域清单。

| Value | 实际位置/数值 | Disposition | 理由 |
|---|---|---|---|
| camera roles | dual_camera：ACER HD User Facing / Dabai DC1 | LOCAL_MACHINE_CONFIG | 名称/PnP身份；0/1是历史枚举，不永久固定 |
| Camera backend | source CAP_DSHOW | KEEP_IN_CODE | 明确Windows实现；其他平台另profile |
| Camera resolution/fps | source/dual_camera；model224×224 | MOVE_TO_CONFIG | 采集与模型resize分开 |
| CAN VID/PID/bus | gs_usb 1D50:606F / adapter_bus | LOCAL_MACHINE_CONFIG | 型号默认+设备实例选择 |
| CAN bitrate | 1Mbps | LOCAL_MACHINE_CONFIG | 现场总线协议，需匹配 |
| CAN IDs/endian | 150/151/155–157/159/471；2A1/2A5–8/261–266；>ii/>iHBB | KEEP_IN_CODE | 版本化协议codec，不任意运行改写 |
| host/alias | 127.0.0.1 / rtx6000_Mingkai | LOCAL_MACHINE_CONFIG | 私有隧道profile；不隐式公网 |
| ports | 1000=8020、5000=8021；历史8000/8002/8010 | MOVE_TO_CONFIG | 实验服务profile，保存override |
| checkpoint/dataset | legacy固定路径；current从training report读取 | MOVE_TO_CONFIG | manifest/identity选模型 |
| absolute paths | E:/01_NUS/...、/mnt/ssd/users/...多在docs/config/provenance/old tools | LOCAL_MACHINE_CONFIG | 当前训练ROOT由__file__推导；不能声称全核心绝对硬编码 |
| prompt | PIPER_PROMPT和tasks配置/instruction | MOVE_TO_CONFIG | 同一任务authority，匹配训练来源 |
| internal dim/horizon/output | config32/16→7 | KEEP_IN_CODE | policy版本契约，不任意改shape |
| data fps/offset | 10fps/0.1s；历史2fps/0.5s | MOVE_TO_CONFIG | dataset与identity需一致 |
| joint conversion | π/180000及round×180000/π | KEEP_IN_CODE | 共享名义helper，标定另记录 |
| gripper conversion | 1e-6 / round×1e6 | UNKNOWN | SDK名义标度，机械开口比例尚未确认 |
| SDK joint bounds | ±150；[0,180]；[-170,0]；±100；±70；±120° | SAFETY_CONSTANT | 与controller J6±170分开，保守交集 |
| gripper ranges | bounded0..100000，commission0..70000，OEM54015 | SAFETY_CONSTANT | 不同授权范围，先测映射 |
| joint per-step | main60；choices1/5/10/20/60；batch≤60000raw | MOVE_TO_CONFIG | 授权参数，不是rad/s |
| gripper per-step | 1000raw；invalid <0/>0.1 hold | SAFETY_CONSTANT | 不猜负值闭合行为 |
| feedback freshness/skew | 120ms / joint20ms | SAFETY_CONSTANT | 必须新鲜完整状态 |
| stability | ≥300ms历史；max−min≤50raw(.05°)，gap≤120ms | SAFETY_CONSTANT | 静止commissioning，不能直接当连续跟踪规则 |
| camera freshness/skew/wait | 250ms/100ms/2s，startup3s | MOVE_TO_CONFIG | 感知profile，非曝光同步 |
| source/TTL/heartbeat | 250ms，gripper120ms | SAFETY_CONSTANT | 单调时钟来源和授权期限 |
| arrival | 5s timeout、0.5°默认/可选.1°；夹爪1mm | MOVE_TO_CONFIG | 与静止容差分别定义 |
| speed mode | 151 can_joint byte speed5（名义5%） | SAFETY_CONSTANT | 未实测rad/s，60°不是速度 |
| reconnect/sleep | poll.65、postpoll.35、gc/sleep.5，每步重开 | MOVE_TO_CONFIG | 以后连续session代替 |
| record/playback | MJPG AVI30fps，read callback每帧写/flush | MOVE_TO_CONFIG | 播放率不等于真实到达频率 |
| long期限 | ≤120s/120updates；父duration+35 | MOVE_TO_CONFIG | 会话上限，guard可提前停 |
| velocity/acceleration | offline_guard参数；controller speed文档.3rad/s、accel尺度冲突 | UNKNOWN | 当前bounded不调用offline_guard；不能称已生效 |
| flags/site | worker三个site=True，operator_approved=True | MOVE_TO_CONFIG | CLI授权后typed grant持久记录更清楚 |
| training resource | GPU2/memory.95/batch1/num_workers0/wandb off | MOVE_TO_CONFIG | 实验profile，非算法常量 |

## 14. Stale / Conflicting Documents

STALE指把旧状态当现在会误导，不表示整份文件无价值。以下14项需要更正状态或入口说明。

| Path | 用途/状态证据 | 替代依据 | 仍被引用 | 独特功能 | 归档风险 | 建议 | Confidence |
|---|---|---|---|---|---|---|---|
| `README.md` | 当前摘要仍偏prediction-only/旧单示教 | CURRENT_PROJECT_PLAN最新记录+task_trial5000 | docs/CURRENT_PROJECT_PLAN.md; docs/DATA_SCHEMA.md; docs/PROJECT_LAYOUT.md | 项目根导航/背景 | 高：根入口不能移走 | 保留并更新；旧快照可归档 | HIGH |
| `docs/PROJECT_LAYOUT.md` | 部分模块仍按planned/相机无IO描述 | 实际camera/session/Bridge/worker | docs/CURRENT_PROJECT_PLAN.md; docs/README.md; README.md | 目录职责地图 | 中：旧链接仍使用 | 保留更新 | HIGH |
| `docs/DIAGNOSTIC_SCRIPT_SPEC.md` | 诊断脚本规格仍描述stub/待实现 | query_*/probe_*实际实现 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 原始验收要求 | 中：仍是设计依据 | 保留规格，去掉当前状态误读 | HIGH |
| `docs/NETWORK_OPTIONS.md` | 网络决策待选 | 现用SSH/loopback8020/8021 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | VPN备选说明 | 中：备选仍有用 | 保留备选，记录实际方案 | HIGH |
| `docs/USER_DECISIONS.md` | 原始决策pending | plan+训练/运行provenance | docs/CHECKPOINT_INIT_DECISION.md; docs/PI05_PIPER_PHASE_GATES.json | 决策来历 | 中：不可丢决策历史 | 分离历史和最新 | HIGH |
| `docs/CHECKPOINT_INIT_DECISION.md` | 权重选择待定 | 实际pi05_base training provenance | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 选择理由 | 中：旧状态不是旧理由失效 | 保留标历史状态 | HIGH |
| `docs/OPENPI_PROVENANCE.md` | LOCAL_PIN_PENDING | provenance/gitlink/server HEAD一致pin | docs/PI05_PIPER_PHASE_GATES.json | 上游来源说明 | 高：来源必须保留 | 更新pin状态 | HIGH |
| `docs/SSH_TUNNEL_SETUP.md` | 8000旧部署例子 | 当前服务argv8020/8021 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | SSH方法仍有效 | 中：历史base smoke仍8000 | 区分不同服务例子 | HIGH |
| `laptop/README_FIRST_CONNECTION.md` | 旧P2 handoff/8000入口 | 当前可复用capture/train/serve入口 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 首次配置顺序 | 中：用户可能按旧命令执行 | 保留更新 | HIGH |
| `laptop/piper/README.md` | adapter规划描述 | PiperSession/Bridge/transport | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | unit_conversion说明 | 中：单位模块仍import | 链接真实adapter | HIGH |
| `scripts/README.md` | 目录仍plan-only/planned | 12个实际Python工具及迁移地图 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 工具索引 | 低：指南过短过旧 | 保留重写索引 | HIGH |
| `server/training/README.md` | 以单episode旧tiny-fit为主要入口 | TRAINING_USAGE/train_demonstrations/multi pilot | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 历史实验操作方法 | 中：旧实验仍复现 | 更新主入口，旧流程标历史 | HIGH |
| `configs/project_contract.yaml` | PLAN_ONLY；D1/D2/D5未选择 | 实际factory/report/argv | tests/contracts/test_piper_contract.py | 权限/时间/存储硬约束 | 高：contract validators读取 | 保留契约，更新状态；不整份归档 | HIGH |
| `configs/network_template.yaml` | selected=null/USER_DECISION_REQUIRED | SSH与loopback实际部署 | docs/CURRENT_PROJECT_PLAN.md; docs/PI05_PIPER_PHASE_GATES.json; laptop/client/network_latency_probe.py | 可复用多方案模板 | 中：不是当前部署profile | 保留模板并新增当前profile | HIGH |

model/pi05_piper_v1明示初始化意图，因此归HISTORICAL而非把其NOT_STARTED字段当当前错误。
PHASE_GATES仍是正式标准；实验通路训练/运动不等于全部gate通过。
docs/PI05_PIPER_PHASE_GATES.json的PLAN_ONLY/旧状态也不能作为运行telemetry，
应保留原阶段规范并由最新report另证明进展，不自动填PASS。
CURRENT_PROJECT_PLAN超过两千行，最新段落与旧段落并列；必须按日期和运行报告理解。
部分字符/显示污染亦需之后修正，本轮保留内容。

## 15. Redundancy / Archive Candidates

特殊分类逐项给引用、替代、独特价值与风险。只有后续人工批准才迁移。

### High Confidence

以下3项适合条件式归档；HIGH不等于无条件删除，源码/历史全文仍需保留。

| Path | 用途/状态证据 | 替代依据 | 仍被引用 | 独特功能 | 归档风险 | 建议 | Confidence |
|---|---|---|---|---|---|---|---|
| `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | 另一份初始计划全文 | PI05_PIPER_MASTER_PLAN_HISTORY.md字节相同 | docs/PI05_PIPER_MASTER_PLAN.md | 无独特字节；名称/链接有兼容价值 | 低到中：MASTER_PLAN第9行仍链接 | 未来archive/historical_docs；先改引用，留一份全文 | HIGH |
| `scripts/add_final_gate_aliases.py` | 固定把初始manifest写PASS/YES的一次性补丁 | 现有deployment_manifest成品及当前运行report | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 历史补丁生成过程 | 中：重放会强行写gate PASS；保留源码 | 未来archive/deprecated_scripts，不再作当前状态工具 | HIGH |
| `scripts/annotate_pi05_smoke_metadata.py` | 对固定ALOHA base smoke加元数据的一次性补丁 | 现有smoke成品和当前服务自动metadata/provenance | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 旧报告注释的生成过程 | 低到中：历史重现可能需要 | 未来archive/deprecated_scripts，不删除 | HIGH |

### Medium Confidence

这些一次性runner/生成目录有历史复现用途，需人工确认，不以mtime判冗余。

| Path / group | 用途与替代 | 独特价值/引用风险 | 建议/Confidence |
|---|---|---|---|
| validation/camera_base_retry_runner_20261007_r1.py、camera_index_scan_runner_20261007_r1.py | 一次性相机排障；当前dual_camera/source是复用入口 | 历史backend/index对照仍独特，固定report路径可能复现 | 先确认引用后archive/old_camera_diagnostics，MEDIUM |
| validation/derived_state_shadow_loopback_20261007.py | 旧loopback/identity验证；当前serve wrapper常规化 | 旧dataset/checkpoint/metadata比较，非完全替代 | 暂保留，MEDIUM |
| validation/inspect_cando_function.py | OEM函数分析；capture_oem_can负责通用捕获 | 函数检查本身独特、第三方版本依赖 | 人工决定archive/oem_diagnostics，MEDIUM |
| .pytest-run*、.pytest_cache、validation/.pytest-* | 测试生成物，无生产替代需求 | 部分不可读，是否正在使用/有未保存日志未知 | 确认停止使用后整理，MEDIUM |

### Historical Documents

下表覆盖所有HISTORICAL分类；旧raw sender暂保留，不能批量搬走。

| Path | 用途/状态证据 | 替代依据 | 仍被引用 | 独特功能 | 归档风险 | 建议 | Confidence |
|---|---|---|---|---|---|---|---|
| `docs/PI05_PIPER_MASTER_PLAN.md` | 整理后的初始计划 | CURRENT_PROJECT_PLAN | docs/CURRENT_PROJECT_PLAN.md | 原阶段划分和历史链接 | 中：索引仍引用两副本 | 保留历史标识 | HIGH |
| `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md` | 初始计划全文 | 当前进度由CURRENT_PROJECT_PLAN补充 | docs/PI05_PIPER_MASTER_PLAN.md | 原始全文 | 高：至少保留一个历史全文 | 保留为历史原文主副本 | HIGH |
| `docs/LAPTOP_SETUP_PLAN.md` | 初装环境计划 | 现在requirements/Conda路径/plan | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 机器重建步骤 | 中：重建可能要复用 | 保留；未来historical_docs | MEDIUM |
| `docs/SERVER_SETUP_PLAN.md` | 初装服务器计划 | 实际runtime_root/provenance | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 环境重建步骤 | 中：不可仅按日期删除 | 保留；未来historical_docs | MEDIUM |
| `docs/project-diagnosis-week-plan-2026-10-02.md` | 2026-10-02周计划 | 当前plan最新条目 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 当时问题与决策 | 中：历史诊断记录 | 保留；未来historical_docs | HIGH |
| `configs/model/pi05_piper_v1.yaml` | 明示initialization intent only | 实际piper_openpi_config factory/training report | docs/CURRENT_PROJECT_PLAN.md; server/piper_policy/piper_policy_config.py; server/training/check_piper_readiness.py; server/training/piper_openpi_config.py; server/training/piper_train_entry.py; scripts/write_deployment_manifest.py; reports/P0_P1_DEPLOYMENT_REPORT.md; provenance/deployment_manifest.json | formal绝对位置初始意图 | 高：piper_train_entry/contract/tests引用 | 保留；不作当前训练状态 | HIGH |
| `reports/P0_P1_DEPLOYMENT_REPORT.md` | 2026-09-19初装验收 | 最新validation补充，不覆盖历史 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 初装测量/结果 | 高：provenance证据 | 保留不可变报告 | HIGH |
| `reports/pi05_official_config_audit.md` | 官方ALOHA/base配置审计 | 当前PiPER factory/identity补充 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | ALOHA sanity与PiPER差别 | 高：来源和环境证据 | 保留 | HIGH |
| `reports/piper_gripper_teach_audit.md` | P0/P1 teaching夹爪文档审计 | OEM/CAN_CTRL实测补充 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 未证明teach-mode独立夹爪支持 | 中：CAN_CTRL成功不等价teaching支持 | 保留历史上下文 | HIGH |
| `reports/piper_teach_mode_audit.md` | P0/P1程序进入teaching文档审计 | 后续mode-init补充 | 本次第一方静态入站引用未找到；独立CLI/人工访问仍可能存在 | 进入示教与CAN_CTRL切换区别 | 中：不能互相冒充 | 保留 | HIGH |
| `laptop/robot_client/supervised_j1_step_trial.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | docs/CURRENT_PROJECT_PLAN.md; laptop/robot_client/supervised_j1_one_degree_trial.py; laptop/robot_client/supervised_j5_probe.py; laptop/robot_client/supervised_joint_replay.py; laptop/robot_client/supervised_restore_gripper.py; laptop/robot_client/supervised_restore_start.py; laptop/robot_client/supervised_smooth_joint_replay.py; tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_j1_one_degree_trial.py; tests/laptop/test_supervised_j1_step_trial.py; tests/laptop/test_supervised_restore_start.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_j1_one_degree_trial.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_j1_one_degree_trial.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_j5_probe.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_j5_probe.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_joint_replay.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | tests/laptop/test_legacy_hil_guard.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_restore_gripper.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | docs/CURRENT_PROJECT_PLAN.md; tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_restore_gripper.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_restore_start.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | laptop/robot_client/supervised_j5_probe.py; laptop/robot_client/supervised_joint_replay.py; laptop/robot_client/supervised_restore_gripper.py; laptop/robot_client/supervised_smooth_joint_replay.py; tests/laptop/test_legacy_hil_guard.py; tests/laptop/test_supervised_restore_start.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |
| `laptop/robot_client/supervised_smooth_joint_replay.py` | 旧direct gs_usb sender；execute先guard阻断 | 当前Bridge委托commissioning/bounded/home；非一一替代 | docs/CURRENT_PROJECT_PLAN.md; tests/laptop/test_legacy_hil_guard.py | 旧编码/恢复/平滑以及guard回归，部分只读helper共享 | 高：tests和legacy互相import | 暂保留；先抽helper并修引用再决定 | HIGH |

### Do Not Archive Yet

这些文件存在当前调用、测试、权限或来源链用途，旧名称不构成归档理由。

- SafetyBridge、GsUsbPiperCanTransport、position_commands、hil_guard：真实发送边界与7个legacy拒绝回归。
- unit_conversion与replay_preflight：bounded target、position codec、preview等仍import。
- supervised_policy_step、verified_policy_command、P12GateEvidence、site template：formal验收通路，当前commissioning未替代。
- bounded_zero_return与supervised_near_zero_home：起始范围、分段和夹爪处理不同。
- server/piper_policy、旧proxy/public/ALOHA服务与配置：契约/历史测试仍引用。
- raw/source_map/outcomes/dataset/norm/checkpoint/training provenance：当前3demo模型的来源，不是可丢弃冗余。
- provenance/.gitmodules/官方OpenPI pin：来源必须保留。
- world_land_sea相关文件、testfoo.txt：归属UNKNOWN，不允许自动处理。

## 16. Proposed Repository Structure

仅建议未来结构，不搬文件，不替换OpenPI子模块，不把Windows IO和Linux训练强行合成一个进程。

```text
project/
  src/piper_vla/
    camera/          # public acquisition / identity
    robot/           # session / feedback / nominal units
    observation/     # schema / packing / time contracts
    policy/          # client / identity / output semantics
    control/         # target interpreter / scheduler (implementation needed)
    safety/          # Bridge / state machine / limits / typed grants
    transport/       # CAN codec / gs_usb backend
    dataset/         # verify / align / labels / proxy contracts
  apps/laptop/       # capture / shadow / supervised task entrypoints
  apps/server/       # serving entrypoint
  training/          # config / stats / train / eval wrappers
  tools/diagnostics/ + tools/dataset/ + tools/hil/ + tools/provenance/
  configs/hardware/ + configs/policy/ + configs/runtime/ + configs/tasks/
  docs/ + provenance/ + tests/unit/ + tests/integration/ + tests/contracts/
  third_party/openpi/ # official unchanged pinned submodule
  archive/historical_docs/ + deprecated_scripts/ + old_training_experiments/
  datasets/ + checkpoints/ + cache/ + validation/ # local-only runtime/evidence
```

文件存在项目目录不等于被Git track；weights/raw多数已ignore。
实际问题是validation中部分CAN/视频/临时生成物仍出现在untracked清单。

## 17. Proposed File Migration Map

后续先建兼容wrapper和回归证明，再改变使用指南；搬文件不会自动实现scheduler。

| Current path | Proposed path | Reason / compatibility |
|---|---|---|
| camera/source + 公共_camera_loop | src/piper_vla/camera/ | 消除live_shadow私有helper跨入口依赖 |
| robot_client/piper_session | src/piper_vla/robot/session.py | 长寿命硬件session，旧路径导出兼容 |
| piper/unit_conversion + feedback decode | robot/units.py + feedback.py | 纯换算/协议复用，先冻结数值 |
| observation_schema + select_observation | observation/ | schema/packing/timing纯函数 |
| client/live_shadow | apps/laptop/shadow + camera/policy/observation | 入口保留编排，分离IO/网络/合同 |
| client/bounded_model_trial | apps/laptop/supervised_task + control/scheduler | 实际执行入口；scheduler另实现 |
| safety/runtime/transport | src/piper_vla对应子包 | 已有实现收敛接口，不另写平行raw sender |
| record_demonstrations | apps/laptop/record_demonstrations | 保留用户倒计时/标签/可复用入口 |
| align_raw/verify_source/next_state_proxy | dataset/ + tools/dataset wrapper | pure转换与CLI分开 |
| train/eval wrappers、pilot/config | training/ | 当前数据/训练契约和上游pin保持 |
| serve_trained_demonstrations | apps/server/serve_trained | 继续由training report选择checkpoint |
| supervised mode/enable/home/OEM/axis/gripper | tools/hil/ | 不是无副作用unit tests |
| scripts/validate_piper_contract、validate_piper_continuous | tools/contracts + tests/contracts | CLI校验与自动断言分开 |
| scripts/audit_laptop_probe、refresh_environment_audit | tools/diagnostics | 静态/环境报告工具 |
| scripts/run_pi05_base_smoke、run_local_policy_server_smoke、run_server_performance | tools/provenance | ALOHA sanity/性能复现，不冒充PiPER任务 |
| scripts/write_checkpoint_provenance、write_deployment_manifest、write_socket_binding_report | tools/provenance | 来源记录生成，非runtime加载器 |
| scripts/add_final_gate_aliases、annotate_pi05_smoke_metadata | archive/deprecated_scripts | 三项高置信中的历史补丁，不重放填PASS |
| MASTER_PLAN_ORIGINAL | archive/historical_docs | 相同全文保留一份，先改索引 |

## 18. Technical Debt

按实际控制影响排序；不把所有历史文件都当同样严重问题。

| Severity | Finding | 定位与验收 |
|---|---|---|
| Critical | 未验收断联/异常硬件stop或safe-hold | Bridge.watchdog_tick说明非后台watchdog；停止发送/kill进程不证明物理stop，扩展无人监督前需现场验收 |
| High | 静止commissioning规则与连续控制耦合 | snapshot ≥300ms/.05° vs arrival.5°；应分模式，不删所有gate |
| High | 动作时间不匹配10fps标签 | 每轮只选一行，等到位/reopen/sleep；scheduler测实际周期 |
| High | proxy→命令映射缺机械标定 | joints/gripper units、零位、方向/比例，尤其夹爪范围不一致 |
| High | 推理timeout实际可能等待线程 | ThreadPoolExecutor退出wait，250ms异常不保证250ms停止 |
| High | 相机同步writer/flush拖慢读取 | record_frame/_camera_loop；wrist110 vs base2048，需异步记录与gap监测 |
| High | owner只限制对象/线程 | transport.claim无OS进程锁，重开USB/自身echo竞争规避脆弱 |
| High | server预测权限与Laptop TX标志scope混淆 | prediction_only=true不能表示整个实验无TX；formal/commissioning grant分别登记 |
| High | 当前源码未完整纳入Git来源 | HEAD仅bootstrap、22modified/2277untracked；source manifest绑定run，审查后提交 |
| Medium | 单位/parser/observation重复 | 已有helper仍手写，纯函数复用先保证不改变结果 |
| Medium | 多个当前状态truth | PLAN_ONLY/NOT_STARTED与COMPLETE并存；模板和最新manifest分层 |
| Medium | 初始化错误记录不完整 | worker identity/prewarm/建目录在try之前；父可能仅WORKER_ERROR |
| Medium | kill/部分TX时末步结果缺失 | progress在完整step后写，CAN JSONL有TX但report不完整；保存delivery_unknown和未完成步骤 |
| Medium | hardware limits来源不同且未统一登记 | SDK/controller/机械/夹爪端点不直接等价 |
| Medium | 环境复现分层不足 | OpenPI有uv.lock/pin；Laptop requirements未提供统一安装锁/机器profile；不是全项目无lock |
| Medium | 平台耦合/import技巧 | post_capture import laptop.recorder纯转换；sys.path/importlib/_CONFIGS_DICT私有耦合，保持pin后package化 |
| Low | 历史全文/metadata patchers | 高置信3项，先修引用后archive |
| Low | 无关地图/临时文本混root | UNKNOWN，人工决定归属 |

异常处理不全是吞错：worker返回STOPPED_NO_RETRY，父进程返回WORKER_ERROR。
问题在前置初始化、部分TX和kill后记录粒度。Bridge已存在runtime state machine，
缺口是长循环调度/停止验收/跨进程权限，不能写“完全无状态机”。
没有足够证据把无production import的CLI都判dead code。

## 19. Safety-Critical Findings

本节列当前长循环的主要停止、裁剪和授权条件，解释为何未跑满。
数值对应本轮源码；历史report可能使用旧容差。条件不是全部都来自同一个“安全门”。

| 阶段 | 条件/实际数字 | 结果 | 源码依据 |
|---|---|---|---|
| 会话授权 | long需site-cleared-long-loop-authorized；duration在(0,120]；短模式另有授权flag | 未授权/参数错误不启动 | bounded_model_trial.main |
| 身份/网络 | identity schema、checkpoint/dataset/norm/hash、语义/shape/OpenPI pin不匹配；WebSocket/warmup失败 | 初始化失败，可能只记录WORKER_ERROR | load_artifact_identity_lock/validate_policy_metadata/prewarm_policy |
| 相机身份/文件 | 两role缺失/名称不唯一/无法绑定；目录已存在、VideoWriter不能打开 | 拒绝或异常 | camera_identity(inventory)/record_frame |
| 相机启动 | 3s内没有两路age≤250ms图像 | startup timeout | worker59–66 |
| CAN接口 | 无/多适配器、USB驱动/read异常、transport faulted、owner线程不符 | 故障或拒绝，不自动retry | PiperSession/GsUsb transport |
| 状态新鲜 | status、三组joint、六motor feedback≤120ms且非未来；gripper≤120ms | snapshot/TX拒绝 | validate_j1_commissioning_snapshot/Bridge |
| joint skew | 三组joint feedback时间差≤20ms | snapshot拒绝 | MAX_COMMISSIONING_JOINT_SKEW_NS |
| 模式/健康 | ctrl=1、arm_status=0、move_mode=1、error=0、六轴enabled | 不满足不发 | snapshot.status_payload/motor flags |
| 竞争控制 | 接到0x150–15F或471等控制帧；自身TX echo可能误判 | 拒绝/故障；worker重开规避 | Bridge.J1_COMPETING_COMMAND_IDS/worker |
| 现场 | estop/low_speed/envelope均True | worker由CLI授权后写True，并非物理传感验证 | worker75/snapshot validator |
| 当前/历史SDK范围 | J1±150、J2[0,180]、J3[-170,0]、J4±100、J5±70、J6±120° | 越界拒绝，不用controller J6±170扩大 | position_commands/replay_preflight |
| 静止历史 | 最近≤420ms内≥2样本、跨度≥300ms、gap≤120ms、每轴max−min≤50raw(.05°)、最新历史等于snapshot | 不满足拒绝；stable_snapshot最多等3s | position_commands385–411；worker19–30 |
| 静止复查位置 | 相机等待>20ms后和infer后直接commissioning_snapshot | 此处无完整3s等待，同样规则可即时停 | worker92–95/113 |
| gripper健康 | status_raw & 0xBF非零 | 拒绝；0x40 enable位不是fault | worker78/Bridge |
| 推理图像 | 每路age≤250ms，两相机差≤100ms；wait最多2s；reader exception | 无新鲜图像/读取故障停，不发新目标 | worker81–96 |
| 推理/source age | started后250ms未完成或处理完超过250ms | TimeoutError；executor退出可能仍等infer线程 | worker102–115 |
| 输出 | actions必须(16,7)且finite | NaN/Inf/shape错误拒绝 | worker110–111 |
| infer期间位姿变化 | 任轴>50raw=.05°；夹爪>1000raw=名义1mm | 不接受旧observation，停止 | worker116–117 |
| horizon | index=floor(source age/100ms)，index≥16 | horizon过期；通常先被250ms时限覆盖 | worker118–119 |
| 单步关节 | CLI默认60°，允许1/5/10/20/60；每轴≤chosen；目标SDK bounds | helper先clip，batch再validate；不是总共60° | limit_target/BoundedTrialBatch |
| 夹爪目标 | 名义0..0.1m、单步≤0.001m；模型负值/>.1无效 | 无效预测hold；有效clip；state超恢复区拒绝 | limit_target/batch |
| 累计范围 | long_loop=None，无累计限制；short=updates×step，current另+.1°容差 | 只有短试跑触发累计限 | worker55/80/123 |
| typed grant/source | batch sequence/current joints/gripper须与Bridge fresh snapshot一致，operator approved，正确mode/READY | 不一致拒绝 | Bridge.send_bounded_model_trial/transport |
| TTL/heartbeat | expires≤started+250ms，每帧仍有效；heartbeat≤250ms | 发送前拒绝或部分TX后故障；不是后台watchdog | batch.validate_time/Bridge |
| transport最终边界 | owner=safety-bridge；已connected、无position_delivery_unknown、不能LISTEN_ONLY；request不可重用且每transport最多3次 | 不满足拒绝；当前每步新建transport使预算每轮重置 | gs_usb_piper_can.send_bounded_model_trial 683–705 |
| adapter发送异常 | 任帧accepted不明确/异常 | unknown delivery锁，部分帧可能已送达 | transport._latch_position_delivery_unknown |
| 到位等待 | 每次≤5s；当前joint tolerance=.5°（500raw），可选.1°；fresh/healthy/allenabled且joint反馈在发送后更新 | 非JOINT_FEEDBACK_MATCH停止 | PiperSession.wait_joint_target/worker130–132 |
| 发送后 | postpoll.35s；complete_fresh/error0/ctrl1/allenabled；gripper target error≤1mm | 不满足停止 | worker133–135 |
| 总期限/步数 | ≤120s，最多120updates；每步/发前检查deadline | 正常结束也只是TIME_BOUNDED_TRIAL_COMPLETE，不是任务成功 | worker67–69/124/142 |
| 父进程/收尾 | timeout=duration+35（120时155s）；camera join2s仍alive | 终止worker或reader did not stop | main/run_bounded/finally |
| 其他异常/中断 | IO/JSON/USB异常STOPPED_NO_RETRY；子进程BaseException→WORKER_ERROR | 不自动重试、不自动回零/硬件急停 | worker except/bounded_process |
| 当前未接入保护 | offline_guard速度/加速度，formal全P12，watchdog_tick后台周期，完整shadow camera/CAN skew | 不得列为当前生效上限 | worker实际call graph与formal/shadow对比 |

最新5000 r1在第一个成功运动更新前报告inference/source age exceeds250ms；
r2有24次关节match后报告history stable-window错误。不能把两次都说成模型角度不够大。
r2错误字符串同时覆盖历史不足、gap、SDK历史越界、drift、latest mismatch，
只凭字符串不能确定具体子条件；后续需要逐检查测量值/reason code。

到位.5°通过不表示300ms/.05°静止通过。静止规则适合commissioning，
持续执行需要有运动过程状态和轨迹调度，不能靠放宽所有规则得到正确控制周期。

最大安全缺口：停止发送、关闭USB、kill进程都不是已经发送并确认的硬件stop/safe-hold。
当前有TTL/heartbeat检查，但没有独立持续watchdog；网络线程挂住会暴露这一区别。
保留协议/机械边界、故障锁、来源时序、现场急停；停止/断联行为需要另行受监督验收。

## 20. Recommended Single Sources of Truth

按领域设一个维护authority，并保留不可变证据；下面新路径都是建议，本轮未创建。

| 领域 | 当前依据 | 未来唯一维护入口 |
|---|---|---|
| runtime | parser/实际report/服务argv，bounded未读RuntimeConfig | configs/runtime/<named-supervised-profile>.json + typed loader，保存effective config |
| hardware | current DSHOW inventory/role确认/CAN query/SDK | configs/hardware/<site>.json + calibration refs，index不永久身份 |
| action | proxy builder/contract/metadata + bounded解释 | 版本化统一action contract，区分training proxy/execution command/units |
| status | CURRENT_PROJECT_PLAN最新条目 + validation | plan最新状态/下一步，README只摘要/导航，实验日志另存 |
| limits | SDK/position_commands + controller YAML + site gate | 集中model标来源、模式、机械标定；静止验收与运动tracking分开 |
| checkpoint | COMPLETE training report/provenance/identity | deployment manifest仅指一个训练报告及匹配hash，不另手填路径 |
| dataset | meta/info/source_map/outcomes/raw hashes/norm provenance | immutable dataset manifest，不丢标签/代理语义 |
| software | dirty local tree + server runtime_root，5文件hash相同 | source manifest：HEAD+dirty file SHA+OpenPI pin+env lock，绑定每次run |

## 21. Recommended Next Refactor Order

顺序以减少当前错误归因和回归风险为目标，先有可比证据再搬目录。

1. 更新状态/导航，登记真实入口、model/port/profile；分清实验闭环和formal验收。
2. 冻结proxy→执行目标语义，保存typed session grant和effective config，明确flag scope。
3. 抽纯feedback decoder、nominal conversion、observation packing；用已有数据/fixture证明结果不变。
4. 分层登记SDK/controller/机械、stability/arrival/timing；逐个reject记录reason code和测量值。
5. 在现有PiperSession/Bridge/transport上做长寿命session与竞争/自身echo识别，不另造raw sender。
6. 做异步相机记录、gap监测、有界可取消网络等待、异常停止记录；现场停止验收另行授权。
7. 实现匹配10fps未来目标的scheduler/trajectory，记录实际控制周期，再对照1000与5000。
8. 合并应用入口和usage，保留旧module兼容，补环境/source manifest，再审查提交有效源码。
9. 最后修3项高置信归档的引用、整理历史文档/生成物；旧guard/helper完成迁移后再处理。

本轮不执行重构、测试、归档、Git提交或运动。

## 22. Files Requiring Human Decision

以下信息影响后续整理/正式部署，本轮全部保留原位。

| 文件/领域 | 需要决定 | 保留原因 |
|---|---|---|
| 7个legacy HIL + tests | 是否继续历史raw编码离线复现，先抽helper还是长期guard regression | 互相import，smooth replay独特，不是都已等价替换 |
| server/piper_policy及proxy/public/ALOHA服务 | 是否继续支持旧契约/smoke | current multi factory不是所有旧fixture等价替代 |
| world_land_sea*、knowledge_landsea_experiment、artifacts/knowledge_world_map、testfoo.txt | 独立实验归属/是否移出 | UNKNOWN，没有清理授权 |
| validation runner/OEM inspector/旧目录 | 哪些实验需要复跑/保留 | 设备/版本/identity背景独特 |
| DRAFT site profile/gripper mapping | 单位、方向、机械范围、夹爪零位如何验收 | OEM目标成功不证明实际开口标尺 |
| CURRENT_PROJECT_PLAN旧条目 | 是否拆历史日志并保留当前摘要 | 不改写既有实测事实 |
| runtime_root与server root | 以后部署authority/同步策略 | 本轮只比5关键文件，不代表全仓同步 |
| validation CAN/video/raw和.gitignore | local-only保留或另立release流程 | 当前规则真实captures local-only，不混入root push |

## 23. Final Classification Table

下表按第2节范围逐文件分类。GENERATED可仍是重要证据；UNKNOWN保留原位。
HIGH archive仅指条件式未来迁移，其余历史默认保留；CORE_ACTIVE计数不包含入口/HIL工具。清单不含本轮新写的两份审计文档，以免把输出算作输入。

逐文件静态审计共**635个文件**：Python成功AST解析258个，Markdown/YAML/YML/TXT共61个；CORE_ACTIVE 36个，当前主入口6个。

| Classification | Count |
|---|---|
| ARCHIVE_CANDIDATE | 2 |
| CORE_ACTIVE | 36 |
| DUPLICATE | 1 |
| ENTRYPOINT_ACTIVE | 6 |
| EXPERIMENTAL | 20 |
| GENERATED | 299 |
| HIL_TOOL_ACTIVE | 18 |
| HISTORICAL | 17 |
| STALE_DOCUMENT | 14 |
| SUPPORT_ACTIVE | 122 |
| TEST_ACTIVE | 95 |
| UNKNOWN | 5 |

| Path | Classification | Keep? | Archive candidate? | Confidence | Reason |
|---|---|---|---|---|---|
| `.gitignore` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `.gitmodules` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/OPENPI_PROVENANCE.md |
| `AGENTS.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `README.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节；引用 docs/CURRENT_PROJECT_PLAN.md, docs/DATA_SCHEMA.md 等 |
| `artifacts/knowledge_world_map/knowledge_world_map.py` | UNKNOWN | 保留 | 人工复核 | LOW | 非PiPER主链或用途未建立；人工确认，禁止自动清理；inside:175, classify:191, png_chunk:198 |
| `configs/camera_template.yaml` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `configs/dataset_template.yaml` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 tests/contracts/test_piper_contract.py |
| `configs/model/pi05_piper_v1.yaml` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节；引用 docs/CURRENT_PROJECT_PLAN.md, provenance/deployment_manifest.json 等 |
| `configs/network_template.yaml` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PI05_PIPER_PHASE_GATES.json 等 |
| `configs/piper_joint_limits.yaml` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 configs/piper_state_v1.yaml, docs/CURRENT_PROJECT_PLAN.md 等 |
| `configs/piper_state_v1.yaml` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 provenance/deployment_manifest.json, reports/P0_P1_DEPLOYMENT_REPORT.md 等 |
| `configs/policy/pi05_derived_state_10fps_shadow.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, server/policy_server/proxy_config.py 等 |
| `configs/policy/pi05_proxy_shadow.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 configs/runtime/pi05_proxy_shadow_identity_lock.json, configs/runtime/policy_reject_only.json 等 |
| `configs/project_contract.yaml` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节；引用 tests/contracts/test_piper_contract.py |
| `configs/robot_template.yaml` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `configs/runtime/diagnostic_query.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/diagnostics/can_diagnostic_session.py 等 |
| `configs/runtime/pi05_proxy_shadow_identity_lock.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/live_shadow.py 等 |
| `configs/runtime/policy_reject_only.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/shadow_preview_smoke.py 等 |
| `configs/runtime/shadow.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md |
| `configs/runtime/supervised_site_profile.template.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 tests/laptop/test_supervised_live_step.py |
| `configs/tasks/pick_place_v1.yaml` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 provenance/deployment_manifest.json, scripts/write_deployment_manifest.py 等 |
| `datasets/manifests/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `docs/AGILEX_SOURCE_AUDIT.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `docs/CHECKPOINT_INIT_DECISION.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `docs/CURRENT_PROJECT_PLAN.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/PI05_PIPER_MASTER_PLAN.md, docs/README.md 等 |
| `docs/DATA_SCHEMA.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/PI05_PIPER_PHASE_GATES.json, laptop/recorder/README.md 等 |
| `docs/DIAGNOSTIC_SCRIPT_SPEC.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `docs/GRIPPER_TEACH_OPTIONS.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `docs/LAPTOP_SETUP_PLAN.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `docs/NETWORK_OPTIONS.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `docs/OPENPI_PROVENANCE.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节；引用 docs/PI05_PIPER_PHASE_GATES.json |
| `docs/PHASE_GATES.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, docs/README.md 等 |
| `docs/PI05_PIPER_MASTER_PLAN.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节；引用 docs/CURRENT_PROJECT_PLAN.md |
| `docs/PI05_PIPER_MASTER_PLAN_HISTORY.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节；引用 docs/PI05_PIPER_MASTER_PLAN.md |
| `docs/PI05_PIPER_MASTER_PLAN_ORIGINAL.md` | DUPLICATE | 保留；仅后续批准迁移 | 条件式候选 | HIGH | 字节重复全文或固定历史报告补丁；详第15节条件式归档证据；引用 docs/PI05_PIPER_MASTER_PLAN.md |
| `docs/PI05_PIPER_PHASE_GATES.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 README.md |
| `docs/PROJECT_LAYOUT.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节；引用 docs/CURRENT_PROJECT_PLAN.md, docs/README.md 等 |
| `docs/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 README.md |
| `docs/REAL_ROBOT_SAFETY_PLAN.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, docs/README.md 等 |
| `docs/REMOTE_ARCHITECTURE.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/PI05_PIPER_PHASE_GATES.json |
| `docs/SERVER_SETUP_PLAN.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `docs/SSH_TUNNEL_SETUP.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `docs/TEACH_DATA_PIPELINE.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `docs/USER_DECISIONS.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节；引用 docs/CHECKPOINT_INIT_DECISION.md, docs/PI05_PIPER_PHASE_GATES.json |
| `docs/project-diagnosis-week-plan-2026-10-02.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `knowledge_landsea_experiment/test.txt` | UNKNOWN | 保留 | 人工复核 | LOW | 非PiPER主链或用途未建立；人工确认，禁止自动清理 |
| `laptop/README_FIRST_CONNECTION.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `laptop/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/camera/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md |
| `laptop/camera/dual_camera.py` | ENTRYPOINT_ACTIVE | 保留 | 否/暂缓 | HIGH | inventory/resolve_roles/capture双路设备名绑定；inventory:19, resolve_roles:30, _write_report:48；引用 docs/CURRENT_PROJECT_PLAN.md, docs/project-diagnosis-week-plan-2026-10-02.md 等 |
| `laptop/camera/source.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | open_camera共享DSHOW工厂；backend_constant:2, open_camera:11；引用 laptop/client/live_shadow.py, laptop/diagnostics/probe_dual_cameras.py |
| `laptop/client/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `laptop/client/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/client/action_shadow_replay.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Replay verified, file-backed PiPER action episodes against a prediction-only server.；validate_action_shadow_metadata:36, _inside:70, _read_rgb:79；引用 server/training/README.md, tests/laptop/test_action_shadow_replay.py |
| `laptop/client/audit_live_shadow.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Replay saved live-shadow predictions through the pure offline veto.；_nominal_joint_bounds:19, audit:36, main:99；引用 tests/laptop/test_live_shadow.py |
| `laptop/client/bounded_model_trial.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | worker真实模型单步及120s监督循环；stable_snapshot:19, worker:33, main:160；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/bounded_zero_return.py 等 |
| `laptop/client/latency_probe.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Small wrapper around the dependency-light official client benchmark.；main:13；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/network_latency_probe.py 等 |
| `laptop/client/live_model_preview.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Reusable live prediction entrypoint using the commissioned dual-camera resolver.；main:10；引用 docs/CURRENT_PROJECT_PLAN.md |
| `laptop/client/live_shadow.py` | ENTRYPOINT_ACTIVE | 保留 | 否/暂缓 | HIGH | select_observation、_can_loop、_camera_loop、metadata/prewarm；validate_artifact_identity_lock:57, load_artifact_identity_lock:130, validate_policy_metadata:136；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `laptop/client/network_latency_probe.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Bounded synthetic-observation benchmark for the private OpenPI WebSocket path.；percentile_summary:31, _bounded_infer:47, _close_client:70；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/shadow_preview_smoke.py 等 |
| `laptop/client/observation_schema.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 7D和图像/prompt schema、warmup；make_piper_proxy_warmup_observation:11, make_official_aloha_dummy_observation:22, validate_piper_semantic_observation:42；引用 laptop/client/bounded_model_trial.py, laptop/client/live_shadow.py 等 |
| `laptop/client/offline_latency_replay.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Time saved dual-camera observations over a prediction-only SSH tunnel.；_first_rgb:21, replay:33, main:87 |
| `laptop/client/policy_client_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Run sequential official openpi-client requests against localhost.；_percentile:18, run_roundtrip:22, main:77；引用 laptop/client/latency_probe.py, laptop/client/README.md 等 |
| `laptop/client/public_piper_transport_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Private-tunnel π0.5 request with synthetic input; never controls PiPER.；offline_veto:13, main:27；引用 docs/project-diagnosis-week-plan-2026-10-02.md |
| `laptop/client/requirements-dev.txt` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `laptop/client/requirements.txt` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/DIAGNOSTIC_SCRIPT_SPEC.md, docs/PROJECT_LAYOUT.md 等 |
| `laptop/client/shadow_preview_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Send a recent passive PiPER preview to a prediction-only policy server.；_client_factory:34, run:40, main:166；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `laptop/client/supervised_live_step.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Run one batch or a bounded, explicitly approved short PiPER policy session.；live_robot_status_ready:48, post_send_feedback_outcome:59, _metadata_identities:81；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/README.md 等 |
| `laptop/client/synthetic_policy_reject_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Exercise synthetic observation -> remote proxy -> SafetyBridge reject-only.；_client_factory:37, _check_loopback_listener:43, _bounded_prediction:48；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `laptop/control/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/control/contracts.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | TimingContract/Snapshot/Prediction/Proposal；_vector7:30, _finite_real:41, _canonical_int:50；引用 laptop/client/live_shadow.py, laptop/robot_client/policy_reject_only.py 等 |
| `laptop/diagnostics/OEM_CAN_CAPTURE.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md |
| `laptop/diagnostics/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `laptop/diagnostics/audit_feedback_log.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Offline P2 continuity check for an ACK-only feedback JSONL log.；audit:15, main:81；引用 laptop/diagnostics/compare_gripper_feedback.py, laptop/diagnostics/README.md 等 |
| `laptop/diagnostics/bounded_process.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | spawn worker父超时终止；_entry:10, run_bounded:19；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/bounded_model_trial.py 等 |
| `laptop/diagnostics/camera_identity.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 设备身份及角色核验；_parse_pnputil_camera_inventory:10, _pnputil_camera_inventory:35, windows_camera_inventory:43；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/bounded_model_trial.py 等 |
| `laptop/diagnostics/camera_stream_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Bounded dual-camera stream check without recording or assigning roles.；run:13, main:62；引用 docs/PROJECT_LAYOUT.md, tests/laptop/test_camera_stream_smoke.py |
| `laptop/diagnostics/can_diagnostic_session.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | SafetyBridge-owned session for explicitly approved PiPER getter CAN TX.；DiagnosticFeedbackPreflightError:27, DiagnosticQueryRejected:31, CanDiagnosticSession:37；引用 laptop/diagnostics/query_all_joint_parameters.py, laptop/diagnostics/query_joint_limits.py 等 |
| `laptop/diagnostics/capture_oem_can.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Observe OEM candle_frame_send for exactly a 5-second armed window.；decode_gripper:17, main:26；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/diagnostics/OEM_CAN_CAPTURE.md 等 |
| `laptop/diagnostics/compare_gripper_feedback.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Compare two passive CAN logs with an operator-observed OEM gripper display.；_settled_gripper:18, compare:30, main:67；引用 tests/laptop/test_compare_gripper_feedback.py |
| `laptop/diagnostics/list_candlelight_adapters.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | List candleLight USB bus/address identities without opening CAN.；main:10；引用 laptop/diagnostics/README.md, laptop/recorder/README.md |
| `laptop/diagnostics/list_directshow_cameras.ps1` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/camera/dual_camera.py 等 |
| `laptop/diagnostics/oem_can_send_hook.js` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 laptop/diagnostics/capture_oem_can.py |
| `laptop/diagnostics/probe_cameras.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Read-only laptop camera enumeration and ten-frame image probe.；_serial_candidates:17, _realsense_intrinsics:33, probe:48；引用 docs/DIAGNOSTIC_SCRIPT_SPEC.md, docs/LAPTOP_SETUP_PLAN.md |
| `laptop/diagnostics/probe_can_ack_only.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Five-second CAN receive probe that permits hardware ACK, never data TX.；probe:28, main:87；引用 tests/laptop/test_can_ack_only_guard.py |
| `laptop/diagnostics/probe_can_listen_only.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Bounded receive-only probe for one gs_usb CAN adapter.；probe:33, main:107；引用 tests/laptop/test_can_listen_only_guard.py |
| `laptop/diagnostics/probe_dual_cameras.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Probe two explicitly selected cameras without recording video or touching PiPER.；_percentile:22, _summary:34, _nearest_skew_ms:49；引用 docs/project-diagnosis-week-plan-2026-10-02.md, laptop/camera/dual_camera.py 等 |
| `laptop/diagnostics/probe_motor_enable_ack_only.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Bounded read-only PiPER six-motor enable status from 0x261..0x266.；decode_motor_enable:22, probe:29；引用 laptop/robot_client/piper_tool.py, laptop/robot_client/supervised_j1_one_degree_trial.py 等 |
| `laptop/diagnostics/probe_network.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Read-only localhost/SSH-tunnel health and one official payload probe.；dummy_aloha_observation:17, main:31；引用 docs/DIAGNOSTIC_SCRIPT_SPEC.md |
| `laptop/diagnostics/probe_piper.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Safe specification stub for a future read-only PiPER hardware audit.；main:48；引用 docs/DIAGNOSTIC_SCRIPT_SPEC.md, docs/LAPTOP_SETUP_PLAN.md 等 |
| `laptop/diagnostics/probe_piper_feedback_ack_only.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Decode bounded PiPER feedback without transmitting CAN data frames.；probe:38, main:240；引用 laptop/diagnostics/README.md, laptop/recorder/passive_capture.py 等 |
| `laptop/diagnostics/probe_piper_readonly.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Emit a future PiPER read-only audit contract without touching hardware.；main:51；引用 laptop/README_FIRST_CONNECTION.md, scripts/audit_laptop_probe.py |
| `laptop/diagnostics/query_all_joint_parameters.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Read PiPER's live joint limits and state using bounded CAN queries.；request_payload:32, decode_limit_payload:40, decode_acceleration_payload:58；引用 laptop/diagnostics/query_piper_readiness_snapshot.py, tests/laptop/test_diagnostic_queries.py 等 |
| `laptop/diagnostics/query_joint_limits.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Bounded non-motion PiPER joint-limit query (0x472 -> 0x473).；request_payload:19, decode_limit_payload:25, query:35；引用 tests/laptop/test_diagnostic_queries.py, tests/laptop/test_query_joint_limits.py |
| `laptop/diagnostics/query_piper_firmware.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Read the PiPER firmware version with one bounded documented CAN query.；decode_firmware_payloads:28, query:45, main:105；引用 laptop/diagnostics/query_piper_readiness_snapshot.py, tests/laptop/test_diagnostic_queries.py 等 |
| `laptop/diagnostics/query_piper_gripper_parameters.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Read PiPER gripper travel configuration after fresh feedback is present.；decode_gripper_parameter_payload:33, evaluate_feedback_preflight:51, dispatch_query_if_feedback_fresh:90；引用 laptop/diagnostics/query_piper_readiness_snapshot.py, tests/laptop/test_diagnostic_queries.py 等 |
| `laptop/diagnostics/query_piper_readiness_snapshot.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Collect one bounded firmware, joint, and gripper getter snapshot.；DiagnosticReportError:35, _stage_plan:39, _validated_tx_count:53；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `laptop/piper/README.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `laptop/piper/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/piper/unit_conversion.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | SDK名义单位双向换算；_scale_or_error:25, sdk_joint_to_rad:32, rad_to_sdk_joint:39；引用 laptop/piper/__init__.py, laptop/robot_client/policy_sdk_preview.py 等 |
| `laptop/recorder/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `laptop/recorder/USAGE.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md |
| `laptop/recorder/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/recorder/align_raw.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | host-arrival近邻对齐；_sha256:43, _inside:51, _lines:60；引用 docs/DATA_SCHEMA.md, docs/project-diagnosis-week-plan-2026-10-02.md 等 |
| `laptop/recorder/audit_episode.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Read-only integrity and timing audit of an existing raw episode.；main:12；引用 docs/project-diagnosis-week-plan-2026-10-02.md, laptop/recorder/README.md |
| `laptop/recorder/capture_session.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 独立episode/session和success标签；_sha256:26, _append:34, run_session:39；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/recorder/record_demonstrations.py 等 |
| `laptop/recorder/episode.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 完成/质量检查、边界帧判定；_write_json_line:21, EpisodeWriter:25, validate_episode:99；引用 docs/project-diagnosis-week-plan-2026-10-02.md, laptop/recorder/audit_episode.py 等 |
| `laptop/recorder/label_episode.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 人工success标签及来源验证；_sha256:14, label_episode:22, verify_label:66；引用 laptop/recorder/capture_session.py, laptop/recorder/next_state_proxy.py 等 |
| `laptop/recorder/manual_teach_capture.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Record one bounded operator-led raw demonstration; never command PiPER.；main:16；引用 laptop/recorder/README.md |
| `laptop/recorder/next_state_proxy.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | future-state proxy build；_sha256:25, _state:29, audit_transition_deltas:41；引用 configs/runtime/pi05_proxy_shadow_identity_lock.json, docs/CURRENT_PROJECT_PLAN.md 等 |
| `laptop/recorder/passive_capture.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 相机与CAN RX原始数据记录；assemble:27, capture:51, _capture_worker:207；引用 docs/project-diagnosis-week-plan-2026-10-02.md, laptop/recorder/capture_session.py 等 |
| `laptop/recorder/piper_command_targets.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Validate passive PiPER command-frame logs and derive timestamped targets.；CommandTarget:28, _can_id:52, _read_command_rows:64；引用 laptop/client/action_shadow_replay.py, laptop/recorder/align_raw.py 等 |
| `laptop/recorder/record_demonstrations.py` | ENTRYPOINT_ACTIVE | 保留 | 否/暂缓 | HIGH | 可复用手动示教/倒计时/identity入口；camera_identity:16, build_parser:27, main:43；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/bounded_model_trial.py 等 |
| `laptop/recorder/synthetic_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Generate a tiny synthetic episode to exercise files and rejection gates.；run:12, main:51；引用 laptop/recorder/README.md, server/dataset_tools/synthetic_lerobot_smoke.py 等 |
| `laptop/recorder/verify_source.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 原始文件/hash/source关系审核；_hash:25, _video_frames:29, verify_source:50；引用 docs/DATA_SCHEMA.md, laptop/client/action_shadow_replay.py 等 |
| `laptop/robot_client/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md |
| `laptop/robot_client/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/robot_client/bounded_policy_target.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | limit_target把proxy解释为有界名义目标；limit_target:5；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/bounded_model_trial.py 等 |
| `laptop/robot_client/bounded_zero_return.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Authorized joint zero return in 5-degree segments, preserving gripper position.；zero_target:16, execute:22, main:65；引用 docs/CURRENT_PROJECT_PLAN.md, tests/laptop/test_bounded_zero_return.py |
| `laptop/robot_client/gripper_command_preview.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Offline-only preview of the PiPER V2 0x159 gripper target frame.；GripperPreview:14, preview_opening:22；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/supervised_restore_gripper.py 等 |
| `laptop/robot_client/hil_guard.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | reject_legacy_raw_can_tx在设备打开前阻断；LegacyHILBlockedError:10, reject_legacy_raw_can_tx:19；引用 laptop/robot_client/supervised_j1_one_degree_trial.py, laptop/robot_client/supervised_j1_step_trial.py 等 |
| `laptop/robot_client/joint_command_preview.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Pure PiPER joint-command preview; there is deliberately no CAN sender.；CanFramePreview:19, JointStepPreview:25, _joints:32；引用 laptop/robot_client/policy_sdk_preview.py, laptop/robot_client/single_step_dry_run.py 等 |
| `laptop/robot_client/offline_replay.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Replay recorded candidate actions through the offline guard without sending.；FakeRobotTransport:24, OfflineReplayClient:39, replay:82；引用 docs/project-diagnosis-week-plan-2026-10-02.md, laptop/robot_client/README.md 等 |
| `laptop/robot_client/piper_session.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | poll/state/typed send与TX/RX日志；PiperSession:19；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/bounded_model_trial.py 等 |
| `laptop/robot_client/piper_tool.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Reusable, bounded PiPER operations with exclusive per-invocation reports.；parser:20, dispatch:49, main:86；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/README.md |
| `laptop/robot_client/policy_reject_only.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Replay a saved shadow report through SafetyBridge with CAN TX fixed at zero.；_decision_dict:21, _canonical_audit_value:30, _malformed_decision:48；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/shadow_preview_smoke.py 等 |
| `laptop/robot_client/policy_sdk_preview.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Non-executable mapping preview from Pi0.5's 7D output to PiPER V2 SDK values.；PolicySdkPreview:34, _future_action_index:64, preview_policy_sdk_values:71；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/supervised_policy_step.py 等 |
| `laptop/robot_client/replay_preflight.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | SDK名义范围、replay数据检查；_frame:23, analyze:37, main:104；引用 laptop/robot_client/bounded_policy_target.py, laptop/robot_client/joint_command_preview.py 等 |
| `laptop/robot_client/single_step_dry_run.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Offline single-step proposal rehearsal through SafetyBridge only.；P12GateEvidence:30, preflight_hil_single_step:69, rehearse_single_step:183；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/supervised_policy_session.py 等 |
| `laptop/robot_client/supervised_axis_commissioning.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | One separately approved 1-degree commissioning step on J2-J6.；execute:9, main:31；引用 laptop/robot_client/piper_tool.py |
| `laptop/robot_client/supervised_enable_j6.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | One approved EnableArm(6), with fresh zero-pose and six-motor readback.；execute:13, main:60；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/piper_tool.py 等 |
| `laptop/robot_client/supervised_gripper_commissioning.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | One approved nominal 1 mm gripper opening with persistent raw bus audit.；execute:11；引用 laptop/robot_client/piper_tool.py, laptop/robot_client/README.md |
| `laptop/robot_client/supervised_gripper_max_open.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | One explicitly authorized nominal 70mm gripper opening at 0.1Nm.；execute:12, main:35 |
| `laptop/robot_client/supervised_j1_commissioning.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | One approved +0.100 or +1.000-degree J1 reference step.；_read_for:40, _feedback_outcome:60, execute:116；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/bounded_model_trial.py 等 |
| `laptop/robot_client/supervised_j1_one_degree_trial.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；preflight:23, execute:42, main:116；引用 tests/laptop/test_legacy_hil_guard.py, tests/laptop/test_supervised_j1_one_degree_trial.py |
| `laptop/robot_client/supervised_j1_step_trial.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；Observed:31, preflight:50, _read_for:78；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/supervised_j1_one_degree_trial.py 等 |
| `laptop/robot_client/supervised_j5_probe.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；execute:22, main:95；引用 tests/laptop/test_legacy_hil_guard.py, tests/laptop/test_supervised_j5_probe.py |
| `laptop/robot_client/supervised_joint_replay.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；load_targets:36, interpolate_targets:60, execute:75；引用 tests/laptop/test_legacy_hil_guard.py |
| `laptop/robot_client/supervised_mode_initialization.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Operator-approved reset -> standby -> CAN; reset may cause the arm to drop.；_read_for:14, execute:20, main:69；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/piper_tool.py 等 |
| `laptop/robot_client/supervised_near_zero_home.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Single authorized near-zero return followed by empty-gripper closure; no retry.；execute:15, healthy:55, main:61；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/supervised_gripper_max_open.py 等 |
| `laptop/robot_client/supervised_oem_gripper_test.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Bounded test using observed OEM effort=0 and repeated gripper targets.；execute:14, main:51；引用 docs/CURRENT_PROJECT_PLAN.md |
| `laptop/robot_client/supervised_policy_session.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | One-step operator consent bound to a policy identity and safety envelope.；OneStepOperatorGrant:44, ShortLoopOperatorGrant:242, ShortLoopOperatorGrantTicket:506；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/supervised_live_step.py 等 |
| `laptop/robot_client/supervised_policy_step.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | prepare/send单次正式授权目标；PreparedSupervisedPolicyStep:37, prepare_supervised_policy_step:63, send_prepared_supervised_policy_step:127；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/supervised_live_step.py 等 |
| `laptop/robot_client/supervised_restore_gripper.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；_read_for:26, execute:46, main:116；引用 docs/CURRENT_PROJECT_PLAN.md, tests/laptop/test_legacy_hil_guard.py 等 |
| `laptop/robot_client/supervised_restore_start.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；recorded_start_raw:35, plan_waypoints:51, checked_feedback:72；引用 laptop/robot_client/supervised_j5_probe.py, laptop/robot_client/supervised_joint_replay.py 等 |
| `laptop/robot_client/supervised_site_profile.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | Load an operator-maintained PiPER commissioning record.；SupervisedSiteProfile:43, _reject_duplicate_keys:57, _reject_non_finite_number:66；引用 configs/runtime/supervised_site_profile.template.json, docs/CURRENT_PROJECT_PLAN.md 等 |
| `laptop/robot_client/supervised_smooth_joint_replay.py` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 旧direct sender execute先guard拒绝；tests/helper引用，暂保留；load_keyframes:37, knot_slopes:60, hermite:78；引用 docs/CURRENT_PROJECT_PLAN.md, tests/laptop/test_legacy_hil_guard.py |
| `laptop/robot_client/verified_policy_command.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | P12 verified preview→CommandProposal；command_proposal_from_verified_preview:26；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/supervised_policy_step.py 等 |
| `laptop/runtime/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/runtime/bridge_protocol.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | bridge/policy元数据和时间契约；encode_message:22, decode_message:36；引用 tests/laptop/test_bridge_protocol.py |
| `laptop/runtime/config.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | JSON加载及mode/TX/identity校验；RuntimeConfig:13；引用 laptop/client/shadow_preview_smoke.py, laptop/client/synthetic_policy_reject_smoke.py 等 |
| `laptop/runtime/modes.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | RuntimeMode/BridgeState枚举；RuntimeMode:6, require_offline_mode:24；引用 laptop/client/bounded_model_trial.py, laptop/client/live_shadow.py 等 |
| `laptop/safety/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `laptop/safety/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/safety/bridge.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | SafetyBridge状态机、freshness、授权及typed TX；BridgeState:44, SafetyLimits:53, BridgeDecision:133；引用 laptop/client/bounded_model_trial.py, laptop/client/live_shadow.py 等 |
| `laptop/safety/offline_guard.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Pure fail-closed validation of a candidate PiPER 7D position action.；SafetyPolicy:17, Decision:56, _vector:62；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/client/audit_live_shadow.py 等 |
| `laptop/test_policy_tunnel.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Laptop-side SSH-tunnel readiness check; never a robot test.；main:13；引用 laptop/README_FIRST_CONNECTION.md |
| `laptop/transport/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `laptop/transport/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `laptop/transport/bounded_trial.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | BoundedTrialBatch的范围/TTL/4帧目标；BoundedTrialBatch:7；引用 laptop/client/bounded_model_trial.py, laptop/robot_client/bounded_zero_return.py 等 |
| `laptop/transport/can_transport.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | CanFrame/抽象transport/fake接口；TransportOwnershipError:26, CanFrame:31, PiperCanTransport:53；引用 laptop/client/live_shadow.py, laptop/diagnostics/can_diagnostic_session.py 等 |
| `laptop/transport/diagnostic_queries.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | getter strict allowlist；DiagnosticCanQuery:15, firmware_version_query:64, joint_limit_query:68；引用 laptop/diagnostics/can_diagnostic_session.py, laptop/diagnostics/query_all_joint_parameters.py 等 |
| `laptop/transport/gripper_commissioning.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 单步/开闭端点/OEM限定帧；OemGripperTestFrame:8, GripperOpeningStep:30；引用 laptop/robot_client/piper_tool.py, laptop/robot_client/README.md 等 |
| `laptop/transport/gs_usb_piper_can.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | candleLight RX/typed TX/owner/unknown-delivery fault；CanAdapterNotUniqueError:35, CanBitrateError:41, CanListenOnlyUnsupportedError:45；引用 laptop/client/bounded_model_trial.py, laptop/client/live_shadow.py 等 |
| `laptop/transport/mode_initialization.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 模式/reset/J6 enable typed frames；ModeInitializationFrame:15, J6EnableFrame:35, feedback_snapshot:47；引用 docs/CURRENT_PROJECT_PLAN.md, laptop/robot_client/piper_session.py 等 |
| `laptop/transport/position_commands.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | SDK编码、snapshot/history/授权验证；CanTxFrame:60, PositionFrameWrite:74, J1CommissioningSnapshot:89；引用 laptop/robot_client/supervised_j1_commissioning.py, laptop/robot_client/supervised_near_zero_home.py 等 |
| `provenance/deployment_manifest.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 scripts/add_final_gate_aliases.py, scripts/write_deployment_manifest.py |
| `provenance/gpu_assignment.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 reports/environment_audit.json |
| `provenance/openpi_contract.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 configs/project_contract.yaml, docs/OPENPI_PROVENANCE.md 等 |
| `provenance/openpi_git.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 AGENTS.md, docs/PI05_PIPER_MASTER_PLAN.md 等 |
| `provenance/pi05_base_checkpoint.json` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/PI05_PIPER_MASTER_PLAN.md, provenance/deployment_manifest.json 等 |
| `reports/P0_P1_DEPLOYMENT_REPORT.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `reports/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `reports/environment_audit.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/environment_audit_initial.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/local_client_performance_25.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/local_client_roundtrip.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/local_policy_server_smoke.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/pi05_base_inference_smoke.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/pi05_official_config_audit.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `reports/pi05_server_performance.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `reports/piper_gripper_teach_audit.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `reports/piper_teach_mode_audit.md` | HISTORICAL | 保留 | 否/暂缓 | HIGH | 历史初装/计划/审计/初始化意图，独特来源不可由当前report替代；详第15节 |
| `scripts/README.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `scripts/add_final_gate_aliases.py` | ARCHIVE_CANDIDATE | 保留；仅后续批准迁移 | 条件式候选 | HIGH | 字节重复全文或固定历史报告补丁；详第15节条件式归档证据；main:10 |
| `scripts/annotate_pi05_smoke_metadata.py` | ARCHIVE_CANDIDATE | 保留；仅后续批准迁移 | 条件式候选 | HIGH | 字节重复全文或固定历史报告补丁；详第15节条件式归档证据；main:10 |
| `scripts/audit_laptop_probe.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Static safety audit for the project-local PiPER read-only probe.；audit:26, main:49 |
| `scripts/refresh_environment_audit.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Add the uv and final host-driver fields to the environment audit.；main:12 |
| `scripts/run_local_policy_server_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Start, audit, exercise, and cleanly stop the local policy server.；socket_snapshot:21, wait_for_listen:27, main:45；引用 scripts/run_server_performance.py |
| `scripts/run_pi05_base_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Official pi05_base + pi05_aloha load and inference smoke.；_gpu_memory_mib:28, main:47 |
| `scripts/run_server_performance.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Measure a 25-request official base server run with GPU sampling.；gpu_sample:17, main:37 |
| `scripts/validate_piper_continuous.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Overwrite the unit report with continuous random-value quantization checks.；main:21 |
| `scripts/validate_piper_contract.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Generate synthetic P1 PiPER contract validation artifacts.；write:19, main:24 |
| `scripts/write_checkpoint_provenance.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Inventory and hash the project-local pi05_base checkpoint cache.；sha256_file:12, main:20 |
| `scripts/write_deployment_manifest.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Write the final P0/P1 deployment provenance manifest.；sha256:18, command:26, main:33 |
| `scripts/write_socket_binding_report.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Persist the observed localhost-only binding from the server smoke.；main:10 |
| `server/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `server/dataset_tools/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `server/dataset_tools/audit_proxy_lerobot.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Audit an offline proxy LeRobot dataset and optionally attach row-level provenance.；_verify_source_audit:27, _read_parquet_and_verify:57, _decode_and_verify_videos:94；引用 tests/data/test_proxy_lerobot_audit.py |
| `server/dataset_tools/audit_training_data.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Audit an immutable LeRobot dataset, split by episode, and hash-bind stats.；_sha256:18, _files:26, _digest:41；引用 docs/DATA_SCHEMA.md, server/dataset_tools/README.md 等 |
| `server/dataset_tools/compare_proxy_sampling_rates.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Verify paired 2 fps and higher-rate proxy variants from one raw episode.；_sha256:24, _json:32, _variant:39；引用 tests/data/test_proxy_lerobot_audit.py, tests/data/test_proxy_sampling_rate_comparison.py |
| `server/dataset_tools/convert_next_state_proxy.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | convert_many多episode LeRobot/source_map；sha256:25, _file:33, _seven:42；引用 docs/CURRENT_PROJECT_PLAN.md, server/dataset_tools/audit_proxy_lerobot.py 等 |
| `server/dataset_tools/convert_piper_aligned.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Validate reviewed PiPER aligned episodes, then derive a local LeRobot dataset.；_sha256:43, _contained_file:51, _seven:60；引用 docs/DATA_SCHEMA.md, docs/project-diagnosis-week-plan-2026-10-02.md 等 |
| `server/dataset_tools/synthetic_lerobot_smoke.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | Tiny synthetic LeRobot write/read test in a temporary directory only.；run:22, main:67 |
| `server/piper_policy/__init__.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 包标识/导出，保留兼容imports |
| `server/piper_policy/piper_data_config.py` | EXPERIMENTAL | 保留 | 否/暂缓 | MEDIUM | DataConfig placeholder for a future PiPER-specific OpenPI training run.；PiperDataConfig:9 |
| `server/piper_policy/piper_inputs.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Semantic PiPER input contract and official OpenPI-compatible preprocessing.；_canonical_image:25, PiperInputs:37；引用 scripts/validate_piper_contract.py, server/piper_policy/__init__.py 等 |
| `server/piper_policy/piper_outputs.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | PiPER action padding and canonical output contract.；_validate_action_array:19, action_pad_mask:31, pad_action_7_to_model:39；引用 scripts/validate_piper_contract.py, server/piper_policy/__init__.py 等 |
| `server/piper_policy/piper_policy_config.py` | EXPERIMENTAL | 保留 | 否/暂缓 | MEDIUM | PiPER policy initialization metadata; training is intentionally disabled.；PiperPolicyConfig:9 |
| `server/policy_server/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `server/policy_server/artifact_identity.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | artifact hash/identity支持；sha256_file:10, sha256_text_file:22, tree_sha256:32；引用 configs/runtime/pi05_proxy_shadow_identity_lock.json, docs/CURRENT_PROJECT_PLAN.md 等 |
| `server/policy_server/proxy_config.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Strict loader for the prediction-only PiPER proxy server configuration.；ProxyPolicyConfig:28；引用 server/serve_proxy_piper_shadow.py, tests/server/test_proxy_policy_config.py |
| `server/public_piper_smoke_infer.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | One prediction from a public LeRobot sample; no robot interfaces are imported.；main:14 |
| `server/serve_derived_state_multi_shadow.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | inspect_artifacts/create_server/hash/loopback限制；_sha256:33, validate_runtime:41, validate_training_record:51；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `server/serve_piper_action_shadow.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Loopback-only server for a fine-tuned PiPER action checkpoint.；build_policy:35, main:125；引用 server/training/README.md |
| `server/serve_policy_local.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Run the official OpenPI WebSocket server on localhost only.；main:28；引用 docs/PI05_PIPER_MASTER_PLAN.md, provenance/deployment_manifest.json 等 |
| `server/serve_proxy_piper_shadow.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Loopback-only PiPER proxy policy server for prediction-only shadow tests.；main:38；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `server/serve_public_piper_smoke.py` | EXPERIMENTAL | 保留 | 否/暂缓 | MEDIUM | Serve the public-data pi0.5 smoke policy on localhost, prediction only.；main:14 |
| `server/training/README.md` | STALE_DOCUMENT | 保留 | 否/暂缓 | HIGH | 旧状态/入口说明滞后，仍有契约或导航价值；详第14节 |
| `server/training/TRAINING_USAGE.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；引用 docs/CURRENT_PROJECT_PLAN.md |
| `server/training/action_target_contract.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Semantic gates separating observed-state targets from recorded commands.；is_verified_raw_command_episode:19, has_raw_command_action_semantics:30；引用 server/training/check_piper_readiness.py, tests/server/test_action_target_contract.py |
| `server/training/check_piper_readiness.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Mode-specific, read-only offline training readiness checks.；_json:21, _sha256:29, inspect:36；引用 docs/DATA_SCHEMA.md, docs/project-diagnosis-week-plan-2026-10-02.md 等 |
| `server/training/evaluate_demonstrations.py` | ENTRYPOINT_ACTIVE | 保留 | 否/暂缓 | HIGH | COMPLETE report→same-data预测评估；main:6；引用 docs/CURRENT_PROJECT_PLAN.md, server/training/TRAINING_USAGE.md |
| `server/training/piper_action_eval.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Offline PiPER command-target evaluation; predictions are never sent to CAN.；_sha256:15, _dataset_tree_sha256:23, _check_checkpoint_provenance:37；引用 server/training/README.md, tests/server/test_piper_action_eval.py |
| `server/training/piper_openpi_config.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | PiperOpenPIInputs/Outputs/DataConfig；_image:24, _vector:39, PiperOpenPIInputs:50；引用 docs/project-diagnosis-week-plan-2026-10-02.md, server/dataset_tools/synthetic_lerobot_smoke.py 等 |
| `server/training/piper_train_entry.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Mode-gated offline OpenPI training; never connects to a robot.；_sha256_file:22, _dataset_tree_identity:30, build_config:46；引用 docs/project-diagnosis-week-plan-2026-10-02.md, server/serve_piper_action_shadow.py 等 |
| `server/training/post_capture_pipeline.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | prepare + 默认20步/上限100步pilot；_sha256:48, _utc_run_id:56, _repo_id:60；引用 docs/CURRENT_PROJECT_PLAN.md, docs/PROJECT_LAYOUT.md 等 |
| `server/training/proxy_dataset_contract.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Validate immutable, source-audited proxy LeRobot datasets without OpenPI.；ProxyDatasetSpec:24, sha256_file:83, dataset_tree_sha256:91；引用 server/training/proxy_piper_eval.py, server/training/proxy_piper_tiny_fit.py 等 |
| `server/training/proxy_latency_profile.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Offline loopback latency profile of the pinned PiPER proxy checkpoint.；profile:19, main:103；引用 tests/server/test_proxy_latency_profile.py |
| `server/training/proxy_multi_episode_contract.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | 成功多episode、10fps/action/source校验；_sha256:27, dataset_tree_sha256:35, _json:56；引用 docs/CURRENT_PROJECT_PLAN.md, server/serve_derived_state_multi_shadow.py 等 |
| `server/training/proxy_multi_episode_pilot.py` | CORE_ACTIVE | 保留 | 否/暂缓 | HIGH | build_config/execute/evaluate/stats/provenance；_sha256:32, _tree_sha256:40, _resolve_dataset:59；引用 docs/CURRENT_PROJECT_PLAN.md, server/serve_derived_state_multi_shadow.py 等 |
| `server/training/proxy_piper_eval.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Offline same-episode proxy comparison; never sends predictions to PiPER.；main:15；引用 docs/CURRENT_PROJECT_PLAN.md, server/training/README.md |
| `server/training/proxy_piper_tiny_fit.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Bounded pi0.5 tiny-fit of one proxy episode; prediction only, never robot control.；_sha256:33, _inspect_2fps_dataset:37, _spec:60；引用 docs/CURRENT_PROJECT_PLAN.md, server/serve_proxy_piper_shadow.py 等 |
| `server/training/public_data_contract.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Explicitly distinguish public PiPER smoke data from local robot actions.；inspect_public_acceptance:11；引用 laptop/client/public_piper_transport_smoke.py, server/serve_public_piper_smoke.py 等 |
| `server/training/public_piper_smoke.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Bounded public-data pi0.5 smoke; never imports a robot or sends CAN.；PublicPiperDataConfig:36, make_config:51, inspect_dataset:83；引用 docs/project-diagnosis-week-plan-2026-10-02.md, server/public_piper_smoke_infer.py 等 |
| `server/training/serve_trained_demonstrations.py` | ENTRYPOINT_ACTIVE | 保留 | 否/暂缓 | HIGH | report→policy/identity/background服务；main:5；引用 docs/CURRENT_PROJECT_PLAN.md |
| `server/training/summarize_training_loss.py` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | HIGH | Reusable loss summary, CSV and optional curve from OpenPI training logs.；summarize:5, main:10；引用 docs/CURRENT_PROJECT_PLAN.md, server/training/TRAINING_USAGE.md |
| `server/training/train_demonstrations.py` | ENTRYPOINT_ACTIVE | 保留 | 否/暂缓 | HIGH | foreground/background GPU/steps/report；parser:8, run:26, main:61；引用 docs/CURRENT_PROJECT_PLAN.md, server/training/TRAINING_USAGE.md 等 |
| `testfoo.txt` | UNKNOWN | 保留 | 人工复核 | LOW | 非PiPER主链或用途未建立；人工确认，禁止自动清理 |
| `tests/README.md` | SUPPORT_ACTIVE | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `tests/__init__.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/contracts/__init__.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/contracts/test_control_contracts.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/contracts/test_piper_contract.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/contracts/test_piper_units.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/contracts/test_runtime_config.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/__init__.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_align_raw.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_align_raw_to_lerobot.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_convert_next_state_proxy.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_next_state_proxy.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_piper_command_targets.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_proxy_lerobot_audit.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_proxy_sampling_rate_comparison.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/data/test_source_verification.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/__init__.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_action_shadow_replay.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_bounded_model_trial.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_bounded_policy_target.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_bounded_process.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_bounded_zero_return.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；test_zero_segments_converge_without_overshoot:4, test_invalid_joint_vector_rejected:12 |
| `tests/laptop/test_bridge_protocol.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_camera_identity_inventory.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_camera_stream_smoke.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_can_ack_only_guard.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_can_listen_only_guard.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_capture_boundary_frames.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_capture_supervision.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_compare_gripper_feedback.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_diagnostic_queries.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_dual_camera_capture.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_dual_camera_probe.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_episode_recorder.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_fake_can_single_step_rehearsal.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_feedback_log_audit.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_firmware_query.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_gripper_command_preview.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_gripper_commissioning.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_gripper_parameter_query.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_j1_commissioning_bridge.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_joint_command_preview.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_legacy_hil_guard.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_live_shadow.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_near_zero_home.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_network_latency_probe.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_oem_can_capture.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_oem_gripper_test.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_offline_safety_bridge.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_passive_capture.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_piper_can_transport.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_piper_feedback_ack_only_guard.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_piper_session.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_piper_tool.py` | HIL_TOOL_ACTIVE | 保留 | 否/暂缓 | HIGH | 辅助配置/文档/工具；用途依据正文与调用方；test_existing_report_prevents_device_access:6, test_step_routes_exact_scope:16, test_read_approval_is_not_implicit:29 |
| `tests/laptop/test_policy_reject_only.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_policy_sdk_preview.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_probe_motor_enable_ack_only.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_process_exit_watchdog.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_query_all_joint_parameters.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_query_joint_limits.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_query_piper_readiness_snapshot.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_record_demonstrations.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_replay_preflight.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_safety_bridge_runtime.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_shadow_preview_smoke.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_single_step_dry_run.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_enable_j6.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_j1_one_degree_trial.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_j1_step_trial.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_j5_probe.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_live_step.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_mode_initialization.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_policy_session.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_policy_step.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_position_bridge.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_position_transport.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_restore_gripper.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_restore_start.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_supervised_site_profile.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_synthetic_policy_reject_smoke.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/laptop/test_verified_policy_command.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/__init__.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_action_target_contract.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_derived_state_multi_shadow.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_piper_action_eval.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_piper_aligned_converter.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_piper_openpi_offline.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_piper_train_entry.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_policy_artifact_identity.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_post_capture_pipeline.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_proxy_dataset_contract.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_proxy_latency_profile.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_proxy_multi_episode_contract.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_proxy_policy_config.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_public_data_contract.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_train_demonstrations.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_training_data_audit.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `tests/server/test_training_modes.py` | TEST_ACTIVE | 保留 | 否/暂缓 | HIGH | 软件回归/测试包；静态fake/guard路径核查，本轮未执行，不授予hardware权限 |
| `validation/action_target_raw_corpus_inventory_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/action_target_source_audit_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_acer_device_recheck_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_acer_directshow_bind_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_acer_format_probe_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_acer_msmf_probe_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_acer_wrist_recheck_after_broadcast_probe_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_after_broadcast_exit_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_after_broadcast_stopped_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_base_after_container_stop_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_base_after_windows_camera_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_base_retry_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_base_retry_runner_20261007_r1.py` | EXPERIMENTAL | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `validation/camera_broadcast_ab_diagnostic_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_broadcast_wrist_stream_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_directshow_enumeration_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_directshow_inventory_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_dual_unsandboxed_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_high_index_scan_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_identity_inventory_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_index2_wrist_candidate_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_index_scan_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_index_scan_runner_20261007_r1.py` | EXPERIMENTAL | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `validation/camera_initial_debug_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_initial_debug_summary_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_skew_test_20261008_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_access_recheck_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_after_broadcast_off_1_2_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_after_broadcast_off_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_backend_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_current_0_1_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_current_1_2_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_index_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_properties_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_qualification_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_qualified_current_1_2_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_quality_guard_validation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_smoke_0_1_30s_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_smoke_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_smoke_20261006_0835.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_stream_smoke_indices_1_2_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/camera_wrist_probe_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/can_ack_only_recheck_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/can_bus_after_interrupt5000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/can_joint_parameters_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/can_passive_feedback_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/can_passive_feedback_live_recheck_20261006_0838.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/can_passive_feedback_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/cef166e2_visual_alignment_20261003.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/control_contract_type_validation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/derived_state_shadow_identity_lock_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/derived_state_shadow_loopback_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/derived_state_shadow_loopback_20261007.py` | EXPERIMENTAL | 保留 | 否/暂缓 | HIGH | Run a prediction-only WebSocket loopback on three saved LeRobot frames.；_sha256:29, _load_json:37, _install_import_paths:41；引用 docs/CURRENT_PROJECT_PLAN.md |
| `validation/derived_state_tiny_fit_reuse_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/diagnostic_query_bridge_validation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/dual_camera_reusable_live_20261007_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/dual_camera_reusable_live_20261007_r2/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/enable_j6_before5000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/eval1000_report_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/eval20_vs1000_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/feedback_change_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/first_demo_alignment_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/first_demo_reaudit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/gripper_parameter_query_offline_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/hil_entrypoint_bridge_gate_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/inspect_cando_function.py` | EXPERIMENTAL | 保留 | 否/暂缓 | MEDIUM | 辅助配置/文档/工具；用途依据正文与调用方 |
| `validation/j1_commissioning_bridge_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/j1_preflight_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/j2_j3_limit_query_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/joint_limit_software_display_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_camera_preflight_20261006_170029.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_can_readonly_preflight_20261006_170113.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_firmware_query_20261006_170233.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_gripper_closed_feedback_20261006_171202.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_gripper_endpoint_feedback_comparison_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_gripper_fully_open_feedback_20261006_171731.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_gripper_parameter_read_20261006_170212.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_gripper_signed_feedback_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_joint_parameter_read_20261006_170250.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_live_shadow_20261006_170525.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_live_shadow_channel_diagnostics_20261006_1836.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_live_shadow_post_timing_contract_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_live_shadow_signed_gripper_recheck_20261006_1840.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_live_shadow_startup_fix_20261006_1719.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_live_shadow_startup_fix_20261006_172633.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/lab_passive_feedback_recheck_post_restart_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/laptop_probe_static_audit.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live1000_identity_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live1000_preview_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live5000_before60deg_retry_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live5000_identity_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live5000_preview_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live5000_preview_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live5000_restart_r2_identity_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live_shadow_recheck_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/live_shadow_recheck_20261006_0852.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/measured_policy_update_rate_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/mode_initialization_preparation_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/mode_initialization_preview_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/motor_before5000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/motor_mode_check_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/network_latency_probe_unit_tests_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/network_latency_ssh_tunnel_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/network_latency_ssh_tunnel_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_after_model_step2_readonly_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_after_model_step_readonly_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_bounded_preview_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_live_at_zero_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_live_identity_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_live_preview_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_live_preview_20261007_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_model_trial_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_model_trial_20261007_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_model_trial_20261007_r3.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_model_trial_20261007_r4.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_model_trial_20261007_r5.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_model_trial_summary_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_server_dataset_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_tinyfit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_demo_tinyfit_20261007_corrected.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_four_demo_audit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/new_model_mode_init_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/non_camera_deployment_followup_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/non_can_readiness_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/non_can_readiness_supplement_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/nvidia_broadcast_camera_input_recheck_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/oem_gripper_capture_20261007_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/oem_gripper_capture_20261007_r2/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/oem_gripper_capture_20261007_r3/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/oem_gripper_capture_analysis_20261007_r3.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/oem_hook_probe_20261007_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/official_sdk_semantics_audit_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_contract_and_hil_guard_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_readiness_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_safety_configuration_validation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_identity_update_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_mainline_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_network_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_network_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_project_layout_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_server_work_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/offline_test_execution_watchdog_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/p12_offline_preflight_timing_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/p12_proxy_target_source_audit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/passive_can_probe_safety_bridge_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_action_transform_test.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_control_contract_static_20261002.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_feedback_ack_only_recheck_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_feedback_ack_only_timestamped_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_feedback_listen_only_recheck_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_feedback_listen_only_timestamped_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_feedback_timing_audit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_input_contract_smoke.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_probe_not_run.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/piper_unit_roundtrip.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/policy_reject_only_offline_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/post_capture_tinyfit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/process_exit_watchdog_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_10fps_tiny_fit_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_10fps_training_preflight_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_2fps_baseline_audited_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_2fps_server_eval_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_artifact_identity_lock_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_latency_segments_20261003.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_sampling_rate_comparison_20261005.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_target_step_envelope_audit_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/proxy_tiny_fit_comparison_20261003.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/public_piper_smoke.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/recorded_preview_reject_replay_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/safety_bridge_clock_validation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/safety_bridge_offline_audit_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/safety_bridge_state_machine_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_authorized_offline_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_current_local_tiny_fit_gate_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_current_survivor_8003_synthetic_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_current_synthetic_reject_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_current_synthetic_reject_continuation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_current_synthetic_reject_followup_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_current_synthetic_request_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_data_readiness_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_openpi_environment_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_post_restart_synthetic_reject_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_proxy_deployment_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_proxy_prediction_only_live_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_proxy_prediction_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_raw_episode_upload_recheck_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/server_service_consolidation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_1000_preflight_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_after_gripper_5mm_readonly_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_after_j1_one_degree_readonly_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_after_j1_readonly_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_after_j2_readonly_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_close_at_zero_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_feedback_check_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_gripper_max_open_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_gripper_max_open_20261007_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_gripper_max_open_observe_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_gripper_max_open_observe_20261007_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_gripper_open_1mm_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_gripper_open_5mm_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_home_and_close_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j1_commissioning_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j1_one_degree_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j2_one_degree_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j3_one_degree_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j4_one_degree_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j5_one_degree_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j6_enable_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_j6_one_degree_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_motor_enable_check_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_oem_gripper_test_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_oem_gripper_test_20261007_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_oem_test_gripper_parameters_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_pre_home_status_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_seven_actuator_response_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_zero_motor_enable_check_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/site_zero_pose_check_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/socket_binding.txt` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/ssh_tunnel_drop_recovery_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/status_after_interrupt5000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/status_after_interrupt5000_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/status_before5000_restart_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/status_mode_check_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/supervised_adapter_offline_recheck_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/supervised_live_step_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/supervised_post_send_feedback_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/supervised_short_loop_grant_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/supervised_site_profile_overlay_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/synthetic_policy_reject_smoke_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_model_preflight_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_scene_1000_after_20261008_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_scene_1000_preflight_20261008_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_scene_after_segment1_20261007/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_scene_preflight_20261007_r1/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_scene_preflight_20261007_r2/report.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_5deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_5deg_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long10deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long10deg_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long10deg_20261008_r1/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long20deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long20deg_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long20deg_20261008_r1/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long20deg_nocumulative_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long20deg_nocumulative_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long20deg_nocumulative_20261008_r1/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long5deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long5deg_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long5deg_20261008_r1/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_long_20261008_r1/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial1000_outcome_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_60deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_60deg_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_60deg_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_60deg_20261008_r2/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_60deg_20261008_r2/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r2/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r3.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r3/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r4.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r4/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r4/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r5.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r5/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r6.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long20deg_nocumulative_20261008_r6/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_long_20261008_attempts_r5_r6.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r1/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r2/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r2/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r3.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r3/progress.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial5000_restart_long_20261008_r3/summary.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial_20261007_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/task_trial_outcome_20261007.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/train1000_loss_summary_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/train1000_report_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/train5000_loss_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/train5000_report_20261008.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/transport_unification_validation_20261006.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_before20deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_before20deg_nocumulative_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_before5000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_before5000_60deg_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_before5000_60deg_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_before5000_long_20261008_r2.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `validation/zero_interrupt5000_20261008_r1.json` | GENERATED | 保留 | 否/暂缓 | HIGH | 生成的审计/训练/实测证据；仅对应该次运行，保留来源链 |
| `world_land_sea_90x180.csv` | UNKNOWN | 保留 | 人工复核 | LOW | 非PiPER主链或用途未建立；人工确认，禁止自动清理；引用 world_land_sea_reconstruction.py |
| `world_land_sea_reconstruction.py` | UNKNOWN | 保留 | 人工复核 | LOW | 非PiPER主链或用途未建立；人工确认，禁止自动清理；in_polygon:116, in_ellipse:129, classify:134 |

### Git readiness and evidence retention

仓库已初始化；本轮没有Git mutation，以下是下一阶段审查事项。

- 当前HEAD仅bootstrap，大量有效模块untracked，单记HEAD不足以重建当前部署。
- .gitignore已排除venv/cache/checkpoints/raw/incoming/lerobot/常规logs和key/token类文件；
  但validation CAN JSONL、AVI/图像目录、capture_sessions、Log及部分pytest目录未完整覆盖。
- tracked清单未发现>100MiB文件或已track的原始视频/大模型权重/venv；
  含checkpoint名称的provenance JSON不是weights本身。
- 这是路径/类型/大小核查，不是完整秘密内容扫描；本轮没有读取/输出凭据。
- 两个tracked tests旧路径已删除，现有tests/contracts中对应测试存在；属审计前改动。
- OpenPI官方submodule的gitlink pin和服务器HEAD一致，本轮不修改、替换或推送上游。
- 可开始状态/来源与链接整理，尚不建议批量archive、控制代码大规模迁移或直接git add全部。

### Source snapshot for traceability

核心地图中的源码SHA记录如下；以后源码变化需重新核对相关结论。
服务器抽查train wrapper、pilot、OpenPI config、serve wrapper和multi shadow五项hash与本地一致。
checkpoint/tree哈希来自training report，不在此冒充重新全树计算。

| Path | Local SHA-256 |
|---|---|
| `laptop/camera/dual_camera.py` | 96c849e03a1b6324ede68ed55d6cd684d0222c2ff29b8a51f5ca87426a659caa |
| `laptop/camera/source.py` | eb86a2c66e6dfec6fb83f3805c5ac39b1119345613219755e35293f052275f09 |
| `laptop/client/bounded_model_trial.py` | f2f571eb2be8457a7f89d17cad3757a96cb59b5c6f0d99ff20c1230045caa30d |
| `laptop/client/live_shadow.py` | f3e8ca72d1c42285af30e182a6f3492d700b3c51289cfd6b6b88bbedd0eecb3c |
| `laptop/client/observation_schema.py` | 3e13cc833c9bdbfe8d9a690b8e972dae469c17fb590a9ade2bf388b63419af5f |
| `laptop/control/contracts.py` | 6b05a3f3700e6d5f23c37d555415e56eaa1473962bbc4ff55ef89b524300c7a2 |
| `laptop/diagnostics/bounded_process.py` | c5dade9f850fb8afdf75c23c4bb6fcd53093e90e0bc0b21e3dd076a11816f2c2 |
| `laptop/diagnostics/camera_identity.py` | a360c8f67098436492dcbfbff108ad222ba467d962256f62d886e6088a309311 |
| `laptop/piper/unit_conversion.py` | f1a92193620444b2aeffd3147761e8193342040991dda8668bde0319a7651166 |
| `laptop/recorder/align_raw.py` | d6dc1ad6ce02252b9e9d16fe32616b7856208605052936084ca8a6ea204365e7 |
| `laptop/recorder/capture_session.py` | 82492330bafad1d496d88a71b239f236f17ad6a009f2d26adc5ce1e2c60c3962 |
| `laptop/recorder/episode.py` | d43bd64148070c3bb0cef24a2bd7ccb00922ba742db5e8bd80a8ef764b79b2ab |
| `laptop/recorder/label_episode.py` | 58b91eee279111fefd7af71c8b49431971c7f4b904a9cf24522e693f7c5b0385 |
| `laptop/recorder/next_state_proxy.py` | ee87de7528eb4f641c20163b74b9662648d1ac23571cacfcda492b32650929fd |
| `laptop/recorder/passive_capture.py` | dc8ab41a7e8591ce9a408c60f2f8226e19b6694ab584392a6021244ac897f9ca |
| `laptop/recorder/record_demonstrations.py` | 68da4edd5b668e7c5cc10a116c37f5b2b7fe741f6e8d5d170fda011ae06dc851 |
| `laptop/recorder/verify_source.py` | 14892c311b4d6628656c4092ff26b9578e955f58ffbc7764b9084cb923422c93 |
| `laptop/robot_client/bounded_policy_target.py` | 489179f9b01080ca5fb428ee2c8772256d669ab318b681aae962d2a24c61c010 |
| `laptop/robot_client/hil_guard.py` | 771ffbac5a00def3fe65db5c580195b6e584f1fe61abe00c70ce5367f529b923 |
| `laptop/robot_client/piper_session.py` | bf2732d79a271e29e5fa79ae8043c6f7739d211b5e9c7d41d8d5c6ff212cc63e |
| `laptop/robot_client/replay_preflight.py` | 9c0a4bd9961cceb496590181a9bba8444bd8e19c1d4e697db0a6ffd187294b86 |
| `laptop/robot_client/supervised_policy_step.py` | daf3fa1e4289c71fe12f9bc59554c011f3ec3bde6363c3d8af0fa2118452b9b2 |
| `laptop/robot_client/verified_policy_command.py` | 0fd8bbcb8ee3fcf4953b7434e2948c458ac8f385e7bfe4a78af8fc5b44afb7c9 |
| `laptop/runtime/bridge_protocol.py` | 0004885aada3142101c3215ae02c2efabc9f43cd28a8cda60d4e344255634323 |
| `laptop/runtime/config.py` | 308ca606dcc44c24f2e3474fc61686e90e92b6da2c2cbd2b79ced3af66deb6ca |
| `laptop/runtime/modes.py` | f723ccb28274d57d24c92b6c538223aece544ecf7026f58b7bf48822be9a7ebb |
| `laptop/safety/bridge.py` | 821f5242f5f3948768b742d0d674fa8d793ff92db1a26d37ff385e6ffab6f2ab |
| `laptop/transport/bounded_trial.py` | 216a02431be46f68b9fa292e6adee3c55f5be2e9831a5d088428690aad36c235 |
| `laptop/transport/can_transport.py` | 1ebb004b714b93395e54c3b241445919cdde04f5e879fe78931cef2ee7ce05dd |
| `laptop/transport/diagnostic_queries.py` | cd22f5a1b0953a6ffbd890a391c480bcdb713989574d8c2a7090a0f2211cef74 |
| `laptop/transport/gripper_commissioning.py` | fadc32f1728e3d51d74e27fc9f7ab468b46b1ead6ba53b080a6a727f9a990c22 |
| `laptop/transport/gs_usb_piper_can.py` | bcfbd27d0c8bedde55d348ca8435534e74f50e88890afef7446e299effa32f2b |
| `laptop/transport/mode_initialization.py` | fb786be0b5a5518ce56600c19630b77c3d461848c12517fd831f842781ababe2 |
| `laptop/transport/position_commands.py` | 3cf920558e43367df729a91e84d9e0264219e876c425188421eb373a8380f9a1 |
| `server/dataset_tools/convert_next_state_proxy.py` | d67d6f5a2177bac26c2a780e5756186e7900ab1ee1f797938f6e4bd4cef60678 |
| `server/policy_server/artifact_identity.py` | 992cd9ebc3cb6e756bc9e1badf043fa1d1af5e4e453731ab2daa2cae1f866b60 |
| `server/serve_derived_state_multi_shadow.py` | 90c99469abe838862d2713487fe7556933cfd4f0bbd79ec19170dbadbeef14e6 |
| `server/training/evaluate_demonstrations.py` | 3f7df90af5027b2328fa7e4aab8fb5a68c311776fb8b30ca907f9109f95f301d |
| `server/training/piper_openpi_config.py` | 2b6db0d08796678a02037008c80ca07b1aaf057f343a7417f9caf507a7c65480 |
| `server/training/post_capture_pipeline.py` | d531d8d7c67f710ad0545ad9def0af544e9139b7c1b8a8d20f0e954975351765 |
| `server/training/proxy_multi_episode_contract.py` | 007a9d0aab2e450701ace954ff1e77da8cec6d6223f59e9d2d882646370fbe9b |
| `server/training/proxy_multi_episode_pilot.py` | f965b85c0c1c6abf7e63b02a0a191465051f751cdabe24cda15f4aa9da707fba |
| `server/training/serve_trained_demonstrations.py` | 3c58c4d991e1197c3ddbb4a08abc1f687e2f4dfe519f476367643f2ee4cee055 |
| `server/training/train_demonstrations.py` | f4329152ea7303341c81cd742e52dd6b91b81bc004e2db62bb30f63c69f27b10 |

Python语法异常：未发现；JSON解析异常：未发现。这些是静态格式检查，不是模型/硬件测试。
