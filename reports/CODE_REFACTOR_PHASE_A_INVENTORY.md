# Phase A extraction inventory

所有列出的模块已静态读取/AST检查，未导入执行其硬件入口。

- `laptop/piper/unit_conversion.py`：_scale_or_error, sdk_joint_to_rad, rad_to_sdk_joint, sdk_gripper_to_m, m_to_sdk_gripper
- `laptop/client/live_shadow.py`：validate_artifact_identity_lock, load_artifact_identity_lock, validate_policy_metadata, prewarm_policy, build_policy_prediction, segment_latency, initial_sensor_readiness, new_can_diagnostics, record_can_frame_diagnostic, select_observation, state_snapshot_from_live_observation, _camera_loop, _can_loop, _worker, run, main
- `laptop/client/observation_schema.py`：make_piper_proxy_warmup_observation, make_official_aloha_dummy_observation, validate_piper_semantic_observation
- `laptop/client/bounded_model_trial.py`：stable_snapshot, worker, main
- `laptop/robot_client/piper_session.py`：PiperSession
- `laptop/robot_client/replay_preflight.py`：_frame, analyze, main
- `laptop/safety/bridge.py`：BridgeState, SafetyLimits, BridgeDecision, DiagnosticQueryDecision, HILGateDecision, SafetyBridge
- `laptop/transport/gs_usb_piper_can.py`：CanAdapterNotUniqueError, CanBitrateError, CanListenOnlyUnsupportedError, CanReceiveMode, CanQueryDeliveryUnknown, CanInitializationDeliveryUnknown, CanPositionDeliveryUnknown, CanJ1CommissioningDeliveryUnknown, _ReceiveAdapter, ensure_libusb_backend, enumerate_candlelight_usb_adapters, _GsUsbReceiveAdapter, GsUsbPiperCanTransport
- `laptop/transport/position_commands.py`：CanTxFrame, PositionFrameWrite, J1CommissioningSnapshot, J1CommissioningBatch, J1CommissioningAuthorization, J1CommissioningWriteReceipt, validate_j1_commissioning_snapshot, validate_j1_commissioning_execution, PositionBatchWriteReceipt, PositionCommandBatch, SupervisedPositionAuthorization, validate_position_execution_context
- `laptop/recorder/passive_capture.py`：assemble, capture, _capture_worker, capture_supervised, main
- `laptop/recorder/align_raw.py`：_sha256, _inside, _lines, _raw_episode, _nearest, _positive_number, plan_alignment, _review, _extract_video, align, main
- `laptop/recorder/next_state_proxy.py`：_sha256, _state, audit_transition_deltas, build, main
- `server/training/piper_openpi_config.py`：_image, _vector, PiperOpenPIInputs, PiperOpenPIOutputs, PiperOpenPIDataConfig, make_unregistered_config
- `laptop/control/contracts.py`：_vector7, _finite_real, _canonical_int, TimingContract, RobotStateSnapshot, PolicyPrediction, CommandProposal, command_from_proxy

## Old implementation / callers / current semantics / target authority

| OLD | CALLERS | CURRENT SEMANTICS | TARGET AUTHORITY |
|---|---|---|---|
| inline raw*pi/180000, round(rad*180000/pi) | live_shadow / bounded_model_trial / align_raw / verify_source / feedback probe | Python scalar arithmetic, round→int; no added validation | unit_conversion scalar helpers (existing NumPy APIs unchanged) |
| raw*1e-6 vs raw/1e6 | live_shadow vs bounded runtime | floating arithmetic orders differ; retain separate exact scalar variants | same unit_conversion module |
| status/joint/gripper payload reads | PiperSession | signed joints/effort, unsigned status; aggregation/freshness remains session | robot_client.feedback pure frame decoder |
| SDK message field projection | live_shadow CAN loop / feedback probe | SDK parser retains incremental state and enum string representations | feedback SDK snapshot projection, retain vendor DecodeMessage |
| RGB,state,prompt dict | shadow / bounded / warmup | RGB conversion remains caller; float32 state; shadow validates, bounded does not add validation | observation_schema builder |
| _camera_loop | live_shadow / bounded / supervised_live_step | exact loop, perf_counter, callback, locking/errors/release | camera.worker.camera_loop + old alias |
| identity validation / prewarm | live_shadow reused by bounded and preview | same metadata gates, no timeout/identity changes | client.policy_client + compatibility exports |

## Preserved differences / stop boundaries

NumPy SDK conversion APIs use different rounding/type/overflow/NaN handling: do not replace them with scalar variants.
Multiplication and division gripper paths cannot be collapsed into one arithmetic expression.
SafetyBridge decoders are intentionally untouched because they participate in authorization checks.
Passive_capture assembles already-decoded rows rather than decoding raw CAN.
Server OpenPI transforms and evaluation builder remain untouched: training/adapter boundary, dtype may differ.
next_state_proxy scale constant retained with original behavior; data generation not run.
Contracts types already exist; no replacement Prediction/Proposal/Snapshot types are introduced.
Latency smoke host handling and timeout process are different; leave constructor unchanged.

## Existing user edits

Git-wide baseline preserved in CODE_REFACTOR_PHASE_A_BASELINE.md. Current contents—not HEAD—are the old implementation oracle.
Updates will use exact snippets/AST boundaries, abort if content differs; unrelated edits and legacy scripts untouched.
