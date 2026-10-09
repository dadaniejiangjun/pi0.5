---
Document status: REFERENCE
Current authority: docs/RUNBOOK.md
Last reviewed: 2026-10-09
---

# Active code index

本清单依据本地实际入口及其静态依赖。运行参数/授权见 [RUNBOOK](RUNBOOK.md)；本轮未改变机器人技术状态。历史代码与测试见 [snapshot](../archive/legacy_snapshot_20261009/README.md)。

| 链路 | 当前入口 |
|---|---|
| 相机 | laptop.camera.dual_camera |
| 通信、查询、模式与 commissioning | laptop.robot_client.piper_tool / piper_session |
| 回零与夹爪现场操作 | bounded_zero_return / supervised_near_zero_home / supervised_gripper_max_open / supervised_oem_gripper_test |
| 数采、标签、质量 | laptop.recorder.record_demonstrations / capture_session / label_episode / align_raw / audit_episode |
| 转换与流程 | server.dataset_tools.convert_next_state_proxy / server.training.post_capture_pipeline |
| 训练与评估 | train_demonstrations / evaluate_demonstrations / summarize_training_loss |
| 服务 | serve_trained_demonstrations → server.serve_derived_state_multi_shadow |
| inference / 受监督执行 | laptop.client.live_model_preview / bounded_model_trial |
| 公共 authority | camera.worker / observation_schema / policy_client / unit_conversion / feedback / SafetyBridge / typed transport |

## Retained source files

保留当前入口、传递依赖、通信/相机诊断与现有控制 authority。当前训练实际依赖的 proxy_multi_episode_* 与 piper_openpi_config 必须保留。

| Path | Reason |
|---|---|
| [laptop/__init__.py](../laptop/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/camera/dual_camera.py](../laptop/camera/dual_camera.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/camera/source.py](../laptop/camera/source.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/camera/worker.py](../laptop/camera/worker.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/__init__.py](../laptop/client/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/audit_live_shadow.py](../laptop/client/audit_live_shadow.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/bounded_model_trial.py](../laptop/client/bounded_model_trial.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/live_model_preview.py](../laptop/client/live_model_preview.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/live_shadow.py](../laptop/client/live_shadow.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/observation_schema.py](../laptop/client/observation_schema.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/client/policy_client.py](../laptop/client/policy_client.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/control/__init__.py](../laptop/control/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/control/contracts.py](../laptop/control/contracts.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/audit_feedback_log.py](../laptop/diagnostics/audit_feedback_log.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/bounded_process.py](../laptop/diagnostics/bounded_process.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/camera_identity.py](../laptop/diagnostics/camera_identity.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/camera_stream_smoke.py](../laptop/diagnostics/camera_stream_smoke.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/can_diagnostic_session.py](../laptop/diagnostics/can_diagnostic_session.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/capture_oem_can.py](../laptop/diagnostics/capture_oem_can.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/compare_gripper_feedback.py](../laptop/diagnostics/compare_gripper_feedback.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/list_candlelight_adapters.py](../laptop/diagnostics/list_candlelight_adapters.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_cameras.py](../laptop/diagnostics/probe_cameras.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_can_ack_only.py](../laptop/diagnostics/probe_can_ack_only.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_can_listen_only.py](../laptop/diagnostics/probe_can_listen_only.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_dual_cameras.py](../laptop/diagnostics/probe_dual_cameras.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_motor_enable_ack_only.py](../laptop/diagnostics/probe_motor_enable_ack_only.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_network.py](../laptop/diagnostics/probe_network.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_piper.py](../laptop/diagnostics/probe_piper.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_piper_feedback_ack_only.py](../laptop/diagnostics/probe_piper_feedback_ack_only.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/probe_piper_readonly.py](../laptop/diagnostics/probe_piper_readonly.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/query_all_joint_parameters.py](../laptop/diagnostics/query_all_joint_parameters.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/query_joint_limits.py](../laptop/diagnostics/query_joint_limits.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/query_piper_firmware.py](../laptop/diagnostics/query_piper_firmware.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/query_piper_gripper_parameters.py](../laptop/diagnostics/query_piper_gripper_parameters.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/diagnostics/query_piper_readiness_snapshot.py](../laptop/diagnostics/query_piper_readiness_snapshot.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/piper/__init__.py](../laptop/piper/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/piper/unit_conversion.py](../laptop/piper/unit_conversion.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/__init__.py](../laptop/recorder/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/align_raw.py](../laptop/recorder/align_raw.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/audit_episode.py](../laptop/recorder/audit_episode.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/capture_session.py](../laptop/recorder/capture_session.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/episode.py](../laptop/recorder/episode.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/label_episode.py](../laptop/recorder/label_episode.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/next_state_proxy.py](../laptop/recorder/next_state_proxy.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/passive_capture.py](../laptop/recorder/passive_capture.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/piper_command_targets.py](../laptop/recorder/piper_command_targets.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/record_demonstrations.py](../laptop/recorder/record_demonstrations.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/recorder/verify_source.py](../laptop/recorder/verify_source.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/__init__.py](../laptop/robot_client/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/bounded_policy_target.py](../laptop/robot_client/bounded_policy_target.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/bounded_zero_return.py](../laptop/robot_client/bounded_zero_return.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/feedback.py](../laptop/robot_client/feedback.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/joint_command_preview.py](../laptop/robot_client/joint_command_preview.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/piper_session.py](../laptop/robot_client/piper_session.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/piper_tool.py](../laptop/robot_client/piper_tool.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/replay_preflight.py](../laptop/robot_client/replay_preflight.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/single_step_dry_run.py](../laptop/robot_client/single_step_dry_run.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_axis_commissioning.py](../laptop/robot_client/supervised_axis_commissioning.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_enable_j6.py](../laptop/robot_client/supervised_enable_j6.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_gripper_commissioning.py](../laptop/robot_client/supervised_gripper_commissioning.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_gripper_max_open.py](../laptop/robot_client/supervised_gripper_max_open.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_j1_commissioning.py](../laptop/robot_client/supervised_j1_commissioning.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_mode_initialization.py](../laptop/robot_client/supervised_mode_initialization.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_near_zero_home.py](../laptop/robot_client/supervised_near_zero_home.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_oem_gripper_test.py](../laptop/robot_client/supervised_oem_gripper_test.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/robot_client/supervised_site_profile.py](../laptop/robot_client/supervised_site_profile.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/runtime/__init__.py](../laptop/runtime/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/runtime/bridge_protocol.py](../laptop/runtime/bridge_protocol.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/runtime/config.py](../laptop/runtime/config.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/runtime/modes.py](../laptop/runtime/modes.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/safety/__init__.py](../laptop/safety/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/safety/bridge.py](../laptop/safety/bridge.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/safety/offline_guard.py](../laptop/safety/offline_guard.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/__init__.py](../laptop/transport/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/bounded_trial.py](../laptop/transport/bounded_trial.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/can_transport.py](../laptop/transport/can_transport.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/diagnostic_queries.py](../laptop/transport/diagnostic_queries.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/gripper_commissioning.py](../laptop/transport/gripper_commissioning.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/gs_usb_piper_can.py](../laptop/transport/gs_usb_piper_can.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/mode_initialization.py](../laptop/transport/mode_initialization.py) | 当前入口/依赖/通信诊断或控制 authority |
| [laptop/transport/position_commands.py](../laptop/transport/position_commands.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/__init__.py](../server/__init__.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/dataset_tools/convert_next_state_proxy.py](../server/dataset_tools/convert_next_state_proxy.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/policy_server/artifact_identity.py](../server/policy_server/artifact_identity.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/serve_derived_state_multi_shadow.py](../server/serve_derived_state_multi_shadow.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/evaluate_demonstrations.py](../server/training/evaluate_demonstrations.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/piper_openpi_config.py](../server/training/piper_openpi_config.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/post_capture_pipeline.py](../server/training/post_capture_pipeline.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/proxy_multi_episode_contract.py](../server/training/proxy_multi_episode_contract.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/proxy_multi_episode_pilot.py](../server/training/proxy_multi_episode_pilot.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/serve_trained_demonstrations.py](../server/training/serve_trained_demonstrations.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/summarize_training_loss.py](../server/training/summarize_training_loss.py) | 当前入口/依赖/通信诊断或控制 authority |
| [server/training/train_demonstrations.py](../server/training/train_demonstrations.py) | 当前入口/依赖/通信诊断或控制 authority |

## Tests

tests/ 保留 60 个当前回归模块，33 个历史测试已归档。结果见 [cleanup report](../reports/ACTIVE_CHAIN_CLEANUP_REPORT.md)。默认 pytest 仅发现 tests/；本轮未运行硬件。
