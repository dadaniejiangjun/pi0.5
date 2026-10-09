# Remaining conversions after Phase A

本表包括时间单位和测试值；保留字符串不等于机器人转换重复。
单位authority内的表达式属INTENTIONAL，协议和安全raw计算不迁移。

| Path:line | Classification | Expression |
|---|---|---|
| `laptop/camera/dual_camera.py:107` | INTENTIONAL | `period = 1_000_000_000/image_fps` |
| `laptop/camera/dual_camera.py:119` | INTENTIONAL | `if stamp-last_progress[0] >= 1_000_000_000:` |
| `laptop/client/action_shadow_replay.py:182` | INTENTIONAL | `"prediction_round_trip_ms": (finished_ns - started_ns) / 1e6,` |
| `laptop/client/bounded_model_trial.py:99` | INTENTIONAL | `camera_wait_ms=(time.perf_counter_ns()-camera_wait_started)/1e6` |
| `laptop/client/bounded_model_trial.py:128` | INTENTIONAL | `record=dict(step=step,request_id=batch.prediction_request_id,action_index=index,inference_ms=(received-started)/1e6,camera_wait_ms=camera_wait_ms,bounded=bounded,target_joint_raw=list(targets),target_gripper_raw=gripper)` |
| `laptop/client/live_shadow.py:273` | INTENTIONAL | `"camera_skew_ms": abs(times[0] - times[1]) / 1e6,` |
| `laptop/client/live_shadow.py:274` | INTENTIONAL | `"oldest_sensor_age_ms": (now_ns - min(times + can_times)) / 1e6,` |
| `laptop/client/live_shadow.py:452` | INTENTIONAL | `startup["wait_ms"] = (time.perf_counter_ns() - startup_started_ns) / 1e6` |
| `laptop/client/live_shadow.py:461` | INTENTIONAL | `startup["wait_ms"] = (time.perf_counter_ns() - startup_started_ns) / 1e6` |
| `laptop/client/live_shadow.py:508` | INTENTIONAL | `latency = segment_latency(result, (response_ns - request_ns) / 1e6)` |
| `laptop/client/live_shadow.py:526` | INTENTIONAL | `"inference_latency_ms": (response_ns - request_ns) / 1e6,` |
| `laptop/client/network_latency_probe.py:160` | INTENTIONAL | `roundtrip_ms = (finished_ns - started_ns) / 1e6` |
| `laptop/client/network_latency_probe.py:206` | INTENTIONAL | `reconnect_ms = (time.perf_counter_ns() - reconnect_started_ns) / 1e6` |
| `laptop/client/offline_latency_replay.py:63` | INTENTIONAL | `roundtrip_ms = (time.perf_counter_ns() - start_ns) / 1e6` |
| `laptop/client/policy_client.py:227` | INTENTIONAL | `"latency_ms": (finished_ns - started_ns) / 1e6,` |
| `laptop/client/shadow_preview_smoke.py:86` | HISTORICAL | `row["joint_rad_nominal_only"] + [row["gripper_raw"] * 1e-6],` |
| `laptop/client/shadow_preview_smoke.py:104` | INTENTIONAL | `latency_ms = (response_ns - observation_ns) / 1e6` |
| `laptop/control/contracts.py:182` | INTENTIONAL | `offset = value * 1_000_000_000` |
| `laptop/control/contracts.py:213` | INTENTIONAL | `self.observation_monotonic_ns + round(offset_s * 1_000_000_000)` |
| `laptop/diagnostics/audit_feedback_log.py:35` | INTENTIONAL | `intervals = [(b - a) / 1e6 for a, b in zip(times, times[1:])]` |
| `laptop/diagnostics/probe_can_ack_only.py:53` | INTENTIONAL | `end_ns = start_ns + int(duration_s * 1_000_000_000)` |
| `laptop/diagnostics/probe_can_ack_only.py:69` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),` |
| `laptop/diagnostics/probe_can_ack_only.py:71` | INTENTIONAL | `"bitrate": 1_000_000,` |
| `laptop/diagnostics/probe_can_listen_only.py:37` | INTENTIONAL | `if bitrate != 1_000_000:` |
| `laptop/diagnostics/probe_can_listen_only.py:72` | INTENTIONAL | `end_ns = start_ns + int(duration_s * 1_000_000_000)` |
| `laptop/diagnostics/probe_can_listen_only.py:89` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),` |
| `laptop/diagnostics/probe_can_listen_only.py:110` | INTENTIONAL | `parser.add_argument("--bitrate", type=int, default=1_000_000)` |
| `laptop/diagnostics/probe_dual_cameras.py:35` | INTENTIONAL | `intervals_ms = [(right - left) / 1_000_000 for left, right in zip(timestamps, timestamps[1:])]` |
| `laptop/diagnostics/probe_dual_cameras.py:56` | INTENTIONAL | `skews.append(min(abs(timestamp - other) for other in neighbors) / 1_000_000)` |
| `laptop/diagnostics/probe_dual_cameras.py:185` | INTENTIONAL | `end_ns = start_ns + int(duration_s * 1_000_000_000)` |
| `laptop/diagnostics/probe_dual_cameras.py:198` | INTENTIONAL | `or timestamp - last_quality_sample_ns >= 1_000_000_000):` |
| `laptop/diagnostics/probe_dual_cameras.py:227` | INTENTIONAL | `elapsed_s = (time.perf_counter_ns() - start_ns) / 1_000_000_000` |
| `laptop/diagnostics/probe_motor_enable_ack_only.py:72` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),` |
| `laptop/diagnostics/probe_piper_feedback_ack_only.py:118` | INTENTIONAL | `end_ns = start_ns + int(duration_s * 1_000_000_000)` |
| `laptop/diagnostics/probe_piper_feedback_ack_only.py:149` | INTENTIONAL | `timestamp=arrival_ns / 1_000_000_000,` |
| `laptop/diagnostics/probe_piper_feedback_ack_only.py:179` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),` |
| `laptop/diagnostics/query_all_joint_parameters.py:79` | INTENTIONAL | `timestamp=frame.received_monotonic_ns / 1_000_000_000,` |
| `laptop/diagnostics/query_all_joint_parameters.py:174` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),` |
| `laptop/diagnostics/query_all_joint_parameters.py:176` | INTENTIONAL | `"bitrate": 1_000_000,` |
| `laptop/diagnostics/query_piper_firmware.py:80` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3),` |
| `laptop/diagnostics/query_piper_firmware.py:82` | INTENTIONAL | `"bitrate": 1_000_000,` |
| `laptop/diagnostics/query_piper_gripper_parameters.py:77` | INTENTIONAL | `ages_ms[f"0x{can_id:03X}"] = round(age_ns / 1_000_000, 3)` |
| `laptop/diagnostics/query_piper_gripper_parameters.py:86` | INTENTIONAL | `"freshness_limit_ms": max_age_ns / 1_000_000,` |
| `laptop/diagnostics/query_piper_gripper_parameters.py:143` | INTENTIONAL | `timestamp=arrival_ns / 1_000_000_000,` |
| `laptop/diagnostics/query_piper_gripper_parameters.py:189` | INTENTIONAL | `elapsed_s = round((time.perf_counter_ns() - start_ns) / 1_000_000_000, 3)` |
| `laptop/diagnostics/query_piper_gripper_parameters.py:201` | INTENTIONAL | `"bitrate": 1_000_000,` |
| `laptop/diagnostics/query_piper_readiness_snapshot.py:151` | INTENTIONAL | `"duration_s": round((time.perf_counter_ns() - started_ns) / 1_000_000_000, 3),` |
| `laptop/piper/unit_conversion.py:65` | INTENTIONAL | `"""Python scalar raw * pi / 180000; no added coercion or validation."""` |
| `laptop/piper/unit_conversion.py:66` | INTENTIONAL | `return raw * math.pi / 180000` |
| `laptop/piper/unit_conversion.py:70` | INTENTIONAL | `"""Python round(rad * 180000 / pi), retaining exceptions and ties-to-even."""` |
| `laptop/piper/unit_conversion.py:71` | INTENTIONAL | `return round(rad * 180000 / math.pi)` |
| `laptop/piper/unit_conversion.py:75` | INTENTIONAL | `"""Multiply by 1e-6 as in the shadow/data path, not jaw calibration."""` |
| `laptop/piper/unit_conversion.py:76` | INTENTIONAL | `return raw * 1e-6` |
| `laptop/piper/unit_conversion.py:80` | INTENTIONAL | `"""Divide by 1e6 as in bounded runtime; preserve its arithmetic order."""` |
| `laptop/piper/unit_conversion.py:81` | INTENTIONAL | `return raw / 1e6` |
| `laptop/piper/unit_conversion.py:85` | INTENTIONAL | `"""Python round(meters * 1e6) as in bounded runtime."""` |
| `laptop/piper/unit_conversion.py:86` | INTENTIONAL | `return round(meters * 1e6)` |
| `laptop/recorder/align_raw.py:133` | INTENTIONAL | `tolerance_ns = round(_positive_number(max_skew_ms, "max_skew_ms") * 1_000_000)` |
| `laptop/recorder/align_raw.py:149` | INTENTIONAL | `period = round(1_000_000_000 / fps)` |
| `laptop/recorder/episode.py:183` | INTENTIONAL | `skew_ms.append(min(abs(ts - other) for other in nearby) / 1e6)` |
| `laptop/recorder/episode.py:202` | INTENTIONAL | `nearest_ms.append(min(abs(ts - other) for other in nearby) / 1e6)` |
| `laptop/recorder/next_state_proxy.py:21` | INTENTIONAL | `GRIPPER_METERS_PER_RAW_NOMINAL = 1e-6` |
| `laptop/recorder/next_state_proxy.py:122` | INTENTIONAL | `period_ns = round(1_000_000_000 / meta["fps"])` |
| `laptop/recorder/piper_command_targets.py:21` | INTENTIONAL | `GRIPPER_METERS_PER_RAW = 1e-6` |
| `laptop/recorder/synthetic_smoke.py:25` | INTENTIONAL | `start_ns = 1_000_000_000` |
| `laptop/robot_client/piper_session.py:99` | INTENTIONAL | `not 0 <= now - self._latest[i][0] <= max_age_ms * 1_000_000]` |
| `laptop/robot_client/single_step_dry_run.py:278` | INTENTIONAL | `created_monotonic_ns=now_ns - 1_000_000,` |
| `laptop/robot_client/supervised_j1_one_degree_trial.py:54` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/robot_client/supervised_j1_step_trial.py:104` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/robot_client/supervised_j5_probe.py:34` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/robot_client/supervised_joint_replay.py:89` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/robot_client/supervised_restore_gripper.py:59` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/robot_client/supervised_restore_start.py:109` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/robot_client/supervised_smooth_joint_replay.py:186` | INTENTIONAL | `if not device.set_bitrate(1_000_000):` |
| `laptop/safety/bridge.py:433` | PROTOCOL_REQUIRED | `cutoff = frame.received_monotonic_ns - 1_000_000_000` |
| `laptop/safety/offline_guard.py:166` | PROTOCOL_REQUIRED | `seconds = horizon / 1_000_000_000` |
| `laptop/transport/gs_usb_piper_can.py:189` | PROTOCOL_REQUIRED | `if not device.set_bitrate(1_000_000):` |
| `server/dataset_tools/convert_piper_aligned.py:166` | INTENTIONAL | `if abs((row["action_target_timestamp_ns"] - policy_ts) - offset) > 1_000_000:` |
| `server/training/proxy_latency_profile.py:66` | INTENTIONAL | `roundtrip_ms = (time.perf_counter_ns() - start_ns) / 1e6` |
| `scripts/validate_piper_contract.py:27` | TEST_EXPECTATION | `raw_joint = rng.integers(-180_000, 180_001, size=10_000, dtype=np.int64)` |
| `tests/contracts/test_control_contracts.py:20` | TEST_EXPECTATION | `"observation_monotonic_ns": 1_000_000_000,` |
| `tests/contracts/test_control_contracts.py:36` | TEST_EXPECTATION | `"captured_monotonic_ns": 1_000_000_000,` |
| `tests/contracts/test_control_contracts.py:98` | TEST_EXPECTATION | `{"captured_monotonic_ns": 1_000_000_000.0},` |
| `tests/contracts/test_control_contracts.py:115` | TEST_EXPECTATION | `{"observation_monotonic_ns": 1_000_000_000.0},` |
| `tests/contracts/test_piper_units.py:20` | TEST_EXPECTATION | `raw = rng.integers(-180_000, 180_001, size=10_000, dtype=np.int64)` |
| `tests/data/test_align_raw.py:32` | TEST_EXPECTATION | `ts = 1_000_000_000 + index * 100_000_000` |
| `tests/data/test_align_raw.py:34` | TEST_EXPECTATION | `writer.add("wrist", ts + 1_000_000, {"frame_id": index})` |
| `tests/data/test_align_raw.py:36` | TEST_EXPECTATION | `writer.add(channel, ts + offset * 1_000_000,` |
| `tests/data/test_align_raw.py:48` | TEST_EXPECTATION | `base_ts = 1_000_000_000 + tick * 100_000_000` |
| `tests/data/test_align_raw.py:120` | TEST_EXPECTATION | `"joint_degrees_per_raw": 0.001, "gripper_meters_per_raw": 1e-6,` |
| `tests/data/test_align_raw.py:155` | TEST_EXPECTATION | `"gripper_meters_per_raw": 1e-6,` |
| `tests/data/test_align_raw_to_lerobot.py:40` | TEST_EXPECTATION | `"gripper_meters_per_raw": 1e-6,` |
| `tests/data/test_piper_command_targets.py:29` | TEST_EXPECTATION | `def complete_frames(base_ns: int = 1_000_000_000, *, offset: int = 0):` |
| `tests/data/test_source_verification.py:66` | TEST_EXPECTATION | `"gripper_meters_per_raw": 1e-6,` |
| `tests/laptop/test_action_shadow_replay.py:29` | TEST_EXPECTATION | `"gripper_meters_per_raw": 1e-6,` |
| `tests/laptop/test_bounded_model_trial.py:98` | TEST_EXPECTATION | `assert stable_snapshot(robot,{},time.perf_counter_ns()+1_000_000_000)=='stable'` |
| `tests/laptop/test_bounded_model_trial.py:103` | TEST_EXPECTATION | `with pytest.raises(PermissionError,match='motor disabled'):stable_snapshot(fault,{},time.perf_counter_ns()+1_000_000_000)` |
| `tests/laptop/test_can_listen_only_guard.py:13` | TEST_EXPECTATION | `result = probe(10, 1_000_000)` |
| `tests/laptop/test_can_listen_only_guard.py:28` | TEST_EXPECTATION | `1_000_000,` |
| `tests/laptop/test_compare_gripper_feedback.py:18` | TEST_EXPECTATION | `"monotonic_ns": index * 1_000_000_000,` |
| `tests/laptop/test_diagnostic_queries.py:191` | TEST_EXPECTATION | `received_monotonic_ns=NOW - 1_000_000,` |
| `tests/laptop/test_dual_camera_capture.py:17` | TEST_EXPECTATION | `on_frame(role, image, 1_000_000_000+index*50_000_000, index)` |
| `tests/laptop/test_dual_camera_capture.py:38` | TEST_EXPECTATION | `assert [r["received_monotonic_ns"] for r in rows] == [1_000_000_000, 1_050_000_000, 1_100_000_000]` |
| `tests/laptop/test_fake_can_single_step_rehearsal.py:60` | TEST_EXPECTATION | `bridge.record_heartbeat(now_ns - 1_000_000)` |
| `tests/laptop/test_fake_can_single_step_rehearsal.py:77` | TEST_EXPECTATION | `("state_stale", lambda now: make_inputs(now - 1_000_000_000)),` |
| `tests/laptop/test_fake_can_single_step_rehearsal.py:88` | TEST_EXPECTATION | `bridge.record_heartbeat(now_ns - 1_000_000)` |
| `tests/laptop/test_fake_can_single_step_rehearsal.py:114` | TEST_EXPECTATION | `bridge.record_heartbeat(now_ns - 1_000_000)` |
| `tests/laptop/test_fake_can_single_step_rehearsal.py:132` | TEST_EXPECTATION | `bridge.record_heartbeat(now_ns - 1_000_000)` |
| `tests/laptop/test_feedback_log_audit.py:25` | TEST_EXPECTATION | `for ts in range(1_000_000_000, 3_100_000_000, 50_000_000):` |
| `tests/laptop/test_feedback_log_audit.py:40` | TEST_EXPECTATION | `for ts in [1_000_000_000, *range(1_000_000_000, 3_100_000_000, 50_000_000)]:` |
| `tests/laptop/test_gripper_parameter_query.py:63` | TEST_EXPECTATION | `timestamps[0x2A8] = now_ns - 251_000_000` |
| `tests/laptop/test_j1_commissioning_bridge.py:332` | TEST_EXPECTATION | `_post_feedback(target, now - 1_000_000),` |
| `tests/laptop/test_j1_commissioning_bridge.py:333` | TEST_EXPECTATION | `[(now - 1_000_000, target)], source, target, now,` |
| `tests/laptop/test_j1_commissioning_bridge.py:346` | TEST_EXPECTATION | `_post_feedback(wrong, now - 1_000_000),` |
| `tests/laptop/test_j1_commissioning_bridge.py:347` | TEST_EXPECTATION | `[(now - 1_000_000, wrong)], source, target, now,` |
| `tests/laptop/test_live_shadow.py:393` | TEST_EXPECTATION | `self.now = 1_000_000_000` |
| `tests/laptop/test_offline_safety_bridge.py:17` | TEST_EXPECTATION | `NOW = 1_000_000_000` |
| `tests/laptop/test_passive_capture.py:76` | TEST_EXPECTATION | `"frame_timestamps_ns": {"base": [1_000_000_000 + i * 100_000_000 for i in range(10)],` |
| `tests/laptop/test_passive_capture.py:90` | TEST_EXPECTATION | `"can_id": can_id, "monotonic_ns": 1_000_000_000 + i * 100_000_000 + offset,` |
| `tests/laptop/test_passive_capture.py:145` | TEST_EXPECTATION | `"frame_timestamps_ns": {"base": [1_000_000_000], "wrist": [1_001_000_000]}}` |
| `tests/laptop/test_passive_capture.py:159` | TEST_EXPECTATION | `"can_id": can_id, "monotonic_ns": 1_000_000_000 + offset,` |
| `tests/laptop/test_passive_capture.py:226` | TEST_EXPECTATION | `stamps[role].append(1_010_000_000 + index * 100_000_000 + (0 if role == "base" else 1_000_000))` |
| `tests/laptop/test_passive_capture.py:243` | TEST_EXPECTATION | `base = 1_000_000_000 + index * 100_000_000` |
| `tests/laptop/test_phase_a_pure_core.py:34` | TEST_EXPECTATION | `@pytest.mark.parametrize('raw', [0,1,-1,929,-12345,150000,-150000,180000,-170000,` |
| `tests/laptop/test_phase_a_pure_core.py:38` | TEST_EXPECTATION | `old = raw * math.pi / 180000` |
| `tests/laptop/test_phase_a_pure_core.py:40` | TEST_EXPECTATION | `assert units.nominal_rad_to_joint_raw(old) == round(old*180000/math.pi)` |
| `tests/laptop/test_phase_a_pure_core.py:46` | TEST_EXPECTATION | `assert units.gripper_raw_to_nominal_m(raw) == raw * 1e-6` |
| `tests/laptop/test_phase_a_pure_core.py:47` | TEST_EXPECTATION | `assert units.gripper_raw_to_nominal_m_divided(raw) == raw / 1e6` |
| `tests/laptop/test_phase_a_pure_core.py:48` | TEST_EXPECTATION | `for v in (raw*1e-6, raw/1e6):` |
| `tests/laptop/test_phase_a_pure_core.py:49` | TEST_EXPECTATION | `assert units.nominal_m_to_gripper_raw(v) == round(v*1e6)` |
| `tests/laptop/test_phase_a_pure_core.py:55` | TEST_EXPECTATION | `for new, old in [(units.nominal_rad_to_joint_raw,lambda v:round(v*180000/math.pi)),` |
| `tests/laptop/test_phase_a_pure_core.py:56` | TEST_EXPECTATION | `(units.nominal_m_to_gripper_raw,lambda v:round(v*1e6))]:` |
| `tests/laptop/test_phase_a_pure_core.py:111` | TEST_EXPECTATION | `now=1_000_000_000` |
| `tests/laptop/test_phase_a_pure_core.py:138` | TEST_EXPECTATION | `now=1_000_000_000;base=np.zeros((5,6,3),dtype=np.uint8);base[:,:,0]=7;base[:,:,2]=91` |
| `tests/laptop/test_phase_a_pure_core.py:180` | TEST_EXPECTATION | `monkeypatch.setattr(policy_client.time,'perf_counter_ns',lambda:1_000_000_000)` |
| `tests/laptop/test_phase_a_pure_core.py:228` | TEST_EXPECTATION | `formulas={'joint_raw_to_nominal_rad':'VALUE*math.pi/180000','nominal_rad_to_joint_raw':'round(VALUE*180000/math.pi)',` |
| `tests/laptop/test_phase_a_pure_core.py:229` | TEST_EXPECTATION | `'gripper_raw_to_nominal_m_divided':'VALUE/1e6','nominal_m_to_gripper_raw':'round(VALUE*1e6)'}` |
| `tests/laptop/test_piper_can_transport.py:190` | TEST_EXPECTATION | `return bitrate == 1_000_000` |
| `tests/laptop/test_piper_can_transport.py:251` | TEST_EXPECTATION | `return bitrate == 1_000_000` |
| `tests/laptop/test_piper_can_transport.py:311` | TEST_EXPECTATION | `return bitrate == 1_000_000` |
| `tests/laptop/test_piper_can_transport.py:399` | TEST_EXPECTATION | `return bitrate == 1_000_000` |
| `tests/laptop/test_policy_reject_only.py:13` | TEST_EXPECTATION | `observation_ns = 1_000_000_000` |
| `tests/laptop/test_policy_sdk_preview.py:19` | TEST_EXPECTATION | `observation_monotonic_ns=1_000_000_000,` |
| `tests/laptop/test_policy_sdk_preview.py:33` | TEST_EXPECTATION | `captured_monotonic_ns=received_ns - 1_000_000,` |
| `tests/laptop/test_safety_bridge_runtime.py:93` | TEST_EXPECTATION | `bridge.record_heartbeat(NOW - 1_000_000)` |
| `tests/laptop/test_safety_bridge_runtime.py:168` | TEST_EXPECTATION | `values["max_state_age_ns"] = 1_000_000_000` |
| `tests/laptop/test_safety_bridge_runtime.py:169` | TEST_EXPECTATION | `values["max_prediction_response_latency_ns"] = 1_000_000_000` |
| `tests/laptop/test_safety_bridge_runtime.py:170` | TEST_EXPECTATION | `values["max_proposal_age_ns"] = 1_000_000_000` |
| `tests/laptop/test_safety_bridge_runtime.py:342` | TEST_EXPECTATION | `valid_until_monotonic_ns=NOW + 1_000_000_000,` |
| `tests/laptop/test_supervised_enable_j6.py:53` | TEST_EXPECTATION | `stamp = clock[0] - 361_000_000 + index * 6_000_000` |
| `tests/laptop/test_supervised_j1_one_degree_trial.py:12` | TEST_EXPECTATION | `now = 1_000_000_000` |
| `tests/laptop/test_supervised_j1_step_trial.py:11` | TEST_EXPECTATION | `clock = 1_000_000_000` |
| `tests/laptop/test_supervised_mode_initialization.py:64` | TEST_EXPECTATION | `stamp = clock[0] - 361_000_000 + index * 6_000_000` |
| `tests/laptop/test_supervised_policy_session.py:84` | TEST_EXPECTATION | `observation_monotonic_ns=started_ns - 1_000_000,` |
| `tests/laptop/test_supervised_policy_step.py:54` | TEST_EXPECTATION | `received_monotonic_ns=observation_ns + 1_000_000,` |
| `tests/laptop/test_supervised_policy_step.py:66` | TEST_EXPECTATION | `received_monotonic_ns=now - 1_000_000,` |
| `tests/laptop/test_supervised_position_bridge.py:71` | TEST_EXPECTATION | `received_monotonic_ns=base - 1_000_000,` |
| `tests/laptop/test_supervised_position_transport.py:71` | TEST_EXPECTATION | `created_monotonic_ns=now - 1_000_000,` |
| `tests/laptop/test_supervised_position_transport.py:89` | TEST_EXPECTATION | `CanTxFrame(0x159, struct.pack(">iHBB", round(proposal.target[6] * 1_000_000), 120, 1, 0)),` |
| `tests/laptop/test_supervised_position_transport.py:364` | TEST_EXPECTATION | `return bitrate == 1_000_000` |
| `tests/laptop/test_supervised_restore_start.py:14` | TEST_EXPECTATION | `now = 1_000_000_000` |
| `tests/laptop/test_verified_policy_command.py:26` | TEST_EXPECTATION | `observation_monotonic_ns=1_000_000_000,` |
| `tests/laptop/test_verified_policy_command.py:44` | TEST_EXPECTATION | `captured_monotonic_ns=received_ns - 1_000_000,` |
| `tests/server/test_piper_aligned_converter.py:51` | TEST_EXPECTATION | `ts = 1_000_000_000 + index * 100_000_000` |
| `tests/server/test_piper_aligned_converter.py:56` | TEST_EXPECTATION | `"wrist_timestamp_ns": ts + 1_000_000,` |
| `tests/server/test_proxy_multi_episode_contract.py:73` | TEST_EXPECTATION | `timestamp = episode_index * 1_000_000_000 + local_index * 100_000_000` |
