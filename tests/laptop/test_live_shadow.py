from __future__ import annotations

import inspect
import unittest
import json
import tempfile
from pathlib import Path

import numpy as np

from laptop.client import live_shadow
from laptop.client.live_shadow import (
    ARTIFACT_IDENTITY_SCHEMA,
    ARTIFACT_LOCK_SCHEMA,
    build_policy_prediction,
    initial_sensor_readiness,
    load_artifact_identity_lock,
    new_can_diagnostics,
    prewarm_policy,
    record_can_frame_diagnostic,
    segment_latency,
    select_observation,
    state_snapshot_from_live_observation,
    run,
    validate_policy_metadata,
)
from laptop.client.audit_live_shadow import audit


class LiveShadowTest(unittest.TestCase):
    def test_can_shadow_io_uses_the_bridge_owned_transport(self) -> None:
        can_loop = inspect.getsource(live_shadow._can_loop)
        worker = inspect.getsource(live_shadow._worker)

        self.assertIn("safety_bridge.receive_frame", can_loop)
        self.assertNotIn("GsUsb.scan", can_loop)
        self.assertNotIn("device.send", can_loop)
        self.assertIn("GsUsbPiperCanTransport()", worker)
        self.assertIn("transport=transport", worker)
        self.assertIn("initial_sensor_readiness", worker)
        self.assertIn("startup_readiness", worker)
        self.assertIn('"can_diagnostics": can_diagnostics', worker)

    def test_can_receive_diagnostics_separate_missing_frames_from_decode_failures(self) -> None:
        diagnostics = new_can_diagnostics()
        record_can_frame_diagnostic(
            diagnostics,
            arbitration_id=0x2A8,
            is_error_frame=False,
            data_length=8,
            received_monotonic_ns=100,
            channel="gripper",
            decoded=False,
        )
        record_can_frame_diagnostic(
            diagnostics,
            arbitration_id=0x2A8,
            is_error_frame=False,
            data_length=8,
            received_monotonic_ns=200,
            channel="gripper",
            decoded=True,
        )
        record_can_frame_diagnostic(
            diagnostics,
            arbitration_id=0x2A8,
            is_error_frame=False,
            data_length=8,
            received_monotonic_ns=300,
            channel="gripper",
            decoded=None,
        )
        record_can_frame_diagnostic(
            diagnostics,
            arbitration_id=0x123,
            is_error_frame=True,
            data_length=4,
            received_monotonic_ns=400,
            channel=None,
        )

        self.assertEqual(diagnostics["received_frames_total"], 4)
        self.assertEqual(diagnostics["received_frames_by_can_id"]["0x2A8"], 3)
        self.assertEqual(diagnostics["feedback_frames_by_channel"]["gripper"], 3)
        self.assertEqual(
            diagnostics["decoded_feedback_frames_by_channel"]["gripper"], 1
        )
        self.assertEqual(diagnostics["decode_failures_by_channel"]["gripper"], 1)
        self.assertEqual(
            diagnostics["last_decoded_feedback_monotonic_ns_by_channel"]["gripper"],
            200,
        )
        self.assertEqual(diagnostics["error_frames"], 1)
        self.assertEqual(diagnostics["short_frames"], 1)

    def test_can_receive_diagnostics_reject_invalid_counter_input(self) -> None:
        with self.assertRaisesRegex(ValueError, "positive integer"):
            record_can_frame_diagnostic(
                new_can_diagnostics(),
                arbitration_id=0x2A8,
                is_error_frame=False,
                data_length=8,
                received_monotonic_ns=0,
                channel="gripper",
            )

    def policy_metadata(self):
        digest = "a" * 64
        return {
            "artifact_identity_schema": ARTIFACT_IDENTITY_SCHEMA,
            "prediction_only": True,
            "control_enabled": False,
            "robot_control_ready": False,
            "action_semantics": "next_observed_state_proxy_nominal_units",
            "config_schema": "pi05_proxy_shadow_v1",
            "action_horizon": 16,
            "first_target_dt_s": 0.5,
            "checkpoint": "cef166e2_two_step/1",
            "checkpoint_sha256": digest,
            "policy_config_sha256": "b" * 64,
            "policy_pipeline_code_sha256": "e" * 64,
            "training_config_code_sha256": "9" * 64,
            "normalization_identity": "proxy_dataset_train_split_stats",
            "normalization_sha256": "c" * 64,
            "dataset": "local/pi05_proxy_cef166e2_2fps",
            "dataset_tree_sha256": "d" * 64,
            "openpi_commit": "f" * 40,
            "hardware_control_allowed": False,
        }

    def identity_lock(self):
        metadata = self.policy_metadata()
        return {
            "schema": ARTIFACT_LOCK_SCHEMA,
            "artifact_identity_schema": metadata["artifact_identity_schema"],
            "config_schema": metadata["config_schema"],
            "checkpoint": metadata["checkpoint"],
            "dataset": metadata["dataset"],
            "normalization_identity": metadata["normalization_identity"],
            "action_semantics": metadata["action_semantics"],
            "hardware_control_allowed": False,
            "openpi_commit": metadata["openpi_commit"],
            "checkpoint_sha256": metadata["checkpoint_sha256"],
            "policy_config_sha256": metadata["policy_config_sha256"],
            "policy_pipeline_code_sha256": metadata["policy_pipeline_code_sha256"],
            "training_config_code_sha256": metadata["training_config_code_sha256"],
            "normalization_sha256": metadata["normalization_sha256"],
            "dataset_tree_sha256": metadata["dataset_tree_sha256"],
        }

    def derived_multi_metadata(self):
        return {
            "artifact_identity_schema": "piper_derived_state_multi_shadow_identity_v1",
            "config_schema": "pi05_derived_state_multi_episode_shadow_v1",
            "prediction_only": True,
            "control_enabled": False,
            "robot_control_ready": False,
            "hardware_control_allowed": False,
            "can_opened": False,
            "can_data_tx_calls": 0,
            "action_semantics": "next_observed_state_proxy_nominal_units",
            "action_target_kind": "derived_future_observed_state_position_target",
            "action_horizon": 16,
            "action_dimension": 7,
            "first_target_dt_s": 0.1,
            "checkpoint": "checkpoints/pi05_derived_state/five_demos/19",
            "checkpoint_tree_sha256": "a" * 64,
            "dataset": "local/five_demos",
            "dataset_identity": "local/five_demos",
            "dataset_sha256": "b" * 64,
            "normalization_identity": "train_only_stats:local/five_demos",
            "normalization_sha256": "c" * 64,
            "openpi_commit": "d" * 40,
            "training_provenance_sha256": "e" * 64,
        }

    def derived_multi_identity_lock(self):
        metadata = self.derived_multi_metadata()
        fields = (
            "artifact_identity_schema", "config_schema", "checkpoint",
            "checkpoint_tree_sha256", "dataset", "dataset_sha256",
            "normalization_identity", "normalization_sha256", "action_semantics",
            "action_target_kind", "action_horizon", "action_dimension",
            "first_target_dt_s", "hardware_control_allowed", "prediction_only",
            "openpi_commit", "training_provenance_sha256",
        )
        return {"schema": "piper_derived_state_multi_shadow_identity_lock_v1",
                **{field: metadata[field] for field in fields}}

    def test_multi_demo_shadow_metadata_and_lock_match_exact_training_artifacts(self) -> None:
        metadata = self.derived_multi_metadata()
        lock = self.derived_multi_identity_lock()
        validate_policy_metadata(metadata, lock)
        prediction = build_policy_prediction(
            {"actions": np.zeros((16, 7), dtype=np.float32)}, metadata,
            request_id="derived-shadow-1", request_sequence=1,
            observation_ns=100, response_ns=200,
            expected_identity_lock=lock,
        )
        self.assertEqual(prediction.target_dt_s[0], 0.1)
        self.assertEqual(prediction.target_dt_s[-1], 1.6)
        self.assertIn("sha256:" + "a" * 64, prediction.checkpoint_identity)
        self.assertIn("training-provenance-sha256:" + "e" * 64,
                      prediction.config_identity)
        self.assertIn("tree-sha256:" + "b" * 64, prediction.dataset_identity)
        self.assertFalse(prediction.hardware_units_verified)

    def test_multi_demo_shadow_rejects_identity_drift_or_control_metadata(self) -> None:
        metadata = self.derived_multi_metadata()
        lock = self.derived_multi_identity_lock()
        for field, value in (
            ("dataset_sha256", "f" * 64),
            ("normalization_sha256", "0" * 64),
            ("action_target_kind", "raw_can_command"),
            ("hardware_control_allowed", True),
            ("can_data_tx_calls", 1),
        ):
            with self.subTest(field=field):
                with self.assertRaises((ValueError, PermissionError)):
                    validate_policy_metadata({**metadata, field: value}, lock)

    def test_policy_prewarm_uses_no_hardware_observation(self) -> None:
        class FakeClient:
            observations = []

            def infer(self, observation):
                self.observations.append(observation)
                return {"actions": np.zeros((16, 7), dtype=np.float32)}

        client = FakeClient()
        result = prewarm_policy(client)
        self.assertEqual(result["status"], "PASS")
        self.assertFalse(result["hardware_observation_used"])
        self.assertEqual(result["action_shape"], [16, 7])
        self.assertEqual(len(client.observations), 1)
        self.assertEqual(client.observations[0]["state"].shape, (7,))

    def test_policy_result_keeps_proxy_semantics_and_time_axis(self) -> None:
        prediction = build_policy_prediction(
            {"actions": np.zeros((16, 7), dtype=np.float32)},
            self.policy_metadata(),
            request_id="shadow-1",
            request_sequence=1,
            observation_ns=100,
            response_ns=200,
        )
        self.assertEqual(prediction.action_semantics,
                         "next_observed_state_proxy_nominal_units")
        self.assertEqual(prediction.target_dt_s[0], 0.5)
        self.assertEqual(prediction.target_dt_s[-1], 8.0)
        self.assertFalse(prediction.hardware_units_verified)
        self.assertIn("sha256:" + "a" * 64, prediction.checkpoint_identity)
        self.assertIn("sha256:" + "b" * 64, prediction.config_identity)
        self.assertIn("pipeline-sha256:" + "e" * 64, prediction.config_identity)
        self.assertIn("training-config-sha256:" + "9" * 64, prediction.config_identity)
        self.assertIn("openpi:" + "f" * 40, prediction.config_identity)
        self.assertIn("sha256:" + "c" * 64, prediction.normalization_identity)
        self.assertIn("tree-sha256:" + "d" * 64, prediction.dataset_identity)

    def test_server_metadata_fails_closed_without_complete_artifact_hashes(self) -> None:
        metadata = self.policy_metadata()
        validate_policy_metadata(metadata)
        for field, value in (
            ("checkpoint_sha256", None),
            ("policy_config_sha256", "ABCDEF"),
            ("normalization_sha256", "0" * 63),
            ("dataset_tree_sha256", "g" * 64),
            ("policy_pipeline_code_sha256", "z" * 64),
            ("training_config_code_sha256", "not-a-digest"),
        ):
            with self.subTest(field=field):
                broken = {**metadata, field: value}
                with self.assertRaisesRegex(ValueError, field):
                    validate_policy_metadata(broken)

    def test_metadata_must_match_exact_local_identity_lock(self) -> None:
        metadata = self.policy_metadata()
        lock = self.identity_lock()
        validate_policy_metadata(metadata, lock)
        for field, value in (
            ("checkpoint_sha256", "9" * 64),
            ("policy_pipeline_code_sha256", "8" * 64),
            ("training_config_code_sha256", "6" * 64),
            ("openpi_commit", "7" * 40),
            ("hardware_control_allowed", True),
        ):
            with self.subTest(field=field):
                with self.assertRaisesRegex(PermissionError, field):
                    validate_policy_metadata({**metadata, field: value}, lock)

    def test_approved_shadow_fails_closed_without_local_identity_lock(self) -> None:
        result = run(5, 8002, 0, 1, approved=True)
        self.assertEqual(result["status"], "BLOCKED_ARTIFACT_IDENTITY_LOCK_REQUIRED")
        self.assertFalse(result["robot_command_sent"])
        self.assertEqual(result["can_data_tx_calls"], 0)

    def test_approved_shadow_fails_closed_without_camera_identity_binding(self) -> None:
        result = run(
            5, 8002, 0, 1, approved=True,
            expected_identity_lock=self.identity_lock(),
        )
        self.assertEqual(result["status"], "BLOCKED_CAMERA_IDENTITY_UNVERIFIED")
        self.assertFalse(result["camera_opened"])
        self.assertFalse(result["can_opened"])
        self.assertFalse(result["robot_command_sent"])
        self.assertEqual(result["can_data_tx_calls"], 0)

    def test_approved_shadow_rejects_camera_identity_inventory_failure(self) -> None:
        original = live_shadow.bind_camera_roles
        try:
            live_shadow.bind_camera_roles = lambda *args, **kwargs: (_ for _ in ()).throw(
                ValueError("camera is absent"))
            result = run(
                5, 8002, 0, 1, approved=True,
                expected_identity_lock=self.identity_lock(),
                base_device_id="BASE-ID", wrist_device_id="WRIST-ID",
                roles_visually_confirmed=True,
            )
        finally:
            live_shadow.bind_camera_roles = original
        self.assertEqual(result["status"], "BLOCKED_CAMERA_IDENTITY_CHECK_FAILED")
        self.assertIn("camera is absent", result["error"])
        self.assertFalse(result["camera_opened"])
        self.assertFalse(result["can_opened"])
        self.assertEqual(result["can_data_tx_calls"], 0)

    def test_approved_shadow_passes_camera_binding_to_bounded_worker(self) -> None:
        from laptop.diagnostics import bounded_process

        identity = {
            "schema": "piper_camera_role_identity_v1",
            "index_to_instance_id_automatically_proven": False,
        }
        original_binding = live_shadow.bind_camera_roles
        original_worker = bounded_process.run_bounded
        observed = {}
        try:
            live_shadow.bind_camera_roles = lambda *args, **kwargs: identity

            def fake_worker(target, *, args, timeout_s):
                observed["args"] = args
                observed["timeout_s"] = timeout_s
                return {"status": "COMPLETE", "result": {"status": "PARTIAL"}}

            bounded_process.run_bounded = fake_worker
            result = run(
                5, 8002, 0, 1, approved=True,
                expected_identity_lock=self.identity_lock(),
                base_device_id="BASE-ID", wrist_device_id="WRIST-ID",
                roles_visually_confirmed=True,
            )
        finally:
            live_shadow.bind_camera_roles = original_binding
            bounded_process.run_bounded = original_worker
        self.assertEqual(result["status"], "PARTIAL")
        self.assertIs(observed["args"][-1], identity)

    def test_checked_in_artifact_identity_lock_is_well_formed(self) -> None:
        lock = load_artifact_identity_lock(
            Path(__file__).resolve().parents[2]
            / "configs/runtime/pi05_proxy_shadow_identity_lock.json"
        )
        self.assertEqual(lock["checkpoint"],
                         "checkpoints/pi05_piper_next_state_proxy_tiny_fit/cef166e2_two_step/1")
        self.assertFalse(lock["hardware_control_allowed"])

    def test_server_metadata_must_remain_reject_only_and_timing_pinned(self) -> None:
        metadata = self.policy_metadata()
        for field, value in (
            ("prediction_only", False),
            ("control_enabled", True),
            ("robot_control_ready", True),
            ("first_target_dt_s", 0.04),
        ):
            with self.subTest(field=field):
                broken = {**metadata, field: value}
                with self.assertRaises((ValueError, PermissionError)):
                    validate_policy_metadata(broken)

    def test_latency_segments_and_missing_timing(self) -> None:
        timing = segment_latency({"policy_timing": {"infer_ms": 1000.0},
                                  "server_timing": {"infer_ms": 1040.0}}, 1110.0)
        self.assertEqual(timing["model_sampling_ms"], 1000.0)
        self.assertEqual(timing["server_non_model_ms"], 40.0)
        self.assertEqual(timing["client_server_residual_ms"], 70.0)
        with self.assertRaisesRegex(ValueError, "missing or invalid"):
            segment_latency({}, 1110.0)
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            segment_latency({"policy_timing": {"infer_ms": 1200.0},
                             "server_timing": {"infer_ms": 1000.0}}, 1110.0)

    def setUp(self) -> None:
        self.now = 1_000_000_000
        self.cameras = {
            "base": {"frame": np.zeros((8, 8, 3), dtype=np.uint8), "timestamp_ns": self.now - 20_000_000},
            "wrist": {"frame": np.zeros((8, 8, 3), dtype=np.uint8), "timestamp_ns": self.now - 25_000_000},
        }
        self.feedback = {
            "timestamps_ns": {key: self.now - 15_000_000 for key in
                              ("status", "joint_12", "joint_34", "joint_56", "gripper")},
            "joint_raw": [0, 1000, 0, 0, 0, 0], "gripper_raw": 40200,
            "status": {"err_code": 0},
        }

    def test_fresh_observation_has_two_cameras_and_7d_state(self) -> None:
        observation, timing = select_observation(self.cameras, self.feedback, self.now)
        self.assertEqual(observation["state"].shape, (7,))
        self.assertAlmostEqual(float(observation["state"][6]), 0.0402, places=5)
        self.assertEqual(timing["camera_skew_ms"], 5)

    def test_robot_state_snapshot_preserves_can_freshness_and_stays_unverified(self) -> None:
        self.feedback["timestamps_ns"].update({
            "status": self.now - 18_000_000,
            "joint_12": self.now - 20_000_000,
            "joint_34": self.now - 16_000_000,
            "joint_56": self.now - 14_000_000,
            "gripper": self.now - 12_000_000,
        })
        observation, timing = select_observation(self.cameras, self.feedback, self.now)
        snapshot = state_snapshot_from_live_observation(
            observation,
            timing,
            self.feedback,
            sequence=11,
            connected=True,
        )

        self.assertEqual(snapshot.sequence, 11)
        self.assertEqual(snapshot.captured_monotonic_ns, self.now - 20_000_000)
        self.assertEqual(snapshot.received_monotonic_ns, self.now - 12_000_000)
        self.assertEqual(snapshot.joint_position_rad, tuple(observation["state"][:6]))
        self.assertAlmostEqual(snapshot.gripper_position_m, 0.0402, places=5)
        self.assertTrue(snapshot.status_ok)
        self.assertTrue(snapshot.connected)
        self.assertFalse(snapshot.units_verified)

    def test_startup_readiness_names_missing_camera_and_can_channels(self) -> None:
        empty = initial_sensor_readiness({}, {})
        self.assertFalse(empty["ready"])
        self.assertEqual(empty["missing_camera_roles"], ["base", "wrist"])
        self.assertEqual(empty["missing_can_channels"],
                         ["status", "joint_12", "joint_34", "joint_56", "gripper"])

        partial = initial_sensor_readiness(
            {"base": self.cameras["base"]},
            {"timestamps_ns": {
                key: self.feedback["timestamps_ns"][key]
                for key in ("status", "joint_12", "joint_34", "joint_56")
            }, "status": dict(self.feedback["status"]),
                "joint_raw": list(self.feedback["joint_raw"]),
                "gripper_raw": self.feedback["gripper_raw"]},
        )
        self.assertFalse(partial["ready"])
        self.assertEqual(partial["missing_camera_roles"], ["wrist"])
        self.assertEqual(partial["missing_can_channels"], ["gripper"])

    def test_startup_readiness_is_true_only_after_all_initial_snapshots_exist(self) -> None:
        ready = initial_sensor_readiness(self.cameras, self.feedback)
        self.assertTrue(ready["ready"])
        self.assertEqual(ready["missing_camera_roles"], [])
        self.assertEqual(ready["missing_can_channels"], [])

    def test_signed_gripper_feedback_is_present_and_preserved(self) -> None:
        self.feedback["gripper_raw"] = -100
        ready = initial_sensor_readiness(self.cameras, self.feedback)
        self.assertTrue(ready["ready"])
        self.assertEqual(ready["missing_can_channels"], [])

        observation, _ = select_observation(self.cameras, self.feedback, self.now)
        self.assertAlmostEqual(float(observation["state"][6]), -0.0001, places=7)

        self.feedback["gripper_raw"] = True
        self.assertFalse(initial_sensor_readiness(self.cameras, self.feedback)["ready"])
        with self.assertRaisesRegex(ValueError, "invalid joint/gripper"):
            select_observation(self.cameras, self.feedback, self.now)

    def test_missing_or_stale_feedback_rejected(self) -> None:
        del self.feedback["timestamps_ns"]["gripper"]
        with self.assertRaisesRegex(ValueError, "missing CAN"):
            select_observation(self.cameras, self.feedback, self.now)
        self.feedback["timestamps_ns"]["gripper"] = self.now - 300_000_000
        with self.assertRaisesRegex(ValueError, "stale CAN"):
            select_observation(self.cameras, self.feedback, self.now)

    def test_skew_and_unapproved_run_rejected(self) -> None:
        self.cameras["wrist"]["timestamp_ns"] = self.now - 150_000_000
        with self.assertRaisesRegex(ValueError, "camera skew"):
            select_observation(self.cameras, self.feedback, self.now)
        self.assertEqual(run(5, 8002, 0, 1)["status"], "BLOCKED_APPROVAL_REQUIRED")

    def test_saved_proxy_predictions_are_vetoed_offline(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            limits = root / "limits.yaml"
            limits.write_text("hardware_status: NOT_REAL_CONFIRMED_LIMITS\n" +
                              "\n".join(f"joint{i}: {{min: -3, max: 3}}" for i in range(1, 7)) +
                              "\nstatus: PENDING_GETSDKGRIPPERRANGEPARAM\n")
            report = root / "report.json"
            prediction = {
                "request_ns": self.now, "response_ns": self.now + 2_000_000_000,
                "can_timestamps_ns": self.feedback["timestamps_ns"],
                "base_timestamp_ns": self.cameras["base"]["timestamp_ns"],
                "wrist_timestamp_ns": self.cameras["wrist"]["timestamp_ns"],
                "state": [0.0] * 6 + [0.04],
                "first_predicted_proxy": [0.2] * 6 + [0.05],
            }
            report.write_text(json.dumps({
                "status": "LIVE_SHADOW_CONTINUOUS_PASS",
                "robot_command_sent": False, "can_data_tx_calls": 0,
                "control_enabled": False, "units_hardware_verified": False,
                "predictions": [prediction, prediction],
            }))
            result = audit(report, limits)
            self.assertEqual(result["rejected"], 2)
            self.assertEqual(result["reason_counts"]["unit_mismatch"], 2)
            self.assertFalse(result["hardware_control_allowed"])


if __name__ == "__main__":
    unittest.main()
