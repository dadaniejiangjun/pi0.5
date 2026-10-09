from __future__ import annotations

import copy
from dataclasses import asdict
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from laptop.robot_client.offline_replay import FakeRobotTransport, main, replay
from laptop.safety.offline_guard import SafetyPolicy, validate_candidate


NOW = 1_000_000_000


def fixture():
    policy = SafetyPolicy(
        joint_min_rad=(-3.0,) * 6, joint_max_rad=(3.0,) * 6,
        gripper_min_m=0.0, gripper_max_m=0.07,
        max_joint_speed_rad_s=0.2, max_joint_accel_rad_s2=2.0,
        max_gripper_speed_m_s=0.02,
        max_state_age_ns=50_000_000, max_camera_age_ns=50_000_000,
        max_response_age_ns=50_000_000, max_camera_skew_ns=10_000_000,
        max_watchdog_gap_ns=50_000_000,
    )
    snapshot = {
        "request_id": "offline-one", "request_seq": 1,
        "request_issued_ns": NOW - 30_000_000,
        "request_deadline_ns": NOW + 30_000_000,
        "state_timestamp_ns": NOW - 10_000_000,
        "base_timestamp_ns": NOW - 10_000_000,
        "wrist_timestamp_ns": NOW - 10_000_000,
        "watchdog_timestamp_ns": NOW - 10_000_000,
        "action_horizon_ns": 100_000_000,
        "ctrl_mode": "CAN_CTRL(0x1)", "arm_status": "NORMAL(0x0)", "err_code": 0,
        "joint_unit": "radian", "gripper_unit": "meter",
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "units_hardware_verified": True,
        "state": [0.0] * 7, "prior_joint_velocity_rad_s": [0.0] * 6,
    }
    response = {
        "request_id": "offline-one", "response_seq": 1,
        "received_timestamp_ns": NOW - 5_000_000,
        "joint_unit": "radian", "gripper_unit": "meter",
        "action_semantics": "absolute_joint_position_rad_gripper_meter",
        "action": [0.01] * 6 + [0.001],
    }
    return policy, snapshot, response


class OfflineSafetyBridgeTest(unittest.TestCase):
    def check_rejected(self, expected: str, edit) -> None:
        policy, snapshot, response = fixture()
        edit(snapshot, response)
        result = validate_candidate(snapshot, response, policy, now_ns=NOW, last_response_seq=0)
        self.assertFalse(result.accepted_for_offline_analysis)
        self.assertFalse(result.hardware_control_allowed)
        self.assertIn(expected, result.reasons)

    def test_valid_offline_candidate_still_has_no_hardware_permission(self) -> None:
        policy, snapshot, response = fixture()
        result = validate_candidate(snapshot, response, policy, now_ns=NOW, last_response_seq=0)
        self.assertTrue(result.accepted_for_offline_analysis)
        self.assertFalse(result.hardware_control_allowed)

    def test_drop_and_reordering(self) -> None:
        policy, snapshot, response = fixture()
        missing = validate_candidate(snapshot, None, policy, now_ns=NOW, last_response_seq=0)
        self.assertIn("response_missing", missing.reasons)
        self.check_rejected("response_out_of_order", lambda s, r: r.update(response_seq=0))
        self.check_rejected("response_out_of_order", lambda s, r: r.update(request_id="other"))

    def test_policy_network_drop_is_audited_and_never_sent(self) -> None:
        policy, snapshot, _ = fixture()
        report = replay([{
            "snapshot": snapshot,
            "response": None,
            "policy_transport_error": "timeout",
            "now_ns": NOW,
        }], policy)
        self.assertEqual(report["rejected"], 1)
        self.assertIn("response_missing", report["decisions"][0]["reasons"])
        self.assertIn("policy_network_timeout", report["decisions"][0]["reasons"])
        self.assertEqual(report["send_attempts"], 0)
        self.assertFalse(report["hardware_control_allowed"])

    def test_policy_network_failures_are_audited_and_never_sent(self) -> None:
        policy, snapshot, _ = fixture()
        expected_reasons = {
            "timeout": "policy_network_timeout",
            "connection_refused": "policy_network_connection_refused",
            "connection_reset": "policy_network_connection_reset",
        }
        for error, expected_reason in expected_reasons.items():
            with self.subTest(error=error):
                report = replay([{
                    "snapshot": snapshot,
                    "response": None,
                    "policy_transport_error": error,
                    "now_ns": NOW,
                }], policy)
                self.assertEqual(report["rejected"], 1)
                self.assertIn(expected_reason, report["decisions"][0]["reasons"])
                self.assertEqual(report["send_attempts"], 0)
                self.assertFalse(report["hardware_control_allowed"])

    def test_unclassified_network_error_fails_closed(self) -> None:
        policy, snapshot, _ = fixture()
        report = replay([{
            "snapshot": snapshot,
            "response": None,
            "policy_transport_error": "unrecognized_network_error",
            "now_ns": NOW,
        }], policy)
        self.assertEqual(report["rejected"], 1)
        self.assertIn("unknown_policy_transport_error", report["decisions"][0]["reasons"])
        self.assertEqual(report["send_attempts"], 0)
        self.assertFalse(report["hardware_control_allowed"])

    def test_response_with_transport_error_fails_closed(self) -> None:
        policy, snapshot, response = fixture()
        report = replay([{
            "snapshot": snapshot,
            "response": response,
            "policy_transport_error": "connection_reset",
            "now_ns": NOW,
        }], policy)
        self.assertEqual(report["rejected"], 1)
        self.assertIn("response_with_transport_error", report["decisions"][0]["reasons"])
        self.assertEqual(report["send_attempts"], 0)
        self.assertFalse(report["hardware_control_allowed"])

    def test_timeout_and_stale_inputs(self) -> None:
        self.check_rejected("request_expired_or_unordered",
                            lambda s, r: s.update(request_deadline_ns=NOW - 1))
        self.check_rejected("response_stale_or_missing",
                            lambda s, r: r.update(received_timestamp_ns=NOW - 100_000_000))
        self.check_rejected("state_stale_or_missing",
                            lambda s, r: s.update(state_timestamp_ns=NOW - 100_000_000))
        self.check_rejected("base_stale_or_missing",
                            lambda s, r: s.update(base_timestamp_ns=None))
        self.check_rejected("camera_skew",
                            lambda s, r: s.update(wrist_timestamp_ns=NOW - 45_000_000))
        self.check_rejected("watchdog_stale_or_missing",
                            lambda s, r: s.update(watchdog_timestamp_ns=NOW - 100_000_000))

    def test_nan_units_bounds_velocity_acceleration(self) -> None:
        self.check_rejected("invalid_action", lambda s, r: r["action"].__setitem__(0, float("nan")))
        self.check_rejected("invalid_state", lambda s, r: s["state"].__setitem__(0, float("inf")))
        self.check_rejected("unit_mismatch", lambda s, r: s.update(joint_unit="degree"))
        self.check_rejected("response_unit_mismatch", lambda s, r: r.update(gripper_unit="millimeter"))
        self.check_rejected("action_out_of_bounds", lambda s, r: r["action"].__setitem__(0, 4.0))
        self.check_rejected("state_out_of_bounds", lambda s, r: s["state"].__setitem__(6, 0.1))
        self.check_rejected("velocity_limit", lambda s, r: r["action"].__setitem__(0, 0.05))
        self.check_rejected("acceleration_limit",
                            lambda s, r: s["prior_joint_velocity_rad_s"].__setitem__(0, -0.2))

    def test_proxy_action_or_unverified_hardware_units_rejected(self) -> None:
        self.check_rejected("action_semantics_unverified",
                            lambda s, r: r.update(action_semantics="next_observed_state_proxy_nominal_units"))
        self.check_rejected("hardware_units_unverified",
                            lambda s, r: s.update(units_hardware_verified=False))

    def test_mode_health_and_horizon_fail_closed(self) -> None:
        self.check_rejected("control_mode_unverified",
                            lambda s, r: s.update(ctrl_mode="TEACHING_MODE(0x2)"))
        self.check_rejected("robot_health_unverified",
                            lambda s, r: s.update(err_code=1))
        self.check_rejected("invalid_action_horizon",
                            lambda s, r: s.update(action_horizon_ns=10_000_000_000))

    def test_fake_transport_never_sends(self) -> None:
        policy, snapshot, response = fixture()
        row = {"snapshot": snapshot, "response": response, "now_ns": NOW}
        second = copy.deepcopy(row)
        report = replay([row, second], policy)
        self.assertEqual(report["accepted_for_offline_analysis"], 1)
        self.assertEqual(report["rejected"], 1)
        self.assertIn("response_out_of_order", report["decisions"][1]["reasons"])
        self.assertEqual(report["send_attempts"], 0)
        self.assertFalse(report["hardware_control_allowed"])
        fake = FakeRobotTransport([])
        with self.assertRaises(PermissionError):
            fake.send(response["action"])
        self.assertEqual(fake.send_attempts, 1)

    def test_file_replay_cli_is_read_only(self) -> None:
        policy, snapshot, response = fixture()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            policy_path = root / "offline-policy.json"
            events_path = root / "events.jsonl"
            policy_path.write_text(json.dumps(asdict(policy)), encoding="utf-8")
            events_path.write_text(json.dumps({
                "snapshot": snapshot, "response": response, "now_ns": NOW,
            }) + "\n", encoding="utf-8")
            stdout = io.StringIO()
            with patch("sys.argv", ["offline_replay", "--events", str(events_path),
                                    "--policy", str(policy_path)]), redirect_stdout(stdout):
                self.assertEqual(main(), 0)
            report = json.loads(stdout.getvalue())
            self.assertEqual(report["status"], "OFFLINE_REPLAY_ONLY")
            self.assertEqual(report["send_attempts"], 0)
            self.assertFalse(report["hardware_control_allowed"])


if __name__ == "__main__":
    unittest.main()
