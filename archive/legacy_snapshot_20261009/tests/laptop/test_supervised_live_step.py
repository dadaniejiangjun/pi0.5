from __future__ import annotations

from pathlib import Path
import sys
import time
import types

import numpy as np
import pytest

from laptop.client import live_shadow
from laptop.client.observation_schema import PIPER_PROMPT
from laptop.control.contracts import PolicyPrediction
from laptop.client.supervised_live_step import (
    _worker,
    live_robot_status_ready,
    observation_follows_write,
    post_send_feedback_outcome,
    run_supervised_one_step,
)
from laptop.robot_client.single_step_dry_run import P12GateEvidence
from laptop.robot_client.supervised_policy_session import (
    SHORT_LOOP_CONFIRMATION_PHRASE,
    request_short_loop_operator_grant,
)
from laptop.robot_client.supervised_site_profile import SupervisedSiteProfile
from laptop.safety.bridge import SafetyLimits
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.position_commands import P12_SITE_GATE_NAMES, CanTxFrame


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SITE_TEMPLATE = PROJECT_ROOT / "configs" / "runtime" / "supervised_site_profile.template.json"


def test_supervised_live_step_default_does_not_open_devices_or_policy():
    result = run_supervised_one_step()

    assert result["status"] == "BLOCKED_EXPLICIT_SUPERVISED_STEP_FLAG_REQUIRED"
    assert result["camera_opened"] is False
    assert result["can_opened"] is False
    assert result["position_send_attempts"] == 0


def test_draft_site_profile_blocks_before_camera_binding_or_policy(monkeypatch):
    def unexpected_camera_binding(*args, **kwargs):
        raise AssertionError("camera binding must not run before site-profile validation")

    monkeypatch.setattr(
        "laptop.client.supervised_live_step.bind_camera_roles",
        unexpected_camera_binding,
    )
    result = run_supervised_one_step(
        approved=True,
        expected_identity_lock={},
        site_profile_path=SITE_TEMPLATE,
        port=8002,
        base_index=0,
        wrist_index=1,
        base_device_id="base-camera-id",
        wrist_device_id="wrist-camera-id",
        roles_visually_confirmed=True,
        input_fn=lambda prompt: "AUTHORIZE ONE POSITION STEP",
        output_fn=lambda line: None,
    )

    assert result["status"] == "BLOCKED_SUPERVISED_PREFLIGHT"
    assert "not declared ready" in result["error"]
    assert result["camera_opened"] is False
    assert result["can_opened"] is False
    assert result["position_send_attempts"] == 0


def test_missing_profile_blocks_before_any_device_access():
    result = run_supervised_one_step(
        approved=True,
        expected_identity_lock={},
        site_profile_path=None,
        base_index=0,
        wrist_index=1,
        base_device_id="base-camera-id",
        wrist_device_id="wrist-camera-id",
        roles_visually_confirmed=True,
    )

    assert result["status"] == "BLOCKED_SITE_PROFILE_REQUIRED"
    assert result["camera_opened"] is False
    assert result["can_opened"] is False
    assert result["position_send_attempts"] == 0


def test_short_loop_requires_ready_profile_before_any_device_access():
    result = run_supervised_one_step(
        approved=True,
        expected_identity_lock={},
        site_profile_path=SITE_TEMPLATE,
        base_index=0,
        wrist_index=1,
        base_device_id="base-camera-id",
        wrist_device_id="wrist-camera-id",
        roles_visually_confirmed=True,
        _max_steps=3,
        input_fn=lambda prompt: "AUTHORIZE 3 POSITION STEPS",
        output_fn=lambda line: None,
    )

    assert result["status"] == "BLOCKED_SUPERVISED_PREFLIGHT"
    assert "not declared ready" in result["error"]
    assert result["camera_opened"] is False
    assert result["can_opened"] is False
    assert result["position_send_attempts"] == 0


def test_short_loop_next_observation_requires_both_cameras_and_all_can_after_write():
    write_ns = 1_000
    timing = {
        "base_timestamp_ns": 1_001,
        "wrist_timestamp_ns": 1_002,
        "can_timestamps_ns": {
            "status": 1_003,
            "joint_12": 1_004,
            "joint_34": 1_005,
            "joint_56": 1_006,
            "gripper": 1_007,
        },
    }
    assert observation_follows_write(timing, write_ns)

    timing["wrist_timestamp_ns"] = write_ns
    assert not observation_follows_write(timing, write_ns)

    timing["wrist_timestamp_ns"] = write_ns + 2
    timing["can_timestamps_ns"]["gripper"] = write_ns
    assert not observation_follows_write(timing, write_ns)


@pytest.mark.parametrize("freeze_after_first_batch", [False, True])
def test_short_loop_worker_stops_or_continues_on_memory_feedback(monkeypatch,
                                                               freeze_after_first_batch):
    class MemoryAdapter:
        def __init__(self):
            self.sent: list[CanTxFrame] = []

        def open_read_only(self):
            pass

        def receive_frame(self, timeout_ms):
            return None

        def send_diagnostic_query(self, query):
            return True

        def send_position_frame(self, frame):
            self.sent.append(frame)
            return True

        def close(self):
            pass

    class FakePolicy:
        def __init__(self, host, port):
            pass

        def get_server_metadata(self):
            return {}

        def infer(self, observation):
            return {}

    adapter = MemoryAdapter()
    transport = GsUsbPiperCanTransport(adapter_factory=lambda: adapter)
    monkeypatch.setattr(
        "laptop.client.supervised_live_step.GsUsbPiperCanTransport",
        lambda: transport,
    )
    client_module = types.ModuleType("openpi_client.websocket_client_policy")
    client_module.WebsocketClientPolicy = FakePolicy
    package_module = types.ModuleType("openpi_client")
    package_module.__path__ = []
    monkeypatch.setitem(sys.modules, "openpi_client", package_module)
    monkeypatch.setitem(sys.modules, "openpi_client.websocket_client_policy", client_module)

    identities = {
        "checkpoint_identity": "checkpoint:short-loop-test",
        "config_identity": "config:short-loop-test",
        "normalization_identity": "normalization:short-loop-test",
        "dataset_identity": "dataset:short-loop-test",
    }
    monkeypatch.setattr(
        "laptop.client.supervised_live_step._metadata_identities",
        lambda metadata, lock: identities,
    )
    monkeypatch.setattr(live_shadow, "validate_policy_metadata", lambda metadata, lock: None)

    def fake_camera_loop(index, role, shared, lock, stop, errors):
        while not stop.is_set():
            if not freeze_after_first_batch or len(adapter.sent) < 4:
                with lock:
                    shared[role] = {"frame": None, "timestamp_ns": time.perf_counter_ns()}
            time.sleep(0.001)

    def fake_can_loop(bridge, feedback, lock, stop, errors):
        bridge.start_read_only()
        while not stop.is_set():
            if not freeze_after_first_batch or len(adapter.sent) < 4:
                stamp = time.perf_counter_ns()
                with lock:
                    feedback.update({
                        "status": {
                            "ctrl_mode": "CAN_CTRL(0x1)",
                            "arm_status": "NORMAL(0x0)",
                            "err_code": 0,
                        },
                        "joint_raw": [0] * 6,
                        "gripper_raw": 50_000,
                        "timestamps_ns": {
                            name: stamp for name in (
                                "status", "joint_12", "joint_34", "joint_56", "gripper",
                            )
                        },
                    })
            time.sleep(0.001)

    def fake_initial_readiness(cameras, feedback):
        ready = set(cameras) >= {"base", "wrist"} and len(feedback.get("timestamps_ns", {})) == 5
        return {"ready": ready, "status": "READY" if ready else "WAITING"}

    def fake_select_observation(cameras, feedback, now_ns):
        can_times = dict(feedback["timestamps_ns"])
        return {"state": np.asarray([0.0] * 6 + [0.05], dtype=np.float32)}, {
            "base_timestamp_ns": cameras["base"]["timestamp_ns"],
            "wrist_timestamp_ns": cameras["wrist"]["timestamp_ns"],
            "can_timestamps_ns": can_times,
            "observation_built_ns": now_ns,
        }

    def fake_prediction(result, metadata, *, request_id, request_sequence,
                        observation_ns, response_ns, expected_identity_lock):
        return PolicyPrediction(
            request_id=request_id,
            request_sequence=request_sequence,
            observation_monotonic_ns=observation_ns,
            received_monotonic_ns=response_ns,
            actions=tuple((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05) for _ in range(16)),
            target_dt_s=tuple(0.1 * (index + 1) for index in range(16)),
            **identities,
        )

    monkeypatch.setattr(live_shadow, "_camera_loop", fake_camera_loop)
    monkeypatch.setattr(live_shadow, "_can_loop", fake_can_loop)
    monkeypatch.setattr(live_shadow, "initial_sensor_readiness", fake_initial_readiness)
    monkeypatch.setattr(live_shadow, "select_observation", fake_select_observation)
    monkeypatch.setattr(live_shadow, "build_policy_prediction", fake_prediction)

    limits = SafetyLimits(
        joint_min_rad=(-1.0,) * 6,
        joint_max_rad=(1.0,) * 6,
        max_joint_delta_rad=(0.01,) * 6,
        gripper_min_m=0.0,
        gripper_max_m=0.08,
        max_gripper_delta_m=0.001,
    )
    gates = P12GateEvidence(**{name: True for name in P12_SITE_GATE_NAMES})
    profile = SupervisedSiteProfile(
        site_id="memory-test",
        robot_serial="memory-test",
        firmware_version="memory-test",
        commissioned_by="test",
        commissioned_at_utc="2026-10-07T00:00:00+00:00",
        gripper_effort_raw=120,
        limits=limits,
        p12_evidence=gates,
        evidence_refs=tuple((name, "memory fixture") for name in P12_SITE_GATE_NAMES),
    )
    grant = request_short_loop_operator_grant(
        task_description=PIPER_PROMPT,
        **identities,
        gripper_effort_raw=120,
        limits=limits,
        site_gates=gates,
        input_fn=lambda prompt: SHORT_LOOP_CONFIRMATION_PHRASE,
        output_fn=lambda line: None,
    )

    result = _worker(
        3.0,
        8002,
        0,
        1,
        {},
        {"base": "base", "wrist": "wrist"},
        profile,
        grant.to_worker_ticket(),
    )

    if freeze_after_first_batch:
        assert result["status"] == "POSITION_BATCH_ACCEPTED_NO_FRESH_FEEDBACK_STOP_DO_NOT_RETRY"
        assert len(result["steps"]) == 1
        assert result["steps"][0]["post_send_feedback_observed"] is False
        assert result["steps"][0]["retry_prohibited_until_read_only_feedback"] is True
        assert result["position_frames_accepted"] == 4
        assert len(adapter.sent) == 4
    else:
        assert result["status"] == "SHORT_LOOP_COMPLETE_FRESH_FEEDBACK_OBSERVED", result
        assert len(result["steps"]) == 3
        assert all(step["post_send_feedback_observed"] for step in result["steps"])
        assert result["position_frames_accepted"] == 12
        assert len(adapter.sent) == 12
    assert result["robot_motion_confirmed"] is False


def test_live_position_worker_requires_existing_can_control_mode_and_healthy_arm():
    assert live_robot_status_ready({
        "ctrl_mode": "CAN_CTRL(0x1)",
        "arm_status": "NORMAL(0x0)",
        "err_code": 0,
    })
    assert not live_robot_status_ready({
        "ctrl_mode": "STANDBY(0x0)",
        "arm_status": "NORMAL(0x0)",
        "err_code": 0,
    })
    assert not live_robot_status_ready({
        "ctrl_mode": "TEACHING_MODE(0x2)",
        "arm_status": "NORMAL(0x0)",
        "err_code": 0,
    })
    assert not live_robot_status_ready({
        "ctrl_mode": "CAN_CTRL(0x1)",
        "arm_status": "NORMAL(0x0)",
        "err_code": 7,
    })


def test_adapter_acceptance_without_fresh_feedback_requires_stop_and_no_retry():
    outcome = post_send_feedback_outcome(None)

    assert outcome == {
        "status": "POSITION_BATCH_ACCEPTED_NO_FRESH_FEEDBACK_STOP_DO_NOT_RETRY",
        "post_send_feedback_observed": False,
        "retry_prohibited_until_read_only_feedback": True,
        "robot_motion_confirmed": False,
    }


def test_fresh_post_send_feedback_does_not_claim_motion_was_confirmed():
    outcome = post_send_feedback_outcome({"sequence": 12})

    assert outcome == {
        "status": "POSITION_BATCH_ACCEPTED_FRESH_FEEDBACK_OBSERVED",
        "post_send_feedback_observed": True,
        "retry_prohibited_until_read_only_feedback": False,
        "robot_motion_confirmed": False,
    }
