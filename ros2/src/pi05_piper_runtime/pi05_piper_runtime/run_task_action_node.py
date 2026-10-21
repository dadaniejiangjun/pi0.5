"""Fail-closed RunTask action endpoint.

The default profile rejects goals. ``fake_test`` exercises the action lifecycle
without robot inputs. ``prediction_only`` observes validated rolling previews
and always aborts as unverified; it never publishes a command or opens hardware.
Its preview liveness timeout is not a physical robot watchdog.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import threading
import time
import uuid
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from pi05_piper_interfaces.action import RunTask
from pi05_piper_interfaces.msg import StateSnapshot


class RunTaskActionNode(Node):
    """Expose RunTask while keeping the physical execution path unavailable."""

    def __init__(self) -> None:
        super().__init__("pi05_run_task_action")
        self.declare_parameter("execution_mode", "disabled")
        self.declare_parameter("fake_result_delay_s", 0.25)
        self.declare_parameter("first_preview_timeout_s", 2.0)
        self.declare_parameter("preview_gap_timeout_s", 1.0)
        self.execution_mode = str(self.get_parameter("execution_mode").value)
        self.fake_result_delay_s = float(self.get_parameter("fake_result_delay_s").value)
        self.first_preview_timeout_s = float(
            self.get_parameter("first_preview_timeout_s").value
        )
        self.preview_gap_timeout_s = float(
            self.get_parameter("preview_gap_timeout_s").value
        )
        if self.execution_mode not in {"disabled", "fake_test", "prediction_only"}:
            raise ValueError(
                "execution_mode must be 'disabled', 'fake_test', or 'prediction_only'"
            )
        if not 0.0 <= self.fake_result_delay_s <= 5.0:
            raise ValueError("fake_result_delay_s must be within [0, 5] seconds")
        if (not math.isfinite(self.first_preview_timeout_s)
                or not 0.1 <= self.first_preview_timeout_s <= 10.0):
            raise ValueError("first_preview_timeout_s must be within [0.1, 10] seconds")
        if (not math.isfinite(self.preview_gap_timeout_s)
                or not 0.1 <= self.preview_gap_timeout_s <= 10.0):
            raise ValueError("preview_gap_timeout_s must be within [0.1, 10] seconds")

        self._active_lock = threading.Lock()
        self._active = False
        self._active_preview_identity: tuple[str, str] | None = None
        self._prediction_updates = 0
        self._action_started_monotonic_ns = 0
        self._last_valid_preview_monotonic_ns: int | None = None
        self._last_preview_state_sequence = 0
        self._seen_observation_ids: set[str] = set()
        self._preview_fault = ""
        self._callback_group = ReentrantCallbackGroup()
        project_root = Path(os.environ.get(
            "PI05_PROJECT_ROOT", str(Path(__file__).resolve().parents[4])
        )).resolve()
        if str(project_root) not in sys.path:
            sys.path.insert(0, str(project_root))
        from pi05_piper_interfaces.msg import TargetPreview
        from laptop.piper.control_profile import (
            CONTINUOUS_GRIPPER_MAX_M,
            CONTINUOUS_GRIPPER_MAX_STEP_M,
            CONTINUOUS_GRIPPER_MIN_M,
            TASK_JOINT_BOUNDS_DEG,
        )
        self._joint_bounds_rad = tuple(
            (math.radians(low), math.radians(high)) for low, high in TASK_JOINT_BOUNDS_DEG
        )
        self._gripper_min_m = CONTINUOUS_GRIPPER_MIN_M
        self._gripper_max_m = CONTINUOUS_GRIPPER_MAX_M
        self._gripper_step_m = CONTINUOUS_GRIPPER_MAX_STEP_M
        self._latest_state: Any | None = None
        self._latest_state_received_ns = 0
        self._state_lock = threading.Lock()

        self._preview_sub = self.create_subscription(
            TargetPreview, "/pi05/target_preview", self._on_target_preview, 10
        )
        self._state_sub = self.create_subscription(
            StateSnapshot, "/piper/state_snapshot", self._on_state, 10
        )
        self._server = ActionServer(
            self,
            RunTask,
            "/pi05/run_task",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._callback_group,
        )
        if self.execution_mode == "disabled":
            self.get_logger().warning(
                "RunTask is disabled: no trusted permit issuer or hardware backend is configured"
            )
        elif self.execution_mode == "fake_test":
            self.get_logger().warning(
                "FAKE TEST ONLY: accepted goals will abort without robot commands"
            )
        else:
            self.get_logger().warning(
                "PREDICTION ONLY: RunTask can observe model previews but cannot command hardware"
            )

    def _goal(self, request: Any) -> GoalResponse:
        if self.execution_mode == "disabled":
            self.get_logger().warning("rejecting RunTask: execution_mode is disabled")
            return GoalResponse.REJECT
        if self.execution_mode == "fake_test":
            if not self._valid_fake_goal(request):
                self.get_logger().warning("rejecting malformed or non-fake RunTask goal")
                return GoalResponse.REJECT
        elif not self._valid_prediction_only_goal(request):
            self.get_logger().warning(
                "rejecting malformed RunTask prediction-only goal"
            )
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active:
                self.get_logger().warning("rejecting RunTask: another goal is active")
                return GoalResponse.REJECT
            self._active = True
            if self.execution_mode == "prediction_only":
                self._active_preview_identity = (
                    str(request.model_profile), str(request.checkpoint_id)
                )
                self._prediction_updates = 0
                self._action_started_monotonic_ns = time.monotonic_ns()
                self._last_valid_preview_monotonic_ns = None
                self._last_preview_state_sequence = 0
                self._seen_observation_ids.clear()
                self._preview_fault = ""
        return GoalResponse.ACCEPT

    @staticmethod
    def _valid_fake_goal(request: Any) -> bool:
        required = (
            request.task,
            request.model_profile,
            request.checkpoint_id,
            request.authorization_id,
        )
        return (
            all(isinstance(value, str) and 0 < len(value.strip()) <= 256 for value in required)
            and request.authorization_id.startswith("FAKE_TEST_ONLY:")
            and type(request.record_full_evidence) is bool
            and type(request.max_duration_s) is int
            and 1 <= request.max_duration_s <= 120
        )

    @staticmethod
    def _valid_prediction_only_goal(request: Any) -> bool:
        required = (
            (request.task, 256),
            (request.model_profile, 256),
            (request.checkpoint_id, 1024),
            (request.authorization_id, 256),
        )
        return (
            all(
                isinstance(value, str) and 0 < len(value.strip()) <= maximum
                for value, maximum in required
            )
            and request.task.strip()
            == "Pick up the black rod and place it on the green desk mat."
            and request.authorization_id.startswith("PREDICTION_ONLY:")
            and type(request.record_full_evidence) is bool
            and request.record_full_evidence is False
            and type(request.max_duration_s) is int
            and 1 <= request.max_duration_s <= 120
        )

    def _on_state(self, message: Any) -> None:
        with self._state_lock:
            self._latest_state = message
            self._latest_state_received_ns = time.monotonic_ns()

    def _on_target_preview(self, message: Any) -> None:
        """Count only fresh, bounded predictions matching the active goal."""
        if self.execution_mode != "prediction_only":
            return
        with self._active_lock:
            expected = self._active_preview_identity
            if not self._active or expected is None:
                return
            if (str(message.model_id), str(message.checkpoint_id)) != expected:
                self._preview_fault = "model_or_checkpoint_changed_during_action"
                return
            positions = [*message.joint_position_rad, message.gripper_nominal_m]
            now_ns = time.monotonic_ns()
            target_offset_ns = int(message.target_local_monotonic_ns) - now_ns
            expiry_ns = int(message.target_expiry_monotonic_ns)
            with self._state_lock:
                state = self._latest_state
                state_received_ns = self._latest_state_received_ns
            state_current = bool(
                state is not None
                and state.feedback_fresh
                and state_received_ns > 0
                and 0 <= now_ns-state.source_local_monotonic_ns <= 120_000_000
                and state_received_ns <= now_ns
                and now_ns-state_received_ns <= 120_000_000
                and state.sequence >= int(message.state_sequence)
                and str(message.source_observation_id).startswith(str(state.session_id) + ":")
            )
            target_within_current_limits = False
            if state_current and len(message.joint_position_rad) == 6:
                target_within_current_limits = all(
                    lower <= float(target) <= upper
                    and abs(float(target)-float(current)) <= math.radians(20.0) + 1e-12
                    for target, current, (lower, upper) in zip(
                        message.joint_position_rad, state.joint_position_rad,
                        self._joint_bounds_rad,
                    )
                ) and (
                    self._gripper_min_m <= float(message.gripper_nominal_m) <= self._gripper_max_m
                    and self._gripper_min_m <= float(state.gripper_nominal_m) <= self._gripper_max_m
                    and abs(float(message.gripper_nominal_m)-float(state.gripper_nominal_m))
                    <= self._gripper_step_m + 1e-12
                )
            valid = (
                message.prediction_only is True
                and message.emitted_to_hardware is False
                and message.hardware_control_allowed is False
                and message.local_clock_id == "wsl_perf_counter_ns"
                and message.action_semantics == "next_observed_state_proxy_nominal_units"
                and message.speed_percent == 5
                and int(message.state_age_ns) <= 120_000_000
                and int(message.observation_age_ns) <= 500_000_000
                and int(message.prediction_age_ns) <= 250_000_000
                and int(message.chunk_age_ns) <= 250_000_000
                and int(message.inference_latency_ns) <= 500_000_000
                and target_offset_ns <= 100_000_000
                and now_ns < expiry_ns
                and expiry_ns == int(message.target_local_monotonic_ns) + 100_000_000
                and len(message.joint_position_rad) == 6
                and all(math.isfinite(float(value)) for value in positions)
                and target_within_current_limits
                and int(message.target_local_monotonic_ns) > 0
                and int(message.state_sequence) > self._last_preview_state_sequence
                and bool(message.source_observation_id)
            )
            if not valid:
                self._preview_fault = "preview_contract_or_ordering_invalid"
                return
            observation_id = str(message.source_observation_id)
            if observation_id in self._seen_observation_ids:
                self._preview_fault = "duplicate_observation_preview"
                return
            self._seen_observation_ids.add(observation_id)
            self._last_preview_state_sequence = int(message.state_sequence)
            self._prediction_updates += 1
            self._last_valid_preview_monotonic_ns = time.monotonic_ns()

    def _cancel(self, _goal_handle: Any) -> CancelResponse:
        return (
            CancelResponse.ACCEPT
            if self.execution_mode in {"fake_test", "prediction_only"}
            else CancelResponse.REJECT
        )

    def _execute(self, goal_handle: Any) -> Any:
        if self.execution_mode == "prediction_only":
            return self._execute_prediction_only(goal_handle)
        request = goal_handle.request
        result = RunTask.Result()
        result.success = False
        result.outcome = "unverified"
        result.trial_id = str(uuid.uuid4())
        result.target_updates = 0
        result.matched_feedback_updates = 0
        started = time.monotonic()
        try:
            deadline = started + self.fake_result_delay_s
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.reason = "fake_test_cancelled_no_hardware_stop_to_verify"
                    return result
                feedback = RunTask.Feedback()
                feedback.phase = "FAKE_TEST_NO_HARDWARE"
                feedback.elapsed_s = max(0.0, time.monotonic() - started)
                feedback.target_updates = 0
                feedback.progress = min(
                    1.0,
                    feedback.elapsed_s / max(self.fake_result_delay_s, 1e-6),
                )
                goal_handle.publish_feedback(feedback)
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
            goal_handle.abort()
            result.reason = "fake_test_backend_has_no_hardware_execution"
            return result
        finally:
            with self._active_lock:
                self._active = False

    def _execute_prediction_only(self, goal_handle: Any) -> Any:
        from .preview_watchdog_core import preview_watchdog_fault

        request = goal_handle.request
        result = RunTask.Result()
        result.success = False
        result.outcome = "unverified"
        result.trial_id = str(uuid.uuid4())
        result.target_updates = 0
        result.matched_feedback_updates = 0
        result.prediction_updates = 0
        started = time.monotonic()
        deadline = started + int(request.max_duration_s)
        try:
            while time.monotonic() < deadline:
                with self._active_lock:
                    predictions = self._prediction_updates
                    fault = self._preview_fault
                    started_ns = self._action_started_monotonic_ns
                    last_preview_ns = self._last_valid_preview_monotonic_ns
                result.prediction_updates = predictions
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.reason = "prediction_only_cancelled_without_hardware_access"
                    return result
                if fault:
                    goal_handle.abort()
                    result.reason = f"prediction_only_preview_rejected:{fault}"
                    return result
                watchdog_fault = preview_watchdog_fault(
                    started_monotonic_ns=started_ns,
                    last_preview_monotonic_ns=last_preview_ns,
                    now_monotonic_ns=time.monotonic_ns(),
                    first_preview_timeout_ns=int(self.first_preview_timeout_s * 1e9),
                    preview_gap_timeout_ns=int(self.preview_gap_timeout_s * 1e9),
                )
                if watchdog_fault:
                    goal_handle.abort()
                    result.reason = f"prediction_only_watchdog:{watchdog_fault}"
                    return result
                elapsed = time.monotonic() - started
                feedback = RunTask.Feedback()
                feedback.phase = "PREDICTION_ONLY"
                feedback.elapsed_s = elapsed
                feedback.target_updates = 0
                feedback.prediction_updates = predictions
                feedback.matched_feedback_updates = 0
                feedback.progress = min(1.0, elapsed / int(request.max_duration_s))
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
            with self._active_lock:
                result.prediction_updates = self._prediction_updates
            goal_handle.abort()
            result.reason = (
                "prediction_only_window_complete_no_robot_command"
                if result.prediction_updates
                else "prediction_only_window_complete_without_matching_preview"
            )
            return result
        finally:
            with self._active_lock:
                self._active = False
                self._active_preview_identity = None
                self._action_started_monotonic_ns = 0
                self._last_valid_preview_monotonic_ns = None
                self._seen_observation_ids.clear()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = RunTaskActionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
