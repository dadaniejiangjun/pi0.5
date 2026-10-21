"""Convert standard ROS game-controller input to bounded PiPER previews."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from pi05_piper_interfaces.msg import StateSnapshot

from laptop.piper.control_profile import (
    MANUAL_GAMEPAD_GRIPPER_LIMITS as CONTINUOUS_GRIPPER_LIMITS,
    GRIPPER_NOMINAL_METERS_PER_RAW,
    TASK_JOINT_BOUNDS_DEG,
)

from .targets import JOINT_NAMES, integrate_target
from .cartesian_control import (
    CartesianContinuation,
    bounded_cartesian_step,
    legalize_feedback_joints_rad,
    pose_increment,
    preset_segment,
)
from .kinematics import (
    PiperKinematics,
    Pose,
    apply_pose_delta,
    resolve_default_piper_urdf,
)
from .preset_store import PosePresetStore
from .tracking_stall import TrackingStallGuard


class GamepadOperator(Node):
    def __init__(self):
        super().__init__("pi05_gamepad_operator")
        defaults = (
            ("axis_deadzone", 0.12),
            ("joint_step_deg", 5.0),
            ("joint_target_lead_deg", 5.0),
            ("gripper_step_raw", 12500),
            ("gripper_target_lead_raw", 12500),
            ("deadman_button", 10),
            ("gripper_close_button", 2),
            ("gripper_open_button", 3),
            ("feedback_max_age_ms", 120),
            ("joy_max_age_ms", 250),
            ("allow_synthetic", False),
            ("require_deadman", False),
            ("mapping_profile", "gamepad_piper"),
            ("gripper_control", "triggers"),
            ("joint_velocity_deg_s", 30.0),
            ("input_source", "xinput"),
            ("cartesian_coordinate_frame", "base"),
            ("cartesian_linear_speed_m_s", 0.30),
            ("tool_translation_xyz", [0.0, 0.0, 0.0]),
            ("urdf_path", ""),
            ("kinematics_backend", "builtin"),
            ("tracking_stall_enabled", False),
            ("pose_preset_path", ""),
            ("tracking_stall_window_s", 1.0),
            ("tracking_stall_error_deg", 0.5),
            ("tracking_stall_progress_deg", 0.1),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.stall_guard = TrackingStallGuard(
            window_s=float(self.get_parameter("tracking_stall_window_s").value),
            error_deg=float(self.get_parameter("tracking_stall_error_deg").value),
            progress_deg=float(self.get_parameter("tracking_stall_progress_deg").value))
        self.stall_notice = None
        self.tracking_stall_enabled = bool(self.get_parameter("tracking_stall_enabled").value)
        self.kinematics_backend = str(self.get_parameter("kinematics_backend").value)
        if self.kinematics_backend not in ("builtin", "trac_ik"):
            raise ValueError("kinematics_backend must be builtin or trac_ik")
        self.deadzone = float(self.get_parameter("axis_deadzone").value)
        self.step_deg = float(self.get_parameter("joint_step_deg").value)
        self.joint_target_lead_deg = float(
            self.get_parameter("joint_target_lead_deg").value
        )
        self.gripper_step = int(self.get_parameter("gripper_step_raw").value)
        self.gripper_target_lead = int(
            self.get_parameter("gripper_target_lead_raw").value
        )
        self.deadman = int(self.get_parameter("deadman_button").value)
        self.close_button = int(self.get_parameter("gripper_close_button").value)
        self.open_button = int(self.get_parameter("gripper_open_button").value)
        self.feedback_timeout_ns = int(self.get_parameter("feedback_max_age_ms").value) * 1_000_000
        self.joy_timeout_ns = int(self.get_parameter("joy_max_age_ms").value) * 1_000_000
        self.allow_synthetic = bool(self.get_parameter("allow_synthetic").value)
        self.require_deadman = bool(self.get_parameter("require_deadman").value)
        self.input_source = str(self.get_parameter("input_source").value)
        self.input_connected = False
        self.input_status_received_ns = 0
        self.mapping_profile = str(self.get_parameter("mapping_profile").value)
        self.gripper_control = str(self.get_parameter("gripper_control").value)
        self.joint_speed = float(self.get_parameter("joint_velocity_deg_s").value)
        if not math.isfinite(self.joint_speed):
            raise ValueError("joint_velocity_deg_s must be finite")
        self.speed_modes = (30.0, 60.0, 100.0)
        if self.joint_speed not in self.speed_modes:
            raise ValueError("joint_velocity_deg_s must be 30, 60, or 100 degrees/s")
        self.cartesian_frame = str(self.get_parameter("cartesian_coordinate_frame").value)
        if self.cartesian_frame not in ("base", "local"):
            raise ValueError("cartesian_coordinate_frame must be 'base' or 'local'")
        self.cartesian_linear_speed = float(
            self.get_parameter("cartesian_linear_speed_m_s").value
        )
        if not math.isfinite(self.cartesian_linear_speed) or not 0 < self.cartesian_linear_speed <= 0.30:
            raise ValueError("cartesian_linear_speed_m_s (low gear) must be within (0, 0.30]")
        self.cartesian_speed_modes = tuple(
            self.cartesian_linear_speed * multiplier for multiplier in (1.0, 1.5, 2.0))
        self.cartesian_linear_speed = self.cartesian_speed_modes[
            self.speed_modes.index(self.joint_speed)]
        self.tool_translation = tuple(
            float(value) for value in self.get_parameter("tool_translation_xyz").value
        )
        if (len(self.tool_translation) != 3
                or any(not math.isfinite(value) for value in self.tool_translation)):
            raise ValueError("tool_translation_xyz must contain three finite metres")
        self.urdf_path = str(self.get_parameter("urdf_path").value)
        self.kinematics = None
        self.effective_joint_bounds_deg = tuple(TASK_JOINT_BOUNDS_DEG)
        self.control_mode = "joint"
        self.last_speed_announce_ns = 0
        self.cartesian_goal: Pose | None = None
        self.cartesian_seed: tuple[float, ...] | None = None
        self.restore_goal: tuple[float, ...] | None = None
        self.restore_preset_id: str | None = None
        self.preset_index = -1
        preset_path = str(self.get_parameter("pose_preset_path").value)
        if not preset_path:
            try:
                preset_path = str(resolve_default_piper_urdf().parents[5]
                    / "gamepad_capture" / "data" / "presets" / "positions.json")
            except ValueError:
                preset_path = str(Path.cwd() / "gamepad_capture" / "data"
                                  / "presets" / "positions.json")
        try:
            self.preset_store = PosePresetStore(preset_path)
            if self.preset_store.presets:
                self.preset_index = len(self.preset_store.presets) - 1
            self.preset_store_error = None
        except ValueError as exc:
            self.preset_store = None
            self.preset_store_error = str(exc)
        self.lb_pressed = False
        self.previous_buttons: tuple[int, ...] = ()
        self.pending_mode_switch_at_ns = 0
        self.recording_paused = False
        self.resume_pending_run_id: str | None = None
        self.neutral_after_resume = False
        self.recording_resume_armed = False
        self.recording_run_id: str | None = None
        self.last_tick_ns = time.perf_counter_ns()
        if (not 0 <= self.deadzone < 1 or not 0 < self.step_deg <= 5
                or not 0 < self.joint_target_lead_deg <= 5):
            raise ValueError("invalid gamepad mapping limits")
        if (not 0 < self.gripper_step <= CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw
                or not 0 < self.gripper_target_lead <= CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw):
            raise ValueError("gripper step and feedback lead must fit the manual gamepad profile")

        self.state = None
        self.state_received_ns = 0
        self.joy = None
        self.joy_received_ns = 0
        self.target = None
        self.session_id = None
        self.last_event = ""
        self.cartesian_continuation = CartesianContinuation()

        self.target_pub = self.create_publisher(JointState, "/gamepad/target_preview", 10)
        self.gate_pub = self.create_publisher(Bool, "/gamepad/operator_enabled", 10)
        self.events_pub = self.create_publisher(String, "/gamepad/target_events", 50)
        self.create_subscription(
            StateSnapshot, "/piper/state_snapshot", self.on_state, qos_profile_sensor_data
        )
        self.create_subscription(Joy, "/joy", self.on_joy, qos_profile_sensor_data)
        self.create_subscription(String, "/gamepad/input_status", self.on_input_status, 50)
        self.create_subscription(String, "/gamepad/recording_lifecycle",
                                 self.on_recording_lifecycle, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.publish_gate(False)
        self.get_logger().info(
            "gamepad preview ready in joint mode; START toggles Cartesian preview, "
            "LB selects 30/60/100 deg/s and Cartesian 0.30/0.45/0.60 m/s (default configuration); "
            "RB remains reserved; this node never sends hardware commands"
        )
        if self.preset_store_error:
            self.get_logger().error(f"pose presets disabled: {self.preset_store_error}")

    def on_input_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            connected = (status.get("schema") == "piper_gamepad_input_status_v1"
                         and status.get("connected") is True)
        except (ValueError, AttributeError, TypeError):
            connected = False
        self.input_connected = connected
        self.input_status_received_ns = time.perf_counter_ns()
        if not connected:
            self.target = None
            self.restore_goal = None
            self.cartesian_goal = None
            self.recording_resume_armed = False
            self.lb_pressed = False
            self.publish_gate(False)

    def _load_kinematics(self) -> PiperKinematics:
        if self.kinematics is not None:
            return self.kinematics
        path = Path(self.urdf_path).expanduser() if self.urdf_path else resolve_default_piper_urdf()
        if self.kinematics_backend == "trac_ik":
            from .trac_ik_backend import TracIKPiperKinematics
            model = TracIKPiperKinematics(path, self.tool_translation)
        else:
            model = PiperKinematics(path, self.tool_translation)
        model_limits_deg = tuple(
            (math.degrees(low), math.degrees(high))
            for low, high in model.urdf_limits_rad
        )
        shared = tuple(
            (max(task_low, urdf_low), min(task_high, urdf_high))
            for (task_low, task_high), (urdf_low, urdf_high)
            in zip(TASK_JOINT_BOUNDS_DEG, model_limits_deg)
        )
        if any(low >= high for low, high in shared):
            raise ValueError("PiPER URDF and shared task joint limits do not overlap")
        self.kinematics = model
        self.effective_joint_bounds_deg = shared
        return model

    @staticmethod
    def _feedback_joints(state) -> tuple[float, ...]:
        values = tuple(float(value) for value in state.joint_position_rad)
        if len(values) != 6 or any(not math.isfinite(value) for value in values):
            raise ValueError("six finite joint feedback values are required")
        return values

    def _reset_cartesian_anchor(self, joints_rad: tuple[float, ...]) -> None:
        self.cartesian_continuation.reset()
        if self.control_mode == "cartesian":
            model = self._load_kinematics()
            if hasattr(model, 'previous_solution'):
                model.previous_solution = None
            seed = legalize_feedback_joints_rad(
                joints_rad, joint_bounds_deg=self.effective_joint_bounds_deg
            )
            # Anchor pose uses the unmodified measurement; only the numerical
            # solver seed is snapped into strict command bounds.
            self.cartesian_goal = model.forward(joints_rad)
            self.cartesian_seed = seed
        else:
            self.cartesian_goal = None
            self.cartesian_seed = None

    def _motion_neutral(self, axes=None, buttons=None) -> bool:
        joy = self.joy
        axes = tuple(joy.axes) if axes is None and joy is not None else tuple(axes or ())
        buttons = (tuple(joy.buttons) if buttons is None and joy is not None
                   else tuple(buttons or ()))
        return (
            len(axes) >= 6 and len(buttons) >= 15
            and all(abs(float(value)) <= self.deadzone for value in axes[:6])
            and not any(buttons[11:15])
        )

    def _button_edge(self, index: int) -> bool:
        return (len(self.joy.buttons) > index and bool(self.joy.buttons[index])
                and (len(self.previous_buttons) <= index
                     or not self.previous_buttons[index]))

    def on_recording_lifecycle(self, message: String) -> None:
        try:
            event = json.loads(message.data)
            if (not isinstance(event, dict)
                    or event.get("schema") != "piper_gamepad_recording_lifecycle_v1"):
                raise ValueError("unsupported recording lifecycle schema")
            name = event.get("event")
            run_id = event.get("run_id")
            if not isinstance(run_id, str) or not run_id or len(run_id) > 128:
                raise ValueError("recording lifecycle run_id is invalid")
            capture_ns = event.get(
                "capture_end_monotonic_ns", event.get("capture_monotonic_ns")
            )
            if type(capture_ns) is not int or capture_ns <= 0:
                raise ValueError("recording lifecycle timestamp is invalid")
            if name == "CAPTURE_STOP_REQUESTED":
                if event.get("target_generation_action") != "REVOKE_AND_HOLD":
                    raise ValueError("capture stop must request REVOKE_AND_HOLD")
                if self.recording_run_id not in (None, run_id):
                    raise ValueError("capture stop run_id does not match active run")
                self.recording_run_id = run_id
                self.recording_paused = True
                self.resume_pending_run_id = None
                self.neutral_after_resume = False
                self.recording_resume_armed = False
                self.pending_mode_switch_at_ns = 0
                self.target = None
                self.restore_goal = None
                self.cartesian_goal = None
                self.cartesian_seed = None
                self.publish_gate(False)
                self.emit("RECORDING_PAUSED_SOFTWARE_ONLY", lifecycle_event=name,
                          run_id=run_id, recording_state="PAUSED",
                          reason=str(event.get("reason", "")),
                          target_generation_action="REVOKE_AND_HOLD",
                          physical_stop_verified=False)
            elif name == "CAPTURE_START":
                if self.recording_run_id == run_id:
                    raise ValueError("CAPTURE_START must use a new run_id")
                self.recording_run_id = run_id
                self.recording_paused = True
                self.resume_pending_run_id = run_id
                self.neutral_after_resume = False
                self.recording_resume_armed = False
                self.pending_mode_switch_at_ns = 0
                self.target = None
                self.restore_goal = None
                self.cartesian_goal = None
                self.cartesian_seed = None
                self.publish_gate(False)
                self.emit("RECORDING_START_WAITING_FOR_NEUTRAL",
                          lifecycle_event=name, run_id=run_id,
                          recording_state="PAUSED", physical_stop_verified=False)
            else:
                raise ValueError("unsupported recording lifecycle event")
        except (ValueError, TypeError, AttributeError) as exc:
            self.emit("REJECTED", reason="recording_lifecycle_invalid",
                      detail=str(exc))

    def publish_gate(self, enabled: bool) -> None:
        message = Bool()
        message.data = bool(enabled)
        self.gate_pub.publish(message)

    def emit(self, status: str, **fields) -> None:
        payload = {
            "schema": "piper_gamepad_target_event_v1",
            "status": status,
            "local_monotonic_ns": time.perf_counter_ns(),
            "source_session_id": self.session_id,
            "can_tx": False,
            **fields,
        }
        message = String()
        message.data = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        self.events_pub.publish(message)

    def on_state(self, message: StateSnapshot) -> None:
        if self.session_id is not None and message.session_id != self.session_id:
            self.stall_guard.reset()
            self.target = None
            self.restore_goal = None
            self.cartesian_goal = None
            self.cartesian_seed = None
            self.publish_gate(False)
            self.emit("SESSION_CHANGED", new_session_id=message.session_id)
        self.session_id = message.session_id
        self.state = message
        self.state_received_ns = time.perf_counter_ns()

    def on_joy(self, message: Joy) -> None:
        # A/B/X/Y are unassigned during demonstration capture. Mask locally so
        # legacy gripper-button mappings cannot turn them into motion either.
        message.buttons = [0 if index < 4 else value
                           for index, value in enumerate(message.buttons)]
        self.joy = message
        self.joy_received_ns = time.perf_counter_ns()
        lb = len(message.buttons) > 9 and bool(message.buttons[9])
        if lb and not self.lb_pressed:
            index = self.speed_modes.index(self.joint_speed)
            self.joint_speed = self.speed_modes[(index + 1) % len(self.speed_modes)]
            self.cartesian_linear_speed = self.cartesian_speed_modes[
                self.speed_modes.index(self.joint_speed)]
            self.emit("TARGET_RATE_PREVIEW", selected_joint_rate_deg_s=self.joint_speed,
                      selected_cartesian_linear_speed_m_s=self.cartesian_linear_speed,
                      controller_speed_percent=(15,30,50)[self.speed_modes.index(self.joint_speed)],
                      physical_joint_speed_verified=False)
            self.get_logger().info(
                f"速度档位 / Target rate: {self.joint_speed:g} deg/s, "
                f"Cartesian {self.cartesian_linear_speed:g} m/s; "
                f"controller {(15,30,50)[self.speed_modes.index(self.joint_speed)]}% (requested).")
        self.lb_pressed = lb
        if self._button_edge(6):
            # START also starts a capture. Defer mode switching briefly so the
            # lifecycle event can claim that same button press first.
            self.pending_mode_switch_at_ns = time.perf_counter_ns() + 400_000_000
        self.previous_buttons = tuple(message.buttons)
        if self.resume_pending_run_id is not None:
            self.neutral_after_resume = self._motion_neutral(message.axes, message.buttons)
        if self.require_deadman and (len(message.buttons) <= self.deadman or not message.buttons[self.deadman]):
            self.publish_gate(False)
            self.target = None

    def _save_preset(self) -> None:
        if self.recording_paused:
            self.emit("REJECTED", reason="recording_operator_paused",
                      action="save_preset")
            return
        if self.preset_store is None:
            self.emit("REJECTED", reason="pose_preset_store_unavailable",
                      detail=self.preset_store_error)
            return
        rejection = self._state_rejection(time.perf_counter_ns())
        if rejection is not None:
            self.emit("REJECTED", reason="preset_save_feedback_rejected",
                      detail=rejection)
            return
        state = self.state
        try:
            preset = self.preset_store.save_feedback(
                joints_rad=self._feedback_joints(state),
                gripper_raw=int(state.gripper_raw),
                session_id=str(state.session_id),
                state_sequence=int(state.sequence),
            )
            self.preset_index = len(self.preset_store.presets) - 1
            self.emit("PRESET_SAVED", preset_id=preset["preset_id"],
                      source_session_id=preset["source_session_id"],
                      source_state_sequence=preset["source_state_sequence"],
                      joint_position_rad=preset["joint_position_rad"],
                      gripper_raw=preset["gripper_raw"],
                      coordinate_frame=preset["coordinate_frame"])
        except (OSError, ValueError, TypeError) as exc:
            self.emit("REJECTED", reason="preset_save_failed", detail=str(exc))

    def _cycle_preset(self) -> None:
        if self.recording_paused:
            self.emit("REJECTED", reason="recording_operator_paused",
                      action="cycle_preset")
            return
        if self.preset_store is None:
            self.emit("REJECTED", reason="pose_preset_store_unavailable",
                      detail=self.preset_store_error)
            return
        try:
            self.preset_index = self.preset_store.cycle(self.preset_index)
            preset = self.preset_store.selected(self.preset_index)
            self.emit("PRESET_SELECTED", preset_id=preset["preset_id"],
                      selected_index=self.preset_index,
                      preset_count=len(self.preset_store.presets))
        except ValueError as exc:
            self.emit("REJECTED", reason="preset_selection_failed", detail=str(exc))

    def _begin_preset_restore(self) -> None:
        if self.recording_paused:
            self.emit("REJECTED", reason="recording_operator_paused",
                      action="restore_preset")
            return
        if not self._motion_neutral():
            self.emit("REJECTED", reason="preset_restore_requires_neutral_motion_input")
            return
        if self.preset_store is None:
            self.emit("REJECTED", reason="pose_preset_store_unavailable",
                      detail=self.preset_store_error)
            return
        rejection = self._state_rejection(time.perf_counter_ns())
        if rejection is not None:
            self.emit("REJECTED", reason="preset_restore_feedback_rejected",
                      detail=rejection)
            return
        try:
            preset = self.preset_store.selected(self.preset_index)
            saved_goal = tuple(float(value) for value in preset["joint_position_rad"])
            goal = legalize_feedback_joints_rad(
                saved_goal, joint_bounds_deg=self.effective_joint_bounds_deg
            )
            self.restore_goal = goal + (float(preset["gripper_raw"]),)
            self.restore_preset_id = str(preset["preset_id"])
            self.target = None
            self.emit("PRESET_RESTORE_STARTED", preset_id=self.restore_preset_id,
                      target_joint_position_rad=list(goal),
                      saved_feedback_joint_rad=list(saved_goal),
                      target_gripper_raw=int(preset["gripper_raw"]),
                      segmented_feedback_bounded=True,
                      execution_by_this_node=False)
        except (ValueError, TypeError, KeyError) as exc:
            self.emit("REJECTED", reason="preset_restore_failed", detail=str(exc))

    def _preset_reached(self, current_joints: tuple[float, ...],
                        current_gripper_raw: int) -> bool:
        if self.restore_goal is None:
            return False
        joint_tolerance = math.radians(0.25)
        return (
            all(abs(current - target) <= joint_tolerance
                for current, target in zip(current_joints, self.restore_goal[:6]))
            and abs(current_gripper_raw - int(self.restore_goal[6])) <= 100
        )

    def _switch_mode(self) -> None:
        rejection = self._state_rejection(time.perf_counter_ns())
        if rejection is not None:
            self.emit("REJECTED", reason="mode_switch_feedback_rejected",
                      detail=rejection)
            return
        try:
            if self.control_mode == "joint":
                model = self._load_kinematics()
                joints = self._feedback_joints(self.state)
                seed = legalize_feedback_joints_rad(
                    joints, joint_bounds_deg=self.effective_joint_bounds_deg
                )
                self.cartesian_goal = model.forward(joints)
                self.cartesian_seed = seed
                self.control_mode = "cartesian"
            else:
                self.control_mode = "joint"
                self.cartesian_goal = None
                self.cartesian_seed = None
            self.restore_goal = None
            self.target = None
            self.publish_gate(False)
            self.emit("CONTROL_MODE", mode=self.control_mode,
                      coordinate_frame=self.cartesian_frame if self.control_mode == "cartesian" else None,
                      anchor_feedback_joint_rad=list(self.state.joint_position_rad),
                      can_tx=False)
            self.get_logger().info(
                f"控制模式 / Mode: {self.control_mode}; "
                f"frame={self.cartesian_frame}; IK={self.kinematics_backend}")
        except (ValueError, OSError) as exc:
            self.emit("REJECTED", reason="cartesian_model_unavailable_or_invalid",
                      detail=str(exc))

    def _state_rejection(self, now_ns: int) -> str | None:
        state = self.state
        if state is None:
            return "waiting_for_feedback"
        if now_ns - self.state_received_ns > self.feedback_timeout_ns:
            return "feedback_arrival_stale"
        if not state.feedback_fresh:
            return "feedback_not_fresh"
        if state.fake and not self.allow_synthetic:
            return "synthetic_feedback_disabled"
        if not state.hardware_connected and not (state.fake and self.allow_synthetic):
            return "robot_not_connected"
        if state.ctrl_mode_raw != 1 or state.arm_status_raw or state.error_code:
            return "robot_mode_or_status_rejected"
        if not all(state.motor_enabled):
            return "motor_enable_incomplete"
        source_age = time.perf_counter_ns() - state.source_local_monotonic_ns
        uncertainty = state.source_clock_mapping_uncertainty_ns
        if source_age < -uncertainty or source_age + uncertainty > self.feedback_timeout_ns:
            return "source_feedback_clock_stale_or_uncertain"
        return None

    def tick(self) -> None:
        now_ns = time.perf_counter_ns()
        if now_ns - self.last_speed_announce_ns >= 1_000_000_000:
            self.last_speed_announce_ns = now_ns
            self.emit('TARGET_RATE_PREVIEW', selected_joint_rate_deg_s=self.joint_speed,
                      controller_speed_percent=(15,30,50)[self.speed_modes.index(self.joint_speed)],
                      selected_cartesian_linear_speed_m_s=self.cartesian_linear_speed,
                      physical_joint_speed_verified=False)
        dt = max(1e-6, min(0.1, (now_ns-self.last_tick_ns)/1e9))
        self.last_tick_ns = now_ns

        if self.resume_pending_run_id is not None:
            ready = (
                self.neutral_after_resume
                and self.joy is not None
                and now_ns - self.joy_received_ns <= self.joy_timeout_ns
                and (self.input_source != "xinput" or (
                    self.input_connected
                    and now_ns - self.input_status_received_ns <= self.joy_timeout_ns
                ))
                and self._state_rejection(now_ns) is None
            )
            if ready:
                try:
                    self._reset_cartesian_anchor(self._feedback_joints(self.state))
                    run_id = self.resume_pending_run_id
                    self.recording_paused = False
                    self.resume_pending_run_id = None
                    self.recording_resume_armed = True
                    self.target = None
                    self.restore_goal = None
                    self.publish_gate(True)
                    self.emit("RECORDING_RESUMED_SOFTWARE_ONLY", run_id=run_id,
                              recording_state="RESUMED", neutral_input_observed=True,
                              old_target_replayed=False,
                              physical_stop_verified=False)
                except (ValueError, OSError) as exc:
                    self.emit("REJECTED", reason="recording_resume_reanchor_failed",
                              detail=str(exc), physical_stop_verified=False)
            else:
                self.publish_gate(False)
                self.target = None
                self.restore_goal = None
                return
        if self.recording_paused:
            self.stall_guard.cancel_pending()
            self.publish_gate(False)
            self.target = None
            self.restore_goal = None
            return

        if (self.pending_mode_switch_at_ns
                and now_ns >= self.pending_mode_switch_at_ns
                and len(self.joy.buttons) > 6 and not self.joy.buttons[6]):
            self.pending_mode_switch_at_ns = 0
            self._switch_mode()

        rejection = self._state_rejection(now_ns)
        if self.input_source == "xinput" and (
            not self.input_connected
            or now_ns - self.input_status_received_ns > self.joy_timeout_ns
        ):
            rejection = "input_disconnected_or_status_stale"
        if rejection is not None:
            self.stall_guard.cancel_pending()
            self.publish_gate(False)
            self.target = None
            if rejection != self.last_event:
                disabled = ([i+1 for i, enabled in enumerate(self.state.motor_enabled)
                             if not enabled] if self.state is not None else [])
                self.emit("REJECTED", reason=rejection, disabled_joints=disabled,
                          automatic_fault_clear=False, automatic_enable=False)
                if disabled:
                    self.get_logger().info(
                        f"Motors disabled at joints {disabled}; manual fault recovery required, "
                        "no automatic enable or target replay.")
                self.last_event = rejection
            return
        if self.joy is None or now_ns - self.joy_received_ns > self.joy_timeout_ns:
            self.stall_guard.cancel_pending()
            self.publish_gate(False)
            self.target = None
            if self.last_event != "joy_stale_or_missing":
                self.emit("REJECTED", reason="joy_stale_or_missing")
                self.last_event = "joy_stale_or_missing"
            return
        if len(self.joy.buttons) <= max(self.deadman, self.close_button, self.open_button, 14):
            self.publish_gate(False)
            self.target = None
            if self.last_event != "unsupported_joy_layout":
                self.emit("REJECTED", reason="unsupported_joy_layout",
                          button_count=len(self.joy.buttons), axis_count=len(self.joy.axes))
                self.last_event = "unsupported_joy_layout"
            return
        if self.require_deadman and not self.joy.buttons[self.deadman]:
            self.publish_gate(False)
            self.target = None
            self.last_event = "deadman_released"
            return
        if len(self.joy.axes) < 4:
            self.publish_gate(False)
            self.target = None
            self.emit("REJECTED", reason="four_analog_axes_required",
                      axis_count=len(self.joy.axes))
            return

        state = self.state
        current_joints = self._feedback_joints(state)
        motion_neutral = self._motion_neutral()
        if self.restore_goal is not None and not motion_neutral:
            self.emit("PRESET_RESTORE_CANCELLED_BY_INPUT",
                      preset_id=self.restore_preset_id,
                      feedback_joint_rad=list(current_joints),
                      physical_stop_verified=False)
            self.restore_goal = None
            self.restore_preset_id = None

        if self.restore_goal is not None:
            try:
                if self._preset_reached(current_joints, int(state.gripper_raw)):
                    preset_id = self.restore_preset_id
                    self.restore_goal = None
                    self.restore_preset_id = None
                    self.target = None
                    self._reset_cartesian_anchor(current_joints)
                    self.publish_gate(False)
                    self.emit("PRESET_RESTORE_COMPLETED_SOFTWARE_TARGET",
                              preset_id=preset_id,
                              feedback_joint_rad=list(current_joints),
                              feedback_gripper_raw=int(state.gripper_raw),
                              physical_motion_verified=False)
                    return
                target = preset_segment(
                    current_joints_rad=current_joints,
                    target_joints_rad=tuple(self.restore_goal[:6]),
                    current_gripper_raw=int(state.gripper_raw),
                    target_gripper_raw=int(self.restore_goal[6]),
                    joint_bounds_deg=self.effective_joint_bounds_deg,
                    joint_speed_deg_s=self.joint_speed,
                    dt_s=dt,
                    joint_step_limit_deg=self.step_deg,
                    joint_target_lead_deg=self.joint_target_lead_deg,
                    gripper_bounds_raw=(CONTINUOUS_GRIPPER_LIMITS.minimum_raw,
                                        CONTINUOUS_GRIPPER_LIMITS.maximum_raw),
                    gripper_step_raw=min(
                        CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
                        max(1, round(self.gripper_step * dt / 0.1)),
                    ),
                    gripper_target_lead_raw=self.gripper_target_lead,
                )
                action = "preset_restore_segment"
                extra = {"preset_id": self.restore_preset_id,
                         "segmented_feedback_bounded": True}
            except (ValueError, TypeError, OverflowError) as exc:
                self.target = None
                self.restore_goal = None
                self.restore_preset_id = None
                self.publish_gate(False)
                self.emit("REJECTED", reason="preset_restore_segment_rejected",
                          detail=str(exc), physical_stop_verified=False)
                return
        elif not self.require_deadman and motion_neutral:
            self.stall_guard.cancel_pending()
            self.target = None
            self.restore_goal = None
            if self.control_mode == "cartesian":
                try:
                    self._reset_cartesian_anchor(current_joints)
                except (ValueError, OSError) as exc:
                    self.cartesian_goal = None
                    self.cartesian_seed = None
                    self.emit("REJECTED", reason="cartesian_reanchor_failed",
                              detail=str(exc))
            self.publish_gate(self.recording_resume_armed)
            self.last_event = "neutral_input"
            return

        target_mode_fields = {"control_mode": self.control_mode}
        if self.restore_goal is not None:
            pass
        elif self.control_mode == "cartesian":
            ik_diagnostics = {}
            solve_started_ns = time.perf_counter_ns()
            try:
                model = self._load_kinematics()
                if self.cartesian_goal is None or self.cartesian_seed is None:
                    self.cartesian_continuation.reset()
                    self.cartesian_goal = model.forward(current_joints)
                    self.cartesian_seed = legalize_feedback_joints_rad(
                        current_joints,
                        joint_bounds_deg=self.effective_joint_bounds_deg,
                    )
                translation, rotation = pose_increment(
                    axes=tuple(self.joy.axes), buttons=tuple(self.joy.buttons),
                    axis_deadzone=self.deadzone,
                    linear_speed_m_s=self.cartesian_linear_speed,
                    angular_speed_deg_s=self.joint_speed, dt_s=dt,
                )
                cartesian_active = any(abs(value) > 0.0
                                       for value in (*translation, *rotation))
                gripper_target = integrate_target(
                    current_joints_rad=current_joints,
                    current_gripper_raw=int(state.gripper_raw),
                    previous_target=None,
                    axes=(0.0, 0.0, 0.0, 0.0,
                          float(self.joy.axes[4]), float(self.joy.axes[5])),
                    buttons=tuple(
                        0 if 11 <= index <= 14 else int(value)
                        for index, value in enumerate(self.joy.buttons)
                    ),
                    joint_bounds_deg=self.effective_joint_bounds_deg,
                    gripper_bounds_raw=(CONTINUOUS_GRIPPER_LIMITS.minimum_raw,
                                        CONTINUOUS_GRIPPER_LIMITS.maximum_raw),
                    axis_deadzone=self.deadzone,
                    joint_step_deg=min(self.step_deg, self.joint_speed * dt),
                    joint_target_lead_deg=self.joint_target_lead_deg,
                    gripper_step_raw=min(
                        CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
                        max(1, round(self.gripper_step * dt / 0.1)),
                    ),
                    gripper_target_lead_raw=self.gripper_target_lead,
                    deadman_button=self.deadman,
                    close_button=self.close_button,
                    open_button=self.open_button,
                    require_deadman=self.require_deadman,
                    mapping_profile=self.mapping_profile,
                    gripper_control=self.gripper_control,
                )[6]
                if cartesian_active:
                    if self.target is None:
                        self.cartesian_continuation.reset()
                    proposed, candidate, step_scale = self.cartesian_continuation.step(
                        model=model, current_joints_rad=current_joints,
                        translation_m=translation, rotation_rad=rotation,
                        frame=self.cartesian_frame,
                        joint_bounds_deg=self.effective_joint_bounds_deg,
                        joint_step_deg=min(self.step_deg, self.joint_speed * dt),
                        joint_target_lead_deg=self.joint_target_lead_deg,
                        diagnostics=ik_diagnostics,
                    )
                    self.cartesian_goal = proposed
                    self.cartesian_seed = candidate
                    joints_target = candidate
                    target_mode_fields.update(
                        cartesian_frame=self.cartesian_frame,
                        cartesian_step_scale=step_scale,
                        kinematics_solver=getattr(model, "last_solver", "builtin"),
                        target_tcp_position_m=list(proposed.position_m),
                        target_tcp_rotation_rows=[list(row) for row in proposed.rotation],
                        ik_diagnostics=ik_diagnostics,
                        cartesian_requested_translation_m=list(translation),
                        cartesian_requested_rotation_rad=list(rotation),
                        control_dt_s=dt,
                        joy_received_monotonic_ns=self.joy_received_ns,
                        feedback_received_monotonic_ns=self.state_received_ns,
                    )
                else:
                    self.cartesian_goal = model.forward(current_joints)
                    # Feedback can sit a few encoder counts outside the strict
                    # command envelope. Keep the measured pose for the anchor,
                    # but legalize the actual hold target sent downstream.
                    self.cartesian_seed = legalize_feedback_joints_rad(
                        current_joints,
                        joint_bounds_deg=self.effective_joint_bounds_deg,
                    )
                    joints_target = self.cartesian_seed
                target = tuple(joints_target) + (float(gripper_target),)
                action = "cartesian_feedback_bounded_target"
                extra = {}
            except (ValueError, TypeError, OverflowError, OSError) as exc:
                self.cartesian_continuation.reset()
                self.target = None
                self.publish_gate(False)
                notice = "cartesian_target_rejected:" + str(exc)
                if self.last_event != notice:
                    self.get_logger().info(f"末端目标未发送 / Cartesian target rejected: {exc}")
                self.last_event = notice
                self.emit("REJECTED", reason="cartesian_target_rejected",
                          detail=str(exc), can_tx=False,
                          feedback_joint_rad=list(current_joints),
                          feedback_sequence=int(state.sequence),
                          control_mode=self.control_mode,
                          cartesian_frame=self.cartesian_frame,
                          input_axes=list(self.joy.axes), input_buttons=list(self.joy.buttons),
                          ik_diagnostics=ik_diagnostics,
                          solve_elapsed_ms=(time.perf_counter_ns()-solve_started_ns)/1e6)
                return
        else:
            try:
                target = integrate_target(
                    current_joints_rad=current_joints,
                    current_gripper_raw=int(state.gripper_raw),
                    previous_target=self.target,
                    axes=tuple(self.joy.axes),
                    buttons=tuple(self.joy.buttons),
                    joint_bounds_deg=self.effective_joint_bounds_deg,
                    gripper_bounds_raw=(CONTINUOUS_GRIPPER_LIMITS.minimum_raw,
                                        CONTINUOUS_GRIPPER_LIMITS.maximum_raw),
                    axis_deadzone=self.deadzone,
                    joint_step_deg=min(self.step_deg, self.joint_speed*dt),
                    joint_target_lead_deg=self.joint_target_lead_deg,
                    gripper_step_raw=min(CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
                        max(1, round(self.gripper_step*dt/0.1))),
                    gripper_target_lead_raw=self.gripper_target_lead,
                    deadman_button=self.deadman,
                    close_button=self.close_button,
                    open_button=self.open_button,
                    require_deadman=self.require_deadman,
                    mapping_profile=self.mapping_profile,
                    gripper_control=self.gripper_control,
                )
                action = "feedback_bounded_absolute_position_target"
                extra = {}
            except (ValueError, TypeError, OverflowError) as exc:
                self.publish_gate(False)
                self.target = None
                self.emit("REJECTED", reason=str(exc))
                return

        tracking = self.stall_guard.inspect(
            current_joints, target[:6], now_s=now_ns/1e9,
            source=(state.session_id, state.sequence)) if self.tracking_stall_enabled else {
                "allowed": True, "recovered_joints": [], "blocked_joints": [],
                "new_stalled_joints": []}
        if tracking["recovered_joints"]:
            self.emit("TRACKING_RETREAT_OBSERVED", joints=tracking["recovered_joints"],
                      collision_verified=False, physical_stop_verified=False)
        if not tracking["allowed"]:
            # Drop the whole pose rather than clipping IK joints independently.
            # A false gate asks the existing executor for a measured-pose hold.
            self.target = None
            self.restore_goal = None
            self.cartesian_goal = None
            self.cartesian_seed = None
            self.recording_resume_armed = False
            self.publish_gate(False)
            notice = tuple(tracking["blocked_joints"])
            if notice != self.stall_notice or tracking["new_stalled_joints"]:
                self.emit("TRACKING_STALL_RETREAT_REQUIRED", joints=list(notice),
                          reason="persistent_target_without_measured_progress",
                          reverse_input_allowed=True, collision_verified=False,
                          physical_stop_verified=False)
                self.get_logger().info(
                    f"Tracking stalled at joints {list(notice)}; release stick or reverse to retreat. "
                    "Disabled/faulted motors require separate recovery.")
            self.stall_notice = notice
            return
        self.stall_notice = None
        self.target = target
        self.recording_resume_armed = False
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = list(JOINT_NAMES)
        output.position = list(target[:6]) + [
            target[6] * GRIPPER_NOMINAL_METERS_PER_RAW
        ]
        self.target_pub.publish(output)
        self.publish_gate(True)
        self.emit(
            "TARGET_PREVIEW",
            input_axes=list(self.joy.axes),
            input_buttons=list(self.joy.buttons),
            feedback_sequence=int(state.sequence),
            feedback_joint_rad=list(state.joint_position_rad),
            feedback_gripper_raw=int(state.gripper_raw),
            target_joint_rad=list(target[:6]),
            target_gripper_raw=int(target[6]),
            target_gripper_nominal_m=output.position[6],
            action_semantics=action,
            joint_target_feedback_delta_deg=[
                round(math.degrees(target_value - feedback_value), 4)
                for target_value, feedback_value in zip(target[:6], state.joint_position_rad)
            ],
            gripper_target_feedback_delta_raw=int(target[6]) - int(state.gripper_raw),
            execution_by_this_node=False,
            **target_mode_fields,
            **extra,
        )
        self.last_event = "target_preview"


def main(args=None) -> int:
    # Publish the deadman-off state while the context is still valid on Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GamepadOperator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_gate(False)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
