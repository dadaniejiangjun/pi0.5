"""Persistent PiPER receive/control session shared by collection and policy code.

One session owns one configured SafetyBridge. Call poll regularly; this API is
synchronous and does not create a hidden receiver or movement loop.
"""
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import struct
import threading
import time

from laptop.control.contracts import RobotStateSnapshot
from laptop.control.feedback_acceptance import (
    MANUAL_FEEDBACK_LIMITS,
)
from laptop.piper.unit_conversion import gripper_raw_to_nominal_m, joint_raw_to_nominal_rad
from laptop.robot_client.feedback import decode_feedback_frame
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.safety.task_scope import TaskControlSnapshot, TaskScopedPositionAuthorization
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


class PiperSession:
    """Connect, receive, inspect, transmit typed commands, and close one CAN owner.

    Supply an explicitly configured bridge for control. The default bridge is
    receive-only. Units reported here are nominal SDK units, not calibration.
    """

    def __init__(self, log_path, *, bridge=None, log_rx_frames=True):
        if type(log_rx_frames) is not bool:
            raise TypeError("log_rx_frames must be a bool")
        self.bridge = bridge if bridge is not None else SafetyBridge(
            mode=RuntimeMode.POLICY_REJECT_ONLY, transport=GsUsbPiperCanTransport(),
            limits=UNVERIFIED_ANALYSIS_LIMITS)
        self.log_path = Path(log_path)
        self.log_rx_frames = log_rx_frames
        self._log = None
        self._thread = None
        self._closed = False
        self._latest = {}
        self._sequence = 0
        self._task_snapshot_sequence = 0

    def _owner(self):
        if self._log is None or self._closed:
            raise RuntimeError("session is not connected")
        if threading.get_ident() != self._thread:
            raise RuntimeError("poll and send must use the session owner thread")

    def _record(self, event, **fields):
        self._sequence += 1
        def serialize(value):
            if is_dataclass(value):
                return asdict(value)
            if isinstance(value, bytes):
                return value.hex()
            if hasattr(value, "value"):
                return value.value
            raise TypeError(f"unsupported audit value {type(value).__name__}")
        self._log.write(json.dumps(dict(sequence=self._sequence, event=event,
            monotonic_ns=time.perf_counter_ns(), utc=datetime.now(timezone.utc).isoformat(),
            **fields), default=serialize) + "\n")
        self._log.flush()

    def connect(self):
        if self._log is not None or self._closed:
            raise RuntimeError("session is single-use; create a new session after close")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("x", encoding="utf-8")
        self._thread = threading.get_ident()
        try:
            self._record("connecting", mode=self.bridge.mode)
            self.bridge.start_read_only()
            self._record("connected")
        except BaseException:
            self.close()
            raise
        return self

    def poll(self, duration_s=0.1):
        self._owner()
        if not 0 < duration_s <= 5:
            raise ValueError("poll duration must be within (0, 5] seconds")
        deadline = time.monotonic() + duration_s
        count = 0
        while time.monotonic() < deadline:
            frame = self.bridge.receive_frame(timeout_ms=min(50, max(1,
                int((deadline - time.monotonic()) * 1000))))
            if frame is None:
                continue
            if self.log_rx_frames:
                self._record("rx", frame=frame)
            count += 1
            if not frame.is_extended_id and not frame.is_error_frame and len(frame.data) == 8:
                self._latest[frame.arbitration_id] = (frame.received_monotonic_ns, frame.data)
        return count

    def record_state_snapshot(self, state):
        """Write one bounded-rate state record without dumping every raw RX frame."""
        self._owner()
        if not isinstance(state, dict):
            raise TypeError("state snapshot must be a dictionary")
        self._record("state_snapshot", state=state)

    def state(self, max_age_ms=120):
        self._owner()
        if not 0 < max_age_ms <= 5000:
            raise ValueError("state maximum age must be within (0, 5000] ms")
        now = time.perf_counter_ns()
        required = (0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8, *range(0x261, 0x267))
        missing = [hex(i) for i in required if i not in self._latest]
        stale = [hex(i) for i in required if i in self._latest and
                 not 0 <= now - self._latest[i][0] <= max_age_ms * 1_000_000]
        result = dict(missing_channels=missing, stale_channels=stale,
                      complete_fresh=not missing and not stale, units_hardware_verified=False,
                      bridge_state=self.bridge.state.value)
        status = self._latest.get(0x2A1)
        if status:
            result.update(decode_feedback_frame(0x2A1, status[1], status[0]).fields)
        if all(i in self._latest for i in (0x2A5, 0x2A6, 0x2A7)):
            raw = [v for i in (0x2A5, 0x2A6, 0x2A7)
                   for v in decode_feedback_frame(i, self._latest[i][1],
                                                   self._latest[i][0]).fields["joint_pair"]]
            stamps = [self._latest[i][0] for i in (0x2A5, 0x2A6, 0x2A7)]
            result.update(joint_raw_0p001deg=raw, joint_deg_nominal=[v / 1000 for v in raw],
                          joint_skew_ns=max(stamps) - min(stamps))
        if 0x2A8 in self._latest:
            stamp, data = self._latest[0x2A8]
            result["gripper"] = decode_feedback_frame(0x2A8, data, stamp).fields
        result["motor_enabled"] = [bool(self._latest[i][1][5] & 0x40)
            if i in self._latest else None for i in range(0x261, 0x267)]
        result["motor_status_codes"] = [self._latest[i][1][5]
            if i in self._latest else None for i in range(0x261, 0x267)]
        result["channel_timestamps_ns"] = {hex(i): self._latest[i][0]
                                             for i in required if i in self._latest}
        return result

    def task_control_snapshot(self) -> TaskControlSnapshot:
        """Build a task-ready typed snapshot from this session's live feedback."""
        self._owner()
        authorization = self.bridge.active_task_position_authorization
        if type(authorization) is not TaskScopedPositionAuthorization:
            raise PermissionError("task snapshots require an active scoped authorization")
        report = self.state(max_age_ms=120)
        if not report["complete_fresh"]:
            channels = report["missing_channels"] + report["stale_channels"]
            raise PermissionError("task feedback is incomplete or stale: " + ", ".join(channels))
        required = (0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8,
                    0x261, 0x262, 0x263, 0x264, 0x265, 0x266)
        timestamp_map = report["channel_timestamps_ns"]
        timestamps = tuple((can_id, timestamp_map[hex(can_id)]) for can_id in required)
        joint_raw = tuple(report["joint_raw_0p001deg"])
        if len(joint_raw) != 6:
            raise PermissionError("task feedback does not contain six joint positions")
        gripper_raw = report["gripper"]["position_raw"]
        gates = dict(authorization.site_gate_attestations)
        units_verified = gates["joint_state_units_verified"] and gates["gripper_units_verified"]
        self._task_snapshot_sequence += 1
        state = RobotStateSnapshot(
            sequence=self._task_snapshot_sequence,
            captured_monotonic_ns=min(stamp for _, stamp in timestamps),
            received_monotonic_ns=max(stamp for _, stamp in timestamps),
            joint_position_rad=tuple(joint_raw_to_nominal_rad(value) for value in joint_raw),
            gripper_position_m=gripper_raw_to_nominal_m(gripper_raw),
            status_ok=(report["arm_status_raw"] == 0 and report["error_code"] == 0),
            connected=bool(self.bridge.transport.connected),
            units_verified=units_verified,
        )
        motor_codes = tuple(report["motor_status_codes"])
        if any(value is None for value in motor_codes):
            raise PermissionError("task feedback is missing motor status bytes")
        return TaskControlSnapshot(
            robot_state=state,
            ctrl_mode_raw=report["ctrl_mode_raw"],
            arm_status_raw=report["arm_status_raw"],
            move_mode_raw=report["move_mode_raw"],
            teach_status_raw=report["teach_status_raw"],
            error_code=report["error_code"],
            gripper_status_raw=report["gripper"]["status_raw"],
            motor_enabled=tuple(report["motor_enabled"]),
            motor_status_codes=motor_codes,
            feedback_timestamps_ns=timestamps,
        )

    def _send(self, operation, payload, callback):
        self._owner()
        self._record("tx_requested", operation=operation, payload=payload)
        try:
            receipt = callback()
        except Exception as exc:
            self._record("tx_failed_no_retry", operation=operation,
                         error_type=type(exc).__name__, reason=str(exc),
                         delivery_diagnostics=getattr(exc, "delivery_diagnostics", None))
            raise
        self._record("tx_result", operation=operation, receipt=receipt)
        return receipt

    def send_position(self, *, snapshot, batch, authorization):
        """Six joint + gripper absolute targets; existing proposal/site gates apply."""
        self._owner()
        self.bridge.record_heartbeat(time.perf_counter_ns())
        return self._send("position", dict(batch=batch, authorization=authorization),
            lambda: self.bridge.send_supervised_position_batch(
                snapshot=snapshot, batch=batch, authorization=authorization))

    def begin_task_position_control(self, authorization: TaskScopedPositionAuthorization):
        self._owner()
        result = self.bridge.begin_task_position_control(authorization)
        self._record("task_position_control_begin", result=result)
        return result

    def send_task_position(self, *, snapshot: TaskControlSnapshot, batch):
        self._owner()
        authorization = self.bridge.active_task_position_authorization
        scope_id = authorization.task_scope_id if authorization is not None else ""
        self.bridge.record_heartbeat(time.perf_counter_ns())
        return self._send(
            "task_position",
            dict(task_scope_id=scope_id, snapshot=snapshot, batch=batch),
            lambda: self.bridge.send_task_scoped_position_batch(snapshot=snapshot, batch=batch),
        )

    def end_task_position_control(self, *, task_scope_id: str):
        self._owner()
        result = self.bridge.end_task_position_control(task_scope_id=task_scope_id)
        self._record("task_position_control_end", result=result)
        return result

    def commissioning_snapshot(self, **site):
        self._owner()
        return self.bridge.create_j1_commissioning_snapshot(**site)

    def watchdog_tick(self):
        self._owner()
        result = self.bridge.watchdog_tick(time.perf_counter_ns())
        self._record("watchdog", result=result)
        return result

    def wait_joint_target(self, target_raw, *, timeout_s=1.5,
                          tolerance_raw=MANUAL_FEEDBACK_LIMITS.target_joint_tolerance_raw,
                          stable_for_ns=MANUAL_FEEDBACK_LIMITS.stable_duration_ns,
                          stable_max_gap_ns=MANUAL_FEEDBACK_LIMITS.stable_max_gap_ns,
                          stability_drift_raw=MANUAL_FEEDBACK_LIMITS.held_joint_drift_tolerance_raw):
        """Require a fresh target match that remains stable; never resend."""
        self._owner()
        target = tuple(target_raw)
        if len(target) != 6 or any(type(v) is not int for v in target):
            raise ValueError("target requires six integer raw joint positions")
        if (not 0 < timeout_s <= 5 or type(tolerance_raw) is not int or tolerance_raw < 0
                or type(stable_for_ns) is not int or stable_for_ns <= 0
                or type(stable_max_gap_ns) is not int or stable_max_gap_ns <= 0
                or type(stability_drift_raw) is not int or stability_drift_raw < 0):
            raise ValueError("invalid bounded timeout or raw tolerance")
        started = time.perf_counter_ns()
        deadline = time.monotonic() + timeout_s
        stable_samples = []
        last_state = self.state()
        while time.monotonic() < deadline:
            self.poll(min(0.05, max(0.001, deadline - time.monotonic())))
            state = self.state()
            stamps = state["channel_timestamps_ns"]
            last_state = state
            joint_stamps = tuple(stamps.get(hex(i), 0) for i in (0x2A5, 0x2A6, 0x2A7))
            sample_stamp = max(joint_stamps, default=0)
            healthy = (state["complete_fresh"] and state.get("joint_skew_ns", 20_000_001) <= 20_000_000
                    and state.get("ctrl_mode_raw") == 1 and state.get("arm_status_raw") == 0
                    and state.get("error_code") == 0 and all(state["motor_enabled"])
                    and all(stamp > started for stamp in joint_stamps)
                    and sample_stamp > 0)
            matches = healthy and all(
                abs(a - b) <= tolerance_raw
                for a, b in zip(state.get("joint_raw_0p001deg", ()), target)
            )
            if matches:
                current = tuple(state["joint_raw_0p001deg"])
                if stable_samples and (
                    sample_stamp <= stable_samples[-1][0]
                    or sample_stamp - stable_samples[-1][0] > stable_max_gap_ns
                ):
                    stable_samples = []
                if stable_samples and any(
                    max(row[1][axis] for row in (*stable_samples, (sample_stamp, current)))
                    - min(row[1][axis] for row in (*stable_samples, (sample_stamp, current)))
                    > stability_drift_raw for axis in range(6)
                ):
                    stable_samples = []
                stable_samples.append((sample_stamp, current))
            else:
                stable_samples = []
            stable_observed_ns = (
                stable_samples[-1][0] - stable_samples[0][0] if stable_samples else 0
            )
            if matches and stable_observed_ns >= stable_for_ns:
                residual = [b-a for a, b in zip(state["joint_raw_0p001deg"], target)]
                result = dict(status="JOINT_FEEDBACK_MATCH", state=state,
                              target_residual_raw= residual,
                              target_tolerance_raw=tolerance_raw,
                              stable_duration_required_ns=stable_for_ns,
                              stable_duration_observed_ns=stable_observed_ns,
                              stable_samples=len(stable_samples),
                              stability_drift_tolerance_raw=stability_drift_raw)
                self._record("target_check", result=result)
                return result
        final_raw = last_state.get("joint_raw_0p001deg")
        residual = None if not final_raw or len(final_raw) != 6 else [b-a for a, b in zip(final_raw, target)]
        result = dict(status="TARGET_NOT_CONFIRMED_NO_RETRY", state=last_state,
                      target_residual_raw=residual, target_tolerance_raw=tolerance_raw,
                      stable_duration_required_ns=stable_for_ns,
                      stable_duration_observed_ns=(stable_samples[-1][0]-stable_samples[0][0]
                                                   if stable_samples else 0),
                      stable_samples=len(stable_samples),
                      stability_drift_tolerance_raw=stability_drift_raw)
        self._record("target_check", result=result)
        return result

    def send_bounded_model_trial(self, *, snapshot, batch, operator_approved):
        self._owner()
        self.bridge.record_heartbeat(time.perf_counter_ns())
        return self._send("bounded_model_trial", dict(batch=batch),
            lambda: self.bridge.send_bounded_model_trial(snapshot=snapshot,
                batch=batch, operator_approved=operator_approved))

    def create_manual_gamepad_snapshot(self, **site):
        self._owner()
        return self.bridge.create_manual_gamepad_snapshot(**site)

    def send_manual_gamepad_target(self, *, snapshot, batch):
        self._owner()
        self.bridge.record_heartbeat(time.perf_counter_ns())
        return self._send("manual_gamepad_target", dict(snapshot=snapshot, batch=batch),
            lambda: self.bridge.send_manual_gamepad_target(snapshot=snapshot, batch=batch))

    def send_j1(self, *, snapshot, batch, authorization):
        self._owner()
        self.bridge.record_heartbeat(time.perf_counter_ns())
        return self._send("j1", dict(batch=batch, authorization=authorization),
            lambda: self.bridge.send_supervised_j1_commissioning_step(
                snapshot=snapshot, batch=batch, authorization=authorization))

    def send_query(self, query, *, operator_approved):
        return self._send("query", query, lambda: self.bridge.send_diagnostic_query(
            query, operator_approved=operator_approved, now_monotonic_ns=time.perf_counter_ns()))

    def set_mode_stage(self, stage, **site):
        return self._send("mode_stage", dict(stage=stage, **site),
            lambda: self.bridge.send_mode_initialization_stage(stage, **site))

    def enable_j6(self, **site):
        return self._send("enable_j6", site, lambda: self.bridge.send_j6_enable_once(**site))

    def open_gripper_1mm(self, *, operator_approved, **site):
        return self.open_gripper_step(operator_approved=operator_approved, **site)

    def open_gripper_step(self, *, operator_approved, delta_raw=1000, effort_raw=100, **site):
        payload = dict(operator_approved=operator_approved, delta_raw=delta_raw,
                       effort_raw=effort_raw, **site)
        return self._send("gripper_opening", payload,
            lambda: self.bridge.send_gripper_opening_once(**payload))

    def close(self):
        if self._closed:
            return
        if self._log is not None:
            self._owner()
        try:
            self.bridge.close()
        finally:
            if self._log is not None:
                try:
                    self._record("closed", physical_stop_claimed=False)
                finally:
                    self._log.close()
            self._closed = True

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
