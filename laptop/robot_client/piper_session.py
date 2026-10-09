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

from laptop.robot_client.feedback import decode_feedback_frame
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


class PiperSession:
    """Connect, receive, inspect, transmit typed commands, and close one CAN owner.

    Supply an explicitly configured bridge for control. The default bridge is
    receive-only. Units reported here are nominal SDK units, not calibration.
    """

    def __init__(self, log_path, *, bridge=None):
        self.bridge = bridge if bridge is not None else SafetyBridge(
            mode=RuntimeMode.POLICY_REJECT_ONLY, transport=GsUsbPiperCanTransport(),
            limits=UNVERIFIED_ANALYSIS_LIMITS)
        self.log_path = Path(log_path)
        self._log = None
        self._thread = None
        self._closed = False
        self._latest = {}
        self._sequence = 0

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
            self._record("rx", frame=frame)
            count += 1
            if not frame.is_extended_id and not frame.is_error_frame and len(frame.data) == 8:
                self._latest[frame.arbitration_id] = (frame.received_monotonic_ns, frame.data)
        return count

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
        result["channel_timestamps_ns"] = {hex(i): self._latest[i][0]
                                             for i in required if i in self._latest}
        return result

    def _send(self, operation, payload, callback):
        self._owner()
        self._record("tx_requested", operation=operation, payload=payload)
        try:
            receipt = callback()
        except Exception as exc:
            self._record("tx_failed_no_retry", operation=operation,
                         error_type=type(exc).__name__, reason=str(exc))
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

    def commissioning_snapshot(self, **site):
        self._owner()
        return self.bridge.create_j1_commissioning_snapshot(**site)

    def watchdog_tick(self):
        self._owner()
        result = self.bridge.watchdog_tick(time.perf_counter_ns())
        self._record("watchdog", result=result)
        return result

    def wait_joint_target(self, target_raw, *, timeout_s=1.5, tolerance_raw=50):
        """Receive feedback until matching or timeout; never resend a command."""
        self._owner()
        target = tuple(target_raw)
        if len(target) != 6 or any(type(v) is not int for v in target):
            raise ValueError("target requires six integer raw joint positions")
        if not 0 < timeout_s <= 5 or type(tolerance_raw) is not int or tolerance_raw < 0:
            raise ValueError("invalid bounded timeout or raw tolerance")
        started = time.perf_counter_ns()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(min(0.05, max(0.001, deadline - time.monotonic())))
            state = self.state()
            stamps = state["channel_timestamps_ns"]
            if (state["complete_fresh"] and state.get("joint_skew_ns", 20_000_001) <= 20_000_000
                    and state.get("ctrl_mode_raw") == 1 and state.get("arm_status_raw") == 0
                    and state.get("error_code") == 0 and all(state["motor_enabled"])
                    and all(stamps[hex(i)] > started for i in (0x2A5, 0x2A6, 0x2A7))
                    and all(abs(a - b) <= tolerance_raw
                            for a, b in zip(state["joint_raw_0p001deg"], target))):
                result = dict(status="JOINT_FEEDBACK_MATCH", state=state)
                self._record("target_check", result=result)
                return result
        result = dict(status="TARGET_NOT_CONFIRMED_NO_RETRY", state=self.state())
        self._record("target_check", result=result)
        return result

    def send_bounded_model_trial(self, *, snapshot, batch, operator_approved):
        self._owner()
        self.bridge.record_heartbeat(time.perf_counter_ns())
        return self._send("bounded_model_trial", dict(batch=batch),
            lambda: self.bridge.send_bounded_model_trial(snapshot=snapshot,
                batch=batch, operator_approved=operator_approved))

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
