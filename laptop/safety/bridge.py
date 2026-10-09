"""Fail-closed policy, diagnostics, and one-step position transport boundary.

SHADOW and POLICY_REJECT_ONLY never transmit. DIAGNOSTIC_QUERY has one
separate, allowlisted getter-only transport path; it cannot send command
proposals and never calls ``transport.send``. SUPERVISED_POLICY_STEP has one
dedicated four-frame position path behind an explicit runtime gate.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import asdict, replace
from enum import Enum
import hashlib
import json
import math
from numbers import Real
import struct
import time
from typing import Optional, Tuple

from laptop.control.contracts import (
    CommandProposal,
    PolicyPrediction,
    RobotStateSnapshot,
    TimingContract,
)
from laptop.runtime.modes import RuntimeMode, require_offline_mode
from laptop.transport.can_transport import CanFrame, FakeCanTransport, PiperCanTransport
from laptop.transport.diagnostic_queries import DiagnosticCanQuery
from laptop.transport.position_commands import (
    J1CommissioningAuthorization,
    J1CommissioningBatch,
    J1CommissioningSnapshot,
    PositionBatchWriteReceipt,
    PositionCommandBatch,
    SupervisedPositionAuthorization,
    J1CommissioningWriteReceipt,
    validate_j1_commissioning_execution,
    COMMISSIONING_FEEDBACK_IDS,
)


class BridgeState(str, Enum):
    """Offline lifecycle state; faults stay latched until the bridge is rebuilt."""

    READY = "READY"
    FAULT_LATCHED = "FAULT_LATCHED"
    CLOSED = "CLOSED"


@dataclass(frozen=True)
class SafetyLimits:
    joint_min_rad: Tuple[float, ...]
    joint_max_rad: Tuple[float, ...]
    max_joint_delta_rad: Tuple[float, ...]
    gripper_min_m: float
    gripper_max_m: float
    max_gripper_delta_m: float
    max_state_age_ns: int = 250_000_000
    max_prediction_response_latency_ns: int = 250_000_000
    max_proposal_age_ns: int = 250_000_000
    max_proposal_validity_ns: int = 250_000_000
    max_heartbeat_age_ns: int = 250_000_000

    def __post_init__(self) -> None:
        for field_name in ("joint_min_rad", "joint_max_rad", "max_joint_delta_rad"):
            try:
                raw_values = tuple(getattr(self, field_name))
            except TypeError as exc:
                raise ValueError(f"{field_name} must contain 6 finite real values") from exc
            if len(raw_values) != 6 or any(
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(float(value))
                for value in raw_values
            ):
                raise ValueError(f"{field_name} must contain 6 finite values")
            values = tuple(float(value) for value in raw_values)
            object.__setattr__(self, field_name, values)
        for field_name in ("gripper_min_m", "gripper_max_m", "max_gripper_delta_m"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{field_name} must be a finite number")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{field_name} must be a finite number")
            object.__setattr__(self, field_name, value)
        if any(low >= high for low, high in zip(self.joint_min_rad, self.joint_max_rad)):
            raise ValueError("each joint minimum must be below its maximum")
        if any(value <= 0 for value in self.max_joint_delta_rad):
            raise ValueError("joint delta limits must be positive")
        if not self.gripper_min_m < self.gripper_max_m:
            raise ValueError("gripper minimum must be below maximum")
        if self.max_gripper_delta_m <= 0:
            raise ValueError("gripper delta limit must be positive")
        for field_name in (
            "max_state_age_ns",
            "max_prediction_response_latency_ns",
            "max_proposal_age_ns",
            "max_proposal_validity_ns",
            "max_heartbeat_age_ns",
        ):
            value = getattr(self, field_name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{field_name} must be a positive integer")

    @property
    def timing_contract(self) -> TimingContract:
        return TimingContract(
            max_state_age_ns=self.max_state_age_ns,
            max_prediction_response_latency_ns=self.max_prediction_response_latency_ns,
            max_proposal_age_ns=self.max_proposal_age_ns,
            max_proposal_validity_ns=self.max_proposal_validity_ns,
            max_heartbeat_age_ns=self.max_heartbeat_age_ns,
        )


# These broad bounds exist only so reject-only replay can classify malformed
# proposals.  They are not verified PiPER hardware limits and never authorize
# transmission.
UNVERIFIED_ANALYSIS_LIMITS = SafetyLimits(
    joint_min_rad=(-3.2,) * 6,
    joint_max_rad=(3.2,) * 6,
    max_joint_delta_rad=(0.1,) * 6,
    gripper_min_m=0.0,
    gripper_max_m=0.10,
    max_gripper_delta_m=0.01,
)


@dataclass(frozen=True)
class BridgeDecision:
    mode: RuntimeMode
    proposal_id: Optional[str]
    source_request_id: str
    accepted_for_offline_analysis: bool
    hardware_control_allowed: bool
    transport_send_attempted: bool
    reasons: Tuple[str, ...]
    event_type: str
    decision_monotonic_ns: int
    input_sha256: str
    bridge_state: BridgeState
    timing_contract: Tuple[Tuple[str, object], ...] = ()

    def audit_event(self) -> dict:
        """Return the immutable decision as a serializable audit record."""
        return {
            "schema": "piper_safety_bridge_audit_v1",
            "event_type": self.event_type,
            "decision_monotonic_ns": self.decision_monotonic_ns,
            "input_sha256": self.input_sha256,
            "bridge_state": self.bridge_state.value,
            "timing_contract": dict(self.timing_contract),
            "mode": self.mode.value,
            "proposal_id": self.proposal_id,
            "source_request_id": self.source_request_id,
            "accepted_for_offline_analysis": self.accepted_for_offline_analysis,
            "hardware_control_allowed": self.hardware_control_allowed,
            "transport_send_attempted": self.transport_send_attempted,
            "transport_scope": ("fake_in_memory_record_only"
                                if self.mode is RuntimeMode.FAKE_CAN_REHEARSAL
                                else "robot_transport"),
            "can_data_tx_enabled": False,
            "reasons": list(self.reasons),
        }


@dataclass(frozen=True)
class DiagnosticQueryDecision:
    query_name: str
    request_can_id: int
    request_payload_sha256: str
    expected_response_can_id: int
    operator_approved: bool
    transport_attempted: bool
    query_sent: bool
    hardware_control_allowed: bool
    command_tx_enabled: bool
    reasons: Tuple[str, ...]
    decision_monotonic_ns: int
    input_sha256: str
    bridge_state: BridgeState

    def audit_event(self) -> dict:
        return {
            "schema": "piper_safety_bridge_diagnostic_query_v1",
            "event_type": "diagnostic_query_decision",
            "query_name": self.query_name,
            "request_can_id": f"0x{self.request_can_id:03X}",
            "request_payload_sha256": self.request_payload_sha256,
            "expected_response_can_id": f"0x{self.expected_response_can_id:03X}",
            "operator_approved": self.operator_approved,
            "transport_attempted": self.transport_attempted,
            "query_sent": self.query_sent,
            "hardware_control_allowed": self.hardware_control_allowed,
            "command_tx_enabled": self.command_tx_enabled,
            "reasons": list(self.reasons),
            "decision_monotonic_ns": self.decision_monotonic_ns,
            "input_sha256": self.input_sha256,
            "bridge_state": self.bridge_state.value,
        }


@dataclass(frozen=True)
class HILGateDecision:
    """Auditable fail-closed result for an HIL entrypoint not yet enabled."""

    entrypoint: str
    decision_monotonic_ns: int
    input_sha256: str
    reasons: Tuple[str, ...]

    def audit_event(self) -> dict:
        return {
            "schema": "piper_safety_bridge_hil_gate_v1",
            "event_type": "hil_gate_rejection",
            "runtime_mode": RuntimeMode.HIL_SINGLE_STEP.value,
            "entrypoint": self.entrypoint,
            "decision_monotonic_ns": self.decision_monotonic_ns,
            "input_sha256": self.input_sha256,
            "hardware_control_allowed": False,
            "can_tx_attempted": False,
            "reasons": list(self.reasons),
        }


class SafetyBridge:
    """Fail-closed bridge; the only send-like path is exact-Fake-only rehearsal."""

    OWNER = "safety-bridge"
    DIAGNOSTIC_FEEDBACK_IDS = frozenset((0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8))
    J1_COMPETING_COMMAND_IDS = frozenset((*range(0x150, 0x160), 0x471))
    _NON_LATCHING_REASONS = frozenset(
        {
            "next_observed_state_proxy_is_not_command",
            "proxy_to_command_mapping_unverified",
            "runtime_mode_forbids_can_tx",
        }
    )

    def __init__(
        self,
        *,
        mode: RuntimeMode,
        transport: PiperCanTransport,
        limits: SafetyLimits,
        diagnostic_query_tx_enabled: bool = False,
        fake_rehearsal_enabled: bool = False,
        supervised_position_tx_enabled: bool = False,
        supervised_j1_commissioning_tx_enabled: bool = False,
        supervised_mode_initialization_tx_enabled: bool = False,
    ) -> None:
        if (supervised_mode_initialization_tx_enabled
                and mode is not RuntimeMode.SUPERVISED_MODE_INITIALIZATION):
            raise ValueError("initialization TX cannot be combined with another runtime mode")
        if mode is RuntimeMode.SUPERVISED_MODE_INITIALIZATION:
            from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
            if type(transport) is not GsUsbPiperCanTransport:
                raise PermissionError("initialization requires the candleLight transport")
            if supervised_mode_initialization_tx_enabled is not True:
                raise PermissionError("initialization requires its explicit transmit gate")
            if (diagnostic_query_tx_enabled or fake_rehearsal_enabled
                    or supervised_position_tx_enabled or supervised_j1_commissioning_tx_enabled):
                raise ValueError("initialization cannot combine other transmit gates")
        elif mode is RuntimeMode.DIAGNOSTIC_QUERY:
            if type(diagnostic_query_tx_enabled) is not bool or not diagnostic_query_tx_enabled:
                raise PermissionError("diagnostic query mode requires its explicit runtime gate")
            if fake_rehearsal_enabled or supervised_position_tx_enabled:
                raise ValueError("other transmit gates cannot be combined with diagnostic queries")
            if supervised_j1_commissioning_tx_enabled:
                raise ValueError("J1 commissioning gate cannot be combined with diagnostic queries")
        elif mode is RuntimeMode.FAKE_CAN_REHEARSAL:
            if type(transport) is not FakeCanTransport:
                raise PermissionError("FAKE_CAN_REHEARSAL accepts only the exact FakeCanTransport")
            if (type(fake_rehearsal_enabled) is not bool or not fake_rehearsal_enabled
                    or not transport.allow_recorded_send):
                raise PermissionError("fake-only rehearsal requires two explicit in-memory gates")
            if diagnostic_query_tx_enabled:
                raise ValueError("diagnostic query gate cannot be enabled in fake rehearsal mode")
            if supervised_position_tx_enabled:
                raise ValueError("supervised position gate cannot be enabled in fake rehearsal mode")
            if supervised_j1_commissioning_tx_enabled:
                raise ValueError("J1 commissioning gate cannot be enabled in fake rehearsal mode")
        elif mode is RuntimeMode.SUPERVISED_POLICY_STEP:
            from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport

            if type(transport) is not GsUsbPiperCanTransport:
                raise PermissionError(
                    "SUPERVISED_POLICY_STEP accepts only the exact GsUsbPiperCanTransport"
                )
            if (type(supervised_position_tx_enabled) is not bool
                    or not supervised_position_tx_enabled):
                raise PermissionError("supervised position TX requires its explicit runtime gate")
            if diagnostic_query_tx_enabled or fake_rehearsal_enabled:
                raise ValueError("supervised position mode cannot combine other transmit gates")
            if supervised_j1_commissioning_tx_enabled:
                raise ValueError("policy position mode cannot combine J1 commissioning TX")
        elif mode is RuntimeMode.SUPERVISED_J1_COMMISSIONING:
            from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport

            if type(transport) is not GsUsbPiperCanTransport:
                raise PermissionError(
                    "SUPERVISED_J1_COMMISSIONING accepts only the candleLight transport"
                )
            if (type(supervised_j1_commissioning_tx_enabled) is not bool
                    or not supervised_j1_commissioning_tx_enabled):
                raise PermissionError("J1 commissioning TX requires its explicit runtime gate")
            if (diagnostic_query_tx_enabled or fake_rehearsal_enabled
                    or supervised_position_tx_enabled):
                raise ValueError("J1 commissioning mode cannot combine other transmit gates")
        else:
            require_offline_mode(mode)
            if (diagnostic_query_tx_enabled or fake_rehearsal_enabled
                    or supervised_position_tx_enabled or supervised_j1_commissioning_tx_enabled):
                raise ValueError("diagnostic query TX cannot be enabled in an offline policy mode")
        self.mode = mode
        self.transport = transport
        self.limits = limits
        self.diagnostic_query_tx_enabled = diagnostic_query_tx_enabled
        self.fake_rehearsal_enabled = fake_rehearsal_enabled
        self.supervised_position_tx_enabled = supervised_position_tx_enabled
        self.supervised_j1_commissioning_tx_enabled = supervised_j1_commissioning_tx_enabled
        self.supervised_mode_initialization_tx_enabled = supervised_mode_initialization_tx_enabled
        self._mode_initialization_index = 0
        self._mode_initialization_last_write_ns = 0
        self._mode_initialization_last_frame = None
        self.transport.claim(self.OWNER)
        self._state = BridgeState.READY
        self._last_state_sequence = -1
        self._last_heartbeat_ns: Optional[int] = None
        self._read_only_started = False
        self._diagnostic_feedback: dict[int, tuple[int, bytes]] = {}
        self._used_j1_commissioning_approval_ids: set[str] = set()
        self._j1_feedback: dict[int, tuple[int, bytes]] = {}
        self._j1_joint_history: list[tuple[int, tuple[int, ...]]] = []
        self._j1_competing_command_seen = False
        self._j1_snapshot_sequence = 0

    @staticmethod
    def reject_unimplemented_hil_entrypoint(entrypoint: str) -> HILGateDecision:
        """Create a SafetyBridge audit decision without opening a transport.

        Legacy HIL scripts use this shared veto until live hardware gates pass
        and a command-capable bridge is deliberately implemented. This method
        never constructs a CAN transport and cannot send a frame.
        """
        if not isinstance(entrypoint, str) or not entrypoint.strip():
            raise ValueError("HIL entrypoint name is required")
        now_ns = time.perf_counter_ns()
        reasons = (
            "hil_single_step_bridge_not_implemented",
            "live_hardware_gates_unverified",
            "runtime_mode_forbids_can_tx",
        )
        payload = json.dumps(
            {
                "schema": "piper_hil_gate_input_v1",
                "runtime_mode": RuntimeMode.HIL_SINGLE_STEP.value,
                "entrypoint": entrypoint,
                "reasons": reasons,
                "decision_monotonic_ns": now_ns,
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return HILGateDecision(
            entrypoint=entrypoint,
            decision_monotonic_ns=now_ns,
            input_sha256=hashlib.sha256(payload).hexdigest(),
            reasons=reasons,
        )

    @property
    def state(self) -> BridgeState:
        return self._state

    def start_read_only(self) -> None:
        """Start CAN receive through the bridge's exclusively claimed transport."""
        if self._state is not BridgeState.READY:
            raise PermissionError("only a ready safety bridge can start CAN receive")
        if self._read_only_started:
            raise RuntimeError("SafetyBridge CAN receive is already active")
        try:
            self.transport.start_read_only(self.OWNER)
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise
        self._read_only_started = True

    def receive_frame(self, timeout_ms: int = 100) -> Optional[CanFrame]:
        """Receive one CAN frame through the bridge-owned transport."""
        if self._state is not BridgeState.READY or not self._read_only_started:
            raise PermissionError("SafetyBridge read-only CAN receive is not active")
        try:
            frame = self.transport.receive_frame(self.OWNER, timeout_ms)
            if (isinstance(frame, CanFrame)
                    and not frame.is_extended_id
                    and not frame.is_error_frame
                    and len(frame.data) == 8
                    and frame.arbitration_id in self.DIAGNOSTIC_FEEDBACK_IDS):
                self._diagnostic_feedback[frame.arbitration_id] = (
                    frame.received_monotonic_ns,
                    frame.data,
                )
            if isinstance(frame, CanFrame) and not frame.is_extended_id and not frame.is_error_frame:
                if frame.arbitration_id == 0x2A8 and len(frame.data) == 8:
                    self._gripper_commissioning_feedback = (frame.received_monotonic_ns, frame.data)
                if frame.arbitration_id in self.J1_COMPETING_COMMAND_IDS:
                    own_initialization_echo = (
                        self.mode is RuntimeMode.SUPERVISED_MODE_INITIALIZATION
                        and self._mode_initialization_last_frame is not None
                        and frame.received_monotonic_ns > self._mode_initialization_last_write_ns
                        and frame.arbitration_id == self._mode_initialization_last_frame.arbitration_id
                        and frame.data == self._mode_initialization_last_frame.data
                    )
                    if not own_initialization_echo:
                        self._j1_competing_command_seen = True
                if frame.arbitration_id in COMMISSIONING_FEEDBACK_IDS and len(frame.data) == 8:
                    self._j1_feedback[frame.arbitration_id] = (
                        frame.received_monotonic_ns, frame.data,
                    )
                    if frame.arbitration_id == 0x2A7 and all(
                        can_id in self._j1_feedback for can_id in (0x2A5, 0x2A6, 0x2A7)
                    ):
                        joints = tuple(
                            value
                            for can_id in (0x2A5, 0x2A6, 0x2A7)
                            for value in struct.unpack(">ii", self._j1_feedback[can_id][1])
                        )
                        self._j1_joint_history.append((frame.received_monotonic_ns, joints))
                        cutoff = frame.received_monotonic_ns - 1_000_000_000
                        self._j1_joint_history = [
                            item for item in self._j1_joint_history if item[0] >= cutoff
                        ]
            return frame
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise

    def mode_initialization_snapshot(self, *, require_zero: bool = False) -> dict:
        from laptop.transport.mode_initialization import feedback_snapshot
        if (self.mode is not RuntimeMode.SUPERVISED_MODE_INITIALIZATION
                or self._state is not BridgeState.READY or not self._read_only_started):
            raise PermissionError("an active initialization bridge is required")
        if self._j1_competing_command_seen:
            raise PermissionError("another control frame was received; stop initialization")
        return feedback_snapshot(
            self._j1_feedback, after_ns=self._mode_initialization_last_write_ns,
            require_zero=require_zero, history=self._j1_joint_history,
        )

    def send_mode_initialization_stage(
        self, stage: str, *, operator_authorized_reset: bool,
        zero_pose_supported: bool, physical_estop_ready: bool,
        motion_envelope_cleared: bool,
    ) -> dict:
        from laptop.transport.mode_initialization import ModeInitializationFrame, STAGES
        if not self.supervised_mode_initialization_tx_enabled:
            raise PermissionError("initialization TX is disabled")
        if not all(value is True for value in (
                operator_authorized_reset, zero_pose_supported,
                physical_estop_ready, motion_envelope_cleared)):
            raise PermissionError("reset approval, supported zero pose, E-stop and cleared space are required")
        if (self._mode_initialization_index >= len(STAGES)
                or stage != STAGES[self._mode_initialization_index]):
            raise PermissionError("only the fixed reset/standby/CAN sequence is permitted")
        snapshot = self.mode_initialization_snapshot(require_zero=True)
        expected = (2,) if stage == "reset" else ((0,) if stage == "can_joint" else (0, 1, 2))
        if snapshot["ctrl_mode_raw"] not in expected:
            raise PermissionError("mode feedback does not match this initialization stage")
        if stage == "reset" and snapshot["teach_status_raw"] not in (0, 2):
            raise PermissionError("teaching recording or playback must be stopped before reset")
        frame = ModeInitializationFrame(stage)
        self._mode_initialization_index += 1
        try:
            stamp = self.transport.send_mode_initialization_frame(self.OWNER, frame)
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise
        self._mode_initialization_last_write_ns = stamp
        self._mode_initialization_last_frame = frame
        return {"stage": stage, "can_id": hex(frame.arbitration_id),
                "data_hex": frame.data.hex(), "adapter_accepted_monotonic_ns": stamp,
                "pre_write_feedback": snapshot}

    def send_j6_enable_once(self, *, operator_authorized_enable: bool,
                            zero_pose_supported: bool, physical_estop_ready: bool,
                            motion_envelope_cleared: bool) -> dict:
        from laptop.transport.mode_initialization import J6EnableFrame
        if not self.supervised_mode_initialization_tx_enabled:
            raise PermissionError("dedicated initialization TX is required")
        if not all(value is True for value in (
                operator_authorized_enable, zero_pose_supported,
                physical_estop_ready, motion_envelope_cleared)):
            raise PermissionError("enable approval and onsite conditions are required")
        snapshot = self.mode_initialization_snapshot(require_zero=True)
        if snapshot["ctrl_mode_raw"] != 1:
            raise PermissionError("J6 enable requires verified CAN_CTRL")
        if not all(snapshot["motor_enabled"][:5]) or snapshot["motor_enabled"][5]:
            raise PermissionError("this path is only for J6 disabled with J1-J5 enabled")
        frame = J6EnableFrame()
        try:
            stamp = self.transport.send_j6_enable_once(self.OWNER, frame)
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise
        self._mode_initialization_last_write_ns = stamp
        self._mode_initialization_last_frame = frame
        return {"can_id": hex(frame.arbitration_id), "data_hex": frame.data.hex(),
                "adapter_accepted_monotonic_ns": stamp, "pre_write_feedback": snapshot}

    def send_diagnostic_query(
        self,
        query: DiagnosticCanQuery,
        *,
        operator_approved: bool,
        now_monotonic_ns: object,
    ) -> DiagnosticQueryDecision:
        """Send one allowlisted getter only after fresh PiPER feedback.

        This channel is deliberately disjoint from command proposals. It is
        unavailable in SHADOW and POLICY_REJECT_ONLY, and it never enables,
        moves, resets, or reconfigures the robot.
        """
        if not isinstance(query, DiagnosticCanQuery):
            raise TypeError("query must be a validated DiagnosticCanQuery")
        reasons: list[str] = []
        now_valid = type(now_monotonic_ns) is int and now_monotonic_ns > 0
        if not now_valid:
            reasons.append("invalid_current_time")
        if self.mode is not RuntimeMode.DIAGNOSTIC_QUERY or not self.diagnostic_query_tx_enabled:
            reasons.append("runtime_mode_forbids_diagnostic_query")
        if type(operator_approved) is not bool or not operator_approved:
            reasons.append("operator_query_approval_missing")
        if self._state is not BridgeState.READY:
            reasons.append("bridge_not_ready")
        if not self._read_only_started:
            reasons.append("CAN_receive_session_not_active")
        if not self.transport.connected:
            reasons.append("CAN_transport_disconnected")

        max_age_ns = min(self.limits.max_state_age_ns, 250_000_000)
        feedback_ages: dict[str, Optional[int]] = {}
        for can_id in sorted(self.DIAGNOSTIC_FEEDBACK_IDS):
            entry = self._diagnostic_feedback.get(can_id)
            age = now_monotonic_ns - entry[0] if now_valid and entry else None
            feedback_ages[f"0x{can_id:03X}"] = age
            if entry is None:
                reasons.append(f"diagnostic_feedback_missing_0x{can_id:03X}")
            elif not now_valid or age is None or age < 0 or age > max_age_ns:
                reasons.append(f"diagnostic_feedback_stale_0x{can_id:03X}")
        status = self._diagnostic_feedback.get(0x2A1)
        if status is not None and (status[1][1] != 0 or status[1][6:8] != b"\x00\x00"):
            reasons.append("arm_status_reports_error")

        reasons = list(dict.fromkeys(reasons))
        input_record = {
            "schema": "piper_diagnostic_query_input_v1",
            "mode": self.mode.value,
            "request_can_id": query.request_can_id,
            "request_payload_hex": query.payload.hex(),
            "expected_response_can_id": query.expected_response_can_id,
            "operator_approved": operator_approved is True,
            "now_monotonic_ns": now_monotonic_ns if now_valid else None,
            "feedback_ages_ns": feedback_ages,
            "max_feedback_age_ns": max_age_ns,
            "reasons": reasons,
        }
        encoded = json.dumps(
            input_record, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        input_hash = hashlib.sha256(encoded).hexdigest()
        attempted = False
        sent = False
        if not reasons:
            attempted = True
            try:
                sent = self.transport.send_diagnostic_query(self.OWNER, query) is True
                if not sent:
                    reasons.append("diagnostic_query_not_confirmed_delivery_unknown")
            except Exception:
                reasons.append("diagnostic_query_delivery_unknown_no_retry")
            if reasons:
                self._state = BridgeState.FAULT_LATCHED
        return DiagnosticQueryDecision(
            query_name=query.name,
            request_can_id=query.request_can_id,
            request_payload_sha256=query.payload_sha256,
            expected_response_can_id=query.expected_response_can_id,
            operator_approved=operator_approved is True,
            transport_attempted=attempted,
            query_sent=sent,
            hardware_control_allowed=False,
            command_tx_enabled=False,
            reasons=tuple(reasons),
            decision_monotonic_ns=now_monotonic_ns if now_valid else 0,
            input_sha256=input_hash,
            bridge_state=self._state,
        )

    def gripper_opening_snapshot(self, **site):
        snapshot = self.create_j1_commissioning_snapshot(**site)
        feedback = getattr(self, "_gripper_commissioning_feedback", None)
        now = time.perf_counter_ns()
        if feedback is None or not 0 <= now - feedback[0] <= 120_000_000:
            raise PermissionError("fresh bridge-owned gripper feedback required")
        if feedback[1][6] & 0xBF:
            raise PermissionError("gripper feedback reports a non-enable status flag")
        return snapshot, feedback

    def send_oem_gripper_test_frame(self, *, command, operator_approved):
        if (operator_approved is not True or self.mode is not RuntimeMode.SUPERVISED_J1_COMMISSIONING
                or not self.supervised_j1_commissioning_tx_enabled or self._state is not BridgeState.READY):
            raise PermissionError('explicit supervised OEM test authorization required')
        feedback=getattr(self,'_gripper_commissioning_feedback',None)
        if feedback is None or command.feedback_monotonic_ns != feedback[0] or feedback[1][6]&0xBF:
            raise PermissionError('command requires bridge-owned healthy gripper feedback')
        try:
            return self.transport.send_oem_gripper_test_frame(self.OWNER,command)
        except Exception:
            self._state=BridgeState.FAULT_LATCHED
            raise

    def send_gripper_opening_once(self, *, operator_approved, delta_raw=1000, effort_raw=100, close_to_zero=False, open_to_max=False, **site):
        from laptop.transport.gripper_commissioning import GripperOpeningStep
        if operator_approved is not True:
            raise PermissionError("explicit single +1 mm gripper opening approval required")
        snapshot, feedback = self.gripper_opening_snapshot(**site)
        command = GripperOpeningStep(int.from_bytes(feedback[1][:4], "big", signed=True),
                                    feedback[0], time.perf_counter_ns(), delta_raw, effort_raw, close_to_zero, open_to_max)
        command.validate(time.perf_counter_ns())
        try:
            return self.transport.send_gripper_opening_once(self.OWNER, command)
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise

    def send_supervised_position_batch(
        self,
        *,
        snapshot: RobotStateSnapshot,
        batch: PositionCommandBatch,
        authorization: SupervisedPositionAuthorization,
    ) -> PositionBatchWriteReceipt:
        """Send one exact, bounded position batch through the sole live bridge.

        This method does not make site attestations true. It enforces the
        software contract and requires a separately supplied, one-use operator
        authorization for this proposal.
        """
        if (self.mode is not RuntimeMode.SUPERVISED_POLICY_STEP
                or not self.supervised_position_tx_enabled):
            raise PermissionError("runtime mode does not enable supervised position TX")
        if type(self.transport).__name__ != "GsUsbPiperCanTransport":
            raise PermissionError("supervised position TX requires the candleLight transport")
        if type(snapshot) is not RobotStateSnapshot:
            raise TypeError("snapshot must be a RobotStateSnapshot")
        if type(batch) is not PositionCommandBatch:
            raise TypeError("batch must be a PositionCommandBatch")
        if type(authorization) is not SupervisedPositionAuthorization:
            raise TypeError("authorization must be a SupervisedPositionAuthorization")
        from laptop.transport.gs_usb_piper_can import (
            CanPositionDeliveryUnknown,
            GsUsbPiperCanTransport,
        )

        if type(self.transport) is not GsUsbPiperCanTransport:
            raise PermissionError("supervised position TX requires the candleLight transport")
        if not self._read_only_started or not self.transport.connected:
            raise PermissionError("supervised position TX requires active bridge-owned CAN receive")

        now_ns = time.perf_counter_ns()
        decision = self.inspect_command_proposal(
            snapshot=snapshot,
            proposal=batch.proposal,
            now_monotonic_ns=now_ns,
        )
        if not decision.accepted_for_offline_analysis:
            raise PermissionError(
                "supervised position proposal rejected: " + ", ".join(decision.reasons)
            )
        authorization.validate_for(batch.proposal, now_monotonic_ns=now_ns)

        try:
            return self.transport.send_supervised_position(
                self.OWNER, batch, authorization, snapshot, self.limits,
            )
        except CanPositionDeliveryUnknown:
            self._state = BridgeState.FAULT_LATCHED
            raise

    def send_bounded_model_trial(self, *, snapshot, batch, operator_approved):
        """An approved small SDK-unit experiment, not full P12 certification."""
        from laptop.transport.bounded_trial import BoundedTrialBatch
        if (self.mode is not RuntimeMode.SUPERVISED_J1_COMMISSIONING
                or not self.supervised_j1_commissioning_tx_enabled
                or self._state is not BridgeState.READY or operator_approved is not True):
            raise PermissionError("dedicated supervised trial approval required")
        if type(batch) is not BoundedTrialBatch:
            raise TypeError("typed trial required")
        self._validate_j1_snapshot_matches_bridge(snapshot)
        now = time.perf_counter_ns()
        gripper = self._gripper_commissioning_feedback
        if gripper is None or not 0 <= now-gripper[0] <= 120_000_000:
            raise PermissionError("fresh gripper feedback required")
        if int.from_bytes(gripper[1][:4], "big", signed=True) != batch.current_gripper:
            raise PermissionError("trial gripper source mismatch")
        if gripper[1][6] & 0xBF:
            raise PermissionError("gripper reports a fault")
        if self._last_heartbeat_ns is None or not 0 <= now-self._last_heartbeat_ns <= 250_000_000:
            raise PermissionError("fresh heartbeat required")
        try:
            return self.transport.send_bounded_model_trial(self.OWNER, snapshot, batch)
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise

    def send_supervised_j1_commissioning_step(
        self,
        *,
        snapshot: J1CommissioningSnapshot,
        batch: J1CommissioningBatch,
        authorization: J1CommissioningAuthorization,
    ) -> J1CommissioningWriteReceipt:
        """Send one explicitly selected fixed joint commissioning step.

        This mode is separate from policy control and never accepts a gripper
        target or arbitrary joint vector. Onsite E-stop, low-speed, and motion-
        envelope checks remain explicit operator attestations.
        """
        if (self.mode is not RuntimeMode.SUPERVISED_J1_COMMISSIONING
                or not self.supervised_j1_commissioning_tx_enabled):
            raise PermissionError("runtime mode does not enable J1 commissioning TX")
        if self._state is not BridgeState.READY:
            raise PermissionError("SafetyBridge is not ready for J1 commissioning")
        from laptop.transport.gs_usb_piper_can import (
            CanJ1CommissioningDeliveryUnknown,
            GsUsbPiperCanTransport,
        )

        if type(self.transport) is not GsUsbPiperCanTransport:
            raise PermissionError("J1 commissioning TX requires the candleLight transport")
        if not self._read_only_started or not self.transport.connected:
            raise PermissionError("J1 commissioning TX requires bridge-owned CAN feedback")
        if type(snapshot) is not J1CommissioningSnapshot:
            raise TypeError("snapshot must be a J1CommissioningSnapshot")
        if type(batch) is not J1CommissioningBatch:
            raise TypeError("batch must be a J1CommissioningBatch")
        if type(authorization) is not J1CommissioningAuthorization:
            raise TypeError("authorization must be a J1CommissioningAuthorization")
        self._validate_j1_snapshot_matches_bridge(snapshot)
        if authorization.approval_id in self._used_j1_commissioning_approval_ids:
            raise PermissionError("J1 commissioning authorization is one-use")
        now_ns = time.perf_counter_ns()
        heartbeat_ns = self._last_heartbeat_ns
        if (heartbeat_ns is None or heartbeat_ns > now_ns
                or now_ns - heartbeat_ns > self.limits.max_heartbeat_age_ns):
            raise PermissionError("fresh SafetyBridge heartbeat is required for J1 commissioning")
        validate_j1_commissioning_execution(
            snapshot, batch, authorization, now_monotonic_ns=now_ns,
        )
        self._used_j1_commissioning_approval_ids.add(authorization.approval_id)
        try:
            return self.transport.send_supervised_j1_commissioning_step(
                self.OWNER, snapshot, batch, authorization,
            )
        except CanJ1CommissioningDeliveryUnknown:
            self._state = BridgeState.FAULT_LATCHED
            raise

    def create_j1_commissioning_snapshot(
        self,
        *,
        physical_estop_ready: bool,
        low_speed_confirmed: bool,
        motion_envelope_cleared: bool,
    ) -> J1CommissioningSnapshot:
        """Build a commissioning snapshot only from feedback this bridge read."""
        if (self.mode is not RuntimeMode.SUPERVISED_J1_COMMISSIONING
                or not self.supervised_j1_commissioning_tx_enabled
                or not self._read_only_started
                or self._state is not BridgeState.READY):
            raise PermissionError("J1 snapshot requires an active dedicated commissioning bridge")
        missing = [can_id for can_id in COMMISSIONING_FEEDBACK_IDS
                   if can_id not in self._j1_feedback]
        if missing:
            raise PermissionError("J1 commissioning feedback is incomplete")
        self._j1_snapshot_sequence += 1
        joints = tuple(
            value
            for can_id in (0x2A5, 0x2A6, 0x2A7)
            for value in struct.unpack(">ii", self._j1_feedback[can_id][1])
        )
        snapshot = J1CommissioningSnapshot(
            sequence=self._j1_snapshot_sequence,
            joint_raw_0p001deg=joints,
            status_payload=self._j1_feedback[0x2A1][1],
            feedback_timestamps_ns=tuple(
                (can_id, self._j1_feedback[can_id][0])
                for can_id in COMMISSIONING_FEEDBACK_IDS
            ),
            motor_enabled=tuple(
                bool(self._j1_feedback[can_id][1][5] & 0x40)
                for can_id in range(0x261, 0x267)
            ),
            stable_joint_samples=tuple(self._j1_joint_history),
            competing_command_seen=self._j1_competing_command_seen,
            physical_estop_ready=physical_estop_ready,
            low_speed_confirmed=low_speed_confirmed,
            motion_envelope_cleared=motion_envelope_cleared,
        )
        from laptop.transport.position_commands import validate_j1_commissioning_snapshot

        validate_j1_commissioning_snapshot(snapshot, now_monotonic_ns=time.perf_counter_ns())
        return snapshot

    def _validate_j1_snapshot_matches_bridge(self, snapshot: J1CommissioningSnapshot) -> None:
        """Reject caller-created or stale snapshots that differ from received CAN feedback."""
        if any(can_id not in self._j1_feedback for can_id in COMMISSIONING_FEEDBACK_IDS):
            raise PermissionError("bridge has not received all J1 commissioning feedback")
        latest_timestamps = tuple(
            (can_id, self._j1_feedback[can_id][0])
            for can_id in COMMISSIONING_FEEDBACK_IDS
        )
        latest_joints = tuple(
            value
            for can_id in (0x2A5, 0x2A6, 0x2A7)
            for value in struct.unpack(">ii", self._j1_feedback[can_id][1])
        )
        latest_motors = tuple(
            bool(self._j1_feedback[can_id][1][5] & 0x40)
            for can_id in range(0x261, 0x267)
        )
        if (snapshot.feedback_timestamps_ns != latest_timestamps
                or snapshot.status_payload != self._j1_feedback[0x2A1][1]
                or snapshot.joint_raw_0p001deg != latest_joints
                or snapshot.motor_enabled != latest_motors
                or snapshot.competing_command_seen != self._j1_competing_command_seen
                or snapshot.stable_joint_samples != tuple(self._j1_joint_history)):
            raise PermissionError("J1 commissioning snapshot does not match bridge-received feedback")

    def record_heartbeat(self, monotonic_ns: int) -> None:
        if self._state is BridgeState.CLOSED:
            raise PermissionError("closed safety bridge cannot accept heartbeats")
        if type(monotonic_ns) is not int or monotonic_ns <= 0:
            raise ValueError("heartbeat timestamp must be positive")
        if (self._last_heartbeat_ns is not None
                and monotonic_ns <= self._last_heartbeat_ns):
            raise ValueError("heartbeat must advance monotonically")
        self._last_heartbeat_ns = monotonic_ns

    def watchdog_tick(self, now_monotonic_ns: object) -> BridgeDecision:
        """Audit heartbeat health and latch faults without sending or holding.

        A caller must schedule this tick at a bounded cadence. This offline
        hook is not a background watchdog and does not implement robot safe-hold.
        """
        reasons = []
        if self._state is BridgeState.FAULT_LATCHED:
            reasons.append("bridge_fault_latched")
        elif self._state is BridgeState.CLOSED:
            reasons.append("bridge_closed")

        valid_now = type(now_monotonic_ns) is int and now_monotonic_ns > 0
        if not valid_now:
            reasons.append("invalid_current_time")
        elif self._last_heartbeat_ns is None:
            reasons.append("heartbeat_missing")
        elif now_monotonic_ns < self._last_heartbeat_ns:
            reasons.append("heartbeat_from_future")
        elif now_monotonic_ns - self._last_heartbeat_ns > self.limits.max_heartbeat_age_ns:
            reasons.append("heartbeat_stale")

        self._latch_fault(reasons)
        encoded = json.dumps(
            {
                "schema": "piper_safety_bridge_watchdog_input_v1",
                "mode": self.mode.value,
                "bridge_state": self._state.value,
                "now_monotonic_ns": now_monotonic_ns if valid_now else None,
                "invalid_now_type": None if valid_now else type(now_monotonic_ns).__name__,
                "last_heartbeat_ns": self._last_heartbeat_ns,
                "max_heartbeat_age_ns": self.limits.max_heartbeat_age_ns,
                "timing_contract": asdict(self.limits.timing_contract),
                "reasons": list(dict.fromkeys(reasons)),
            },
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return BridgeDecision(
            mode=self.mode,
            proposal_id=None,
            source_request_id="watchdog",
            accepted_for_offline_analysis=not reasons,
            hardware_control_allowed=False,
            transport_send_attempted=False,
            reasons=tuple(dict.fromkeys(reasons)),
            event_type="watchdog_decision",
            decision_monotonic_ns=now_monotonic_ns if valid_now else 0,
            input_sha256=hashlib.sha256(encoded).hexdigest(),
            bridge_state=self._state,
            timing_contract=tuple(sorted(asdict(self.limits.timing_contract).items())),
        )

    def close(self) -> None:
        if self._state is BridgeState.CLOSED:
            return
        try:
            if self._read_only_started:
                self.transport.stop_read_only(self.OWNER)
                self._read_only_started = False
            if self.transport.owner == self.OWNER:
                self.transport.release(self.OWNER)
        except Exception:
            self._state = BridgeState.FAULT_LATCHED
            raise
        self._state = BridgeState.CLOSED

    def inspect_prediction(
        self,
        *,
        snapshot: RobotStateSnapshot,
        prediction: PolicyPrediction,
        now_monotonic_ns: int,
    ) -> BridgeDecision:
        reasons = list(self._state_reasons(snapshot, now_monotonic_ns))
        if prediction.request_sequence != snapshot.sequence:
            reasons.append("prediction_state_sequence_mismatch")
        if prediction.observation_monotonic_ns != snapshot.captured_monotonic_ns:
            reasons.append("prediction_observation_timestamp_mismatch")
        if (prediction.response_latency_ns
                > self.limits.max_prediction_response_latency_ns):
            reasons.append("prediction_response_latency_exceeded")
        if type(now_monotonic_ns) is int and now_monotonic_ns > 0:
            prediction_age_ns = now_monotonic_ns - prediction.received_monotonic_ns
            if prediction_age_ns < 0:
                reasons.append("prediction_received_from_future")
            elif prediction_age_ns > self.limits.max_proposal_age_ns:
                reasons.append("prediction_stale")
            if (self.limits.timing_contract.first_target_must_be_future_at_evaluation
                    and now_monotonic_ns >= prediction.target_timestamps_monotonic_ns[0]):
                reasons.append("prediction_target_deadline_passed")
        reasons.extend(
            (
                "next_observed_state_proxy_is_not_command",
                "proxy_to_command_mapping_unverified",
                "runtime_mode_forbids_can_tx",
            )
        )
        self._remember(snapshot)
        # Unverified units reject the proxy prediction for analysis/control,
        # but do not make the passive observation bridge unhealthy. Command
        # proposals still treat this same condition as a latched fault.
        self._latch_fault(
            [reason for reason in reasons if reason != "state_units_unverified"]
        )
        decision = BridgeDecision(
            mode=self.mode,
            proposal_id=None,
            source_request_id=prediction.request_id,
            accepted_for_offline_analysis=not any(
                reason
                for reason in reasons
                if reason not in {
                    "next_observed_state_proxy_is_not_command",
                    "proxy_to_command_mapping_unverified",
                    "runtime_mode_forbids_can_tx",
                }
            ),
            hardware_control_allowed=False,
            transport_send_attempted=False,
            reasons=tuple(dict.fromkeys(reasons)),
            event_type="policy_prediction_decision",
            decision_monotonic_ns=(
                now_monotonic_ns
                if type(now_monotonic_ns) is int and now_monotonic_ns > 0
                else 0
            ),
            input_sha256=self._input_sha256("policy_prediction", snapshot, prediction),
            bridge_state=self._state,
            timing_contract=tuple(sorted(asdict(self.limits.timing_contract).items())),
        )
        return decision

    def inspect_command_proposal(
        self,
        *,
        snapshot: RobotStateSnapshot,
        proposal: CommandProposal,
        now_monotonic_ns: int,
    ) -> BridgeDecision:
        reasons = list(self._state_reasons(snapshot, now_monotonic_ns))
        if proposal.source_state_sequence != snapshot.sequence:
            reasons.append("proposal_state_sequence_mismatch")
        if type(now_monotonic_ns) is int and now_monotonic_ns > 0:
            if proposal.created_monotonic_ns > now_monotonic_ns:
                reasons.append("proposal_from_future")
            if proposal.created_monotonic_ns < snapshot.received_monotonic_ns:
                reasons.append("proposal_created_before_source_state_received")
            if now_monotonic_ns - proposal.created_monotonic_ns > self.limits.max_proposal_age_ns:
                reasons.append("proposal_stale")
            if now_monotonic_ns > proposal.valid_until_monotonic_ns:
                reasons.append("proposal_expired")
        if (proposal.valid_until_monotonic_ns - proposal.created_monotonic_ns
                > self.limits.max_proposal_validity_ns):
            reasons.append("proposal_validity_window_exceeded")
        if not proposal.hardware_units_verified:
            reasons.append("hardware_units_unverified")
        if not proposal.mapping_verified:
            reasons.append("command_mapping_unverified")
        for index, (value, low, high) in enumerate(
            zip(proposal.target[:6], self.limits.joint_min_rad, self.limits.joint_max_rad),
            start=1,
        ):
            if not low <= value <= high:
                reasons.append(f"joint_{index}_position_limit")
        for index, (target, current, maximum) in enumerate(
            zip(
                proposal.target[:6],
                snapshot.joint_position_rad,
                self.limits.max_joint_delta_rad,
            ),
            start=1,
        ):
            if abs(target - current) > maximum:
                reasons.append(f"joint_{index}_delta_limit")
        gripper = proposal.target[6]
        if not self.limits.gripper_min_m <= gripper <= self.limits.gripper_max_m:
            reasons.append("gripper_position_limit")
        if abs(gripper - snapshot.gripper_position_m) > self.limits.max_gripper_delta_m:
            reasons.append("gripper_delta_limit")
        offline_reasons = tuple(dict.fromkeys(reasons))
        supervised_position_mode = (
            self.mode is RuntimeMode.SUPERVISED_POLICY_STEP
            and self.supervised_position_tx_enabled
        )
        if (self.mode is not RuntimeMode.FAKE_CAN_REHEARSAL
                and not supervised_position_mode):
            reasons.append("runtime_mode_forbids_can_tx")
        self._remember(snapshot)
        self._latch_fault(offline_reasons)
        decision = BridgeDecision(
            mode=self.mode,
            proposal_id=proposal.proposal_id,
            source_request_id=proposal.source_request_id,
            accepted_for_offline_analysis=not offline_reasons,
            hardware_control_allowed=False,
            transport_send_attempted=False,
            reasons=tuple(dict.fromkeys(reasons)),
            event_type="command_proposal_decision",
            decision_monotonic_ns=(
                now_monotonic_ns
                if type(now_monotonic_ns) is int and now_monotonic_ns > 0
                else 0
            ),
            input_sha256=self._input_sha256("command_proposal", snapshot, proposal),
            bridge_state=self._state,
            timing_contract=tuple(sorted(asdict(self.limits.timing_contract).items())),
        )
        return decision

    def record_fake_single_step(
        self,
        *,
        snapshot: RobotStateSnapshot,
        proposal: CommandProposal,
        now_monotonic_ns: int,
    ) -> BridgeDecision:
        """Run the normal proposal checks, then record one item in Fake memory.

        This method is not a robot sender: construction is exact-Fake-only,
        requires explicit fake-only gates, and the real USB-CAN transport
        unconditionally rejects command sends.
        """
        if (self.mode is not RuntimeMode.FAKE_CAN_REHEARSAL
                or type(self.transport) is not FakeCanTransport
                or not self.fake_rehearsal_enabled
                or not self.transport.allow_recorded_send):
            raise PermissionError("fake-only single-step rehearsal is not enabled")
        if not isinstance(snapshot, RobotStateSnapshot):
            raise TypeError("snapshot must be a RobotStateSnapshot")
        if type(proposal) is not CommandProposal:
            if isinstance(proposal, PolicyPrediction):
                raise TypeError("next-state proxy predictions are not command proposals")
            raise TypeError("proposal must be a CommandProposal")
        decision = self.inspect_command_proposal(
            snapshot=snapshot,
            proposal=proposal,
            now_monotonic_ns=now_monotonic_ns,
        )
        if not decision.accepted_for_offline_analysis:
            return replace(decision, event_type="fake_command_rehearsal_rejected")
        try:
            self.transport.send(self.OWNER, proposal)
        except Exception:
            self._latch_fault(("fake_transport_record_failed",))
            return replace(
                decision,
                accepted_for_offline_analysis=False,
                transport_send_attempted=True,
                reasons=tuple(dict.fromkeys((*decision.reasons, "fake_transport_record_failed"))),
                event_type="fake_command_rehearsal_rejected",
                bridge_state=self._state,
            )
        return replace(
            decision,
            transport_send_attempted=True,
            event_type="fake_command_recorded_no_can_tx",
        )

    def _input_sha256(self, input_kind: str, snapshot: RobotStateSnapshot, message: object) -> str:
        encoded = json.dumps(
            {
                "schema": "piper_safety_bridge_input_v1",
                "input_kind": input_kind,
                "snapshot": asdict(snapshot),
                "message": asdict(message),
                "timing_contract": asdict(self.limits.timing_contract),
            },
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _state_reasons(
        self, snapshot: RobotStateSnapshot, now_monotonic_ns: int
    ) -> Tuple[str, ...]:
        reasons = []
        if self._state is BridgeState.FAULT_LATCHED:
            reasons.append("bridge_fault_latched")
        elif self._state is BridgeState.CLOSED:
            reasons.append("bridge_closed")
        valid_now = type(now_monotonic_ns) is int and now_monotonic_ns > 0
        if not valid_now:
            reasons.append("invalid_current_time")
        elif now_monotonic_ns < snapshot.received_monotonic_ns:
            reasons.append("state_received_from_future")
        elif now_monotonic_ns - snapshot.received_monotonic_ns > self.limits.max_state_age_ns:
            reasons.append("state_stale")
        if snapshot.sequence <= self._last_state_sequence:
            reasons.append("state_out_of_order_or_replayed")
        if not snapshot.connected:
            reasons.append("robot_disconnected")
        if not self.transport.connected:
            reasons.append("can_transport_disconnected")
            transport_fault_reasons = {
                "usb_disconnect": "usb_disconnected",
                "transport_process_exit": "transport_process_exited",
            }
            fault_reason = transport_fault_reasons.get(self.transport.disconnect_reason)
            if fault_reason:
                reasons.append(fault_reason)
        if not snapshot.status_ok:
            reasons.append("robot_status_not_ok")
        if not snapshot.units_verified:
            reasons.append("state_units_unverified")
        if self._last_heartbeat_ns is None:
            reasons.append("heartbeat_missing")
        elif not valid_now:
            pass
        elif now_monotonic_ns < self._last_heartbeat_ns:
            reasons.append("heartbeat_from_future")
        elif now_monotonic_ns - self._last_heartbeat_ns > self.limits.max_heartbeat_age_ns:
            reasons.append("heartbeat_stale")
        return tuple(reasons)

    def _latch_fault(self, reasons: Tuple[str, ...] | list[str]) -> None:
        if self._state is not BridgeState.READY:
            return
        if any(reason not in self._NON_LATCHING_REASONS for reason in reasons):
            self._state = BridgeState.FAULT_LATCHED

    def _remember(self, snapshot: RobotStateSnapshot) -> None:
        self._last_state_sequence = max(self._last_state_sequence, snapshot.sequence)
