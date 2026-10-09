"""SafetyBridge-owned session for explicitly approved PiPER getter CAN TX.

Getter frames are CAN data transmissions, even though they do not command
motion. This session has a fixed query allowlist and a fresh-feedback
preflight. It does not expose the gs_usb device or a generic CAN-frame sender.
"""

from __future__ import annotations

import time
from pathlib import Path

from laptop.runtime.config import RuntimeConfig
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import CanFrame, PiperCanTransport
from laptop.transport.diagnostic_queries import DiagnosticCanQuery
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


RUNTIME_CONFIG_PATH = Path(__file__).resolve().parents[2] / "configs/runtime/diagnostic_query.json"
MAX_PREFLIGHT_TIMEOUT_S = 5.0
MAX_FEEDBACK_AGE_NS = 250_000_000
MAX_QUERY_BUDGET = 16


class DiagnosticFeedbackPreflightError(RuntimeError):
    pass


class DiagnosticQueryRejected(PermissionError):
    def __init__(self, event: dict) -> None:
        super().__init__("SafetyBridge rejected diagnostic query: " + ", ".join(event["reasons"]))
        self.event = event


class CanDiagnosticSession:
    """Own one SafetyBridge session and send only explicit getter queries."""

    def __init__(
        self,
        *,
        approved: bool,
        preflight_timeout_s: float = 3.0,
        max_queries: int = 1,
        transport: PiperCanTransport | None = None,
        runtime_config_path: Path = RUNTIME_CONFIG_PATH,
    ) -> None:
        if type(approved) is not bool or not approved:
            raise PermissionError("explicit approval is required for diagnostic CAN queries")
        if (type(preflight_timeout_s) not in (int, float)
                or not 0 < preflight_timeout_s <= MAX_PREFLIGHT_TIMEOUT_S):
            raise ValueError("feedback preflight timeout must be within (0, 5] seconds")
        if type(max_queries) is not int or not 1 <= max_queries <= MAX_QUERY_BUDGET:
            raise ValueError("diagnostic query budget must be an integer in [1, 16]")
        self.config = RuntimeConfig.load(runtime_config_path)
        if (self.config.mode is not RuntimeMode.DIAGNOSTIC_QUERY
                or not self.config.can_tx_enabled
                or self.config.command_tx_enabled
                or self.config.single_can_owner != SafetyBridge.OWNER):
            raise PermissionError("diagnostic runtime config does not authorize getter-only mode")
        self.approved = approved
        self.preflight_timeout_s = float(preflight_timeout_s)
        self.max_queries = max_queries
        self.query_attempts = 0
        self.bridge = SafetyBridge(
            mode=self.config.mode,
            transport=transport if transport is not None else GsUsbPiperCanTransport(),
            limits=UNVERIFIED_ANALYSIS_LIMITS,
            diagnostic_query_tx_enabled=True,
        )
        self.preflight_frames: list[CanFrame] = []
        self.frames_after_preflight: list[CanFrame] = []
        self.audit_events: list[dict] = []
        self._entered = False

    def __enter__(self) -> "CanDiagnosticSession":
        try:
            self.bridge.start_read_only()
            self._entered = True
            deadline = time.monotonic() + self.preflight_timeout_s
            latest: dict[int, int] = {}
            while time.monotonic() < deadline:
                now_ns = time.perf_counter_ns()
                if self._feedback_ready(latest, now_ns):
                    return self
                remaining_ms = max(1, min(100, int((deadline - time.monotonic()) * 1000)))
                frame = self.bridge.receive_frame(timeout_ms=remaining_ms)
                if frame is None or not self._is_required_feedback(frame):
                    continue
                self.preflight_frames.append(frame)
                latest[frame.arbitration_id] = frame.received_monotonic_ns
                if frame.arbitration_id == 0x2A1 and (
                    frame.data[1] != 0 or frame.data[6:8] != b"\x00\x00"
                ):
                    raise DiagnosticFeedbackPreflightError(
                        "PiPER arm status reports an error; no diagnostic query sent"
                    )
            missing = sorted(SafetyBridge.DIAGNOSTIC_FEEDBACK_IDS - latest.keys())
            stale = sorted(
                can_id for can_id, timestamp in latest.items()
                if time.perf_counter_ns() - timestamp > MAX_FEEDBACK_AGE_NS
            )
            raise DiagnosticFeedbackPreflightError(
                f"fresh PiPER feedback preflight failed; missing={missing}, stale={stale}; "
                "no diagnostic query sent"
            )
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _is_required_feedback(frame: CanFrame) -> bool:
        return (
            frame.arbitration_id in SafetyBridge.DIAGNOSTIC_FEEDBACK_IDS
            and not frame.is_extended_id
            and not frame.is_error_frame
            and len(frame.data) == 8
        )

    @staticmethod
    def _feedback_ready(latest: dict[int, int], now_ns: int) -> bool:
        if not SafetyBridge.DIAGNOSTIC_FEEDBACK_IDS.issubset(latest):
            return False
        return all(0 <= now_ns - latest[can_id] <= MAX_FEEDBACK_AGE_NS
                   for can_id in SafetyBridge.DIAGNOSTIC_FEEDBACK_IDS)

    def send_query(self, query: DiagnosticCanQuery) -> dict:
        if not self._entered:
            raise RuntimeError("diagnostic session is not active")
        if self.query_attempts >= self.max_queries:
            raise PermissionError("diagnostic query budget exhausted; no additional query sent")
        self.query_attempts += 1
        decision = self.bridge.send_diagnostic_query(
            query,
            operator_approved=self.approved,
            now_monotonic_ns=time.perf_counter_ns(),
        )
        event = decision.audit_event()
        self.audit_events.append(event)
        if not decision.query_sent:
            raise DiagnosticQueryRejected(event)
        return event

    def receive_frame(self, timeout_ms: int = 100) -> CanFrame | None:
        if not self._entered:
            raise RuntimeError("diagnostic session is not active")
        frame = self.bridge.receive_frame(timeout_ms=timeout_ms)
        if frame is not None:
            self.frames_after_preflight.append(frame)
        return frame

    def close(self) -> None:
        if self.bridge.state is not BridgeState.CLOSED:
            self._entered = False
            self.bridge.close()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
