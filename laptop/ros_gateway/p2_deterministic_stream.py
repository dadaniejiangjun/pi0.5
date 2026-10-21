"""Locally approved, model-free P2 position stream over the ROS control RPC.

This path is deliberately separate from P5 task authorization. It accepts one
small joint probe, one bounded gripper hold request, then a fixed absolute
joint target at 10 Hz for at most 60 seconds through the existing owner queue.
"""
from __future__ import annotations

from concurrent.futures import TimeoutError as FutureTimeoutError
import math
import threading
import time

from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_LIMITS,
    validate_continuous_gripper_target,
)
from laptop.piper.unit_conversion import rad_to_sdk_joint
from laptop.ros_gateway.control_rpc import validate_control_request
from laptop.ros_gateway.control_rpc_dispatcher import _result, _validate_dispatch_feedback, _validated_arguments
from laptop.ros_gateway.manual_commissioning import ManualCommissioningTarget


MAX_FEEDBACK_AGE_NS = 120_000_000
REQUEST_VALIDITY_MAX_MS = 250
MAX_SOURCE_OBSERVATION_AGE_NS = REQUEST_VALIDITY_MAX_MS * 1_000_000
JOINT_STABILITY_WINDOW_NS = 300_000_000
JOINT_STABILITY_TOLERANCE_RAW = 50  # 0.050 degree, matching the CAN-side gate.
MAX_STABILITY_SAMPLE_GAP_NS = MAX_FEEDBACK_AGE_NS
JOINT_STEP_RAW = 1_000  # 1 degree for this deterministic motion; hard cap is 20 degrees.
MAX_JOINT_STEP_RAW = 20_000
STREAM_DURATION_NS = 60_000_000_000
STREAM_RATE_HZ = 10
STREAM_PERIOD_NS = 100_000_000
STREAM_EXPECTED_UPDATES = 600
P2_PROBE_REQUESTS = 2
P2_BOUNDED_TRIAL_REQUEST_BUDGET = P2_PROBE_REQUESTS + STREAM_EXPECTED_UPDATES
MIN_STREAM_ADAPTER_INTERVAL_NS = 80_000_000


def wait_for_fresh_feedback(*, latest, spin_once, now_ns, deadline_ns):
    """Wait within this update slot; never substitute an expired observation."""
    while now_ns() < deadline_ns:
        sample = latest()
        if sample is not None:
            return sample
        remaining_s = (deadline_ns - now_ns()) / 1e9
        if remaining_s <= 0:
            break
        spin_once(min(0.01, remaining_s))
    return None


def source_ready_for_rpc(sample, now_ns):
    """Admit source age within one period, reserving the remaining 20 ms.

    The bridge still enforces 120 ms at callback time. Arrival age alone can
    hide transport time and leave no budget for the ROS service callback.
    """
    stamp = sample.get("source_local_monotonic_ns")
    uncertainty = sample.get("source_clock_mapping_uncertainty_ns")
    if (type(stamp) is not int or stamp <= 0
            or type(uncertainty) is not int or uncertainty < 0):
        return False
    age = now_ns - stamp
    return -uncertainty <= age and age + uncertainty <= STREAM_PERIOD_NS


def summarize_stable_joint_feedback_window(
    samples: list[dict], latest_sample: dict | None, *,
    now_local_monotonic_ns: int, session_id: str,
) -> dict:
    """Check that fresh same-session joint feedback satisfies the CAN gate.

    This helper only decides when the runner may submit its next request. It
    keeps the controller-side 300 ms / 0.050 degree checks intact; the owner
    thread repeats the authoritative validation immediately before each send.
    """
    result = {
        "stable": False,
        "window_required_ns": JOINT_STABILITY_WINDOW_NS,
        "drift_tolerance_raw_0p001deg": JOINT_STABILITY_TOLERANCE_RAW,
        "maximum_sample_gap_ns": MAX_STABILITY_SAMPLE_GAP_NS,
        "sample_count": 0,
        "window_observed_ns": 0,
        "max_joint_drift_raw_0p001deg": None,
        "reason": "waiting for a stable same-session feedback window",
    }
    if (type(now_local_monotonic_ns) is not int or now_local_monotonic_ns <= 0
            or not isinstance(session_id, str) or not session_id
            or not isinstance(latest_sample, dict)):
        result["reason"] = "latest state or local monotonic time is unavailable"
        return result
    latest_ns = latest_sample.get("source_local_monotonic_ns")
    latest_sequence = latest_sample.get("sequence")
    if (type(latest_ns) is not int or type(latest_sequence) is not int
            or latest_sample.get("session_id") != session_id
            or latest_sample.get("fake") is not False
            or latest_sample.get("hardware_connected") is not True
            or latest_sample.get("feedback_fresh") is not True
            or latest_sample.get("ctrl_mode_raw") != 1
            or latest_sample.get("arm_status_raw") != 0
            or latest_sample.get("error_code") != 0
            or len(latest_sample.get("motor_enabled", ())) != 6
            or not all(latest_sample.get("motor_enabled", ()))
            or not 0 <= now_local_monotonic_ns - latest_ns <= MAX_FEEDBACK_AGE_NS):
        result["reason"] = "latest same-session feedback is stale or unhealthy"
        return result

    history = []
    oldest_allowed_ns = latest_ns - JOINT_STABILITY_WINDOW_NS - MAX_STABILITY_SAMPLE_GAP_NS
    for sample in samples:
        if not isinstance(sample, dict) or sample.get("session_id") != session_id:
            continue
        stamp = sample.get("source_local_monotonic_ns")
        sequence = sample.get("sequence")
        if type(stamp) is not int or type(sequence) is not int:
            continue
        if stamp < oldest_allowed_ns or stamp > latest_ns:
            continue
        if (sample.get("fake") is not False
                or sample.get("hardware_connected") is not True
                or sample.get("feedback_fresh") is not True
                or sample.get("ctrl_mode_raw") != 1
                or sample.get("arm_status_raw") != 0
                or sample.get("error_code") != 0
                or len(sample.get("motor_enabled", ())) != 6
                or not all(sample.get("motor_enabled", ()))):
            result["reason"] = "unhealthy sample inside the stability window"
            return result
        joints = sample.get("joint_position_rad")
        if not isinstance(joints, (list, tuple)) or len(joints) != 6:
            result["reason"] = "joint feedback is incomplete"
            return result
        try:
            raw = tuple(int(rad_to_sdk_joint(value)) for value in joints)
        except (TypeError, ValueError, OverflowError):
            result["reason"] = "joint feedback contains an invalid value"
            return result
        history.append((stamp, sequence, raw))

    history.sort(key=lambda item: (item[0], item[1]))
    if not history or history[-1][1] != latest_sequence or history[-1][0] != latest_ns:
        result["reason"] = "stability history does not end at the current feedback sample"
        return result
    if any(right[0] <= left[0] or right[1] <= left[1]
           for left, right in zip(history, history[1:])):
        result["reason"] = "feedback sequence or timestamp is not strictly increasing"
        return result
    result["sample_count"] = len(history)
    result["window_observed_ns"] = history[-1][0] - history[0][0]
    max_drift = max(
        max(item[2][joint] for item in history) - min(item[2][joint] for item in history)
        for joint in range(6)
    )
    result["max_joint_drift_raw_0p001deg"] = max_drift
    if result["window_observed_ns"] < JOINT_STABILITY_WINDOW_NS:
        result["reason"] = "feedback has not yet covered the required 300 ms window"
        return result
    if any(right[0] - left[0] > MAX_STABILITY_SAMPLE_GAP_NS
           for left, right in zip(history, history[1:])):
        result["reason"] = "feedback gap exceeds the freshness limit"
        return result
    if max_drift > JOINT_STABILITY_TOLERANCE_RAW:
        result["reason"] = "joint drift exceeds the existing 0.050 degree gate"
        return result
    result["stable"] = True
    result["reason"] = "fresh joint feedback is stable across the existing window"
    return result


def derive_minimal_j1_target(state: dict, *, delta_raw: int = JOINT_STEP_RAW) -> tuple[float, ...]:
    """Choose a one-degree J1 offset from fresh feedback within task bounds."""
    if type(delta_raw) is not int or not 0 < delta_raw <= MAX_JOINT_STEP_RAW:
        raise ValueError("J1 step must be in 1..20000 SDK raw units")
    _validate_dispatch_feedback(state)
    current = tuple(int(rad_to_sdk_joint(value)) for value in state["joint_position_rad"])
    from laptop.transport.position_commands import TASK_JOINT_BOUNDS_RAW

    low, high = TASK_JOINT_BOUNDS_RAW[0]
    if current[0] + delta_raw <= high:
        target_j1 = current[0] + delta_raw
    elif current[0] - delta_raw >= low:
        target_j1 = current[0] - delta_raw
    else:
        raise ValueError("neither one-degree J1 direction fits the active task envelope")
    target_raw = (target_j1, *current[1:])
    return tuple(math.radians(value / 1000.0) for value in target_raw)


class P2DeterministicStreamDispatcher:
    """Enforce one local P2 test plan while each target uses the owner queue."""

    def __init__(
        self,
        *,
        queue,
        latest_state_provider,
        source_state_provider,
        session_id: str,
        task_scope_id: str,
        joint_target_rad: tuple[float, ...],
        gripper_target_raw: int,
        valid_until_monotonic_ns: int,
        operator_approved: bool,
        physical_estop_ready: bool,
        low_speed_confirmed: bool,
        motion_envelope_cleared: bool,
        stream_duration_ns: int = STREAM_DURATION_NS,
        expected_updates: int = STREAM_EXPECTED_UPDATES,
    ) -> None:
        if (not isinstance(session_id, str) or not session_id.isascii() or not session_id
                or not isinstance(task_scope_id, str) or not task_scope_id.isascii()
                or not task_scope_id):
            raise ValueError("bounded session and task scope IDs are required")
        target = _validated_arguments("set_joint_target", {"joint_position_rad": joint_target_rad})
        if (type(gripper_target_raw) is not int
                or not CONTINUOUS_GRIPPER_LIMITS.minimum_raw
                <= gripper_target_raw <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw):
            raise ValueError("gripper target is outside the shared continuous raw envelope")
        if type(stream_duration_ns) is not int or stream_duration_ns != STREAM_DURATION_NS:
            raise ValueError("P2 deterministic test duration is fixed at 60 seconds")
        if type(expected_updates) is not int or expected_updates != STREAM_EXPECTED_UPDATES:
            raise ValueError("P2 deterministic test requires 600 updates at 10 Hz")
        now = time.perf_counter_ns()
        if (type(valid_until_monotonic_ns) is not int
                or not 0 < valid_until_monotonic_ns - now <= 120_000_000_000):
            raise ValueError("local P2 approval must expire within 120 seconds")
        confirmations = (operator_approved, physical_estop_ready,
                        low_speed_confirmed, motion_envelope_cleared)
        if any(value is not True for value in confirmations):
            raise PermissionError("current local operator, low-speed and site confirmations required")
        self.queue = queue
        self.latest = latest_state_provider
        self.source_state = source_state_provider
        self.session_id = session_id
        self.task_scope_id = task_scope_id
        self.joint_target_rad = tuple(target["joint_position_rad"])
        self.target_joint_raw = tuple(int(rad_to_sdk_joint(value)) for value in self.joint_target_rad)
        self.gripper_target_raw = gripper_target_raw
        self.expiry_ns = valid_until_monotonic_ns
        self.site = dict(operator_approved=True, physical_estop_ready=True,
                         low_speed_confirmed=True, motion_envelope_cleared=True)
        self.stream_duration_ns = stream_duration_ns
        self.expected_updates = expected_updates
        self.lock = threading.Lock()
        self.phase = "joint_probe"
        self.pending = False
        self.failed_reason = ""
        self.stream_started_ns: int | None = None
        self.stream_request_times_ns: list[int] = []
        self.last_stream_adapter_accept_ns: int | None = None
        self.records: list[dict] = []

    @property
    def stream_complete(self) -> bool:
        with self.lock:
            if self.stream_started_ns is None:
                return False
            return (len(self.stream_request_times_ns) >= self.expected_updates
                    or time.perf_counter_ns() - self.stream_started_ns >= self.stream_duration_ns)

    def revoke_session(self, session_id: str) -> None:
        if session_id == self.session_id:
            with self.lock:
                self.failed_reason = "ROS control session disconnected; local P2 approval revoked"

    def dispatch(self, payload: object, *, session_id: str,
                 used_request_ids: set[str]) -> dict:
        request_id = payload.get("request_id", "") if isinstance(payload, dict) else ""
        received_ns = time.perf_counter_ns()
        reservation_owned = False

        def reject(reason: str, *, latch: bool = True) -> dict:
            nonlocal reservation_owned
            with self.lock:
                if reservation_owned:
                    self.pending = False
                    reservation_owned = False
                if latch and not self.failed_reason:
                    self.failed_reason = reason[:192]
                self.records.append({"phase": self.phase, "request_id": request_id,
                                     "received_monotonic_ns": time.perf_counter_ns(),
                                     "accepted": False, "reason": reason[:192]})
            return _result(session_id, request_id, decision="REJECTED_P2_DETERMINISTIC_SCOPE",
                           reason=reason)

        try:
            validate_control_request(payload, session_id=session_id)
            if time.perf_counter_ns() >= self.expiry_ns:
                return reject("local P2 approval expired")
            if session_id != self.session_id or payload["task_scope_id"] != self.task_scope_id:
                return reject("request is outside the locally approved P2 session and scope")
            if payload["request_id"] in used_request_ids:
                return reject("request ID replay")
            if self.failed_reason:
                return reject(self.failed_reason)
            with self.lock:
                already_pending = self.pending
                if not already_pending:
                    self.pending = True
                    reservation_owned = True
            if already_pending:
                return reject("previous owner-queue request is still in flight")
            validity_ms = payload["validity_ms"]
            if type(validity_ms) is not int or not 1 <= validity_ms <= REQUEST_VALIDITY_MAX_MS:
                return reject("P2 position request validity must be 1..250 ms")

            source = self.source_state(payload["source_state_sequence"], session_id)
            request_deadline_ns = received_ns + validity_ms * 1_000_000
            latest = self.latest()
            now = time.perf_counter_ns()
            phase = self.phase

            def latest_is_fresh(state, stamp_ns: int) -> bool:
                return bool(
                    state is not None
                    and state.get("feedback_fresh") is True
                    and type(state.get("gateway_snapshot_monotonic_ns")) is int
                    and 0 <= stamp_ns - state["gateway_snapshot_monotonic_ns"]
                    <= MAX_FEEDBACK_AGE_NS
                )

            def source_latest_pair_is_fresh() -> bool:
                return bool(
                    source
                    and source.get("feedback_fresh") is True
                    and latest is not None
                    and latest.get("session_id") in (None, "", session_id)
                    and source.get("session_id") == session_id
                    and type(latest.get("sequence")) is int
                    and type(source.get("sequence")) is int
                    and source["sequence"] == payload["source_state_sequence"]
                    and source["sequence"] <= latest["sequence"]
                    and all(type(state.get("gateway_snapshot_monotonic_ns")) is int
                            and 0 <= now - state["gateway_snapshot_monotonic_ns"] <= age_limit
                            for state, age_limit in (
                                (latest, MAX_FEEDBACK_AGE_NS),
                                (source, MAX_SOURCE_OBSERVATION_AGE_NS)))
                )

            # Owner sends can briefly age the latest combined snapshot beyond
            # 120 ms. Wait only within this request's existing 250 ms validity
            # window for a new current snapshot; never send from stale feedback.
            while not latest_is_fresh(latest, now):
                if now >= request_deadline_ns:
                    return reject("current hardware feedback stayed stale through request validity")
                time.sleep(min(0.005, max(0.0, (request_deadline_ns - now) / 1e9)))
                latest = self.latest()
                now = time.perf_counter_ns()
            if not source_latest_pair_is_fresh():
                return reject("source/latest feedback is stale, future, cross-session or unavailable")
            _validate_dispatch_feedback(latest)

            operation = payload["operation"]
            arguments = _validated_arguments(operation, payload["arguments"])
            pacing_wait_ns = 0
            if phase == "joint_probe":
                if operation != "set_joint_target" or not self._same_joint(arguments["joint_position_rad"]):
                    return reject("first P2 command must match the approved minimal J1 target")
            elif phase == "gripper_probe":
                if (operation != "set_gripper"
                        or arguments["gripper_raw"] != self.gripper_target_raw
                        or arguments["nominal_effort_nm"] * 1000 != CONTINUOUS_GRIPPER_LIMITS.effort_raw):
                    return reject("gripper probe must match the approved raw hold target and effort")
                validate_continuous_gripper_target(
                    int(latest["gripper_raw"]), self.gripper_target_raw,
                    CONTINUOUS_GRIPPER_LIMITS.effort_raw,
                )
            elif phase == "stream":
                if operation != "set_joint_target" or not self._same_joint(arguments["joint_position_rad"]):
                    return reject("stream target differs from the locally approved fixed joint target")
                if self.stream_started_ns is None:
                    self.stream_started_ns = now
                if now - self.stream_started_ns >= self.stream_duration_ns:
                    return reject("60-second deterministic stream window has ended", latch=False)
                if len(self.stream_request_times_ns) >= self.expected_updates:
                    return reject("600 deterministic stream updates already completed", latch=False)
                if self.last_stream_adapter_accept_ns is not None:
                    earliest_send_ns = self.last_stream_adapter_accept_ns + MIN_STREAM_ADAPTER_INTERVAL_NS
                    if now < earliest_send_ns:
                        if earliest_send_ns >= request_deadline_ns:
                            return reject("request validity expires before the next adapter pacing slot")
                        pacing_wait_ns = earliest_send_ns - now
                        time.sleep(pacing_wait_ns / 1_000_000_000)
                        now = time.perf_counter_ns()
                        if now >= request_deadline_ns:
                            return reject("request expired while waiting for the adapter pacing slot")
                        source = self.source_state(payload["source_state_sequence"], session_id)
                        latest = self.latest()
                        while not latest_is_fresh(latest, now):
                            if now >= request_deadline_ns:
                                return reject("current hardware feedback stayed stale during adapter pacing wait")
                            time.sleep(min(0.005, max(0.0, (request_deadline_ns - now) / 1e9)))
                            latest = self.latest()
                            now = time.perf_counter_ns()
                        if not source_latest_pair_is_fresh():
                            return reject("source/latest feedback expired during adapter pacing wait")
                        _validate_dispatch_feedback(latest)
            else:
                return reject("P2 deterministic plan is no longer accepting requests", latch=False)

            current_joints = tuple(int(rad_to_sdk_joint(value)) for value in latest["joint_position_rad"])
            target_joints = (self.target_joint_raw if operation == "set_joint_target"
                             else current_joints)
            if any(abs(target - current) > JOINT_STEP_RAW
                   for current, target in zip(current_joints, target_joints)):
                return reject("target exceeds the minimal one-degree per-update test step")
            current_gripper = int(latest["gripper_raw"])
            target_gripper = (self.gripper_target_raw if operation == "set_gripper"
                              else current_gripper)
            validate_continuous_gripper_target(
                current_gripper, target_gripper,
                CONTINUOUS_GRIPPER_LIMITS.effort_raw,
            )

            deadline = min(self.expiry_ns, request_deadline_ns)
            target = ManualCommissioningTarget(
                request_id=payload["request_id"], operation=operation,
                target_joint_raw=tuple(target_joints),
                target_gripper_raw=target_gripper,
                valid_until_monotonic_ns=deadline,
                gripper_step_limit_raw=CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
                gripper_profile="continuous_task",
                **self.site,
            )
            record = {"phase": phase, "request_id": payload["request_id"],
                      "received_monotonic_ns": received_ns,
                      "adapter_pacing_wait_ns": pacing_wait_ns,
                      "source_state_sequence": source["sequence"],
                      "dispatch_state_sequence": latest["sequence"],
                      "operation": operation, "accepted": None}
        except Exception as exc:
            return reject(f"{type(exc).__name__}: {exc}")

        try:
            future = self.queue.submit_manual_commissioning(target)
            writes = future.result(timeout=max(0.0, (deadline - time.perf_counter_ns()) / 1e9))
            accepted_ns = time.perf_counter_ns()
            ids = [int(write.arbitration_id) for write in writes]
            result = _result(
                session_id, payload["request_id"],
                decision="ADAPTER_ACCEPTED_P2_MANUAL_RAW_BATCH",
                reason="typed bounded P2 target accepted by the Windows adapter; feedback is evaluated separately",
                accepted=True, can_tx_calls=len(writes), delivery_status="ADAPTER_ACCEPTED",
            )
            result["adapter_frame_ids"] = ids
            result["adapter_accepted_monotonic_ns"] = [
                int(write.adapter_accepted_monotonic_ns) for write in writes
            ]
            record.update({"accepted": True, "adapter_completed_monotonic_ns": accepted_ns,
                           "adapter_frame_ids": ids,
                           "adapter_accepted_monotonic_ns": result["adapter_accepted_monotonic_ns"]})
            with self.lock:
                if phase == "joint_probe":
                    self.phase = "gripper_probe"
                elif phase == "gripper_probe":
                    self.phase = "stream"
                else:
                    self.stream_request_times_ns.append(received_ns)
                    self.last_stream_adapter_accept_ns = int(
                        result["adapter_accepted_monotonic_ns"][-1]
                    )
                self.records.append(record)
            used_request_ids.add(payload["request_id"])
            return result
        except FutureTimeoutError:
            record.update({"accepted": False, "delivery": "UNKNOWN_NO_RETRY",
                           "finished_monotonic_ns": time.perf_counter_ns()})
            with self.lock:
                self.failed_reason = "owner-queue delivery timed out; result unknown; no retry"
                self.records.append(record)
            return _result(session_id, payload["request_id"],
                           decision="P2_DELIVERY_UNKNOWN_NO_RETRY",
                           reason=self.failed_reason, can_tx_calls=None,
                           delivery_status="UNKNOWN")
        except Exception as exc:
            record.update({"accepted": False, "delivery": "FAILED_NO_RETRY",
                           "error": f"{type(exc).__name__}: {exc}"[:192],
                           "finished_monotonic_ns": time.perf_counter_ns()})
            with self.lock:
                self.failed_reason = record["error"]
                self.records.append(record)
            return _result(session_id, payload["request_id"],
                           decision="P2_OWNER_FAILED_NO_RETRY",
                           reason=self.failed_reason, can_tx_calls=None,
                           delivery_status="UNKNOWN")
        finally:
            with self.lock:
                self.pending = False

    def _same_joint(self, values) -> bool:
        return (len(values) == 6 and all(
            math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-12)
            for actual, expected in zip(values, self.joint_target_rad)
        ))
