"""Optional, process-local dispatch boundary for position control RPCs.

The production Windows gateway does not construct this dispatcher. If a
trusted Windows application injects it, a remote request can only become a
short-lived position proposal after a local session-authorizer approves the
current TCP session and a typed task scope is already active in the owner
queue. The injected target factory remains local code and must build a typed
``TaskControlSnapshot``/``PositionCommandBatch`` for PiperSession and
SafetyBridge to validate. This module does not issue authorization, encode raw
CAN frames, or prove hardware units or site gates.
"""
from __future__ import annotations

from concurrent.futures import TimeoutError as FutureTimeoutError
from collections.abc import Mapping
import math
from numbers import Real
import time
from types import MappingProxyType
from typing import Callable

from laptop.ros_gateway.control_rpc import (
    MAX_REQUEST_ID_LENGTH,
    MAX_TASK_SCOPE_ID_LENGTH,
    validate_control_request,
)
from laptop.ros_gateway.session_owner_queue import (
    OwnerQueueError,
    OwnerQueueFull,
    SessionOwnerTaskQueue,
)
from laptop.transport.position_commands import (
    MAX_POSITION_APPROVAL_WINDOW_NS,
    PositionBatchWriteReceipt,
    PositionCommandBatch,
)
from laptop.safety.task_scope import TaskControlSnapshot, TaskScopedPositionAuthorization
from laptop.control.contracts import ABSOLUTE_JOINT_POSITION_COMMAND
from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_LIMITS,
    GRIPPER_NOMINAL_METERS_PER_RAW,
    TASK_JOINT_BOUNDS_DEG,
    validate_continuous_gripper_target,
)


POSITION_OPERATIONS = frozenset({"set_joint_target", "set_gripper"})
MAX_POSITION_RPC_VALIDITY_MS = MAX_POSITION_APPROVAL_WINDOW_NS // 1_000_000
MAX_FEEDBACK_AGE_NS = 120_000_000
MAX_CONTROL_REQUESTS_PER_SESSION = 4096


def _validate_dispatch_feedback(state: dict) -> None:
    """Require a healthy, enabled, in-envelope controller snapshot at dispatch."""
    if state.get("fake") is not False or state.get("hardware_connected") is not True:
        raise ValueError("current feedback is not from a connected real controller")
    if state.get("ctrl_mode_raw") != 1:
        raise ValueError("current controller is not in CAN_CTRL mode")
    if state.get("move_mode_raw") != 1:
        raise ValueError("current controller is not in MOVE_J mode")
    if state.get("teach_status_raw") not in (0, 2):
        raise ValueError("current controller is still in teaching mode")
    if state.get("arm_status_raw") != 0 or state.get("error_code") != 0:
        raise ValueError("current controller reports abnormal arm status or a fault")
    gripper_status = state.get("gripper_status_raw")
    if type(gripper_status) is not int or gripper_status & 0xBF:
        raise ValueError("current gripper feedback reports a fault or status flag")
    motors = state.get("motor_enabled")
    if (not isinstance(motors, (list, tuple)) or len(motors) != 6
            or any(type(enabled) is not bool or not enabled for enabled in motors)):
        raise ValueError("current controller does not report all six motors enabled")
    joints = state.get("joint_position_rad")
    if not isinstance(joints, (list, tuple)) or len(joints) != 6:
        raise ValueError("current feedback must contain six joint positions")
    for index, (value, (lower_deg, upper_deg)) in enumerate(
        zip(joints, TASK_JOINT_BOUNDS_DEG), start=1
    ):
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(float(value)):
            raise ValueError("current feedback has a non-finite joint position")
        if not math.radians(lower_deg) <= float(value) <= math.radians(upper_deg):
            raise ValueError(f"current joint_{index} is outside the configured task motion envelope")
    gripper = state.get("gripper_raw")
    if (type(gripper) is not int
            or not CONTINUOUS_GRIPPER_LIMITS.minimum_raw <= gripper <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw):
        raise ValueError("current gripper feedback is outside the tested continuous-task envelope")


def _result(session_id: str, request_id: str, *, decision: str, reason: str,
            accepted: bool = False, can_tx_calls: int | None = 0,
            delivery_status: str = "NOT_DISPATCHED") -> dict:
    return {
        "session_id": session_id,
        "request_id": request_id,
        "accepted": accepted,
        "decision": decision,
        "reason": reason[:192],
        "can_tx_calls": can_tx_calls,
        "delivery_status": delivery_status,
    }


def _validated_arguments(operation: str, arguments: object) -> MappingProxyType:
    if not isinstance(arguments, Mapping):
        raise ValueError("arguments must be a map")
    if operation == "set_joint_target":
        if set(arguments) != {"joint_position_rad"}:
            raise ValueError("joint target arguments must contain only joint_position_rad")
        values = arguments["joint_position_rad"]
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ValueError("joint_position_rad must contain six values")
        if any(isinstance(value, bool) or not isinstance(value, Real)
               or not math.isfinite(float(value)) for value in values):
            raise ValueError("joint_position_rad values must be finite numbers")
        target = tuple(float(value) for value in values)
        for index, (value, (lower_deg, upper_deg)) in enumerate(
            zip(target, TASK_JOINT_BOUNDS_DEG), start=1
        ):
            if not math.radians(lower_deg) <= value <= math.radians(upper_deg):
                raise ValueError(f"joint_{index} target is outside the configured task motion envelope")
        return MappingProxyType({"joint_position_rad": target})

    if operation == "set_gripper":
        expected = {"gripper_raw", "nominal_position_m", "nominal_effort_nm"}
        if set(arguments) != expected:
            raise ValueError("gripper arguments do not match the service contract")
        raw = arguments["gripper_raw"]
        position = arguments["nominal_position_m"]
        effort = arguments["nominal_effort_nm"]
        if (type(raw) is not int
                or not CONTINUOUS_GRIPPER_LIMITS.minimum_raw <= raw <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw):
            raise ValueError("gripper_raw is outside the tested continuous-task command envelope")
        if any(isinstance(value, bool) or not isinstance(value, Real)
               or not math.isfinite(float(value)) for value in (position, effort)):
            raise ValueError("gripper nominal values must be finite numbers")
        if abs(float(position) - raw * GRIPPER_NOMINAL_METERS_PER_RAW) > GRIPPER_NOMINAL_METERS_PER_RAW / 2:
            raise ValueError("gripper raw and nominal position disagree")
        if float(effort) * 1000 != CONTINUOUS_GRIPPER_LIMITS.effort_raw:
            raise ValueError("gripper effort differs from the continuous-task profile")
        return MappingProxyType({
            "gripper_raw": raw,
            "nominal_position_m": float(position),
            "nominal_effort_nm": float(effort),
        })

    raise ValueError("operation is not a position target")


class LocalPositionControlRpcDispatcher:
    """Validate and enqueue position requests under local task/session authority.

    ``session_authorizer`` and ``target_factory`` are trusted, in-process
    callbacks. Neither callback can be supplied through MsgPack. The factory
    signature is ``factory(session, operation=..., arguments=...,
    request_id=..., task_scope_id=..., source_state_sequence=...,
    valid_until_monotonic_ns=..., gateway_state=...)`` and must return a typed
    ``(TaskControlSnapshot, PositionCommandBatch)`` pair. It must not open a
    second CAN owner or call CAN directly.
    """

    def __init__(self, *, task_request_queue: SessionOwnerTaskQueue,
                 latest_state_provider: Callable[[], dict | None],
                 session_authorizer: Callable[[str, str], bool],
                 target_factory: Callable[..., tuple[TaskControlSnapshot, PositionCommandBatch]],
                 source_state_provider: Callable[[int, str], dict | None] | None = None,
                 monotonic_ns: Callable[[], int] = time.perf_counter_ns) -> None:
        if type(task_request_queue) is not SessionOwnerTaskQueue:
            raise TypeError("dispatcher requires the Windows session owner queue")
        if not callable(latest_state_provider) or not callable(session_authorizer):
            raise TypeError("state provider and local session authorizer are required")
        if not callable(target_factory) or not callable(monotonic_ns):
            raise TypeError("local target factory and monotonic clock are required")
        self._queue = task_request_queue
        self._latest_state_provider = latest_state_provider
        self._source_state_provider = source_state_provider
        self._session_authorizer = session_authorizer
        self._target_factory = target_factory
        self._monotonic_ns = monotonic_ns

    def _fresh_source(self, *, session_id: str, now_ns: int,
                      source_sequence: int | None = None) -> dict:
        try:
            state = self._latest_state_provider()
        except Exception as exc:
            raise ValueError(
                f"latest Windows feedback snapshot could not be read: {type(exc).__name__}"
            ) from exc
        if not isinstance(state, dict):
            raise ValueError("latest Windows feedback snapshot is unavailable or stale")
        if state.get("feedback_fresh") is not True:
            raise ValueError("latest Windows feedback snapshot is not fresh")
        latest_sequence = state.get("sequence")
        if type(latest_sequence) is not int or latest_sequence <= 0:
            raise ValueError("latest Windows feedback sequence is invalid")
        stamp = state.get("oldest_feedback_monotonic_ns")
        if type(stamp) is not int or stamp <= 0:
            raise ValueError("latest Windows feedback snapshot has no local monotonic stamp")
        age_ns = now_ns - stamp
        if age_ns < 0 or age_ns > MAX_FEEDBACK_AGE_NS:
            raise ValueError("latest Windows feedback snapshot is stale or from the future")
        _validate_dispatch_feedback(state)
        # The request's session is the active hello session. The feedback stream
        # is owned by this gateway process and sequence check binds it to this
        # request without trusting a timestamp from the remote host.
        if not session_id:
            raise ValueError("active TCP session is required")
        if state.get("session_id") not in (None, "", session_id):
            raise ValueError("latest feedback belongs to a different TCP session")
        if source_sequence is not None:
            if type(source_sequence) is not int or source_sequence <= 0:
                raise ValueError("requested source sequence must be positive")
            if source_sequence > latest_sequence:
                raise ValueError("source_state_sequence refers to a future feedback state")
            if self._source_state_provider is None:
                if source_sequence != latest_sequence or state.get("session_id") != session_id:
                    raise ValueError("requested source state is not retained in this TCP session")
                source = state
            else:
                try:
                    # WindowsReadOnlyGateway retains the public lookup shape
                    # (sequence, session_id); keep this call order aligned so
                    # a valid same-session recent state is not spuriously lost.
                    source = self._source_state_provider(source_sequence, session_id)
                except Exception as exc:
                    raise ValueError(f"requested source state lookup failed: {type(exc).__name__}") from exc
            if not isinstance(source, dict) or source.get("feedback_fresh") is not True:
                raise ValueError("requested source state is unavailable or stale")
            if type(source.get("sequence")) is not int or source["sequence"] != source_sequence:
                raise ValueError("requested source state sequence does not match the retained snapshot")
            if source.get("session_id") != session_id:
                raise ValueError("requested source state belongs to a different TCP session")
            source_stamp = source.get("oldest_feedback_monotonic_ns")
            if type(source_stamp) is not int or source_stamp <= 0:
                raise ValueError("requested source state has no local monotonic timestamp")
            source_age = now_ns - source_stamp
            if source_age < 0 or source_age > MAX_FEEDBACK_AGE_NS:
                raise ValueError("requested source state is stale or from the future")
        return dict(state)

    def _validate_requested_step(self, operation: str, arguments: MappingProxyType,
                                 current_state: dict) -> None:
        if operation == "set_joint_target":
            current = current_state.get("joint_position_rad")
            if not isinstance(current, (list, tuple)) or len(current) != 6:
                raise ValueError("current feedback must contain six joint positions")
            for index, (target, value, (lower_deg, upper_deg)) in enumerate(
                zip(arguments["joint_position_rad"], current, TASK_JOINT_BOUNDS_DEG), start=1
            ):
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError("current feedback has a non-finite joint position")
                if not math.radians(lower_deg) <= value <= math.radians(upper_deg):
                    raise ValueError(f"current joint_{index} is outside the configured task motion envelope")
                if abs(target - value) > math.radians(20.0) + 1e-12:
                    raise ValueError(f"joint_{index} target exceeds the 20 degree single-step limit")
        else:
            current_raw = current_state.get("gripper_raw")
            if type(current_raw) is not int:
                raise ValueError("current feedback has no integer gripper position")
            validate_continuous_gripper_target(
                current_raw, arguments["gripper_raw"], CONTINUOUS_GRIPPER_LIMITS.effort_raw,
            )

    def dispatch(self, payload: object, *, session_id: str,
                 used_request_ids: set[str]) -> dict:
        request_id = payload.get("request_id", "") if isinstance(payload, Mapping) else ""
        if not isinstance(request_id, str) or not request_id.isascii() or len(request_id) > MAX_REQUEST_ID_LENGTH:
            request_id = ""
        if not isinstance(used_request_ids, set):
            return _result(session_id, request_id, decision="REJECTED_INVALID_LOCAL_REPLAY_WINDOW",
                           reason="gateway request replay window is unavailable")
        try:
            request_id, operation = validate_control_request(payload, session_id=session_id)
        except (TypeError, ValueError) as exc:
            return _result(session_id, request_id, decision="REJECTED_INVALID_CONTROL_REQUEST",
                           reason=str(exc))

        if request_id in used_request_ids:
            return _result(session_id, request_id, decision="REJECTED_REPLAYED_CONTROL_REQUEST",
                           reason="request_id has already been used in this gateway session")
        if len(used_request_ids) >= MAX_CONTROL_REQUESTS_PER_SESSION:
            return _result(session_id, request_id, decision="REJECTED_REQUEST_WINDOW_FULL",
                           reason="control request replay window is full; reconnect to start a new session")
        used_request_ids.add(request_id)

        if operation not in POSITION_OPERATIONS:
            return _result(session_id, request_id,
                           decision="REJECTED_LOCAL_MANAGEMENT_AUTHORIZATION_REQUIRED",
                           reason="mode, motor, and home operations require a separate Windows-local management authorization")
        scope_id = payload["task_scope_id"]
        if not scope_id or len(scope_id) > MAX_TASK_SCOPE_ID_LENGTH:
            return _result(session_id, request_id, decision="REJECTED_INVALID_TASK_SCOPE",
                           reason="position request requires a bounded active task scope")
        if not 1 <= payload["validity_ms"] <= MAX_POSITION_RPC_VALIDITY_MS:
            return _result(session_id, request_id, decision="REJECTED_INVALID_POSITION_VALIDITY",
                           reason="position request validity must be within 1..250 ms")
        if self._queue.active_task_scope_id != scope_id:
            return _result(session_id, request_id, decision="REJECTED_TASK_SCOPE_NOT_ACTIVE",
                           reason="position request does not match a locally active task scope")
        try:
            authorized = self._session_authorizer(session_id, scope_id)
        except Exception as exc:
            return _result(session_id, request_id, decision="REJECTED_LOCAL_SESSION_AUTHORIZATION",
                           reason=f"local session authorization failed: {type(exc).__name__}")
        if authorized is not True:
            return _result(session_id, request_id, decision="REJECTED_LOCAL_SESSION_AUTHORIZATION",
                           reason="this TCP session is not locally authorized for the active task")
        try:
            arguments = _validated_arguments(operation, payload["arguments"])
        except (TypeError, ValueError) as exc:
            return _result(session_id, request_id, decision="REJECTED_INVALID_TARGET_ARGUMENTS",
                           reason=str(exc))

        now_ns = self._monotonic_ns()
        deadline_ns = now_ns + payload["validity_ms"] * 1_000_000
        try:
            source = self._fresh_source(
                session_id=session_id,
                source_sequence=payload["source_state_sequence"],
                now_ns=now_ns,
            )
        except (TypeError, ValueError) as exc:
            return _result(session_id, request_id,
                           decision="REJECTED_STALE_OR_MISMATCHED_SOURCE_STATE",
                           reason=str(exc))

        dispatch_source_sequence: list[int] = []

        def make_typed_target(session):
            current_ns = self._monotonic_ns()
            if current_ns >= deadline_ns:
                raise TimeoutError("position request expired before owner-thread mapping")
            current_source = self._fresh_source(
                session_id=session_id,
                now_ns=current_ns,
            )
            self._validate_requested_step(operation, arguments, current_source)
            if self._session_authorizer(session_id, scope_id) is not True:
                raise PermissionError("local session authorization was revoked before owner dispatch")
            if self._queue.active_task_scope_id != scope_id:
                raise PermissionError("task scope was revoked before owner dispatch")
            dispatch_source_sequence.append(current_source["sequence"])
            snapshot, batch = self._target_factory(
                session,
                operation=operation,
                arguments=arguments,
                request_id=request_id,
                task_scope_id=scope_id,
                source_state_sequence=current_source["sequence"],
                valid_until_monotonic_ns=deadline_ns,
                gateway_state=MappingProxyType(current_source),
            )
            if type(snapshot) is not TaskControlSnapshot:
                raise TypeError("local mapper must return a typed task feedback snapshot")
            if type(batch) is not PositionCommandBatch:
                raise TypeError("local mapper must return a typed SDK position batch")
            proposal = batch.proposal
            active_authorization = getattr(
                getattr(session, "bridge", None),
                "active_task_position_authorization",
                None,
            )
            if type(active_authorization) is not TaskScopedPositionAuthorization:
                raise PermissionError("PiperSession has no active local task authorization")
            if (active_authorization.task_scope_id != scope_id
                    or proposal.model_profile != active_authorization.model_profile
                    or proposal.checkpoint_id != active_authorization.checkpoint_id):
                raise PermissionError("typed proposal does not match the active local task authorization")
            if (proposal.source_request_id != request_id
                    or proposal.task_scope_id != scope_id
                    or proposal.source_state_sequence != snapshot.robot_state.sequence
                    or proposal.action_semantics != ABSOLUTE_JOINT_POSITION_COMMAND
                    or proposal.created_monotonic_ns >= deadline_ns
                    or proposal.valid_until_monotonic_ns > deadline_ns):
                raise ValueError("local typed proposal does not bind the request and active task scope")
            if operation == "set_joint_target":
                if tuple(proposal.target[:6]) != arguments["joint_position_rad"]:
                    raise ValueError("local typed proposal does not preserve the requested joint target")
                if proposal.target[6] != snapshot.robot_state.gripper_position_m:
                    raise ValueError("joint-only request must hold the latest gripper feedback target")
            if operation == "set_gripper":
                if proposal.target[6] != arguments["nominal_position_m"]:
                    raise ValueError("local typed proposal does not preserve the requested gripper position")
                if tuple(proposal.target[:6]) != snapshot.robot_state.joint_position_rad:
                    raise ValueError("gripper-only request must hold all six latest joint targets")
            return snapshot, batch

        try:
            future = self._queue.submit_target_factory(
                request_id=request_id,
                task_scope_id=scope_id,
                valid_until_monotonic_ns=deadline_ns,
                factory=make_typed_target,
            )
        except OwnerQueueFull as exc:
            return _result(session_id, request_id, decision="REJECTED_CONTROL_QUEUE_FULL",
                           reason=str(exc))
        except (OwnerQueueError, TypeError, ValueError) as exc:
            return _result(session_id, request_id, decision="REJECTED_OWNER_QUEUE",
                           reason=str(exc))

        remaining_s = max(0.0, (deadline_ns - self._monotonic_ns()) / 1_000_000_000)
        try:
            receipt = future.result(timeout=remaining_s)
        except FutureTimeoutError:
            return _result(session_id, request_id, decision="REJECTED_OWNER_DISPATCH_TIMEOUT",
                           reason="owner-thread deadline elapsed; adapter delivery cannot be inferred",
                           can_tx_calls=None, delivery_status="UNKNOWN")
        except Exception as exc:
            return _result(session_id, request_id, decision="REJECTED_OWNER_DISPATCH_FAILED",
                           reason=f"owner-thread operation failed: {type(exc).__name__}: {exc}",
                           can_tx_calls=None, delivery_status="UNKNOWN")

        if type(receipt) is not PositionBatchWriteReceipt:
            return _result(session_id, request_id, decision="REJECTED_INVALID_ADAPTER_RECEIPT",
                           reason="owner thread returned no typed four-frame adapter receipt",
                           can_tx_calls=None, delivery_status="UNKNOWN")
        return {
            "session_id": session_id,
            "request_id": request_id,
            "accepted": True,
            "decision": "ADAPTER_ACCEPTED_POSITION_BATCH",
            "reason": "SafetyBridge returned a complete adapter receipt; this does not prove robot motion or task success",
            "can_tx_calls": len(receipt.writes),
            "delivery_status": "ADAPTER_ACCEPTED",
            "source_gateway_state_sequence": payload["source_state_sequence"],
            "request_source_state_sequence": payload["source_state_sequence"],
            "dispatch_gateway_state_sequence": (
                dispatch_source_sequence[-1] if dispatch_source_sequence else None
            ),
            "source_task_state_sequence": receipt.source_state_sequence,
            "adapter_frame_ids": [write.arbitration_id for write in receipt.writes],
            "adapter_accept_monotonic_ns": [
                write.adapter_accepted_monotonic_ns for write in receipt.writes
            ],
        }
