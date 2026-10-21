"""Typed local manual P2 requests using the existing raw commissioning path.

This path does not grant model-task permission or attest physical meter units.
The existing transport sends CAN/MOVE_J/5% plus one bounded position batch.
"""
from dataclasses import dataclass
import time
import threading
from concurrent.futures import TimeoutError

from laptop.piper.unit_conversion import rad_to_sdk_joint, m_to_sdk_gripper
from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_LIMITS,
    MANUAL_GAMEPAD_GRIPPER_LIMITS,
    SINGLE_STEP_GRIPPER_LIMITS,
    validate_continuous_gripper_target,
)
from laptop.ros_gateway.control_rpc import validate_control_request
from laptop.ros_gateway.control_rpc_dispatcher import _validated_arguments, _result

from laptop.transport.bounded_trial import BoundedTrialBatch


@dataclass(frozen=True)
class ManualCommissioningTarget:
    request_id: str
    operation: str
    target_joint_raw: tuple[int, ...]
    target_gripper_raw: int
    valid_until_monotonic_ns: int
    operator_approved: bool
    physical_estop_ready: bool
    low_speed_confirmed: bool
    motion_envelope_cleared: bool
    gripper_step_limit_raw: int = SINGLE_STEP_GRIPPER_LIMITS.maximum_step_raw
    joint_step_limit_raw: int = 1000
    gripper_profile: str = "single_step_commissioning"
    controller_speed_percent: int = 5

    def __post_init__(self):
        if (type(self.controller_speed_percent) is not int
                or self.controller_speed_percent not in (5,10,15,30,50)
                or (self.gripper_profile != 'manual_gamepad' and self.controller_speed_percent != 5)):
            raise ValueError("speed override requires manual gamepad profile and a supported gear")
        step_cap = (MANUAL_GAMEPAD_GRIPPER_LIMITS.maximum_step_raw
                    if self.gripper_profile == 'manual_gamepad' else 10000)
        if type(self.gripper_step_limit_raw) is not int or not 1 <= self.gripper_step_limit_raw <= step_cap:
            raise ValueError("locally approved gripper step exceeds selected profile")
        if type(self.joint_step_limit_raw) is not int or not 1 <= self.joint_step_limit_raw <= 20_000:
            raise ValueError("locally approved joint step must be 1..20000 raw")
        if not isinstance(self.request_id, str) or not self.request_id.isascii() or not 1 <= len(self.request_id) <= 128:
            raise ValueError("bounded ASCII request ID required")
        if (type(self.target_joint_raw) is not tuple or len(self.target_joint_raw) != 6
                or any(type(v) is not int for v in self.target_joint_raw)):
            raise ValueError("six immutable integer SDK joint targets required")
        if self.operation not in {"set_joint_target", "set_gripper"}:
            raise ValueError("manual target operation must be joint or gripper position")
        if self.gripper_profile == "single_step_commissioning":
            gripper_limits = SINGLE_STEP_GRIPPER_LIMITS
            if self.joint_step_limit_raw > 1_000:
                raise ValueError("single-step commissioning joint limit is 1 degree")
        elif self.gripper_profile in ("continuous_task", "manual_gamepad"):
            gripper_limits = (MANUAL_GAMEPAD_GRIPPER_LIMITS if self.gripper_profile == "manual_gamepad"
                              else CONTINUOUS_GRIPPER_LIMITS)
            if self.joint_step_limit_raw > 5_000:
                raise ValueError("manual gamepad joint step cannot exceed 5 degrees")
        else:
            raise ValueError("unsupported manual gripper profile")
        joints_only = (self.gripper_profile in ("continuous_task", "manual_gamepad")
                       and self.operation == "set_joint_target")
        if type(self.target_gripper_raw) is not int or (not joints_only and not (
            gripper_limits.minimum_raw <= self.target_gripper_raw <= gripper_limits.maximum_raw
        )):
            raise ValueError("manual gripper target outside the selected tested envelope")
        if type(self.valid_until_monotonic_ns) is not int:
            raise ValueError("Windows monotonic deadline required")
        for name in ("operator_approved", "physical_estop_ready", "low_speed_confirmed", "motion_envelope_cleared"):
            if getattr(self, name) is not True:
                raise PermissionError(f"current local {name} confirmation required")

    def execute(self, session):
        """Run once on the existing owner; no new CAN connection or retries."""
        now = time.perf_counter_ns()
        if not 0 < self.valid_until_monotonic_ns-now <= 250_000_000:
            raise PermissionError("manual target expired or deadline too long")
        if self.gripper_profile in ("continuous_task", "manual_gamepad"):
            from laptop.transport.manual_gamepad import ManualGamepadBatch, bounded_hold_joints

            snapshot = session.create_manual_gamepad_snapshot(
                physical_estop_ready=self.physical_estop_ready,
                low_speed_confirmed=self.low_speed_confirmed,
                motion_envelope_cleared=self.motion_envelope_cleared,
            )
            target_joints = (
                self.target_joint_raw if self.operation == "set_joint_target"
                else bounded_hold_joints(snapshot.joint_raw_0p001deg)
            )
            target_gripper = (
                snapshot.gripper_raw if self.operation == "set_joint_target"
                else self.target_gripper_raw
            )
            batch = ManualGamepadBatch(
                request_id=self.request_id,
                source_sequence=snapshot.sequence,
                current_joints=snapshot.joint_raw_0p001deg,
                target_joints=target_joints,
                current_gripper=snapshot.gripper_raw,
                target_gripper=target_gripper,
                joints_only=self.operation == "set_joint_target",
                controller_speed_percent=self.controller_speed_percent,
                expires_ns=self.valid_until_monotonic_ns,
                joint_step_limit_raw=self.joint_step_limit_raw,
                gripper_step_limit_raw=self.gripper_step_limit_raw,
                gripper_effort_raw=CONTINUOUS_GRIPPER_LIMITS.effort_raw,
                operator_approved=self.operator_approved,
                physical_estop_ready=self.physical_estop_ready,
                low_speed_confirmed=self.low_speed_confirmed,
                motion_envelope_cleared=self.motion_envelope_cleared,
            )
            return session.send_manual_gamepad_target(snapshot=snapshot, batch=batch)

        snapshot = session.commissioning_snapshot(
            physical_estop_ready=self.physical_estop_ready,
            low_speed_confirmed=self.low_speed_confirmed,
            motion_envelope_cleared=self.motion_envelope_cleared,
        )
        state = session.state(max_age_ms=120)
        if not state["complete_fresh"]:
            raise PermissionError("complete fresh manual feedback required")
        current_joints = tuple(state["joint_raw_0p001deg"])
        current_gripper = state["gripper"]["position_raw"]
        gripper_limits = (
            CONTINUOUS_GRIPPER_LIMITS if self.gripper_profile == "continuous_task"
            else SINGLE_STEP_GRIPPER_LIMITS
        )
        target_joints = (
            self.target_joint_raw if self.operation == "set_joint_target" else current_joints
        )
        target_gripper = (
            current_gripper if self.operation == "set_joint_target" else self.target_gripper_raw
        )
        if self.gripper_profile == "continuous_task":
            validate_continuous_gripper_target(
                current_gripper, target_gripper,
                CONTINUOUS_GRIPPER_LIMITS.effort_raw,
            )
            if self.gripper_step_limit_raw != CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw:
                raise PermissionError("P2 stream must use the shared continuous gripper step limit")
        elif abs(target_gripper-current_gripper) > self.gripper_step_limit_raw:
            raise PermissionError("manual gripper target exceeds its locally approved one-step limit")
        batch = BoundedTrialBatch(
            source_sequence=snapshot.sequence,
            current_joints=snapshot.joint_raw_0p001deg,
            target_joints=target_joints,
            current_gripper=current_gripper,
            target_gripper=target_gripper,
            prediction_request_id=self.request_id,
            expires_ns=self.valid_until_monotonic_ns,
            joint_step_limit_raw=self.joint_step_limit_raw,
            gripper_step_limit_raw=self.gripper_step_limit_raw,
            gripper_profile=self.gripper_profile,
            gripper_effort_raw=gripper_limits.effort_raw,
        )
        return session.send_bounded_model_trial(
            snapshot=snapshot, batch=batch, operator_approved=self.operator_approved,
        )


class ManualCommissioningDispatcher:
    """One locally approved exact target for one TCP session; never a task grant.

The local caller must confirm the target and site conditions before construction.
Network requests cannot create or replace this approval. A disconnect revokes it.
"""

    def __init__(self, *, queue, latest_state_provider, session_id, task_scope_id,
                 operation, approved_arguments, valid_until_monotonic_ns,
                 operator_approved, physical_estop_ready, low_speed_confirmed,
                 motion_envelope_cleared, source_state_provider=None,
                 gripper_step_limit_raw=SINGLE_STEP_GRIPPER_LIMITS.maximum_step_raw):
        if operation not in {"set_joint_target", "set_gripper"}:
            raise ValueError("manual position operation required")
        self.arguments = _validated_arguments(operation, approved_arguments)
        self.operation = operation
        if type(gripper_step_limit_raw) is not int or not 1 <= gripper_step_limit_raw <= 10000:
            raise ValueError("locally approved gripper step must be 1..10000 raw")
        self.gripper_step_limit_raw = gripper_step_limit_raw
        self.session_id = session_id
        self.scope_id = task_scope_id
        if any(not isinstance(v,str) or not v or not v.isascii() or len(v)>128
               for v in (session_id,task_scope_id)):
            raise ValueError("bounded local session and scope IDs required")
        now = time.perf_counter_ns()
        if type(valid_until_monotonic_ns) is not int or not 0<valid_until_monotonic_ns-now<=120_000_000_000:
            raise ValueError("manual local approval must expire within 120 seconds")
        self.expiry = valid_until_monotonic_ns
        self.site = dict(operator_approved=operator_approved,
                        physical_estop_ready=physical_estop_ready,
                        low_speed_confirmed=low_speed_confirmed,
                        motion_envelope_cleared=motion_envelope_cleared)
        if any(v is not True for v in self.site.values()):
            raise PermissionError("current manual operator and site approval required")
        self.queue = queue
        self.latest = latest_state_provider
        self.source_state_provider = source_state_provider
        self.lock = threading.Lock()
        self.used = False

    def revoke_session(self, session_id):
        if session_id == self.session_id:
            with self.lock:
                self.used = True

    def dispatch(self, payload, *, session_id, used_request_ids):
        queued = False
        request_id = payload.get("request_id", "") if isinstance(payload,dict) else ""
        def rejected(reason):
            return _result(session_id, request_id, decision="REJECTED_MANUAL_COMMISSIONING", reason=reason)
        try:
            validate_control_request(payload, session_id=session_id)
            request = payload
            arguments = _validated_arguments(request["operation"], request["arguments"])
            with self.lock:
                if self.used or time.perf_counter_ns()>=self.expiry:
                    return rejected("manual approval expired, revoked, or already consumed")
                if (session_id != self.session_id or request["session_id"]!=self.session_id
                        or request["task_scope_id"]!=self.scope_id
                        or request["operation"]!=self.operation or arguments!=self.arguments):
                    return rejected("request differs from exact local manual approval")
                if request_id in used_request_ids:
                    return rejected("request ID replay")
                latest = self.latest()
                state = (self.source_state_provider(session_id, request["source_state_sequence"])
                         if self.source_state_provider is not None else latest)
                now_ns = time.perf_counter_ns()
                source_seq = request["source_state_sequence"]
                if (not latest or not latest.get("feedback_fresh")
                        or not 0 <= now_ns-latest.get("gateway_snapshot_monotonic_ns", 0) <= 120_000_000
                        or type(latest.get("sequence")) is not int
                        or not state or not state.get("feedback_fresh")
                        or type(state.get("sequence")) is not int
                        or state["sequence"] != source_seq
                        or source_seq > latest["sequence"]
                        or state.get("session_id") not in (None, "", session_id)
                        or not 0 <= now_ns-state.get("gateway_snapshot_monotonic_ns", 0) <= 120_000_000):
                    source_age_ms = (None if not state else
                        (now_ns-state.get("gateway_snapshot_monotonic_ns", 0))/1e6)
                    return rejected(
                        "manual source is stale, future, cross-session, or unavailable: "
                        f"requested_sequence={request['source_state_sequence']}, "
                        f"latest_sequence={None if not latest else latest['sequence']}, "
                        f"source_available={state is not None}, source_age_ms={source_age_ms}")
                validity = request["validity_ms"]
                if type(validity) is not int or not 1<=validity<=250:
                    return rejected("manual request validity must be 1..250 ms")
                joints = tuple(int(rad_to_sdk_joint(v)) for v in state["joint_position_rad"])
                gripper = state["gripper_raw"]
                if self.operation=="set_joint_target":
                    joints = tuple(int(rad_to_sdk_joint(v)) for v in arguments["joint_position_rad"])
                else:
                    if int(m_to_sdk_gripper(arguments["nominal_position_m"]))!=arguments["gripper_raw"]:
                        return rejected("gripper raw and nominal target disagree")
                    if arguments["nominal_effort_nm"] != 0:
                        return rejected("reused bounded raw contract uses effort=0")
                    gripper = arguments["gripper_raw"]
                deadline = min(self.expiry,time.perf_counter_ns()+validity*1_000_000)
                target = ManualCommissioningTarget(request_id, self.operation, joints, gripper, deadline,
                    gripper_step_limit_raw=self.gripper_step_limit_raw, **self.site)
                self.used = True
                used_request_ids.add(request_id)
                future = self.queue.submit_manual_commissioning(target)
                queued = True
            writes = future.result(timeout=max(0,(deadline-time.perf_counter_ns())/1e9))
            result = _result(session_id,request_id,decision="ADAPTER_ACCEPTED_MANUAL_RAW_BATCH",
                             reason="nominal SDK commissioning only; physical tracking not verified",
                             accepted=True,can_tx_calls=len(writes),delivery_status="ADAPTER_ACCEPTED")
            result["adapter_frame_ids"] = [w.arbitration_id for w in writes]
            result["physical_motion_verified"] = False
            result["request_source_state_sequence"] = request["source_state_sequence"]
            result["dispatch_gateway_state_sequence"] = latest["sequence"]
            return result
        except TimeoutError:
            return _result(session_id,request_id,decision="MANUAL_DELIVERY_UNKNOWN",
                           reason="owner completion unknown; no retry",can_tx_calls=None,delivery_status="UNKNOWN")
        except Exception as exc:
            if queued:
                return _result(session_id,request_id,decision="MANUAL_OWNER_FAILED_DELIVERY_UNCONFIRMED",
                               reason=f"{type(exc).__name__}: {exc}",can_tx_calls=None,delivery_status="UNKNOWN")
            return rejected(f"{type(exc).__name__}: {exc}")


class ManualGamepadTeleopDispatcher:
    """Time-limited operator gamepad scope over the existing P2 owner queue.

    This is a human teleoperation path, not a model/P12 authorization. Every
    request still uses same-session fresh feedback, the shared joint envelope,
    a 5-degree maximum joint step, and the tested raw gripper step envelope.
    """

    MAX_JOINT_STEP_RAW = 5_000
    MAX_SESSION_NS = 120_000_000_000

    def __init__(self, *, queue, latest_state_provider, session_id, task_scope_id,
                 valid_until_monotonic_ns, operator_approved,
                 physical_estop_ready, low_speed_confirmed,
                 motion_envelope_cleared, source_state_provider,
                 joint_step_limit_raw=5_000,
                 gripper_step_limit_raw=CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw):
        from laptop.ros_gateway.session_owner_queue import SessionOwnerTaskQueue

        if type(queue) is not SessionOwnerTaskQueue:
            raise TypeError("gamepad teleop requires the existing session owner queue")
        if not callable(latest_state_provider) or not callable(source_state_provider):
            raise TypeError("latest and source state providers are required")
        for value, name in ((session_id, "session_id"), (task_scope_id, "task_scope_id")):
            if (not isinstance(value, str) or not value or not value.isascii()
                    or len(value) > 128):
                raise ValueError(f"bounded local {name} required")
        now = time.perf_counter_ns()
        if (type(valid_until_monotonic_ns) is not int
                or not 0 < valid_until_monotonic_ns - now <= self.MAX_SESSION_NS):
            raise ValueError("gamepad approval must expire within 120 seconds")
        if type(joint_step_limit_raw) is not int or not 1 <= joint_step_limit_raw <= self.MAX_JOINT_STEP_RAW:
            raise ValueError("gamepad joint step must be within 1..5 degrees")
        if (type(gripper_step_limit_raw) is not int
                or not 1 <= gripper_step_limit_raw <= CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw):
            raise ValueError("gamepad gripper step exceeds the continuous-task profile")
        confirmations = (operator_approved, physical_estop_ready,
                         low_speed_confirmed, motion_envelope_cleared)
        if any(value is not True for value in confirmations):
            raise PermissionError("current operator, E-stop, low-speed and clear-envelope confirmations required")

        self.queue = queue
        self.latest = latest_state_provider
        self.source_state_provider = source_state_provider
        self.session_id = session_id
        self.scope_id = task_scope_id
        self.expiry = valid_until_monotonic_ns
        self.joint_step_limit_raw = joint_step_limit_raw
        self.gripper_step_limit_raw = gripper_step_limit_raw
        self.lock = threading.Lock()
        self.revoked = False
        self.used_source_sequences = set()

    def revoke_session(self, session_id):
        if session_id == self.session_id:
            with self.lock:
                self.revoked = True

    def dispatch(self, payload, *, session_id, used_request_ids):
        queued = False
        request_id = payload.get("request_id", "") if isinstance(payload, dict) else ""

        def rejected(reason):
            return _result(session_id, request_id,
                           decision="REJECTED_MANUAL_GAMEPAD", reason=reason)

        try:
            validate_control_request(payload, session_id=session_id)
            operation = payload["operation"]
            if operation not in {"set_joint_target", "set_gripper"}:
                return rejected("gamepad scope accepts position targets only")
            arguments = _validated_arguments(operation, payload["arguments"])
            with self.lock:
                now_ns = time.perf_counter_ns()
                if self.revoked or now_ns >= self.expiry:
                    return rejected("gamepad approval expired or revoked")
                if (session_id != self.session_id
                        or payload["task_scope_id"] != self.scope_id):
                    return rejected("request differs from the locally approved gamepad scope")
                if request_id in used_request_ids:
                    return rejected("request ID replay")

                latest = self.latest()
                source = self.source_state_provider(
                    session_id, payload["source_state_sequence"]
                )
                now_ns = time.perf_counter_ns()
                for state, label in ((latest, "latest"), (source, "source")):
                    if (not state or state.get("feedback_fresh") is not True
                            or type(state.get("sequence")) is not int
                            or state.get("session_id") not in (None, "", session_id)
                            or type(state.get("gateway_snapshot_monotonic_ns")) is not int
                            or not 0 <= now_ns - state["gateway_snapshot_monotonic_ns"] <= 120_000_000):
                        return rejected(f"{label} feedback is stale, incomplete, or cross-session")
                if (source["sequence"] != payload["source_state_sequence"]
                        or source["sequence"] > latest["sequence"]):
                    return rejected("source feedback sequence is not retained in this session")
                if source["sequence"] in self.used_source_sequences:
                    return rejected("gamepad source feedback sequence was already used")
                if not 1 <= payload["validity_ms"] <= 250:
                    return rejected("gamepad request validity must be 1..250 ms")
                if (latest.get("ctrl_mode_raw") != 1 or latest.get("move_mode_raw") != 1
                        or latest.get("arm_status_raw") != 0 or latest.get("error_code") != 0
                        or latest.get("motor_enabled") != [True] * 6
                        or source.get("ctrl_mode_raw") != 1 or source.get("move_mode_raw") != 1
                        or source.get("arm_status_raw") != 0 or source.get("error_code") != 0
                        or source.get("motor_enabled") != [True] * 6):
                    return rejected("manual gamepad requires healthy CAN_CTRL/MOVE_J and six enabled motors")
                if (latest.get("gripper_status_raw", 0) & 0xBF
                        or source.get("gripper_status_raw", 0) & 0xBF):
                    return rejected("manual gamepad gripper feedback reports a fault")
                if len(used_request_ids) >= 1200:
                    return rejected("manual gamepad session reached its 1200-target budget")

                current_joints = tuple(
                    int(rad_to_sdk_joint(value))
                    for value in source["joint_position_rad"]
                )
                current_gripper = source["gripper_raw"]
                if (len(current_joints) != 6 or type(current_gripper) is not int
                        or not CONTINUOUS_GRIPPER_LIMITS.minimum_raw
                        <= current_gripper <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw):
                    return rejected("source feedback is outside the tested P2 command envelope")

                if operation == "set_joint_target":
                    target_joints = tuple(
                        int(rad_to_sdk_joint(value))
                        for value in arguments["joint_position_rad"]
                    )
                    target_gripper = current_gripper
                else:
                    target_joints = current_joints
                    if int(m_to_sdk_gripper(arguments["nominal_position_m"])) != arguments["gripper_raw"]:
                        return rejected("gripper raw and nominal target disagree")
                    target_gripper = arguments["gripper_raw"]

                if any(abs(target - current) > self.joint_step_limit_raw
                       for target, current in zip(target_joints, current_joints)):
                    return rejected("gamepad target exceeds the 5-degree per-update limit")
                if abs(target_gripper - current_gripper) > self.gripper_step_limit_raw:
                    return rejected("gamepad gripper target exceeds its per-update limit")
                deadline = min(
                    self.expiry,
                    time.perf_counter_ns() + payload["validity_ms"] * 1_000_000,
                )
                target = ManualCommissioningTarget(
                    request_id=request_id,
                    operation=operation,
                    target_joint_raw=target_joints,
                    target_gripper_raw=target_gripper,
                    valid_until_monotonic_ns=deadline,
                    operator_approved=True,
                    physical_estop_ready=True,
                    low_speed_confirmed=True,
                    motion_envelope_cleared=True,
                    gripper_step_limit_raw=self.gripper_step_limit_raw,
                    joint_step_limit_raw=self.joint_step_limit_raw,
                    gripper_profile="continuous_task",
                )
                used_request_ids.add(request_id)
                self.used_source_sequences.add(source["sequence"])
                future = self.queue.submit_manual_commissioning(target)
                queued = True

            writes = future.result(timeout=max(0, (deadline-time.perf_counter_ns())/1e9))
            result = _result(
                session_id, request_id,
                decision="ADAPTER_ACCEPTED_MANUAL_GAMEPAD_BATCH",
                reason="P2 operator gamepad target accepted; physical tracking remains unverified",
                accepted=True, can_tx_calls=len(writes), delivery_status="ADAPTER_ACCEPTED",
            )
            result["adapter_frame_ids"] = [write.arbitration_id for write in writes]
            result["physical_motion_verified"] = False
            result["task_scope_id"] = self.scope_id
            result["source_state_sequence"] = payload["source_state_sequence"]
            return result
        except TimeoutError:
            with self.lock:
                self.revoked = True
            return _result(session_id, request_id,
                           decision="MANUAL_GAMEPAD_DELIVERY_UNKNOWN",
                           reason="owner completion unknown; approval revoked and no retry",
                           can_tx_calls=None, delivery_status="UNKNOWN")
        except Exception as exc:
            if queued:
                with self.lock:
                    self.revoked = True
                return _result(session_id, request_id,
                               decision="MANUAL_GAMEPAD_DELIVERY_UNKNOWN",
                               reason=f"{type(exc).__name__}: {exc}; approval revoked, no retry",
                               can_tx_calls=None, delivery_status="UNKNOWN")
            return rejected(f"{type(exc).__name__}: {exc}")
