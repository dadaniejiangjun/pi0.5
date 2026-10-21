"""Single-owner native manual services; no automatic reset or motor enable."""
from collections import deque
import json
import math
import time

import rclpy
from std_msgs.msg import String
from pi05_piper_interfaces.msg import CommandReceipt
from pi05_piper_interfaces.srv import HomeArm, SetJointTarget, SetGripper
from laptop.transport.socketcan_manual import SocketCanManualAdapter
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.ros_gateway.manual_commissioning import ManualCommissioningTarget
from laptop.transport.manual_gamepad import ManualGamepadBatch
from laptop.piper.unit_conversion import rad_to_sdk_joint, GRIPPER_NOMINAL_METERS_PER_RAW
from laptop.piper.control_profile import MANUAL_GAMEPAD_GRIPPER_LIMITS as CONTINUOUS_GRIPPER_LIMITS
from laptop.control.feedback_acceptance import (
    MANUAL_FEEDBACK_LIMITS,
    ZERO_RETURN_FINAL_TOLERANCE_RAW,
    ZERO_RETURN_SEGMENT_TOLERANCE_RAW,
)
from laptop.robot_client.bounded_zero_return import (
    ZERO_RETURN_MAX_SEGMENTS,
    ZERO_RETURN_SEGMENT_RAW,
    zero_target,
)
from laptop.transport.position_commands import TASK_JOINT_BOUNDS_RAW
from laptop.robot_client.native_piper_tool import ensure_can_control
from .native_can_state import NativeCanState
from .control_source_core import validate_bridge_control_source


def selected_controller_speed(event, session_id, now_ns):
    if (event.get('schema') != 'piper_gamepad_target_event_v1'
            or event.get('status') != 'TARGET_RATE_PREVIEW'
            or event.get('source_session_id') != session_id):
        raise ValueError('speed event is not from the active feedback session')
    stamp = event.get('local_monotonic_ns')
    if type(stamp) is not int or not 0 <= now_ns-stamp <= 1_500_000_000:
        raise ValueError('speed event is stale')
    speed = {30.0:15, 60.0:30, 100.0:50}.get(event.get('selected_joint_rate_deg_s'))
    if speed is None or event.get('controller_speed_percent') != speed:
        raise ValueError('unsupported manual speed gear')
    return speed


class NativeManualGamepad(NativeCanState):
    def __init__(self):
        self.sources = deque(maxlen=32)
        self.used_requests = set()
        self.manual_enabled = False
        self.homing = False
        self.event_file = None
        self.controller_speed_percent = 15
        super().__init__(bridge_factory=self.make_bridge)
        self.events = self.create_publisher(String, '/gamepad/command_send_events', 100)
        self.receipts = self.create_publisher(CommandReceipt, '/piper/command_receipt', 100)
        self.create_service(SetJointTarget, '/piper/control/set_joint_target', self.joint)
        self.create_service(SetGripper, '/piper/control/set_gripper', self.gripper)
        self.create_service(HomeArm, '/piper/control/home_arm', self.home_arm)
        self.speed_subscription = self.create_subscription(
            String, '/gamepad/target_events', self.on_speed_event, 50)
        self.get_logger().info('Native manual services ready; TX enabled=' + str(self.manual_enabled))

    def on_speed_event(self, msg):
        try:
            speed = selected_controller_speed(json.loads(msg.data), self.session_id, time.perf_counter_ns())
        except (ValueError, TypeError, AttributeError):
            return
        if self.manual_enabled and time.perf_counter_ns() < self.deadline:
            if self.controller_speed_percent != speed:
                self.event(dict(event='CONTROLLER_SPEED_SELECTED', speed_percent=speed,
                                can_tx_calls=0, applied_on_next_command=True))
            self.controller_speed_percent = speed

    def make_bridge(self, node, interface):
        for name, value in [('enable_manual_tx', False), ('task_scope_id', ''),
                            ('duration_s', 600), ('operator_approved', False),
                            ('physical_estop_ready', False), ('low_speed_confirmed', False),
                            ('motion_envelope_cleared', False)]:
            self.declare_parameter(name, value)
        self.scope = str(self.get_parameter('task_scope_id').value)
        duration = self.get_parameter('duration_s').value
        if type(duration) is not int or not 1 <= duration <= 3600:
            raise ValueError('manual scope duration must be 1..3600 seconds')
        self.request_budget = duration * 20
        self.manual_enabled = self.get_parameter('enable_manual_tx').value is True
        self.confirmations = {key: self.get_parameter(key).value for key in (
            'operator_approved', 'physical_estop_ready', 'low_speed_confirmed', 'motion_envelope_cleared')}
        if self.manual_enabled and (not self.scope.strip() or len(self.scope) > 128
                                    or not all(v is True for v in self.confirmations.values())):
            raise PermissionError('explicit scope and all onsite confirmations required')
        self.deadline = time.perf_counter_ns() + int(duration * 1e9)
        path = str(self.get_parameter('log_path').value) + '.sends.jsonl'
        self.event_file = open(path, 'x', encoding='utf-8')
        self.manual_adapter = SocketCanManualAdapter(interface,
            transmit_enabled=self.manual_enabled, event_sink=self.event)
        transport = GsUsbPiperCanTransport(adapter_factory=lambda: self.manual_adapter,
            manual_gamepad_request_budget=self.request_budget)
        return SafetyBridge(mode=(RuntimeMode.SUPERVISED_MANUAL_GAMEPAD if self.manual_enabled else RuntimeMode.POLICY_REJECT_ONLY),
            transport=transport, limits=UNVERIFIED_ANALYSIS_LIMITS,
            supervised_manual_gamepad_tx_enabled=self.manual_enabled)

    def event(self, record):
        record.setdefault('monotonic_ns', time.perf_counter_ns())
        encoded = json.dumps(record, ensure_ascii=False)
        self.event_file.write(encoded + '\n')
        self.event_file.flush()
        if hasattr(self, 'events'):
            self.events.publish(String(data=encoded))

    def on_state(self, wire):
        self.sources.append((self.session_id, self.sequence, wire['host_monotonic_ns'], 0, True))

    def joint(self, request, response):
        return self.dispatch(request, response, 'set_joint_target')

    def gripper(self, request, response):
        return self.dispatch(request, response, 'set_gripper')

    def home_arm(self, request, response):
        """Send one zero target, then monitor arrival on the existing CAN owner."""
        response.request_id = request.request_id
        response.accepted = False
        response.decision = 'REJECTED_SAFE_HOME'
        steps = []
        try:
            now = time.perf_counter_ns()
            if self.homing:
                raise PermissionError('bounded home return is already active')
            if not self.manual_enabled or now >= self.deadline:
                raise PermissionError('manual scope disabled or expired')
            if (request.request_id in self.used_requests
                    or not request.request_id.isascii()
                    or not 1 <= len(request.request_id) <= 128):
                raise PermissionError('home request is invalid or already consumed')
            if request.confirmation_token != self.scope:
                raise PermissionError('home confirmation scope does not match the active control scope')
            if not all(v is True for v in self.confirmations.values()):
                raise PermissionError('all onsite confirmations must remain true')

            self.homing = True
            self.used_requests.add(request.request_id)
            mode_result = ensure_can_control(self.session, self.manual_adapter,
                                             deadline_ns=self.deadline)
            self.event(dict(event='PRE_HOME_CAN_MODE_RESULT',
                            request_id=request.request_id,
                            status=mode_result['status'], reset_sent=False,
                            automatic_enable=False))
            deadline = min(time.monotonic() + 90.0, self.deadline / 1e9)
            zero_sent = False
            for _ in range(ZERO_RETURN_MAX_SEGMENTS):
                if time.monotonic() >= deadline:
                    raise TimeoutError('bounded home return deadline')
                self.session.poll(0.20)
                state = self.session.state()
                if not state.get('complete_fresh'):
                    raise PermissionError('complete fresh feedback is required before each home segment')
                if state['gripper']['status_raw'] & 0xBF:
                    raise PermissionError('gripper fault blocks joint home return')
                current = tuple(state['joint_raw_0p001deg'])
                if all(abs(v) <= ZERO_RETURN_FINAL_TOLERANCE_RAW for v in current):
                    response.accepted = True
                    response.decision = 'ZERO_FEEDBACK_MATCH'
                    response.reason = json.dumps(dict(
                        status='ZERO_FEEDBACK_MATCH', steps=steps, final_joint_raw=current,
                        final_tolerance_raw=ZERO_RETURN_FINAL_TOLERANCE_RAW,
                        gripper_held_raw=state['gripper']['position_raw'],
                        physical_stop_verified=False,
                    ), ensure_ascii=False)
                    self.event(dict(event='SAFE_HOME_RESULT', request_id=request.request_id,
                                    accepted=True, steps=len(steps), final_joint_raw=current,
                                    can_tx_calls=self.manual_adapter.tx_calls))
                    return response

                target = (0,) * 6
                if zero_sent:
                    continue
                segment_id = f'{request.request_id}-{len(steps)+1:02d}'
                segment_now = time.perf_counter_ns()
                segment_deadline = min(self.deadline, segment_now + 200_000_000)
                self.manual_adapter.context = dict(
                    request_id=segment_id, source_session_id=self.session_id,
                    source_state_sequence=self.sequence,
                )
                self.event(dict(self.manual_adapter.context, event='SAFE_HOME_TARGET_INTENT',
                                operation='set_joint_target', joint_raw=target,
                                gripper_raw=state['gripper']['position_raw'],
                                current_joint_raw=current))
                snapshot = self.session.create_manual_gamepad_snapshot(
                    physical_estop_ready=True, low_speed_confirmed=True,
                    motion_envelope_cleared=True)
                target_request = ManualGamepadBatch(
                    request_id=segment_id, source_sequence=snapshot.sequence,
                    current_joints=snapshot.joint_raw_0p001deg, target_joints=target,
                    current_gripper=snapshot.gripper_raw, target_gripper=snapshot.gripper_raw,
                    expires_ns=segment_deadline, joints_only=True, zero_return=True,
                    operator_approved=True, physical_estop_ready=True,
                    low_speed_confirmed=True, motion_envelope_cleared=True,
                    joint_step_limit_raw=180_000,
                    gripper_step_limit_raw=CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
                    controller_speed_percent=10,
                )
                writes = self.session.send_manual_gamepad_target(snapshot=snapshot, batch=target_request)
                zero_sent = True
                check = {'status': 'TARGET_NOT_CONFIRMED_NO_RETRY'}
                while time.monotonic() < deadline:
                    check = self.session.wait_joint_target(
                        target, timeout_s=min(5.0, max(0.001, deadline-time.monotonic())),
                        tolerance_raw=ZERO_RETURN_FINAL_TOLERANCE_RAW,
                        stable_for_ns=MANUAL_FEEDBACK_LIMITS.stable_duration_ns,
                        stable_max_gap_ns=MANUAL_FEEDBACK_LIMITS.stable_max_gap_ns,
                        stability_drift_raw=MANUAL_FEEDBACK_LIMITS.held_joint_drift_tolerance_raw,
                    )
                    if check['status'] == 'JOINT_FEEDBACK_MATCH':
                        break
                    checked_state = check.get('state', {})
                    if (not checked_state.get('complete_fresh')
                            or checked_state.get('error_code') != 0
                            or checked_state.get('arm_status_raw') != 0
                            or not all(checked_state.get('motor_enabled', [False]))):
                        raise RuntimeError('home feedback lost or motor fault; no retry')
                step = dict(source=list(current), target=list(target),
                            accepted_frames=len(writes), feedback_check=check)
                steps.append(step)
                self.event(dict(event='SAFE_HOME_SEGMENT_RESULT', request_id=segment_id,
                                status=check['status'], target_residual_raw=check.get('target_residual_raw'),
                                target_tolerance_raw=check.get('target_tolerance_raw')))
                if check['status'] != 'JOINT_FEEDBACK_MATCH':
                    raise RuntimeError('home segment feedback not confirmed; no retry')
            raise TimeoutError('bounded home return segment budget')
        except Exception as exc:
            response.reason = json.dumps(dict(
                status='STOPPED_NO_RETRY', error_type=type(exc).__name__, reason=str(exc),
                steps=steps, physical_stop_verified=False,
            ), ensure_ascii=False)
            self.event(dict(event='SAFE_HOME_RESULT', request_id=request.request_id,
                            accepted=False, reason=str(exc), steps=len(steps),
                            can_tx_calls=self.manual_adapter.tx_calls))
        finally:
            self.homing = False
        return response

    def dispatch(self, request, response, operation):
        response.request_id = request.request_id
        response.accepted = False
        response.decision = 'REJECTED_MANUAL_GAMEPAD'
        before = self.manual_adapter.tx_calls
        try:
            now = time.perf_counter_ns()
            if self.homing:
                raise PermissionError('manual targets are paused during bounded home return')
            if not self.manual_enabled or now >= self.deadline:
                raise PermissionError('manual scope disabled or expired')
            if request.task_scope_id != self.scope or not 1 <= request.validity_ms <= 250:
                raise PermissionError('wrong task scope or validity')
            if request.request_id in self.used_requests or len(self.used_requests) >= self.request_budget:
                raise PermissionError('request already consumed or session budget exhausted')
            if not self.sources:
                raise PermissionError('no published feedback')
            state = self.session.state()
            latest = self.sources[-1]
            try:
                validate_bridge_control_source(recent_state_sources=self.sources,
                    source_session_id=request.source_session_id, source_state_sequence=request.source_state_sequence,
                    active_session_id=self.session_id, latest_state_sequence=self.sequence,
                    latest_state_monotonic_ns=latest[2], latest_mapping_uncertainty_ns=0,
                    latest_state_feedback_fresh=state['complete_fresh'], now_monotonic_ns=now)
            except ValueError as exc:
                if str(exc) in ('latest gateway feedback is not marked fresh',
                                'latest gateway state is stale or outside clock uncertainty',
                                'requested gateway feedback is not marked fresh',
                                'requested source state is stale or outside clock uncertainty'):
                    response.decision = 'SKIPPED_STALE_FEEDBACK_NO_TX'
                raise
            joints = tuple(state['joint_raw_0p001deg'])
            grip = state['gripper']['position_raw']
            if operation == 'set_joint_target':
                t = request.target
                if (list(t.name) != [f'joint{i}' for i in range(1, 7)] or len(t.position) != 6
                        or t.velocity or t.effort or not all(math.isfinite(v) for v in t.position)):
                    raise ValueError('six canonical finite joint radians without velocity/effort required')
                joints = tuple(int(v) for v in rad_to_sdk_joint(t.position))
                if any(not lo <= v <= hi for v, (lo, hi) in zip(joints, TASK_JOINT_BOUNDS_RAW)):
                    raise ValueError('requested joint target outside shared envelope')
                requested_joints = joints
                current = state['joint_raw_0p001deg']
                joints = tuple(max(lo, min(hi, max(c-5000, min(c+5000, v))))
                    for v, c, (lo, hi) in zip(joints, current, TASK_JOINT_BOUNDS_RAW))
                if joints != requested_joints:
                    self.event(dict(event='TARGET_RECLAMPED_TO_LATEST_FEEDBACK',
                        request_id=request.request_id, requested_joint_raw=requested_joints,
                        applied_joint_raw=joints))
            else:
                grip = request.gripper_raw
                if not CONTINUOUS_GRIPPER_LIMITS.minimum_raw <= grip <= CONTINUOUS_GRIPPER_LIMITS.maximum_raw:
                    raise ValueError('requested gripper target outside manual envelope')
                if (not math.isfinite(request.nominal_position_m) or not math.isfinite(request.nominal_effort_nm)
                        or abs(request.nominal_position_m - grip * GRIPPER_NOMINAL_METERS_PER_RAW)
                            > GRIPPER_NOMINAL_METERS_PER_RAW / 2
                        or request.nominal_effort_nm * 1000 != CONTINUOUS_GRIPPER_LIMITS.effort_raw):
                    raise ValueError('gripper raw/nominal units or effort mismatch')
                requested_grip = grip
                current_grip = state['gripper']['position_raw']
                step = CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw
                grip = max(current_grip-step, min(current_grip+step, grip))
                if grip != requested_grip:
                    self.event(dict(event='GRIPPER_RECLAMPED_TO_LATEST_FEEDBACK',
                        request_id=request.request_id, requested_gripper_raw=requested_grip,
                        applied_gripper_raw=grip))
            self.manual_adapter.context = dict(request_id=request.request_id,
                source_session_id=request.source_session_id, source_state_sequence=request.source_state_sequence)
            self.event(dict(self.manual_adapter.context, event='TARGET_INTENT', operation=operation,
                            joint_raw=joints, gripper_raw=grip, feedback=state))
            target = ManualCommissioningTarget(request_id=request.request_id, operation=operation,
                target_joint_raw=joints, target_gripper_raw=grip,
                valid_until_monotonic_ns=min(self.deadline, now + request.validity_ms * 1_000_000),
                joint_step_limit_raw=5000, gripper_step_limit_raw=CONTINUOUS_GRIPPER_LIMITS.maximum_step_raw,
                gripper_profile='manual_gamepad', **self.confirmations)
            from dataclasses import replace
            target = replace(target, controller_speed_percent=self.controller_speed_percent)
            self.used_requests.add(request.request_id)
            target.execute(self.session)
            response.accepted = True
            response.decision = 'ADAPTER_ACCEPTED_MANUAL_GAMEPAD_BATCH'
            response.reason = 'SocketCAN kernel accepted; physical execution unverified'
        except Exception as exc:
            response.reason = str(exc)
            from laptop.transport.manual_gamepad import recoverable_manual_rejection
            if recoverable_manual_rejection(exc,
                    can_tx_calls=self.manual_adapter.tx_calls-before,
                    bridge_ready=self.session.bridge.state.name == 'READY'):
                response.decision = 'SKIPPED_TRANSIENT_TARGET_NO_TX'
        receipt = CommandReceipt()
        receipt.header.stamp = self.get_clock().now().to_msg()
        receipt.request_id = request.request_id
        receipt.decision, receipt.reason = response.decision, response.reason
        receipt.sequence = self.sequence
        receipt.source_monotonic_ns = time.perf_counter_ns()
        receipt.adapter_accepted = response.accepted
        receipt.feedback_matched = False
        receipt.can_tx_calls = self.manual_adapter.tx_calls - before
        self.receipts.publish(receipt)
        self.event(dict(event='SERVICE_RESULT', request_id=request.request_id,
                        accepted=response.accepted, reason=response.reason, can_tx_calls=receipt.can_tx_calls))
        return response

    def close(self):
        super().close()
        if self.event_file is not None:
            self.event_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NativeManualGamepad()
        rclpy.spin(node)  # Single owner: receive, state, and send callbacks share one thread.
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
