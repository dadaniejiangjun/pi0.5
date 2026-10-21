"""Reusable native WSL PiPER status, no-reset mode switch, and zero return."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import struct
import subprocess
import time
import uuid

from laptop.robot_client.piper_session import PiperSession
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.socketcan_receive import SocketCanReceiveTransport
from laptop.transport.socketcan_manual import SocketCanManualAdapter
from laptop.transport.position_commands import CanTxFrame, TASK_JOINT_BOUNDS_RAW
from laptop.transport.mode_initialization import ModeInitializationFrame
from laptop.transport.manual_gamepad import bounded_hold_joints


def teaching_exit_frame():
    # Official MotionCtrl_1(emergency_stop=0, track_ctrl=0, grag_teach_ctrl=2).
    return CanTxFrame(0x150, bytes.fromhex('0000020000000000'))


def require_switch_state(state):
    if (not state['complete_fresh'] or state['ctrl_mode_raw'] not in (0, 1, 2)
            or state['arm_status_raw'] or state['error_code']
            or state['motor_status_codes'] != [0x40]*6
            or state['gripper']['status_raw'] & 0xBF):
        raise PermissionError('fresh, fault-free, all-six-enabled feedback required; no automatic reset/enable')
    raw = tuple(state['joint_raw_0p001deg'])
    legal = bounded_hold_joints(raw)
    if any(abs(a-b)>500 for a,b in zip(raw, legal)):
        raise PermissionError('mode switch hold correction exceeds 0.5 degree')


def status(output):
    bridge = SafetyBridge(mode=RuntimeMode.POLICY_REJECT_ONLY,
        transport=SocketCanReceiveTransport('can0'), limits=UNVERIFIED_ANALYSIS_LIMITS)
    with PiperSession(str(output)+'.can.jsonl', bridge=bridge, log_rx_frames=False) as robot:
        robot.poll(.7)
        state = robot.state()
    return dict(status='STATE_READ' if state['complete_fresh'] else 'INCOMPLETE_STATE_NO_TX', state=state, can_tx_calls=0,
                robot_motion_commanded=False)


def ensure_can_control(robot, adapter, *, deadline_ns=None):
    """Reuse the no-reset switch inside an existing exclusive CAN session."""
    report = {}
    def read():
        if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
            raise TimeoutError("control scope expired before mode switch")
        robot.poll(.5)
        state = robot.state()
        robot.record_state_snapshot(state)
        require_switch_state(state)
        return state
    initial = read(); report['initial_state'] = initial
    if initial['ctrl_mode_raw'] == 1 and initial['move_mode_raw'] == 1:
        report.update(status='ALREADY_CAN_CTRL_NO_WRITES', final_state=initial)
    else:
        stable = read()
        measured_q = stable['joint_raw_0p001deg']
        q = bounded_hold_joints(tuple(measured_q))
        if any(abs(a-b)>100 for a,b in zip(initial['joint_raw_0p001deg'],measured_q)):
            raise PermissionError('arm not stable; switch withheld')
        # Replace stale position goals, without requesting any new pose.
        adapter.context = dict(stage='measured_pose_hold_before_mode_switch')
        for i in range(3):
            adapter.send_position_frame(CanTxFrame(0x155+i, struct.pack('>ii', *q[i*2:i*2+2])))
        if stable['ctrl_mode_raw'] == 2:
            adapter.context = dict(stage='exit_teaching_no_reset')
            adapter._send(teaching_exit_frame())
            report['after_teach_exit'] = read()
        adapter.context = dict(stage='can_move_j_5_percent')
        adapter.send_mode_initialization_frame(ModeInitializationFrame('can_joint'))
        final = read(); report['final_state'] = final
        if final['ctrl_mode_raw'] != 1 or final['move_mode_raw'] != 1:
            raise PermissionError('CAN_CTRL not confirmed; no reset or retry')
        if any(abs(a-b)>500 for a,b in zip(q,final['joint_raw_0p001deg'])):
            raise PermissionError('unexpected pose change; stop')
        report['status'] = 'CAN_CTRL_CONFIRMED_NO_RESET'
    return report


def switch_can(output, *, site_confirmed=False):
    if not site_confirmed:
        raise PermissionError('hands/sticks released, stable arm, clear path and E-stop confirmation required')
    active = subprocess.run(['pgrep', '-af',
        '/lib/(pi05_piper_bridge/native_(can_state|manual_gamepad)|pi05_piper_gamepad/gamepad_operator|pi05_piper_quest/quest_execution)( |$)'],
        capture_output=True, text=True)
    if active.returncode != 1:
        raise PermissionError('stop existing CAN/control nodes before switching: '+active.stdout)
    report = dict(status='STARTED', reset_sent=False, disable_sent=False,
                  encoder_zero_changed=False, automatic_retry=False)
    with open(str(output)+'.frames.jsonl', 'x', encoding='utf-8') as log:
        def event(record):
            log.write(json.dumps(record)+'\n'); log.flush()
        adapter = SocketCanManualAdapter('can0', transmit_enabled=True, event_sink=event)
        bridge = SafetyBridge(mode=RuntimeMode.POLICY_REJECT_ONLY,
            transport=GsUsbPiperCanTransport(adapter_factory=lambda: adapter),
            limits=UNVERIFIED_ANALYSIS_LIMITS)
        try:
            with PiperSession(str(output)+'.can.jsonl', bridge=bridge, log_rx_frames=False) as robot:
                report.update(ensure_can_control(robot, adapter))
        except Exception as exc:
            report.update(status='STOPPED_NO_RETRY', reason=str(exc), error_type=type(exc).__name__)
        report['can_tx_calls'] = adapter.tx_calls
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation', choices=('status', 'can-mode', 'zero', 'clear-j5'))
    p.add_argument('--site-confirmed', action='store_true')
    p.add_argument('--max-start-deg', type=float, choices=(60, 90, 130, 180), default=90)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    out = args.output or Path('cache/native_piper_tool') / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:8]+'_'+args.operation+'.json')
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x', encoding='utf-8') as report_file:
        report_file.write('{"status":"STARTED"}'); report_file.flush()
        try:
            if args.operation == 'status':
                report = status(out)
            else:
                if not args.site_confirmed:
                    raise PermissionError('--site-confirmed required')
                if args.operation == 'can-mode':
                    report = switch_can(out, site_confirmed=True)
                else:
                    # No teleoperation owner may issue new targets during recovery.
                    active = subprocess.run(['pgrep','-af',
                        '/lib/(pi05_piper_bridge/native_(can_state|manual_gamepad)|pi05_piper_gamepad/gamepad_operator|pi05_piper_quest/quest_execution)( |$)'],
                        capture_output=True, text=True)
                    if active.returncode != 1:
                        raise PermissionError('stop existing CAN/control nodes first: '+active.stdout)
                    if args.operation == 'zero':
                        from laptop.robot_client.bounded_zero_return import execute
                        report = execute(out, args.max_start_deg, 'socketcan', 'can0')
                    else:
                        from laptop.robot_client.native_joint_fault_recovery import execute
                        report = execute(out, site_confirmed=True)
        except Exception as exc:
            report = dict(status='STOPPED_NO_RETRY', reason=str(exc), error_type=type(exc).__name__)
        report_file.seek(0); report_file.truncate()
        json.dump(report, report_file, indent=2)
    print(json.dumps(dict(report_path=str(out), **report), indent=2))
    return 0 if report['status'] in ('STATE_READ', 'ALREADY_CAN_CTRL_NO_WRITES',
        'CAN_CTRL_CONFIRMED_NO_RESET', 'ZERO_FEEDBACK_MATCH', 'ALL_SIX_HEALTHY_ENABLED') else 1


if __name__ == '__main__':
    raise SystemExit(main())
