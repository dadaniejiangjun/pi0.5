"""Explicit J5 fault recovery without reset, disable, or encoder-zero writes.

Separate from teleoperation: the normal manual bridge correctly refuses faulty
feedback. This bounded recovery tool requires fresh, stable, uncaught feedback
and a current operator clearance before each one-use recovery write.
"""
import argparse
import json
from pathlib import Path
import struct
import time

from laptop.robot_client.piper_session import PiperSession
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.socketcan_manual import SocketCanManualAdapter
from laptop.transport.position_commands import CanTxFrame, TASK_JOINT_BOUNDS_RAW
from laptop.transport.mode_initialization import ModeInitializationFrame


def recovery_frame(stage):
    # Official SDK JointConfig(5, set_zero=0, acc_effective=0,
    # max_joint_acc=0x7fff, clear_err=0xae), and EnableArm(5, 2).
    if stage == 'clear_j5':
        return CanTxFrame(0x475, bytes.fromhex('0500007fffae0000'))
    if stage == 'enable_j5':
        return CanTxFrame(0x471, bytes.fromhex('0502000000000000'))
    raise ValueError('only J5 clear and J5 enable are available')


def check_state(state, *, j5_fault_allowed=False):
    if (not state['complete_fresh'] or state['ctrl_mode_raw'] != 1
            or state['move_mode_raw'] != 1 or state['arm_status_raw'] or state['error_code']):
        raise PermissionError('fresh healthy CAN_CTRL/MOVE_J state required')
    codes = state['motor_status_codes']
    if any(code != 0x40 for i, code in enumerate(codes) if i != 4):
        raise PermissionError('all other five motors must remain healthy and enabled')
    if not j5_fault_allowed and codes[4] & 0xBF:
        raise PermissionError('J5 fault remains; no enable or retry')
    if any(not lo <= v <= hi for v, (lo, hi) in zip(state['joint_raw_0p001deg'], TASK_JOINT_BOUNDS_RAW)):
        raise PermissionError('recovery pose outside joint target limits')


def execute(output, *, site_confirmed=False):
    if not site_confirmed:
        raise PermissionError('operator must confirm arm untrapped, hands clear, E-stop ready')
    report = dict(status='STARTED', writes=[], reset_sent=False, disable_sent=False,
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
                def read():
                    robot.poll(.4)
                    state = robot.state()
                    robot.record_state_snapshot(state)
                    return state
                initial = read(); check_state(initial, j5_fault_allowed=True)
                report['initial_state'] = initial
                stable = read(); check_state(stable, j5_fault_allowed=True)
                if any(abs(b-a)>100 for a,b in zip(initial['joint_raw_0p001deg'],stable['joint_raw_0p001deg'])):
                    raise PermissionError('arm drifting; physical support required before recovery')
                used = set()
                def once(stage):
                    if stage in used:
                        raise PermissionError('one-use recovery stage; no retry')
                    used.add(stage)
                    frame = recovery_frame(stage)
                    adapter.context = dict(recovery_stage=stage)
                    # Dedicated, exact SDK recovery frames after the checks above;
                    # no public arbitrary-frame or reset/disable operation exists.
                    adapter._send(frame)
                    report['writes'].append(dict(stage=stage, can_id=frame.arbitration_id,
                        data_hex=frame.data.hex(), kernel_accepted=True))
                if stable['motor_status_codes'][4] & 0xBF:
                    once('clear_j5')
                cleared = read(); report['after_clear'] = cleared
                check_state(cleared)
                if not cleared['motor_enabled'][4]:
                    # Replace old position goals with the measured pose before
                    # enabling the disabled joint, retaining 5% MOVE_J mode.
                    adapter.context = dict(recovery_stage='measured_pose_hold')
                    adapter.send_mode_initialization_frame(ModeInitializationFrame('can_joint'))
                    q = cleared['joint_raw_0p001deg']
                    for i in range(3):
                        adapter.send_position_frame(CanTxFrame(0x155+i, struct.pack('>ii', *q[i*2:i*2+2])))
                    report['measured_pose_hold_sent'] = True
                    before_enable = read(); check_state(before_enable)
                    if any(abs(a-b)>100 for a,b in zip(q,before_enable['joint_raw_0p001deg'])):
                        raise PermissionError('pose changed before enable; stop')
                    once('enable_j5')
                final = read(); report['final_state'] = final; check_state(final)
                if not all(final['motor_enabled']):
                    raise PermissionError('all-six enable not confirmed; no retry')
                if any(abs(a-b)>500 for a,b in zip(stable['joint_raw_0p001deg'],final['joint_raw_0p001deg'])):
                    raise PermissionError('unexpected movement during recovery; no zero return')
                report['status'] = 'ALL_SIX_HEALTHY_ENABLED'
        except Exception as exc:
            report.update(status='STOPPED_NO_RETRY', reason=str(exc), error_type=type(exc).__name__)
        report['can_tx_calls'] = adapter.tx_calls
    Path(output).write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--site-confirmed', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = execute(args.output, site_confirmed=args.site_confirmed)
    print(json.dumps(report, indent=2))
    return 0 if report['status']=='ALL_SIX_HEALTHY_ENABLED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
