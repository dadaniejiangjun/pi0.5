"""Single authorized near-zero return followed by empty-gripper closure; no retry."""
import argparse
import json
import struct
import time
import uuid
from pathlib import Path
from laptop.robot_client.piper_session import PiperSession
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.position_commands import CanTxFrame, J1CommissioningBatch, J1CommissioningAuthorization


def execute(log_path, close_only=False):
    site = dict(physical_estop_ready=True, low_speed_confirmed=True, motion_envelope_cleared=True)
    bridge = SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=GsUsbPiperCanTransport(), limits=UNVERIFIED_ANALYSIS_LIMITS,
        supervised_j1_commissioning_tx_enabled=True)
    with PiperSession(log_path, bridge=bridge) as robot:
        robot.poll(0.6)
        if close_only:
            state = robot.state()
            if not healthy(state) or any(abs(v)>100 for v in state['joint_raw_0p001deg']):
                return dict(status='CLOSE_BLOCKED_NOT_AT_ZERO', final_feedback=state)
            bridge.send_gripper_opening_once(operator_approved=True, delta_raw=0,
                effort_raw=100, close_to_zero=True, **site)
            robot.poll(2)
            state=robot.state()
            matched=healthy(state) and all(abs(v)<=100 for v in state['joint_raw_0p001deg']) and abs(state.get('gripper',{}).get('position_raw',999999))<=100
            return dict(status='HOME_AND_CLOSED_FEEDBACK_MATCH' if matched else 'FINAL_FEEDBACK_UNCONFIRMED_NO_RETRY',
                        final_feedback=state, joint_targets_sent=False, automatic_retry=False)
        snapshot = robot.commissioning_snapshot(**site)
        batch = J1CommissioningBatch(snapshot.sequence,
            max(t for _, t in snapshot.feedback_timestamps_ns), snapshot.joint_raw_0p001deg,
            (0,)*6, (CanTxFrame(0x151, bytes((1,1,5,0,0,0,0,0))),
                     *(CanTxFrame(i, struct.pack('>ii',0,0)) for i in (0x155,0x156,0x157))),
            configure_control_mode=True, near_zero_home=True)
        now = time.perf_counter_ns()
        approval = J1CommissioningAuthorization('home-'+uuid.uuid4().hex, snapshot.sequence,
            (0,)*6, now, now+200_000_000, True, True, True, True)
        bridge.record_heartbeat(now)
        receipt = robot.send_j1(snapshot=snapshot, batch=batch, authorization=approval)
        result = dict(source_joint_raw=list(snapshot.joint_raw_0p001deg), target_joint_raw=[0]*6,
                      joint_frame_count=len(receipt.writes), automatic_retry=False)
        robot.poll(2)
        state = robot.state()
        if not healthy(state) or any(abs(v)>100 for v in state['joint_raw_0p001deg']):
            return dict(result, status='HOME_UNCONFIRMED_GRIPPER_NOT_SENT', final_feedback=state)
        # Close this owner before the next preflight so its own mode/joint echo
        # is not mistaken for a competing sender. Never clear collision flags.
        return dict(result, status='HOME_FEEDBACK_MATCH', final_feedback=state)


def healthy(state):
    return bool(state['complete_fresh'] and state.get('ctrl_mode_raw')==1 and
                state.get('arm_status_raw')==0 and state.get('error_code')==0 and
                all(state.get('motor_enabled',[])) and not (state.get('gripper',{}).get('status_raw',255)&0xBF))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--site-cleared-home-and-close-authorized',action='store_true')
    parser.add_argument('--close-only-at-zero',action='store_true')
    args=parser.parse_args()
    if not args.site_cleared_home_and_close_authorized:
        return 2
    with args.output.open('x',encoding='utf-8') as handle:
        handle.write('{"status":"STARTED"}\n'); handle.flush()
        try:
            result=execute(args.output.with_suffix('.can.jsonl'),close_only=args.close_only_at_zero)
            if result['status']=='HOME_FEEDBACK_MATCH':
                closure=execute(args.output.with_suffix('.close.can.jsonl'),close_only=True)
                result=dict(closure, home_result=result)
        except Exception as exc:
            result=dict(status='STOPPED_NO_RETRY', reason=str(exc), error_type=type(exc).__name__)
        handle.seek(0); handle.truncate(); json.dump(result,handle,indent=2)
    print(json.dumps(result,indent=2))
    return 0 if result['status']=='HOME_AND_CLOSED_FEEDBACK_MATCH' else 1


if __name__=='__main__':
    raise SystemExit(main())
