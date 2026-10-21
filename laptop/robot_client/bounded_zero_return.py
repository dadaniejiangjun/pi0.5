"""Authorized joint zero return in 5-degree segments, preserving gripper position."""
import argparse
import gc
import json
import time
import uuid
from pathlib import Path
from laptop.diagnostics.bounded_process import run_bounded
from laptop.robot_client.piper_session import PiperSession
from laptop.control.feedback_acceptance import (
    MANUAL_FEEDBACK_LIMITS,
    ZERO_RETURN_FINAL_TOLERANCE_RAW,
    ZERO_RETURN_SEGMENT_TOLERANCE_RAW,
)
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.bounded_trial import BoundedTrialBatch


ZERO_RETURN_SEGMENT_RAW = 5000
ZERO_RETURN_MAX_SEGMENTS = 40


def zero_target(current):
    if len(current) != 6 or any(type(v) is not int for v in current):
        raise ValueError('six integer joints required')
    return tuple(v-max(-ZERO_RETURN_SEGMENT_RAW,min(ZERO_RETURN_SEGMENT_RAW,v))
                 for v in current)


def zero_bridge(backend='gs_usb', interface='can0'):
    if backend == 'socketcan':
        from laptop.transport.socketcan_manual import SocketCanManualAdapter
        adapter = SocketCanManualAdapter(interface, transmit_enabled=True)
        return SafetyBridge(mode=RuntimeMode.SUPERVISED_MANUAL_GAMEPAD,
            transport=GsUsbPiperCanTransport(adapter_factory=lambda: adapter),
            limits=UNVERIFIED_ANALYSIS_LIMITS, supervised_manual_gamepad_tx_enabled=True)
    if backend != 'gs_usb':
        raise ValueError('unknown zero-return backend')
    return SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=GsUsbPiperCanTransport(), limits=UNVERIFIED_ANALYSIS_LIMITS,
        supervised_j1_commissioning_tx_enabled=True)


def execute(output,max_start_deg=60,backend='gs_usb',interface='can0'):
    # The shared PiPER joint limits allow poses above 90 degrees (for example,
    # J2 on a collision-avoidance pose).  Callers must explicitly widen the
    # start envelope; every transmitted segment remains capped at five degrees.
    if not 0<max_start_deg<=180:raise ValueError("start envelope must be in (0,180]")
    report=dict(status='STARTED',steps=[],gripper_held=True,backend=backend,interface=interface,
        controller_speed_percent=10,
        final_joint_tolerance_raw=ZERO_RETURN_FINAL_TOLERANCE_RAW,
        segment_joint_tolerance_raw=ZERO_RETURN_SEGMENT_TOLERANCE_RAW,
        stable_duration_ns=MANUAL_FEEDBACK_LIMITS.stable_duration_ns,
        stability_drift_tolerance_raw=MANUAL_FEEDBACK_LIMITS.held_joint_drift_tolerance_raw)
    deadline=time.monotonic()+90
    try:
        for index in range(ZERO_RETURN_MAX_SEGMENTS):
            if time.monotonic()>=deadline:raise TimeoutError('zero return deadline')
            bridge=zero_bridge(backend, interface)
            with PiperSession(Path(output).with_suffix(f'.step{index}.can.jsonl'),bridge=bridge,
                              log_rx_frames=(backend!='socketcan')) as robot:
                robot.poll(0.65)
                snapshot_factory=(robot.create_manual_gamepad_snapshot if backend=='socketcan'
                                  else robot.commissioning_snapshot)
                snapshot=snapshot_factory(physical_estop_ready=True,
                    low_speed_confirmed=True,motion_envelope_cleared=True,
                    **({'joints_only': True} if backend=='socketcan' else {}))
                state=robot.state()
                robot.record_state_snapshot(state)
                current=snapshot.joint_raw_0p001deg
                if not state['complete_fresh'] or state['gripper']['status_raw']&0xBF:
                    raise RuntimeError('feedback incomplete or gripper fault')
                if any(abs(v)>round(max_start_deg*1000) for v in current):raise ValueError('zero return exceeds authorized start envelope')
                if all(abs(v)<=ZERO_RETURN_FINAL_TOLERANCE_RAW for v in current):
                    report.update(status='ZERO_FEEDBACK_MATCH',final_state=state)
                    break
                target=zero_target(current)
                grip=state['gripper']['position_raw']
                if backend=='socketcan':
                    from laptop.transport.manual_gamepad import ManualGamepadBatch
                    batch=ManualGamepadBatch(request_id='zero-'+uuid.uuid4().hex,
                        source_sequence=snapshot.sequence,current_joints=current,target_joints=target,
                        current_gripper=grip,target_gripper=grip,joints_only=True,
                        expires_ns=time.perf_counter_ns()+200_000_000,
                        controller_speed_percent=10,
                        operator_approved=True,physical_estop_ready=True,
                        low_speed_confirmed=True,motion_envelope_cleared=True)
                else:
                    batch=BoundedTrialBatch(snapshot.sequence,current,target,grip,grip,
                        'zero-'+uuid.uuid4().hex,time.perf_counter_ns()+200_000_000,
                        joint_step_limit_raw=ZERO_RETURN_SEGMENT_RAW,
                        gripper_profile='single_step_commissioning',
                        gripper_step_limit_raw=10_000,
                        controller_speed_percent=10)
                record=dict(source=list(current),target=list(target));report['steps'].append(record)
                writes=(robot.send_manual_gamepad_target(snapshot=snapshot,batch=batch)
                    if backend=='socketcan' else robot.send_bounded_model_trial(
                        snapshot=snapshot,batch=batch,operator_approved=True))
                record['send_interface']={'status':'ADAPTER_ACCEPTED','frame_count':len(writes),'physical_motion_proven':False}
                record['accepted_frames']=len(writes)
                check=robot.wait_joint_target(
                    target, timeout_s=5,
                    tolerance_raw=ZERO_RETURN_SEGMENT_TOLERANCE_RAW,
                    stable_for_ns=MANUAL_FEEDBACK_LIMITS.stable_duration_ns,
                    stable_max_gap_ns=MANUAL_FEEDBACK_LIMITS.stable_max_gap_ns,
                    stability_drift_raw=MANUAL_FEEDBACK_LIMITS.held_joint_drift_tolerance_raw,
                )
                record['feedback_check']=check
                checked_state=check.get('state', {})
                observed=checked_state.get('joint_raw_0p001deg', [])
                active=[axis for axis,(before,goal) in enumerate(zip(current,target)) if abs(goal-before)>MANUAL_FEEDBACK_LIMITS.motion_noise_joint_raw]
                direction_ok=len(observed)==6 and bool(active) and all(
                    (observed[axis]-current[axis])*(1 if target[axis]>current[axis] else -1)
                    > MANUAL_FEEDBACK_LIMITS.motion_noise_joint_raw
                    for axis in active
                )
                record['acceptance']={
                    'send_interface':'ACCEPTED',
                    'observed_motion':'EXPECTED_DIRECTION_CONFIRMED' if direction_ok else 'NOT_CONFIRMED',
                    'precision_arrival':check['status'],
                    'target_residual_raw':check.get('target_residual_raw'),
                    'target_tolerance_raw':check.get('target_tolerance_raw'),
                }
                if check['status']!='JOINT_FEEDBACK_MATCH':raise RuntimeError('zero segment not confirmed; no retry')
                robot.poll(0.5)
            del robot,bridge
            gc.collect()
            time.sleep(0.5)
        else:raise TimeoutError('zero return segment budget')
    except Exception as exc:
        report.update(status='STOPPED_NO_RETRY',reason=str(exc),error_type=type(exc).__name__)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-start-deg',type=float,choices=(60,90,130,180),default=60)
    parser.add_argument('--backend',choices=('gs_usb','socketcan'),default='gs_usb')
    parser.add_argument('--interface',default='can0')
    parser.add_argument('--site-cleared-zero-authorized',action='store_true')
    args=parser.parse_args()
    if not args.site_cleared_zero_authorized:raise PermissionError('zero return authorization required')
    with args.output.open('x',encoding='utf-8') as f:
        f.write('{"status":"STARTED"}');f.flush()
        result=run_bounded(execute,args=(args.output,args.max_start_deg,args.backend,args.interface),timeout_s=100)
        report=result['result'] if result['status']=='COMPLETE' else result
        f.seek(0);f.truncate();json.dump(report,f,indent=2)
    print(json.dumps(dict(status=report['status'],steps=len(report.get('steps',[])),reason=report.get('reason'))))


if __name__=='__main__':main()
