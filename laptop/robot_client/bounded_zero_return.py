"""Authorized joint zero return in 5-degree segments, preserving gripper position."""
import argparse
import gc
import json
import time
import uuid
from pathlib import Path
from laptop.diagnostics.bounded_process import run_bounded
from laptop.robot_client.piper_session import PiperSession
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.bounded_trial import BoundedTrialBatch


def zero_target(current):
    if len(current) != 6 or any(type(v) is not int for v in current):
        raise ValueError('six integer joints required')
    return tuple(v-max(-5000,min(5000,v)) for v in current)


def execute(output,max_start_deg=60):
    if not 0<max_start_deg<=90:raise ValueError("start envelope must be in (0,90]")
    report=dict(status='STARTED',steps=[],gripper_held=True)
    deadline=time.monotonic()+90
    try:
        for index in range(20):
            if time.monotonic()>=deadline:raise TimeoutError('zero return deadline')
            bridge=SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
                transport=GsUsbPiperCanTransport(),limits=UNVERIFIED_ANALYSIS_LIMITS,
                supervised_j1_commissioning_tx_enabled=True)
            with PiperSession(Path(output).with_suffix(f'.step{index}.can.jsonl'),bridge=bridge) as robot:
                robot.poll(0.65)
                snapshot=robot.commissioning_snapshot(physical_estop_ready=True,
                    low_speed_confirmed=True,motion_envelope_cleared=True)
                state=robot.state()
                current=snapshot.joint_raw_0p001deg
                if not state['complete_fresh'] or state['gripper']['status_raw']&0xBF:
                    raise RuntimeError('feedback incomplete or gripper fault')
                if any(abs(v)>round(max_start_deg*1000) for v in current):raise ValueError('zero return exceeds authorized start envelope')
                if all(abs(v)<=100 for v in current):
                    report.update(status='ZERO_FEEDBACK_MATCH',final_state=state)
                    break
                target=zero_target(current)
                grip=state['gripper']['position_raw']
                batch=BoundedTrialBatch(snapshot.sequence,current,target,grip,grip,
                    'zero-'+uuid.uuid4().hex,time.perf_counter_ns()+200_000_000,
                    joint_step_limit_raw=5000)
                record=dict(source=list(current),target=list(target));report['steps'].append(record)
                writes=robot.send_bounded_model_trial(snapshot=snapshot,batch=batch,operator_approved=True)
                record['accepted_frames']=len(writes)
                check=robot.wait_joint_target(target,timeout_s=5,tolerance_raw=500)
                record['feedback_check']=check
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
    parser.add_argument('--max-start-deg',type=float,choices=(60,90),default=60)
    parser.add_argument('--site-cleared-zero-authorized',action='store_true')
    args=parser.parse_args()
    if not args.site_cleared_zero_authorized:raise PermissionError('zero return authorization required')
    with args.output.open('x',encoding='utf-8') as f:
        f.write('{"status":"STARTED"}');f.flush()
        result=run_bounded(execute,args=(args.output,args.max_start_deg),timeout_s=100)
        report=result['result'] if result['status']=='COMPLETE' else result
        f.seek(0);f.truncate();json.dump(report,f,indent=2)
    print(json.dumps(dict(status=report['status'],steps=len(report.get('steps',[])),reason=report.get('reason'))))


if __name__=='__main__':main()
