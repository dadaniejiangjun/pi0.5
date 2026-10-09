"""Explicitly authorized bounded short or 120-second task trial using reusable cameras and PiperSession."""
import argparse,json,math,threading,time,uuid
from pathlib import Path
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from laptop.camera.dual_camera import inventory
from laptop.recorder.record_demonstrations import camera_identity
from laptop.camera.worker import camera_loop
from laptop.client.policy_client import load_artifact_identity_lock,validate_policy_metadata,prewarm_policy
from laptop.piper.unit_conversion import (joint_raw_to_nominal_rad, nominal_rad_to_joint_raw, gripper_raw_to_nominal_m_divided, nominal_m_to_gripper_raw)
from laptop.client.observation_schema import PIPER_PROMPT, build_piper_observation
from laptop.robot_client.piper_session import PiperSession
from laptop.robot_client.bounded_policy_target import limit_target
from laptop.transport.bounded_trial import BoundedTrialBatch
from laptop.safety.bridge import SafetyBridge,UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.diagnostics.bounded_process import run_bounded


def stable_snapshot(robot, site, deadline_ns):
    """Wait without TX for a stable pose; other commissioning failures stop immediately."""
    wait_until=min(deadline_ns,time.perf_counter_ns()+3_000_000_000)
    while True:
        try:
            return robot.commissioning_snapshot(**site)
        except PermissionError as exc:
            if 'joint feedback must stay within SDK bounds and drift' not in str(exc):
                raise
            if time.perf_counter_ns()>=wait_until:
                raise
            robot.poll(0.05)


def worker(lock_path,output,port,updates,duration_s=20,long_loop=False,joint_step_deg=5,joint_tolerance_deg=0.5):
    import cv2
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
    lock=load_artifact_identity_lock(lock_path)
    client=WebsocketClientPolicy(host='127.0.0.1',port=port)
    metadata=client.get_server_metadata();validate_policy_metadata(metadata,lock)
    prewarm_policy(client)
    identity=camera_identity(inventory());cameras={};mutex=threading.Lock();stop=threading.Event();errors=[]
    video_dir=Path(output).with_suffix('');video_dir.mkdir(exist_ok=False)
    writers={};logs={};counts={}
    def record_frame(role,frame,stamp):
        if role not in writers:
            h,w=frame.shape[:2]
            writers[role]=cv2.VideoWriter(str(video_dir/f'{role}.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30,(w,h))
            if not writers[role].isOpened():raise RuntimeError('video writer failed: '+role)
            logs[role]=(video_dir/f'{role}.timestamps.jsonl').open('x',encoding='utf-8');counts[role]=0
        writers[role].write(frame)
        logs[role].write(json.dumps(dict(frame_index=counts[role],host_arrival_ns=stamp))+'\n');logs[role].flush()
        counts[role]+=1
    threads=[threading.Thread(target=camera_loop,args=(v['opencv_index'],k,cameras,mutex,stop,errors,record_frame),daemon=True) for k,v in identity['roles'].items()]
    report=dict(status='STARTED',steps=[],camera_identity=identity,physical_gripper_scale_verified=False,task_success_claim=False)
    origin=None;deadline=None
    envelope_raw=None if long_loop else round(updates*joint_step_deg*1000)
    report.update(long_loop=long_loop,duration_limit_s=duration_s,max_updates=updates,video_directory=str(video_dir),joint_step_limit_deg=joint_step_deg,joint_arrival_tolerance_deg=joint_tolerance_deg,joint_arrival_timeout_s=5,gripper_step_limit_nominal_m=None,cumulative_joint_limit_deg=None if long_loop else updates*joint_step_deg)
    try:
        for t in threads:t.start()
        camera_deadline=time.monotonic()+3
        while time.monotonic()<camera_deadline:
            with mutex: ready=dict(cameras)
            now=time.perf_counter_ns()
            if len(ready)==2 and all(0<=now-v['timestamp_ns']<=250_000_000 for v in ready.values()):break
            if errors:raise RuntimeError('camera startup failed: '+str(errors))
            time.sleep(0.01)
        else:raise TimeoutError('dual-camera startup timeout')
        loop_started=time.perf_counter_ns();deadline=loop_started+int(duration_s*1e9)
        for step in range(updates):
            if time.perf_counter_ns()>=deadline:break
            bridge=SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,transport=GsUsbPiperCanTransport(),limits=UNVERIFIED_ANALYSIS_LIMITS,supervised_j1_commissioning_tx_enabled=True)
            with PiperSession(Path(output).with_name(Path(output).stem+f'.step{step}.can.jsonl'),bridge=bridge) as robot:
                robot.poll(0.65)
                # Reopening after each completed update avoids mistaking own TX echo for another sender.
                # Every new owner must independently pass collision, stability, and freshness checks.
                site=dict(physical_estop_ready=True,low_speed_confirmed=True,motion_envelope_cleared=True)
                stable_snapshot(robot,site,deadline)
                state=robot.state();start_joints=state['joint_raw_0p001deg']
                if not state['complete_fresh'] or state['gripper']['status_raw']&0xBF:raise RuntimeError('feedback incomplete or gripper fault')
                if origin is None:origin=start_joints
                if envelope_raw is not None and any(abs(a-b)>envelope_raw+100 for a,b in zip(start_joints,origin)):raise RuntimeError('trial left bounded cumulative envelope')
                camera_wait_started=time.perf_counter_ns()
                camera_wait_deadline=min(deadline,camera_wait_started+2_000_000_000)
                while True:
                    with mutex:images=dict(cameras)
                    now=time.perf_counter_ns()
                    if errors:raise RuntimeError('camera reader failed: '+str(errors))
                    if len(images)==2 and all(0<=now-v['timestamp_ns']<=250_000_000 for v in images.values()) and abs(images['base']['timestamp_ns']-images['wrist']['timestamp_ns'])<=100_000_000:
                        break
                    if now>=camera_wait_deadline:raise TimeoutError('fresh synchronized images unavailable after bounded wait; no target sent')
                    # Receive only while waiting; never use a stale image for control.
                    robot.poll(0.01)
                if now-camera_wait_started>20_000_000:
                    robot.poll(0.01)
                    robot.commissioning_snapshot(**site)
                    state=robot.state();start_joints=state['joint_raw_0p001deg']
                    if not state['complete_fresh'] or state['gripper']['status_raw']&0xBF:raise RuntimeError('feedback incomplete after camera wait')
                camera_wait_ms=(time.perf_counter_ns()-camera_wait_started)/1e6
                observed=[joint_raw_to_nominal_rad(v) for v in start_joints]+[gripper_raw_to_nominal_m_divided(state['gripper']['position_raw'])]
                observation=build_piper_observation(cv2.cvtColor(images['base']['frame'],cv2.COLOR_BGR2RGB),cv2.cvtColor(images['wrist']['frame'],cv2.COLOR_BGR2RGB),observed,PIPER_PROMPT)
                started=time.perf_counter_ns()
                # Keep the sole CAN owner receiving while the network inference runs.
                with ThreadPoolExecutor(max_workers=1) as pool:
                    future=pool.submit(client.infer,observation)
                    while not future.done():
                        robot.poll(0.01)
                        if time.perf_counter_ns()-started>250_000_000:
                            raise TimeoutError('inference/source age exceeds 250 ms')
                    prediction=future.result()
                received=time.perf_counter_ns()
                actions=np.asarray(prediction.get('actions'))
                if actions.shape!=(16,7) or not np.isfinite(actions).all():raise ValueError('invalid model output')
                robot.poll(0.01)
                snapshot=robot.commissioning_snapshot(**site);fresh=robot.state()
                now=time.perf_counter_ns()
                if now-started>250_000_000:raise TimeoutError('inference/source age exceeds 250 ms')
                if any(abs(a-b)>50 for a,b in zip(start_joints,snapshot.joint_raw_0p001deg)):raise RuntimeError('robot moved during inference')
                if abs(fresh['gripper']['position_raw']-state['gripper']['position_raw'])>1000:raise RuntimeError('gripper moved during inference')
                index=int((now-started)//100_000_000)
                if index>=16:raise TimeoutError('action horizon expired')
                current=[joint_raw_to_nominal_rad(v) for v in snapshot.joint_raw_0p001deg]+[gripper_raw_to_nominal_m_divided(fresh['gripper']['position_raw'])]
                bounded=limit_target(current,actions[index].tolist(),joint_step_deg=joint_step_deg)
                targets=tuple(nominal_rad_to_joint_raw(v) for v in bounded['bounded_target'][:6]);gripper=nominal_m_to_gripper_raw(bounded['bounded_target'][6])
                if envelope_raw is not None and any(abs(a-b)>envelope_raw for a,b in zip(targets,origin)):raise ValueError('target exceeds bounded cumulative envelope')
                if time.perf_counter_ns()>=deadline:break
                batch=BoundedTrialBatch(snapshot.sequence,snapshot.joint_raw_0p001deg,targets,fresh['gripper']['position_raw'],gripper,'trial-'+uuid.uuid4().hex,min(started+250_000_000,deadline),joint_step_limit_raw=round(joint_step_deg*1000),gripper_step_limit_raw=None)
                record=dict(step=step,request_id=batch.prediction_request_id,action_index=index,inference_ms=(received-started)/1e6,camera_wait_ms=camera_wait_ms,bounded=bounded,target_joint_raw=list(targets),target_gripper_raw=gripper)
                report['steps'].append(record)
                writes=robot.send_bounded_model_trial(snapshot=snapshot,batch=batch,operator_approved=True)
                record['adapter_accepted_frames']=[dict(can_id=w.arbitration_id,data_hex=w.data.hex(),monotonic_ns=w.adapter_accepted_monotonic_ns) for w in writes]
                checked=robot.wait_joint_target(targets,timeout_s=5,tolerance_raw=round(joint_tolerance_deg*1000))
                record['feedback_check']=checked
                if checked['status']!='JOINT_FEEDBACK_MATCH':raise RuntimeError('joint tracking unconfirmed; no retry')
                robot.poll(0.35);after=robot.state();record['final_state']=after
                if not after['complete_fresh'] or after['error_code'] or after['ctrl_mode_raw']!=1 or not all(after['motor_enabled']):raise RuntimeError('post-write robot fault')
                if abs(after['gripper']['position_raw']-gripper)>1000:raise RuntimeError('gripper tracking unconfirmed; no retry')
            (video_dir/'progress.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            # Release the previous PyUSB context before reopening for a new preflight.
            import gc
            del robot,bridge
            gc.collect()
            time.sleep(0.5)
        report['status']='TIME_BOUNDED_TRIAL_COMPLETE' if long_loop else 'BOUNDED_UPDATES_FEEDBACK_MATCH'
        report['loop_elapsed_s']=(time.perf_counter_ns()-loop_started)/1e9
    except Exception as e:
        report.update(status='STOPPED_NO_RETRY',error_type=type(e).__name__,reason=str(e))
    finally:
        stop.set()
        for t in threads:t.join(timeout=2)
        if any(t.is_alive() for t in threads):
            report.update(status='STOPPED_NO_RETRY',reason='camera reader did not stop')
        else:
            for writer in writers.values():writer.release()
            for log in logs.values():log.close()
        report['video_frames']=counts
        report['timestamp_note']='Host arrival timestamps; AVI playback uses fixed 30 fps, not exposure synchronization.'
        (video_dir/'progress.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--identity-lock',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=8020)
    p.add_argument('--updates',type=int,choices=(1,2,3),default=3)
    p.add_argument('--site-cleared-three-updates-authorized',action='store_true')
    p.add_argument('--joint-step-deg',type=float,choices=(1,5,10,20,60),default=60)
    p.add_argument('--joint-tolerance-deg',type=float,choices=(0.1,0.5),default=0.5)
    p.add_argument('--long-loop',action='store_true')
    p.add_argument('--duration',type=float,default=120)
    p.add_argument('--site-cleared-long-loop-authorized',action='store_true')
    a=p.parse_args()
    if a.long_loop:
        if not a.site_cleared_long_loop_authorized:raise PermissionError('explicit long-loop/site authorization required')
        if not 0<a.duration<=120:raise ValueError('duration must be in (0,120]')
    elif not a.site_cleared_three_updates_authorized:raise PermissionError('explicit trial/site authorization required')
    with a.output.open('x',encoding='utf-8') as f:
        json.dump({'status':'STARTED'},f);f.flush()
        result=run_bounded(worker,args=(a.identity_lock,a.output,a.port,120 if a.long_loop else a.updates,a.duration if a.long_loop else 20,a.long_loop,a.joint_step_deg,a.joint_tolerance_deg),timeout_s=a.duration+35 if a.long_loop else 40)
        report=result['result'] if result['status']=='COMPLETE' else result
        f.seek(0);f.truncate();json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
