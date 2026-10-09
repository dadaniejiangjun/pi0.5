"""Bounded test using observed OEM effort=0 and repeated gripper targets."""
import argparse
import json
import time
from pathlib import Path
from laptop.robot_client.piper_session import PiperSession
from laptop.robot_client.supervised_near_zero_home import healthy
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.gripper_commissioning import OemGripperTestFrame


def execute(log_path,report):
    bridge=SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=GsUsbPiperCanTransport(),limits=UNVERIFIED_ANALYSIS_LIMITS,
        supervised_j1_commissioning_tx_enabled=True)
    with PiperSession(log_path,bridge=bridge) as robot:
        robot.poll(0.6)
        bridge.gripper_opening_snapshot(physical_estop_ready=True,low_speed_confirmed=True,
                                       motion_envelope_cleared=True)
        initial=robot.state()
        if not healthy(initial): raise PermissionError('healthy complete state required')
        report['initial_feedback']=initial
        start=time.perf_counter_ns()
        deadline=start+5_000_000_000
        while time.perf_counter_ns()<deadline-20_000_000:
            state=robot.state()
            report['final_feedback']=state
            if not healthy(state) or any(abs(a-b)>100 for a,b in zip(initial['joint_raw_0p001deg'],state['joint_raw_0p001deg'])):
                raise PermissionError('feedback stale, fault or unexpected joint movement')
            position=state['gripper']['position_raw']
            # 0x47E query in this session preparation read max_range_config_mm=100.
            # This is a controller feedback bound, not measured physical travel.
            if not 0 <= position <= 100000: raise PermissionError('gripper feedback outside read-back configured range')
            if abs(position-54015)<=500:
                report['status']='OEM_GRIPPER_TARGET_FEEDBACK_MATCH'
                break
            command=OemGripperTestFrame(start,state['channel_timestamps_ns']['0x2a8'])
            robot._send('oem_gripper',command,lambda:bridge.send_oem_gripper_test_frame(command=command,operator_approved=True))
            report['sent_frames']+=1
            robot.poll(0.013)
        else:
            report['status']='WINDOW_ENDED_TARGET_UNCONFIRMED'
        robot.poll(0.5)
        report['final_feedback']=robot.state()
        report['elapsed_s']=(time.perf_counter_ns()-start)/1e9
        return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--site-cleared-oem-test-authorized',action='store_true')
    args=p.parse_args()
    if not args.site_cleared_oem_test_authorized:return 2
    report=dict(status='STARTED',target_raw=54015,effort_raw=0,control_code=1,set_zero_code=0,
                send_window_max_s=5,requested_interval_s=0.013,sent_frames=0,joint_targets_sent=False,
                automatic_retry=False,unit_scale_physically_verified=False)
    report.update(controller_readback_max_range_mm=100,
                  operator_reported_ui_range_mm=70,operator_reported_physical_travel_mm=[0,10],
                  range_disagreement_unresolved=True)
    with args.output.open('x',encoding='utf-8') as handle:
        json.dump(report,handle);handle.flush()
        try:execute(args.output.with_suffix('.can.jsonl'),report)
        except Exception as exc:report.update(status='STOPPED_NO_RETRY',error_type=type(exc).__name__,reason=str(exc))
        handle.seek(0);handle.truncate();json.dump(report,handle,indent=2)
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='OEM_GRIPPER_TARGET_FEEDBACK_MATCH' else 1


if __name__=='__main__':raise SystemExit(main())
