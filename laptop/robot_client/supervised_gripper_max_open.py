"""One explicitly authorized nominal 70mm gripper opening at 0.1Nm."""
import argparse
import json
from pathlib import Path
from laptop.robot_client.piper_session import PiperSession
from laptop.robot_client.supervised_near_zero_home import healthy
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


def execute(log_path):
    bridge=SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=GsUsbPiperCanTransport(),limits=UNVERIFIED_ANALYSIS_LIMITS,
        supervised_j1_commissioning_tx_enabled=True)
    site=dict(physical_estop_ready=True,low_speed_confirmed=True,motion_envelope_cleared=True)
    with PiperSession(log_path,bridge=bridge) as robot:
        robot.poll(0.6)
        before=robot.state()
        if not healthy(before):
            raise PermissionError('healthy fresh state required')
        receipt=bridge.send_gripper_opening_once(operator_approved=True,delta_raw=0,
            effort_raw=100,open_to_max=True,**site)
        robot.poll(3)
        after=robot.state()
        held=healthy(after) and all(abs(a-b)<=100 for a,b in zip(before['joint_raw_0p001deg'],after['joint_raw_0p001deg']))
        matched=held and abs(after['gripper']['position_raw']-70000)<=100
        return dict(status='MAX_OPEN_FEEDBACK_MATCH' if matched else 'MAX_OPEN_UNCONFIRMED_NO_RETRY',
            target_gripper_raw=70000,effort_raw=100,source_feedback=before,final_feedback=after,
            adapter_accepted_frame=dict(can_id=hex(receipt.arbitration_id),data_hex=receipt.data.hex(),
                                       accepted_monotonic_ns=receipt.adapter_accepted_monotonic_ns),
            held_joints_confirmed=held,joint_targets_sent=False,automatic_retry=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-open-authorized-site-cleared',action='store_true')
    args=parser.parse_args()
    if not args.max_open_authorized_site_cleared:
        return 2
    with args.output.open('x',encoding='utf-8') as handle:
        handle.write('{"status":"STARTED"}\n');handle.flush()
        try:
            result=execute(args.output.with_suffix('.can.jsonl'))
        except Exception as exc:
            result=dict(status='STOPPED_NO_RETRY',reason=str(exc),error_type=type(exc).__name__)
        handle.seek(0);handle.truncate();json.dump(result,handle,indent=2)
    print(json.dumps(result,indent=2))
    return 0 if result['status']=='MAX_OPEN_FEEDBACK_MATCH' else 1


if __name__=='__main__':
    raise SystemExit(main())
