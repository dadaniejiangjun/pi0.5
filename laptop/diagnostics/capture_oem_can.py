"""Observe OEM candle_frame_send for exactly a 5-second armed window."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
EXPECTED = Path(r'E:\01_NUS\02_5003\10_gripper\01_Pipper\ArmRobotUA_V1.6.3.260828_release.exe')


def decode_gripper(row):
    if row.get('can_id_raw') != 0x159 or row.get('dlc') != 8:
        return None
    target, effort, code, zero = struct.unpack('>iHBB', bytes.fromhex(row['data_hex']))
    return dict(target_raw=target, nominal_opening_mm=target/1000,
                effort_raw=effort, nominal_effort_nm=effort/1000,
                control_code=code, set_zero_code=zero)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid',type=int,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--probe-hook',action='store_true',help='attach/unload unarmed hook; no capture window')
    parser.add_argument('--countdown',type=int,default=0,choices=range(0,11),help='preparation seconds before fixed 5-second window')
    args=parser.parse_args()
    output=args.output_dir
    output.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(ROOT/'cache/tooldeps/oem_capture'))
    import frida
    session=None
    script=None
    ended=threading.Event()
    rows=[]
    lock=threading.Lock()
    report=dict(schema='piper_oem_send_capture_v1',status='STARTED',duration_s=5,
                can_api_called_by_logger=False, robot_commands_sent_by_logger=False,
                hook_modifies_process_memory=True, function_arguments_modified=False,
                timestamp_kind='source_QueryPerformanceCounter_at_send_entry',
                robot_execution_proven=False,frida_version=frida.__version__)
    try:
        session=frida.attach(args.pid)
        check=session.create_script('rpc.exports={path(){return Process.enumerateModules()[0].path;}};')
        check.load()
        actual=Path(check.exports_sync.path()).resolve()
        check.unload()
        if actual != EXPECTED.resolve():
            raise ValueError('PID does not belong to the verified OEM executable')
        report['exe_sha256']=hashlib.sha256(EXPECTED.read_bytes()).hexdigest()
        source=Path(__file__).with_name('oem_can_send_hook.js').read_text(encoding='utf-8')
        script=session.create_script(source)
        with (output/'send_frames.jsonl').open('x',encoding='utf-8') as log:
            def message(message,data):
                row=message.get('payload',{}) if message['type']=='send' else {'event':'script_error','detail':message}
                row={**row,'logger_received_monotonic_ns':time.perf_counter_ns()}
                if row.get('event')=='oem_send':
                    row['decoded_gripper']=decode_gripper(row)
                with lock:
                    rows.append(row);log.write(json.dumps(row)+'\n');log.flush()
                if row.get('event') in ('capture_end','script_error'):
                    ended.set()
            script.on('message',message)
            script.load()
            report['hook']=script.exports_sync.info()
            if args.probe_hook:
                report['status']='HOOK_READY_UNARMED_NO_CAPTURE'
            else:
                print('READY: press Enter when prepared; record window is 5 seconds.',flush=True)
                input()
                for remaining in range(args.countdown,0,-1):
                    print(f'PREPARE: {remaining}s until 5-second capture',flush=True)
                    time.sleep(1)
                script.exports_sync.arm()
                print('\aCAPTURE_START: operate the gripper in OEM UI now; 5 seconds.',flush=True)
                if not ended.wait(7):
                    raise TimeoutError('5-second capture end was not received')
                print('CAPTURE_END',flush=True)
                report['status']='CAPTURED' if any(r.get('event')=='oem_send' for r in rows) else 'NO_SEND_CALLS_OBSERVED'
            script.exports_sync.stop()
        report['can_id_counts']=dict(Counter(hex(r['can_id_raw']) for r in rows if r.get('event')=='oem_send'))
        report['gripper_commands']=[r for r in rows if r.get('decoded_gripper')]
        report['alternate_send_calls']=[r for r in rows if r.get('event')=='alternate_send_call']
        if report['alternate_send_calls'] and report['status']=='NO_SEND_CALLS_OBSERVED':
            report['status']='ALTERNATE_SEND_PATH_OBSERVED_NOT_DECODED'
        report['errors']=[r for r in rows if r.get('event') in ('decode_error','script_error')]
        if report['errors']:
            report['status']='CAPTURE_ERRORS'
    except Exception as exc:
        report.update(status='STOPPED',error=str(exc))
    finally:
        if script is not None:
            try: script.unload()
            except Exception: pass
        if session is not None:
            try: session.detach()
            except Exception: pass
        report['finished_utc']=datetime.now(timezone.utc).isoformat()
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    return 0 if report['status'] in ('CAPTURED','HOOK_READY_UNARMED_NO_CAPTURE') else 1


if __name__=='__main__':
    raise SystemExit(main())
