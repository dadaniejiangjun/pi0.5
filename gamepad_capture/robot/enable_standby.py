"""Reusable bounded standby enable; no reset, disable or fault clearing."""
import json
from pathlib import Path
import struct
import subprocess
import uuid
from laptop.robot_client.piper_session import PiperSession
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.runtime.modes import RuntimeMode
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.socketcan_manual import SocketCanManualAdapter
from laptop.transport.position_commands import CanTxFrame
from laptop.transport.mode_initialization import ModeInitializationFrame
from laptop.transport.manual_gamepad import bounded_hold_joints


def execute(out, *, site_confirmed=False):
    if not site_confirmed:
        raise PermissionError("Explicit onsite confirmation required; no hardware opened")
    out = Path(out)
    if out.exists():
        raise FileExistsError(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    report = dict(status='STARTED', reset_sent=False, disable_sent=False, gripper_tx=False, automatic_retry=False)
    active = subprocess.run(['pgrep', '-af', '/lib/(pi05_piper_bridge/native_(can_state|manual_gamepad)|pi05_piper_gamepad/gamepad_operator|pi05_piper_quest/quest_execution)( |$)'], capture_output=True, text=True)
    if active.returncode != 1:
        raise PermissionError('existing control owner or uncertain process check: ' + active.stdout)
    with open(str(out) + '.frames.jsonl', 'x') as log:
        def event(record):
            log.write(json.dumps(record) + '\n'); log.flush()
        adapter = SocketCanManualAdapter('can0', transmit_enabled=True, event_sink=event)
        bridge = SafetyBridge(mode=RuntimeMode.POLICY_REJECT_ONLY, transport=GsUsbPiperCanTransport(adapter_factory=lambda: adapter), limits=UNVERIFIED_ANALYSIS_LIMITS)
        try:
            with PiperSession(str(out) + '.can.jsonl', bridge=bridge, log_rx_frames=False) as robot:
                def read():
                    robot.poll(.8)
                    s = robot.state(); robot.record_state_snapshot(s)
                    if not s['complete_fresh'] or s['arm_status_raw'] or s['error_code'] or s['gripper']['status_raw'] & 0xBF:
                        raise PermissionError('fresh fault-free feedback required')
                    if any(c & 0xBF for c in s['motor_status_codes']):
                        raise PermissionError('motor fault; no automatic clearing')
                    if any(abs(q) > 90000 for q in s['joint_raw_0p001deg']):
                        raise PermissionError('outside authorized zero start envelope')
                    return s
                a = read(); b = read()
                report['initial_state'] = a
                if a['ctrl_mode_raw'] != 0 or b['ctrl_mode_raw'] != 0 or any(b['motor_enabled']):
                    raise PermissionError('this one-use helper requires standby and all six disabled')
                if any(abs(x-y) > 100 for x,y in zip(a['joint_raw_0p001deg'], b['joint_raw_0p001deg'])):
                    raise PermissionError('pose drifting; enable withheld')
                q = bounded_hold_joints(tuple(b['joint_raw_0p001deg']))
                report['bounded_initial_hold_raw'] = q
                adapter.context = dict(stage='preload_nearest_legal_hold_while_disabled')
                for i in range(3):
                    adapter.send_position_frame(CanTxFrame(0x155+i, struct.pack('>ii', *q[2*i:2*i+2])))
                adapter.context = dict(stage='can_move_j_5_percent')
                adapter.send_mode_initialization_frame(ModeInitializationFrame('can_joint'))
                c = read(); report['before_enable'] = c
                if c['ctrl_mode_raw'] != 1 or c['move_mode_raw'] != 1 or any(c['motor_enabled']):
                    raise PermissionError('mode/disabled state not confirmed')
                if any(abs(x-y)>100 for x,y in zip(b['joint_raw_0p001deg'], c['joint_raw_0p001deg'])):
                    raise PermissionError('unexpected motion before enable')
                adapter.context = dict(stage='official_EnableArm_7_2_once')
                adapter._send(CanTxFrame(0x471, bytes((7,2,0,0,0,0,0,0))))
                d = read(); e = read(); report['final_state'] = e
                if not all(d['motor_enabled']) or not all(e['motor_enabled']):
                    raise PermissionError('all-six enable not confirmed; no retry')
                if any(abs(x-y)>2500 for x,y in zip(q,e['joint_raw_0p001deg'])):
                    raise PermissionError('unexpected enable displacement; no zero')
                report['status'] = 'ALL_SIX_HEALTHY_ENABLED_CAN_MOVE_J'
        except Exception as exc:
            report.update(status='STOPPED_NO_RETRY', reason=str(exc), error_type=type(exc).__name__)
        report['can_tx_calls'] = adapter.tx_calls
    out.write_text(json.dumps(report, indent=2))
    return dict(report_path=str(out), **report)


def main():
    import argparse
    from datetime import datetime, timezone
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site-confirmed", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    out = args.output or Path("cache/native_piper_tool") / (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")+"_"+uuid.uuid4().hex[:8]+"_enable.json")
    report = execute(out, site_confirmed=args.site_confirmed)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ALL_SIX_HEALTHY_ENABLED_CAN_MOVE_J" else 1


if __name__ == "__main__":
    raise SystemExit(main())
