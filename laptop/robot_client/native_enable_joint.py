"""One-use native enable at measured pose, without reset or fault clearing."""
import argparse
import json
import struct
import time
from pathlib import Path
from laptop.robot_client.piper_session import PiperSession
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.transport.socketcan_manual import SocketCanManualAdapter
from laptop.transport.position_commands import CanTxFrame
from laptop.transport.manual_gamepad import bounded_hold_joints
from laptop.transport.mode_initialization import ModeInitializationFrame


def execute(joint, output):
    if joint not in range(1, 7):
        raise ValueError("joint must be 1..6")
    report = {"joint": joint, "reset_sent": False, "fault_clear_sent": False}
    with open(str(output)+'.frames.jsonl', 'x') as log:
        def event(x):
            log.write(json.dumps(x)+'\n'); log.flush()
        adapter = SocketCanManualAdapter('can0', transmit_enabled=True, event_sink=event)
        bridge = SafetyBridge(mode=RuntimeMode.POLICY_REJECT_ONLY,
            transport=GsUsbPiperCanTransport(adapter_factory=lambda: adapter),
            limits=UNVERIFIED_ANALYSIS_LIMITS)
        with PiperSession(str(output)+'.can.jsonl', bridge=bridge, log_rx_frames=False) as robot:
            def read():
                robot.poll(.5)
                s = robot.state()
                if (not s['complete_fresh'] or s['ctrl_mode_raw'] != 1
                        or s['arm_status_raw'] or s['error_code']
                        or s['gripper']['status_raw'] & 0xBF
                        or any(c & 0xBF for c in s['motor_status_codes'])
                        or any(not enabled for i, enabled in enumerate(s['motor_enabled']) if i != joint-1)):
                    raise RuntimeError('fresh fault-free CAN_CTRL and other five axes enabled required')
                return s
            initial = read(); stable = read()
            if any(abs(a-b)>100 for a,b in zip(initial['joint_raw_0p001deg'],stable['joint_raw_0p001deg'])):
                raise RuntimeError('arm is not stable; enable withheld')
            report['initial_state'] = initial
            if not stable['motor_enabled'][joint-1]:
                q = bounded_hold_joints(tuple(stable['joint_raw_0p001deg']))
                adapter.send_mode_initialization_frame(ModeInitializationFrame('can_joint'))
                for i in range(3):
                    adapter.send_position_frame(CanTxFrame(0x155+i, struct.pack('>ii', *q[2*i:2*i+2])))
                before = read()
                if any(abs(a-b)>100 for a,b in zip(q,before['joint_raw_0p001deg'])):
                    raise RuntimeError('pose changed before enable')
                adapter._send(CanTxFrame(0x471, bytes([joint, 2, 0, 0, 0, 0, 0, 0])))
            final = read()
            report.update(final_state=final, can_tx_calls=adapter.tx_calls)
            if not all(final['motor_enabled']):
                raise RuntimeError('enable not confirmed; no retry')
            if any(abs(a-b)>500 for a,b in zip(initial['joint_raw_0p001deg'],final['joint_raw_0p001deg'])):
                raise RuntimeError('unexpected pose change after enable')
            report['status'] = 'ALL_SIX_ENABLED_CONFIRMED'
    Path(output).write_text(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--joint', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--site-confirmed', action='store_true', required=True)
    a = p.parse_args()
    print(json.dumps(execute(a.joint, a.output)))
