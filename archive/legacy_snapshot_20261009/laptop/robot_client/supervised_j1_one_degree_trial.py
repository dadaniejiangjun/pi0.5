"""One authorized, supervised J1 +1 degree trial, never a trajectory sender.

This requires a human at the physical emergency stop. It sends no enable,
home, mode, gripper, hold, or stop command. Closing USB is not a robot stop.
"""

from __future__ import annotations

import argparse
import json
import struct
import time

from laptop.diagnostics.probe_motor_enable_ack_only import FEEDBACK_IDS, decode_motor_enable
from laptop.diagnostics.probe_piper_feedback_ack_only import _ensure_libusb_backend
from laptop.robot_client.hil_guard import reject_legacy_raw_can_tx
from laptop.robot_client.supervised_j1_step_trial import (
    MAX_FEEDBACK_AGE_NS, OBSERVE_AFTER_S, PRECHECK_S, REQUIRED_IDS,
    Observed, _read_for,
)


def preflight(observed: Observed, now_ns: int) -> tuple[int, ...]:
    if observed.command_ids:
        raise RuntimeError("another controller sent CAN control frames")
    if any(can not in observed.latest or now_ns - observed.latest[can][0] > MAX_FEEDBACK_AGE_NS
           or observed.latest[can][0] > now_ns for can in REQUIRED_IDS):
        raise RuntimeError("required feedback missing or stale")
    status = observed.latest[0x2A1][1]
    if (status[0] != 0x01 or status[1] != 0 or status[2] != 0x01
            or status[6:8] != b"\x00\x00"):
        raise RuntimeError("CAN joint mode or arm health not ready")
    if any(not decode_motor_enable(can, observed.latest[can][1])[1] for can in FEEDBACK_IDS):
        raise RuntimeError("not all six motors enabled")
    recent = [joints for ts, joints in observed.joint_history if now_ns - ts <= 300_000_000]
    if len(recent) < 2 or any(not (20 <= q[0] <= 60)
                              or any(abs(axis) > 20 for axis in q[1:]) for q in recent):
        raise RuntimeError("J1 is not stable near prior +0.039 degree feedback")
    return recent[-1]


def execute(*, operator_confirmed: bool) -> dict:
    reject_legacy_raw_can_tx("supervised_j1_one_degree_trial")
    if not operator_confirmed:
        raise PermissionError("fresh operator approval for J1 +1.0 degree required")
    from gs_usb.gs_usb import GsUsb, GsUsbFrame

    _ensure_libusb_backend()
    devices = [d for d in GsUsb.scan()
               if (d.gs_usb.idVendor, d.gs_usb.idProduct) == (0x1D50, 0x606F)]
    if len(devices) != 1:
        raise RuntimeError("expected exactly one candleLight USB-CAN adapter")
    device = devices[0]
    if not device.set_bitrate(1_000_000):
        raise RuntimeError("1 Mbit/s bitrate unsupported")
    started = False
    sent_ids: list[int] = []
    observed_before = Observed()
    observed_after = Observed()
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        _read_for(device, PRECHECK_S, observed_before)
        current = preflight(observed_before, time.perf_counter_ns())
        target = (current[0] + 1000, *current[1:])  # exactly +1.000 degree J1
        if not (1020 <= target[0] <= 1060) or any(abs(q) > 20 for q in target[1:]):
            raise RuntimeError("unexpected joint target")
        for pair in range(3):
            can_id = 0x155 + pair
            frame_data = struct.pack(">ii", *target[2 * pair:2 * pair + 2])
            device.send(GsUsbFrame(can_id=can_id, data=frame_data))
            sent_ids.append(can_id)
        _read_for(device, OBSERVE_AFTER_S, observed_after)
    finally:
        if started:
            device.stop()  # USB close only; not a hold or emergency stop.

    now_ns = time.perf_counter_ns()
    latest = observed_after.joint_history[-1][1] if observed_after.joint_history else None
    status_entry = observed_after.latest.get(0x2A1)
    healthy = bool(status_entry and now_ns - status_entry[0] <= MAX_FEEDBACK_AGE_NS
                   and status_entry[1][0] == 0x01 and status_entry[1][1] == 0
                   and status_entry[1][2] == 0x01 and status_entry[1][6:8] == b"\x00\x00")
    enabled = all(can in observed_after.latest
                  and now_ns - observed_after.latest[can][0] <= MAX_FEEDBACK_AGE_NS
                  and decode_motor_enable(can, observed_after.latest[can][1])[1]
                  for can in FEEDBACK_IDS)
    joints_fresh = bool(observed_after.joint_history
                        and now_ns - observed_after.joint_history[-1][0] <= MAX_FEEDBACK_AGE_NS)
    trajectory_safe = all(
        -20 <= joints[0] <= target[0] + 200
        and all(abs(axis) <= 50 for axis in joints[1:])
        for _, joints in observed_after.joint_history
    )
    matched = bool(latest and joints_fresh and healthy and enabled and trajectory_safe
                   and abs(latest[0] - target[0]) <= 100
                   and all(abs(q) <= 50 for q in latest[1:]))
    return {
        "status": "SINGLE_STEP_FEEDBACK_MATCH" if matched else "FEEDBACK_MISMATCH_NO_MORE_COMMANDS",
        "start_joint_raw_0p001deg": list(current),
        "target_joint_raw_0p001deg": list(target),
        "last_joint_raw_0p001deg": list(latest) if latest else None,
        "command_ids_sent": [f"0x{can_id:03X}" for can_id in sent_ids],
        "can_data_tx_calls": len(sent_ids),
        "post_healthy": healthy,
        "post_all_six_enabled": enabled,
        "post_joint_feedback_fresh": joints_fresh,
        "observed_within_step_envelope": trajectory_safe,
        "software_stop_verified": False,
        "full_trajectory_control_allowed": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-j1-plus-one-degree", action="store_true")
    args = parser.parse_args()
    result = execute(operator_confirmed=args.operator_confirmed_j1_plus_one_degree)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "SINGLE_STEP_FEEDBACK_MATCH" else 1


if __name__ == "__main__":
    raise SystemExit(main())
