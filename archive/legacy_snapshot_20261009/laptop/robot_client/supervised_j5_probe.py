"""One supervised J5 +1.0 degree diagnostic from the stopped restore state.

No enable, mode, home, gripper, reset, or stop command is sent. Not a policy
or replay interface. The operator must remain by the physical E-stop.
"""

from __future__ import annotations

import argparse
import json
import struct
import time

from laptop.diagnostics.probe_piper_feedback_ack_only import _ensure_libusb_backend
from laptop.robot_client.hil_guard import reject_legacy_raw_can_tx
from laptop.robot_client.supervised_j1_step_trial import Observed, _read_for
from laptop.robot_client.supervised_restore_start import checked_feedback

EXPECTED_START_RAW = (1009, 578, -542, 1030, 0, -1500)


def execute(*, operator_confirmed: bool) -> dict:
    reject_legacy_raw_can_tx("supervised_j5_probe")
    if not operator_confirmed:
        raise PermissionError("supervised J5 +1.0 degree authorization required")
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
    sent = 0
    before = Observed()
    after = Observed()
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        _read_for(device, 1.0, before)
        current = checked_feedback(before, time.perf_counter_ns())
        recent = [q for ts, q in before.joint_history
                  if time.perf_counter_ns() - ts <= 300_000_000]
        if len(recent) < 2 or any(any(abs(a - b) > 50 for a, b in zip(q, EXPECTED_START_RAW))
                                  for q in recent):
            raise RuntimeError("live start differs from stopped restore state")
        target = (*current[:4], current[4] + 1000, current[5])
        if not 950 <= target[4] <= 1050:
            raise RuntimeError("unexpected J5 target")
        allowed_echoes = set()
        for pair in range(3):
            can_id = 0x155 + pair
            payload = struct.pack(">ii", *target[2 * pair:2 * pair + 2])
            if device.send(GsUsbFrame(can_id=can_id, data=payload)) is not True:
                raise RuntimeError("CAN send failed")
            allowed_echoes.add((can_id, payload))
            sent += 1
        deadline = time.perf_counter() + 3.0
        final = None
        while time.perf_counter() < deadline:
            _read_for(device, 0.05, after)
            if len(after.latest) < 10:
                continue
            final = checked_feedback(after, time.perf_counter_ns(), frozenset(allowed_echoes))
            if (not all(min(a, b) - 200 <= q <= max(a, b) + 200
                        for q, a, b in zip(final, current, target))):
                raise RuntimeError("J5 probe left bounded feedback envelope")
            if abs(final[4] - target[4]) <= 150 and all(
                abs(q - a) <= 150 for i, (q, a) in enumerate(zip(final, current)) if i != 4
            ):
                break
        success = bool(final and abs(final[4] - target[4]) <= 150
                       and all(abs(q - a) <= 150 for i, (q, a) in enumerate(zip(final, current))
                               if i != 4))
        return {
            "status": "J5_FEEDBACK_MATCH" if success else "J5_FEEDBACK_MISMATCH_NO_MORE_COMMANDS",
            "start_raw_0p001deg": list(current),
            "target_raw_0p001deg": list(target),
            "final_raw_0p001deg": list(final) if final else None,
            "can_data_tx_calls": sent,
            "other_axis_target_held": True,
            "software_stop_verified": False,
            "full_trajectory_control_allowed": False,
        }
    finally:
        if started:
            device.stop()  # USB close only; not a robot stop.


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-j5-probe", action="store_true")
    args = parser.parse_args()
    result = execute(operator_confirmed=args.operator_confirmed_j5_probe)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["status"] == "J5_FEEDBACK_MATCH" else 1


if __name__ == "__main__":
    raise SystemExit(main())
