"""One authorized PiPER V2 gripper opening to the recorded 7.1 mm start.

The 0x159 command necessarily contains a gripper-enable bit. It does not
enable arm joints, home, alter mode, or replay a trajectory. A human must
remain beside the physical E-stop. USB close is not an emergency stop.
"""

from __future__ import annotations

import argparse
import json
import struct
import time

from laptop.diagnostics.probe_piper_feedback_ack_only import _ensure_libusb_backend
from laptop.robot_client.hil_guard import reject_legacy_raw_can_tx
from laptop.robot_client.gripper_command_preview import preview_opening
from laptop.robot_client.supervised_j1_step_trial import Observed
from laptop.robot_client.supervised_restore_start import checked_feedback

EXPECTED_JOINT_RAW = (1559, 8542, -8391, 13687, 4937, -19289)
EXPECTED_GRIPPER_RAW = 300
TARGET_GRIPPER_RAW = 7100


def _read_for(device: object, seconds: float, observed: Observed,
              gripper: list[tuple[int, int, int]]) -> None:
    from gs_usb.gs_usb import GsUsbFrame

    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        frame = GsUsbFrame()
        try:
            ok = device.read(frame, timeout_ms=50)
        except struct.error:
            continue
        if not ok or frame.is_error_frame or frame.can_dlc != 8:
            continue
        now_ns = time.perf_counter_ns()
        payload = bytes(frame.data[:8])
        observed.add(frame.arbitration_id, payload, now_ns)
        if frame.arbitration_id == 0x2A8:
            gripper.append((now_ns, struct.unpack(">i", payload[:4])[0], payload[6]))


def execute(*, operator_confirmed: bool) -> dict:
    reject_legacy_raw_can_tx("supervised_restore_gripper")
    if not operator_confirmed:
        raise PermissionError("explicit supervised 7.1 mm gripper opening approval required")
    from gs_usb.gs_usb import GsUsb, GsUsbFrame

    preview = preview_opening(TARGET_GRIPPER_RAW, effort_raw=0)
    _ensure_libusb_backend()
    devices = [d for d in GsUsb.scan()
               if (d.gs_usb.idVendor, d.gs_usb.idProduct) == (0x1D50, 0x606F)]
    if len(devices) != 1:
        raise RuntimeError("expected exactly one candleLight USB-CAN adapter")
    device = devices[0]
    if not device.set_bitrate(1_000_000):
        raise RuntimeError("1 Mbit/s bitrate unsupported")
    started = False
    sent = False
    observed = Observed()
    gripper: list[tuple[int, int, int]] = []
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        _read_for(device, 1.0, observed, gripper)
        current_joints = checked_feedback(observed, time.perf_counter_ns())
        if any(abs(a - b) > 100 for a, b in zip(current_joints, EXPECTED_JOINT_RAW)):
            raise RuntimeError("joint start pose drifted")
        if not gripper or time.perf_counter_ns() - gripper[-1][0] > 120_000_000:
            raise RuntimeError("gripper feedback stale")
        current_gripper = gripper[-1][1]
        if abs(current_gripper - EXPECTED_GRIPPER_RAW) > 200:
            raise RuntimeError("gripper is not at expected 0.3 mm start")
        if device.send(GsUsbFrame(can_id=preview.arbitration_id, data=preview.data)) is not True:
            raise RuntimeError("gripper CAN send failed")
        sent = True
        allowed = frozenset({(preview.arbitration_id, preview.data)})
        deadline = time.perf_counter() + 3.0
        reached = False
        while time.perf_counter() < deadline:
            _read_for(device, 0.05, observed, gripper)
            joints = checked_feedback(observed, time.perf_counter_ns(), allowed)
            if any(abs(a - b) > 200 for a, b in zip(joints, current_joints)):
                raise RuntimeError("arm joint moved during gripper-only command")
            if not gripper or time.perf_counter_ns() - gripper[-1][0] > 120_000_000:
                raise RuntimeError("gripper feedback stale after command")
            value = gripper[-1][1]
            if not current_gripper - 200 <= value <= TARGET_GRIPPER_RAW + 200:
                raise RuntimeError("gripper feedback outside bounded opening range")
            if abs(value - TARGET_GRIPPER_RAW) <= 200:
                reached = True
                break
        return {
            "status": "GRIPPER_START_RESTORED_SUPERVISED" if reached
                      else "GRIPPER_FEEDBACK_MISMATCH_NO_MORE_COMMANDS",
            "start_gripper_raw_0p001mm": current_gripper,
            "target_gripper_raw_0p001mm": TARGET_GRIPPER_RAW,
            "final_gripper_raw_0p001mm": gripper[-1][1] if gripper else None,
            "last_gripper_status_code": gripper[-1][2] if gripper else None,
            "can_data_tx_calls": int(sent),
            "gripper_enable_bit_sent": sent,
            "arm_joint_command_sent": False,
            "software_stop_verified": False,
            "full_trajectory_control_allowed": False,
        }
    finally:
        if started:
            device.stop()  # Adapter close is not a hold or E-stop.


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-gripper-start-opening", action="store_true")
    args = parser.parse_args()
    result = execute(operator_confirmed=args.operator_confirmed_gripper_start_opening)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["status"] == "GRIPPER_START_RESTORED_SUPERVISED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
