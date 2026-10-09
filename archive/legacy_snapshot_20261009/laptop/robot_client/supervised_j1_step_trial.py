"""One-off supervised J1 +0.1 degree trial; NEVER a policy or trajectory sender.

This is the first physical command test, not a SafetyBridge. It does not
enable, home, change mode, operate the gripper, or implement a verified
software stop. A human must remain at the physical emergency stop.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import struct
import time

from laptop.diagnostics.probe_motor_enable_ack_only import FEEDBACK_IDS, decode_motor_enable
from laptop.diagnostics.probe_piper_feedback_ack_only import _ensure_libusb_backend
from laptop.robot_client.hil_guard import reject_legacy_raw_can_tx
from laptop.robot_client.joint_command_preview import preview_single_joint_step


REQUIRED_IDS = frozenset((0x2A1, 0x2A5, 0x2A6, 0x2A7, *FEEDBACK_IDS))
MAX_FEEDBACK_AGE_NS = 120_000_000
PRECHECK_S = 1.0
OBSERVE_AFTER_S = 2.0
TARGET_J1_RAW = 100  # 0.100 degrees
COMPETING_COMMAND_IDS = frozenset((*range(0x150, 0x160), 0x471))


@dataclass
class Observed:
    latest: dict[int, tuple[int, bytes]] = field(default_factory=dict)
    command_ids: set[int] = field(default_factory=set)
    command_frames: list[tuple[int, bytes, int]] = field(default_factory=list)
    joint_history: list[tuple[int, tuple[int, ...]]] = field(default_factory=list)

    def add(self, can_id: int, payload: bytes, now_ns: int) -> None:
        if can_id in COMPETING_COMMAND_IDS:
            self.command_ids.add(can_id)
            self.command_frames.append((can_id, payload, now_ns))
        if can_id not in REQUIRED_IDS or len(payload) != 8:
            return
        self.latest[can_id] = (now_ns, payload)
        if all(can in self.latest for can in (0x2A5, 0x2A6, 0x2A7)):
            pair_values = [struct.unpack(">ii", self.latest[can][1])
                           for can in (0x2A5, 0x2A6, 0x2A7)]
            self.joint_history.append((now_ns, tuple(v for pair in pair_values for v in pair)))


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
    for can_id in FEEDBACK_IDS:
        _, enabled, _ = decode_motor_enable(can_id, observed.latest[can_id][1])
        if not enabled:
            raise RuntimeError("not all six motors enabled")
    if len(observed.joint_history) < 2:
        raise RuntimeError("joint feedback history missing")
    recent = [joints for ts, joints in observed.joint_history
              if now_ns - ts <= 300_000_000]
    if len(recent) < 2 or any(any(abs(q) > 20 for q in joints) for joints in recent):
        raise RuntimeError("zero pose not stable within 0.020 degrees")
    current = recent[-1]
    preview = preview_single_joint_step([q / 1000 for q in current],
                                        joint_index=0, delta_deg=0.1)
    if preview.target_deg != (0.1, 0.0, 0.0, 0.0, 0.0, 0.0):
        raise RuntimeError("unexpected target after quantization")
    return current


def _read_for(device: object, seconds: float, observed: Observed) -> None:
    from gs_usb.gs_usb import GsUsbFrame

    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        frame = GsUsbFrame()
        try:
            ok = device.read(frame, timeout_ms=50)
        except struct.error:
            continue
        if ok and not frame.is_error_frame and frame.can_dlc == 8:
            observed.add(frame.arbitration_id, bytes(frame.data[:8]), time.perf_counter_ns())


def execute(*, operator_confirmed: bool) -> dict:
    reject_legacy_raw_can_tx("supervised_j1_step_trial")
    if not operator_confirmed:
        raise PermissionError("operator confirmation required immediately before the live step")
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
    observed = Observed()
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        _read_for(device, PRECHECK_S, observed)
        current = preflight(observed, time.perf_counter_ns())
        preview = preview_single_joint_step([q / 1000 for q in current],
                                            joint_index=0, delta_deg=0.1)
        for frame in preview.frames:
            device.send(GsUsbFrame(can_id=frame.arbitration_id, data=frame.data))
            sent_ids.append(frame.arbitration_id)
        observed_after = Observed()
        _read_for(device, OBSERVE_AFTER_S, observed_after)
    finally:
        if started:
            device.stop()  # Closes USB-CAN; it is NOT a robot hold or E-stop.
    post_now_ns = time.perf_counter_ns()
    latest = observed_after.joint_history[-1][1] if observed_after.joint_history else None
    post_status = observed_after.latest.get(0x2A1)
    post_healthy = bool(post_status and post_now_ns - post_status[0] <= MAX_FEEDBACK_AGE_NS
                        and post_status[1][0] == 0x01
                        and post_status[1][1] == 0
                        and post_status[1][6:8] == b"\x00\x00")
    post_enabled = all(
        can_id in observed_after.latest
        and post_now_ns - observed_after.latest[can_id][0] <= MAX_FEEDBACK_AGE_NS
        and decode_motor_enable(can_id, observed_after.latest[can_id][1])[1]
        for can_id in FEEDBACK_IDS
    )
    latest_joints_fresh = bool(observed_after.joint_history and
                               post_now_ns - observed_after.joint_history[-1][0]
                               <= MAX_FEEDBACK_AGE_NS)
    any_overshoot = any(abs(joints[0]) > 250 or any(abs(q) > 50 for q in joints[1:])
                        for _, joints in observed_after.joint_history)
    ok = (latest is not None and latest_joints_fresh and post_healthy and post_enabled
          and not any_overshoot
          and abs(latest[0] - TARGET_J1_RAW) <= 50
          and all(abs(q) <= 50 for q in latest[1:]))
    return {
        "status": "SINGLE_STEP_FEEDBACK_MATCH" if ok else "FEEDBACK_MISMATCH_NO_MORE_COMMANDS",
        "command_ids_sent": [f"0x{can_id:03X}" for can_id in sent_ids],
        "can_data_tx_calls": len(sent_ids),
        "target_joint_raw_0p001deg": [100, 0, 0, 0, 0, 0],
        "last_joint_raw_0p001deg": list(latest) if latest else None,
        "post_healthy": post_healthy,
        "post_all_six_enabled": post_enabled,
        "post_joint_feedback_fresh": latest_joints_fresh,
        "observed_overshoot_or_other_joint_motion": any_overshoot,
        "software_stop_verified": False,
        "operator_physical_estop_required": True,
        "full_trajectory_control_allowed": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-supervised-j1-step", action="store_true")
    args = parser.parse_args()
    report = execute(operator_confirmed=args.operator_confirmed_supervised_j1_step)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "SINGLE_STEP_FEEDBACK_MATCH" else 1


if __name__ == "__main__":
    raise SystemExit(main())
