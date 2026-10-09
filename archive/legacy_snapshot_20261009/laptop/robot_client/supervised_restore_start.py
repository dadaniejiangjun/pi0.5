"""One-shot supervised, joint-only return to a recorded episode start.

Not a general trajectory player or policy interface. Each waypoint differs
by at most 1.0 degree per joint. No enable, mode, home, gripper, or software
stop command is sent. The operator must watch and own the physical E-stop.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import time

from laptop.diagnostics.probe_motor_enable_ack_only import FEEDBACK_IDS, decode_motor_enable
from laptop.diagnostics.probe_piper_feedback_ack_only import _ensure_libusb_backend
from laptop.robot_client.hil_guard import reject_legacy_raw_can_tx
from laptop.robot_client.replay_preflight import NOMINAL_JOINT_BOUNDS_DEG
from laptop.robot_client.supervised_j1_step_trial import (
    MAX_FEEDBACK_AGE_NS, REQUIRED_IDS, Observed, _read_for,
)

EPISODE_ID = "cef166e2cc6240b397db241a916ece3a"
START_ROW_PATH = Path("datasets/manifests/generated/cef166e2_preview_2fps/frames.jsonl")
EXPECTED_CURRENT_RAW = (1009, 578, -542, 1030, 1030, -1500)
MAX_SEGMENT_RAW = 1000  # 1.0 degree
ARRIVAL_TOLERANCE_RAW = 150  # 0.15 degree
MAX_PATH_DEVIATION_RAW = 200  # 0.2 degree
STEP_TIMEOUT_S = 2.0
MIN_STEP_DWELL_S = 0.25


def recorded_start_raw(path: Path = START_ROW_PATH) -> tuple[int, ...]:
    with path.open(encoding="utf-8") as handle:
        row = json.loads(handle.readline())
    joints = row.get("joint_rad_nominal_only")
    if row.get("frame_index") != 0 or row.get("status", {}).get("arm_status") != "NORMAL(0x0)":
        raise ValueError("recorded frame zero or arm status invalid")
    if (not isinstance(joints, list) or len(joints) != 6 or any(
        type(q) not in (int, float) or not math.isfinite(q) for q in joints
    )):
        raise ValueError("recorded joint vector invalid")
    target = tuple(round(math.degrees(q) * 1000) for q in joints)
    if any(not lo <= q / 1000 <= hi for q, (lo, hi) in zip(target, NOMINAL_JOINT_BOUNDS_DEG)):
        raise ValueError("recorded start exceeds nominal joint bounds")
    return target


def plan_waypoints(start: tuple[int, ...], target: tuple[int, ...]) -> tuple[tuple[int, ...], ...]:
    if (len(start) != 6 or len(target) != 6
            or any(type(q) is not int for q in (*start, *target))):
        raise ValueError("six integer millidegree joints required")
    if any(not lo <= q / 1000 <= hi for vector in (start, target)
           for q, (lo, hi) in zip(vector, NOMINAL_JOINT_BOUNDS_DEG)):
        raise ValueError("start or target outside nominal bounds")
    count = max(1, math.ceil(max(abs(b - a) for a, b in zip(start, target)) / MAX_SEGMENT_RAW))
    if count > 100:
        raise ValueError("return path exceeds 100 bounded segments")
    result = tuple(tuple(a + (1 if b >= a else -1) *
                         min(abs(b - a), index * MAX_SEGMENT_RAW)
                         for a, b in zip(start, target)) for index in range(1, count + 1))
    prior = start
    for waypoint in result:
        if max(abs(b - a) for a, b in zip(prior, waypoint)) > MAX_SEGMENT_RAW:
            raise AssertionError("waypoint exceeds segment cap")
        prior = waypoint
    return result


def checked_feedback(observed: Observed, now_ns: int,
                     allowed_echoes: frozenset[tuple[int, bytes]] = frozenset()) -> tuple[int, ...]:
    if any((can_id, payload) not in allowed_echoes
           for can_id, payload, _ in observed.command_frames):
        raise RuntimeError("another controller detected")
    if any(can not in observed.latest or now_ns - observed.latest[can][0] > MAX_FEEDBACK_AGE_NS
           or observed.latest[can][0] > now_ns for can in REQUIRED_IDS):
        raise RuntimeError("required feedback missing or stale")
    status = observed.latest[0x2A1][1]
    if status[0] != 1 or status[1] != 0 or status[2] != 1 or status[6:8] != b"\x00\x00":
        raise RuntimeError("CAN joint mode or arm status changed")
    if any(not decode_motor_enable(can, observed.latest[can][1])[1] for can in FEEDBACK_IDS):
        raise RuntimeError("motor disabled during return")
    if not observed.joint_history:
        raise RuntimeError("joint feedback missing")
    return observed.joint_history[-1][1]


def within_segment(measured: tuple[int, ...], prior: tuple[int, ...],
                   target: tuple[int, ...]) -> bool:
    return all(min(a, b) - MAX_PATH_DEVIATION_RAW <= q <= max(a, b) + MAX_PATH_DEVIATION_RAW
               for q, a, b in zip(measured, prior, target))


def execute(*, operator_confirmed: bool, output: Path | None = None) -> dict:
    reject_legacy_raw_can_tx("supervised_restore_start")
    if not operator_confirmed:
        raise PermissionError("explicit supervised start-return authorization required")
    from gs_usb.gs_usb import GsUsb, GsUsbFrame

    target = recorded_start_raw()
    _ensure_libusb_backend()
    devices = [d for d in GsUsb.scan()
               if (d.gs_usb.idVendor, d.gs_usb.idProduct) == (0x1D50, 0x606F)]
    if len(devices) != 1:
        raise RuntimeError("expected exactly one candleLight USB-CAN adapter")
    device = devices[0]
    if not device.set_bitrate(1_000_000):
        raise RuntimeError("1 Mbit/s bitrate unsupported")
    observed = Observed()
    started = False
    sent_segments = 0
    last_sent: tuple[int, ...] | None = None
    owned_frames: set[tuple[int, bytes]] = set()
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        _read_for(device, 1.0, observed)
        current = checked_feedback(observed, time.perf_counter_ns())
        recent = [q for ts, q in observed.joint_history
                  if time.perf_counter_ns() - ts <= 300_000_000]
        if len(recent) < 2 or any(
            any(abs(q - expected) > 30 for q, expected in zip(state, EXPECTED_CURRENT_RAW))
            for state in recent
        ):
            raise RuntimeError("live start drifted from confirmed post-step pose")
        waypoints = plan_waypoints(current, target)
        print(f"PRECHECK_PASS segments={len(waypoints)} target_raw={target}", flush=True)
        prior = current
        for index, waypoint in enumerate(waypoints, start=1):
            # Sending all six targets uses the official 0x155-0x157 joint contract.
            for pair in range(3):
                can_id = 0x155 + pair
                payload = struct.pack(">ii", *waypoint[2 * pair:2 * pair + 2])
                frame = GsUsbFrame(can_id=can_id, data=payload)
                if device.send(frame) is not True:
                    raise RuntimeError(f"CAN send failed at segment {index}")
                owned_frames.add((can_id, payload))
            sent_segments = index
            last_sent = waypoint
            started_at = time.perf_counter()
            reached = False
            while time.perf_counter() - started_at < STEP_TIMEOUT_S:
                _read_for(device, 0.05, observed)
                measured = checked_feedback(observed, time.perf_counter_ns(),
                                            frozenset(owned_frames))
                if not within_segment(measured, prior, waypoint):
                    raise RuntimeError(f"segment {index} left 0.2-degree feedback envelope")
                if (time.perf_counter() - started_at >= MIN_STEP_DWELL_S
                        and all(abs(a - b) <= ARRIVAL_TOLERANCE_RAW
                                for a, b in zip(measured, waypoint))):
                    reached = True
                    break
            if not reached:
                raise RuntimeError(f"segment {index} feedback timeout; no further targets sent")
            prior = waypoint
            if index == 1 or index % 5 == 0 or index == len(waypoints):
                print(f"segment {index}/{len(waypoints)} feedback_raw={measured}", flush=True)
        _read_for(device, 1.0, observed)
        final = checked_feedback(observed, time.perf_counter_ns(), frozenset(owned_frames))
        if any(abs(a - b) > ARRIVAL_TOLERANCE_RAW for a, b in zip(final, target)):
            raise RuntimeError("final target not reached after one-second read-only hold")
        report = {
            "status": "JOINT_START_RESTORED_SUPERVISED",
            "episode_id": EPISODE_ID,
            "start_raw_0p001deg": list(current),
            "target_raw_0p001deg": list(target),
            "final_raw_0p001deg": list(final),
            "segments_sent": sent_segments,
            "can_data_tx_calls": sent_segments * 3,
            "max_segment_deg": MAX_SEGMENT_RAW / 1000,
            "gripper_restored": False,
            "collision_model_verified": False,
            "software_stop_verified": False,
            "full_trajectory_control_allowed": False,
        }
        if output is not None:
            output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report
    except Exception as exc:
        print(f"ABORT: {exc}; sent_segments={sent_segments}; last_sent={last_sent}; "
              "NO FURTHER TARGETS. Operator should press physical E-stop if motion is abnormal.",
              flush=True)
        raise
    finally:
        if started:
            device.stop()  # USB close is not a robot stop.


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-supervised-start-return", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = execute(operator_confirmed=args.operator_confirmed_supervised_start_return,
                     output=args.output)
    print(json.dumps(result, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
