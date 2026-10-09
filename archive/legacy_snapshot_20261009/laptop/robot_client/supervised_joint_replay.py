"""One supervised, retimed, joint-only replay of the saved PiPER episode.

This is a one-off observation trial, not a policy interface. It uses recorded
joint positions only; the gripper channel is left at its current position.
Each target is interpolated to at most 1 degree per axis, then checked against
live CAN feedback before the next target. The operator owns the physical E-stop.
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
from laptop.robot_client.supervised_restore_start import checked_feedback

EPISODE_ID = "cef166e2cc6240b397db241a916ece3a"
FRAMES_PATH = Path("datasets/manifests/generated/cef166e2_preview_2fps/frames.jsonl")
STEP_CAP_RAW = 1000
ARRIVAL_TOLERANCE_RAW = 500
ENVELOPE_MARGIN_RAW = 1000
STEP_TIMEOUT_S = 2.0
MIN_STEP_DWELL_S = 0.25


def load_targets(path: Path = FRAMES_PATH) -> tuple[tuple[int, ...], ...]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(rows) < 2 or rows[0].get("frame_index") != 0:
        raise ValueError("recorded episode frames missing")
    targets: list[tuple[int, ...]] = []
    previous_ts = -1
    for index, row in enumerate(rows):
        if row.get("frame_index") != index:
            raise ValueError("frame indices are not contiguous")
        timestamp = row.get("policy_timestamp_ns")
        joints = row.get("joint_rad_nominal_only")
        if (type(timestamp) is not int or timestamp <= previous_ts or not isinstance(joints, list)
                or len(joints) != 6 or any(type(q) not in (int, float) or not math.isfinite(q)
                                           for q in joints)):
            raise ValueError("invalid timestamp or joint vector in recorded frame")
        current = tuple(round(math.degrees(q) * 1000) for q in joints)
        if any(not lo <= q / 1000 <= hi for q, (lo, hi) in
               zip(current, NOMINAL_JOINT_BOUNDS_DEG)):
            raise ValueError("recorded joint target outside nominal bounds")
        previous_ts = timestamp
        targets.append(current)
    return tuple(targets)


def interpolate_targets(keyframes: tuple[tuple[int, ...], ...]) -> tuple[tuple[int, ...], ...]:
    output: list[tuple[int, ...]] = []
    previous = keyframes[0]
    for target in keyframes[1:]:
        count = math.ceil(max(abs(b - a) for a, b in zip(previous, target)) / STEP_CAP_RAW)
        for index in range(1, count + 1):
            point = tuple(round(a + (b - a) * index / count)
                          for a, b in zip(previous, target))
            if max(abs(b - a) for a, b in zip(previous if not output else output[-1], point)) > STEP_CAP_RAW:
                raise AssertionError("interpolation exceeded one-degree step cap")
            output.append(point)
        previous = target
    return tuple(output)


def execute(*, operator_confirmed: bool, output: Path | None = None) -> dict:
    reject_legacy_raw_can_tx("supervised_joint_replay")
    if not operator_confirmed:
        raise PermissionError("explicit supervised joint replay authorization required")
    from gs_usb.gs_usb import GsUsb, GsUsbFrame

    keyframes = load_targets()
    waypoints = interpolate_targets(keyframes)
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
    sent_steps = 0
    owned_frames: set[tuple[int, bytes]] = set()
    last_feedback: tuple[int, ...] | None = None
    started_at = time.perf_counter()
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        _read_for(device, 1.0, observed)
        current = checked_feedback(observed, time.perf_counter_ns())
        recorded_start = keyframes[0]
        if any(abs(a - b) > 150 for a, b in zip(current, recorded_start)):
            raise RuntimeError("live pose is not at recorded episode start")
        latest_time = time.perf_counter_ns()
        if any(latest_time - observed.latest[can][0] > MAX_FEEDBACK_AGE_NS for can in REQUIRED_IDS):
            raise RuntimeError("live feedback stale at replay start")
        if any(not decode_motor_enable(can, observed.latest[can][1])[1] for can in FEEDBACK_IDS):
            raise RuntimeError("one or more arm motors are disabled")
        print(f"PRECHECK_PASS keyframes={len(keyframes)} waypoints={len(waypoints)} "
              f"duration_floor_s={len(waypoints) * MIN_STEP_DWELL_S:.1f}", flush=True)

        previous = current
        for index, target in enumerate(waypoints, start=1):
            for pair in range(3):
                can_id = 0x155 + pair
                payload = struct.pack(">ii", *target[2 * pair:2 * pair + 2])
                if device.send(GsUsbFrame(can_id=can_id, data=payload)) is not True:
                    raise RuntimeError(f"joint CAN send failed at waypoint {index}")
                owned_frames.add((can_id, payload))
            sent_steps = index
            command_at = time.perf_counter()
            arrived = False
            while time.perf_counter() - command_at < STEP_TIMEOUT_S:
                _read_for(device, 0.05, observed)
                measured = checked_feedback(observed, time.perf_counter_ns(),
                                            frozenset(owned_frames))
                if not all(min(a, b) - ENVELOPE_MARGIN_RAW <= q <=
                           max(a, b) + ENVELOPE_MARGIN_RAW
                           for q, a, b in zip(measured, previous, target)):
                    raise RuntimeError(f"feedback left bounded envelope at waypoint {index}")
                if (time.perf_counter() - command_at >= MIN_STEP_DWELL_S
                        and all(abs(a - b) <= ARRIVAL_TOLERANCE_RAW
                                for a, b in zip(measured, target))):
                    arrived = True
                    last_feedback = measured
                    break
            if not arrived:
                raise RuntimeError(f"feedback timeout at waypoint {index}; no more targets sent")
            previous = target
            if index % 20 == 0 or index == len(waypoints):
                elapsed = time.perf_counter() - started_at
                print(f"replay {index}/{len(waypoints)} elapsed={elapsed:.1f}s "
                      f"joint_raw={last_feedback}", flush=True)

        _read_for(device, 1.0, observed)
        final = checked_feedback(observed, time.perf_counter_ns(), frozenset(owned_frames))
        if any(abs(a - b) > ARRIVAL_TOLERANCE_RAW for a, b in zip(final, keyframes[-1])):
            raise RuntimeError("final replay feedback outside 0.5-degree tolerance")
        report = {
            "status": "SUPERVISED_JOINT_ONLY_REPLAY_COMPLETE",
            "episode_id": EPISODE_ID,
            "keyframes": len(keyframes),
            "retimed_waypoints": len(waypoints),
            "steps_sent": sent_steps,
            "can_data_tx_calls": sent_steps * 3,
            "step_cap_deg_per_axis": STEP_CAP_RAW / 1000,
            "arrival_tolerance_deg": ARRIVAL_TOLERANCE_RAW / 1000,
            "start_joint_raw_0p001deg": list(current),
            "end_target_joint_raw_0p001deg": list(keyframes[-1]),
            "final_joint_raw_0p001deg": list(final),
            "gripper_commands_sent": 0,
            "policy_model_connected": False,
            "software_stop_verified": False,
        }
        if output is not None:
            output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report
    except Exception as exc:
        print(f"REPLAY_ABORT: {exc}; completed_waypoints={sent_steps}; "
              f"last_joint_raw={last_feedback}; NO FURTHER TARGETS SENT. "
              "Use physical E-stop if motion is abnormal.", flush=True)
        raise
    finally:
        if started:
            device.stop()  # closes adapter only, not a robot stop.


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-joint-only-replay", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = execute(operator_confirmed=args.operator_confirmed_joint_only_replay,
                     output=args.output)
    print(json.dumps(result, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
