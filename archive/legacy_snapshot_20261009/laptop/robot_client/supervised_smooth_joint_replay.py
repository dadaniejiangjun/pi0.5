"""Smooth supervised joint-only replay of the saved PiPER episode.

The recorded episode is sampled into a continuous cubic Hermite position
profile, time-scaled to conservative velocity/acceleration limits, and sent at
fixed frequency. It first reverses the same path to the episode start, then
plays forward. The gripper and policy model are not connected.
"""

from __future__ import annotations

import argparse
import bisect
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
    MAX_FEEDBACK_AGE_NS, REQUIRED_IDS, Observed,
)
from laptop.robot_client.supervised_restore_start import checked_feedback

EPISODE_ID = "cef166e2cc6240b397db241a916ece3a"
FRAMES_PATH = Path("datasets/manifests/generated/cef166e2_preview_2fps/frames.jsonl")
SAMPLE_HZ = 25
MAX_SPEED_DEG_S = 10.0
MAX_ACCEL_DEG_S2 = 30.0
MAX_TRACKING_ERROR_RAW = 2500  # 2.5 degrees
MAX_FEEDBACK_AGE_S = MAX_FEEDBACK_AGE_NS / 1e9


def load_keyframes(path: Path = FRAMES_PATH) -> tuple[list[float], list[tuple[float, ...]]]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(rows) < 2:
        raise ValueError("recorded episode needs at least two frames")
    times: list[float] = []
    positions: list[tuple[float, ...]] = []
    origin = rows[0].get("policy_timestamp_ns")
    for index, row in enumerate(rows):
        ts, joints = row.get("policy_timestamp_ns"), row.get("joint_rad_nominal_only")
        if (row.get("frame_index") != index or type(ts) is not int or type(origin) is not int
                or not isinstance(joints, list) or len(joints) != 6
                or any(type(q) not in (int, float) or not math.isfinite(q) for q in joints)):
            raise ValueError("invalid recorded keyframe")
        times.append((ts - origin) / 1e9)
        qdeg = tuple(math.degrees(q) for q in joints)
        if any(not lo <= q <= hi for q, (lo, hi) in zip(qdeg, NOMINAL_JOINT_BOUNDS_DEG)):
            raise ValueError("recorded target outside nominal joint bounds")
        positions.append(qdeg)
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("recorded timestamps are not strictly increasing")
    return times, positions


def knot_slopes(times: list[float], positions: list[tuple[float, ...]]) -> list[tuple[float, ...]]:
    dimensions = len(positions[0])
    secants = [[(positions[i + 1][j] - positions[i][j]) /
                (times[i + 1] - times[i]) for j in range(dimensions)]
               for i in range(len(times) - 1)]
    slopes = [[0.0] * dimensions for _ in times]
    slopes[0] = [0.0] * dimensions
    slopes[-1] = [0.0] * dimensions
    for i in range(1, len(times) - 1):
        for j in range(dimensions):
            left, right = secants[i - 1][j], secants[i][j]
            if left * right > 0:
                h0, h1 = times[i] - times[i - 1], times[i + 1] - times[i]
                w1, w2 = 2 * h1 + h0, h1 + 2 * h0
                slopes[i][j] = (w1 + w2) / (w1 / left + w2 / right)
    return [tuple(row) for row in slopes]


def hermite(y0: float, y1: float, m0: float, m1: float, h: float, u: float) -> tuple[float, float, float]:
    u2, u3 = u * u, u * u * u
    value = ((2*u3 - 3*u2 + 1) * y0 + (u3 - 2*u2 + u) * h * m0
             + (-2*u3 + 3*u2) * y1 + (u3 - u2) * h * m1)
    du = ((6*u2 - 6*u) * y0 + (3*u2 - 4*u + 1) * h * m0
          + (-6*u2 + 6*u) * y1 + (3*u2 - 2*u) * h * m1)
    ddu = ((12*u - 6) * y0 + (6*u - 4) * h * m0
           + (-12*u + 6) * y1 + (6*u - 2) * h * m1)
    return value, du / h, ddu / (h * h)


def profile(times: list[float], positions: list[tuple[float, ...]]) -> dict:
    durations = []
    for i in range(len(times) - 1):
        original_dt = times[i + 1] - times[i]
        largest_step = max(abs(b - a) for a, b in zip(positions[i], positions[i + 1]))
        durations.append(max(original_dt, 1.5 * largest_step / MAX_SPEED_DEG_S,
                             math.sqrt(6 * largest_step / MAX_ACCEL_DEG_S2)))
    converged = False
    for _ in range(60):
        retimed = [0.0]
        for dt in durations:
            retimed.append(retimed[-1] + dt)
        slopes = knot_slopes(retimed, positions)
        updates = [1.0] * len(durations)
        for i, h in enumerate(durations):
            max_v = max_a = 0.0
            for sample in range(41):
                u = sample / 40
                for axis in range(6):
                    _, velocity, acceleration = hermite(
                        positions[i][axis], positions[i + 1][axis],
                        slopes[i][axis], slopes[i + 1][axis], h, u)
                    max_v, max_a = max(max_v, abs(velocity)), max(max_a, abs(acceleration))
            updates[i] = max(1.0, max_v / MAX_SPEED_DEG_S,
                             math.sqrt(max_a / MAX_ACCEL_DEG_S2))
        worst = max(updates)
        if worst <= 1.002:
            converged = True
            break
        durations = [dt * max(1.0, factor * 1.01) for dt, factor in zip(durations, updates)]
    if not converged:
        raise RuntimeError("local trajectory retiming did not converge")
    retimed = [0.0]
    for dt in durations:
        retimed.append(retimed[-1] + dt)
    slopes = knot_slopes(retimed, positions)
    duration = retimed[-1]
    count = math.ceil(duration * SAMPLE_HZ)
    max_v = max_a = 0.0
    sampled: list[tuple[int, ...]] = []
    for index in range(count + 1):
        t = min(duration, index / SAMPLE_HZ)
        segment = min(len(retimed) - 2, max(0, bisect.bisect_right(retimed, t) - 1))
        h = retimed[segment + 1] - retimed[segment]
        u = min(1.0, max(0.0, (t - retimed[segment]) / h))
        values_list = []
        for axis in range(6):
            value, velocity, acceleration = hermite(
                positions[segment][axis], positions[segment + 1][axis],
                slopes[segment][axis], slopes[segment + 1][axis], h, u)
            values_list.append(value)
            max_v, max_a = max(max_v, abs(velocity)), max(max_a, abs(acceleration))
        values = tuple(values_list)
        if any(not lo <= q <= hi for q, (lo, hi) in zip(values, NOMINAL_JOINT_BOUNDS_DEG)):
            raise ValueError("interpolated profile exceeds nominal joint bounds")
        sampled.append(tuple(round(q * 1000) for q in values))
    return {
        "targets": tuple(sampled),
        "duration_s": duration,
        "sample_hz": SAMPLE_HZ,
        "estimated_peak_speed_deg_s": max_v,
        "estimated_peak_accel_deg_s2": max_a,
    }


def _read_available(device: object, observed: Observed) -> None:
    from gs_usb.gs_usb import GsUsbFrame

    while True:
        frame = GsUsbFrame()
        try:
            ok = device.read(frame, timeout_ms=1)
        except struct.error:
            continue
        if not ok:
            return
        if not frame.is_error_frame and frame.can_dlc == 8:
            observed.add(frame.arbitration_id, bytes(frame.data[:8]), time.perf_counter_ns())


def execute(*, operator_confirmed: bool, output: Path | None = None) -> dict:
    reject_legacy_raw_can_tx("supervised_smooth_joint_replay")
    if not operator_confirmed:
        raise PermissionError("explicit supervised smooth replay authorization required")
    from gs_usb.gs_usb import GsUsb, GsUsbFrame

    times, positions = load_keyframes()
    result = profile(times, positions)
    targets = result["targets"]
    if len(targets) < 2:
        raise RuntimeError("smooth profile produced too few points")
    _ensure_libusb_backend()
    devices = [d for d in GsUsb.scan()
               if (d.gs_usb.idVendor, d.gs_usb.idProduct) == (0x1D50, 0x606F)]
    if len(devices) != 1:
        raise RuntimeError("expected exactly one candleLight USB-CAN adapter")
    device = devices[0]
    if not device.set_bitrate(1_000_000):
        raise RuntimeError("1 Mbit/s bitrate unsupported")
    observed = Observed()
    owned: set[tuple[int, bytes]] = set()
    started = False
    sent_samples = 0
    attempted_frame: tuple[int, bytes] | None = None
    phase = "preflight"
    try:
        device.start(flags=0)
        started = True
        if device.device_flags != 0:
            raise RuntimeError("unexpected adapter flags")
        # Short bounded receive window. No control data is sent here.
        deadline = time.perf_counter() + 1.0
        while time.perf_counter() < deadline:
            _read_available(device, observed)
            time.sleep(0.002)
        current = checked_feedback(observed, time.perf_counter_ns())
        last_pose = tuple(round(q * 1000) for q in positions[-1])
        if any(abs(a - b) > 600 for a, b in zip(current, last_pose)):
            raise RuntimeError("live arm is not at the prior replay endpoint")
        now_ns = time.perf_counter_ns()
        if any(now_ns - observed.latest[can][0] > MAX_FEEDBACK_AGE_NS for can in REQUIRED_IDS):
            raise RuntimeError("required feedback stale at replay start")
        if any(not decode_motor_enable(can, observed.latest[can][1])[1] for can in FEEDBACK_IDS):
            raise RuntimeError("one or more arm motors are disabled")
        print(f"PRECHECK_PASS samples={len(targets)} hz={SAMPLE_HZ} "
              f"one_way_duration_s={result['duration_s']:.1f}; returning along same path first",
              flush=True)

        sequence = tuple(reversed(targets[:-1])) + targets
        previous = current
        phase = "reverse_to_start_then_forward"
        phase_started = time.perf_counter()
        next_tick = phase_started
        for index, target in enumerate(sequence, start=1):
            now = time.perf_counter()
            next_tick += 1.0 / SAMPLE_HZ
            if next_tick > now:
                time.sleep(next_tick - now)
            for pair in range(3):
                can_id = 0x155 + pair
                payload = struct.pack(">ii", *target[2 * pair:2 * pair + 2])
                attempted_frame = (can_id, payload)
                if device.send(GsUsbFrame(can_id=can_id, data=payload)) is not True:
                    raise RuntimeError(f"CAN send failed at sample {index}")
                owned.add((can_id, payload))
            sent_samples = index
            _read_available(device, observed)
            measured = checked_feedback(observed, time.perf_counter_ns(), frozenset(owned))
            if any(abs(a - b) > MAX_TRACKING_ERROR_RAW for a, b in zip(measured, target)):
                raise RuntimeError(f"tracking error exceeded 2.5 degrees at sample {index}")
            if not all(min(a, b) - 3000 <= q <= max(a, b) + 3000
                       for q, a, b in zip(measured, previous, target)):
                raise RuntimeError(f"feedback left local envelope at sample {index}")
            previous = target
            if index % (SAMPLE_HZ * 5) == 0 or index == len(sequence):
                print(f"smooth replay {index}/{len(sequence)} "
                      f"elapsed={time.perf_counter()-phase_started:.1f}s "
                      f"joint_raw={measured}", flush=True)
        # Read-only stabilization at the episode endpoint.
        deadline = time.perf_counter() + 0.5
        while time.perf_counter() < deadline:
            _read_available(device, observed)
            time.sleep(0.002)
        final = checked_feedback(observed, time.perf_counter_ns(), frozenset(owned))
        end_raw = targets[-1]
        if any(abs(a - b) > 800 for a, b in zip(final, end_raw)):
            raise RuntimeError("final endpoint feedback outside 0.8-degree tolerance")
        report = {
            "status": "SUPERVISED_SMOOTH_JOINT_REPLAY_COMPLETE",
            "episode_id": EPISODE_ID,
            "source_keyframes": len(positions),
            "samples_per_direction": len(targets),
            "sample_hz": SAMPLE_HZ,
            "one_way_duration_s": round(result["duration_s"], 3),
            "there_and_back_duration_s_observed": round(time.perf_counter() - phase_started, 3),
            "estimated_peak_speed_deg_s": round(result["estimated_peak_speed_deg_s"], 3),
            "estimated_peak_accel_deg_s2": round(result["estimated_peak_accel_deg_s2"], 3),
            "max_tracking_error_deg": MAX_TRACKING_ERROR_RAW / 1000,
            "can_data_tx_calls": sent_samples * 3,
            "gripper_commands_sent": 0,
            "policy_model_connected": False,
            "final_feedback_raw_0p001deg": list(final),
            "software_stop_verified": False,
        }
        if output is not None:
            output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report
    except Exception as exc:
        attempted = (f"; last_usb_write_attempt=0x{attempted_frame[0]:03X}:"
                     f"{attempted_frame[1].hex()} (delivery status UNKNOWN)"
                     if attempted_frame is not None else "")
        print(f"SMOOTH_REPLAY_ABORT: {exc}; phase={phase}{attempted}; "
              f"sent_samples={sent_samples}; NO FURTHER TARGETS SENT. "
              "Do not retry an uncertain frame. Use physical E-stop if motion is abnormal.",
              flush=True)
        raise
    finally:
        if started:
            device.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-confirmed-smooth-joint-replay", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = execute(operator_confirmed=args.operator_confirmed_smooth_joint_replay,
                     output=args.output)
    print(json.dumps(result, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
