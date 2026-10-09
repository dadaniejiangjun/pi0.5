"""One approved +0.100 or +1.000-degree J1 reference step.

The 1-degree selection first configures CAN/MOVE_J/5% position-speed mode.
This command never opens cameras, enables motors, homes, commands the gripper,
or retries an ambiguous USB write. Keep a hand on the
physical E-stop. A successful USB write is not proof of robot motion.
"""

from __future__ import annotations

import argparse
import json
import struct
import time
import uuid
from typing import Callable

from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import BridgeState, SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.can_transport import CanFrame
from laptop.transport.gs_usb_piper_can import (
    CanJ1CommissioningDeliveryUnknown,
    GsUsbPiperCanTransport,
)
from laptop.transport.position_commands import (
    COMMISSIONING_FEEDBACK_IDS,
    J1CommissioningAuthorization,
    J1CommissioningBatch,
    J1CommissioningSnapshot,
)


CONFIRMATION_PHRASE = "AUTHORIZE J1 +0.100 DEGREE COMMISSIONING STEP"
CONFIRMATION_PHRASE_ONE_DEGREE = "AUTHORIZE J1 +1.000 DEGREE COMMISSIONING STEP"
PRECHECK_SECONDS = 0.55
POSTCHECK_SECONDS = 1.5
MAX_FEEDBACK_AGE_NS = 120_000_000


def _read_for(bridge: SafetyBridge, seconds: float,
              latest: dict[int, tuple[int, bytes]], joint_history: list[tuple[int, tuple[int, ...]]]) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame = bridge.receive_frame(timeout_ms=50)
        if (not isinstance(frame, CanFrame) or frame.is_error_frame
                or frame.is_extended_id or len(frame.data) != 8):
            continue
        if frame.arbitration_id in COMMISSIONING_FEEDBACK_IDS:
            latest[frame.arbitration_id] = (frame.received_monotonic_ns, frame.data)
            if frame.arbitration_id == 0x2A7 and all(
                can_id in latest for can_id in (0x2A5, 0x2A6, 0x2A7)
            ):
                joints = tuple(
                    value for can_id in (0x2A5, 0x2A6, 0x2A7)
                    for value in struct.unpack(">ii", latest[can_id][1])
                )
                joint_history.append((frame.received_monotonic_ns, joints))


def _feedback_outcome(
    latest: dict[int, tuple[int, bytes]],
    joint_history: list[tuple[int, tuple[int, ...]]],
    initial: tuple[int, ...],
    target: tuple[int, ...],
    now_ns: int,
) -> dict:
    fresh = all(
        can_id in latest
        and 0 <= now_ns - latest[can_id][0] <= MAX_FEEDBACK_AGE_NS
        for can_id in COMMISSIONING_FEEDBACK_IDS
    )
    status = latest.get(0x2A1)
    healthy = bool(
        status and status[1][0] == 0x01 and status[1][1] == 0
        and status[1][2] == 0x01 and status[1][6:8] == b"\x00\x00"
    )
    all_enabled = all(
        can_id in latest and bool(latest[can_id][1][5] & 0x40)
        for can_id in range(0x261, 0x267)
    )
    current = joint_history[-1][1] if joint_history else None
    changed = [index for index in range(6) if target[index] != initial[index]]
    axis = changed[0] if len(changed) == 1 else 0
    held = [index for index in range(6) if index != axis]
    overshoot = any(
        joints[axis] < min(initial[axis], target[axis]) - 50
        or joints[axis] > max(initial[axis], target[axis]) + 50
        or any(abs(joints[index] - initial[index]) > 50 for index in held)
        for _, joints in joint_history
    )
    match = bool(
        fresh and healthy and all_enabled and current is not None and not overshoot
        and abs(current[axis] - target[axis]) <= 50
        and all(abs(current[index] - initial[index]) <= 50 for index in held)
    )
    return {
        "status": "SINGLE_STEP_FEEDBACK_MATCH" if match
        else "FRAMES_ACCEPTED_FEEDBACK_UNCONFIRMED_NO_RETRY",
        "feedback_fresh": fresh,
        "arm_healthy_in_can_ctrl": healthy,
        "all_six_motors_enabled": all_enabled,
        "last_joint_raw_0p001deg": list(current) if current else None,
        "target_joint_raw_0p001deg": list(target),
        "observed_overshoot_or_other_joint_motion": overshoot,
        "joint_history_bounds_raw_0p001deg": [
            [min(values[index] for _, values in joint_history),
             max(values[index] for _, values in joint_history)]
            for index in range(6)] if joint_history else None,
        "joint_history": [{"received_monotonic_ns": stamp,
                           "joint_raw_0p001deg": list(values)}
                          for stamp, values in joint_history],
        "automatic_retry": False,
    }


def execute(
    *,
    commissioning_j1_plus_0_1deg: bool = False,
    commissioning_j1_plus_1deg: bool = False,
    physical_estop_ready: bool,
    low_speed_confirmed: bool,
    motion_envelope_cleared: bool,
    confirm: Callable[[dict], str],
    transport: GsUsbPiperCanTransport | None = None,
    selected_axis: int = 1,
) -> dict:
    """Run a separately approved live step; never called by offline tests."""
    if (type(commissioning_j1_plus_0_1deg) is not bool
            or type(commissioning_j1_plus_1deg) is not bool
            or int(commissioning_j1_plus_0_1deg) + int(commissioning_j1_plus_1deg) != 1):
        raise PermissionError("exactly one explicit fixed-J1 commissioning flag is required")
    step_raw = 1000 if commissioning_j1_plus_1deg else 100
    phrase = CONFIRMATION_PHRASE_ONE_DEGREE if commissioning_j1_plus_1deg else CONFIRMATION_PHRASE
    if type(selected_axis) is not int or selected_axis not in range(1, 7):
        raise ValueError("selected axis must be J1 through J6")
    if selected_axis != 1:
        if not commissioning_j1_plus_1deg:
            raise ValueError("J2-J6 commissioning requires exactly a nominal 1-degree step")
        step_raw = -1000 if selected_axis == 3 else 1000
        phrase = f"AUTHORIZE J{selected_axis} {step_raw / 1000:+.3f} DEGREE COMMISSIONING STEP"
    if (physical_estop_ready is not True or low_speed_confirmed is not True
            or motion_envelope_cleared is not True):
        raise PermissionError("onsite E-stop, low-speed, and motion-envelope checks are required")
    if not callable(confirm):
        raise TypeError("an immediate operator confirmation callback is required")

    can_transport = transport if transport is not None else GsUsbPiperCanTransport()
    bridge = SafetyBridge(
        mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=can_transport,
        limits=UNVERIFIED_ANALYSIS_LIMITS,
        supervised_j1_commissioning_tx_enabled=True,
    )
    latest: dict[int, tuple[int, bytes]] = {}
    joint_history: list[tuple[int, tuple[int, ...]]] = []
    try:
        bridge.start_read_only()
        _read_for(bridge, PRECHECK_SECONDS, latest, joint_history)
        snapshot: J1CommissioningSnapshot = bridge.create_j1_commissioning_snapshot(
            physical_estop_ready=physical_estop_ready,
            low_speed_confirmed=low_speed_confirmed,
            motion_envelope_cleared=motion_envelope_cleared,
        )
        batch = J1CommissioningBatch.for_fixed_j1_step(
            snapshot, step_raw=step_raw, configure_control_mode=commissioning_j1_plus_1deg,
            selected_axis=selected_axis)
        preview = {
            "mode": RuntimeMode.SUPERVISED_J1_COMMISSIONING.value,
            "current_joint_raw_0p001deg": list(batch.current_joint_raw_0p001deg),
            "target_joint_raw_0p001deg": list(batch.target_joint_raw_0p001deg),
            "target_step": f"J{selected_axis} {step_raw / 1000:+.3f} degree by nominal SDK scale from measured pose; other joints held; gripper untouched",
            "selected_axis": selected_axis,
            "can_ids": [f"0x{frame.arbitration_id:03X}" for frame in batch.frames],
            "camera_opened": False,
            "mode_or_motor_enable_commanded": batch.configure_control_mode,
            "mode_frame_configures_5percent": batch.configure_control_mode,
            "motor_enable_commanded": False,
        }
        print(json.dumps(preview, indent=2))
        if confirm(preview) != phrase:
            raise PermissionError("exact J1 commissioning confirmation phrase was not entered")

        now_ns = time.perf_counter_ns()
        authorization = J1CommissioningAuthorization(
            approval_id=f"j1-{uuid.uuid4().hex}",
            source_state_sequence=snapshot.sequence,
            target_joint_raw_0p001deg=batch.target_joint_raw_0p001deg,
            approved_at_monotonic_ns=now_ns,
            valid_until_monotonic_ns=now_ns + 200_000_000,
            operator_confirmed=True,
            physical_estop_ready=True,
            low_speed_confirmed=True,
            motion_envelope_cleared=True,
        )
        bridge.record_heartbeat(now_ns)
        receipt = bridge.send_supervised_j1_commissioning_step(
            snapshot=snapshot, batch=batch, authorization=authorization,
        )
        _read_for(bridge, POSTCHECK_SECONDS, latest, joint_history)
        outcome = _feedback_outcome(
            latest, joint_history, batch.current_joint_raw_0p001deg,
            batch.target_joint_raw_0p001deg,
            now_ns=time.perf_counter_ns(),
        )
        return {
            **preview,
            **outcome,
            "adapter_accepted_frames": [
                {
                    "can_id": f"0x{write.arbitration_id:03X}",
                    "data_hex": write.data.hex(),
                    "accepted_monotonic_ns": write.adapter_accepted_monotonic_ns,
                }
                for write in receipt.writes
            ],
            "robot_motion_claimed": outcome["status"] == "SINGLE_STEP_FEEDBACK_MATCH",
            "can_data_tx_calls": len(receipt.writes),
            "operator_physical_estop_required": True,
            "full_trajectory_control_allowed": False,
        }
    except CanJ1CommissioningDeliveryUnknown as exc:
        return {
            "status": "DELIVERY_UNKNOWN_NO_RETRY_REQUIRES_OPERATOR_CHECK",
            "confirmed_frame_count": exc.confirmed_frame_count,
            "failed_frame_id": f"0x{exc.failed_frame_id:03X}",
            "can_data_tx_calls": exc.confirmed_frame_count + 1,
            "robot_motion_claimed": False,
            "automatic_retry": False,
        }
    finally:
        if bridge.state is not BridgeState.CLOSED:
            bridge.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--commissioning-j1-plus-0.1deg", dest="commissioning_j1_plus_0_1deg", action="store_true")
    selection.add_argument("--commissioning-j1-plus-1deg", action="store_true")
    parser.add_argument("--physical-estop-ready", action="store_true")
    parser.add_argument("--low-speed-confirmed", action="store_true")
    parser.add_argument("--motion-envelope-cleared", action="store_true")
    args = parser.parse_args()

    result = execute(
        commissioning_j1_plus_0_1deg=args.commissioning_j1_plus_0_1deg,
        commissioning_j1_plus_1deg=args.commissioning_j1_plus_1deg,
        physical_estop_ready=args.physical_estop_ready,
        low_speed_confirmed=args.low_speed_confirmed,
        motion_envelope_cleared=args.motion_envelope_cleared,
        confirm=lambda _preview: input(
            f"Type exactly {(CONFIRMATION_PHRASE_ONE_DEGREE if args.commissioning_j1_plus_1deg else CONFIRMATION_PHRASE)!r} immediately before the one-step write: "
        ),
    )
    print(json.dumps(result, indent=2))
    return 0 if result.get("status") == "SINGLE_STEP_FEEDBACK_MATCH" else 1


if __name__ == "__main__":
    raise SystemExit(main())
