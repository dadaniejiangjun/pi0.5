"""Pure input-to-target mapping for gamepad teleoperation."""

from __future__ import annotations

import math


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7)) + ("gripper",)


def apply_deadzone(value: float, deadzone: float) -> float:
    if not math.isfinite(value) or not 0.0 <= deadzone < 1.0:
        raise ValueError("invalid joystick value or deadzone")
    value = max(-1.0, min(1.0, value))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign((magnitude - deadzone) / (1.0 - deadzone), value)


def integrate_target(
    *,
    current_joints_rad: tuple[float, ...],
    current_gripper_raw: int,
    previous_target: tuple[float, ...] | None,
    axes: tuple[float, ...],
    buttons: tuple[int, ...],
    joint_bounds_deg: tuple[tuple[float, float], ...],
    gripper_bounds_raw: tuple[int, int],
    axis_deadzone: float = 0.12,
    joint_step_deg: float = 5.0,
    joint_target_lead_deg: float = 5.0,
    gripper_step_raw: int = 100,
    gripper_target_lead_raw: int = 12500,
    deadman_button: int = 10,
    close_button: int = 2,
    open_button: int = 3,
    require_deadman: bool = True,
    mapping_profile: str = "legacy",
    gripper_control: str = "buttons",
) -> tuple[float, ...]:
    """Return one bounded 7D position target from the latest Xbox state.

    Full-scale stick input changes a joint by at most ``joint_step_deg`` per
    update. Each output is also capped relative to the latest feedback so a
    slow or stalled robot cannot accumulate an arbitrarily distant target.
    D-pad buttons jog J4/J5; X/Y jog the gripper. The executor performs an
    independent second clamp.
    """
    if len(current_joints_rad) != 6 or len(joint_bounds_deg) != 6:
        raise ValueError("six feedback joints and six limits are required")
    if not 0.0 < joint_step_deg <= 5.0:
        raise ValueError("gamepad joint step must be within (0, 5] degrees")
    if not 0.0 < joint_target_lead_deg <= 5.0:
        raise ValueError("joint target lead must be within (0, 5] degrees")
    if type(gripper_step_raw) is not int or not 0 < gripper_step_raw <= 12500:
        raise ValueError("gripper step must be an integer within 1..12500 raw")
    if (type(gripper_target_lead_raw) is not int
            or not 0 < gripper_target_lead_raw <= 12500):
        raise ValueError("gripper target lead must be an integer within 1..12500 raw")
    required_buttons = max(deadman_button, close_button, open_button, 14)
    if len(axes) < 4 or len(buttons) <= required_buttons:
        raise ValueError("joystick message does not match the configured Xbox layout")
    if require_deadman and not buttons[deadman_button]:
        raise ValueError("deadman is not held")
    if mapping_profile not in ("legacy", "gamepad_piper") or gripper_control not in ("buttons", "triggers"):
        raise ValueError("unsupported mapping profile or gripper control")
    if any(not math.isfinite(float(value)) for value in current_joints_rad):
        raise ValueError("joint feedback must be finite")
    low_raw, high_raw = gripper_bounds_raw
    # Measured end-stop offsets are telemetry, not command targets. Keep the
    # original state upstream; use a legal reference for target integration.
    if type(current_gripper_raw) is not int:
        raise ValueError("integer gripper feedback required")
    current_gripper_raw = max(low_raw, min(high_raw, current_gripper_raw))

    if previous_target is None:
        target_joints = list(current_joints_rad)
        target_gripper = int(current_gripper_raw)
    else:
        if len(previous_target) != 7 or any(not math.isfinite(v) for v in previous_target):
            raise ValueError("previous target must be a finite seven-axis vector")
        target_joints = list(previous_target[:6])
        target_gripper = round(previous_target[6])

    # game_controller_node's SDL mapping: LX, LY, RX, RY at axes 0..3.
    # Positive motion signs are deliberately explicit so the mapping can be
    # calibrated through preview before enabling the arm.
    joint_commands = (
        (0, axes[0], 1.0),  # LX -> J1
        (1, axes[1], -1.0), # LY -> J2
        (2, axes[3], -1.0), # RY -> J3
        (5, axes[2], 1.0),  # RX -> J6
    )
    directions = [0.0] * 6
    for joint_index, axis_value, sign in joint_commands:
        directions[joint_index] = sign * apply_deadzone(float(axis_value), axis_deadzone)
    # SDL game-controller button order: D-pad up/down/left/right = 11..14.
    directions[3] = float(bool(buttons[14])) - float(bool(buttons[13]))
    directions[4] = float(bool(buttons[11])) - float(bool(buttons[12]))
    if mapping_profile == "gamepad_piper":
        # Adapt Gamepad_PiPER's joint equations to our SDL-compatible axes.
        directions[0] *= -1
        directions[2] *= -1
        directions[4] *= -1

    for index, direction in enumerate(directions):
        low_deg, high_deg = joint_bounds_deg[index]
        feedback_deg = math.degrees(current_joints_rad[index])
        target_deg = math.degrees(target_joints[index])
        # Never carry a stale target through a direction change. Starting at
        # feedback on neutral also releases just this axis while other axes
        # remain active.
        if direction > 0.0:
            base_deg = max(feedback_deg, target_deg)
        elif direction < 0.0:
            base_deg = min(feedback_deg, target_deg)
        else:
            base_deg = feedback_deg
        next_deg = max(low_deg, min(high_deg, base_deg + direction * joint_step_deg))
        target_joints[index] = math.radians(next_deg)

    if gripper_control == "triggers":
        if len(axes) < 6 or any(not math.isfinite(float(v)) or not 0 <= v <= 1 for v in axes[4:6]):
            raise ValueError("LT/RT must be normalized to 0..1")
        # Proportional closing/opening; no upstream end-stop toggle jump.
        close = apply_deadzone(float(axes[4]), axis_deadzone)
        open_ = apply_deadzone(float(axes[5]), axis_deadzone)
        gripper_delta = round((open_ - close) * gripper_step_raw)
    else:
        close = bool(buttons[close_button])
        open_ = bool(buttons[open_button])
        gripper_delta = (-gripper_step_raw if close else gripper_step_raw) if close != open_ else 0
    if gripper_delta > 0:
        target_gripper = max(current_gripper_raw, target_gripper) + gripper_delta
    elif gripper_delta < 0:
        target_gripper = min(current_gripper_raw, target_gripper) + gripper_delta
    else:
        target_gripper = int(current_gripper_raw)
    target_gripper = max(low_raw, min(high_raw, target_gripper))

    # Bound the accumulated absolute target against the newest measured pose.
    # This limits requested lead only; it is not a physical stop-distance bound.
    for index, (feedback_rad, (low_deg, high_deg)) in enumerate(
            zip(current_joints_rad, joint_bounds_deg)):
        feedback_deg = math.degrees(feedback_rad)
        lower = max(low_deg, feedback_deg - joint_target_lead_deg)
        upper = min(high_deg, feedback_deg + joint_target_lead_deg)
        if lower > upper:
            raise ValueError("joint feedback does not overlap the target lead envelope")
        target_deg = math.degrees(target_joints[index])
        target_joints[index] = math.radians(max(lower, min(upper, target_deg)))

    target_gripper = max(
        max(low_raw, current_gripper_raw - gripper_target_lead_raw),
        min(
            min(high_raw, current_gripper_raw + gripper_target_lead_raw),
            target_gripper,
        ),
    )
    return tuple(target_joints) + (float(target_gripper),)
