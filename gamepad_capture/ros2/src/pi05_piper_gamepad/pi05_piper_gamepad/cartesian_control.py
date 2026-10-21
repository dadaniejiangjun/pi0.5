"""Pure gamepad mappings for six-dimensional PiPER pose control."""

from __future__ import annotations

import math
import time

from laptop.piper.unit_conversion import rad_to_sdk_joint, sdk_joint_to_rad
from laptop.transport.manual_gamepad import bounded_hold_joints

from .targets import apply_deadzone
from .kinematics import apply_pose_delta, _rotation_vector, _matmul, _transpose


def bounded_cartesian_step(*, model, current_joints_rad, translation_m,
                           rotation_rad, frame, joint_bounds_deg,
                           joint_step_deg, joint_target_lead_deg,
                           previous_joints_rad=None, preferred_scale=1.0,
                           diagnostics=None):
    """Backtrack the TCP increment from fresh feedback, never clip IK joints.

    Unaccepted motion is discarded rather than accumulated for later ticks.
    Each candidate must satisfy IK, strict limits and the same feedback bounds.
    """
    if (len(current_joints_rad) != 6 or len(joint_bounds_deg) != 6
            or not all(math.isfinite(v) for v in current_joints_rad)
            or not 0 < joint_step_deg <= 5
            or not 0 < joint_target_lead_deg <= 5):
        raise ValueError("invalid Cartesian feedback or joint step bounds")
    if (len(translation_m) != 3 or len(rotation_rad) != 3
            or not all(math.isfinite(v) for v in (*translation_m, *rotation_rad))):
        raise ValueError("Cartesian delta must contain six finite values")
    limits = tuple((math.radians(low), math.radians(high))
                   for low, high in joint_bounds_deg)
    if any(not math.isfinite(low) or not math.isfinite(high) or low >= high
           for low, high in limits):
        raise ValueError("invalid Cartesian joint limits")
    seed = legalize_feedback_joints_rad(
        tuple(current_joints_rad), joint_bounds_deg=joint_bounds_deg)
    origin = model.forward(tuple(current_joints_rad))
    bound = math.radians(min(joint_step_deg, joint_target_lead_deg))
    solve_limits = limits
    if previous_joints_rad is not None:
        if (len(previous_joints_rad) != 6 or
                not all(math.isfinite(v) for v in previous_joints_rad)):
            raise ValueError('invalid previous Cartesian solution')
        # A previous target is a continuity reference, never an accumulated
        # pose goal. Drop it when measured feedback leaves its envelope.
        if all(low <= old <= high and abs(old-q) <= bound
               for old,q,(low,high) in zip(previous_joints_rad,current_joints_rad,limits)):
            # Start the built-in differential solve at feedback. Reusing an
            # old seed within the loose 1 mm / 1 deg tolerance can return the
            # identical old target repeatedly (a staircase). TRAC-IK retains
            # its upstream prior-solution seed internally.
            # Check consecutive target continuity after IK. Clamping each
            # iteration into a moving previous-target box distorts the solve.
            previous_joints_rad = tuple(previous_joints_rad)
        else:
            previous_joints_rad = None
    if getattr(model, "use_feedback_solver_limits", False):
        solve_limits = tuple((max(low,q-bound),min(high,q+bound))
                             for q,(low,high) in zip(current_joints_rad,limits))
        if any(low >= high for low,high in solve_limits):
            raise ValueError("feedback step envelope does not overlap strict joint limits")
    if not math.isfinite(preferred_scale) or not 0 < preferred_scale <= 1:
        raise ValueError('invalid preferred Cartesian scale')
    started = time.perf_counter()
    if diagnostics is not None:
        diagnostics.update(attempts=[], continuity_reference_used=previous_joints_rad is not None)
    last_error = "no admissible Cartesian increment"
    deadline = time.perf_counter() + 0.08
    next_scale = preferred_scale
    for attempt in range(10):
        if attempt and time.perf_counter() >= deadline:
            raise ValueError(f"Cartesian solve time budget exceeded: {last_error}")
        scale = next_scale
        # Small convergence/continuity corrections first. Retain deeper
        # backtracking for truly difficult directions within the same budget.
        next_scale = scale*(.8 if attempt < 5 else .5)
        proposed = apply_pose_delta(
            origin, tuple(v * scale for v in translation_m),
            tuple(v * scale for v in rotation_rad), frame=frame)
        # User-selected teleop residual tolerances, separate from joint limits.
        position_tol = .001
        rotation_tol = math.radians(1.)
        try:
            candidate = tuple(model.inverse(
                proposed, seed, joint_limits_rad=solve_limits,
                allow_damped_singularity=True, max_iterations=12,
                position_tolerance_m=position_tol, orientation_tolerance_rad=rotation_tol,
                time_budget_s=max(0.,deadline-time.perf_counter()),
                minimum_joint_change_rad=1e-7, polish_iterations=2))
            if len(candidate) != 6 or not all(math.isfinite(v) for v in candidate):
                raise ValueError("IK returned invalid joint values")
            largest_lead = max(abs(a-b) for a,b in zip(candidate,current_joints_rad))
            if largest_lead > bound:
                # Use the measured excess rather than always halving speed.
                next_scale = scale*max(.1,min(.95,.95*bound/largest_lead))
            if any(not low <= value <= high
                   or abs(value-feedback) > bound + 1e-9
                   for value, feedback, (low, high)
                   in zip(candidate, current_joints_rad, limits)):
                raise ValueError("IK_STEP_OR_LIMIT: solution exceeds feedback step or strict limit")
            actual = model.forward(candidate)
            residual_r = _rotation_vector(_matmul(proposed.rotation,_transpose(actual.rotation)))
            if (math.dist(actual.position_m,proposed.position_m)>position_tol
                    or math.sqrt(sum(v*v for v in residual_r))>rotation_tol):
                raise ValueError('IK_RESIDUAL: independent FK exceeds teleop tolerances')
            if max(abs(a-b) for a, b in zip(candidate, current_joints_rad)) < 1e-7:
                raise ValueError("Cartesian increment below solver resolution")
            if previous_joints_rad is not None and any(
                    abs(a-b) > bound+1e-9 for a,b in zip(candidate,previous_joints_rad)):
                raise ValueError('IK_CONTINUITY: consecutive target step exceeded')
            if diagnostics is not None:
                diagnostics.update(elapsed_ms=(time.perf_counter()-started)*1000,
                    accepted_scale=scale, position_residual_m=math.dist(actual.position_m,proposed.position_m),
                    orientation_residual_rad=math.sqrt(sum(v*v for v in residual_r)),
                    solver_details=getattr(model,'last_ik_diagnostics',{}))
            if hasattr(model, "accept_solution"):
                model.accept_solution(candidate)
            return proposed, candidate, scale
        except ValueError as exc:
            last_error = str(exc)
            if diagnostics is not None:
                diagnostics['attempts'].append(dict(scale=scale,reason=last_error))
                diagnostics['elapsed_ms']=(time.perf_counter()-started)*1000
                diagnostics['solver_details']=getattr(model,'last_ik_diagnostics',{})
    raise ValueError(f"Cartesian step unavailable after 10 bounded attempts: {last_error}")


class CartesianContinuation:
    """Constrain nearby IK targets and recover feasible speed gradually.

    Reset on release, direction change, session change or failure. The goal
    remains measured-pose-relative, so a stalled arm gains no target backlog.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.joints = None
        self.direction = None
        self.scale = 1.0

    def step(self, *, translation_m, rotation_rad, diagnostics=None, **kwargs):
        direction = tuple(translation_m) + tuple(v*.1 for v in rotation_rad)
        norm = math.sqrt(sum(v*v for v in direction))
        unit = tuple(v/norm for v in direction) if norm else (0.,)*6
        if self.direction is not None and sum(a*b for a,b in zip(unit,self.direction)) < .95:
            self.reset()
            if hasattr(kwargs['model'], 'previous_solution'):
                kwargs['model'].previous_solution = None
        try:
            result = bounded_cartesian_step(translation_m=translation_m,
                rotation_rad=rotation_rad, previous_joints_rad=self.joints,
                preferred_scale=min(1.,self.scale*1.002), diagnostics=diagnostics, **kwargs)
        except ValueError:
            self.reset()
            raise
        self.direction = unit
        self.joints = result[1]
        self.scale = result[2]
        return result


def legalize_feedback_joints_rad(
        joints_rad: tuple[float, ...], *,
        joint_bounds_deg: tuple[tuple[float, float], ...] | None = None
        ) -> tuple[float, ...]:
    """Validate readback with the shared 5-degree tolerance and legalize it.

    The saved/readback value is never modified by this helper.  It returns a
    separate, strict in-envelope seed/target, matching the native manual
    operator's ``bounded_hold_joints`` policy (5000 raw = 5 degrees).
    """
    if len(joints_rad) != 6 or any(not math.isfinite(float(value))
                                   for value in joints_rad):
        raise ValueError("six finite joint feedback values are required")
    raw = tuple(int(value) for value in rad_to_sdk_joint(joints_rad))
    legal_raw = bounded_hold_joints(raw)
    legal = tuple(float(value) for value in sdk_joint_to_rad(legal_raw))
    if joint_bounds_deg is not None:
        if len(joint_bounds_deg) != 6:
            raise ValueError("six strict joint limits are required")
        legalized = []
        for value, (low, high) in zip(legal, joint_bounds_deg):
            if (not math.isfinite(low) or not math.isfinite(high) or low >= high):
                raise ValueError("strict joint limits must be finite and ordered")
            legalized.append(max(math.radians(low), min(math.radians(high), value)))
        legal = tuple(legalized)
    return legal


def pose_increment(*, axes: tuple[float, ...], buttons: tuple[int, ...],
                   axis_deadzone: float, linear_speed_m_s: float,
                   angular_speed_deg_s: float, dt_s: float) -> tuple[
                       tuple[float, float, float],
                       tuple[float, float, float],
                   ]:
    """Map sticks and D-pad to base/local XYZ and rotation-vector increments.

    Left stick controls X/Y translation, right stick controls Z translation
    and yaw, and the D-pad controls roll/pitch. ``axes`` are SDL-normalized
    ROS Joy values. Deltas are metres and radians per operator update.
    """
    if len(axes) < 6 or len(buttons) < 15:
        raise ValueError("Cartesian mapping needs six axes and fifteen buttons")
    if (not math.isfinite(linear_speed_m_s) or linear_speed_m_s <= 0
            or not math.isfinite(angular_speed_deg_s)
            or angular_speed_deg_s <= 0
            or not math.isfinite(dt_s) or dt_s <= 0):
        raise ValueError("Cartesian speeds and update interval must be positive")
    if any(not math.isfinite(float(value)) for value in axes):
        raise ValueError("joystick axes must be finite")
    linear_step = linear_speed_m_s * dt_s
    angular_step = math.radians(angular_speed_deg_s) * dt_s
    translation = (
        -apply_deadzone(float(axes[1]), axis_deadzone) * linear_step,
        -apply_deadzone(float(axes[0]), axis_deadzone) * linear_step,
        -apply_deadzone(float(axes[3]), axis_deadzone) * linear_step,
    )
    dpad_x = float(bool(buttons[14])) - float(bool(buttons[13]))
    dpad_y = float(bool(buttons[11])) - float(bool(buttons[12]))
    rotation = (
        dpad_x * angular_step,
        -dpad_y * angular_step,
        apply_deadzone(float(axes[2]), axis_deadzone) * angular_step,
    )
    return translation, rotation


def preset_segment(*, current_joints_rad: tuple[float, ...],
                   target_joints_rad: tuple[float, ...],
                   current_gripper_raw: int, target_gripper_raw: int,
                   joint_bounds_deg: tuple[tuple[float, float], ...],
                   joint_speed_deg_s: float, dt_s: float,
                   joint_step_limit_deg: float,
                   joint_target_lead_deg: float,
                   gripper_bounds_raw: tuple[int, int],
                   gripper_step_raw: int,
                   gripper_target_lead_raw: int) -> tuple[float, ...]:
    """Return one feedback-anchored, lead-limited point toward a saved pose."""
    if (len(current_joints_rad) != 6 or len(target_joints_rad) != 6
            or len(joint_bounds_deg) != 6):
        raise ValueError("preset restore requires six joints and six limits")
    if (not math.isfinite(joint_speed_deg_s) or joint_speed_deg_s <= 0
            or not math.isfinite(dt_s) or dt_s <= 0
            or not 0 < joint_step_limit_deg <= 5
            or not 0 < joint_target_lead_deg <= 5):
        raise ValueError("invalid preset joint step or lead bound")
    if (type(current_gripper_raw) is not int or type(target_gripper_raw) is not int
            or type(gripper_step_raw) is not int or gripper_step_raw <= 0
            or type(gripper_target_lead_raw) is not int
            or gripper_target_lead_raw <= 0):
        raise ValueError("invalid preset gripper step or feedback")
    low_grip, high_grip = gripper_bounds_raw
    current_gripper_raw = max(low_grip, min(high_grip, current_gripper_raw))
    if not low_grip <= target_gripper_raw <= high_grip:
        raise ValueError("saved gripper target is outside configured bounds")

    # Readback may sit up to five degrees outside the task envelope by the
    # explicitly approved native feedback allowance. Keep the original values
    # for the lead window, but use separate legal seeds/targets for command
    # construction. ``bounded_hold_joints`` is the shared source of tolerance.
    current_raw = tuple(int(value) for value in rad_to_sdk_joint(current_joints_rad))
    legal_current_raw = bounded_hold_joints(current_raw)
    target_raw = tuple(int(value) for value in rad_to_sdk_joint(target_joints_rad))
    legal_target_raw = bounded_hold_joints(target_raw)
    legal_current = tuple(float(value) for value in sdk_joint_to_rad(legal_current_raw))
    legal_target = tuple(float(value) for value in sdk_joint_to_rad(legal_target_raw))
    step_deg = min(joint_step_limit_deg, joint_speed_deg_s * dt_s)
    output = []
    for feedback, current, target, (low, high) in zip(
            current_joints_rad, legal_current, legal_target, joint_bounds_deg):
        if not math.isfinite(feedback) or not math.isfinite(current) or not math.isfinite(target):
            raise ValueError("preset joint values must be finite radians")
        low_rad, high_rad = math.radians(low), math.radians(high)
        # The native helper clamps to the strict task bounds. Clamp again to
        # the URDF/task intersection used by this operator before emitting.
        current = max(low_rad, min(high_rad, current))
        target = max(low_rad, min(high_rad, target))
        if not low_rad <= target <= high_rad:
            raise ValueError("saved joint target is outside the shared envelope")
        delta = max(-math.radians(step_deg), min(math.radians(step_deg), target-current))
        next_value = current + delta
        lead = math.radians(joint_target_lead_deg)
        lower = max(low_rad, feedback - lead)
        upper = min(high_rad, feedback + lead)
        if lower > upper:
            raise ValueError("joint feedback does not overlap the target lead envelope")
        output.append(max(low_rad, min(high_rad,
            max(lower, min(upper, next_value)))))

    gripper_delta = max(-gripper_step_raw, min(gripper_step_raw,
        target_gripper_raw-current_gripper_raw))
    gripper = current_gripper_raw + gripper_delta
    gripper = max(max(low_grip, current_gripper_raw-gripper_target_lead_raw),
                  min(min(high_grip, current_gripper_raw+gripper_target_lead_raw),
                      gripper))
    return tuple(output) + (float(gripper),)
