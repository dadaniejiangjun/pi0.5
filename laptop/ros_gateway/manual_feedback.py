"""Separate command receipt, observed motion, and precision arrival evidence."""
from __future__ import annotations

import math

from laptop.control.feedback_acceptance import (
    FeedbackAcceptanceLimits,
    MANUAL_FEEDBACK_LIMITS,
)


def _joint_raw(values):
    return tuple(round(math.degrees(float(value)) * 1000.0) for value in values)


def summarize_manual_feedback(
    samples,
    *,
    session_id,
    sent_local_ns,
    target_joint_rad=None,
    target_gripper_raw=None,
    starting_joint_rad=None,
    starting_gripper_raw=None,
    held_joint_rad=None,
    held_gripper_raw=None,
    limits: FeedbackAcceptanceLimits = MANUAL_FEEDBACK_LIMITS,
):
    """Classify fresh feedback without treating adapter receipts as motion.

    ``functional_motion`` means directionally consistent motion above the
    configured noise floor plus acceptable held-axis behavior. ``precision``
    separately reports final residual and whether the tolerance stayed stable.
    The nominal units do not establish mechanical scale.
    """
    if (target_joint_rad is None) == (target_gripper_raw is None):
        raise ValueError("exactly one manual target kind required")
    if not isinstance(limits, FeedbackAcceptanceLimits):
        raise TypeError("limits must be FeedbackAcceptanceLimits")
    if target_joint_rad is not None and (
        len(target_joint_rad) != 6
        or any(not math.isfinite(float(value)) for value in target_joint_rad)
    ):
        raise ValueError("six finite joint targets required")
    if starting_joint_rad is not None and (
        len(starting_joint_rad) != 6
        or any(not math.isfinite(float(value)) for value in starting_joint_rad)
    ):
        raise ValueError("six finite starting joint positions required")
    if starting_gripper_raw is not None and type(starting_gripper_raw) is not int:
        raise ValueError("starting gripper position must be integer raw units")

    accepted = []
    for sample in samples:
        if (
            sample.get("session_id") != session_id
            or sample.get("fake") is not False
            or sample.get("feedback_fresh") is not True
            or sample.get("hardware_connected") is not True
            or sample.get("ctrl_mode_raw") != 1
            or sample.get("arm_status_raw") != 0
            or sample.get("error_code") != 0
            or not all(sample.get("motor_enabled", ()))
        ):
            continue
        try:
            offset = sample["source_monotonic_ns"] - sample["source_local_monotonic_ns"]
            uncertainty = sample["source_clock_mapping_uncertainty_ns"]
            stamps = [sample["status_monotonic_ns"], *sample["motor_monotonic_ns"]]
            stamps += (
                sample["joint_pair_monotonic_ns"]
                if target_joint_rad is not None
                else [sample["gripper_monotonic_ns"]]
            )
            local_received_ns = min(stamps) - offset - uncertainty
        except (KeyError, TypeError, ValueError):
            continue
        if local_received_ns <= sent_local_ns:
            continue
        accepted.append(sample)

    baseline_joints = starting_joint_rad if starting_joint_rad is not None else held_joint_rad
    baseline_gripper = starting_gripper_raw if starting_gripper_raw is not None else held_gripper_raw
    if not accepted:
        return {
            "functional_motion": {"status": "INSUFFICIENT_FRESH_POST_SEND_FEEDBACK", "samples": 0},
            "precision": {"status": "NOT_ASSESSABLE_NO_FRESH_FEEDBACK"},
            "post_dispatch_samples": 0,
            "units": "nominal_sdk_only",
            "physical_scale_verified": False,
        }

    last = accepted[-1]
    held_failure = None
    if held_joint_rad is not None:
        if target_joint_rad is None:
            held_indices = range(6)
        else:
            baseline = starting_joint_rad if starting_joint_rad is not None else held_joint_rad
            target_raw = _joint_raw(target_joint_rad)
            baseline_raw = _joint_raw(baseline)
            held_indices = tuple(
                index for index in range(6)
                if abs(target_raw[index] - baseline_raw[index]) <= limits.motion_noise_joint_raw
            )
        if any(
            abs(last["joint_position_rad"][index] - held_joint_rad[index])
            > math.radians(limits.held_joint_drift_tolerance_raw / 1000.0)
            for index in held_indices
        ):
            held_failure = "HELD_JOINT_DRIFT_EXCEEDED"
    if held_gripper_raw is not None and abs(last["gripper_raw"] - held_gripper_raw) > limits.held_gripper_drift_tolerance_raw:
        held_failure = "HELD_GRIPPER_DRIFT_EXCEEDED"

    directions = []
    direction_evidence = {}
    movement_baseline_available = False
    if target_joint_rad is not None and baseline_joints is not None:
        movement_baseline_available = True
        baseline_raw = _joint_raw(baseline_joints)
        target_raw = _joint_raw(target_joint_rad)
        active = [i for i, (a, b) in enumerate(zip(baseline_raw, target_raw))
                  if abs(b - a) > limits.motion_noise_joint_raw]
        evidence = {i: False for i in active}
        wrong = []
        for sample in accepted:
            observed = _joint_raw(sample["joint_position_rad"])
            for i in range(6):
                if i not in active and abs(observed[i] - baseline_raw[i]) > limits.held_joint_drift_tolerance_raw:
                    held_failure = "HELD_JOINT_DRIFT_EXCEEDED"
                elif i in active:
                    signed = (observed[i] - baseline_raw[i]) * (1 if target_raw[i] > baseline_raw[i] else -1)
                    if signed > limits.motion_noise_joint_raw:
                        evidence[i] = True
                    elif signed < -limits.motion_noise_joint_raw:
                        wrong.append(i + 1)
            if held_gripper_raw is not None and abs(sample["gripper_raw"] - held_gripper_raw) > limits.held_gripper_drift_tolerance_raw:
                held_failure = "HELD_GRIPPER_DRIFT_EXCEEDED"
        direction_evidence = {f"joint_{i + 1}": value for i, value in evidence.items()}
        directions = ["increasing" if target_raw[i] > baseline_raw[i] else "decreasing" for i in active]
        if wrong:
            motion_status = "MOTION_OPPOSITE_TO_TARGET"
        elif not active:
            motion_status = "NO_EXPECTED_MOTION_ABOVE_NOISE_FLOOR"
        elif all(evidence.values()):
            motion_status = "EXPECTED_DIRECTIONAL_MOTION_CONFIRMED"
        else:
            motion_status = "EXPECTED_MOTION_NOT_ABOVE_NOISE_FLOOR"
    elif target_gripper_raw is not None and baseline_gripper is not None:
        movement_baseline_available = True
        difference = target_gripper_raw - baseline_gripper
        if abs(difference) <= limits.motion_noise_gripper_raw:
            motion_status = "NO_EXPECTED_MOTION_ABOVE_NOISE_FLOOR"
        else:
            sign = 1 if difference > 0 else -1
            reached = False
            wrong = False
            for sample in accepted:
                movement = sample["gripper_raw"] - baseline_gripper
                signed = movement * sign
                reached |= signed > limits.motion_noise_gripper_raw
                wrong |= signed < -limits.motion_noise_gripper_raw
                if held_joint_rad is not None and any(
                    abs(a - b) > math.radians(limits.held_joint_drift_tolerance_raw / 1000.0)
                    for a, b in zip(sample["joint_position_rad"], held_joint_rad)
                ):
                    held_failure = "HELD_JOINT_DRIFT_EXCEEDED"
            motion_status = (
                "MOTION_OPPOSITE_TO_TARGET" if wrong else
                "EXPECTED_DIRECTIONAL_MOTION_CONFIRMED" if reached else
                "EXPECTED_MOTION_NOT_ABOVE_NOISE_FLOOR"
            )
            direction_evidence = {"gripper": "increasing" if difference > 0 else "decreasing",
                                  "above_noise_floor": reached}
    else:
        motion_status = "NOT_ASSESSABLE_NO_BASELINE"

    if held_failure:
        motion_status = held_failure

    if target_joint_rad is not None:
        target_raw = _joint_raw(target_joint_rad)
        final_raw = _joint_raw(last["joint_position_rad"])
        residual_raw = [target - actual for target, actual in zip(target_raw, final_raw)]
        target_tolerance = limits.target_joint_tolerance_raw
        in_tolerance = all(abs(value) <= target_tolerance for value in residual_raw)
        precision_residual = {
            "joint_residual_raw_0p001deg": residual_raw,
            "joint_residual_deg": [value / 1000.0 for value in residual_raw],
            "joint_tolerance_raw": target_tolerance,
            "joint_tolerance_deg": target_tolerance / 1000.0,
        }
    else:
        residual = target_gripper_raw - last["gripper_raw"]
        in_tolerance = abs(residual) <= limits.target_gripper_tolerance_raw
        precision_residual = {
            "gripper_residual_raw": residual,
            "gripper_tolerance_raw": limits.target_gripper_tolerance_raw,
            "nominal_unit_scale_verified": False,
        }

    matching = []
    for sample in accepted:
        if target_joint_rad is not None:
            matches = all(abs(a - b) <= math.radians(target_tolerance / 1000.0)
                          for a, b in zip(sample["joint_position_rad"], target_joint_rad))
        else:
            matches = abs(sample["gripper_raw"] - target_gripper_raw) <= limits.target_gripper_tolerance_raw
        if not matches:
            matching = []
            continue
        if matching and (
            sample["sequence"] <= matching[-1]["sequence"]
            or sample["source_local_monotonic_ns"] - matching[-1]["source_local_monotonic_ns"] > limits.stable_max_gap_ns
        ):
            matching = []
        matching.append(sample)
    stable = bool(matching) and (
        matching[-1]["source_local_monotonic_ns"] - matching[0]["source_local_monotonic_ns"]
        >= limits.stable_duration_ns
    )
    precision_status = (
        "WITHIN_TOLERANCE_STABLE" if in_tolerance and stable else
        "OUTSIDE_TARGET_TOLERANCE" if not in_tolerance else
        "WITHIN_TOLERANCE_NOT_STABLE"
    )
    return {
        "functional_motion": {
            "status": motion_status,
            "direction_evidence": direction_evidence,
            "direction_labels": directions,
            "source_baseline_available": movement_baseline_available,
            "held_axis_tolerance_joint_raw_0p001deg": limits.held_joint_drift_tolerance_raw,
            "held_axis_tolerance_gripper_raw": limits.held_gripper_drift_tolerance_raw,
            "movement_noise_threshold_joint_raw_0p001deg": limits.motion_noise_joint_raw,
            "movement_noise_threshold_gripper_raw": limits.motion_noise_gripper_raw,
            "post_dispatch_samples": len(accepted),
        },
        "precision": {
            "status": precision_status,
            **precision_residual,
            "stable_duration_required_ns": limits.stable_duration_ns,
            "stable_duration_observed_ns": (
                matching[-1]["source_local_monotonic_ns"] - matching[0]["source_local_monotonic_ns"]
                if matching else 0
            ),
            "stable_samples": len(matching),
            "stable_max_gap_ns": limits.stable_max_gap_ns,
        },
        "post_dispatch_samples": len(accepted),
        "matching_samples": len(matching),
        "units": "nominal_sdk_only",
        "physical_scale_verified": False,
    }
