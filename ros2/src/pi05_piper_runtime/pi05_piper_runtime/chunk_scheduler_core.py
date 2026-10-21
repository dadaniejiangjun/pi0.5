"""Fail-closed preview gate for rolling PiPER policy chunks.

This module creates an auditable *candidate target preview*. It deliberately
does not publish a ROS command, call SafetyBridge, or access CAN.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers
from typing import Any

from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_MAX_M,
    CONTINUOUS_GRIPPER_MAX_STEP_M,
    CONTINUOUS_GRIPPER_MIN_M,
)


ACTION_SEMANTICS = "next_observed_state_proxy_nominal_units"
ACTION_ROWS = 16
ACTION_COLUMNS = 7
LOCAL_MONOTONIC_CLOCK_ID = "wsl_perf_counter_ns"


class TargetRejected(ValueError):
    """A chunk or state failed a preview gate; no target may be produced."""


@dataclass(frozen=True)
class SchedulerLimits:
    joint_min_rad: tuple[float, ...]
    joint_max_rad: tuple[float, ...]
    gripper_min_m: float = CONTINUOUS_GRIPPER_MIN_M
    gripper_max_m: float = CONTINUOUS_GRIPPER_MAX_M
    max_gripper_step_m: float = CONTINUOUS_GRIPPER_MAX_STEP_M
    max_joint_step_deg: float = 20.0
    speed_percent: int = 5
    update_period_ns: int = 100_000_000
    max_state_age_ns: int = 120_000_000
    max_chunk_age_ns: int = 250_000_000
    max_response_age_ns: int = 250_000_000
    max_observation_age_ns: int = 500_000_000
    max_inference_latency_ns: int = 500_000_000
    max_task_duration_ns: int = 120_000_000_000

    def __post_init__(self) -> None:
        try:
            raw_lower = tuple(self.joint_min_rad)
            raw_upper = tuple(self.joint_max_rad)
        except TypeError as exc:
            raise ValueError("joint limits must contain six values") from exc
        if len(raw_lower) != 6 or len(raw_upper) != 6:
            raise ValueError("joint limits must contain six values")
        lower_values: list[float] = []
        upper_values: list[float] = []
        for lower, upper in zip(raw_lower, raw_upper):
            if (isinstance(lower, bool) or isinstance(upper, bool)
                    or not isinstance(lower, numbers.Real)
                    or not isinstance(upper, numbers.Real)):
                raise ValueError("joint bounds must be finite and ordered")
            lower, upper = float(lower), float(upper)
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError("joint bounds must be finite and ordered")
            lower_values.append(lower)
            upper_values.append(upper)
        object.__setattr__(self, "joint_min_rad", tuple(lower_values))
        object.__setattr__(self, "joint_max_rad", tuple(upper_values))
        if (isinstance(self.gripper_min_m, bool) or isinstance(self.gripper_max_m, bool)
                or not isinstance(self.gripper_min_m, numbers.Real)
                or not isinstance(self.gripper_max_m, numbers.Real)):
            raise ValueError("gripper bounds must be finite and ordered")
        gripper_min, gripper_max = float(self.gripper_min_m), float(self.gripper_max_m)
        if (not math.isfinite(gripper_min) or not math.isfinite(gripper_max)
                or gripper_min >= gripper_max):
            raise ValueError("gripper bounds must be finite and ordered")
        object.__setattr__(self, "gripper_min_m", gripper_min)
        object.__setattr__(self, "gripper_max_m", gripper_max)
        if (isinstance(self.max_gripper_step_m, bool)
                or not isinstance(self.max_gripper_step_m, numbers.Real)
                or not math.isfinite(float(self.max_gripper_step_m))
                or self.max_gripper_step_m <= 0):
            raise ValueError("max_gripper_step_m must be positive and finite")
        object.__setattr__(self, "max_gripper_step_m", float(self.max_gripper_step_m))
        # Continuous task preview must share the same tested raw range and
        # one-step limit as the Windows SafetyBridge.
        if (gripper_min != CONTINUOUS_GRIPPER_MIN_M
                or gripper_max != CONTINUOUS_GRIPPER_MAX_M
                or float(self.max_gripper_step_m) != CONTINUOUS_GRIPPER_MAX_STEP_M):
            raise ValueError("continuous gripper preview must match the shared control profile")
        if (isinstance(self.max_joint_step_deg, bool)
                or not isinstance(self.max_joint_step_deg, numbers.Real)
                or not math.isfinite(float(self.max_joint_step_deg))
                or self.max_joint_step_deg <= 0):
            raise ValueError("max_joint_step_deg must be positive and finite")
        object.__setattr__(self, "max_joint_step_deg", float(self.max_joint_step_deg))
        if type(self.speed_percent) is not int or self.speed_percent != 5:
            raise ValueError("the selected initial scheduler profile fixes speed at 5 percent")
        for field_name in (
            "update_period_ns", "max_state_age_ns", "max_chunk_age_ns",
            "max_response_age_ns", "max_observation_age_ns",
            "max_inference_latency_ns", "max_task_duration_ns",
        ):
            value = getattr(self, field_name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{field_name} must be a positive integer")


@dataclass(frozen=True)
class TargetPreview:
    target_position: tuple[float, ...]
    target_time_monotonic_ns: int
    target_expiry_monotonic_ns: int
    speed_percent: int
    row_index: int
    source_observation_id: str
    state_sequence: int
    state_age_ns: int
    observation_age_ns: int
    prediction_age_ns: int
    chunk_age_ns: int
    inference_latency_ns: int
    task_elapsed_ns: int
    emitted_to_hardware: bool = False
    hardware_control_allowed: bool = False
    semantics: str = ACTION_SEMANTICS


def _field(value: Any, name: str) -> Any:
    if isinstance(value, dict):
        try:
            return value[name]
        except KeyError as exc:
            raise TargetRejected(f"missing {name}") from exc
    try:
        return getattr(value, name)
    except AttributeError as exc:
        raise TargetRejected(f"missing {name}") from exc


def _int(value: Any, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or int(value) < minimum:
        raise TargetRejected(f"{name} must be an integer >= {minimum}")
    return int(value)


def _sequence_component(value: Any, name: str) -> int:
    if not isinstance(value, str) or not value.isascii() or not value.isdecimal():
        raise TargetRejected(f"{name} must be a positive decimal sequence")
    result = int(value)
    if result <= 0:
        raise TargetRejected(f"{name} must be a positive decimal sequence")
    return result


def _bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise TargetRejected(f"{name} must be boolean")
    return value


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise TargetRejected(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise TargetRejected(f"{name} must be a finite number")
    return result


def preview_next_target(
    *,
    chunk: Any,
    state: Any,
    now_monotonic_ns: int,
    task_started_monotonic_ns: int,
    chunk_received_monotonic_ns: int,
    state_received_monotonic_ns: int,
    last_preview_monotonic_ns: int | None,
    last_observation_id: str | None,
    limits: SchedulerLimits,
    allow_fake: bool = False,
) -> TargetPreview:
    """Validate the next receding-horizon target without creating a command.

    Freshness is measured using receive timestamps from this scheduler process.
    Windows source monotonic timestamps are carried for provenance but are not
    compared with the WSL scheduler clock.
    """
    now = _int(now_monotonic_ns, "now_monotonic_ns", minimum=1)
    started = _int(task_started_monotonic_ns, "task_started_monotonic_ns", minimum=1)
    chunk_received = _int(chunk_received_monotonic_ns, "chunk_received_monotonic_ns", minimum=1)
    state_received = _int(state_received_monotonic_ns, "state_received_monotonic_ns", minimum=1)
    if started > now or chunk_received > now or state_received > now:
        raise TargetRejected("scheduler timestamps cannot be in the future")
    elapsed = now - started
    if elapsed >= limits.max_task_duration_ns:
        raise TargetRejected("task duration limit reached")

    if _field(state, "source_clock_id") not in ("windows_perf_counter_ns", "linux_perf_counter_ns"):
        raise TargetRejected("state source timestamp uses an unsupported monotonic clock")
    _int(_field(state, "source_monotonic_ns"), "state.source_monotonic_ns", minimum=1)
    state_local_ns = _int(
        _field(state, "source_local_monotonic_ns"),
        "state.source_local_monotonic_ns", minimum=1,
    )
    if _field(state, "source_local_clock_id") != LOCAL_MONOTONIC_CLOCK_ID:
        raise TargetRejected("state source timestamp is not in the WSL monotonic clock")
    state_uncertainty_ns = _int(
        _field(state, "source_clock_mapping_uncertainty_ns"),
        "state.source_clock_mapping_uncertainty_ns",
    )
    if state_uncertainty_ns > 20_000_000:
        raise TargetRejected("state clock mapping uncertainty exceeds 20 ms")
    if state_local_ns > state_received + state_uncertainty_ns:
        raise TargetRejected("state source timestamp is after its local receive time")
    state_age = now - state_local_ns
    if state_age < -state_uncertainty_ns:
        raise TargetRejected("state source timestamp is in the future")
    if state_age > limits.max_state_age_ns:
        raise TargetRejected("state feedback source timestamp is stale at the scheduler")
    if last_preview_monotonic_ns is not None:
        last_preview = _int(last_preview_monotonic_ns, "last_preview_monotonic_ns", minimum=1)
        if now < last_preview or now - last_preview < limits.update_period_ns:
            raise TargetRejected("10 Hz target update is not due")

    session_id = _field(state, "session_id")
    if not isinstance(session_id, str) or not session_id:
        raise TargetRejected("state session_id is missing")
    sequence = _int(_field(state, "sequence"), "state.sequence", minimum=1)
    if _bool(_field(state, "feedback_fresh"), "state.feedback_fresh") is not True:
        raise TargetRejected("state feedback is marked stale")
    fake = _bool(_field(state, "fake"), "state.fake")
    connected = _bool(_field(state, "hardware_connected"), "state.hardware_connected")
    if fake and not allow_fake:
        raise TargetRejected("synthetic state is disabled in this profile")
    if not fake and not connected:
        raise TargetRejected("hardware state is disconnected")
    if _int(_field(state, "error_code"), "state.error_code") != 0:
        raise TargetRejected("robot reports a fault")
    if _int(_field(state, "arm_status_raw"), "state.arm_status_raw") != 0:
        raise TargetRejected("robot arm status is not NORMAL")
    if not fake:
        if _int(_field(state, "ctrl_mode_raw"), "state.ctrl_mode_raw") != 1:
            raise TargetRejected("hardware is not in CAN_CTRL mode")
        motor_enabled = list(_field(state, "motor_enabled"))
        if len(motor_enabled) != 6 or any(type(value) is not bool or not value for value in motor_enabled):
            raise TargetRejected("all six hardware motors must report enabled")

    raw_joints = list(_field(state, "joint_position_rad"))
    if len(raw_joints) != 6:
        raise TargetRejected("feedback must contain six finite joint positions")
    joints = tuple(_finite_float(value, "joint feedback") for value in raw_joints)
    gripper = _finite_float(_field(state, "gripper_nominal_m"), "gripper feedback")
    for index, (value, lower, upper) in enumerate(
        zip(joints, limits.joint_min_rad, limits.joint_max_rad), start=1
    ):
        if value < lower or value > upper:
            raise TargetRejected(f"current joint {index} is outside SDK nominal bounds")
    if gripper < limits.gripper_min_m or gripper > limits.gripper_max_m:
        raise TargetRejected("current gripper feedback is outside the configured nominal range")

    chunk_session = _field(chunk, "observation_id")
    if not isinstance(chunk_session, str) or not chunk_session.startswith(session_id + ":"):
        raise TargetRejected("policy chunk does not belong to the active sensor session")
    id_fields = chunk_session.split(":")
    if len(id_fields) != 4:
        raise TargetRejected("policy observation id does not contain its state and camera sequences")
    chunk_state_sequence = _sequence_component(id_fields[1], "observation state sequence")
    _sequence_component(id_fields[2], "observation Base sequence")
    _sequence_component(id_fields[3], "observation Wrist sequence")
    if chunk_state_sequence > sequence:
        raise TargetRejected("policy chunk references a future state sequence")
    if last_observation_id is not None and chunk_session == last_observation_id:
        raise TargetRejected("duplicate policy chunk")
    if _field(chunk, "action_semantics") != ACTION_SEMANTICS:
        raise TargetRejected("policy action semantics are not the approved proxy contract")
    if _bool(_field(chunk, "prediction_only"), "chunk.prediction_only") is not True:
        raise TargetRejected("policy chunk is not marked prediction-only")
    if _bool(_field(chunk, "hardware_control_allowed"), "chunk.hardware_control_allowed"):
        raise TargetRejected("policy metadata unexpectedly claims hardware authorization")
    if _bool(_field(chunk, "fake_observation"), "chunk.fake_observation") != fake:
        raise TargetRejected("policy chunk and state fake flags differ")
    if _bool(_field(chunk, "hardware_connected"), "chunk.hardware_connected") != connected:
        raise TargetRejected("policy chunk and state connection flags differ")

    if _field(chunk, "source_clock_id") != _field(state, "source_clock_id"):
        raise TargetRejected("policy and state source clocks differ")
    if _field(chunk, "local_clock_id") != LOCAL_MONOTONIC_CLOCK_ID:
        raise TargetRejected("policy response and targets are not in the WSL monotonic clock")
    observation_local_ns = _int(
        _field(chunk, "observation_local_monotonic_ns"),
        "chunk.observation_local_monotonic_ns", minimum=1,
    )
    if observation_local_ns > now:
        raise TargetRejected("policy observation timestamp is in the future")
    response_local_ns = _int(
        _field(chunk, "response_local_monotonic_ns"),
        "chunk.response_local_monotonic_ns", minimum=1,
    )
    if observation_local_ns > response_local_ns:
        raise TargetRejected("policy response precedes its mapped observation")
    if response_local_ns > chunk_received or chunk_received > now:
        raise TargetRejected("policy response and receipt times are out of order")
    chunk_age = now - chunk_received
    if chunk_age > limits.max_chunk_age_ns:
        raise TargetRejected("policy chunk is stale at the scheduler")
    prediction_age = now - response_local_ns
    if prediction_age > limits.max_response_age_ns:
        raise TargetRejected("policy prediction is stale")
    inference_latency = _int(_field(chunk, "inference_latency_ns"), "chunk.inference_latency_ns")
    if inference_latency > limits.max_inference_latency_ns:
        raise TargetRejected("inference latency exceeds the selected freshness budget")
    observation_age = now - observation_local_ns
    if observation_age > limits.max_observation_age_ns:
        raise TargetRejected("policy observation is stale")
    if _int(_field(chunk, "source_monotonic_ns"), "chunk.source_monotonic_ns", minimum=1) <= 0:
        raise TargetRejected("source timestamp is missing")
    if _int(_field(chunk, "rows"), "chunk.rows") != ACTION_ROWS:
        raise TargetRejected("policy chunk must contain 16 rows")
    if _int(_field(chunk, "columns"), "chunk.columns") != ACTION_COLUMNS:
        raise TargetRejected("policy chunk must contain seven dimensions")
    raw_flat = list(_field(chunk, "target_position"))
    if len(raw_flat) != ACTION_ROWS * ACTION_COLUMNS:
        raise TargetRejected("policy chunk must contain 112 finite values")
    flat = tuple(_finite_float(value, "policy target") for value in raw_flat)

    raw_target_times = list(_field(chunk, "target_local_monotonic_ns"))
    if len(raw_target_times) != ACTION_ROWS:
        raise TargetRejected("policy chunk must contain 16 local target timestamps")
    target_times = tuple(
        _int(value, "chunk.target_local_monotonic_ns[]", minimum=1)
        for value in raw_target_times
    )
    expected_target_times = tuple(
        observation_local_ns + (index + 1) * limits.update_period_ns
        for index in range(ACTION_ROWS)
    )
    if target_times != expected_target_times:
        raise TargetRejected("policy target timestamps do not match the 10 Hz action horizon")

    # Each row remains valid only until the next 10 Hz slot. Skip expired
    # prefixes but retain the model's original schedule and never retimestamp.
    row_index = next(
        (index for index, target_time in enumerate(target_times)
         if now < target_time + limits.update_period_ns),
        None,
    )
    if row_index is None:
        raise TargetRejected("all policy target rows have expired")
    target = flat[row_index * ACTION_COLUMNS:(row_index + 1) * ACTION_COLUMNS]
    max_delta_rad = math.radians(limits.max_joint_step_deg)
    for index, (value, current, lower, upper) in enumerate(
        zip(target[:6], joints, limits.joint_min_rad, limits.joint_max_rad), start=1
    ):
        if value < lower or value > upper:
            raise TargetRejected(f"predicted joint {index} is outside the configured task motion envelope")
        if abs(value - current) > max_delta_rad + 1e-12:
            raise TargetRejected(f"predicted joint {index} exceeds the 20 degree single-step limit")
    if target[6] < limits.gripper_min_m or target[6] > limits.gripper_max_m:
        raise TargetRejected("predicted gripper is outside the configured nominal range")
    if abs(target[6] - gripper) > limits.max_gripper_step_m + 1e-12:
        raise TargetRejected("predicted gripper exceeds the continuous-task step limit")

    return TargetPreview(
        target_position=target,
        target_time_monotonic_ns=target_times[row_index],
        target_expiry_monotonic_ns=target_times[row_index] + limits.update_period_ns,
        speed_percent=limits.speed_percent,
        row_index=row_index,
        source_observation_id=chunk_session,
        state_sequence=sequence,
        state_age_ns=state_age,
        observation_age_ns=observation_age,
        prediction_age_ns=prediction_age,
        chunk_age_ns=chunk_age,
        inference_latency_ns=inference_latency,
        task_elapsed_ns=elapsed,
    )
