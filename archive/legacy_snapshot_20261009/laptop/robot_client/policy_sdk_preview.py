"""Non-executable mapping preview from Pi0.5's 7D output to PiPER V2 SDK values.

The accepted action is a derived future-observed-state target, not a verified
PiPER command. This module only converts nominal units and formats CAN frame
previews; it never constructs a robot SDK interface or sends a frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

from laptop.control.contracts import (
    JOINT_GRIPPER_UNITS,
    NEXT_OBSERVED_STATE_PROXY,
    PolicyPrediction,
    RobotStateSnapshot,
    TimingContract,
)
from laptop.piper.unit_conversion import m_to_sdk_gripper, rad_to_sdk_joint
from laptop.robot_client.joint_command_preview import CanFramePreview
from laptop.robot_client.replay_preflight import NOMINAL_JOINT_BOUNDS_DEG


DERIVED_ACTION_TARGET_KIND = "derived_future_observed_state_position_target"
SDK_JOINT_DEGREES_PER_UNIT = 0.001
SDK_GRIPPER_MM_PER_UNIT = 0.001
GRIPPER_INT32_MIN = -(2**31)
GRIPPER_INT32_MAX = 2**31 - 1


@dataclass(frozen=True)
class PolicySdkPreview:
    """A read-only SDK call/frame preview with no hardware authority."""

    request_id: str
    action_index: int
    target_monotonic_ns: int
    evaluated_monotonic_ns: int
    source_state_sequence: int
    checkpoint_identity: str
    config_identity: str
    action_semantics: str
    action_target_kind: str
    joint_target_rad_nominal: tuple[float, ...]
    gripper_target_m_nominal: float
    joint_delta_rad_nominal: tuple[float, ...]
    gripper_delta_m_nominal: float
    preview_max_joint_delta_rad: tuple[float, ...]
    preview_max_gripper_delta_m: float
    joint_ctrl_args_0p001deg: tuple[int, ...]
    gripper_ctrl_args: tuple[int, int, int, int]
    frames: tuple[CanFramePreview, ...]
    configured_gripper_max_mm: float | None
    gripper_target_exceeds_configured_max: bool | None
    sdk_runtime_gripper_clamp_simulated: bool
    action_units_verified_on_hardware: bool = False
    policy_to_robot_command_mapping_verified: bool = False
    hardware_control_allowed: bool = False
    can_tx_allowed: bool = False


def _future_action_index(prediction: PolicyPrediction, now_monotonic_ns: int) -> int:
    for index, target_ns in enumerate(prediction.target_timestamps_monotonic_ns):
        if target_ns > now_monotonic_ns:
            return index
    raise ValueError("all policy action targets have expired")


def preview_policy_sdk_values(
    prediction: PolicyPrediction,
    *,
    current_state: RobotStateSnapshot,
    now_monotonic_ns: int,
    gripper_effort_raw: int,
    max_joint_delta_rad: tuple[float, ...],
    max_gripper_delta_m: float,
    action_index: int | None = None,
    configured_gripper_max_mm: float | None = None,
    timing: TimingContract = TimingContract(),
) -> PolicySdkPreview:
    """Map one future proxy row to nominal V2 ``JointCtrl``/``GripperCtrl`` args.

    ``gripper_effort_raw`` is required because the 7D policy predicts position
    only. ``configured_gripper_max_mm`` is diagnostic context, not a hardware
    range proof; over-range candidates are reported without silently clamping.
    The selected action must still be in the future on the laptop monotonic
    clock. The policy response and source state must also be fresh, and the
    action must fit caller-supplied preview-only per-step deltas. These
    thresholds are not hardware limits. No
    ``CommandProposal`` is created, so SafetyBridge cannot mistake this
    representation conversion for a validated command mapping.
    """
    if type(prediction) is not PolicyPrediction:
        raise TypeError("a typed prediction-only PolicyPrediction is required")
    if prediction.action_semantics != NEXT_OBSERVED_STATE_PROXY:
        raise ValueError("only next-observed-state proxy predictions are supported")
    if tuple(prediction.units) != JOINT_GRIPPER_UNITS:
        raise ValueError("expected nominal radian x6 plus meter action units")
    if not prediction.prediction_only or prediction.hardware_units_verified:
        raise ValueError("derived proxy predictions must remain prediction-only and nominal")
    if type(current_state) is not RobotStateSnapshot:
        raise TypeError("a typed current RobotStateSnapshot is required")
    if type(timing) is not TimingContract:
        raise TypeError("timing must be a TimingContract")
    if type(now_monotonic_ns) is not int or now_monotonic_ns <= 0:
        raise ValueError("now_monotonic_ns must be a positive integer")
    if prediction.observation_monotonic_ns > now_monotonic_ns:
        raise ValueError("prediction observation timestamp is in the future")
    if prediction.received_monotonic_ns > now_monotonic_ns:
        raise ValueError("policy response timestamp is in the future")
    if prediction.response_latency_ns > timing.max_prediction_response_latency_ns:
        raise ValueError("policy response latency exceeds the configured bound")
    if (now_monotonic_ns - prediction.received_monotonic_ns
            > timing.max_prediction_response_latency_ns):
        raise ValueError("policy response is stale")
    if now_monotonic_ns < current_state.received_monotonic_ns:
        raise ValueError("current time precedes the received robot-state snapshot")
    if (now_monotonic_ns - current_state.received_monotonic_ns
            > timing.max_state_age_ns):
        raise ValueError("current robot-state snapshot is stale")
    if not current_state.connected or not current_state.status_ok:
        raise ValueError("current robot state is disconnected or reports an error")
    try:
        step_limits = tuple(max_joint_delta_rad)
    except TypeError as exc:
        raise ValueError("max_joint_delta_rad must contain six finite positive values") from exc
    if (len(step_limits) != 6 or any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(float(value)) or value <= 0
            for value in step_limits)):
        raise ValueError("max_joint_delta_rad must contain six finite positive values")
    if (isinstance(max_gripper_delta_m, bool)
            or not isinstance(max_gripper_delta_m, (int, float))
            or not math.isfinite(float(max_gripper_delta_m))
            or max_gripper_delta_m <= 0):
        raise ValueError("max_gripper_delta_m must be finite and positive")
    if (type(gripper_effort_raw) is not int
            or not 0 <= gripper_effort_raw <= 5000):
        raise ValueError("explicit integer SDK gripper effort in [0, 5000] is required")
    if configured_gripper_max_mm is not None:
        if (isinstance(configured_gripper_max_mm, bool)
                or not isinstance(configured_gripper_max_mm, (int, float))
                or not math.isfinite(float(configured_gripper_max_mm))
                or configured_gripper_max_mm <= 0):
            raise ValueError("configured_gripper_max_mm must be finite and positive")

    if action_index is None:
        selected_index = _future_action_index(prediction, now_monotonic_ns)
    else:
        if type(action_index) is not int or not 0 <= action_index < len(prediction.actions):
            raise ValueError("action_index must select a row in the prediction horizon")
        selected_index = action_index
    target_ns = prediction.target_timestamps_monotonic_ns[selected_index]
    if target_ns <= now_monotonic_ns:
        raise ValueError("selected policy action target has expired")

    action = prediction.actions[selected_index]
    joint_degrees = tuple(math.degrees(value) for value in action[:6])
    current_joint_degrees = tuple(math.degrees(value)
                                  for value in current_state.joint_position_rad)
    joint_raw: list[int] = []
    joint_delta_rad: list[float] = []
    for index, (degrees, (minimum, maximum)) in enumerate(
            zip(joint_degrees, NOMINAL_JOINT_BOUNDS_DEG)):
        if not minimum <= degrees <= maximum:
            raise ValueError(f"joint_{index + 1} target exceeds the SDK nominal position range")
        current_degrees = current_joint_degrees[index]
        if not minimum <= current_degrees <= maximum:
            raise ValueError(f"joint_{index + 1} current state exceeds the SDK nominal position range")
        raw = int(rad_to_sdk_joint(action[index], SDK_JOINT_DEGREES_PER_UNIT))
        quantized_degrees = raw * SDK_JOINT_DEGREES_PER_UNIT
        if not minimum <= quantized_degrees <= maximum:
            raise ValueError(f"quantized joint_{index + 1} target exceeds the SDK nominal range")
        delta_rad = math.radians(quantized_degrees - current_degrees)
        if abs(delta_rad) > float(step_limits[index]):
            raise ValueError(f"joint_{index + 1} target exceeds the preview single-step envelope")
        joint_raw.append(raw)
        joint_delta_rad.append(delta_rad)

    gripper_target_m = float(action[6])
    gripper_delta_m = gripper_target_m - current_state.gripper_position_m
    if abs(gripper_delta_m) > float(max_gripper_delta_m):
        raise ValueError("gripper target exceeds the preview single-step envelope")
    gripper_raw = int(m_to_sdk_gripper(gripper_target_m, SDK_GRIPPER_MM_PER_UNIT))
    if not GRIPPER_INT32_MIN <= gripper_raw <= GRIPPER_INT32_MAX:
        raise ValueError("gripper target is not representable by the SDK int32 field")
    if gripper_raw < 0:
        raise ValueError("negative gripper stroke is not a valid SDK position target")

    joint_args = tuple(joint_raw)
    gripper_args = (gripper_raw, gripper_effort_raw, 0x01, 0x00)
    frames = tuple(
        CanFramePreview(
            0x155 + pair,
            struct.pack(">ii", *joint_args[2 * pair:2 * pair + 2]),
        )
        for pair in range(3)
    ) + (
        CanFramePreview(0x159, struct.pack(">iHBB", *gripper_args)),
    )
    configured_max = (None if configured_gripper_max_mm is None
                      else float(configured_gripper_max_mm))
    exceeds_configured_max = (
        None if configured_max is None
        else gripper_target_m * 1000.0 > configured_max
    )
    return PolicySdkPreview(
        request_id=prediction.request_id,
        action_index=selected_index,
        target_monotonic_ns=target_ns,
        evaluated_monotonic_ns=now_monotonic_ns,
        source_state_sequence=current_state.sequence,
        checkpoint_identity=prediction.checkpoint_identity,
        config_identity=prediction.config_identity,
        action_semantics=prediction.action_semantics,
        action_target_kind=DERIVED_ACTION_TARGET_KIND,
        joint_target_rad_nominal=tuple(action[:6]),
        gripper_target_m_nominal=gripper_target_m,
        joint_delta_rad_nominal=tuple(joint_delta_rad),
        gripper_delta_m_nominal=gripper_delta_m,
        preview_max_joint_delta_rad=tuple(float(value) for value in step_limits),
        preview_max_gripper_delta_m=float(max_gripper_delta_m),
        joint_ctrl_args_0p001deg=joint_args,
        gripper_ctrl_args=gripper_args,
        frames=frames,
        configured_gripper_max_mm=configured_max,
        gripper_target_exceeds_configured_max=exceeds_configured_max,
        # PiperInterfaceV2 optionally clamps only when
        # start_sdk_gripper_limit=True. This preview does not instantiate it.
        sdk_runtime_gripper_clamp_simulated=False,
    )
