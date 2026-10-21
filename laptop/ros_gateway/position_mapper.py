"""Owner-thread ROS position mapping; authorization remains with SafetyBridge.

Uses the existing SDK unit conversions and typed frame validator. It does not
grant calibration, open CAN, initialize mode, or enable motors.
"""
import struct
import time

from laptop.control.contracts import (
    ABSOLUTE_JOINT_POSITION_COMMAND, JOINT_GRIPPER_UNITS, CommandProposal,
)
from laptop.piper.unit_conversion import m_to_sdk_gripper, rad_to_sdk_joint
from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_LIMITS,
    GRIPPER_NOMINAL_METERS_PER_RAW,
    validate_continuous_gripper_target,
)
from laptop.transport.position_commands import CanTxFrame, PositionCommandBatch


def map_position_target(session, *, operation, arguments, request_id,
                        task_scope_id, source_state_sequence,
                        valid_until_monotonic_ns, gateway_state,
                        holding_effort_raw=0):
    authorization = session.bridge.active_task_position_authorization
    if authorization is None or authorization.task_scope_id != task_scope_id:
        raise PermissionError("matching local task authorization required")
    gates = dict(authorization.site_gate_attestations)
    if not gates.get("policy_to_command_mapping_verified"):
        raise PermissionError("position mapping has not been verified locally")
    if gateway_state["sequence"] != source_state_sequence:
        raise PermissionError("source gateway state changed")
    snapshot = session.task_control_snapshot()
    now = time.perf_counter_ns()
    snapshot.validate_for_position_control(now_monotonic_ns=now)
    state = snapshot.robot_state
    joints, gripper = state.joint_position_rad, state.gripper_position_m
    if type(holding_effort_raw) is not int or holding_effort_raw != CONTINUOUS_GRIPPER_LIMITS.effort_raw:
        raise ValueError("holding effort differs from the continuous-task profile")
    effort = holding_effort_raw
    if operation == "set_joint_target":
        joints = tuple(arguments["joint_position_rad"])
        current_gripper_raw = round(state.gripper_position_m / GRIPPER_NOMINAL_METERS_PER_RAW)
        validate_continuous_gripper_target(current_gripper_raw, current_gripper_raw, effort)
    elif operation == "set_gripper":
        gripper = arguments["nominal_position_m"]
        if int(m_to_sdk_gripper(gripper)) != arguments["gripper_raw"]:
            raise ValueError("gripper raw and nominal position disagree")
        effort = round(arguments["nominal_effort_nm"] * 1000)
        current_gripper_raw = round(state.gripper_position_m / GRIPPER_NOMINAL_METERS_PER_RAW)
        validate_continuous_gripper_target(
            current_gripper_raw, arguments["gripper_raw"], effort,
        )
    else:
        raise ValueError("unsupported position operation")
    created = max(now, state.received_monotonic_ns + 1)
    if valid_until_monotonic_ns <= created:
        raise TimeoutError("position request expired")
    proposal = CommandProposal(
        proposal_id=f"ros:{request_id}", source_request_id=request_id,
        source_state_sequence=state.sequence, created_monotonic_ns=created,
        valid_until_monotonic_ns=valid_until_monotonic_ns,
        target=tuple(joints) + (gripper,),
        action_semantics=ABSOLUTE_JOINT_POSITION_COMMAND,
        units=JOINT_GRIPPER_UNITS, hardware_units_verified=state.units_verified,
        mapping_verified=gates["policy_to_command_mapping_verified"],
        task_scope_id=task_scope_id, model_profile=authorization.model_profile,
        checkpoint_id=authorization.checkpoint_id,
    )
    raw = tuple(int(rad_to_sdk_joint(value)) for value in joints)
    frames = tuple(CanTxFrame(0x155 + i, struct.pack(">ii", *raw[i*2:i*2+2]))
                   for i in range(3)) + (
        CanTxFrame(0x159, struct.pack(">iHBB", int(m_to_sdk_gripper(gripper)), effort, 1, 0)),
    )
    return snapshot, PositionCommandBatch(proposal, frames)
