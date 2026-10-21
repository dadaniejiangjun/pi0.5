"""Explicit nominal-unit limiting for supervised model trials; never sends CAN."""
import math
from laptop.robot_client.replay_preflight import NOMINAL_JOINT_BOUNDS_DEG
from laptop.piper.control_profile import (
    CONTINUOUS_GRIPPER_MAX_M,
    CONTINUOUS_GRIPPER_MAX_STEP_M,
    CONTINUOUS_GRIPPER_MIN_M,
)

def limit_target(state, prediction, *, joint_step_deg=1,
                 gripper_step_m=CONTINUOUS_GRIPPER_MAX_STEP_M):
    if not math.isfinite(joint_step_deg) or not 0<joint_step_deg<=20:
        raise ValueError("joint step must be in (0, 20] degrees")
    current=tuple(float(v) for v in state);raw=tuple(float(v) for v in prediction)
    if len(current)!=7 or len(raw)!=7 or not all(math.isfinite(v) for v in current+raw):
        raise ValueError('finite seven-dimensional state and prediction required')
    targets=[]
    for value,wanted,(lo,hi) in zip(current[:6],raw[:6],NOMINAL_JOINT_BOUNDS_DEG):
        low=max(math.radians(lo),value-math.radians(joint_step_deg))
        high=min(math.radians(hi),value+math.radians(joint_step_deg))
        if low>high:raise ValueError('current joint outside bounded recovery envelope')
        targets.append(max(low,min(high,wanted)))
    # Invalid model gripper values are held, rather than interpreted as a closing command.
    invalid=not CONTINUOUS_GRIPPER_MIN_M<=raw[6]<=CONTINUOUS_GRIPPER_MAX_M
    wanted=current[6] if invalid else raw[6]
    if not CONTINUOUS_GRIPPER_MIN_M<=current[6]<=CONTINUOUS_GRIPPER_MAX_M:
        raise ValueError('current gripper outside tested continuous-task envelope')
    if (not math.isfinite(gripper_step_m)
            or not 0<gripper_step_m<=CONTINUOUS_GRIPPER_MAX_STEP_M):
        raise ValueError('gripper step limit exceeds the continuous-task profile')
    low=max(CONTINUOUS_GRIPPER_MIN_M,current[6]-gripper_step_m)
    high=min(CONTINUOUS_GRIPPER_MAX_M,current[6]+gripper_step_m)
    if low>high:raise ValueError('current gripper outside nominal command envelope')
    targets.append(max(low,min(high,wanted)))
    return dict(raw_prediction=list(raw),source_state=list(current),bounded_target=targets,
                delta=[a-b for a,b in zip(targets,current)],gripper_prediction_invalid=invalid,
                target_changed=targets!=list(raw),units='nominal radian x6 and meter',
                gripper_step_limit_m=gripper_step_m,hardware_control_allowed=False)
