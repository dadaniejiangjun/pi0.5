import math
import pytest
from laptop.robot_client.bounded_policy_target import limit_target

def test_limits_realistic_large_prediction_and_invalid_gripper():
    current=[0,0,0,0,0,0,0.05]
    result=limit_target(current,[1,1,-1,1,1,1,-0.02])
    assert all(abs(v)<=math.radians(1)+1e-12 for v in result['delta'][:6])
    assert result['bounded_target'][6]==current[6]
    assert result['gripper_prediction_invalid']
    assert not result['hardware_control_allowed']

def test_gripper_step_and_j3_upper_limit():
    result=limit_target([0]*7,[0,0,1,0,0,0,0.07])
    assert result['bounded_target'][2]==0
    assert result['bounded_target'][6]==0.07

@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_nonfinite_rejected(value):
    with pytest.raises(ValueError):limit_target([0]*7,[value]*7)


def test_explicit_five_degree_limit_and_gripper_unchanged():
    r=limit_target([0]*7,[1,1,-1,1,1,1,0.07],joint_step_deg=5)
    assert r['delta'][:6]==pytest.approx([math.radians(v) for v in (5,5,-5,5,5,5)])
    assert r['delta'][6]==0.07
    with pytest.raises(ValueError):limit_target([0]*7,[0]*7,joint_step_deg=61)


def test_ten_degree_cap():
    r=limit_target([0]*7,[1,1,-1,1,1,1,0.07],joint_step_deg=10)
    assert r['delta'][:6]==pytest.approx([math.radians(v) for v in (10,10,-10,10,10,10)])
    assert r['delta'][6]==0.07


def test_twenty_degree_limit():
    r=limit_target([0]*7,[1,1,-1,1,1,1,0.07],joint_step_deg=20)
    assert r['delta'][:6]==pytest.approx([math.radians(v) for v in (20,20,-20,20,20,20)])
    assert r['delta'][6]==0.07


def test_sixty_degree_cap_preserves_sdk_limit_and_gripper_limit():
    r=limit_target([0]*7,[2,2,-2,2,2,2,0.07],joint_step_deg=60)
    assert r['delta'][:6]==pytest.approx([math.radians(v) for v in (60,60,-60,60,60,60)])
    assert r['delta'][6]==0.07
    with pytest.raises(ValueError):limit_target([0]*7,[0]*7,joint_step_deg=61)


def test_unlimited_gripper_closing_and_invalid_prediction_hold():
    r=limit_target([0]*6+[0.07],[0]*7)
    assert r['bounded_target'][6]==0
    assert r['gripper_step_limit_m'] is None
    r=limit_target([0]*6+[0.07],[0]*6+[0.2])
    assert r['bounded_target'][6]==0.07
    with pytest.raises(ValueError):limit_target([0]*6+[-0.01],[0]*7)
