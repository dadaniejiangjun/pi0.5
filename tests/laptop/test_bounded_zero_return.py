import pytest
from laptop.robot_client.bounded_zero_return import zero_target

def test_zero_segments_converge_without_overshoot():
    pose=(26335,19869,-10365,131,-21314,-6901)
    for _ in range(6):
        target=zero_target(pose)
        assert all(abs(a-b)<=5000 and abs(b)<=abs(a) and a*b>=0 for a,b in zip(pose,target))
        pose=target
    assert pose==(0,)*6

def test_invalid_joint_vector_rejected():
    with pytest.raises(ValueError):zero_target((0,)*5)
