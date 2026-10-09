import struct
import pytest
from laptop.transport.position_commands import J1CommissioningBatch, CanTxFrame
from laptop.transport.gripper_commissioning import GripperOpeningStep


def batch(current, target=(0,)*6):
    return J1CommissioningBatch(1,1000,current,target,
        (CanTxFrame(0x151,bytes((1,1,5,0,0,0,0,0))),
         *(CanTxFrame(i,struct.pack('>ii',*target[2*p:2*p+2])) for p,i in enumerate((0x155,0x156,0x157)))),
        configure_control_mode=True,near_zero_home=True)


def test_home_exact_zero_and_near_source_only():
    assert batch((929,976,-949,935,984,983)).target_joint_raw_0p001deg==(0,)*6
    with pytest.raises(ValueError):
        batch((1600,0,0,0,0,0))
    with pytest.raises(ValueError):
        batch((929,976,-949,935,984,983),target=(100,0,0,0,0,0))


def test_close_scope_and_no_zero_calibration():
    command=GripperOpeningStep(5400,1000,1000,0,100,True)
    command.validate(1001)
    assert command.frame.data==struct.pack('>iHBB',0,100,1,0)
    with pytest.raises(ValueError):
        GripperOpeningStep(7000,1000,1000,0,100,True).validate(1001)
    with pytest.raises(ValueError):
        GripperOpeningStep(5400,1000,1000,0,300,True).validate(1001)


def test_max_open_exact_scope():
    command=GripperOpeningStep(5400,1000,1000,0,100,False,True)
    command.validate(1001)
    assert command.frame.data==struct.pack('>iHBB',70000,100,1,0)
    with pytest.raises(ValueError):
        GripperOpeningStep(5400,1000,1000,0,100,True,True).validate(1001)
    with pytest.raises(ValueError):
        GripperOpeningStep(5400,1000,1000,0,300,False,True).validate(1001)
