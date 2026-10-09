import struct,time
import pytest
from laptop.transport.bounded_trial import BoundedTrialBatch
from laptop.transport.can_transport import CanFrame
from tests.laptop.test_j1_commissioning_bridge import MemoryJ1Adapter,_bridge,_queue_stable_feedback


def setup_trial(fail_at=None):
    adapter=MemoryJ1Adapter(fail_at=fail_at);bridge=_bridge(adapter)
    _queue_stable_feedback(adapter,joint_pose=(0,)*6)
    adapter.incoming.append(CanFrame(0x2A8,struct.pack('>iHBB',0,0,0,0),False,False,time.perf_counter_ns()))
    while adapter.incoming:bridge.receive_frame(timeout_ms=1)
    snapshot=bridge.create_j1_commissioning_snapshot(physical_estop_ready=True,low_speed_confirmed=True,motion_envelope_cleared=True)
    bridge.record_heartbeat(time.perf_counter_ns())
    batch=BoundedTrialBatch(snapshot.sequence,(0,)*6,(1000,1000,-1000,1000,1000,1000),0,1000,'test-request',time.perf_counter_ns()+100_000_000)
    return adapter,bridge,snapshot,batch


def test_valid_exact_frames_and_one_use():
    a,b,s,batch=setup_trial()
    writes=b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=True)
    assert [w.arbitration_id for w in writes]==[0x151,0x155,0x156,0x157,0x159]
    assert writes[0].data==bytes((1,1,5,0,0,0,0,0))
    assert writes[-1].data==struct.pack('>iHBB',1000,0,1,0)
    with pytest.raises(PermissionError):b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=True)
    assert len(a.sent)==5
    b.close()


def test_missing_approval_no_writes():
    a,b,s,batch=setup_trial()
    with pytest.raises(PermissionError):b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=False)
    assert not a.sent;b.close()


def test_partial_write_latches_no_retry():
    a,b,s,batch=setup_trial(fail_at=3)
    with pytest.raises(OSError):b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=True)
    assert b.transport.position_delivery_unknown
    assert a.closed
    with pytest.raises(PermissionError):b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=True)
    assert len(a.sent)==3;b.close()


def test_excess_delta_rejected():
    with pytest.raises(ValueError):BoundedTrialBatch(0,(0,)*6,(1001,0,0,0,0,0),0,0,'x',1)


def test_expired_rejected_before_write():
    a,b,s,batch=setup_trial()
    from dataclasses import replace
    batch=replace(batch,expires_ns=1)
    with pytest.raises(PermissionError):b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=True)
    assert not a.sent;b.close()



def test_explicit_five_degree_batch_and_excess_rejection():
    from dataclasses import replace
    a,b,s,batch=setup_trial()
    expanded=replace(batch,target_joints=(5000,0,0,0,0,0),joint_step_limit_raw=5000)
    writes=b.send_bounded_model_trial(snapshot=s,batch=expanded,operator_approved=True)
    assert writes[1].data==struct.pack('>ii',5000,0)
    with pytest.raises(ValueError):replace(expanded,target_joints=(5001,0,0,0,0,0))
    with pytest.raises(ValueError):replace(expanded,joint_step_limit_raw=60001)
    b.close()


def test_ten_degree_batch_cap():
    from dataclasses import replace
    a,b,s,batch=setup_trial()
    expanded=replace(batch,target_joints=(10000,0,0,0,0,0),joint_step_limit_raw=10000)
    assert expanded.frames[0].data==struct.pack('>ii',10000,0)
    with pytest.raises(ValueError):replace(expanded,target_joints=(10001,0,0,0,0,0))
    b.close()


def test_twenty_degree_batch_cap():
    from dataclasses import replace
    a,b,s,batch=setup_trial()
    batch=replace(batch,target_joints=(20000,0,0,0,0,0),joint_step_limit_raw=20000)
    assert batch.frames[0].data==struct.pack('>ii',20000,0)
    with pytest.raises(ValueError):replace(batch,target_joints=(20001,0,0,0,0,0))
    b.close()


def test_stability_wait_receives_only_and_preserves_other_errors():
    from laptop.client.bounded_model_trial import stable_snapshot
    class Robot:
        calls=0
        polls=0
        def commissioning_snapshot(self,**site):
            self.calls+=1
            if self.calls==1:raise PermissionError('joint feedback must stay within SDK bounds and drift')
            return 'stable'
        def poll(self,duration):self.polls+=1
    robot=Robot()
    assert stable_snapshot(robot,{},time.perf_counter_ns()+1_000_000_000)=='stable'
    assert robot.polls==1
    class Fault(Robot):
        def commissioning_snapshot(self,**site):raise PermissionError('motor disabled')
    fault=Fault()
    with pytest.raises(PermissionError,match='motor disabled'):stable_snapshot(fault,{},time.perf_counter_ns()+1_000_000_000)
    assert fault.polls==0


def test_sixty_degree_batch_limit_and_sdk_bounds():
    from dataclasses import replace
    a,b,s,batch=setup_trial()
    batch=replace(batch,target_joints=(60000,0,0,0,0,0),joint_step_limit_raw=60000)
    assert batch.frames[0].data==struct.pack('>ii',60000,0)
    with pytest.raises(ValueError):replace(batch,target_joints=(60001,0,0,0,0,0))
    with pytest.raises(ValueError):replace(batch,current_joints=(0,0,0,0,69000,0),target_joints=(0,0,0,0,71000,0))
    b.close()


def test_explicit_no_gripper_step_cap_preserves_range_and_exact_encoding():
    from dataclasses import replace
    a,b,s,batch=setup_trial()
    batch=replace(batch,target_gripper=70000,gripper_step_limit_raw=None)
    writes=b.send_bounded_model_trial(snapshot=s,batch=batch,operator_approved=True)
    assert writes[-1].data==struct.pack('>iHBB',70000,0,1,0)
    with pytest.raises(ValueError):replace(batch,target_gripper=100001)
    with pytest.raises(ValueError):replace(batch,target_gripper=-1)
    with pytest.raises(ValueError):replace(batch,gripper_step_limit_raw=1000)
    b.close()
