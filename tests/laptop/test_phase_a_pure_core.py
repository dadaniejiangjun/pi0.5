"""Golden equivalence against pre-refactor functions, without hardware/network."""
import ast
import copy
import json
import math
import os
import struct
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from laptop.piper import unit_conversion as units
from laptop.client import live_shadow, observation_schema, policy_client
from laptop.camera.worker import camera_loop
from laptop.robot_client.feedback import decode_feedback_frame, sdk_feedback_snapshot
from laptop.robot_client.piper_session import PiperSession

ROOT = Path(__file__).resolve().parents[2]
BEFORE = json.loads((ROOT/'tests/fixtures/phase_a_before.json').read_text(encoding='utf-8'))
FEEDBACK_FIXTURE = Path(os.environ.get('PIPER_TEST_FEEDBACK_FIXTURE', str(ROOT/'tests/fixtures/phase_a_recorded_feedback.json')))
if not FEEDBACK_FIXTURE.is_file():
    FEEDBACK_FIXTURE = ROOT/'tests/fixtures/phase_a_synthetic_feedback.json'
RECORDED = json.loads(FEEDBACK_FIXTURE.read_text(encoding='utf-8'))


def old_function(path, name, namespace):
    tree = ast.parse(BEFORE[path])
    nodes = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(nodes) == 1
    scope = dict(namespace)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), '<phase-a-old-pure-function>', 'exec'), scope)
    return scope[name]


@pytest.mark.parametrize('raw', [0,1,-1,929,-12345,150000,-150000,180000,-170000,
                                100000,-100000,70000,-70000,120000,-120000,
                                149999,150001,0.5,-0.5])
def test_scalar_joint_exact_old_formula(raw):
    old = raw * math.pi / 180000
    assert units.joint_raw_to_nominal_rad(raw) == old
    assert units.nominal_rad_to_joint_raw(old) == round(old*180000/math.pi)
    assert isinstance(units.nominal_rad_to_joint_raw(old), int)


@pytest.mark.parametrize('raw', [0,1,-1,1000,1234,5000,10000,70000,99999,100000,100001])
def test_gripper_arithmetic_variants_exact(raw):
    assert units.gripper_raw_to_nominal_m(raw) == raw * 1e-6
    assert units.gripper_raw_to_nominal_m_divided(raw) == raw / 1e6
    for v in (raw*1e-6, raw/1e6):
        assert units.nominal_m_to_gripper_raw(v) == round(v*1e6)


@pytest.mark.parametrize('value', [0.,1e-10,-1e-10,0.0005,-0.0005,math.pi,math.pi/3,
                                  0.1,float('nan'),float('inf'),-float('inf')])
def test_integer_quantization_preserves_round_and_exceptions(value):
    for new, old in [(units.nominal_rad_to_joint_raw,lambda v:round(v*180000/math.pi)),
                     (units.nominal_m_to_gripper_raw,lambda v:round(v*1e6))]:
        try:expected=old(value)
        except Exception as exc:
            with pytest.raises(type(exc)):new(value)
        else:assert new(value)==expected


def test_existing_numpy_api_unchanged_against_old_source():
    for name in ('sdk_joint_to_rad','rad_to_sdk_joint','sdk_gripper_to_m','m_to_sdk_gripper'):
        old=old_function('laptop/piper/unit_conversion.py',name,vars(units))
        new=getattr(units,name)
        for data in (0,0.5,-0.5,[0,1,-1,150000,100000],np.asarray([0.,0.5,-0.5])):
            np.testing.assert_allclose(new(data),old(data),rtol=0,atol=0)
            assert type(new(data)) is type(old(data))


@pytest.mark.parametrize('row', RECORDED['frames'])
def test_recorded_frame_counts_exact(row):
    cid=row['can_id'];data=bytes.fromhex(row['data_hex']);stamp=row['received_monotonic_ns']
    got=decode_feedback_frame(cid,data,stamp)
    if cid==0x2A1:
        expected=dict(ctrl_mode_raw=data[0],arm_status_raw=data[1],move_mode_raw=data[2],
                      teach_status_raw=data[3],error_code=int.from_bytes(data[6:8],'big'))
    elif cid==0x2A8:
        expected=dict(position_raw=int.from_bytes(data[:4],'big',signed=True),
                      effort_raw=int.from_bytes(data[4:6],'big',signed=True),status_raw=data[6])
    else:expected=dict(joint_pair=struct.unpack('>ii',data))
    assert got.fields==expected
    assert (got.arbitration_id,got.received_monotonic_ns)==(cid,stamp)


def test_signed_boundaries_unknown_and_sdk_projection():
    assert decode_feedback_frame(0x123,bytes(8),5) is None
    got=decode_feedback_frame(0x2A5,struct.pack('>ii',-2147483648,2147483647),7)
    assert got.fields['joint_pair']==(-2147483648,2147483647)
    assert decode_feedback_frame(0x2A8,struct.pack('>ihBB',-1,-32768,255,0),7).fields == dict(position_raw=-1,effort_raw=-32768,status_raw=255)
    import can
    from piper_sdk.piper_msgs.msg_v2 import PiperMessage
    from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
    parser,message=C_PiperParserV2(),PiperMessage()
    for row in RECORDED['frames']:
        payload=bytes.fromhex(row['data_hex'])
        assert parser.DecodeMessage(can.Message(arbitration_id=row['can_id'],data=payload,is_extended_id=False),message)
        old=dict(joint_raw=[getattr(message.arm_joint_feedback,f'joint_{i}') for i in range(1,7)],
                 gripper_raw=dict(angle_or_stroke=message.gripper_feedback.grippers_angle,
                                  effort=message.gripper_feedback.grippers_effort,
                                  status_code=message.gripper_feedback.status_code),
                 status=dict(ctrl_mode=str(message.arm_status_msgs.ctrl_mode),arm_status=str(message.arm_status_msgs.arm_status),
                             teach_status=str(message.arm_status_msgs.teach_status),err_code=message.arm_status_msgs.err_code))
        assert sdk_feedback_snapshot(message,gripper_details=True)==old


def test_complete_snapshot_and_staleness_match_old(monkeypatch):
    import laptop.robot_client.piper_session as module
    old=old_function('laptop/robot_client/piper_session.py','state',vars(module))
    now=1_000_000_000
    monkeypatch.setattr(module.time,'perf_counter_ns',lambda:now)
    latest={r['can_id']:(now-5_000_000,bytes.fromhex(r['data_hex'])) for r in RECORDED['frames']}
    latest.update({i:(now-5_000_000,bytes((0,0,0,0,0,64,0,0))) for i in range(0x261,0x267)})
    session=SimpleNamespace(_owner=lambda:None,_latest=latest,bridge=SimpleNamespace(state=SimpleNamespace(value='FAKE')))
    for age in (120,0.001):assert old(session,age)==PiperSession.state(session,age)
    for key in list(latest):
        reduced=dict(latest);reduced.pop(key);session._latest=reduced
        assert old(session)==PiperSession.state(session)


@pytest.mark.parametrize('state',[list(range(7)),np.arange(7,dtype=np.float64),np.zeros(7,dtype=np.float32)])
def test_observation_images_state_prompt_exact(state):
    base=np.arange(90,dtype=np.uint8).reshape(5,6,3);wrist=base[::-1].copy()
    new=observation_schema.build_piper_observation(base,wrist,state,'task')
    old=dict(base_rgb=base,wrist_rgb=wrist,state=np.asarray(state,dtype=np.float32),prompt='task')
    assert list(new)==list(old)
    for key in ('base_rgb','wrist_rgb','state'):
        assert np.array_equal(new[key],old[key])
        assert new[key].dtype==old[key].dtype
        assert new[key].shape==old[key].shape
    assert new['base_rgb'] is base and new['wrist_rgb'] is wrist
    assert new['prompt']==old['prompt']


def test_shadow_bgr_conversion_and_validation_match_old():
    old=old_function('laptop/client/live_shadow.py','select_observation',vars(live_shadow))
    now=1_000_000_000;base=np.zeros((5,6,3),dtype=np.uint8);base[:,:,0]=7;base[:,:,2]=91
    cameras={k:dict(frame=base.copy(),timestamp_ns=now-10) for k in ('base','wrist')}
    feedback=dict(timestamps_ns={k:now-10 for k in live_shadow.REQUIRED_CAN_CHANNELS},joint_raw=[929,1000,-1000,10000,-12000,7000],gripper_raw=-1234)
    expected,timing=old(cameras,feedback,now);got,newtiming=live_shadow.select_observation(cameras,feedback,now)
    for key in ('base_rgb','wrist_rgb','state'):assert np.array_equal(got[key],expected[key])
    assert got['base_rgb'][0,0].tolist()==[91,0,7]
    assert got['prompt']==expected['prompt'] and timing==newtiming
    for bad in ({},dict(feedback,joint_raw=[0]*5),dict(feedback,gripper_raw=None)):
        with pytest.raises(ValueError) as a:old(cameras,bad,now)
        with pytest.raises(ValueError) as b:live_shadow.select_observation(cameras,bad,now)
        assert str(a.value)==str(b.value)


def outcome(fn,*args):
    try:return ('ok',fn(*args))
    except Exception as exc:return ('error',type(exc),str(exc))


def test_identity_accept_reject_old_new():
    from tests.laptop.test_live_shadow import LiveShadowTest
    fixture=LiveShadowTest();metadata=fixture.policy_metadata();lock=fixture.identity_lock()
    scope=dict(vars(policy_client))
    oldlock=old_function('laptop/client/live_shadow.py','validate_artifact_identity_lock',scope)
    scope['validate_artifact_identity_lock']=oldlock
    oldmeta=old_function('laptop/client/live_shadow.py','validate_policy_metadata',scope)
    assert outcome(oldmeta,metadata,lock)==outcome(policy_client.validate_policy_metadata,metadata,lock)
    for key in metadata:
        bad=dict(metadata);bad[key]=None
        assert outcome(oldmeta,bad,lock)==outcome(policy_client.validate_policy_metadata,bad,lock)
    for key in lock:
        bad=dict(lock);bad[key]=None
        assert outcome(oldlock,bad)==outcome(policy_client.validate_artifact_identity_lock,bad)
    derived=json.loads((ROOT/'validation/live5000_identity_20261008.json').read_text())
    meta=dict(derived,can_opened=False,can_data_tx_calls=0,control_enabled=False,robot_control_ready=False,dataset_identity=derived['dataset'])
    assert outcome(oldmeta,meta,derived)==outcome(policy_client.validate_policy_metadata,meta,derived)
    for key in derived:
        bad=dict(meta);bad[key]=None
        assert outcome(oldmeta,bad,derived)==outcome(policy_client.validate_policy_metadata,bad,derived)


def test_prewarm_no_connection_same_payload_and_rejections(monkeypatch):
    old=old_function('laptop/client/live_shadow.py','prewarm_policy',vars(policy_client))
    monkeypatch.setattr(policy_client.time,'perf_counter_ns',lambda:1_000_000_000)
    class Client:
        def __init__(self,actions):self.actions=actions;self.observation=None
        def infer(self,observation):self.observation=observation;return {'actions':self.actions}
    for actions in (np.zeros((16,7)),np.zeros((7,)),np.full((16,7),np.nan)):
        a,b=Client(actions),Client(actions)
        assert outcome(old,a)==outcome(policy_client.prewarm_policy,b)
        for key in ('base_rgb','wrist_rgb','state'):assert np.array_equal(a.observation[key],b.observation[key])
        assert a.observation['prompt']==b.observation['prompt']


def test_public_camera_worker_exact_body_and_fake_io(monkeypatch):
    old=old_function('laptop/client/live_shadow.py','_camera_loop',vars(live_shadow))
    import inspect
    before=ast.parse(inspect.getsource(camera_loop)).body[0]
    prior=next(n for n in ast.parse(BEFORE['laptop/client/live_shadow.py']).body if isinstance(n,ast.FunctionDef) and n.name=='_camera_loop')
    assert [ast.dump(n) for n in before.body]==[ast.dump(n) for n in prior.body]
    from laptop.camera import source
    def run(fn,fail=False):
        stop=threading.Event();shared={};errors=[];events=[]
        image=np.arange(12,dtype=np.uint8).reshape(2,2,3)
        class Camera:
            def isOpened(self):return True
            def read(self):
                if fail:raise RuntimeError('fixture read failure')
                stop.set();return True,image
            def release(self):events.append('release')
        monkeypatch.setattr(source,'open_camera',lambda index:Camera())
        monkeypatch.setattr(live_shadow.time,'perf_counter_ns',lambda:123)
        fn(0,'base',shared,threading.Lock(),stop,errors,lambda role,frame,stamp:events.append((role,frame.tolist(),stamp)))
        return {k:dict(frame=v['frame'].tolist(),timestamp_ns=v['timestamp_ns']) for k,v in shared.items()},errors,events
    assert run(old)==run(camera_loop)
    assert run(old,True)==run(camera_loop,True)
    assert live_shadow._camera_loop is camera_loop


def test_bounded_worker_control_ast_unchanged_after_helper_expansion():
    import inspect
    from laptop.client.bounded_model_trial import worker
    old=next(n for n in ast.parse(BEFORE['laptop/client/bounded_model_trial.py']).body if isinstance(n,ast.FunctionDef) and n.name=='worker')
    new=ast.parse(inspect.getsource(worker)).body[0]
    class Expand(ast.NodeTransformer):
        def visit_Name(self,n):
            if n.id=='camera_loop':n.id='_camera_loop'
            return n
        def visit_Call(self,n):
            n=self.generic_visit(n)
            if not isinstance(n.func,ast.Name):return n
            formulas={'joint_raw_to_nominal_rad':'VALUE*math.pi/180000','nominal_rad_to_joint_raw':'round(VALUE*180000/math.pi)',
                      'gripper_raw_to_nominal_m_divided':'VALUE/1e6','nominal_m_to_gripper_raw':'round(VALUE*1e6)'}
            if n.func.id in formulas:
                expression=ast.parse(formulas[n.func.id],mode='eval').body
                class Value(ast.NodeTransformer):
                    def visit_Name(self,v):return copy.deepcopy(n.args[0]) if v.id=='VALUE' else v
                return Value().visit(expression)
            if n.func.id=='build_piper_observation':
                state=ast.Call(func=ast.Attribute(value=ast.Name(id='np',ctx=ast.Load()),attr='asarray',ctx=ast.Load()),args=[n.args[2]],keywords=[ast.keyword(arg='dtype',value=ast.Attribute(value=ast.Name(id='np',ctx=ast.Load()),attr='float32',ctx=ast.Load()))])
                return ast.Dict(keys=[ast.Constant(k) for k in ('base_rgb','wrist_rgb','state','prompt')],values=[n.args[0],n.args[1],state,n.args[3]])
            if n.func.id=='dict' and [k.arg for k in n.keywords]==['base_rgb','wrist_rgb','state','prompt']:
                return ast.Dict(keys=[ast.Constant(k.arg) for k in n.keywords],values=[k.value for k in n.keywords])
            return n
    assert ast.dump(Expand().visit(new))==ast.dump(Expand().visit(old))
