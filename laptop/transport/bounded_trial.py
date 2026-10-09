"""Raw SDK-unit contract for an explicitly supervised three-update model trial."""
from dataclasses import dataclass
import struct,time
from laptop.transport.position_commands import CanTxFrame,COMMISSIONING_JOINT_BOUNDS_RAW

@dataclass(frozen=True)
class BoundedTrialBatch:
    source_sequence:int
    current_joints:tuple
    target_joints:tuple
    current_gripper:int
    target_gripper:int
    prediction_request_id:str
    expires_ns:int
    joint_step_limit_raw:int=1000
    gripper_step_limit_raw:int | None=1000
    def __post_init__(self):
        if type(self.source_sequence) is not int or self.source_sequence<0:raise ValueError('sequence required')
        if not self.prediction_request_id:raise ValueError('prediction request required')
        if type(self.expires_ns) is not int or self.expires_ns<=0:raise ValueError('expiry required')
        if type(self.current_joints) is not tuple or type(self.target_joints) is not tuple:raise TypeError('immutable joint tuples required')
        if len(self.current_joints)!=6 or len(self.target_joints)!=6:raise ValueError('six joints required')
        if any(type(v) is not int for v in self.current_joints+self.target_joints+(self.current_gripper,self.target_gripper)):raise ValueError('integer SDK units required')
        if type(self.joint_step_limit_raw) is not int or not 0<self.joint_step_limit_raw<=60000:raise ValueError('joint step limit must be in (0,60000]')
        if any(abs(a-b)>self.joint_step_limit_raw for a,b in zip(self.current_joints,self.target_joints)):raise ValueError('trial exceeds configured joint step')
        if any(not lo<=v<=hi for v,(lo,hi) in zip(self.target_joints,COMMISSIONING_JOINT_BOUNDS_RAW)):raise ValueError('joint target out of SDK range')
        if not 0<=self.current_gripper<=100000 or not 0<=self.target_gripper<=100000:raise ValueError('gripper exceeds SDK range')
        if self.gripper_step_limit_raw is not None:
            if type(self.gripper_step_limit_raw) is not int or not 0<self.gripper_step_limit_raw<=100000:raise ValueError('invalid gripper step limit')
            if abs(self.target_gripper-self.current_gripper)>self.gripper_step_limit_raw:raise ValueError('gripper exceeds configured step limit')
    @property
    def frames(self):
        return tuple(CanTxFrame(0x155+i,struct.pack('>ii',*self.target_joints[2*i:2*i+2])) for i in range(3))+(CanTxFrame(0x159,struct.pack('>iHBB',self.target_gripper,0,1,0)),)
    def validate_time(self,now):
        if not 0<self.expires_ns-now<=250_000_000:raise PermissionError('trial expired or approval window too long')
