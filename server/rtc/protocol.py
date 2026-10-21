"""Versioned canonical-unit RTC requests; server clocks are never compared to WSL."""
import math
import numpy as np

PROTOCOL='piper_guided_rtc_v1'
HORIZON=16
DIMENSION=7
PERIOD_NS=100_000_000


def prefix_weights(delay, capacity, total=16, schedule='EXP'):
    if type(delay) is not int or type(capacity) is not int or not 0<=delay<=capacity<=total:
        raise ValueError('RTC delay/capacity outside horizon')
    w=np.zeros(total,dtype=np.float32);w[:delay]=1
    middle=np.linspace(1,0,capacity-delay+2,dtype=np.float32)[1:-1]
    if schedule=='EXP':middle=middle*np.expm1(middle)/(math.e-1)
    elif schedule=='ZEROS':middle*=0
    elif schedule=='ONES':middle[:]=1
    elif schedule!='LINEAR':raise ValueError('unsupported prefix schedule')
    w[delay:capacity]=middle
    return w


def validate_request(request, identity):
    if not isinstance(request,dict) or request.get('protocol')!=PROTOCOL:
        raise ValueError('unsupported RTC protocol')
    if request.get('mode') not in ('plain','guided'):raise ValueError('unsupported RTC mode')
    seed=request.get('sampling_seed')
    if seed is not None and (type(seed) is not int or not 0<=seed<2**32):
        raise ValueError('invalid paired-evaluation seed')
    for key in ('session_id','request_id','prompt'):
        if not isinstance(request.get(key),str) or not request[key]:raise ValueError('missing RTC '+key)
    for key in ('model_id','checkpoint_id'):
        if request.get(key)!=identity[key]:raise ValueError('RTC identity mismatch: '+key)
    if request.get('period_ns')!=PERIOD_NS or request.get('horizon')!=16 or request.get('dimension')!=7:
        raise ValueError('RTC action contract mismatch')
    obs=request.get('observation_ns');delay=request.get('inference_delay');cap=request.get('execution_horizon')
    if type(obs) is not int or obs<=0:raise ValueError('invalid RTC observation time')
    if type(delay) is not int or type(cap) is not int or not 0<=delay<=cap<=8:
        raise ValueError('RTC delay/capacity invalid or unsupported')
    if request.get('schedule') not in ('EXP','LINEAR','ONES','ZEROS'):raise ValueError('RTC schedule unsupported')
    gain=request.get('max_guidance_weight')
    if isinstance(gain,bool) or not isinstance(gain,(int,float)) or not math.isfinite(gain) or not 0<=gain<=10:
        raise ValueError('RTC guidance weight outside 0..10')
    prefix=np.asarray(request.get('prefix'),dtype=np.float32)
    count=request.get('prefix_length')
    if type(count) is not int or not 0<=count<=cap:raise ValueError('RTC prefix length invalid')
    if count==0:
        if prefix.size or request.get('prefix_times_ns')!=[]:raise ValueError('RTC cold prefix malformed')
        return None
    if request['mode']!='guided' or count<delay:raise ValueError('RTC prefix does not cover estimated delay')
    if prefix.shape!=(count,7) or not np.isfinite(prefix).all():raise ValueError('RTC prefix shape/values invalid')
    if request.get('prefix_times_ns')!=[obs+(i+1)*PERIOD_NS for i in range(count)]:
        raise ValueError('RTC prefix time alignment invalid')
    source=request.get('source')
    if not isinstance(source,dict) or source.get('session_id')!=request['session_id'] or source.get('prompt')!=request['prompt']:
        raise ValueError('RTC prefix session/task mismatch')
    if any(source.get(k)!=identity[k] for k in ('model_id','checkpoint_id')):
        raise ValueError('RTC prefix source identity mismatch')
    if not isinstance(source.get('observation_id'),str) or not source['observation_id'].startswith(request['session_id']+':'):
        raise ValueError('RTC prefix source observation invalid')
    for key,budget in (('observation_age_ns',500_000_000),('response_age_ns',250_000_000)):
        if type(source.get(key)) is not int or not 0<=source[key]<=budget:raise ValueError('RTC stale prefix source')
    return prefix


def make_request(*, observation_ns, now_ns, session_id, request_id, prompt, identity,
                 delay=2, capacity=4, plan=None, mode='guided', gain=10.0, schedule='EXP'):
    r=dict(protocol=PROTOCOL,mode=mode,session_id=session_id,request_id=request_id,prompt=prompt,
        **identity,period_ns=PERIOD_NS,horizon=16,dimension=7,observation_ns=int(observation_ns),
        inference_delay=int(delay),execution_horizon=int(capacity),schedule=schedule,
        max_guidance_weight=float(gain),prefix_length=0,prefix=[],prefix_times_ns=[],source=None,
        prefix_reason='cold_start')
    if mode=='guided' and plan:
        ages=(now_ns-plan['observation_ns'],now_ns-plan['response_ns'])
        compatible=(plan['session_id']==session_id and plan['prompt']==prompt
          and all(plan[k]==identity[k] for k in ('model_id','checkpoint_id'))
          and plan['observation_ns']<observation_ns and 0<=ages[0]<=500_000_000 and 0<=ages[1]<=250_000_000)
        if compatible:
            times=np.asarray(plan['times_ns'],dtype=np.int64);values=np.asarray(plan['actions'],dtype=np.float64)
            if times.shape!=(16,) or values.shape!=(16,7) or not np.isfinite(values).all() or np.any(np.diff(times)!=PERIOD_NS):
                raise ValueError('invalid local prefix plan')
            targets=[observation_ns+(i+1)*PERIOD_NS for i in range(capacity)]
            targets=[t for t in targets if times[0]<=t<=times[-1]]
            if len(targets)>=delay and targets:
                r.update(prefix=np.stack([np.interp(targets,times,values[:,j]) for j in range(7)],axis=1).tolist(),
                  prefix_length=len(targets),prefix_times_ns=targets,prefix_reason='aligned_prior_plan',
                  source=dict(session_id=session_id,prompt=prompt,**identity,observation_id=plan['observation_id'],
                    observation_age_ns=int(ages[0]),response_age_ns=int(ages[1])))
            else:r['prefix_reason']='insufficient_overlap'
        else:r['prefix_reason']='incompatible_or_stale_plan'
    if mode=='plain':r['prefix_reason']='explicit_plain'
    validate_request(r,identity)
    return r


class DelayEstimator:
    def __init__(self, capacity=4):self.samples=[];self.capacity=capacity
    def add(self, latency_ns):
        if latency_ns<0:raise ValueError('negative latency')
        self.samples=(self.samples+[int(latency_ns)])[-64:]
    def steps(self):
        if not self.samples:return 2
        p95=sorted(self.samples)[max(0,math.ceil(.95*len(self.samples))-1)]
        # Do not clamp silently: an unsupported delay must fail admission.
        return max(1,math.ceil(p95/PERIOD_NS))
