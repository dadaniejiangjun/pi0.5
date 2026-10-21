"""Opt-in Guided RTC shadow client; default policy node remains plain."""
import json,time
from pathlib import Path
from .policy_shadow_node import PolicyShadowNode
from .async_preview_guard import state_reason
from server.rtc.protocol import PROTOCOL,make_request,DelayEstimator
from server.rtc.policy import identity_from_metadata


class GuidedPolicyShadowNode(PolicyShadowNode):
    def __init__(self):
        self._rtc_plan=None;self._rtc_connection=None;self._rtc_delay=DelayEstimator();self._rtc_generation=0
        super().__init__()
        self.declare_parameter('rtc_mode','guided');self.declare_parameter('rtc_execution_horizon',4)
        self.declare_parameter('rtc_max_guidance_weight',10.0);self.declare_parameter('rtc_log','')
        self.declare_parameter('rtc_observation_dir','')
        self.rtc_mode=str(self.get_parameter('rtc_mode').value)
        self.rtc_capacity=int(self.get_parameter('rtc_execution_horizon').value)
        self.rtc_gain=float(self.get_parameter('rtc_max_guidance_weight').value)
        self.rtc_log=str(self.get_parameter('rtc_log').value)
        self.rtc_observation_dir=str(self.get_parameter('rtc_observation_dir').value)
        self._rtc_saved=0
        if self.rtc_mode not in ('plain','guided') or not 2<=self.rtc_capacity<=8 or not 0<=self.rtc_gain<=10:
            raise ValueError('invalid Guided RTC client configuration')
    def _connect_and_prewarm(self):
        client,metadata,warmup=super()._connect_and_prewarm()
        if metadata.get('rtc_protocol')!=PROTOCOL or not {'plain','guided'}<=set(metadata.get('rtc_modes',[])):
            client.close();raise ValueError('server does not declare Guided RTC capability')
        return client,metadata,warmup
    def _on_state(self,state):
        now=time.perf_counter_ns()
        if state_reason(state,now)!='ok' or (self._state is not None and
                (state.session_id!=self._state.session_id or state_reason(self._state,now)!='ok')):
            self._rtc_plan=None;self._rtc_generation+=1
        super()._on_state(state)
    def _prepare_request(self,observation,timing):
        if self._rtc_connection is not self._client:
            self._rtc_plan=None;self._rtc_delay=DelayEstimator();self._rtc_connection=self._client
        delay=self._rtc_delay.steps()
        if delay>self.rtc_capacity:
            self._rtc_plan=None;self._rtc_delay=DelayEstimator()
            raise ValueError('RTC estimated delay exceeds prefix capacity; cleared prefix for a fresh cold request')
        r=make_request(observation_ns=int(timing['observation_local_monotonic_ns']),now_ns=time.perf_counter_ns(),
          session_id=timing['session_id'],request_id=timing['observation_id'],prompt=observation['prompt'],
          identity=identity_from_metadata(self._metadata_cache),delay=delay,capacity=self.rtc_capacity,
          plan=self._rtc_plan,mode=self.rtc_mode,gain=self.rtc_gain)
        timing['rtc_request']=r
        timing['rtc_generation']=self._rtc_generation
        if (self.rtc_observation_dir and self._rtc_saved<5
                and (self.rtc_mode=='plain' or r['prefix_length']>0)):
            import numpy as np
            path=Path(self.rtc_observation_dir);path.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(path/f'observation_{self._rtc_saved}.npz',
                **observation,rtc_request_json=json.dumps(r))
            self._rtc_saved+=1
        return dict(observation,_rtc=r)
    def _run_inference(self,observation,timing,client,metadata):
        result=super()._run_inference(observation,timing,client,metadata)
        reply=result['rtc'];r=observation['_rtc'];expected='guided' if r['prefix_length'] and r['mode']=='guided' else 'plain'
        if (not isinstance(reply,dict) or reply.get('protocol')!=PROTOCOL or reply.get('request_id')!=r['request_id']
                or reply.get('mode')!=expected or reply.get('prefix_length')!=r['prefix_length']):
            raise ValueError('RTC response does not match request; no silent fallback')
        return result
    def _publish_chunk(self,output):
        if output['timing']['rtc_generation']!=self._rtc_generation:
            self._last_status='discarded RTC response from before state recovery'
            return
        old=self._prediction_count;super()._publish_chunk(output)
        if self._prediction_count==old:return
        self._rtc_delay.add(round(output['latency_ms']*1e6))
        t=output['timing'];r=t['rtc_request'];now=time.perf_counter_ns()
        if state_reason(self._state,now)=='ok' and 0<=now-t['observation_local_monotonic_ns']<=500_000_000 and 0<=now-output['response_local_monotonic_ns']<=250_000_000:
            self._rtc_plan=dict(session_id=t['session_id'],observation_id=t['observation_id'],prompt=r['prompt'],
              **identity_from_metadata(output['model_metadata']),observation_ns=int(t['observation_local_monotonic_ns']),
              response_ns=int(output['response_local_monotonic_ns']),actions=output['actions'],
              times_ns=[int(t['observation_local_monotonic_ns'])+(i+1)*100_000_000 for i in range(16)])
        else:self._rtc_plan=None
        if self.rtc_log:
            record=dict(request=r,reply=output['rtc'],actual_delay_steps=int((now-t['observation_local_monotonic_ns']+99_999_999)//100_000_000),
                        roundtrip_ms=output['latency_ms'],prediction_only=True,hardware_control_allowed=False)
            with Path(self.rtc_log).open('a') as stream:stream.write(json.dumps(record)+'\n')


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init();node=GuidedPolicyShadowNode()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
