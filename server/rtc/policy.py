"""Dedicated prediction-only RTC wrapper around a loaded pinned OpenPI Policy."""
import time
import numpy as np
from .protocol import PROTOCOL,validate_request,prefix_weights


def identity_from_metadata(metadata):
    return dict(model_id=f"{metadata['config_schema']}:{metadata['dataset']}",
      checkpoint_id=f"{metadata['checkpoint']}#sha256:{metadata['checkpoint_tree_sha256']}")


class GuidedPolicy:
    def __init__(self,policy,metadata):
        if policy._is_pytorch_model:raise ValueError('RTC adapter supports pinned JAX backend only')
        from .openpi_adapter import frozen_sampler
        self.policy=policy;self.identity=identity_from_metadata(metadata)
        self.sample=frozen_sampler(policy._model)
    def infer(self,observation,*,noise=None):
        from openpi.models import model as model_api
        import jax
        import jax.numpy as jnp
        request=observation.get('_rtc')
        clean={k:v for k,v in observation.items() if k!='_rtc'}
        prefix=validate_request(request,self.identity) if request is not None else None
        if request is not None and request.get('sampling_seed') is not None:
            if noise is not None:raise ValueError('provide either explicit noise or seed')
            noise=np.random.default_rng(request['sampling_seed']).standard_normal((16,self.policy._model.action_dim)).astype(np.float32)
        if request is not None and request['prompt']!=clean.get('prompt'):
            raise ValueError('RTC request task differs from observation')
        start=time.perf_counter_ns()
        guided=prefix is not None and request['mode']=='guided'
        if not guided or request['max_guidance_weight']==0:
            result=self.policy.infer(clean,noise=noise)
        else:
            # Transform the canonical prefix through the EXACT checkpoint-bound
            # input pipeline (Normalize/Pad included), then remove the label.
            inputs=self.policy._input_transform(dict(clean,actions=prefix))
            normalized=np.asarray(inputs.pop('actions'),dtype=np.float32)
            model_dim=self.policy._model.action_dim
            count=len(prefix)
            if normalized.shape!=(count,model_dim) or not np.isfinite(normalized).all():
                raise ValueError('RTC normalized prefix shape/values invalid')
            padded=np.zeros((1,16,model_dim),dtype=np.float32);padded[0,:count]=normalized
            weights=np.zeros((1,16,model_dim),dtype=np.float32)
            weights[0,:,:7]=prefix_weights(request['inference_delay'],count,schedule=request['schedule'])[:,None]
            inputs=jax.tree.map(lambda x:jnp.asarray(x)[None,...],inputs)
            self.policy._rng,rng=jax.random.split(self.policy._rng)
            n=None if noise is None else jnp.asarray(noise).reshape(1,16,model_dim)
            actions=self.sample(rng,model_api.Observation.from_dict(inputs),jnp.asarray(padded),jnp.asarray(weights),request['max_guidance_weight'],n)
            # np.asarray synchronizes computation; timing includes the VJP.
            result=self.policy._output_transform(dict(state=np.asarray(inputs['state'][0]),actions=np.asarray(actions[0])))
        if np.asarray(result['actions']).shape!=(16,7) or not np.isfinite(result['actions']).all():
            raise ValueError('RTC output violates finite 16x7 proxy contract')
        result['rtc']=dict(protocol=PROTOCOL,mode='guided' if guided else 'plain',
           request_id=request['request_id'] if request else None,
           prefix_length=len(prefix) if prefix is not None else 0,
           prefix_reason=request['prefix_reason'] if request else 'plain_prewarm',
           inference_delay=request['inference_delay'] if request else 0,
           elapsed_ms=(time.perf_counter_ns()-start)/1e6,implementation='jax_full_endpoint_vjp_v1')
        return result
