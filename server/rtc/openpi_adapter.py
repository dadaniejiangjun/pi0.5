"""Standalone adaptation of pinned OpenPI Pi0.sample_actions for Guided RTC.

The official checkout remains unchanged. Prefix attention/cache and base velocity
follow OpenPI 215abfb217dbac7d5f1273282331b9b1866c0479; guidance follows
HuggingFace RTC, with an explicit differentiable endpoint VJP.
"""
import jax
import jax.numpy as jnp
from .jax_guidance import guided_velocity


def sample_actions(model, rng, observation, *, noise=None, num_steps=10,
                   rtc_prefix=None, rtc_weights=None, max_guidance_weight=10.0):
    if rtc_prefix is None:
        return model.sample_actions(rng,observation,noise=noise,num_steps=num_steps)
    from openpi.models import model as model_api
    from openpi.models.pi0 import make_attn_mask
    import einops
    observation=model_api.preprocess_observation(None,observation,train=False)
    batch=observation.state.shape[0]
    if noise is None:noise=jax.random.normal(rng,(batch,model.action_horizon,model.action_dim))
    tokens,mask,ar=model.embed_prefix(observation)
    _,cache=model.PaliGemma.llm([tokens,None],mask=make_attn_mask(mask,ar),positions=jnp.cumsum(mask,axis=1)-1)
    dt=-1.0/num_steps
    def step(carry):
        x,t=carry
        def velocity(z):
            suffix,sm,sa,cond=model.embed_suffix(observation,z,jnp.broadcast_to(t,batch))
            full=jnp.concatenate([einops.repeat(mask,'b p -> b s p',s=suffix.shape[1]),make_attn_mask(sm,sa)],axis=-1)
            pos=jnp.sum(mask,axis=-1)[:,None]+jnp.cumsum(sm,axis=-1)-1
            (_,out),_=model.PaliGemma.llm([None,suffix],mask=full,positions=pos,kv_cache=cache,adarms_cond=[None,cond])
            return model.action_out_proj(out[:,-model.action_horizon:]).astype(z.dtype)
        v=jax.lax.cond(jnp.any(rtc_weights!=0)&(max_guidance_weight>0),
            lambda z:guided_velocity(velocity,z,t,rtc_prefix,rtc_weights,max_guidance_weight),velocity,x)
        return x+dt*v,t+dt
    out,_=jax.lax.while_loop(lambda carry:carry[1]>=-dt/2,step,(noise,1.0))
    return out


def frozen_sampler(model):
    """Freeze NNX state in the same manner as upstream module_jit."""
    import flax.nnx as nnx
    graph,state=nnx.split(model)
    @jax.jit
    def apply(state,rng,observation,prefix,weights,gain,noise):
        m=nnx.merge(graph,state)
        return sample_actions(m,rng,observation,rtc_prefix=prefix,rtc_weights=weights,
                              max_guidance_weight=gain,noise=noise,num_steps=10)
    return lambda rng,obs,prefix,weights,gain,noise=None:apply(state,rng,obs,prefix,weights,gain,noise)
