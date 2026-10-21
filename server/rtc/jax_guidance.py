"""JAX Guided RTC math, adapted from HuggingFace LeRobot's Apache-2.0 RTC.

Reference commit: 146d8474932c8b566717eb2738ba459337ab71c4.
Uses explicit endpoint VJP, with time=1 noise and time=0 clean (OpenPI).
"""
import jax
import jax.numpy as jnp


def guidance_gain(time, maximum=10.0):
    numerator=time*time+(1-time)*(1-time)
    denominator=jnp.maximum(time*(1-time),1e-8)
    return jnp.minimum(maximum,numerator/denominator)


def guided_velocity(velocity, x, time, prefix, weights, maximum=10.0):
    def endpoint(z):
        v=velocity(z)
        return z-time*v,v
    endpoint_value,pullback,v=jax.vjp(endpoint,x,has_aux=True)
    error=jax.lax.stop_gradient((prefix-endpoint_value)*weights)
    correction=pullback(error)[0]
    return v-guidance_gain(time,maximum)*correction


def sample_flow(velocity, noise, prefix=None, weights=None, maximum=10.0, steps=10):
    """Small flow harness for analytic tests; actual Pi0 adapter shares the step."""
    dt=-1.0/steps
    def step(i,x):
        time=1.0+i*dt
        v=velocity(x,time) if prefix is None else guided_velocity(lambda z:velocity(z,time),x,time,prefix,weights,maximum)
        return x+dt*v
    return jax.lax.fori_loop(0,steps,step,noise)
