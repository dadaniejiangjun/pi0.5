"""Independent loopback Guided RTC service; original plain service is untouched."""
import argparse
from pathlib import Path
from server.serve_derived_state_multi_shadow import inspect_artifacts,build_server_metadata,_sha256
from server.rtc.protocol import PROTOCOL


def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset-path',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--training-provenance',type=Path,required=True)
    p.add_argument('--port',type=int,default=8022);a=p.parse_args()
    if not 1<=a.port<=65535:raise ValueError('invalid RTC port')
    artifacts=inspect_artifacts(a.dataset_path,a.checkpoint,a.training_provenance)
    from server.training.proxy_multi_episode_pilot import build_config
    from openpi.policies.policy_config import create_trained_policy
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer
    from server.rtc.policy import GuidedPolicy
    import os
    os.environ['HF_LEROBOT_HOME']=artifacts['audit']['hf_lerobot_home']
    config=build_config(repo_id=artifacts['audit']['repo_id'],base_params_path=artifacts['base_params_path'],
                       steps=artifacts['training']['steps'],exp_name=artifacts['training']['experiment_name'])
    policy=create_trained_policy(config,artifacts['checkpoint'],sample_kwargs={'num_steps':10})
    metadata=build_server_metadata(artifacts,config_sha256=_sha256(a.training_provenance),sampling_steps=10)
    metadata.update(rtc_protocol=PROTOCOL,rtc_modes=['plain','guided'],rtc_max_execution_horizon=8,
      rtc_reference_commit='146d8474932c8b566717eb2738ba459337ab71c4',rtc_implementation='jax_full_endpoint_vjp_v1')
    wrapper=GuidedPolicy(policy,metadata)
    print(f'RTC_PREDICTION_ONLY_LISTEN=127.0.0.1:{a.port}',flush=True)
    WebsocketPolicyServer(wrapper,host='127.0.0.1',port=a.port,metadata=metadata).serve_forever()
if __name__=='__main__':main()
