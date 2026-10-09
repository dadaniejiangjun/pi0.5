"""Serve a completed demonstration checkpoint and export its checked client lock."""
import argparse,json,os,sys,subprocess,hashlib
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--training-report',type=Path,required=True);p.add_argument('--identity-lock',type=Path,required=True)
 p.add_argument('--gpu-index',type=int,default=0);p.add_argument('--port',type=int,default=8020)
 p.add_argument('--background',action='store_true');a=p.parse_args()
 r=json.loads(a.training_report.read_text())
 if r.get('status')!='COMPLETE':raise ValueError('completed training required')
 if a.identity_lock.exists():raise FileExistsError(a.identity_lock)
 os.environ.update(CUDA_VISIBLE_DEVICES=str(a.gpu_index),XLA_PYTHON_CLIENT_PREALLOCATE='false',PYTHONUNBUFFERED='1')
 if a.background:
  with a.identity_lock.with_suffix('.service.log').open('x') as log:
   child=subprocess.Popen([sys.executable,'-B','-m','server.training.serve_trained_demonstrations',*[v for v in sys.argv[1:] if v!='--background']],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  print(json.dumps({'pid':child.pid,'port':a.port,'identity_lock':str(a.identity_lock)}));return
 from server.serve_derived_state_multi_shadow import inspect_artifacts,create_server
 result=r['result'];provenance=Path(result['training_provenance']);training=json.loads(provenance.read_text())
 artifacts=inspect_artifacts(Path(training['dataset_path']),Path(result['checkpoint']),provenance)
 lock=dict(schema='piper_derived_state_multi_shadow_identity_lock_v1',artifact_identity_schema='piper_derived_state_multi_shadow_identity_v1',config_schema='pi05_derived_state_multi_episode_shadow_v1',checkpoint=str(artifacts['checkpoint']),checkpoint_tree_sha256=artifacts['checkpoint_sha256'],dataset=training['repo_id'],dataset_sha256=training['dataset_sha256'],normalization_identity='train_only_stats:'+training['repo_id'],normalization_sha256=artifacts['normalization_sha256'],action_semantics=training['action_semantics'],action_target_kind=training['action_target_kind'],action_horizon=16,action_dimension=7,first_target_dt_s=0.1,hardware_control_allowed=False,prediction_only=True,openpi_commit=artifacts['openpi_commit'],training_provenance_sha256=hashlib.sha256(provenance.read_bytes()).hexdigest())
 server=create_server(artifacts,host='127.0.0.1',port=a.port,sampling_steps=10)
 with a.identity_lock.open('x') as f:json.dump(lock,f,indent=2)
 print('PREDICTION_ONLY_LISTEN=127.0.0.1:'+str(a.port),flush=True);server.serve_forever()
if __name__=='__main__':main()
