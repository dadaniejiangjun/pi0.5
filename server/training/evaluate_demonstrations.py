"""Reusable offline prediction evaluation of a completed demonstration training run."""
import argparse,json,os
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--training-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gpu-index',type=int,default=0)
    p.add_argument('--samples-per-episode',type=int,default=5)
    a=p.parse_args()
    if a.gpu_index<0:raise ValueError('nonnegative GPU index required')
    r=json.loads(a.training_report.read_text())
    if r.get('status')!='COMPLETE':raise ValueError('training must be complete before evaluation')
    result=r['result']
    provenance=Path(result['training_provenance'])
    training=json.loads(provenance.read_text())
    os.environ.update(CUDA_VISIBLE_DEVICES=str(a.gpu_index),XLA_PYTHON_CLIENT_PREALLOCATE='false')
    from server.training.proxy_multi_episode_pilot import evaluate
    with a.output.open('x',encoding='utf-8') as f:
        json.dump({'status':'RUNNING'},f);f.flush()
        try:
            report=evaluate(Path(training['dataset_path']),Path(result['checkpoint']),provenance,min_episodes=training['dataset_episodes'],max_samples_per_episode=a.samples_per_episode,expected_dataset_sha256=training['dataset_sha256'])
        except Exception as e:
            f.seek(0);f.truncate();json.dump({'status':'FAILED','error_type':type(e).__name__,'error':str(e)},f,indent=2)
            raise
        f.seek(0);f.truncate();json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
