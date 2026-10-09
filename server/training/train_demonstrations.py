"""Reusable foreground/background training for audited PiPER demonstrations."""
from __future__ import annotations
import argparse,json,os,subprocess,sys
from datetime import datetime,timezone
from pathlib import Path


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset-path',type=Path,required=True)
    p.add_argument('--base-params',type=Path,required=True)
    p.add_argument('--steps',type=int,default=5000)
    p.add_argument('--checkpoint-interval',type=int)
    p.add_argument('--keep-period',type=int)
    p.add_argument('--min-episodes',type=int,default=3)
    p.add_argument('--experiment',required=True)
    p.add_argument('--gpu-index',type=int,default=2)
    p.add_argument('--memory-fraction',type=float,default=0.95)
    p.add_argument('--expected-dataset-sha256',required=True)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--background',action='store_true')
    p.add_argument('--train',action='store_true')
    return p


def run(args):
    if not args.train:raise PermissionError('explicit --train required')
    if not 1<=args.steps<=50000 or args.gpu_index<0 or not 0<args.memory_fraction<=0.95:raise ValueError('steps 1..50000, nonnegative GPU, memory fraction within (0,.95]')
    args.checkpoint_interval=args.steps if args.checkpoint_interval is None else args.checkpoint_interval
    if not 1<=args.checkpoint_interval<=args.steps:raise ValueError('checkpoint interval must be within training steps')
    if args.keep_period is not None and args.keep_period<1:raise ValueError('keep period must be positive')
    if args.min_episodes<1:raise ValueError('positive episode count required')
    args.report.parent.mkdir(parents=True,exist_ok=True)
    if args.report.exists():raise FileExistsError(args.report)
    env=os.environ.copy();env.update(CUDA_VISIBLE_DEVICES=str(args.gpu_index),XLA_PYTHON_CLIENT_PREALLOCATE='true',XLA_PYTHON_CLIENT_MEM_FRACTION=str(args.memory_fraction),PYTHONUNBUFFERED='1',WANDB_MODE='disabled')
    if args.background:
        log=args.report.with_suffix('.log');launch=args.report.with_suffix('.launch.json')
        # Reserve both artifacts before launching a worker. Never overwrite a run.
        with launch.open('x',encoding='utf-8') as launch_file,log.open('x',encoding='utf-8') as stream:
            argv=[v for v in sys.argv[1:] if v!='--background']
            child=subprocess.Popen([sys.executable,'-B','-m','server.training.train_demonstrations',*argv],cwd=Path.cwd(),env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            record=dict(status='STARTED',pid=child.pid,steps=args.steps,checkpoint_interval=args.checkpoint_interval,keep_period=args.keep_period,report=str(args.report),log=str(log),automatic_followup_training=False)
            json.dump(record,launch_file,indent=2)
        return record
    os.environ.update({k:env[k] for k in ('CUDA_VISIBLE_DEVICES','XLA_PYTHON_CLIENT_PREALLOCATE','XLA_PYTHON_CLIENT_MEM_FRACTION','PYTHONUNBUFFERED','WANDB_MODE')})
    report=dict(status='RUNNING',steps=args.steps,checkpoint_interval=args.checkpoint_interval,keep_period=args.keep_period,experiment=args.experiment,gpu_index=args.gpu_index,started_utc=datetime.now(timezone.utc).isoformat(),hardware_control_allowed=False)
    with args.report.open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
    try:
        from server.training.proxy_multi_episode_pilot import execute
        report['result']=execute(args.dataset_path,phase='train',base_params_path=args.base_params,min_episodes=args.min_episodes,steps=args.steps,exp_name=args.experiment,expected_dataset_sha256=args.expected_dataset_sha256,allow_derived_state_pilot=True,allow_long_training=True,save_interval=args.checkpoint_interval,keep_period=args.keep_period)
        report['status']='COMPLETE'
    except Exception as e:
        report.update(status='FAILED',error_type=type(e).__name__,error=str(e))
        raise
    finally:
        report['finished_utc']=datetime.now(timezone.utc).isoformat()
        temporary=args.report.with_suffix('.tmp');temporary.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');temporary.replace(args.report)
    return report


def main():
    print(json.dumps(run(parser().parse_args()),indent=2))
if __name__=='__main__':main()
