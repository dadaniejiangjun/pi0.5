"""Reusable live prediction entrypoint using the commissioned dual-camera resolver."""
import argparse,json
from pathlib import Path
from laptop.camera.dual_camera import inventory
from laptop.recorder.record_demonstrations import camera_identity
from laptop.client.live_shadow import _worker
from laptop.client.policy_client import load_artifact_identity_lock
from laptop.diagnostics.bounded_process import run_bounded
from laptop.robot_client.bounded_policy_target import limit_target

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--identity-lock',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=8020)
    p.add_argument('--duration',type=float,default=15)
    p.add_argument('--read-only',action='store_true')
    a=p.parse_args()
    if not a.read_only:raise ValueError('explicit --read-only required')
    if not 0<a.duration<=20:raise ValueError('duration must be within 20 seconds')
    if a.output.exists():raise FileExistsError(a.output)
    lock=load_artifact_identity_lock(a.identity_lock)
    identity=camera_identity(inventory())
    roles=identity['roles']
    result=run_bounded(_worker,args=(a.duration,a.port,roles['base']['opencv_index'],roles['wrist']['opencv_index'],lock,identity),timeout_s=a.duration+25)
    report=result['result'] if result['status']=='COMPLETE' else result
    for item in report.get('predictions', []):
        item['bounded_target_preview'] = limit_target(item['state'], item['first_predicted_proxy'])
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
