from types import SimpleNamespace
from pathlib import Path
import json
from laptop.diagnostics.bounded_process import run_bounded
from laptop.diagnostics.probe_cameras import probe
if __name__=='__main__':
 args=SimpleNamespace(max_index=4,frames=1,output_dir=Path('validation/camera_index_scan_20261007_r1'))
 result=run_bounded(probe,args=(args,),timeout_s=20)
 with Path('validation/camera_index_scan_20261007_r1.json').open('x',encoding='utf-8') as f: json.dump(result,f,indent=2)
 print(json.dumps(result,indent=2))
