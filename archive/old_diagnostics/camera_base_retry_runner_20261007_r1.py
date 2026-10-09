import cv2,time,json
from pathlib import Path
from datetime import datetime,timezone
if __name__=='__main__':
 results=[]
 for label,backend in [('DSHOW',cv2.CAP_DSHOW),('MSMF',cv2.CAP_MSMF)]:
  cap=cv2.VideoCapture(0,backend);start=time.perf_counter();times=[];frame=None
  try:
   while cap.isOpened() and time.perf_counter()-start<3:
    ok,img=cap.read()
    if ok:times.append(time.perf_counter_ns());frame=img
   item={'backend':label,'opened':cap.isOpened(),'frames':len(times),'observed_fps':(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else None,'shape':list(frame.shape) if frame is not None else None}
   if frame is not None:
    path=Path('validation/camera_base_retry_20261007_r1.jpg');cv2.imwrite(str(path),frame);item['preview']=str(path);item['mean_pixel']=float(frame.mean())
   results.append(item)
  finally:cap.release()
  if frame is not None:break
 report={'test_utc':datetime.now(timezone.utc).isoformat(),'index':0,'attempts':results,'robot_accessed':False}
 with Path('validation/camera_base_retry_20261007_r1.json').open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
 print(json.dumps(report))
