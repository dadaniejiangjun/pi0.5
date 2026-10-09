"""Reusable loss summary, CSV and optional curve from OpenPI training logs."""
import argparse,csv,json,re,statistics
from pathlib import Path

def summarize(log_path):
 rows=[(int(s),float(v)) for s,v in re.findall(r'Step (\d+):[^\r\n]*loss=([0-9.eE+-]+)',Path(log_path).read_text(encoding='utf-8'))]
 if not rows or any(b[0]<=a[0] for a,b in zip(rows,rows[1:])):raise ValueError('nonempty monotonically increasing loss log required')
 return rows,dict(logged_steps=len(rows),last_step=rows[-1][0],last_loss=rows[-1][1],first100_mean=statistics.mean(v for _,v in rows[:100]),last100_mean=statistics.mean(v for _,v in rows[-100:]))

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--log',type=Path,required=True);p.add_argument('--output-prefix',type=Path,required=True);p.add_argument('--plot',action='store_true');a=p.parse_args()
 rows,summary=summarize(a.log);paths=[a.output_prefix.with_suffix(ext) for ext in ('.json','.csv')+ (('.png',) if a.plot else ())]
 if any(v.exists() for v in paths):raise FileExistsError('output artifacts already exist')
 if a.plot:
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
 a.output_prefix.parent.mkdir(parents=True,exist_ok=True)
 paths[0].write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
 moving=[statistics.mean(v for _,v in rows[max(0,i-99):i+1]) for i in range(len(rows))]
 with paths[1].open('x',newline='',encoding='utf-8') as f:
  w=csv.writer(f);w.writerow(['step','loss','rolling100_mean']);w.writerows((s,v,m) for (s,v),m in zip(rows,moving))
 if a.plot:
  fig,ax=plt.subplots(figsize=(9,4.5),layout='constrained')
  ax.plot([s for s,_ in rows],[v for _,v in rows],color='#92a5ba',alpha=.45,lw=.6,label='Per-step loss')
  ax.plot([s for s,_ in rows],moving,color='#1767a6',lw=1.6,label='100-step rolling mean')
  ax.set(xlabel='Optimizer step',ylabel='Training loss',title=f'Pi0.5 demonstration training: {len(rows):,} updates')
  ax.grid(alpha=.2);ax.legend(frameon=False);fig.savefig(paths[2],dpi=160);plt.close(fig)
 print(json.dumps(summary))
if __name__=='__main__':main()
