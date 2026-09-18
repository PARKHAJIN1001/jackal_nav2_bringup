import json,math,sys
from pathlib import Path
from collections import defaultdict
import numpy as np
root=Path(__file__).resolve().parent
# Stream the durable log; retain only TF/pose/health needed for analysis.
events=[json.loads(l) for l in (root/'events.jsonl').read_text().splitlines()]
start=next(e['wall'] for e in events if e['event']=='manual_rotation_window_start')
ends=[e['wall'] for e in events if e['event']=='manual_rotation_window_end']
end=ends[-1] if ends else float('inf')
rows=[]
with (root/'samples.jsonl').open() as f:
 for line in f:
  try:r=json.loads(line)
  except ValueError:continue
  if r['wall']>=start-30 and r['kind'] in ['tf','composed','pose','health']:
   if r['kind']=='tf' and r['topic']!='/tf':continue
   if r['kind']=='pose' and r['topic']!='/amcl_pose':continue
   rows.append(r)
last=rows[-1]['wall'];end=min(end,last)
phases=[('before',start-30,start),('rotation_window',start,end)]
if last>end+5:phases.append(('after_response_with_manual_translation',end,min(end+30,last)))
rechecks=[e['wall'] for e in events if e['event']=='stationary_recheck_confirmed']
if rechecks and last>rechecks[-1]+5:phases.append(('stationary_recheck',rechecks[-1],min(rechecks[-1]+30,last)))
result={'start':start,'end':end,'record_last':last,'phases':{}}
for name,a,b in phases:
 phase={'duration_sec':b-a,'poses':{},'inputs':{}};groups=defaultdict(list);health=[]
 for r in rows:
  if not a<=r['wall']<b:continue
  if r['kind']=='tf' and (r['parent'],r['child']) in [('odom','base_link'),('map','odom')]:groups[r['parent']+' -> '+r['child']].append(r)
  elif r['kind']=='composed':groups['map -> base_link (1Hz)'].append(r)
  elif r['kind']=='pose':groups['amcl_pose'].append(r)
  elif r['kind']=='health':health.append(r)
 for k,v in groups.items():
  z=np.array([r['xyz'][2] for r in v]);angles=np.array([r['rpy'] for r in v]); yaw=np.unwrap(angles[:,2]);xy=np.array([r['xyz'][:2] for r in v])
  summary={'n':len(v),'z_min_mm':z.min()*1000,'z_max_mm':z.max()*1000,'z_range_mm':np.ptp(z)*1000,'z_std_mm':z.std()*1000,'z_p05_p95_mm':(np.percentile(z,[5,95])*1000).tolist(),'z_first_last_mm':(z[[0,-1]]*1000).tolist(),'roll_range_deg':np.ptp(angles[:,0])*180/math.pi,'pitch_range_deg':np.ptp(angles[:,1])*180/math.pi,'yaw_range_deg':np.ptp(yaw)*180/math.pi,'xy_max_from_first_m':float(np.max(np.linalg.norm(xy-xy[0],axis=1)))}
  # Linear correlation is descriptive, not causal; omit constant axes.
  for i,axis in [(0,'roll'),(1,'pitch')]:
   if z.std()>1e-9 and angles[:,i].std()>1e-9:summary['z_'+axis+'_correlation']=float(np.corrcoef(z,angles[:,i])[0,1])
  phase['poses'][k]=summary
 for r in health:
  for topic,s in r['topics'].items():
   t=phase['inputs'].setdefault(topic,{'count':0,'max_receipt_gap_sec':0.,'max_age_sec':-1e9,'regressions':0})
   t['count']+=s['n'];t['max_receipt_gap_sec']=max(t['max_receipt_gap_sec'],s['max_receipt_gap']);t['max_age_sec']=max(t['max_age_sec'],s['max_age']);t['regressions']+=s['regressions']
 for topic,s in phase['inputs'].items():s['approx_hz']=s['count']/(b-a)
 if len(health)>1:
  phase['kernel_delta']={f:{k:v-health[0]['kernel'][f].get(k,v) for k,v in d.items()} for f,d in health[-1]['kernel'].items()}
 phase['notes']=['Receive gaps are observer-local and include scheduling; AMCL pose intervals are movement-gated, not a sensor-rate failure test.','Composed map/base samples are 1 Hz; their extrema can miss faster peaks.','Input statistics use overlapping 1-second health bins.']
 result['phases'][name]=phase
(root/'rotation_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
for name,p in result['phases'].items():
 print(name,round(p['duration_sec'],1),'seconds')
 for k,v in p['poses'].items():print(k, {x:round(v[x],3) for x in ['z_range_mm','z_std_mm','roll_range_deg','pitch_range_deg','yaw_range_deg','xy_max_from_first_m']})
