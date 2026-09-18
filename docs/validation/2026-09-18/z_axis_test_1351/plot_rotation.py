import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
r=json.loads((p/'rotation_analysis.json').read_text()); t0=r['start'];end=r['end'];series={k:[] for k in ['odom','map','composed']};health=[]
with (p/'samples.jsonl').open() as f:
 for l in f:
  try:x=json.loads(l)
  except ValueError:continue
  if x['wall']<t0-30 or x['wall']>end+30:continue
  k=None
  if x['kind']=='tf' and x['topic']=='/tf':
   if (x['parent'],x['child'])==('odom','base_link'):k='odom'
   elif (x['parent'],x['child'])==('map','odom'):k='map'
  elif x['kind']=='composed':k='composed'
  elif x['kind']=='health':health.append(x)
  if k:series[k].append(x)
fig,axs=plt.subplots(4,1,figsize=(11,9),sharex=True)
for k,label in [('odom','odom -> base_link (FAST)'),('map','map -> odom'),('composed','map -> base_link (1 Hz sampled)')]:
 v=series[k];t=[x['wall']-t0 for x in v]
 axs[0].plot(t,[x['xyz'][2]*1000 for x in v],label=label,lw=.8)
v=series['odom'];t=[x['wall']-t0 for x in v];angles=np.array([x['rpy'] for x in v])
axs[1].plot(t,np.rad2deg(angles[:,0]),label='FAST roll',lw=.8);axs[1].plot(t,np.rad2deg(angles[:,1]),label='FAST pitch',lw=.8)
axs[2].plot(t,np.rad2deg(np.unwrap(angles[:,2])),label='FAST yaw',lw=.8)
for topic,label in [('/livox/lidar','raw LiDAR'),('/aft_mapped_to_init','FAST odometry')]:
 h=[x for x in health if topic in x['topics']];axs[3].plot([x['wall']-t0 for x in h],[x['topics'][topic]['max_receipt_gap'] for x in h],label=label,lw=.8)
for ax,ylabel in zip(axs,['z (mm)','roll / pitch (deg)','yaw (deg)','max receipt gap (s)']):
 ax.set_ylabel(ylabel);ax.grid(alpha=.25);ax.legend(loc='upper left',fontsize=8);ax.axvline(0,color='gray',ls='--');ax.axvline(end-t0,color='gray',ls='--');ax.axvspan(end-t0,end-t0+30,color='orange',alpha=.10)
axs[0].set_title('Manual spot-turn observation: 128 MiB, Nav2 + perception\nDashed: instruction / completion reply; orange: subsequent ~0.5 m translation (not stationary)')
axs[-1].set_xlabel('Seconds from rotation instruction (includes operator response time)')
fig.tight_layout();fig.savefig(p/'rotation_tf.png',dpi=150)
