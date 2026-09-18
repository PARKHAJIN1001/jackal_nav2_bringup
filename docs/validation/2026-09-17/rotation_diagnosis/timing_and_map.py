import sqlite3,json,math,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path('/tmp/jackal_nav2_validation_20260917_KXP5m6/rotation')
c=sqlite3.connect('file:'+str(P/'bag/bag_0.db3')+'?mode=ro',uri=True)
types={n:get_message(t) for n,t in c.execute('select name,type from topics')}
def msgs(topic):
    for data,rec in c.execute('select m.data,m.timestamp from messages m join topics t on t.id=m.topic_id where t.name=? order by m.timestamp',(topic,)):
        yield deserialize_message(data,types[topic]),rec/1e9
def stamp(h): return h.sec+h.nanosec/1e9
def stats(a):
    a=np.asarray(a)
    return {'n':len(a),'median':float(np.median(a)),'p90':float(np.percentile(a,90)),'max':float(a.max())} if len(a) else {'n':0}
d=np.load(P/'trajectories.npz');fast=d['fast'];rate=d['rate'];amcl=d['amcl'];t0=fast[0,0]
timing={}
for topic in ('/tf','/amcl_pose','/scan','/aft_mapped_to_init','/odom','/livox/imu'):
    for m,r in msgs(topic):
        if topic=='/tf':
            for tr in m.transforms:
                key=tr.header.frame_id+' -> '+tr.child_frame_id
                s=stamp(tr.header.stamp)-(1 if tr.child_frame_id=='odom' else 0)
                timing.setdefault(key,[]).append([s-t0,r-t0,r-s])
        else:
            s=stamp(m.header.stamp)
            timing.setdefault(topic,[]).append([s-t0,r-t0,r-s])
report={}
for k,v in timing.items():
    a=np.array(v)
    report[k]={f'{left}..{right}s':stats(a[(a[:,0]>=left)&(a[:,0]<right),2]) for left,right in ((0,25),(25,34),(34,39),(39,44),(44,60),(60,75))}
fig,axs=plt.subplots(2,1,figsize=(12,7),sharex=True)
for k in ('odom -> base_link','map -> odom','/amcl_pose','/scan','/aft_mapped_to_init'):
    a=np.array(timing[k]);ok=a[:,0]>=0
    axs[0].plot(a[ok,0],a[ok,2],'.',ms=2,label=k)
axs[0].legend();axs[0].set_ylabel('Receipt - measurement (s)')
axs[1].plot(fast[:,0]-t0,rate);axs[1].set_ylabel('FAST yaw rate (rad/s)');axs[1].set_xlabel('Seconds')
for ax in axs: ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(P/'timing.png',dpi=130)
m,_=next(msgs('/map'));res=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y
grid=np.array(m.data).reshape(m.info.height,m.info.width)
report['map']={'frame':m.header.frame_id,'resolution':res,'width':m.info.width,'height':m.info.height,'origin':[ox,oy], 'origin_q':[m.info.origin.orientation.x,m.info.origin.orientation.y,m.info.origin.orientation.z,m.info.origin.orientation.w]}
distance=distance_transform_edt(grid!=100)*res
sample_scans=[]
for m,r in msgs('/scan'):
    t=stamp(m.header.stamp)-t0
    if any(abs(t-target)<.04 for target in [27.3,30.2,33.2,39.8,40.13,43.67]):
        rr=np.array(m.ranges);angles=m.angle_min+np.arange(len(rr))*m.angle_increment
        # Mirror AMCL likelihood field stride, min range and max range rejection.
        step=max(1,(len(rr)-1)//(120-1));ix=np.arange(0,len(rr),step)
        ok=np.isfinite(rr[ix])&(rr[ix]>.4)&(rr[ix]<30);ix=ix[ok]
        xy=np.column_stack((rr[ix]*np.cos(angles[ix]),rr[ix]*np.sin(angles[ix])))
        sample_scans.append((t,xy))
fig,axs=plt.subplots(1,len(sample_scans),figsize=(5*len(sample_scans),6),squeeze=False)
score=[]
for ax,(t,xy) in zip(axs[0],sample_scans):
    pose=amcl[np.argmin(abs(amcl[:,0]-t0-t))];a=pose[4];rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    xy=xy@rot.T+pose[1:3]
    idx=np.floor((xy-[ox,oy])/res).astype(int)
    ok=(idx[:,0]>=0)&(idx[:,0]<grid.shape[1])&(idx[:,1]>=0)&(idx[:,1]<grid.shape[0])
    dd=np.full(len(idx),np.inf);dd[ok]=distance[idx[ok,1],idx[ok,0]]
    score.append({'t_rel':t,'beams':len(xy),'fraction_within_0.15m_map_wall':float(np.mean(dd<.15)),'fraction_within_0.3m_map_wall':float(np.mean(dd<.3)),'median_distance_m':float(np.median(dd))})
    ax.imshow(grid,origin='lower',extent=(ox,ox+grid.shape[1]*res,oy,oy+grid.shape[0]*res),cmap='gray_r',vmin=-1,vmax=100)
    ax.scatter(xy[:,0],xy[:,1],c=np.minimum(dd,1),vmin=0,vmax=1,cmap='coolwarm',s=10)
    ax.plot(pose[1],pose[2],'g^');ax.set_xlim(-8,8);ax.set_ylim(-7,4);ax.set_title(f't={t:.2f}s AMCL pose + scan');ax.set_aspect('equal');ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(P/'map_alignment.png',dpi=110)
report['scan_map_at_amcl_pose']=score
# AMCL updates and no-motion period are distinct from a frozen scan publisher.
report['amcl_last_pose_rel_s']=float(amcl[-1,0]-t0)
report['fast_net_yaw_deg']=float(np.degrees(fast[-1,4]-fast[0,4]))
# Rotation odometry delta split: false translation and motion noise in current alpha settings.
ar=[]
for before,after in zip(amcl[:-1],amcl[1:]):
    a=before[0];b=after[0]
    x1,y1,yaw1=[np.interp(a,fast[:,0],fast[:,j]) for j in (1,2,4)]
    x2,y2,yaw2=[np.interp(b,fast[:,0],fast[:,j]) for j in (1,2,4)]
    dt=np.hypot(x2-x1,y2-y1);dy=math.atan2(math.sin(yaw2-yaw1),math.cos(yaw2-yaw1))
    dr1=math.atan2(math.sin(math.atan2(y2-y1,x2-x1)-yaw1),math.cos(math.atan2(y2-y1,x2-x1)-yaw1)) if dt>=.01 else 0
    dr2=math.atan2(math.sin(dy-dr1),math.cos(dy-dr1))
    def rev_min(x):return min(abs(x),abs(math.atan2(math.sin(x-math.pi),math.cos(x-math.pi))))
    n1=rev_min(dr1);n2=rev_min(dr2)
    sigma_t=math.sqrt(.2*dt*dt+.2*n1*n1+.2*n2*n2)
    sigma_y=math.sqrt(.2*(n1*n1+n2*n2)+.4*dt*dt)
    ar.append([b-t0,float(dt),float(abs(dy)),float(abs(dr1)),float(sigma_t),float(sigma_y)])
arr=np.array(ar)
report['amcl_motion_update_noise_current_alpha']={'translation_delta_m':stats(arr[:,1]),'rot1_abs_deg':stats(np.degrees(arr[:,3])),'translation_noise_std_m':stats(arr[:,4]),'yaw_noise_std_deg':stats(np.degrees(arr[:,5])),'updates_ge_1cm':int((arr[:,1]>=.01).sum()),'rows_t_dt_absdy_absrot1_sigma_t_sigma_y':ar}
(P/'timing_map_summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='amcl_motion_update_noise_current_alpha'},indent=2))
