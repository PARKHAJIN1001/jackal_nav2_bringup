import json, sqlite3, math
from pathlib import Path
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

P=Path('/tmp/jackal_nav2_validation_20260917_KXP5m6/rotation')
c=sqlite3.connect('file:'+str(P/'bag/bag_0.db3')+'?mode=ro',uri=True)
types={n:get_message(t) for n,t in c.execute('select name,type from topics')}
def msgs(topic):
    for data,received in c.execute('select m.data,m.timestamp from messages m join topics t on t.id=m.topic_id where t.name=? order by m.timestamp',(topic,)):
        yield deserialize_message(data,types[topic]),received/1e9
def stamp(h): return h.sec+h.nanosec*1e-9
def stats(x):
    a=np.asarray(x);a=a[np.isfinite(a)]
    return dict(n=len(a),min=float(a.min()),median=float(np.median(a)),p90=float(np.percentile(a,90)),max=float(a.max())) if len(a) else {'n':0}
def tfrow(tr):
    p=tr.transform.translation;q=tr.transform.rotation
    return [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
static=defaultdict(list);dynamic=defaultdict(list)
for msg,rec in msgs('/tf_static'):
    for tr in msg.transforms:
        row=tfrow(tr)
        if row not in static[tr.header.frame_id+' -> '+tr.child_frame_id]: static[tr.header.frame_id+' -> '+tr.child_frame_id].append(row)
for msg,rec in msgs('/tf'):
    for tr in msg.transforms: dynamic[(tr.header.frame_id,tr.child_frame_id)].append([stamp(tr.header.stamp),rec]+tfrow(tr))
print('STATIC',dict(static),flush=True)
mount=static['base_link -> livox_frame'][0]
R=Rotation.from_quat(mount[3:]).as_matrix();trans=np.array(mount[:3])
scans={};scan_rows=[]
for m,rec in msgs('/scan'):
    ranges=np.array(m.ranges)
    scans[round(stamp(m.header.stamp),5)]=(m,ranges)
    valid=ranges[np.isfinite(ranges)&(ranges>=.4)&(ranges<30)]
    scan_rows.append([stamp(m.header.stamp),len(valid)]+[float(np.mean(valid<v)) for v in (.6,1,1.5,2,3)]+[rec-stamp(m.header.stamp)])
sr=np.array(scan_rows)
report={'static_tf':dict(static),'scan_amcl_used_ranges':{'valid_beams':stats(sr[:,1]),**{f'fraction_below_{v}m':stats(sr[:,i+2]) for i,v in enumerate((.6,1,1.5,2,3))}}}
report['dynamic_tf']={}
for (parent,child),rows in dynamic.items():
    a=np.array(rows);dt=np.diff(a[:,0]);norm=np.linalg.norm(a[:,5:9],axis=1)
    report['dynamic_tf'][parent+' -> '+child]={'n':len(a),'backwards_stamps':int((dt<0).sum()),'repeated_stamps':int((dt==0).sum()),'max_stamp_gap':float(dt.max()) if len(dt) else 0,'receipt_minus_stamp':stats(a[:,1]-a[:,0]),'quaternion_norm':stats(norm)}
fast={round(stamp(m.header.stamp),5):m for m,_ in msgs('/aft_mapped_to_init')}
matches=[]
for row in dynamic[('odom','base_link')]:
    if round(row[0],5) in fast:
        m=fast[round(row[0],5)]
        p=m.pose.pose.position;q=m.pose.pose.orientation
        matches.append(np.max(np.abs(np.array(row[2:])-np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w]))))
report['fast_pose_tf_max_component_error']=stats(matches)
traj=np.load(P/'trajectories.npz');f=traj['fast'];rate=traj['rate'];start=f[0,0]
amcl=traj['amcl'];report['amcl_poses']=[{'time_rel':float(a[0]-start),'x':float(a[1]),'y':float(a[2]),'yaw_deg':float(np.degrees(a[4])),'var_x':float(a[7]),'var_y':float(a[8]),'var_yaw':float(a[9])} for a in amcl]
cloud_metrics=[];projection_matches=[];examples=[];z_near=[];xy_near=[]
def project(xyz,minimum,maximum,rmin=.2):
    r=np.linalg.norm(xyz[:,:2],axis=1);angle=np.arctan2(xyz[:,1],xyz[:,0])
    ok=np.isfinite(xyz).all(axis=1)&(xyz[:,2]>=minimum)&(xyz[:,2]<=maximum)&(r>=rmin)&(r<=30)
    xyz=xyz[ok];r=r[ok];angle=angle[ok]
    bins=np.floor((angle+np.pi)/(.5*np.pi/180)).astype(int)
    ranges=np.full(721,np.inf);np.minimum.at(ranges,bins,r)
    return ranges
for i,(m,rec) in enumerate(msgs('/livox/lidar_local')):
    if i%10: continue
    xyz=np.column_stack([np.ndarray((m.width*m.height,),dtype='<f4',buffer=m.data,offset=j,strides=(m.point_step,)) for j in (0,4,8)])
    xyz=xyz@R.T+trans
    t=stamp(m.header.stamp);r=np.linalg.norm(xyz[:,:2],axis=1)
    near=(r>=.4)&(r<1)&(xyz[:,2]>=.1)&(xyz[:,2]<=1.8)
    z_near.extend(xyz[near,2].tolist());xy_near.append(xyz[near,:2])
    current=project(xyz,.1,1.8)
    higher=project(xyz,.5,1.8)
    narrow=project(xyz,.6,1.2)
    matched=scans.get(round(t,5))
    if matched:
        actual=matched[1];n=min(len(actual),len(current));ok=np.isfinite(actual[:n])&np.isfinite(current[:n])
        projection_matches.extend(np.abs(actual[:n][ok]-current[:n][ok]).tolist())
    vals=[]
    for arr in (current,higher,narrow):
        good=arr[np.isfinite(arr)&(arr>=.4)]
        vals.extend([len(good),float(np.median(good)) if len(good) else 0,float(np.mean(good<1)) if len(good) else 0])
    cloud_metrics.append([t,*vals])
    target=(t-start<1) or (27<t-start<28) or (39<t-start<40) or (60<t-start<61)
    if target and (not examples or t-examples[-1][0]>1): examples.append((t,xyz.copy(),current,higher))
cm=np.array(cloud_metrics)
report['reproduced_scan_absolute_error_m']=stats(projection_matches)
report['near_returns_height_m']=stats(z_near)
report['height_filter_comparison']={name:{'beams':stats(cm[:,1+3*j]),'range_median':stats(cm[:,2+3*j]),'fraction_below_1m':stats(cm[:,3+3*j])} for j,name in enumerate(('z_0.1_1.8','z_0.5_1.8','z_0.6_1.2'))}
fig,ax=plt.subplots(2,max(1,len(examples)),figsize=(5*max(1,len(examples)),8),squeeze=False)
for j,(t,xyz,current,higher) in enumerate(examples):
    ok=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz[:,:2],axis=1)<6)&(xyz[:,2]>.1)&(xyz[:,2]<1.8)
    a=ax[0,j];p=a.scatter(xyz[ok,0],xyz[ok,1],c=xyz[ok,2],s=1,vmin=0,vmax=1.8,cmap='viridis');a.set_aspect('equal');a.set_xlim(-4,4);a.set_ylim(-4,4);a.grid();a.set_title(f't={t-start:.1f}s, cloud height (m)')
    a=ax[1,j];angles=-np.pi+np.arange(len(current))*np.pi/360
    for name,rr in [('current z=0.1..1.8',current),('trial z=0.5..1.8',higher)]:
        ok=np.isfinite(rr);a.scatter(rr[ok]*np.cos(angles[ok]),rr[ok]*np.sin(angles[ok]),s=3,label=name)
    a.set_xlim(-4,4);a.set_ylim(-4,4);a.set_aspect('equal');a.grid();a.legend(fontsize=8)
fig.tight_layout();fig.savefig(P/'scan_geometry.png',dpi=130)
(P/'geometry_summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='amcl_poses'},indent=2))
