import json, math, sqlite3
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import median_filter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path('/tmp/jackal_nav2_validation_20260917_KXP5m6/rotation')
c=sqlite3.connect('file:'+str(P/'bag/bag_0.db3')+'?mode=ro',uri=True)
types={name:get_message(typ) for name,typ in c.execute('select name,type from topics')}
def stamp(s): return s.sec+s.nanosec*1e-9
def orientation(q): return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')
def pose_row(m,pose):
 r,p,y=orientation(pose.orientation)
 return [stamp(m.header.stamp),pose.position.x,pose.position.y,pose.position.z,y,r,p]
streams={}
tfs={}
scans=[]
initials=[]
for topic,data,received in c.execute('select t.name,m.data,m.timestamp from messages m join topics t on t.id=m.topic_id where t.name != ? order by m.timestamp',('/livox/lidar_local',)):
 m=deserialize_message(data,types[topic])
 if topic in ('/aft_mapped_to_init','/odom','/j100_0519/platform/odom'):
  streams.setdefault(topic,[]).append(pose_row(m,m.pose.pose)+[m.twist.twist.linear.x,m.twist.twist.angular.z,received*1e-9-stamp(m.header.stamp)])
 elif topic=='/amcl_pose':
  streams.setdefault(topic,[]).append(pose_row(m,m.pose.pose)+[m.pose.covariance[i] for i in (0,7,35)])
 elif topic in ('/livox/imu','/j100_0519/sensors/imu_0/data'):
  streams.setdefault(topic,[]).append([stamp(m.header.stamp),m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
 elif topic=='/tf':
  for tr in m.transforms:
   if tr.child_frame_id=='odom':
    r,p,y=orientation(tr.transform.rotation)
    tfs.setdefault('map_odom',[]).append([stamp(tr.header.stamp),tr.transform.translation.x,tr.transform.translation.y,y])
 elif topic=='/scan':
  a=np.asarray(m.ranges); valid=a[np.isfinite(a)&(a>=m.range_min)&(a<m.range_max)]
  scans.append([stamp(m.header.stamp),received*1e-9-stamp(m.header.stamp),len(valid),np.median(valid) if len(valid) else 0,np.percentile(valid,90) if len(valid) else 0])
 elif topic=='/initialpose': initials.append(pose_row(m,m.pose.pose))
for k,v in streams.items():
 a=np.array(v); a=a[np.argsort(a[:,0])]; a=a[np.r_[True,np.diff(a[:,0])>0]]
 if a.shape[1]>=7: a[:,4]=np.unwrap(a[:,4])
 streams[k]=a
fast=streams['/aft_mapped_to_init']; ft=fast[:,0]; fy=fast[:,4]
rate=median_filter(np.gradient(fy,ft),size=7)
rot=np.abs(rate)>0.15
station=np.abs(rate)<0.03
start=ft[0]
def stats(x):
 a=np.asarray(x); a=a[np.isfinite(a)]
 return {'n':len(a),'min':float(a.min()),'median':float(np.median(a)),'p90':float(np.percentile(a,90)),'max':float(a.max())} if len(a) else {'n':0}
def compare_rate(topic,gyro):
 a=streams[topic]
 if gyro:
  az=(-0.5*a[:,1]+math.sqrt(3)/2*a[:,3]) if topic=='/livox/imu' else a[:,3]
 else: az=median_filter(np.gradient(a[:,4],a[:,0]),size=7)
 matched=np.interp(ft,a[:,0],az)
 bias=float(np.median(matched[station])) if station.any() else 0
 corrected=matched-bias
 return {'stationary_bias_rad_sec':bias,'rotation_correlation':float(np.corrcoef(rate[rot],corrected[rot])[0,1]) if rot.sum()>3 else None,'rotation_rate_difference_abs_rad_sec':stats(np.abs(corrected[rot]-rate[rot])),'integrated_yaw_rad':float(np.trapz(corrected,ft))},corrected
report={'duration_sec':float(ft[-1]-ft[0]),'rotation_samples':int(rot.sum()),'stationary_samples':int(station.sum()),'fast_yaw_rate_abs_rad_sec':stats(np.abs(rate[rot])),'fast_roll_deg':stats(np.degrees(fast[:,5])),'fast_pitch_deg':stats(np.degrees(fast[:,6])),'fast_z_m':stats(fast[:,3]),'fast_xy_extent_m':[float(np.ptp(fast[:,i])) for i in (1,2)],'initialposes':initials,'streams':{k:{'count':len(a),'max_stamp_gap_sec':float(np.diff(a[:,0]).max())} for k,a in streams.items()}}
comparisons={}
for topic,gyro in (('/livox/imu',True),('/j100_0519/sensors/imu_0/data',True),('/j100_0519/platform/odom',False)):
 if topic in streams: report[topic],comparisons[topic]=compare_rate(topic,gyro)
amcl=streams.get('/amcl_pose',np.empty((0,10)))
report['amcl_latched_samples_outside_recording']=int(((amcl[:,0]<ft[0]) | (amcl[:,0]>ft[-1])).sum())
report['amcl_outside_recording_rows']=amcl[(amcl[:,0]<ft[0]) | (amcl[:,0]>ft[-1])].tolist()
amcl=amcl[(amcl[:,0]>=ft[0]) & (amcl[:,0]<=ft[-1])]
if len(amcl): amcl[:,4]=np.unwrap(amcl[:,4])
report['streams']['/amcl_pose']={'count':len(amcl),'max_stamp_gap_sec':float(np.diff(amcl[:,0]).max()) if len(amcl)>1 else 0,'scope':'Only measurement stamps within FAST odometry recording window; latched history excluded.'}
if len(amcl)>1:
 interp_fast=np.interp(amcl[:,0],ft,fy)
 correction=np.unwrap(amcl[:,4]-interp_fast)
 report['amcl']={'yaw_variance_rad2':stats(amcl[:,9]),'xy_variance_m2':stats(np.maximum(amcl[:,7],amcl[:,8])),'yaw_correction_range_deg':float(np.degrees(np.ptp(correction))),'max_yaw_correction_step_deg':float(np.degrees(np.abs(np.diff(correction)).max())),'max_xy_pose_step_m':float(np.linalg.norm(np.diff(amcl[:,1:3],axis=0),axis=1).max())}
scan=np.array(scans)
report['scan']={'count':len(scan),'measurement_age_sec':stats(scan[:,1]),'finite_beams':stats(scan[:,2]),'range_median_m':stats(scan[:,3]),'range_p90_m':stats(scan[:,4])}
cloud=[]
for i,(data,received) in enumerate(c.execute('select m.data,m.timestamp from messages m join topics t on t.id=m.topic_id where t.name=? order by m.timestamp',('/livox/lidar_local',))):
 if i%5: continue
 m=deserialize_message(data,types['/livox/lidar_local'])
 f=next(f for f in m.fields if f.name=='timestamp')
 pts=np.ndarray((m.width*m.height,),dtype='<f8',buffer=m.data,offset=f.offset,strides=(m.point_step,))
 span=(float(pts.max())-float(pts.min()))*1e-9
 angular=abs(float(np.interp(stamp(m.header.stamp),ft,rate)))
 cloud.append([stamp(m.header.stamp),span,angular,angular*span])
ca=np.array(cloud); rotating=ca[:,2]>.15
report['raw_cloud']={'span_sec':stats(ca[:,1]),'rotation_during_cloud_deg':stats(np.degrees(ca[rotating,3])),'lateral_error_at_10m_end_vs_start_m':stats(10*np.sin(ca[rotating,3]))}
(P/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
np.savez(P/'trajectories.npz',fast=fast,rate=rate,amcl=amcl,scan=scan)
fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True)
ax[0].plot(ft-start,np.degrees(fy-fy[0]),label='FAST-LIVO relative yaw')
if len(amcl)>1:
 origin=np.interp(amcl[0,0],ft,fy)-fy[0]
 ax[0].plot(amcl[:,0]-start,np.degrees(amcl[:,4]-amcl[0,4]+origin),'.-',label='AMCL relative yaw')
ax[0].set_ylabel('Yaw (deg)'); ax[0].legend()
ax[1].plot(ft-start,rate,label='FAST-LIVO yaw rate')
for name,ar in comparisons.items(): ax[1].plot(ft-start,ar,alpha=.65,label=name)
ax[1].set_ylabel('Rate (rad/s)'); ax[1].legend(fontsize=8)
if len(amcl)>1:
 ax[2].plot(amcl[:,0]-start,np.degrees(correction-correction[0]),'.-',label='AMCL minus FAST yaw correction')
ax[2].set_ylabel('Correction (deg)'); ax[2].set_xlabel('Seconds'); ax[2].legend()
for a in ax: a.grid(alpha=.3)
fig.tight_layout(); fig.savefig(P/'rotation_diagnostic.png',dpi=150)
print(json.dumps(report,indent=2))
