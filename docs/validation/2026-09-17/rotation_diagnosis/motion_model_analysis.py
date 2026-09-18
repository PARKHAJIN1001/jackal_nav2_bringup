import json, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path('/tmp/jackal_nav2_validation_20260917_KXP5m6/rotation')
s=json.loads((P/'timing_map_summary.json').read_text())
a=np.array(s['amcl_motion_update_noise_current_alpha']['rows_t_dt_absdy_absrot1_sigma_t_sigma_y'])
def st(x):return {'median':float(np.median(x)),'p90':float(np.percentile(x,90)),'max':float(np.max(x))}
omni_t=np.sqrt(.2*(a[:,1]**2+a[:,2]**2))
omni_y=omni_t.copy()
report={'meaning':'Analytical process-noise standard deviations, not measured robot errors and not a full AMCL replay. Update boundaries inferred from recorded AMCL pose stamps.',
 'differential_alpha_0.2':{'translation_std_m':st(a[:,4]),'yaw_std_deg':st(np.degrees(a[:,5]))},
 'differential_all_alpha_0.01':{'translation_std_m':st(a[:,4]*np.sqrt(.01/.2)),'yaw_std_deg':st(np.degrees(a[:,5]*np.sqrt(.01/.2)))},
 'omni_alpha_0.2':{'translation_std_m':st(omni_t),'yaw_std_deg':st(np.degrees(omni_y))}}
data=np.load(P/'trajectories.npz');f=data['fast'];t=f[:,0]-f[0,0]
keep=(t>=26)&(t<=45)
ff=f[keep];n=len(ff);ang=ff[:,4]
A=np.zeros((2*n,4));b=ff[:,1:3].reshape(-1)
A[::2,0]=1;A[1::2,1]=1;A[::2,2]=np.cos(ang);A[::2,3]=-np.sin(ang);A[1::2,2]=np.sin(ang);A[1::2,3]=np.cos(ang)
fit=np.linalg.lstsq(A,b,rcond=None)[0];pred=(A@fit).reshape(-1,2)
err=np.linalg.norm(pred-ff[:,1:3],axis=1)
report['in_place_constant_pivot_fit']={'assumption':'Robot pivot fixed, constant planar lever arm. Skid/slip and odometry drift can violate this assumption; fitted offset is not an extrinsic calibration.', 'pivot_odom_xy':fit[:2].tolist(),'equivalent_body_offset_xy_m':fit[2:].tolist(),'equivalent_radius_m':float(np.linalg.norm(fit[2:])),'residual_m':st(err),'constant_position_residual_m':st(np.linalg.norm(ff[:,1:3]-ff[:,1:3].mean(axis=0),axis=1))}
fig,ax=plt.subplots(2,1,figsize=(11,7),sharex=True)
ax[0].plot(a[:,0],a[:,4],'.-',label='Current Differential alpha=0.2')
ax[0].plot(a[:,0],omni_t,'.-',label='Omni formula, same alpha (comparison only)')
ax[0].set_ylabel('Translation noise std (m)');ax[0].legend()
ax[1].plot(a[:,0],np.degrees(a[:,5]),'.-',label='Current Differential alpha=0.2')
ax[1].plot(a[:,0],np.degrees(omni_y),'.-',label='Omni formula, same alpha (comparison only)')
ax[1].set_ylabel('Yaw noise std (degrees)');ax[1].set_xlabel('Seconds');ax[1].legend()
for aa in ax:aa.grid(alpha=.3)
fig.tight_layout();fig.savefig(P/'motion_noise.png',dpi=130)
(P/'motion_model_summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
