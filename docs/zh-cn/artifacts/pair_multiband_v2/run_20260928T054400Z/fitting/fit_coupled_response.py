"""Constrained empirical two-coordinate model with gravity and fixed spring prior.

c=(hip+aux)/2, d=aux-hip; forces are sum(tau), (tau_aux-tau_hip)/2.
A residual elastic term is diagnostic and MUST NOT be called motor friction or
written on top of a full CAD mechanism. Hardware mapping is still provisional.
"""
from pathlib import Path
import json, math
import numpy as np
from scipy import signal, optimize

R=Path(__file__).resolve().parent
raw=dict(np.load(R.parent/'response_1khz.npz'))
a={k:v[raw['phase']==2] for k,v in raw.items()}
q=a['q_ref_model']-a['position_error_model']; n=len(q)
x=np.column_stack([q.mean(axis=1), q[:,1]-q[:,0]])
v=signal.savgol_filter(x,41,4,deriv=1,delta=.001,axis=0)
acc=signal.savgol_filter(x,41,4,deriv=2,delta=.001,axis=0)
u=a['tau_frame_api'].copy()
for j in (0,1):
 idx=np.maximum.accumulate(np.where(np.isfinite(u[:,j]),np.arange(n),0));u[:,j]=u[idx,j]
gas=json.loads((R/'gas_prior_lookup.json').read_text())
gas_x=np.array(gas['delta_rad']);gas_y=np.array(gas['resisting_generalized_torque_nm'])
d0=-1.32
sos=signal.butter(3,10,fs=1000,output='sos')
def low(y):return signal.sosfiltfilt(sos,y,axis=0)
# Nonlinear load features are evaluated before filtering.
features=low(np.column_stack([v[:,0],v[:,1],np.tanh(v[:,0]/.1),np.tanh(v[:,1]/.02),
 x[:,1]-d0,np.sin(x[:,0]),np.cos(x[:,0]),np.ones(n)]))
af=low(acc)
gf=low(np.interp(x[:,1],gas_x,gas_y))
parts={'train':[2,4,6,8,9,11,13,15,37,39,41,43,49,51,61,63,115,117,119,121,131,133],
 'development':[53,55,65,67,135,137], 'reserved_validation':[79,89]}
indices={k:np.concatenate([np.flatnonzero(a['segment_id']==s)[500:-500:10] for s in ids]) for k,ids in parts.items()}

def unpack(p):
 mc,md=np.exp(p[:2]);cross=.95*np.tanh(p[2])*np.sqrt(mc*md)
 return np.array([[mc,cross],[cross,md]])
def predict_inverse(p,ix):
 f=features[ix]; M=unpack(p)
 gravity=f[:,5:8]
 common=p[3]*f[:,0]+p[5]*f[:,2]+gravity@p[8:11]
 diff=p[4]*f[:,1]+p[6]*f[:,3]+p[7]*f[:,4]+gravity@p[11:14]+gf[ix]
 return af[ix]@M.T+np.column_stack([common,diff])
start=np.array([math.log(.13),math.log(.015),0.,.2,.2,2.,.5,200.,3.,-1.,.5,.2,.2,-10.])
bounds=([-8,-9,-2,0,0,0,0,0,-20,-20,-30,-20,-20,-100],
        [1,1,2,20,100,20,30,2000,20,20,30,20,20,100])
models=[]
for tau in (0.,.002,.006,.010):
 alpha=np.exp(-.001/tau) if tau else 0.
 act=signal.lfilter([1-alpha],[1,-alpha],u,axis=0,zi=(alpha*u[0])[None,:])[0]
 force=low(np.column_stack([act.sum(axis=1),np.diff(act,axis=1)[:,0]/2]))
 ix=indices['train'];scale=np.array([4.,4.])
 def residual(p):return ((predict_inverse(p,ix)-force[ix])/scale).ravel()
 fit=optimize.least_squares(residual,start,bounds=bounds,loss='soft_l1',f_scale=.3,max_nfev=150,xtol=1e-9,ftol=1e-9,gtol=1e-9)
 scores={name:np.sqrt(np.mean((predict_inverse(fit.x,ii)-force[ii])**2,axis=0)).tolist() for name,ii in indices.items() if name!='reserved_validation'}
 result={'actuator_time_constant_ms':tau*1000,'mass_matrix_modal':unpack(fit.x).tolist(),'parameters':fit.x.tolist(),'inverse_rmse_nm':scores,'success':bool(fit.success)}
 models.append(result);print(result,flush=True)
# Freeze based on development inverse response, never on held-out trajectory error.
chosen=min(models,key=lambda m:sum(np.array(m['inverse_rmse_nm']['development'])**2))
p=np.array(chosen['parameters']); Minv=np.linalg.inv(unpack(p));tau=chosen['actuator_time_constant_ms']/1000
alpha=np.exp(-.001/tau) if tau else 0.

def replay(seg):
 ix=np.flatnonzero(a['segment_id']==seg)
 pos=x[ix[0]].copy();vel=v[ix[0]].copy();eff=u[ix[0]].copy()
 out=np.empty((len(ix),2));sent=np.empty((len(ix),2))
 def accel_at(z,w,f):
  si,co=math.sin(z[0]),math.cos(z[0])
  load=np.array([p[3]*w[0]+p[5]*math.tanh(w[0]/.1)+p[8]*si+p[9]*co+p[10],
   p[4]*w[1]+p[6]*math.tanh(w[1]/.02)+p[7]*(z[1]-d0)+p[11]*si+p[12]*co+p[13]+np.interp(z[1],gas_x,gas_y)])
  return Minv@(f-load)
 for k,i in enumerate(ix):
  out[k]=pos
  qq=np.array([pos[0]-pos[1]/2,pos[0]+pos[1]/2]); vv=np.array([vel[0]-vel[1]/2,vel[0]+vel[1]/2])
  request=np.clip(120*(a['q_ref_model'][i]-qq)-3*vv,-40,40);sent[k]=request
  eff=alpha*eff+(1-alpha)*request
  f=np.array([eff.sum(),(eff[1]-eff[0])/2])
  acc0=accel_at(pos,vel,f);pm=pos+.0005*vel;vm=vel+.0005*acc0
  pos+=.001*vm;vel+=.001*accel_at(pm,vm,f)
  if not np.isfinite(pos).all() or np.max(np.abs(vel))>1000:
   out[k+1:]=np.nan;sent[k+1:]=np.nan;break
 trim=slice(1500,-1500);err=out[trim]-x[ix][trim]
 peraxis=np.column_stack([err[:,0]-err[:,1]/2,err[:,0]+err[:,1]/2])
 return {'segment':seg,'modal_prediction_rmse_rad':np.sqrt(np.mean(err**2,axis=0)).tolist(),
         'axis_prediction_rmse_rad':np.sqrt(np.mean(peraxis**2,axis=0)).tolist(),
         'max_simulated_request_nm':np.max(np.abs(sent),axis=0).tolist()},ix,out
metrics=[];traces={}
for seg in (53,65,135,79,89):
 m,ix,out=replay(seg);metrics.append(m);traces[f'{seg}_indices']=ix;traces[f'{seg}_modal']=out
np.savez_compressed(R/'coupled_forward_validation.npz',**traces)
report={'model_type':'empirical_coupled_modal_with_fixed_conditional_CAD_spring_prior',
 'coordinates':['common_average_rad','auxiliary_minus_hip_rad'],
 'scope':'Fixed chassis, installed assembly and gravity. Sine/cosine gravity loads retained. Nominal 1 ms control, 50 Hz recorded targets, simulated feedback during replay.',
 'parameter_names':['log_Mcc','log_Mdd','mass_correlation_latent','B_common','B_difference','F_common','F_difference','residual_difference_stiffness','G_common_sin','G_common_cos','common_bias','G_difference_sin','G_difference_cos','difference_bias'],
 'models':models,'chosen':chosen,'forward_validation':metrics,
 'training_release':False,
 'limitations':['Residual stiffness/bias is NOT motor friction or a replacement gas curve.',
 'Actuator gain, hardware-CAD zero, hard-stop compliance and physical gas preload are not independently identified.',
 'This effective model is not the full rigid-body closed-chain plant; do not apply its mass/load parameters on top of CAD.'],
 'gas_prior_manifest_sha256':gas['source_manifest_sha256']}
(R/'coupled_response_fit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'chosen':chosen,'forward_validation':metrics},indent=2),flush=True)
