#!/usr/bin/env python3
"""Free closed-loop surrogate replay; distinct from frozen-feedback shadow replay."""
import json
from pathlib import Path
import numpy as np
from scipy import signal
from screen_gains import make_step,models
R=Path(__file__).parent
a=dict(np.load(R/'response_1khz.npz'));a={k:v[a['phase']==2] for k,v in a.items()}
q=a['q_ref_model']-a['position_error_model']
true_v=signal.savgol_filter(a['q_api'],11,4,deriv=1,delta=.001,axis=0)
hp=signal.butter(3,[15,80],fs=1000,btype='bandpass',output='sos')
results=[];traces={}
for seg in (71,81):
 ix=np.flatnonzero(a['segment_id']==seg);start=ix[0];prior=np.arange(start-500,start)
 origin=a['q_ref_model'][start]
 meanq=np.mean(q[prior]-origin,axis=0)
 K=models[0]['stiffness']*np.array([[1.,-1.],[-1.,1.]])
 load=np.nanmean(a['tau_frame_api'][prior],axis=0)-K@meanq
 reference=a['q_ref_model'][ix]-origin;dref=a['dq_ref_model'][ix]
 for name,p,i,cap in [('recorded_gain_cap20',10,50,20),('recorded_gain_cap40',10,50,40),('candidate4_20_cap40',4,20,40),('candidate5_25_cap40',5,25,40)]:
  step,n=make_step(models[0],6,p,i,cap=cap,load=load)
  state=np.zeros(n);state[:2]=q[start]-origin;state[2:4]=true_v[start]
  state[4:6]=a['tau_frame_api'][start];state[6:8]=a['torque_integral_model'][start]/i
  # Warm filter/command delay buffers with the measured initial state only.
  state[8:10]=a['dq_api'][start]
  if n>10:state[10:]=np.resize(a['tau_frame_api'][start],n-10)
  # Actual sensor history length is one for the current fitted filters.
  trajectory=np.empty((len(ix),2));commands=np.empty((len(ix),2))
  for k in range(len(ix)):
   trajectory[k]=state[:2]
   state,commands[k]=step(state,reference[k],dref[k])
  window=slice(2000,-2000)
  err=trajectory-reference
  f,psd=signal.welch(err[window],fs=1000,nperseg=4096,axis=0);band=(f>15)&(f<80)
  metrics=dict(segment=seg,candidate=name,kp_velocity=p,ki_velocity=i,cap_nm=cap,
               error_rmse_rad=np.sqrt(np.mean(err[window]**2,axis=0)).tolist(),
               highband_error_rms_rad=np.std(signal.sosfiltfilt(hp,err,axis=0)[window],axis=0).tolist(),
               peak_hz=f[band][np.argmax(psd[band],axis=0)].tolist(),
               command_rms_nm=np.sqrt(np.mean(commands[window]**2,axis=0)).tolist(),
               saturation_fraction=np.mean(abs(commands[window])>=cap-1e-6,axis=0).tolist())
  results.append(metrics)
  traces[f'seg{seg}_{name}']=np.column_stack([trajectory,commands])
  print(json.dumps(metrics),flush=True)
 measured=q[ix]-origin-reference
 results.append(dict(segment=seg,candidate='measured',error_rmse_rad=np.sqrt(np.mean(measured[2000:-2000]**2,axis=0)).tolist(),
                     highband_error_rms_rad=np.std(signal.sosfiltfilt(hp,measured,axis=0)[2000:-2000],axis=0).tolist()))
 traces[f'seg{seg}_reference']=reference
 traces[f'seg{seg}_measured']=q[ix]-origin
np.savez_compressed(R/'surrogate_replays.npz',**traces)
(R/'surrogate_replays.json').write_text(json.dumps({'results':results,'note':'Conditional surrogate predictions only. Static load initialized from 0.5 s preceding each held-out segment; no actual hardware A/B.'},indent=2)+'\n')
