#!/usr/bin/env python3
"""Constrained small-signal surrogate for candidate screening, never RL physics."""
import json
from pathlib import Path
import numpy as np
from scipy import signal,optimize
R=Path(__file__).parent
a=dict(np.load(R/'response_1khz.npz'));a={k:v[a['phase']==2] for k,v in a.items()}
spec=json.loads((R/'loop_analysis.json').read_text())
q=a['q_api'];u=a['tau_frame_api'].copy();dt=.001
for j in (0,1):
 ix=np.maximum.accumulate(np.where(np.isfinite(u[:,j]),np.arange(len(u)),0));u[:,j]=u[ix,j]
sos=signal.butter(3,[5,90],fs=1000,btype='bandpass',output='sos')
v=signal.sosfiltfilt(sos,signal.savgol_filter(q,11,4,deriv=1,delta=dt,axis=0),axis=0)
y=signal.sosfiltfilt(sos,signal.savgol_filter(q,11,4,deriv=2,delta=dt,axis=0),axis=0)
delta=signal.sosfiltfilt(sos,q[:,1]-q[:,0]);uf=signal.sosfiltfilt(sos,u,axis=0)
def indices(ids):return np.concatenate([np.flatnonzero(a['segment_id']==i)[2000:-2000:20] for i in ids])
train,dev,test=[indices(spec[k]) for k in ('train_ids','development_ids','heldout_ids')]
scale=np.std(y[train],axis=0)
def matrices(p):
 m0,m1=np.exp(p[:2]);cross=p[2]*np.sqrt(m0*m1)
 return np.array([[m0,cross],[cross,m1]]),np.diag(p[3:5]),p[5]
def prediction(p,uu,ii):
 M,B,k=matrices(p)
 load=np.column_stack([-delta[ii],delta[ii]])*k
 return np.linalg.solve(M,(uu[ii]-v[ii]@B.T-load).T).T
models=[]
for tau in (0,.002,.004,.006,.008):
 alpha=np.exp(-dt/tau) if tau else 0
 filtered=signal.lfilter([1-alpha],[1,-alpha],uf,axis=0)
 for lag in (0,1,2,3,4,6,8):
  uu=np.roll(filtered,lag,axis=0)
  def residual(p):return ((prediction(p,uu,train)-y[train])/scale).ravel()
  fit=optimize.least_squares(residual,[np.log(.03),np.log(.03),-.1,.2,.2,10],
        bounds=([np.log(.001),np.log(.001),-.95,0,0,0],[np.log(2),np.log(2),.95,100,100,2000]),max_nfev=60)
  M,B,k=matrices(fit.x)
  score=lambda ix:float(np.sqrt(np.mean(((prediction(fit.x,uu,ix)-y[ix])/scale)**2)))
  models.append(dict(tau_s=tau,delay_s=lag*dt,mass=M.tolist(),damping=B.tolist(),stiffness=k,
                     train_nrmse=score(train),dev_nrmse=score(dev),parameters=fit.x.tolist()))
models.sort(key=lambda m:m['dev_nrmse'])
for m in models[:10]:
 alpha=np.exp(-dt/m['tau_s']) if m['tau_s'] else 0
 uu=np.roll(signal.lfilter([1-alpha],[1,-alpha],uf,axis=0),round(m['delay_s']/dt),axis=0)
 m['holdout_nrmse']=float(np.sqrt(np.mean(((prediction(m['parameters'],uu,test)-y[test])/scale)**2)))
(R/'passive_surrogates.json').write_text(json.dumps({'models':models,'meaning':'Empirical 5-90 Hz coupled small-signal screening model. No global gravity/gas-spring identification.'},indent=2)+'\n')
print(json.dumps(models[:4],indent=2))
