import sys,json
from pathlib import Path
import numpy as np
from scipy import signal
R=Path(__file__).resolve().parent
models=json.loads((R/'passive_surrogates.json').read_text())['models'][:5]
sensors=json.loads((R/'loop_analysis.json').read_text())['velocity_reporting_models']
def transition(m,kp,kd,scale=1,extra=0):
 M=np.array(m['mass']);B=np.array(m['damping']);K=m['stiffness']*np.array([[1.,-1.],[-1.,1.]])
 A=np.block([[np.zeros((2,2)),np.eye(2)],[-np.linalg.solve(M,K),-np.linalg.solve(M,B)]])
 U=np.vstack([np.zeros((2,2)),np.linalg.inv(M)])
 Ad,Bd,_,_,_=signal.cont2discrete((A,U,np.eye(4),np.zeros((4,2))),.001)
 alpha=np.exp(-.001/m['tau_s']) if m['tau_s'] else 0
 va=np.exp(-.001/np.array([s['time_constant_s'] for s in sensors]));vg=np.array([s['gain'] for s in sensors])*scale
 vd=np.array([s['delay_s']/.001 for s in sensors]);lags=vd.astype(int);frac=vd-lags;nv=int(max(lags))+1;nu=round(m['delay_s']/.001)+extra
 n=8+nv*2+nu*2
 def tick(x,due):
  plant=x[:4];effort=x[4:6];vh=x[6:6+2*nv].reshape(nv,2);uh=x[6+2*nv:-2].reshape(nu,2)
  vf=va*vh[0]+(1-va)*plant[2:];hist=np.vstack([vf,vh]);meas=np.array([(1-frac[j])*hist[lags[j],j]+frac[j]*hist[lags[j]+1,j] for j in (0,1)])*vg
  u=-kp*plant[:2]-kd*meas if due else x[-2:]
  torque=alpha*effort+(1-alpha)*(uh[-1] if nu else u)
  return np.r_[Ad@plant+Bd@torque,torque,hist[:nv].ravel(),np.vstack([u,uh])[:nu].ravel(),u]
 def cycle(x):
  for k in range(1):x=tick(x,k==0)
  return x
 mat=np.column_stack([cycle(e) for e in np.eye(n)]);eig=np.linalg.eigvals(mat);eig=eig[abs(eig)>1e-8];poles=np.log(eig.astype(complex))/.001
 return max(poles.real)
results=[]
for kp in (40,60,80,100):
 for kd in (.5,1.,1.5,2.,2.5,3.,4.):
  score=[transition(m,kp,kd,s,d) for m in models for s in (.95,1,1.05) for d in (0,1)]
  result=dict(kp=kp,kd=kd,nominal=transition(models[0],kp,kd),worst=max(score));results.append(result);print(result,flush=True)
(R/'rl_pd_1khz_screen.json').write_text(json.dumps({'scope':'1000 Hz recomputed PD, no velocity feedforward; 1 kHz plant/sensor integration; local empirical ensemble sensitivity only, not validated full-chain dynamics','results':results},indent=2)+'\n')
