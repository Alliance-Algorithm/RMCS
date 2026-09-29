#!/usr/bin/env python3
"""Discrete two-axis, filtered-feedback small-signal gain screening."""
import json
from pathlib import Path
import numpy as np
from scipy import signal

ROOT=Path(__file__).parent
models=json.loads((ROOT/'passive_surrogates.json').read_text())['models'][:5]
sensors=json.loads((ROOT/'loop_analysis.json').read_text())['velocity_reporting_models']

def make_step(model,kpp,kpv,kiv,sensor_scale=1.,extra_delay=0,cap=None,load=None):
    M=np.array(model['mass']);B=np.array(model['damping']);k=model['stiffness']
    K=k*np.array([[1.,-1.],[-1.,1.]])
    Ac=np.block([[np.zeros((2,2)),np.eye(2)],[-np.linalg.solve(M,K),-np.linalg.solve(M,B)]])
    Bc=np.vstack([np.zeros((2,2)),np.linalg.inv(M)])
    Ad,Bd,_,_,_=signal.cont2discrete((Ac,Bc,np.eye(4),np.zeros((4,2))),.001)
    actuator_alpha=np.exp(-.001/model['tau_s']) if model['tau_s'] else 0.
    motor_delay=round(model['delay_s']/.001)+extra_delay
    vel_alpha=np.exp(-.001/np.array([s['time_constant_s'] for s in sensors]))
    vd=np.array([s['delay_s']/.001 for s in sensors]);lags=vd.astype(int);fractions=vd-lags
    history=int(max(lags))+1
    size=8+history*2+motor_delay*2
    kpp=np.broadcast_to(np.asarray(kpp),2);kpv=np.broadcast_to(np.asarray(kpv),2);kiv=np.broadcast_to(np.asarray(kiv),2)
    load=np.zeros(2) if load is None else np.asarray(load)
    def step(state,ref=np.zeros(2),dref=np.zeros(2)):
        plant=state[:4];effort=state[4:6];integral=state[6:8]
        vh=state[8:8+history*2].reshape(history,2)
        uh=state[8+history*2:].reshape(motor_delay,2)
        vf=vel_alpha*vh[0]+(1-vel_alpha)*plant[2:]
        all_v=np.vstack([vf,vh])
        measured=np.array([(1-fractions[j])*all_v[lags[j],j]+fractions[j]*all_v[lags[j]+1,j] for j in (0,1)])
        measured*=np.array([s['gain'] for s in sensors])*sensor_scale
        target=dref+kpp*(ref-plant[:2])
        if cap is not None:target=np.clip(target,-4,4)
        ev=target-measured
        candidate=np.where(kiv>0,integral+.001*ev,0.)
        if cap is not None:
            integral_limit=np.divide(20.,kiv,out=np.zeros(2),where=kiv>0)
            candidate=np.clip(candidate,-integral_limit,integral_limit)
            raw=kpv*ev+kiv*candidate
            accept=(abs(raw)<=cap)|((raw>cap)&(ev<0))|((raw<-cap)&(ev>0))
            candidate=np.where(kiv>0,np.where(accept,candidate,integral),0.)
        cmd=kpv*ev+kiv*candidate
        if cap is not None:cmd=np.clip(cmd,-cap,cap)
        delayed=cmd if not motor_delay else uh[-1]
        out_effort=actuator_alpha*effort+(1-actuator_alpha)*delayed
        out_plant=Ad@plant+Bd@(out_effort-load)
        out_v=np.vstack([vf,vh])[:history].ravel()
        out_u=np.vstack([cmd,uh])[:motor_delay].ravel()
        return np.r_[out_plant,out_effort,candidate,out_v,out_u],cmd
    return step,size

def modes(model,kpp,kpv,kiv,sensor_scale=1.,extra_delay=0):
    step,n=make_step(model,kpp,kpv,kiv,sensor_scale,extra_delay)
    mat=np.column_stack([step(e)[0] for e in np.eye(n)])
    eig=np.linalg.eigvals(mat);eig=eig[abs(eig)>1e-8]
    poles=np.log(eig.astype(complex))/.001
    order=np.argsort(-poles.real)
    return [{'decay_per_s':float(poles[i].real),'hz':float(abs(poles[i].imag)/(2*np.pi))} for i in order[:4]]

if __name__=='__main__':
    result=[]
    for p in (1,1.5,2,2.5,3,4,5,6,8,10):
        for ratio in (2,3,5):
            gains={'kp_position':6.,'kp_velocity':p,'ki_velocity':p*ratio}
            baseline=modes(models[0],6,p,p*ratio)
            family=[modes(m,6,p,p*ratio,sensor_scale=sc,extra_delay=d)[0]['decay_per_s']
                    for m in models for sc in (.95,1.,1.05) for d in (0,1)]
            result.append(dict(**gains,nominal_modes=baseline,worst_real_pole=max(family)))
    report={'candidates':result,'baseline_modes':[modes(m,6,10,50) for m in models],
            'uncertainty':'Top five development-selected surrogates; sensor gain +/-5%; plus 0/1 ms command delay. Sensitivity cases, not confidence intervals.',
            'scope':'Local 5-90 Hz linear screening only; no full nonlinear closed-chain performance certification.'}
    (ROOT/'gain_screen.json').write_text(json.dumps(report,indent=2)+'\n')
    print('BASELINE',json.dumps(report['baseline_modes'],indent=2))
    for x in result:
        if x['ki_velocity']==x['kp_velocity']*5:
            print(x['kp_velocity'],x['ki_velocity'],'nominal',x['nominal_modes'][:2],'worst',x['worst_real_pole'])
