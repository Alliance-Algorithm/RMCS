"""Empirical local plant/sensor identification for PD candidate screening only.

This small-signal model separates low-frequency gravity/preload from the
identified band; it cannot replace the complete CAD+gas-spring plant in RL.
Gains passing this screen still require same-trajectory on-rig comparison.
"""
from pathlib import Path
import json

import numpy as np
from scipy import optimize, signal

ROOT = Path(__file__).resolve().parent
SPLIT = {
    'train': [9, 11, 13, 15, 37, 39, 41, 43, 49, 51, 61, 63, 115, 117, 119, 121, 131, 133],
    'development': [53, 55, 65, 67, 135, 137],
    'reserved_validation': [79, 89],
}


def load(stamp):
    run = ROOT.parent / f'run_20260928T{stamp}Z'
    raw = dict(np.load(run/'response_1khz.npz'))
    return {k: v[raw['phase'] == 2] for k,v in raw.items()}


def fit_models(a):
    q = a['q_ref_model'] - a['position_error_model']
    u = a['tau_frame_api'].copy()
    for j in (0, 1):
        ix = np.maximum.accumulate(np.where(np.isfinite(u[:, j]), np.arange(len(u)), 0))
        u[:, j] = u[ix, j]
    # Preserve continuous motor angles; filtering wrapped API phase is invalid.
    sos = signal.butter(3, [5, 70], fs=1000, btype='bandpass', output='sos')
    bp = lambda x: signal.sosfiltfilt(sos, x, axis=0)
    v = bp(signal.savgol_filter(q, 11, 4, deriv=1, delta=.001, axis=0))
    acc = bp(signal.savgol_filter(q, 11, 4, deriv=2, delta=.001, axis=0))
    delta = bp(q[:, 1]-q[:, 0])
    uf = bp(u)
    parts = {name: np.concatenate([np.flatnonzero(a['segment_id'] == s)[2000:-2000:20]
                                    for s in ids]) for name,ids in SPLIT.items()}
    train, dev = parts['train'], parts['development']
    scale = np.std(acc[train], axis=0)

    def matrices(p):
        m0, m1 = np.exp(p[:2])
        cross = p[2]*np.sqrt(m0*m1)
        return np.array([[m0,cross],[cross,m1]]), np.diag(p[3:5]), p[5]

    def predict(p, command, ix):
        m,b,k = matrices(p)
        load = np.column_stack([-delta[ix],delta[ix]])*k
        return np.linalg.solve(m,(command[ix]-v[ix]@b.T-load).T).T

    models = []
    for tau in (0., .002, .004, .006, .008, .010):
        alpha = np.exp(-.001/tau) if tau else 0.
        filtered = signal.lfilter([1-alpha], [1,-alpha], uf, axis=0)
        for lag in (0, 1, 2, 3, 4, 6):
            command = np.roll(filtered,lag,axis=0)
            residual = lambda p: ((predict(p,command,train)-acc[train])/scale).ravel()
            fit = optimize.least_squares(residual,[np.log(.03),np.log(.03),0.,.2,.2,80.],
                bounds=([np.log(.001),np.log(.001),-.95,0,0,0],
                        [np.log(2),np.log(2),.95,100,100,2000]),max_nfev=80)
            m,b,k = matrices(fit.x)
            score = lambda ix: float(np.sqrt(np.mean(((predict(fit.x,command,ix)-acc[ix])/scale)**2)))
            models.append(dict(tau_s=tau,delay_s=lag*.001,mass=m.tolist(),damping=b.tolist(),
                               stiffness=k,train_nrmse=score(train),dev_nrmse=score(dev),
                               parameters=fit.x.tolist(),optimizer_success=bool(fit.success)))
    models.sort(key=lambda m:m['dev_nrmse'])
    for m in models[:5]:
        alpha = np.exp(-.001/m['tau_s']) if m['tau_s'] else 0.
        command = np.roll(signal.lfilter([1-alpha],[1,-alpha],uf,axis=0),round(m['delay_s']/.001),axis=0)
        ix = parts['reserved_validation']
        m['validation_nrmse'] = float(np.sqrt(np.mean(((predict(m['parameters'],command,ix)-acc[ix])/scale)**2)))

    sensors = []
    for j in (0,1):
        spectra = []
        for s in SPLIT['train']:
            ix = np.flatnonzero(a['segment_id']==s)[1000:-1000]
            f,pq = signal.welch(q[ix,j],1000,nperseg=4096)
            _,pv = signal.welch(a['dq_api'][ix,j],1000,nperseg=4096)
            _,pqv = signal.csd(q[ix,j],a['dq_api'][ix,j],1000,nperseg=4096)
            spectra.append([pq,pv,pqv])
        pq,pv,pqv = np.mean(spectra,axis=0)
        coh = abs(pqv)**2/np.maximum(pq.real*pv.real,1e-30)
        mask = (f>3)&(f<70)&(coh>.85)&(pq.real*(2*np.pi*f)**2>1e-5)
        ff = f[mask]; observed = pqv[mask]/pq[mask]/(2j*np.pi*ff)
        weights = np.sqrt(pv[mask].real / max(pv[mask].real))
        def res(p):
            pred = p[0]*np.exp(-2j*np.pi*ff*p[2])/(1+2j*np.pi*ff*p[1])
            e = (pred-observed)*weights
            return np.r_[e.real,e.imag]
        fit = optimize.least_squares(res,[1,.004,.001],bounds=([.5,0,0],[1.5,.03,.02]))
        sensors.append(dict(gain=float(fit.x[0]),time_constant_s=float(fit.x[1]),delay_s=float(fit.x[2]),
                            fitted_bins=int(mask.sum()),complex_weighted_rmse=float(np.sqrt(np.mean(fit.fun**2)))))
    return models, sensors


def max_pole(model, sensors, kp, kd, sensor_scale=1., extra_delay=0):
    m = np.array(model['mass']); b = np.array(model['damping'])
    k = model['stiffness']*np.array([[1.,-1.],[-1.,1.]])
    aa = np.block([[np.zeros((2,2)),np.eye(2)],[-np.linalg.solve(m,k),-np.linalg.solve(m,b)]])
    bb = np.vstack([np.zeros((2,2)),np.linalg.inv(m)])
    ad,bd,_,_,_ = signal.cont2discrete((aa,bb,np.eye(4),np.zeros((4,2))),.001)
    alpha = np.exp(-.001/model['tau_s']) if model['tau_s'] else 0.
    va = np.exp(-.001/np.maximum([s['time_constant_s'] for s in sensors],1e-12))
    vg = np.array([s['gain'] for s in sensors])*sensor_scale
    delays = np.array([s['delay_s']/.001 for s in sensors]); lags=delays.astype(int); frac=delays-lags
    nv = int(max(lags))+1; nu=round(model['delay_s']/.001)+extra_delay
    n = 6+2*nv+2*nu
    def tick(x):
        plant=x[:4];effort=x[4:6];vh=x[6:6+2*nv].reshape(nv,2);uh=x[6+2*nv:].reshape(nu,2)
        vf=va*vh[0]+(1-va)*plant[2:];hist=np.vstack([vf,vh])
        measured=np.array([(1-frac[j])*hist[lags[j],j]+frac[j]*hist[lags[j]+1,j] for j in (0,1)])*vg
        command=-kp*plant[:2]-kd*measured
        torque=alpha*effort+(1-alpha)*(uh[-1] if nu else command)
        return np.r_[ad@plant+bd@torque,torque,hist[:nv].ravel(),np.vstack([command,uh])[:nu].ravel()]
    matrix=np.column_stack([tick(e) for e in np.eye(n)])
    eigenvalues=np.linalg.eigvals(matrix)
    return float(np.max(np.log(np.maximum(abs(eigenvalues),1e-100))/.001))


def main():
    result = {'scope':'Local 5-70 Hz PD stability screen. NOT a global actuator calibration or physical torque curve.',
              'splits':SPLIT, 'training_release':False, 'on_rig_gains_changed':False,
              'accepted_for_gain_selection':False,
              'qualification':'Exploratory inverse fit only. Requires successful free replay of the recorded baseline before gain selection.',
              'sides':{}}
    for side,stamp in [('left','054400'),('right','060953')]:
        a=load(stamp);models,sensors=fit_models(a)
        rows=[]
        for kp in (100,120,140,160,180,200,240):
            for kd in (1.5,2.,2.5,3.,3.5,4.,5.,6.):
                worst=max(max_pole(m,sensors,kp,kd,scale,delay) for m in models[:5]
                          for scale in (.95,1.05) for delay in (0,1))
                rows.append(dict(kp=kp,kd=kd,worst_local_pole_real_per_s=worst))
        result['sides'][side]={'models':models,'velocity_report_relative_to_encoder':sensors,'pd_screen':rows}
        print(side,json.dumps({'models':models[:2],'sensors':sensors,'best_damping_per_kp':[
            min((r for r in rows if r['kp']==kp),key=lambda r:r['worst_local_pole_real_per_s'])
            for kp in (100,120,140,160,180,200,240)]}),flush=True)
        (ROOT/'pd_bag_screen.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
