#!/usr/bin/env python3
"""Offline response/loop diagnosis. Fits empirical dynamics, not CAD parameters."""
import json
from pathlib import Path
import numpy as np
from scipy import signal, optimize

ROOT = Path(__file__).parent
a = dict(np.load(ROOT / 'response_1khz.npz'))
mask = a['phase'] == 2
a = {k: v[mask] for k, v in a.items()}
q, v, u = a['q_api'], a['dq_api'], a['tau_frame_api'].copy()
for j in (0, 1):
    valid = np.isfinite(u[:, j])
    # System-heartbeat frames have no torque; preserve the last torque input.
    latest = np.maximum.accumulate(np.where(valid, np.arange(len(u)), 0))
    u[:, j] = u[latest, j]
dt = .001
qv = signal.savgol_filter(q, 11, 4, deriv=1, delta=dt, axis=0)
qa = signal.savgol_filter(q, 11, 4, deriv=2, delta=dt, axis=0)
sid = a['segment_id']
train_ids = [1, 3, 5, 7, 29, 31, 33, 35, 41, 43, 53, 55, 107, 109, 111, 113, 123, 125]
dev_ids = [45, 47, 57, 59, 127, 129]
holdout_ids = [71, 81]
spectra = []
for segment in train_ids + dev_ids + holdout_ids:
    idx = np.flatnonzero(sid == segment)[2000:-2000]
    for j in (0, 1):
        f, pqq = signal.welch(q[idx, j], fs=1000, nperseg=2048)
        _, pvv = signal.welch(v[idx, j], fs=1000, nperseg=2048)
        _, pqv = signal.csd(q[idx, j], v[idx, j], fs=1000, nperseg=2048)
        coh = abs(pqv) ** 2 / np.maximum(pqq * pvv, 1e-30)
        good = (f > 15) & (f < 80)
        peak = np.flatnonzero(good)[np.argmax(pvv[good])]
        h = pqv[peak] / pqq[peak] / (1j * 2 * np.pi * f[peak])
        def hp_rms(x):
            return float(np.std(signal.sosfiltfilt(signal.butter(3, 15, fs=1000, btype='highpass', output='sos'), x)))
        spectra.append(dict(segment=segment, axis=j, peak_hz=float(f[peak]),
                            position_to_velocity_gain=float(abs(h)), coherence=float(coh[peak]),
                            apparent_velocity_delay_ms=float(-np.angle(h)/(2*np.pi*f[peak])*1000),
                            highpass_P_rms_nm=hp_rms(10*a['speed_error_model'][idx,j]),
                            highpass_I_rms_nm=hp_rms(a['torque_integral_model'][idx,j]),
                            mean_command_nm=float(np.mean(u[idx,j]))))
# An equivalent velocity-measurement filter, relative to differentiated encoder
# position. This includes reporting effects and must NOT be named USB latency.
sensor_models = []
for j in (0, 1):
    fs, hs, weights = [], [], []
    for segment in train_ids:
        idx = np.flatnonzero(sid == segment)[2000:-2000]
        f, pqq = signal.welch(q[idx,j], fs=1000, nperseg=2048)
        _, pqv = signal.csd(q[idx,j],v[idx,j],fs=1000,nperseg=2048)
        _, pvv = signal.welch(v[idx,j],fs=1000,nperseg=2048)
        coh=abs(pqv)**2/np.maximum(pqq*pvv,1e-30)
        good=(f>=10)&(f<=70)&(coh>.98)&(pvv>.01*np.max(pvv[(f>=10)&(f<=70)]))
        fs.extend(f[good]);hs.extend(pqv[good]/pqq[good]/(1j*2*np.pi*f[good]))
        weights.extend(np.sqrt(pvv[good]/max(pvv[good],default=1)))
    w=2*np.pi*np.array(fs); h=np.array(hs); weight=np.array(weights)
    def residual(p):
        pred=p[0]*np.exp(-1j*w*p[2])/(1+1j*w*p[1])
        r=(pred-h)*weight
        return np.r_[r.real,r.imag]
    fit=optimize.least_squares(residual,[1,.004,.001],bounds=([.5,.0001,0],[1.5,.02,.01]))
    sensor_models.append(dict(gain=float(fit.x[0]),time_constant_s=float(fit.x[1]),
                              delay_s=float(fit.x[2]),complex_rmse=float(np.sqrt(np.mean(residual(fit.x)**2))),
                              meaning='Equivalent velocity reporting filter relative to encoder position'))

# Try a coupled local response surrogate. High-pass suppresses posture loads;
# it is not a global spring/gravity fit. Output validation must qualify its use.
sos=signal.butter(3,[5,90],fs=1000,btype='bandpass',output='sos')
features=np.column_stack([qv,q[:,1]-q[:,0]])
xf=signal.sosfiltfilt(sos,features,axis=0)
yf=signal.sosfiltfilt(sos,qa,axis=0)
uf=signal.sosfiltfilt(sos,u,axis=0)
def interior(ids):
    out=[]
    for seg in ids:
        ix=np.flatnonzero(sid==seg)
        out.extend(ix[2000:-2000:5])
    return np.array(out)
train,dev,test=interior(train_ids),interior(dev_ids),interior(holdout_ids)
scale=np.std(yf[train],axis=0)
models=[]
for tau in (0,.001,.002,.003,.004,.006,.008,.012):
    alpha=np.exp(-dt/tau) if tau else 0
    filtered=signal.lfilter([1-alpha],[1,-alpha],uf,axis=0)
    for lag in range(0,11):
        delayed=np.roll(filtered,lag,axis=0)
        x=np.column_stack([delayed,xf])
        coef=np.linalg.lstsq(x[train],yf[train],rcond=None)[0]
        inv_mass=coef[:2].T
        symmetric_eigen=np.linalg.eigvalsh((inv_mass+inv_mass.T)/2)
        if min(symmetric_eigen)<=0:
            continue
        def score(ix):return float(np.sqrt(np.mean(((x[ix]@coef-yf[ix])/scale)**2)))
        models.append(dict(tau_s=tau,delay_s=lag*dt,coef=coef.tolist(),
                           train_nrmse=score(train),dev_nrmse=score(dev),
                           design_condition=float(np.linalg.cond(x[train]/np.std(x[train],axis=0))),
                           inv_mass_asymmetry=float(np.linalg.norm(inv_mass-inv_mass.T)/np.linalg.norm(inv_mass))))
models.sort(key=lambda m:m['dev_nrmse'])
for model in models[:10]:
    tau=model['tau_s'];alpha=np.exp(-dt/tau) if tau else 0
    filtered=signal.lfilter([1-alpha],[1,-alpha],uf,axis=0)
    x=np.column_stack([np.roll(filtered,round(model['delay_s']/dt),axis=0),xf])
    model['heldout_nrmse']=float(np.sqrt(np.mean(((x[test]@np.asarray(model['coef'])-yf[test])/scale)**2)))
report=dict(controller_algebra_max_error=float(np.max(abs(10*a['speed_error_model']+a['torque_integral_model']-a['tau_preclip_model']))),
            spectra=spectra,velocity_reporting_models=sensor_models,surrogate_candidates=models[:10],
            train_ids=train_ids,development_ids=dev_ids,heldout_ids=holdout_ids,
            note='Closed-loop observations can bias a direct plant fit. Surrogate is diagnostic, not calibrated physical parameters.')
(ROOT/'loop_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('spectra','surrogate_candidates')},indent=2))
print('BEST_SURROGATES',json.dumps(models[:3],indent=2))
