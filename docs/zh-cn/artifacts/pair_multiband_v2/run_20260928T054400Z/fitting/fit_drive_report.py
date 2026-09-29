"""Identify command-to-DM-reported-torque response, not load-cell output torque.

Fit dynamic gain, first-order time constant and a lumped report latency using
non-validation segments. Report independent time-domain validation; no q/dq is
fed into the prediction. Host/CAN/report delays cannot be split into USB and FOC.
"""
from pathlib import Path
import json
import numpy as np
from scipy import signal, optimize

R = Path(__file__).resolve().parent
raw = dict(np.load(R.parent/'response_1khz.npz'))
a = {k:v[raw['phase']==2] for k,v in raw.items()}
u = a['tau_frame_api'].copy()
for j in (0,1):
    ix=np.maximum.accumulate(np.where(np.isfinite(u[:,j]),np.arange(len(u)),0))
    u[:,j]=u[ix,j]
y = a['torque_fb_api']
train_ids=[9,11,13,15,37,39,41,43,49,51,61,63,115,117,119,121,131,133]
dev_ids=[53,55,65,67,135,137]
held_ids=[79,89]
results=[]
for j in (0,1):
    spectra=[]
    train_ix=[]
    for seg in train_ids:
        ix=np.flatnonzero(a['segment_id']==seg)[500:-500]
        train_ix.extend(ix[::10])
        f,pu=signal.welch(u[ix,j],1000,nperseg=4096,noverlap=2048)
        _,py=signal.welch(y[ix,j],1000,nperseg=4096,noverlap=2048)
        _,puy=signal.csd(u[ix,j],y[ix,j],1000,nperseg=4096,noverlap=2048)
        spectra.append((pu,py,puy))
    pu,py,puy=[np.mean([s[k] for s in spectra],axis=0) for k in range(3)]
    coherence=np.abs(puy)**2/np.maximum(pu*py,1e-20)
    keep=(f>=.5)&(f<=100)&(coherence>.9)&(pu>pu.max()*1e-5)
    ff=f[keep]; target=puy[keep]/pu[keep]
    weights=np.sqrt(pu[keep]/pu[keep].max())
    def response(p):
        gain,tau,delay=p
        return gain*np.exp(-2j*np.pi*ff*delay)/(1+2j*np.pi*ff*tau)
    def residual(p):
        r=(response(p)-target)*weights
        return np.r_[r.real,r.imag]
    fits=[optimize.least_squares(residual,[1,tau,.001],bounds=([.5,0,0],[1.5,.03,.02]))
          for tau in (.0001,.002,.008)]
    fit=min(fits,key=lambda x:np.sum(x.fun**2))
    gain,tau,delay=fit.x
    alpha=np.exp(-.001/tau) if tau>1e-9 else 0
    filtered=signal.lfilter([1-alpha],[1,-alpha],u[:,j],zi=[alpha*u[0,j]])[0]
    sample=np.arange(len(u))
    predicted=gain*np.interp(sample-delay/.001,sample,filtered)
    bias=float(np.median(y[train_ix,j]-predicted[train_ix]))
    predicted+=bias
    scores={}
    for name,ids in [('development',dev_ids),('reserved_validation',held_ids)]:
        ix=np.concatenate([np.flatnonzero(a['segment_id']==s)[1000:-1000] for s in ids])
        e=predicted[ix]-y[ix,j]
        scores[name]={'rmse_nm':float(np.sqrt(np.mean(e*e))),
            'relative_to_report_std':float(np.sqrt(np.mean(e*e))/np.std(y[ix,j])),
            'identity_rmse_nm':float(np.sqrt(np.mean((u[ix,j]-y[ix,j])**2)))}
    result={'axis':['left_hip','left_auxiliary_knee'][j],
            'dynamic_report_gain':float(gain),'time_constant_ms':float(tau*1000),
            'lumped_latency_ms':float(delay*1000),'report_bias_nm':bias,
            'fit_frequency_bins':int(keep.sum()),'validation':scores}
    results.append(result)
report={'source_bag':'left-pair-pd-loaded-20260928T054400Z',
        'scope':'Host sampled queued-command to DM-estimated torque report; NOT independently measured shaft torque or USB-only delay',
        'axes':results,'sample_period_ms':1,'heldout_segments':held_ids,
        'release_status':'diagnostic candidate; physical actuator release requires closed-chain forward validation and register/torque scale verification'}
(R/'drive_report_fit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
