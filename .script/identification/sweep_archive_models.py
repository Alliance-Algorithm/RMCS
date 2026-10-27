"""Time-aligned, integral grey-box fits and independent sweep validation."""
from __future__ import annotations

import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import lfilter

from analyze_joint_sweep import metrics, physical_parameters, previous_hold
from identify_sweep_archive import S, array


def stage_frames(data):
    active=(array(data,S+"state")==3)&(array(data,S+"reference_active")==1)
    frames=[]
    groups=list(data.loc[active].groupby([S+"run_id",S+"stage"],sort=False))
    for number,((run,stage),frame) in enumerate(groups):
        if S+"stage_elapsed_s" in frame:
            elapsed=array(frame,S+"stage_elapsed_s")
            complete=elapsed.min()<.03 and elapsed.max()>float(frame[S+"stage_duration_s"].iloc[0])-.03
        else:
            # Old wheel logs have no segment duration: require a recorded advance
            # to the next stage of the same run, never assume a cut last stage ended.
            complete=(number+1<len(groups) and groups[number+1][0][0]==run
                      and groups[number+1][0][1]==stage+1)
        frames.append((int(run),int(stage),bool(complete),frame))
    return frames


def uniform_series(frame, axes, dt=.002):
    """Align independently timestamped fresh motor frames on a shared host clock."""
    zero=int(frame[S+"control_steady_ns"].iloc[0])
    samples=[]
    for axis in axes:
        m,p="/wheel_leg/"+axis+"/",S+axis+"/"
        seq=array(frame,m+"feedback_sequence")
        ns=array(frame,m+"feedback_steady_ns")
        control=array(frame,S+"control_steady_ns")
        q=array(frame,p+"model_angle",np.nan)
        v=array(frame,p+"model_velocity",np.nan)
        tau=array(frame,m+"torque",np.nan)
        api_v=array(frame,m+"velocity",np.nan)
        moving=np.isfinite(v+api_v)&(np.abs(api_v)>.005)
        sign=float(np.sign(np.median(v[moving]/api_v[moving]))) if moving.any() else 1.
        if moving.any() and not np.allclose(v[moving],sign*api_v[moving],atol=1e-6,rtol=1e-6):
            raise ValueError("API/model velocity sign contract changed within a stage")
        tau=sign*tau
        valid=(np.r_[True,np.diff(seq)>0] & (ns>0) & (control>=ns)
               & (control-ns<3_000_000) & np.isfinite(q+v+tau))
        ii=np.flatnonzero(valid)
        if len(ii)<100:
            return None
        ii=ii[np.r_[True,np.diff(ns[ii])>0]]
        t=(ns[ii]-zero)*1e-9
        samples.append((t,q[ii],v[ii],tau[ii],array(frame,p+"saturated")[ii]))
    start=max(x[0][0] for x in samples)
    end=min(x[0][-1] for x in samples)
    t=np.arange(start,end,dt)
    if len(t)<100:
        return None
    result={"t":t,"q":[],"v":[],"tau":[],"good":np.ones(len(t),dtype=bool)}
    for times,q,v,tau,sat in samples:
        right=np.clip(np.searchsorted(times,t),1,len(times)-1)
        gap=times[right]-times[right-1]
        result["good"] &= (gap<=.005)&(sat[right]==0)&(sat[right-1]==0)
        for key,values in (("q",q),("v",v),("tau",tau)):
            result[key].append(np.interp(t,times,values))
    for key in ("q","v","tau"):
        result[key]=np.asarray(result[key]).T
    return result


def integral_observations(series, window_s=.04, stride_s=.01):
    t,v,q,tau=(series[key] for key in ("t","v","q","tau"))
    dt=float(np.median(np.diff(t)))
    window=max(2,round(window_s/dt))
    lo=np.arange(0,len(t)-window,max(1,round(stride_s/dt)))
    hi=lo+window
    bad=np.r_[0,np.cumsum(~series["good"])]
    keep=(bad[hi+1]-bad[lo]==0)&(t[lo]>.25)&(t[hi]<t[-1]-.1)
    lo,hi=lo[keep],hi[keep]
    duration=(t[hi]-t[lo])[:,None]
    result={"acc":(v[hi]-v[lo])/duration,"t":.5*(t[lo]+t[hi])}
    for key,values in (("q",q),("v",v),("tau",tau),("friction",np.tanh(30*v)),
                       ("quadratic",v*np.abs(v)),("positive",(v>0).astype(float)),
                       ("negative",-(v<0).astype(float))):
        integral=cumulative_trapezoid(values,t,axis=0,initial=0)
        result[key]=(integral[hi]-integral[lo])/duration
    return result


def fit_wheel(observations):
    x=np.column_stack([observations[k].ravel() for k in
                       ("acc","v","quadratic","positive","negative")])
    y=observations["tau"].ravel()
    heldout=observations["heldout"].astype(bool)
    train=~heldout
    if train.sum()<500 or heldout.sum()<300:
        return {"accepted":False,"reason":"insufficient independent wheel observations"}
    norms=np.linalg.norm(x[train],axis=0)
    if np.any(norms<1e-8):
        return {"accepted":False,"reason":"unexcited wheel parameter"}
    condition=float(np.linalg.cond(x[train]/norms))
    bounds=np.array([1e-6,0,0,0,0])*norms,np.array([.5,5,1,5,5])*norms
    weights=np.ones(train.sum())
    for _ in range(5):
        result=lsq_linear(x[train]/norms*weights[:,None],y[train]*weights,
                          bounds=bounds,tol=1e-10)
        p=result.x/norms
        residual=x[train]@p-y[train]
        scale=max(.01,1.4826*np.median(np.abs(residual-np.median(residual))))
        weights=np.sqrt(np.minimum(1,1.5*scale/np.maximum(np.abs(residual),1e-12)))
    prediction=x@p
    return {"accepted":bool(result.success and condition<1e4 and p[0]>1.01e-6),
            "parameter_order":["J_kg_m2","B_nm_s_rad","quadratic_nm_s2_rad2","Fc_positive_nm","Fc_negative_nm"],
            "parameters":p.tolist(),"condition":condition,
            "train":metrics(y[train],prediction[train]),"holdout":metrics(y[heldout],prediction[heldout]),
            "train_samples":int(train.sum()),"holdout_samples":int(heldout.sum()),
            "torque_scope":"drive-current estimate and nominal wheel torque constant"}


def wheel_drag(v,parameters):
    _,b,c,fp,fn=parameters
    # A continuous near-zero approximation is only used in numerical replay;
    # the fitted platform model uses direction-specific Coulomb terms.
    return b*v+c*v*abs(v)+(fp if v>=0 else fn)*np.tanh(v/.05)


def validate_wheel(model,series_list):
    errors=[]
    per_stage=[]
    j=model["parameters"][0]
    for s in series_list:
        if not s["heldout"]:
            continue
        t,v,u=s["t"],s["v"][:,0],s["tau"][:,0]
        local=[]
        for start in range(150,len(t)-100,100):
            if not np.all(s["good"][start:start+101]):
                continue
            speed=v[start]
            for k in range(start,start+100):
                dt=t[k+1]-t[k]
                speed+=dt*(u[k]-wheel_drag(speed,model["parameters"]))/j
                local.append(speed-v[k+1])
        if local:
            per_stage.append({"source":s["source"],"run":s["run"],"stage":s["stage"],
                              "speed_rmse_rad_s":float(np.sqrt(np.mean(np.square(local))))})
            errors.extend(local)
    return {"accepted":bool(len(errors)>1000 and np.sqrt(np.mean(np.square(errors)))<.4),
            "window_s":.2,"speed_rmse_rad_s":float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            "stages":per_stage}


def fit_leg_local(observations):
    from analyze_joint_sweep import design
    q,v,a,tau=(observations[k] for k in ("q","v","acc","tau"))
    heldout=observations["heldout"].astype(bool); train=~heldout
    if train.sum()<500 or heldout.sum()<500:
        return {"accepted":False,"reason":"need complete local training and independent validation"}
    center=np.mean(q[train],axis=0)
    x=design(a,v,q,center)
    x[:,0,6]=observations["friction"][:,0]
    x[:,1,7]=observations["friction"][:,1]
    matrix=x[train].reshape(-1,14)
    norms=np.linalg.norm(matrix,axis=0)
    if np.any(norms<1e-8):
        return {"accepted":False,"reason":"unexcited local mechanical parameter"}
    condition=float(np.linalg.cond(matrix/norms))
    initial=np.linalg.lstsq(matrix,tau[train].ravel(),rcond=None)[0]
    for offset in (0,3):
        aa,bb,cc=initial[offset:offset+3]
        values,vectors=np.linalg.eigh([[aa,bb],[bb,cc]])
        factor=np.linalg.cholesky((vectors*np.maximum(values,1e-4))@vectors.T)
        initial[offset:offset+3]=np.log(factor[0,0]),factor[1,0],np.log(factor[1,1])
    initial[6:8]=np.log(np.maximum(initial[6:8],.01))
    lower,upper=np.full(14,-np.inf),np.full(14,np.inf)
    lower[[0,2,3,5,6,7]],upper[[0,2,3,5,6,7]]=-15,6
    result=least_squares(lambda z:matrix@physical_parameters(z)-tau[train].ravel(),initial,
        bounds=(lower,upper),loss="soft_l1",f_scale=.2,max_nfev=400,x_scale="jac")
    p=physical_parameters(result.x); prediction=np.einsum("nij,j->ni",x,p)
    checks=[metrics(tau[heldout,j],prediction[heldout,j]) for j in (0,1)]
    return {"accepted":bool(result.success and condition<1e5 and all(c["nrmse"]<.4 for c in checks)),
            "M":[[p[0],p[1]],[p[1],p[2]]],"B":[[p[3],p[4]],[p[4],p[5]]],
            "coulomb":p[6:8].tolist(),"load_affine":p[8:].reshape(2,3).tolist(),
            "q_center":center.tolist(),"condition":condition,"holdout_torque":checks,
            "train_torque":[metrics(tau[train,j],prediction[train,j]) for j in (0,1)],
            "optimizer_success":bool(result.success),"train_samples":int(train.sum()),
            "holdout_samples":int(heldout.sum()),"integral_window_s":.04}


def fit_current_delay(segments,axis,minimum_speed=0):
    """Positive-delay command-to-reported-current fit, split by complete segments."""
    groups=[]
    for frame,heldout in segments:
        m,p="/wheel_leg/"+axis+"/",S+axis+"/"
        v=array(frame,p+"model_velocity")
        if np.median(np.abs(v))<minimum_speed:
            continue
        zero=int(frame[S+"control_steady_ns"].iloc[0])
        tx=array(frame,m+"tx_queued_steady_ns"); ns=array(frame,m+"feedback_steady_ns")
        u=array(frame,m+"tau_frame_api",np.nan); y=array(frame,m+"torque",np.nan)
        control=array(frame,S+"control_steady_ns"); seq=array(frame,m+"feedback_sequence")
        ci=np.flatnonzero((array(frame,m+"tx_kind")==1)&np.isfinite(u)&(tx>0))
        if len(ci): ci=ci[np.r_[True,np.diff(tx[ci])>0]]
        fi=np.flatnonzero(np.r_[False,np.diff(seq)>0]&np.isfinite(y)&(control>=ns)
                         &(control-ns<3_000_000)&(array(frame,p+"saturated")==0))
        if len(ci)<500 or len(fi)<500 or np.std(u[ci])<.025:
            continue
        ct,ft=(tx[ci]-zero)*1e-9,(ns[fi]-zero)*1e-9
        start=max(ct[0]+.061,ft[0]); end=min(ct[-1],ft[-1])
        grid=np.arange(start,end,.001)
        if len(grid)<500:continue
        measured=np.interp(grid,ft,y[fi])
        groups.append((grid,ct,u[ci],measured,heldout))
    if not groups or not any(x[-1] for x in groups) or all(x[-1] for x in groups):
        return {"accepted":False,"reason":"need independently excited complete train and holdout torque segments"}
    best=None
    for delay in np.arange(0,.0401,.001):
        for lag in (0,.001,.002,.003,.005,.008,.012,.02,.04):
            responses=[]; measured=[]; heldout=[]
            for grid,ct,u,y,validation in groups:
                drive=previous_hold(ct,u,grid-delay)
                alpha=np.exp(-.001/lag) if lag else 0
                response=lfilter([1-alpha],[1,-alpha],drive,zi=[alpha*drive[0]])[0]
                responses.append(response[250:]);measured.append(y[250:])
                heldout.append(np.full(len(y)-250,validation))
            x=np.column_stack((np.concatenate(responses),np.ones(sum(len(x) for x in responses))))
            y=np.concatenate(measured); val=np.concatenate(heldout); train=~val
            gain,bias=np.linalg.lstsq(x[train],y[train],rcond=None)[0]
            if gain<=0:continue
            pred=x@[gain,bias]; error=metrics(y[train],pred[train])
            if best is None or error["rmse"]<best["train"]["rmse"]:
                best={"delay_ms":float(delay*1000),"time_constant_ms":float(lag*1000),
                      "gain":float(gain),"bias_nm":float(bias),"train":error,
                      "holdout":metrics(y[val],pred[val]),"segments":len(groups)}
    if best is None:return {"accepted":False,"reason":"no positive command-to-current response"}
    best["accepted"]=(best["holdout"]["nrmse"]<.4 and best["delay_ms"]<40
                       and best["time_constant_ms"]<40)
    best["scope"]="host queue to reported current; transport, drive and feedback measurement combined"
    best["resolution_ms"]=1
    return best
