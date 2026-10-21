"""Replay production FSM on recorded feedback; no hardware or dynamics are advanced."""
import ctypes
import hashlib
import json
from pathlib import Path
import numpy as np
import pandas as pd

root=Path(__file__).resolve().parent
repo=root.parents[1]
a=np.load(root/'derived.npz')
summary=json.loads((root/'analysis.json').read_text())
d=pd.read_csv(summary['source']).iloc[:summary['usable_rows']]
lib=ctypes.CDLL(str(root/'libreplay.so'))
ptr=ctypes.POINTER(ctypes.c_double)
lib.replay_create.argtypes=[ctypes.c_char_p]
lib.replay_create.restype=ctypes.c_void_p
lib.replay_error.restype=ctypes.c_char_p
lib.replay_destroy.argtypes=[ctypes.c_void_p]
lib.replay_step.argtypes=[ctypes.c_void_p,ptr,ptr,ptr,ptr,ctypes.c_double,ctypes.c_int,ctypes.c_double,ptr]
names=['left_hip_joint','left_knee_joint','right_hip_joint','right_knee_joint']
gyro=d[['/wheel_leg/imu/angular_velocity/'+k for k in ['x','y','z']]].to_numpy()
wheelq=d[['/wheel_leg/'+k+'/angle' for k in ['left_wheel','right_wheel']]].to_numpy()
wheeldq=d[['/wheel_leg/'+k+'/velocity' for k in ['left_wheel','right_wheel']]].to_numpy()
results=[]
replayed=np.full((len(d),25),np.nan)
for e in summary['episodes'][:5]:
    ids=np.arange(e['start'],e['end']+1)
    ids=ids[a['phase'][ids]>=0]
    ctx=lib.replay_create(str(repo/'rmcs_ws/src/rmcs_core/config/wheel_leg').encode())
    if not ctx: raise RuntimeError(lib.replay_error().decode())
    for i in ids:
        q=np.ascontiguousarray(np.r_[a['qc'][i],wheelq[i]])
        dq=np.ascontiguousarray(np.r_[a['dq'][i],wheeldq[i]])
        g=np.ascontiguousarray(a['g'][i])
        omega=np.ascontiguousarray(gyro[i])
        out=np.zeros(25)
        # The CSV is 200 Hz; hold each snapshot over five native substeps.
        dt=.001 if i==ids[0] else float(np.clip((a['t'][i]-a['t'][i-1])/5,.0001,.003))
        for sub in range(5):
            ok=lib.replay_step(ctx,q.ctypes.data_as(ptr),dq.ctypes.data_as(ptr),g.ctypes.data_as(ptr),omega.ctypes.data_as(ptr),
                float(a['logged_height'][i]),int(a['plausible'][i]),dt,out.ctypes.data_as(ptr))
            if not ok: raise RuntimeError(lib.replay_error().decode())
        replayed[i]=out
    lib.replay_destroy(ctx)
    output=replayed[ids]
    settled=ids[a['t'][ids] >= a['t'][ids[-1]]-.5]
    row=dict(episode=e['episode'],native_start=int(ids[0]),native_end=int(ids[-1]),
        phase_match_fraction=float(np.mean(output[:,21]==a['phase'][ids])),
        replay_phases=np.unique(output[:,21]).tolist(),
        tucking_fraction=float(output[:,20].mean()),
        integral_eligible_count=int(np.sum((output[:,24]<.15)&(output[:,20]>0))),
        minimum_tuck_reference_error_rad=float(output[:,24].min()),
        max_integral_nm=float(np.abs(output[:,16:20]).max()),
        torque_median_error_nm=float(np.median(np.abs(output[:,6:10]-a['tau'][ids]))),
        settled_median_torque_error_nm=float(np.median(np.abs(replayed[settled,6:10]-a['tau'][settled]))),
        final_targets=output[-1,:4].tolist(),final_tuck_reference=output[-1,12:16].tolist(),
        final_integral=output[-1,16:20].tolist(),final_model_torque=output[-1,6:10].tolist(),
        actual_final_model_torque=a['tau'][ids[-1]].tolist())
    results.append(row)
    print(row)
np.savez_compressed(root/'replay.npz',output=replayed)
(root/'replay.json').write_text(json.dumps(dict(method='Production C++ FSM and tuck controller on 200 Hz recorded snapshots, five zero-order-held native substeps; not an exact 1 kHz execution trace or a dynamics prediction',
  library_sha256=hashlib.sha256((root/'libreplay.so').read_bytes()).hexdigest(),results=results),indent=2)+'\n')
