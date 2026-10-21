"""Read-only reconstruction of the supplied vehicle log using pinned CAD geometry."""
from pathlib import Path
import ctypes
import hashlib
import json
import re

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
CSV = Path('/home/noir/wheel_leg_rl_2026-10-21_15-50-21.csv')
d0 = pd.read_csv(CSV)
d = d0.dropna(subset=['/wheel_leg/rl/state']).copy()
N = len(d)
names = ['left_hip_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']
allnames = names + ['left_wheel', 'right_wheel']

def values(prefix, suffixes):
    return d[[prefix + s for s in suffixes]].to_numpy(float)

raw = np.stack([d[f'/wheel_leg/{n}/angle'] for n in names], axis=1)
q = -raw + np.array([1.6, 2.93, -1.6, -2.93])
dq = -np.stack([d[f'/wheel_leg/{n}/velocity'] for n in names], axis=1)
tau = -np.stack([d[f'/wheel_leg/{n}/control_torque'] for n in names], axis=1)
fb_tau = -np.stack([d[f'/wheel_leg/{n}/torque'] for n in names], axis=1)
stamps = np.stack([d[f'/wheel_leg/{n}/feedback_steady_ns'] for n in allnames], axis=1)
sample_ns = np.max(stamps, axis=1)
t = (sample_ns - sample_ns[0]) * 1e-9
qw, qx, qy, qz = values('/wheel_leg/imu/quaternion/', ['w','x','y','z']).T
norm = np.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)
qw, qx, qy, qz = [v / norm for v in [qw,qx,qy,qz]]
g = np.stack([2*(qw*qy-qx*qz), -2*(qy*qz+qw*qx), -1+2*(qx*qx+qy*qy)], axis=1)
gyro = values('/wheel_leg/imu/angular_velocity/', ['x','y','z'])
accel = values('/wheel_leg/imu/acceleration/', ['x','y','z'])
tilt = np.rad2deg(np.arccos(np.clip(-g[:,2],-1,1)))
pitch = np.rad2deg(np.arctan2(g[:,0], -g[:,2]))
roll = np.rad2deg(np.arctan2(-g[:,1], -g[:,2]))
geo_path = ROOT / 'rmcs_ws/src/rmcs_core/config/wheel_leg/tuck_geometry_107_v1.json'
geo = json.loads(geo_path.read_text())
qc = q.copy()
raw_delta = np.stack([q[:,1]-q[:,0], q[:,2]-q[:,3]], axis=1)
valid = np.ones(N, dtype=bool)
knees = np.zeros((N,2))
wheels = np.zeros((N,2,3))
moment = np.tile(geo['base_moment_kg_m'], (N,1))
mass = geo['base_mass_kg']

def rotate(axis, v, angle):
    a = np.asarray(axis)
    return np.cos(angle)[:,None]*v + np.sin(angle)[:,None]*np.cross(a,v) + (1-np.cos(angle))[:,None]*np.sum(a*v,axis=1)[:,None]*a

for side, leg in enumerate(geo['sides']):
    hip = 2*side
    knots = leg['knots']
    grid = np.array([k['delta_rad'] for k in knots])
    diff = qc[:,hip+1] - qc[:,hip]
    qc[:,hip+1] += 2*np.pi*np.rint(((grid[0]+grid[-1])/2 - diff)/(2*np.pi))
    delta = qc[:,hip+1] - qc[:,hip]
    valid &= (delta >= grid[0] - 1e-6) & (delta <= grid[-1] + 1e-6)
    upper = np.clip(np.searchsorted(grid,delta),1,len(grid)-1)
    f = (delta-grid[upper-1])/(grid[upper]-grid[upper-1])
    def interp(key):
        a = np.array([k[key] for k in knots])
        z = f if a.ndim == 1 else f[:,None]
        return a[upper-1] + z*(a[upper]-a[upper-1])
    knees[:,side] = interp('knee_deg')
    pivot = np.array(leg['hip_origin_b_m'])
    angle = qc[:,hip] - leg['hip_reference_rad']
    wheels[:,side] = pivot + rotate(leg['hip_axis_b'], interp('wheel_b_m')-pivot, angle)
    moment += leg['mass_kg']*pivot + rotate(leg['hip_axis_b'],interp('moment_kg_m')-leg['mass_kg']*pivot,angle)
    mass += leg['mass_kg']
com = moment/mass
offset = wheels-com[:,None,:]
forward = np.cross(g, [0.,1.,0.])
forward /= np.linalg.norm(forward,axis=1)[:,None]
error = np.sum(offset*forward[:,None,:],axis=2)
below = np.sum(offset*g[:,None,:],axis=2)
height = np.sum(wheels*g[:,None,:],axis=2)+.06*np.sqrt(np.maximum(0,1-g[:,1,None]**2))
header = (ROOT/'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/chassis_body_geometry.hpp').read_text()
vertices = np.array([[float(x) for x in row] for row in re.findall(r'\{\s*([-0-9.]+)f,\s*([-0-9.]+)f,\s*([-0-9.]+)f\}',header)])
shell = np.max(g@vertices.T,axis=1)
state = d['/wheel_leg/rl/state'].to_numpy()
phase = d['/wheel_leg/rl/recovery/native_phase'].to_numpy()
active = state == 2
support = d['/wheel_leg/rl/recovery/support_confirmed'].to_numpy() == 1
plausible = d['/wheel_leg/rl/recovery/contact_candidate'].to_numpy() == 1
logged_height = d['/wheel_leg/rl/recovery/height_if_grounded'].to_numpy()
angle_gate = (knees >= 40).all(axis=1) & (knees <= 41).all(axis=1)
align_gate = (np.abs(error) <= .005).all(axis=1) & (below > .04).all(axis=1)
raw_hinge_gate = (raw_delta >= -.140638855-.03).all(axis=1) & (raw_delta <= 1.3889088544+.03).all(axis=1)
wheel_dq = np.stack([d[f'/wheel_leg/{n}/velocity'] for n in allnames[-2:]],axis=1)
gates = dict(geometry=valid, knee_40_41=angle_gate, wheel_alignment_5mm=align_gate,
             raw_hinge_guard=raw_hinge_gate, tilt_30deg=tilt<=30,
             angular_speed=np.linalg.norm(gyro,axis=1)<=np.where(tilt<=np.rad2deg(.2),2.,1.),
             leg_speed=np.max(np.abs(dq),axis=1)<=4., wheel_speed=np.max(np.abs(wheel_dq),axis=1)<=20.,
             height_20_29cm=(logged_height>.20)&(logged_height<.29),
             plausible_support=plausible,
             supporting_acceleration=np.sum(accel*-g,axis=1)>=6.,
             shell_clear=(logged_height>shell+.025))

lib = ctypes.CDLL(str(ROOT/'reports/wheel_leg_tuck_20261021/libwheel_leg_tuck_probe.so'))
ptr = ctypes.POINTER(ctypes.c_double)
lib.wheel_leg_tuck_create.argtypes=[ctypes.c_char_p,ctypes.c_double]
lib.wheel_leg_tuck_create.restype=ctypes.c_void_p
lib.wheel_leg_tuck_step.argtypes=[ctypes.c_void_p,ptr,ptr,ctypes.c_double,ctypes.c_int,ptr]
ctx=lib.wheel_leg_tuck_create(str(geo_path).encode(),100.)
assert ctx

def point(i):
    out=np.zeros(15)
    a,b=np.ascontiguousarray(qc[i]),np.ascontiguousarray(g[i])
    code=lib.wheel_leg_tuck_step(ctx,a.ctypes.data_as(ptr),b.ctypes.data_as(ptr),.005,0,out.ctypes.data_as(ptr))
    return dict(index=int(i),time_s=float(t[i]),tilt_deg=float(tilt[i]),pitch_deg=float(pitch[i]),roll_deg=float(roll[i]),
                raw_motor_q=raw[i].tolist(),raw_model_q=q[i].tolist(),paired_model_q=qc[i].tolist(),
                knee_deg=knees[i].tolist(),wheel_com_error_mm=(error[i]*1000).tolist(),
                wheel_offset_from_com_b_m=offset[i].tolist(),height_if_grounded=logged_height[i],
                wheel_height_reconstructed=height[i].tolist(),shell_depth_m=float(shell[i]),
                model_control_torque=tau[i].tolist(),model_feedback_torque=fb_tau[i].tolist(),
                model_dq=dq[i].tolist(),raw_hinge=raw_delta[i].tolist(),
                desired_tuck_q=out[:4].tolist() if code>0 else None,
                desired_tuck_raw_motor_q=(-out[:4]+[1.6,2.93,-1.6,-2.93]).tolist() if code>0 else None,
                gate_results={k:bool(v[i]) for k,v in gates.items()})

edges = np.diff(np.r_[False,active,False].astype(int))
episodes=[]
for n,(a,b) in enumerate(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)),1):
    sl=slice(a,b)
    phases=[int(x) for x in pd.unique(phase[sl])]
    ids=np.flatnonzero(valid[sl])+a
    best=int(ids[np.argmin(np.max(np.abs(knees[ids]-40.5),axis=1))]) if len(ids) else a
    episodes.append(dict(episode=n,start=int(a),end=int(b-1),start_s=float(t[a]),end_s=float(t[b-1]),duration_s=float(t[b-1]-t[a]),
        native_phases=phases, routes=[int(x) for x in pd.unique(d['/wheel_leg/rl/recovery/native_route'].iloc[sl])],
        knee_range_deg=[[float(np.min(knees[sl,i])),float(np.max(knees[sl,i]))] for i in range(2)],
        tilt_range_deg=[float(tilt[sl].min()),float(tilt[sl].max())],
        peak_model_control_torque_nm=np.max(np.abs(tau[sl]),axis=0).tolist(),
        torque_saturation_fraction=np.mean(np.abs(tau[sl])>=39.9,axis=0).tolist(),
        gate_true_fraction={k:float(np.mean(v[sl])) for k,v in gates.items()},
        combined_knee_alignment_count=int(np.sum(valid[sl]&angle_gate[sl]&align_gate[sl])),
        first=point(a),last=point(b-1),best_knee=point(best),
        stop_failure=float(d['/wheel_leg/rl/recovery/failure'].iloc[b]),
        stop_native_failure=float(d['/wheel_leg/rl/recovery/native_failure'].iloc[b])))

data={'source':str(CSV),'sha256':hashlib.sha256(CSV.read_bytes()).hexdigest(),
      'rows':len(d0),'usable_rows':len(d),'ignored_partial_last_rows':len(d0)-len(d),
      'time_basis':'max hardware feedback_steady_ns, offset by first row; file index is 200 Hz nominal',
      'duration_s':float(t[-1]),'geometry_sha256':hashlib.sha256(geo_path.read_bytes()).hexdigest(),
      'frame':'identity imu_to_base; model q=-API q+[1.6,2.93,-1.6,-2.93]; auxiliary full turns restored to sealed mechanical branch',
      'state_counts':{str(k):int(v) for k,v in d['/wheel_leg/rl/state'].value_counts().items()},
      'tuck_columns_present':any('tuck' in c for c in d),'episodes':episodes}
(OUT/'analysis.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
np.savez_compressed(OUT/'derived.npz',t=t,q=q,qc=qc,g=g,tilt=tilt,pitch=pitch,roll=roll,knee=knees,error=error,
                    wheels=wheels,com=com,height=height,shell=shell,state=state,phase=phase,tau=tau,fb_tau=fb_tau,dq=dq,
                    raw_hinge=raw_delta,valid=valid,angle_gate=angle_gate,align_gate=align_gate,
                    logged_height=logged_height,support=support,plausible=plausible)
for e in episodes:
    print({k:v for k,v in e.items() if k not in ['first','last','best_knee']})
    print('last',e['last'])
print('states',data['state_counts'],'usable',len(d),'seconds',t[-1])
