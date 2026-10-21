"""Extract hardware series using bundled read-only CSV dependencies."""
import ctypes
import hashlib
import json
from pathlib import Path
import numpy as np
import pandas as pd
root=Path(__file__).resolve().parent
repo=root.parents[1]
source=Path('/home/noir/wheel_leg_rl_2026-10-21_17-56-31.csv')
df=pd.read_csv(source)
names=['left_hip_joint','left_knee_joint','right_hip_joint','right_knee_joint']
lib=ctypes.CDLL(str(root/'libtargetcheck_ablation.so'))
ptr=ctypes.POINTER(ctypes.c_double)
lib.target_check.argtypes=[ctypes.c_char_p,ptr,ptr,ptr]
geometry=str(repo/'rmcs_ws/src/rmcs_core/config/wheel_leg/tuck_geometry_107_v1.json').encode()
output={}
episodes=json.loads((root/'hardware_episodes.json').read_text())
for name,start,end in [('latest',18995,19045),('first',3577,4047)]:
    rows=[]
    time0=float(df.iloc[start]['/wheel_leg/left_hip_joint/feedback_steady_ns'])
    for i in range(start,end+1):
        row=df.iloc[i]
        q=np.ascontiguousarray(-row[['/wheel_leg/'+n+'/angle' for n in names]].to_numpy(float)+[1.6,2.93,-1.6,-2.93])
        g=np.ascontiguousarray(row[['/wheel_leg/imu/sensor_gravity/'+a for a in 'xyz']].to_numpy(float))
        out=np.zeros(14)
        lib.target_check(geometry,q.ctypes.data_as(ptr),g.ctypes.data_as(ptr),out.ctypes.data_as(ptr))
        knees=[float(row['/wheel_leg/rl/recovery/tuck/'+s+'/knee_deg']) if i<end else None for s in ['left','right']]
        if name=='latest' and i==end:
            knees=next(e['first_invalid_helper']['knees'] for e in episodes if e['start']==start)
        rows.append(dict(index=i,time_s=(float(row['/wheel_leg/left_hip_joint/feedback_steady_ns'])-time0)/1e9,
            roll_deg=float(np.degrees(np.arctan2(-g[1],-g[2]))),
            tilt_deg=float(np.degrees(np.arccos(np.clip(-g[2],-1,1)))),
            knee_deg=knees,
            target_below_mm=(out[8:10]*1000).tolist() if out[13] else None,
            phase=int(row['/wheel_leg/rl/recovery/native_phase']),
            reference=[float(row['/wheel_leg/rl/recovery/'+n+'/reference']) for n in names],
            support=int(row['/wheel_leg/rl/recovery/contact_candidate'])))
    output[name]=rows
(root/'hardware_plot_data.json').write_text(json.dumps(output,indent=2,allow_nan=False))
old=json.loads((root.parent/'wheel_leg_fixed_direction_20261022/provenance.json').read_text())
files=list(old['source_sha256'])+[
    'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/closed_chain_support_observer.cpp',
    'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/controller_recovery.cpp',
    'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/action.cpp',
    'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/wheel_leg_controller.cpp']
hashes={p:hashlib.sha256((repo/p).read_bytes()).hexdigest() for p in files}
(root/'provenance.json').write_text(json.dumps(dict(source_csv=str(source),source_csv_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    source_sha256=hashes,prior_source_matches={p:hashes[p]==h for p,h in old['source_sha256'].items()},
    changes_this_turn='Only report-local analysis, adapters, and an explicitly isolated single-condition ablation. No production source/YAML/model edits.',
    scope='Native trajectory/helpers/support observer, 1 kHz, dynamic IMU, six-DOF base; actor disabled at first ownership event.',
    limitations=['zero initial velocities','CAD mass/COM and catalogue gas spring prior','asset self collisions globally filtered',
        'ideal bounded torque without measured DM delay/power envelope','no ROS/CAN transport or physical acceptance']),indent=2))
print('Prepared hardware figure data; source matches:',all((h==old['source_sha256'].get(p,h)) for p,h in hashes.items()))
