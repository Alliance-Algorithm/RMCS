"""Read-only hardware log alignment; output only selected transitions."""
import ctypes
import hashlib
import json
from pathlib import Path
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
SOURCE = Path('/home/noir/wheel_leg_rl_2026-10-21_17-56-31.csv')
df = pd.read_csv(SOURCE)
prefix = '/wheel_leg/rl/'
phase = df[prefix + 'recovery/native_phase'].to_numpy()
changes = np.flatnonzero(np.r_[True, np.diff(phase) != 0])
columns = ['index'] + [c for c in df.columns if any(s in c for s in (
    '/recovery/', '/rl/state', '/enable_request', '/rl/valid', '/rl/healthy',
    '/rl/input/projected_gravity/', '/rl/input/angular_velocity/',
    '/rl/model/', '/control_torque', '/feedback_steady_ns'))]
events = []
for i in changes:
    row = df.iloc[i]
    event = {'row': int(i), **{c: float(row[c]) for c in columns}}
    g = np.array([row[prefix + 'input/projected_gravity/' + a] for a in 'xyz'])
    event['roll_deg'] = float(np.degrees(np.arctan2(-g[1], -g[2])))
    event['pitch_deg'] = float(np.degrees(np.arcsin(np.clip(g[0], -1, 1))))
    event['tilt_deg'] = float(np.degrees(np.arccos(np.clip(-g[2], -1, 1))))
    events.append(event)
summary = {'source': str(SOURCE), 'sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
           'rows': len(df), 'columns': len(df.columns), 'phase_events': events}
(ROOT / 'hardware_events.json').write_text(json.dumps(summary, indent=2))
for e in events:
    print(json.dumps({
        'row':e['row'], 'index':e['index'],
        **{k.rsplit('/', 1)[-1]:v for k,v in e.items() if k in [
            prefix + 'recovery/' + n for n in ['native_phase','native_failure','failure',
                'tuck/valid','tuck/active','tuck/ready','trajectory_remaining_rad','tracking_error_rad']]},
        'roll':round(e['roll_deg'],3), 'pitch':round(e['pitch_deg'],3),
        'tilt':round(e['tilt_deg'],3),
        'knee':[round(e[prefix+'recovery/tuck/'+side+'/knee_deg'],3) for side in ['left','right']],
        'model_q':[round(e[prefix+'model/'+n+'/angle'],4) for n in [
            'left_hip_joint','left_knee_joint','right_hip_joint','right_knee_joint']],
        'tau':[round(e['/wheel_leg/'+n+'/control_torque'],2) for n in [
            'left_hip_joint','left_knee_joint','right_hip_joint','right_knee_joint']],
        'ns':e['/wheel_leg/left_hip_joint/feedback_steady_ns']}, ensure_ascii=False))
print('SHAPE', df.shape)

lib = ctypes.CDLL(str(ROOT.parent / 'wheel_leg_fixed_direction_20261022/libtrajectory_final.so'))
ptr = ctypes.POINTER(ctypes.c_double)
lib.wheel_leg_tuck_create.argtypes = [ctypes.c_char_p, ctypes.c_double]
lib.wheel_leg_tuck_create.restype = ctypes.c_void_p
lib.wheel_leg_tuck_step.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_double, ctypes.c_int, ptr]
geometry = ROOT.parents[1] / 'rmcs_ws/src/rmcs_core/config/wheel_leg/tuck_geometry_107_v1.json'
ctx = lib.wheel_leg_tuck_create(str(geometry).encode(), 100.)
names = ['left_hip_joint','left_knee_joint','right_hip_joint','right_knee_joint']
episodes = []
starts = np.flatnonzero((phase == 1) & np.r_[True, phase[:-1] != 1])
for start in starts:
    end = start
    while end + 1 < len(df) and phase[end + 1] in [1,7,9]:
        end += 1
    part = df.iloc[start:end+1]
    records = []
    for i in range(start, min(end + 3, len(df))):
        row = df.iloc[i]
        # Hardware angle interfaces survive fault output reset; calibrated exactly as YAML.
        q = np.ascontiguousarray(-row[['/wheel_leg/'+n+'/angle' for n in names]].to_numpy(float)
                                  + [1.6, 2.93, -1.6, -2.93], dtype=np.float64)
        g = np.ascontiguousarray(row[['/wheel_leg/imu/sensor_gravity/'+a for a in 'xyz']].to_numpy(float))
        out = np.zeros(15)
        code = lib.wheel_leg_tuck_step(ctx, q.ctypes.data_as(ptr), g.ctypes.data_as(ptr),
                                       .005, 0, out.ctypes.data_as(ptr))
        records.append({'row':i, 'index':int(row['index']), 'g':g.tolist(), 'q':q.tolist(),
            'helper_code':code, 'knees':out[8:10].tolist(), 'sag_mm':(out[10:12]*1000).tolist(),
            'roll_deg':float(np.degrees(np.arctan2(-g[1],-g[2]))),
            'tilt_deg':float(np.degrees(np.arccos(np.clip(-g[2],-1,1)))),
            'phase':int(phase[i]), 'native_failure':float(row[prefix+'recovery/native_failure'])})
    gpart = part[[prefix+'input/projected_gravity/'+a for a in 'xyz']].to_numpy(float)
    episode = {'start':int(start), 'last_active':int(end), 'after_phase':int(phase[end+1]),
        'native_failure':int(df.iloc[end+1][prefix+'recovery/native_failure']),
        'duration_s':float((df.iloc[end]['/wheel_leg/left_hip_joint/feedback_steady_ns']
                            - df.iloc[start]['/wheel_leg/left_hip_joint/feedback_steady_ns'])/1e9),
        'max_active_tilt_deg':float(np.degrees(np.arccos(np.clip(-gpart[:,2],-1,1))).max()),
        'max_active_roll_deg':float(np.abs(np.degrees(np.arctan2(-gpart[:,1],-gpart[:,2]))).max()),
        'first_invalid_helper':next((r for r in records if r['helper_code'] <= 0),None),
        'initial':records[0], 'last_records':records[-4:]}
    episodes.append(episode)
(ROOT/'hardware_episodes.json').write_text(json.dumps(episodes,indent=2))
print('EPISODE_SUMMARY')
for e in episodes:
    print(json.dumps({k:v for k,v in e.items() if k not in ['last_records','initial']},ensure_ascii=False))
for e in episodes[-2:]:
    print('LATEST',json.dumps(e,ensure_ascii=False))
