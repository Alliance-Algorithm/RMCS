#!/usr/bin/env python3
"""Audit shared complete segments before comparing the three recorded gains."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from compare_wheel_runs import load, summarize

ROOT = Path(__file__).resolve().parent
BAGS = Path('/home/yukikaze/Downloads/identification_bags')
RUNS = ['20260928T093805Z', '20260928T140705Z', '20260928T100509Z']
OUT = ROOT / 'comparison_three_gains'
OUT.mkdir(exist_ok=True)
loaded = [load(BAGS / ('wheels-' + name), ROOT / ('run_' + name) / 'wheel_1khz.npz') for name in RUNS]
plan = loaded[0][1]
assert all(p == plan for _, p, _ in loaded)
excluded = {}
for s in plan['segments']:
    ident = s['id']; events = []
    for name, (d, _, _) in zip(RUNS, loaded):
        idx = np.flatnonzero(d['segment_id'] == ident)
        if not len(idx):
            excluded.setdefault(ident, []).append(name + ': missing'); continue
        t = d['time'][idx]
        # One nominal sample at each end is allowed; the recovered tail is not.
        if t[-1] - t[0] < s['duration_s'] - .005:
            excluded.setdefault(ident, []).append(name + ': incomplete duration')
        if np.any(np.diff(t) > .0018):
            excluded.setdefault(ident, []).append(name + ': scheduling gap > 1.8 ms')
        ref = d['wheel_velocity_target_api'][idx]
        events.append(ref[np.r_[True, np.any(np.diff(ref, axis=0) != 0, axis=1)]])
    if len(events) == 3 and any(e.shape != events[0].shape or not np.allclose(e, events[0], atol=1e-8, rtol=0) for e in events[1:]):
        excluded.setdefault(ident, []).append('held reference events differ')
reports = []
for name, (data, _, params) in zip(RUNS, loaded):
    mask = ~np.isin(data['segment_id'], list(excluded))
    shared = {k: v[mask] for k, v in data.items()}
    reports.append({'run': name, 'all_recorded': summarize(data, plan, params), 'shared_complete': summarize(shared, plan, params)})
result = {'runs': reports, 'excluded_segments': excluded, 'shared_segment_count': len(plan['segments'])-len(excluded),
          'notes': ['0.4 original was not sealed before reboot: recovered copy lacks completion event and final ~0.404 s of zero-current tail.',
                    'Shared metrics exclude scheduling gaps, incomplete segments and differing held target events in every run.',
                    'Initial carrier pose, temperature and run order were not randomized. This is an engineering selection, not causal proof or final deployment qualification.']}
(OUT/'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False})
fig, axes = plt.subplots(3, 2, figsize=(13, 9))
for col, side in enumerate(['left','right']):
    for row, suffix in enumerate(['low_steady','step_0','high_chirp']):
        choices=[s for s in plan['segments'] if s['label']==side+'_'+suffix]
        seg=next(s for s in choices if s['to']==.5) if row==0 else next(s for s in choices if s['to']==2) if row==1 else choices[1]
        ax=axes[row,col]
        for d, _, p in loaded:
            idx=np.flatnonzero(d['segment_id']==seg['id']); t=d['time'][idx]-d['time'][idx[0]]
            ax.plot(t,d['dq_api'][idx,col+4],lw=.85,label=f"Kp={p['wheel_velocity_kp']}")
        ax.plot(t,d['wheel_velocity_target_api'][idx,col],'k--',lw=1,label='目标')
        ax.set_title(('左轮','右轮')[col]+' · '+('0.5 rad/s 平台','0→2 rad/s 阶跃','2–8 Hz 扫频')[row])
        ax.set_xlabel('段内时间 s'); ax.set_ylabel('rad/s'); ax.grid(alpha=.2); ax.legend()
fig.suptitle('轮速 P 三组实测：0.6 跟踪更好，0.4 超调和力矩更低（0.4 包尾截断）')
fig.tight_layout();fig.savefig(OUT/'tracking_three_gains.png',dpi=150)
for r in reports:
    x=r['shared_complete'];print(r['run'],x['kp'],{k:round(v['speed_rmse_rad_s'],5) for k,v in x['groups'].items()})
print('shared',result['shared_segment_count'],'excluded',len(excluded))
