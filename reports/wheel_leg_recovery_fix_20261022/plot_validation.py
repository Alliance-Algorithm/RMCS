#!/usr/bin/env python3
"""Plot frozen native/Isaac evidence; no robot interfaces are accessed."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
FINAL = ROOT / 'isaac_acceptance'
BASELINE = ROOT.parent / 'wheel_leg_freebody_diagnosis_20261022/isaac_baseline_v2'


def read(folder, name):
    return json.loads((folder / name).read_text())


report = read(FINAL, 'report.json')
cases = {r['name']: r for r in report['results']}
baseline = {r['name']: r for r in read(BASELINE, 'report.json')['results']}
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), constrained_layout=True)

ax = axes[0, 0]
for folder, record, label, color in [
        (BASELINE, baseline['standing_50'], 'Original script', '#bd4932'),
        (FINAL, cases['standing_50'], 'Current script', '#247c69')]:
    samples = read(folder, 'standing_50_samples.json')
    samples = [s for s in samples if s['time_s'] <= record['terminal']['time_s'] + 1e-9]
    t = np.array([s['time_s'] for s in samples])
    q = np.array([s['native_output'][:4] for s in samples])
    # Pair-mean rotation isolates the common turn from knee folding.
    common = np.mean(q[:, :2], axis=1)
    travel = np.r_[0, np.cumsum(np.abs(np.diff(common)))] * 180 / np.pi
    ax.plot(t, travel, label=label, color=color, lw=2)
    ax.scatter(t[-1], travel[-1], color=color, s=25)
ax.set(title='Normal pose: extra revolution removed', xlabel='Time from start (s)',
       ylabel='Commanded common rotation, left pair (deg)')
ax.legend()

ax = axes[0, 1]
name = 'csv_latest_asymmetric'
samples = read(FINAL, name + '_samples.json')
stop = cases[name]['terminal']['time_s']
samples = [s for s in samples if s['time_s'] <= stop + 1e-9]
ax.plot([s['time_s'] for s in samples],
        [s['motor_feedback_angle_rad'][1] for s in samples],
        color='#bd4932', lw=2, label='Actual-turn replay: left knee')
earlier = read(ROOT / 'isaac_final', name + '_samples.json')
end = read(ROOT / 'isaac_final', 'report.json')['results'][-1]['terminal']['time_s']
earlier = [s for s in earlier if stop <= s['time_s'] <= end + 1e-9]
offset = np.array(cases[name]['source_model_turn_offset_rad'])
ax.plot([s['time_s'] for s in earlier],
        [2.93 - (s['native_output'][32] + offset[1]) for s in earlier],
        color='#777777', ls='--', label='Earlier run without raw range check')
ax.axhline(12.4, color='#bd4932', ls=':', label='Deployed guard: 12.4 rad')
ax.axvline(stop, color='#444444', ls=':', lw=1)
ax.annotate(f'Stop {stop:.3f} s', (stop, 12.4), xytext=(-10, 20),
            textcoords='offset points', ha='right')
ax.set(title='Latest CSV: feedback range still blocks handover',
       xlabel='Time from start (s)', ylabel='Raw driver API angle (rad)')
ax.legend(fontsize=8)

ax = axes[1, 0]
labels = {'standing_50': '50 deg', 'folded_40_5': 'Already folded', 'roll_18': '18 deg roll',
          'standing_305': '305 mm', 'csv_first_upright': 'CSV first start'}
for name, label in labels.items():
    case = cases[name]
    assert case['terminal']['kind'] == 'handover' and case['actor_failure'] is None
    handover = case['terminal']['time_s']
    samples = read(FINAL, name + '_samples.json')
    samples = [s for s in samples if handover <= s['time_s'] <= handover + 5.0]
    ax.plot([s['time_s'] - handover for s in samples],
            [s['tilt_deg'] for s in samples], label=label, lw=1.5)
ax.set(title='Five cases sustain actual ONNX control',
       xlabel='Time after handover (s)', ylabel='Body tilt from upright (deg)', ylim=(0, 20))
ax.legend(fontsize=8, ncol=2)

ax = axes[1, 1]
for folder, label, style in [
        (FINAL, 'Current script: no handover', '-'),
        (ROOT / 'isaac_direct_actor_comparison', 'Diagnostic: force RL at ~98 deg', '--')]:
    samples = read(folder, 'csv_upside_down_samples.json')
    ax.plot([s['time_s'] for s in samples], [s['tilt_deg'] for s in samples],
            ls=style, label=label, lw=1.7)
forced = read(ROOT / 'isaac_direct_actor_comparison', 'report.json')['results'][0]['terminal']
ax.scatter(forced['time_s'], forced['tilt_deg'], marker='x', color='#bd4932', zorder=4)
ax.set(title='Inverted case: removing the tilt gate is insufficient',
       xlabel='Time from start (s)', ylabel='Body tilt from upright (deg)', ylim=(0, 185))
ax.legend(fontsize=8)
for ax in axes.flat:
    ax.grid(alpha=.2)
fig.suptitle('Wheel-leg recovery: corrections verified, two cases remain unresolved', fontsize=15)
fig.savefig(ROOT / 'recovery_validation.png', dpi=160)
