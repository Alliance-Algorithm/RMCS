#!/usr/bin/env python3
"""Retrospective stick/slip audit of the archived P=.6 bag; no hardware or training writes."""
import json
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent
RUN = OUT.parent/'run_20260928T100509Z'
PHYSICAL = json.loads((OUT/'offline_fit_audit.json').read_text())
PLAN = json.loads((RUN/'trajectory-plan.json').read_text())
with np.load(RUN/'wheel_1khz.npz') as source:
    active = source['phase'] == 2
    data = {k: source[k][active] for k in ('control_steady_ns', 'segment_id', 'q_api', 'dq_api',
        'wheel_velocity_target_api', 'tau_frame_api', 'torque_fb_api')}
data['t'] = (data['control_steady_ns'].astype(np.int64)-int(data['control_steady_ns'][0]))*1e-9


def simulate(t, target, initial, p, fs_pos, fs_neg):
    j, cp, cn, b = p
    w = float(initial)
    stuck = abs(w) < .02
    if stuck:
        w = 0.
    prediction = np.empty_like(t)
    prediction[0] = w
    for k in range(len(t)-1):
        dt = t[k+1]-t[k]
        torque = max(-4.5, min(4.5, .6*(target[k]-w)))
        fs = fs_pos if torque >= 0 else fs_neg
        if stuck and abs(torque) <= fs:
            w = 0.
        else:
            stuck = False
            sign = 1 if w > 0 else -1 if w < 0 else 1 if torque > 0 else -1
            friction = cp if sign > 0 else -cn
            new_w = w + dt*(torque-friction-b*w)/j
            if new_w*w < 0 and abs(torque) <= fs:
                w = 0.
                stuck = True
            else:
                w = new_w
        prediction[k+1] = w
    return prediction


def sample(ids, side):
    ii = np.flatnonzero(np.isin(data['segment_id'], ids))
    assert len(ii) and np.max(np.diff(ii)) == 1
    t = data['t'][ii]-data['t'][ii[0]]
    assert np.min(np.diff(t)) > 0 and np.max(np.diff(t)) < .002
    return ii, t, data['wheel_velocity_target_api'][ii, side], data['dq_api'][ii, side+4]


def onset_candidates(ii, side, target_sign, strict=True):
    # Two windows separated by a 20 ms transition allowance. This deliberately
    # yields an observed torque band, not an exact instantaneous friction value.
    start, end = int(ii[0]), int(ii[-1])
    q = np.unwrap(data['q_api'][:, side+4])
    w = data['dq_api'][:, side+4]
    torque = target_sign*data['tau_frame_api'][:, side+4]
    feedback = target_sign*data['torque_fb_api'][:, side+4]
    result = []
    next_allowed = start
    for k in range(max(70, start), end-20):
        if k < next_allowed:
            continue
        before = slice(k-70, k-20)
        after = slice(k, k+20)
        if (np.quantile(abs(w[before]), .95) > (.02 if strict else .04)
                or np.ptp(q[before]) > (.0003 if strict else .001)):
            continue
        if np.mean(target_sign*w[after] > .05) < .8:
            continue
        if target_sign*(q[k+19]-q[k]) < .001:
            continue
        transition = slice(k-20, k+20)
        result.append({'time_from_entry_s': float(data['t'][k]-data['t'][start]),
            'qualification': 'strict_quasi_stationary' if strict else 'presliding_tolerant_not_strict_rest',
            'nominal_command_stationary_max_nm': float(np.max(torque[before])),
            'nominal_command_transition_max_nm': float(np.max(torque[transition])),
            'reported_stationary_max_nm': float(np.max(feedback[before])),
            'reported_transition_max_nm': float(np.max(feedback[transition])),
            'rest_position_span_rad': float(np.ptp(q[before])),
            'transition_window_s': [-.020, .020], 'stationary_window_s': [-.070, -.020]})
        next_allowed = k+150
    return result


result = {'status': 'retrospective_effective_stiction_study_not_released_to_training',
    'source_npz_sha256': PHYSICAL['provenance']['p06']['npz_sha256'],
    'fixed_controller': {'kp': .6, 'ki': 0, 'kd': 0, 'torque_cap_nm': 4.5},
    'fit_split': 'Only low-entry plus following steady targets +/-0.5 and +/-1.0 rad/s; J/C+/C-/b fixed from the earlier integral study',
    'same_bag_checks': 'Targets +/-0.2 excluded from both physical fit and Fs search. step_1 excluded only from Fs search, already used by the earlier physical fit. Retrospective same-bag checks, not an independent experiment.',
    'limits': ['A constant Fs model excludes position/cogging/temperature dependence, presliding and current dynamics.',
               'Observed stationary/transition torque bands are conditional nominal-current evidence, not exact physical Fs bounds.',
               'An event can follow previous micro-slip; no global history-independent threshold is assumed.'],
    'wheels': []}
for side, name in enumerate(('left', 'right')):
    physical = PHYSICAL['wheels'][side]['integral_fit']
    p = [physical[k] for k in PHYSICAL['parameter_order']]
    series = []
    evidence = []
    for desc in PLAN['segments']:
        if desc['label'] != name+'_low_entry':
            continue
        ii, t, target, speed = sample([desc['id'], desc['id']+1], side)
        series.append((desc, ii, t, target, speed))
        evidence.append({'entry_segment_id': desc['id'], 'target_rad_s': desc['to'],
            'total_abs_position_span_rad': float(np.ptp(np.unwrap(data['q_api'][ii, side+4]))),
            'peak_abs_speed_rad_s': float(np.max(abs(speed))),
            'last_08s_mean_speed_rad_s': float(np.mean(speed[t >= t[-1]-.8])),
            'onsets': onset_candidates(ii, side, np.sign(desc['to'])),
            'presliding_tolerant_onsets': onset_candidates(ii, side, np.sign(desc['to']), strict=False)})
    chosen = []
    grids = []
    for sign in (1, -1):
        candidates = np.linspace(p[1 if sign > 0 else 2], .35, 61)
        train = [s for s in series if np.sign(s[0]['to']) == sign and abs(s[0]['to']) >= .5]
        losses = []
        for fs in candidates:
            errors = []
            for desc, ii, t, target, speed in train:
                prediction = simulate(t, target, speed[0], p, fs, fs)
                errors.append(np.mean((prediction-speed)**2))
            losses.append(float(np.mean(errors)))
        best = int(np.argmin(losses))
        chosen.append(float(candidates[best]))
        near = candidates[np.asarray(losses) <= losses[best]*1.05]
        grids.append({'direction': int(sign), 'grid_fs_nm': candidates.tolist(), 'mse_rad2_s2': losses,
                      'best_fs_nm': float(candidates[best]),
                      'grid_within_5_percent_loss_nm': [float(near.min()), float(near.max())]})
    checks = []
    tests = [(f"low_{desc['to']}", [desc['id'], desc['id']+1], abs(desc['to']) < .5)
             for desc, *_ in series]
    second_steps = [s['id'] for s in PLAN['segments'] if s['label'] == name+'_step_1']
    tests.append(('step_1', second_steps, True))
    for label, ids, heldout in tests:
        ii, t, target, speed = sample(ids, side)
        check = {'label': label, 'segment_ids': ids, 'heldout_from_fs_fit': heldout,
                 'heldout_from_previous_physical_fit': all(i not in PHYSICAL['wheels'][side]['training_segments'] for i in ids),
                 'measured_last_08s_mean_speed_rad_s': float(np.mean(speed[t >= t[-1]-.8])), 'models': {}}
        for model, fs in [('moving_coulomb_only', [p[1], p[2]]), ('constant_stick_threshold', chosen)]:
            prediction = simulate(t, target, speed[0], p, *fs)
            check['models'][model] = {'rmse_rad_s': float(np.sqrt(np.mean((prediction-speed)**2))),
                'mean_abs_error_rad_s': float(np.mean(abs(prediction-speed))),
                'last_08s_mean_speed_rad_s': float(np.mean(prediction[t >= t[-1]-.8])),
                'predicted_abs_displacement_rad': float(abs(np.sum(np.diff(t)*prediction[:-1])))}
        checks.append(check)
    result['wheels'].append({'side': name, 'fixed_parameters': physical, 'fs_positive_negative_nm': chosen,
                            'fs_fit_segment_ids': [i for s in series if abs(s[0]['to']) >= .5 for i in (s[0]['id'], s[0]['id']+1)],
                            'searches': grids, 'onset_evidence': evidence, 'checks': checks})

test_t = np.linspace(0, 1, 1001)
test_p = [.0045, .06, .07, .0004]
assert np.all(simulate(test_t, np.full_like(test_t, .1), 0, test_p, .2, .2) == 0)
assert simulate(test_t, np.ones_like(test_t), 0, test_p, .2, .2)[-1] > .5
assert simulate(test_t, -np.ones_like(test_t), 0, test_p, .2, .2)[-1] < -.5
assert all(c['heldout_from_previous_physical_fit'] for w in result['wheels'] for c in w['checks'] if c['label'] in ('low_0.2', 'low_-0.2'))
result['verification'] = {'subthreshold_stick': True, 'both_direction_release': True,
                          'low_02_not_in_any_fit': True}
(OUT/'breakaway_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
for side, wheel in enumerate(result['wheels']):
    p = [wheel['fixed_parameters'][k] for k in PHYSICAL['parameter_order']]
    for row, magnitude in enumerate((.2, .5)):
        desc = next(s for s in PLAN['segments'] if s['label'] == wheel['side']+'_low_entry' and s['to'] == magnitude)
        ii, t, target, speed = sample([desc['id'], desc['id']+1], side)
        ax = axes[row, side]
        ax.plot(t[::4], speed[::4], color='black', lw=.7, label='Measured')
        ax.plot(t, target, color='gray', lw=.8, linestyle='--', label='Target')
        ax.plot(t, simulate(t, target, speed[0], p, p[1], p[2]), lw=1, label='Moving friction only')
        ax.plot(t, simulate(t, target, speed[0], p, *wheel['fs_positive_negative_nm']), lw=1, label='Constant stick threshold')
        ax.set(title=f"{wheel['side'].title()} +{magnitude:g} rad/s ({'withheld' if row == 0 else 'fitted'})",
               xlabel='Time since entry (s)', ylabel='Wheel speed (rad/s)')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
fig.savefig(OUT/'breakaway_counterexamples.png', dpi=150)
plt.close(fig)
for wheel in result['wheels']:
    print(wheel['side'], 'Fs', wheel['fs_positive_negative_nm'])
    for check in wheel['checks']:
        print(check['label'], 'heldout', check['heldout_from_fs_fit'], check['models'])
    for e in wheel['onset_evidence']:
        print('onsets', e['target_rad_s'], e['onsets'], 'presliding', e['presliding_tolerant_onsets'])
