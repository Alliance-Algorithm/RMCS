#!/usr/bin/env python3
"""Retrospective CPU-only wheel identification audit; writes no runtime configuration."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent
RUNS = {'p02': 'run_20260928T093805Z', 'p06': 'run_20260928T100509Z',
        'p04': 'run_20260928T140705Z'}
GAINS = {'p02': .2, 'p06': .6, 'p04': .4}
NAMES = ['inertia_kg_m2', 'coulomb_positive_nm', 'coulomb_negative_nm', 'viscous_nm_per_rad_s']


def fit_positive(x, y):
    scale = np.sqrt(np.mean(x*x, axis=0))
    normalized = x / np.maximum(scale, 1e-12)
    result = lsq_linear(normalized, y, bounds=(0, np.inf), tol=1e-12)
    return result.x / scale, float(np.linalg.cond(normalized))


def quiet(q, dq):
    return (np.max(np.ptp(np.unwrap(q, axis=0), axis=0)) < .03
            and np.max(np.quantile(np.abs(dq), .95, axis=0)) < .15)


def coast_prediction(t, initial, pars):
    inertia, cp, cn, b = pars
    sign = np.sign(initial)
    fc = cp if sign > 0 else cn
    if b < 1e-10:
        speed = abs(initial) - fc * t / inertia
    else:
        z = -b * t / inertia
        speed = abs(initial) * np.exp(z) + (fc / b) * np.expm1(z)
    return sign * np.maximum(speed, 0)


def replay(t, target, torque, initial, pars, kp, closed_loop):
    inertia, cp, cn, b = pars
    prediction = np.empty_like(t)
    prediction[0] = initial
    for k in range(len(t)-1):
        w = prediction[k]
        command = np.clip(kp*(target[k]-w), -4.5, 4.5) if closed_loop else torque[k]
        friction = cp if w > 0 else -cn if w < 0 else np.clip(command, -cn, cp)
        prediction[k+1] = w + (t[k+1]-t[k])*(command-friction-b*w)/inertia
    return prediction


data = {}
provenance = {}
for key, name in RUNS.items():
    run = ROOT / name
    old = json.loads((run/'fit/wheel_fit.json').read_text())
    plan_path = run/'trajectory-plan.json'
    if not plan_path.exists():
        plan_path = Path(old['source_run'])/'trajectory-plan.json'
    plan = json.loads(plan_path.read_text())
    with np.load(run/'wheel_1khz.npz') as source:
        active = source['phase'] == 2
        arrays = {k: source[k][active] for k in ('control_steady_ns', 'segment_id', 'q_api', 'dq_api',
                  'tau_frame_api', 'torque_fb_api', 'wheel_velocity_target_api', 'feedback_steady_ns')}
        raw = source['feedback_frame_bytes'][active, 32:48].reshape(-1, 2, 8).astype(np.int32)
    rpm = raw[:, :, 2]*256 + raw[:, :, 3]
    rpm = np.where(rpm >= 32768, rpm-65536, rpm)
    current = raw[:, :, 4]*256 + raw[:, :, 5]
    current = np.where(current >= 32768, current-65536, current)
    speed_error = abs(-rpm/15.8*2*np.pi/60 - arrays['dq_api'][:, 4:6])
    torque_error = abs(-current/16384*20*15.8*(.3*187/3591) - arrays['torque_fb_api'][:, 4:6])
    arrays['feedback_speed_consistent'] = speed_error < 1e-9
    arrays['t'] = (arrays['control_steady_ns'].astype(np.int64)-int(arrays['control_steady_ns'][0]))*1e-9
    quality = json.loads((run/'wheel_quality.json').read_text())
    provenance[key] = {'run': name, 'kp_nm_per_rad_s': GAINS[key], 'plan': str(plan_path),
        'npz_sha256': hashlib.sha256((run/'wheel_1khz.npz').read_bytes()).hexdigest(),
        'quality_status': quality['status'], 'quality_reasons': quality['reasons'],
        'raw_feedback_unit_check': {
            'speed_max_error_rad_s': speed_error.max(axis=0).tolist(),
            'torque_max_error_nm': torque_error.max(axis=0).tolist(),
            'speed_mismatch_samples': (speed_error >= 1e-9).sum(axis=0).tolist(),
            'torque_mismatch_samples': (torque_error >= 1e-9).sum(axis=0).tolist(),
            'note': 'Rare row/frame discrepancies flag alignment uncertainty; this is not evidence for converting the gear ratio again.'}}
    data[key] = (arrays, plan, old)

result = {'status': 'retrospective_diagnostic_not_released_to_training', 'provenance': provenance,
    'units': 'wheel output rad/s and nominal output Nm; no additional 15.8 conversion',
    'model': 'J*dw/dt = applied_tau - cp*I(w>0) + cn*I(w<0) - b*w',
    'parameter_order': NAMES, 'fixed_controller': 'recorded P gains; Ki=Kd=0; command cap 4.5 Nm',
    'split': {'fit': 'p06 non-validation non-coast single-wheel segments, 50 ms non-overlapping integral windows',
              'same_bag_check': 'p06 predesignated heldout multisine plus all zero-current coasts excluded from integral fit; not an independent experiment',
              'baseline_split_difference': 'existing_p06_platform_coast already fitted the p06 5/20 rad/s coasts; its 10 rad/s coasts and multisine remain excluded from its fit',
              'cross_run_check': 'p02/p04 previously recorded same-trajectory different-gain bags, retrospectively reused; p04 is incomplete',
              'no_new_blind_experiment': True}, 'wheels': []}

for side, name in enumerate(('left', 'right')):
    axis = side+4
    xrows, yrows, training = [], [], []
    d, plan, old = data['p06']
    for desc in plan['segments']:
        if desc['validation'] or desc['coast'] or not desc['label'].startswith(name+'_'):
            continue
        ids = np.flatnonzero(d['segment_id'] == desc['id'])
        for start in range(0, len(ids)-50, 50):
            ii = ids[start:start+51]
            t = d['t'][ii]
            w = d['dq_api'][ii, axis]
            if np.max(np.diff(t)) > .002 or np.min(np.diff(t)) <= 0:
                continue
            if np.min(np.abs(w)) < 1 or np.min(w)*np.max(w) <= 0:
                continue
            if not np.all(d['feedback_speed_consistent'][ii, side]):
                continue
            if not quiet(d['q_api'][ii, :4], d['dq_api'][ii, :4]):
                continue
            age = (d['control_steady_ns'][ii].astype(np.int64)-d['feedback_steady_ns'][ii, axis].astype(np.int64))*1e-9
            if age.min() < 0 or age.max() > .005:
                continue
            dt = np.diff(t)
            # Recorder values are held between ticks; avoid differentiating noisy velocity.
            xrows.append([w[-1]-w[0], np.sum(dt*(w[:-1] > 0)),
                          -np.sum(dt*(w[:-1] < 0)), np.sum(dt*w[:-1])])
            yrows.append(np.sum(dt*d['tau_frame_api'][ii[:-1], axis]))
            training.append(desc['id'])
    x, y = np.asarray(xrows), np.asarray(yrows)
    assert not any(plan['segments'][i]['validation'] or plan['segments'][i]['coast'] for i in training)
    fitted, condition = fit_positive(x, y)
    old_wheel = old['wheels'][side]
    baseline = np.array([old_wheel['coast_inertia']['effective_kg_m2'],
                         *old_wheel['command_based_resistance']['coefficients']])
    models = {'existing_p06_platform_coast': baseline, 'integral_fit_p06': fitted}
    wheel = {'side': name, 'integral_fit': dict(zip(NAMES, fitted.tolist())),
             'integral_windows': len(y), 'training_segments': sorted(set(training)),
             'scaled_design_condition': condition,
             'fit_integrated_residual_rmse_nm_s': float(np.sqrt(np.mean((x@fitted-y)**2))),
             'checks': [], 'existing_resistance_sensitivity': [], 'stationary_low_speed_evidence': []}
    for key, (d, plan, old) in data.items():
        points = [p for p in old['wheels'][side]['platforms'] if p['accepted']]
        estimates = []
        for omit in (10, 20, 30):
            pp = [p for p in points if abs(abs(p['target'])-omit) > 1e-6]
            w = np.array([p['speed'] for p in pp])
            xx = np.column_stack([w > 0, -(w < 0).astype(float), w])
            est, _ = fit_positive(xx, np.array([p['command_nm'] for p in pp]))
            estimates.append({'omitted_speed_magnitude': omit, 'coefficients': est.tolist()})
        wheel['existing_resistance_sensitivity'].append({'run': key,
            'command_based': old['wheels'][side]['command_based_resistance']['coefficients'],
            'feedback_based': old['wheels'][side]['feedback_based_resistance']['coefficients'],
            'leave_one_speed_pair_out': estimates})
        for desc in plan['segments']:
            if not desc['label'].startswith(name+'_'):
                continue
            ii = np.flatnonzero(d['segment_id'] == desc['id'])
            if not len(ii):
                continue
            t = d['t'][ii]
            w = d['dq_api'][ii, axis]
            if desc['label'].endswith('_low_steady') and abs(desc['to']) <= .5:
                local = ii[t >= t[-1]-.8]
                speed = d['dq_api'][local, axis]
                wheel['stationary_low_speed_evidence'].append({'run': key, 'segment_id': desc['id'],
                    'target_rad_s': desc['to'], 'median_speed_rad_s': float(np.median(speed)),
                    'fraction_abs_speed_below_002': float(np.mean(np.abs(speed) < .02)),
                    'position_range_rad': float(np.ptp(np.unwrap(d['q_api'][local, axis]))),
                    'median_command_nm': float(np.median(d['tau_frame_api'][local, axis])),
                    'median_reported_nm': float(np.median(d['torque_fb_api'][local, axis]))})
            coast = desc['label'].endswith('zero_current_release')
            heldout = desc['validation'] and desc['label'].endswith('heldout_multisine')
            if not coast and not heldout:
                continue
            if coast:
                selected = np.flatnonzero((t-t[0] >= .02) & (abs(w) > .2))
                if not len(selected):
                    continue
                cut = np.flatnonzero((np.diff(selected) != 1) | (np.diff(t[selected]) > .002))
                if len(cut):
                    selected = selected[:cut[0]+1]
                ii = ii[selected]
                if len(ii) < 30:
                    continue
                if np.max(abs(d['tau_frame_api'][ii, axis])) > 1e-12:
                    continue
            t = d['t'][ii]-d['t'][ii[0]]
            w = d['dq_api'][ii, axis]
            carrier = quiet(d['q_api'][ii, :4], d['dq_api'][ii, :4])
            continuous = bool(np.min(np.diff(t)) > 0 and np.max(np.diff(t)) <= .002)
            if not carrier or not continuous:
                wheel['checks'].append({'run': key, 'segment_id': desc['id'], 'excluded': True,
                                        'quiet_carrier': carrier, 'continuous': continuous})
                continue
            check = {'run': key, 'segment_id': desc['id'], 'kind': 'coast' if coast else 'heldout_multisine',
                     'duration_s': float(t[-1]), 'initial_speed_rad_s': float(w[0]),
                     'raw_frame_speed_mismatch_samples': int(np.sum(~d['feedback_speed_consistent'][ii, side])),
                     'models': {}}
            for model, pars in models.items():
                if coast:
                    prediction = coast_prediction(t, w[0], pars)
                    error = float(np.sqrt(np.mean((prediction-w)**2)))
                    check['models'][model] = {'rmse_rad_s': error, 'relative_initial_rmse': error/abs(w[0])}
                else:
                    metrics = {}
                    for closed_loop in (False, True):
                        prediction = replay(t, d['wheel_velocity_target_api'][ii, side],
                                            d['tau_frame_api'][ii, axis], w[0], pars, GAINS[key], closed_loop)
                        prefix = 'closed_loop_known_p' if closed_loop else 'recorded_torque_open_loop'
                        metrics[prefix+'_rmse_rad_s'] = float(np.sqrt(np.mean((prediction-w)**2)))
                    check['models'][model] = metrics
            wheel['checks'].append(check)
    result['wheels'].append(wheel)

rng = np.random.default_rng(2836)
synthetic_speed = np.r_[rng.uniform(1, 30, 100), -rng.uniform(1, 30, 100)]
synthetic_x = np.column_stack([rng.uniform(-1, 1, 200), .05*(synthetic_speed > 0),
                              -.05*(synthetic_speed < 0), .05*synthetic_speed])
known = np.array([.0045, .06, .07, .0004])
recovered, _ = fit_positive(synthetic_x, synthetic_x@known)
assert np.allclose(recovered, known, atol=1e-10, rtol=1e-10)
for initial in (-10., 10.):
    predicted = coast_prediction(np.linspace(0, 2, 101), initial, known)
    assert np.all(np.diff(abs(predicted)) <= 0) and predicted[0] == initial
result['verification'] = {'known_parameter_linear_recovery': True, 'coast_monotonicity_both_directions': True,
                          'no_validation_or_coast_in_integral_training': True}
(OUT/'offline_fit_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
for wheel in result['wheels']:
    print(wheel['side'], 'integral_fit', wheel['integral_fit'], 'windows', wheel['integral_windows'])
    for key in RUNS:
        for kind in ('coast', 'heldout_multisine'):
            checks = [c for c in wheel['checks'] if c['run'] == key and c.get('kind') == kind]
            print(key, kind, 'n', len(checks), {
                model: {metric: float(np.mean([c['models'][model][metric] for c in checks]))
                        for metric in checks[0]['models'][model]} for model in models} if checks else {})
