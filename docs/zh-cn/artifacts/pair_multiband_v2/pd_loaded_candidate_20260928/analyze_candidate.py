"""Analyze the measured 60/2 PD bag and screen a stronger candidate offline.

The new gains are a control experiment, not an identified physical parameter set.
The old high-frequency ensemble is only a sensitivity screen. Shadow efforts use
frozen measured states and cannot predict motion with the new gains.
"""
from pathlib import Path
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
CAL = ROOT.parent / 'closed_loop_tuning_20260928'
RUN = ROOT.parent / 'run_20260928T050017Z'
namespace = {'__file__': str(CAL / 'screen_rl_pd_1khz.py')}
source = (CAL / 'screen_rl_pd_1khz.py').read_text().split('results=[]', 1)[0]
exec(compile(source, str(CAL / 'screen_rl_pd_1khz.py'), 'exec'), namespace)
transition = namespace['transition']
models = namespace['models']
screen = []
for kp, kd in [(60, 2), (80, 2.5), (100, 2.5), (120, 3), (140, 3), (160, 3.5), (200, 4)]:
    poles = [transition(m, kp, kd, scale, delay) for m in models
             for scale in (.95, 1, 1.05) for delay in (0, 1)]
    screen.append(dict(kp=kp, kd=kd, nominal_max_real_pole_per_s=transition(models[0], kp, kd),
                       worst_max_real_pole_per_s=max(poles)))

raw = dict(np.load(RUN / 'response_1khz.npz'))
a = {k: v[raw['phase'] == 2] for k, v in raw.items()}
q = a['q_ref_model'] - a['position_error_model']
d = q[:, 1] - q[:, 0]
dr = a['q_ref_model'][:, 1] - a['q_ref_model'][:, 0]
error, velocity = a['position_error_model'], a['dq_api']
u = a['tau_cmd_api']
shadow = 120 * error - 3 * velocity
segments = {}
for segment in [1, 3, 71, 81, 109, 111]:
    ix = np.flatnonzero(a['segment_id'] == segment)
    # Exclude onset and tail when reporting excitation inside a segment.
    ix = ix[1000:-500]
    segments[str(segment)] = dict(
        measured_difference_ptp_rad=float(np.ptp(d[ix])),
        reference_difference_ptp_rad=float(np.ptp(dr[ix])),
        tracking_rmse_rad=np.sqrt(np.mean(error[ix] ** 2, axis=0)).tolist(),
        command_peak_nm=np.max(np.abs(u[ix]), axis=0).tolist())
report = dict(
    source_mcap_sha256='a60ae06cb0a4178068bdcf563038074157fb1268cf8738b91c9908784838f4c8',
    gain_candidate=dict(kp=[120, 120], kd=[3, 3], axes=['hip', 'auxiliary knee drive'],
                        control_hz=1000, reference_hz=50, torque_cap_nm=40,
                        status='candidate_for_on_rig_validation_not_proven_better'),
    rationale='Measured differential motion is weak; increase restoring gain and paired reference loading with the installed springs retained.',
    measured_baseline=dict(running_samples=len(q),
        algebra_max_error_nm=float(np.max(np.abs(60 * error - 2 * velocity - a['tau_preclip_model']))),
        tracking_rmse_rad=np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
        command_rms_nm=np.sqrt(np.mean(u ** 2, axis=0)).tolist(),
        command_peak_nm=np.max(np.abs(u), axis=0).tolist(),
        saturation_fraction=np.mean(a['torque_limited'], axis=0).tolist(),
        measured_difference_range_rad=[float(min(d)), float(max(d))], segments=segments),
    shadow_effort=dict(peak_nm=np.max(np.abs(shadow), axis=0).tolist(),
                       fraction_over_40nm=np.mean(np.abs(shadow) > 40, axis=0).tolist(),
                       scope='Frozen baseline q/dq and reference, not a forward simulation or a prediction for the larger new trajectory'),
    local_sensitivity=dict(results=screen, scope='Old P+PI bag 5-90 Hz empirical ensemble, not validated full-chain dynamics; stable linear poles do not guarantee rig stability.'),
    identifiability='Gas spring curve remains fixed. Weak baseline motor-difference motion does not identify independent differential inertia, friction, spring load or motor response uniquely.',
    reference_only_loading=dict(offset_rad=.2, stages=4,
        stationary_no_motion_position_term_nm_per_axis=[3, 6, 9, 12],
        explanation='Kp120 * (motor-difference change / 2), ignoring gravity and actual movement; these are not imposed constant torque commands.'),
    profile_sha256=hashlib.sha256((ROOT / 'profile.yaml').read_bytes()).hexdigest(),
)
(ROOT / 'candidate_evidence.json').write_text(json.dumps(report, indent=2) + '\n')
fig, ax = plt.subplots(2, 2, figsize=(13, 7.5), constrained_layout=True)
for col, seg in enumerate([3, 109]):
    ix = np.flatnonzero(a['segment_id'] == seg)[::10]
    t = (a['control_steady_ns'][ix] - a['control_steady_ns'][ix[0]]) / 1e9
    ax[0, col].plot(t, dr[ix], label='Reference difference', lw=1.5)
    ax[0, col].plot(t, d[ix], label='Measured difference', lw=1.2)
    ax[0, col].set(title=f'60/2 baseline: differential chirp, segment {seg}', ylabel='Auxiliary - hip (rad)')
    ax[1, col].plot(t, u[ix, 0], label='Hip request', lw=1)
    ax[1, col].plot(t, u[ix, 1], label='Auxiliary knee request', lw=1)
    ax[1, col].set(xlabel='Time in segment (s)', ylabel='Requested torque (Nm)')
for item in ax.flat:
    item.grid(alpha=.2)
    item.legend(fontsize=8)
fig.suptitle('Installed gas springs retained: baseline differential excitation is insufficient')
fig.savefig(ROOT / 'baseline_differential_response.png', dpi=150)
print(json.dumps({k: report[k] for k in ('measured_baseline', 'shadow_effort', 'local_sensitivity')}, indent=2))
