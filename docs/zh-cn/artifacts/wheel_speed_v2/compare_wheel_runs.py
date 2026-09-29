#!/usr/bin/env python3
"""Compare sealed wheel runs on identical trajectory segments, in wheel-axis units."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml


def load(run, arrays):
    source = np.load(arrays)
    active = source['phase'] == 2
    keys = ('control_steady_ns', 'segment_id', 'wheel_velocity_target_api',
            'dq_api', 'tau_cmd_api', 'tau_preclip_api', 'temperature_c')
    data = {key: source[key][active] for key in keys}
    source.close()
    data['time'] = (data['control_steady_ns'].astype(np.int64)
                    - int(data['control_steady_ns'][0])) * 1e-9
    plan = json.loads((run / 'trajectory-plan.json').read_text())
    config = yaml.safe_load((run / 'profile.yaml').read_text())
    return data, plan, config['wheel_leg_wheel_identification_controller']['ros__parameters']



def step_overshoot(speed, goal, delta):
    """Keep raw peaks plus a 10-sample diagnostic average (~10 ms at 1 kHz)."""
    if abs(delta) < 1e-6:
        raise ValueError("No commanded step to normalize")
    normalized = np.sign(delta) * (np.asarray(speed) - goal) / abs(delta)
    if not len(normalized) or not np.all(np.isfinite(normalized)):
        raise ValueError("Step feedback must be finite and nonempty")
    averaged = np.convolve(normalized, np.ones(10) / 10, mode="valid") if len(normalized) >= 10 else None
    return {
        "overshoot_fraction": float(max(0., np.max(normalized))),
        "overshoot_10ms_fraction": float(max(0., np.max(averaged))) if averaged is not None else None,
    }


def summarize(data, plan, params):
    result = {'kp': params['wheel_velocity_kp'], 'cap_nm': params['wheel_torque_cap'],
              'groups': {}, 'platforms': [], 'steps': []}
    for side, index in (('left', 0), ('right', 1)):
        axis = index + 4
        for name, predicate in (
            ('single_servo', lambda s: s['label'].startswith(side+'_') and not s['coast']),
            ('low_chirp', lambda s: s['label'] == side+'_low_chirp'),
            ('high_chirp', lambda s: s['label'] == side+'_high_chirp'),
            ('multisine', lambda s: s['label'] == side+'_heldout_multisine'),
            ('both', lambda s: s['label'].startswith('both_') and not s['coast']),
        ):
            ids = [s['id'] for s in plan['segments'] if predicate(s) and s['axis_scale'][index]]
            mask = np.isin(data['segment_id'], ids)
            error = data['wheel_velocity_target_api'][mask,index] - data['dq_api'][mask,axis]
            torque = data['tau_cmd_api'][mask,axis]
            result['groups'][side+'_'+name] = {
                'samples': int(mask.sum()), 'speed_rmse_rad_s': float(np.sqrt(np.mean(error**2))),
                'torque_rms_nm': float(np.sqrt(np.mean(torque**2))),
                'peak_torque_nm': float(np.max(np.abs(torque))),
                'limited_fraction': float(np.mean(np.abs(data['tau_preclip_api'][mask,axis]) > params['wheel_torque_cap'])),
                'temperature_range_c': [float(np.min(data['temperature_c'][mask,axis])), float(np.max(data['temperature_c'][mask,axis]))]}
        for segment in plan['segments']:
            if not segment['label'].startswith(side+'_'):
                continue
            idx = np.flatnonzero(data['segment_id'] == segment['id'])
            if not len(idx):
                continue
            t = data['time'][idx]
            speed = data['dq_api'][idx,axis]
            target = data['wheel_velocity_target_api'][idx,index]
            if segment['label'].endswith('_steady'):
                tail = t >= t[-1]-.8
                result['platforms'].append({'side':side, 'id':segment['id'],
                    'target':float(np.median(target[tail])), 'median_speed':float(np.median(speed[tail])),
                    'error_rmse':float(np.sqrt(np.mean((target[tail]-speed[tail])**2))),
                    'speed_std':float(np.std(speed[tail])),
                    'torque_std_nm':float(np.std(data['tau_cmd_api'][idx,axis][tail]))})
            if '_step_' in segment['label']:
                goal = segment['to']*segment['axis_scale'][index]
                prior = plan['segments'][segment['id']-1]
                previous = prior['to']*prior['axis_scale'][index]
                delta = goal-previous
                match = np.flatnonzero(np.isclose(target,goal,rtol=0,atol=1e-9))
                if abs(delta)<1e-6 or not len(match):
                    continue
                start = match[0]; w = speed[start:]; tt=t[start:]-t[start]
                band=max(.1,.05*abs(delta))
                outside=np.flatnonzero(np.abs(w-goal)>band)
                settled_index=outside[-1]+1 if len(outside) else 0
                settling=float(tt[settled_index]) if settled_index < len(tt) else None
                result['steps'].append({'side':side,'id':segment['id'],'from':float(previous),'to':float(goal),
                    **step_overshoot(w, goal, delta),
                    'settling_s':settling,'settling_band_rad_s':band,
                    'last_200ms_bias':float(np.mean(speed[t>=t[-1]-.2]-goal))})
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('baseline_run','baseline_arrays','candidate_run','candidate_arrays','output'):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    a,pa,ca=load(args.baseline_run,args.baseline_arrays)
    b,pb,cb=load(args.candidate_run,args.candidate_arrays)
    if pa != pb:
        raise SystemExit('Generated trajectory manifests differ; no paired comparison')
    for key in ('control_frequency_hz','reference_frequency_hz','wheel_reduction_ratio'):
        if ca[key] != cb[key]: raise SystemExit('Mismatched contract: '+key)
    args.output.mkdir(parents=True,exist_ok=True)
    report={'baseline':summarize(a,pa,ca),'candidate':summarize(b,pb,cb),
            'notes':['Physical run order and initial temperature are not randomized.',
                     'Reference cap changes together with gain; saturated windows are nonlinear.',
                     'Identical multisine is comparative validation, not a new post-selection acceptance run.',
                     'Step settling requires all remaining samples within max(0.1 rad/s, 5% command change).',
                     'Command change uses the preceding planned endpoint; ramp-to-zero quantization is not a step.',
                     '10 ms overshoot uses a 10-sample moving mean at 1 kHz for diagnostics only; control and raw peaks remain unchanged.'],
            'source_runs':[str(args.baseline_run),str(args.candidate_run)]}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False})
    fig,axes=plt.subplots(3,2,figsize=(14,10))
    for col,side in enumerate(('left','right')):
        for row,suffix in enumerate(('low_steady','step_0','high_chirp')):
            choices=[s for s in pa['segments'] if s['label']==side+'_'+suffix]
            s=next((s for s in choices if s['to']==.5),choices[0]) if row==0 else next(s for s in choices if s["to"]==2.) if row==1 else choices[1]
            for data,params in ((a,ca),(b,cb)):
                idx=np.flatnonzero(data['segment_id']==s['id'])
                x=data['time'][idx]-data['time'][idx[0]]
                ax=axes[row,col]
                ax.plot(x,data['dq_api'][idx,col+4],label=f"实测 Kp={params['wheel_velocity_kp']}",lw=.9)
            ax.plot(x,b['wheel_velocity_target_api'][idx,col],label='目标',color='black',ls='--',lw=1)
            ax.set_title(('左轮','右轮')[col]+' · '+('0.5 rad/s 平台','首个阶跃','2–8 Hz 带偏置扫频')[row])
            ax.set_xlabel('段内时间 s');ax.set_ylabel('轮轴速度 rad/s');ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(args.output/'tracking_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({r:report[r]['groups'] for r in ('baseline','candidate')},indent=2))


if __name__=='__main__':main()
