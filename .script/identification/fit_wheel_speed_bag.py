#!/usr/bin/env python3
"""Fit effective wheel resistance on settled platforms, then test coast dynamics.

No gains or training configuration are changed. Nm use the archived nominal
M3508 torque constant and 15.8 transmission, not a shaft torque sensor.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear, minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def resistance(speed, coefficients):
    cp, cn, b = coefficients
    return np.where(speed > 0, cp, np.where(speed < 0, -cn, 0)) + b * speed


def coast_response(time, initial_speed, inertia, coefficients):
    cp, cn, b = coefficients
    sign = np.sign(initial_speed)
    v0 = abs(initial_speed)
    fc = cp if sign > 0 else cn
    if b < 1e-10:
        speed = v0-fc*time/inertia
    else:
        z = -b*time/inertia
        speed = v0*np.exp(z)+(fc/b)*np.expm1(z)
    return sign*np.maximum(speed, 0.)


def fit_resistance(speed, torque):
    speed, torque = np.asarray(speed), np.asarray(torque)
    x = np.column_stack([speed > 0, -(speed < 0).astype(float), speed])
    if len(speed) < 6 or np.sum(speed > 0) < 2 or np.sum(speed < 0) < 2 or np.linalg.matrix_rank(x) < 3:
        return None
    result = lsq_linear(x, torque, bounds=(0., np.inf))
    residual = x@result.x-torque
    return {'coefficients':result.x.tolist(), 'rmse_nm':float(np.sqrt(np.mean(residual**2))),
            'max_abs_residual_nm':float(np.max(np.abs(residual))),
            'coefficient_order':['coulomb_positive_nm','coulomb_negative_nm','viscous_nm_per_rad_s'],
            'design_condition':float(np.linalg.cond(x)),
            'note':'Directional intercepts include any unseparated current offset / direction asymmetry.'}


def quiet_carrier(q, dq):
    return bool(np.max(np.ptp(np.unwrap(q, axis=0), axis=0)) < .03
                and np.max(np.quantile(np.abs(dq), .95, axis=0)) < .15)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('arrays',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    source=np.load(args.arrays)
    active=source['phase']==2
    d={k:source[k][active] for k in ('control_steady_ns','segment_id','wheel_velocity_target_api',
        'q_api','dq_api','tau_frame_api','torque_fb_api','temperature_c','feedback_steady_ns')}
    source.close()
    t=(d['control_steady_ns'].astype(np.int64)-int(d['control_steady_ns'][0]))*1e-9
    plan=json.loads((args.run/'trajectory-plan.json').read_text())
    args.output.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':10})
    report={'source_run':str(args.run),'status':'diagnostic_estimates_not_released_to_training',
        'assumptions':['Body supported; all DM drives disabled; carrier q/dq must remain quiet in accepted windows.',
            'Torque uses nominal M3508 motor Kt and installed 15.8 ratio, with no load-cell calibration.',
            'Airborne data do not identify tyre/ground friction or the loaded torque-speed envelope.'],
        'fitting_split':'Settled single-wheel plateaus fit resistance. Coast releases from 5/20 rad/s fit inertia; releases from 10 rad/s are reserved to check that estimate.',
        'wheels':[]}
    fig,axes=plt.subplots(3,2,figsize=(16,11),sharex='col')
    for i,name in enumerate(('左轮','右轮')):
        j=i+4
        axes[0,i].plot(t[::10],d['wheel_velocity_target_api'][::10,i],label='目标',lw=1)
        axes[0,i].plot(t[::10],d['dq_api'][::10,j],label='实测',lw=.8)
        axes[0,i].set_title(name+'：50 Hz 目标 / 1 kHz 速度 P')
        axes[0,i].set_ylabel('轮轴速度 rad/s')
        axes[1,i].plot(t[::10],d['tau_frame_api'][::10,j],label='命令电流换算',lw=.8)
        axes[1,i].plot(t[::10],d['torque_fb_api'][::10,j],label='反馈电流换算',lw=.6,alpha=.7)
        axes[1,i].set_ylabel('轮轴口径估计力矩 N·m')
        for axis in (2*i,2*i+1):
            axes[2,i].plot(t[::10],d['dq_api'][::10,axis],label='髋' if axis%2==0 else '膝侧',lw=.7)
        axes[2,i].set_ylabel('被动关节速度 rad/s');axes[2,i].set_xlabel('执行时间 s')
    for ax in axes.flat:ax.grid(alpha=.2);ax.legend(loc='upper right')
    fig.tight_layout();fig.savefig(args.output/'wheel_overview.png',dpi=160);plt.close(fig)
    friction_fig,friction_axes=plt.subplots(1,2,figsize=(13,5))
    coast_fig,coast_axes=plt.subplots(2,2,figsize=(13,8))
    for i,side in enumerate(('left','right')):
        j=i+4;points=[];coasts=[]
        for s in plan['segments']:
            if not s['label'].startswith(side+'_'):continue
            indices=np.flatnonzero(d['segment_id']==s['id'])
            if not len(indices):continue
            if s['label'].endswith('_steady'):
                indices=indices[t[indices]>=t[indices[-1]]-.8]
                w=d['dq_api'][indices,j];tt=t[indices]
                median=float(np.median(w));std=float(np.std(w))
                reason=[]
                if len(w)<400:reason.append('short window')
                if abs(median)<.08:reason.append('stationary or near stiction')
                if std>max(.06,.03*abs(median)):reason.append('wheel speed not settled')
                if not quiet_carrier(d['q_api'][indices,:4],d['dq_api'][indices,:4]):reason.append('carrier moving')
                age=(d['control_steady_ns'][indices,None].astype(np.int64)-d['feedback_steady_ns'][indices,4:6].astype(np.int64))*1e-6
                if age.max()>5:reason.append('wheel feedback gap')
                points.append({'segment_id':s['id'],'target':float(np.median(d['wheel_velocity_target_api'][indices,i])),
                    'speed':median,'speed_std':std,'command_nm':float(np.median(d['tau_frame_api'][indices,j])),
                    'reported_nm':float(np.median(d['torque_fb_api'][indices,j])),
                    'temperature_c':float(np.median(d['temperature_c'][indices,j])),
                    'accepted':not reason,'reasons':reason})
            elif s['label'].endswith('zero_current_release'):
                prior=plan['segments'][s['id']-1]
                valid=indices[(t[indices]-t[indices[0]]>=.02)&(np.abs(d['dq_api'][indices,j])>.2)]
                if len(valid)<30:continue
                # Stop at the first discontinuity or reversal, rather than
                # joining disjoint samples above the near-zero threshold.
                cut=np.flatnonzero((np.diff(valid)!=1)|(np.diff(t[valid])>.002))
                if len(cut):valid=valid[:cut[0]+1]
                if len(valid)<30:continue
                quiet=quiet_carrier(d['q_api'][valid,:4],d['dq_api'][valid,:4])
                zero=bool(np.max(np.abs(d['tau_frame_api'][valid,j]))<1e-12)
                coasts.append({'segment_id':s['id'],'runup_target':abs(prior['to']),
                    'validation':abs(abs(prior['to'])-10.)<1e-9,'accepted':quiet and zero,
                    'quiet_carrier':quiet,'zero_current':zero,'time':t[valid]-t[valid[0]],
                    'speed':d['dq_api'][valid,j]})
        accepted=[p for p in points if p['accepted']]
        speeds=[p['speed'] for p in accepted]
        fit=fit_resistance(speeds,[p['command_nm'] for p in accepted])
        feedback_fit=fit_resistance(speeds,[p['reported_nm'] for p in accepted])
        wheel={'side':side,'platforms':points,'command_based_resistance':fit,'feedback_based_resistance':feedback_fit}
        ax=friction_axes[i]
        ax.scatter([p['speed'] for p in points],[p['command_nm'] for p in points],label='平台命令',s=25)
        ax.scatter(speeds,[p['reported_nm'] for p in accepted],label='平台反馈',s=20,marker='x')
        if fit:
            coefficients=fit['coefficients'];xx=np.linspace(-30,30,500)
            ax.plot(xx,resistance(xx,coefficients),label='有效阻力拟合',lw=1)
            train=[c for c in coasts if c['accepted'] and not c['validation']]
            if len(train)>=2:
                def loss(log_j):
                    return np.mean([np.mean((coast_response(c['time'],c['speed'][0],np.exp(log_j),coefficients)-c['speed'])**2) for c in train])
                result=minimize_scalar(loss,bounds=(np.log(1e-6),np.log(.1)),method='bounded')
                inertia=float(np.exp(result.x))
                checks=[]
                for c in coasts:
                    if not c['accepted']:continue
                    prediction=coast_response(c['time'],c['speed'][0],inertia,coefficients)
                    error=float(np.sqrt(np.mean((prediction-c['speed'])**2)))
                    checks.append({'segment_id':c['segment_id'],'validation':c['validation'],
                        'initial_speed':float(c['speed'][0]),'samples':len(c['time']),'rmse_rad_s':error,
                        'relative_rmse':error/abs(float(c['speed'][0]))})
                    axc=coast_axes[1 if c['validation'] else 0,i]
                    axc.plot(c['time'],c['speed'],lw=1,label=f"实测 {c['runup_target']:g}")
                    axc.plot(c['time'],prediction,'--',lw=.8)
                validation=[c for c in checks if c['validation']]
                wheel['coast_inertia']={'effective_kg_m2':inertia,'fit_mse_rad2_s2':float(result.fun),'segments':checks,
                    'heldout_coast_pass':len(validation)>=2 and all(c['relative_rmse']<.1 for c in validation),
                    'qualification':'Coast consistency only; closed-loop trajectory replay still required before training release.'}
        wheel['coast_coverage']=[{k:v for k,v in c.items() if k not in ('time','speed')} for c in coasts]
        report['wheels'].append(wheel)
        ax.set_title(('左轮','右轮')[i]);ax.set_xlabel('实际轮轴速度 rad/s');ax.set_ylabel('估计力矩 N·m');ax.grid(alpha=.2);ax.legend()
        for row in range(2):
            axc=coast_axes[row,i];axc.set_title(('左轮','右轮')[i]+('：拟合滑行段' if row==0 else '：留出 10 rad/s 滑行段'))
            axc.set_xlabel('切零电流后窗口时间 s');axc.set_ylabel('轮速 rad/s');axc.grid(alpha=.2)
            if axc.lines:axc.legend(fontsize=8)
    friction_fig.tight_layout();friction_fig.savefig(args.output/'wheel_resistance.png',dpi=160);plt.close(friction_fig)
    coast_fig.tight_layout();coast_fig.savefig(args.output/'wheel_coast_validation.png',dpi=160);plt.close(coast_fig)
    (args.output/'wheel_fit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'wheels':[{k:v for k,v in w.items() if k not in ('platforms','coast_coverage')} for w in report['wheels']]},indent=2))


if __name__=='__main__':main()
