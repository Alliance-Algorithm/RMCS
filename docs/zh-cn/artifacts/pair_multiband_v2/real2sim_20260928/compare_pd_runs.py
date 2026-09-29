"""Compare two measured PD runs with the same side and reference contract."""
from pathlib import Path
import argparse
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


def read(run):
    raw = dict(np.load(run/'response_1khz.npz'))
    a = {k:v[raw['phase']==2] for k,v in raw.items()}
    meta = json.loads((run/'response_1khz.json').read_text())
    a['q_model'] = a['q_ref_model']-a['position_error_model']
    return a,meta


def metrics(a, sid=None):
    ix = np.arange(len(a['segment_id'])) if sid is None else np.flatnonzero(a['segment_id']==sid)
    e = a['position_error_model'][ix]
    u = a['tau_cmd_api'][ix]
    q = a['q_model'][ix]
    centered=e-e.mean(axis=0)
    result = {
        'segment':None if sid is None else int(sid),'samples':len(ix),
        'axis_rmse_rad':np.sqrt(np.mean(e**2,axis=0)).tolist(),
        'axis_bias_rad':np.mean(e,axis=0).tolist(),
        'axis_centered_rmse_rad':np.sqrt(np.mean(centered**2,axis=0)).tolist(),
        'common_rmse_rad':float(np.sqrt(np.mean(e.mean(axis=1)**2))),
        'difference_rmse_rad':float(np.sqrt(np.mean(np.diff(e,axis=1)**2))),
        'common_centered_rmse_rad':float(np.sqrt(np.mean(centered.mean(axis=1)**2))),
        'difference_centered_rmse_rad':float(np.sqrt(np.mean(np.diff(centered,axis=1)**2))),
        'measured_difference_ptp_rad':float(np.ptp(q[:,1]-q[:,0])),
        'reference_difference_ptp_rad':float(np.ptp(a['q_ref_model'][ix,1]-a['q_ref_model'][ix,0])),
        'torque_peak_nm':np.max(abs(u),axis=0).tolist(),
        'torque_rms_nm':np.sqrt(np.mean(u**2,axis=0)).tolist(),
        'torque_limited_fraction':np.mean(a['torque_limited'][ix],axis=0).tolist(),
    }
    if len(ix) > 1000:
        # Diagnostic energy in reported velocity, including target-hold
        # harmonics and sensor effects; not a mechanical instability verdict.
        sos = signal.butter(4,20,fs=1000,btype='high',output='sos')
        for field, label in (('dq_api','reported_velocity_highpass_20hz_rms_rad_s'),
                             ('tau_cmd_api','torque_highpass_20hz_rms_nm')):
            filtered=signal.sosfiltfilt(sos,a[field][ix],axis=0)[300:-300]
            result[label]=np.sqrt(np.mean(filtered**2,axis=0)).tolist()
    return result


def step_response(a, sid, coordinate):
    ix=np.flatnonzero(np.isin(a['segment_id'],[sid,sid+1]))
    hold=np.flatnonzero(a['segment_id']==sid+1)
    before=np.arange(max(0,ix[0]-300),ix[0])
    project=(lambda x:x.mean(axis=1)) if coordinate=='common' else (lambda x:x[:,1]-x[:,0])
    actual=project(a['q_model']);reference=project(a['q_ref_model'])
    q0=float(np.mean(actual[before]));r0=float(np.mean(reference[before]))
    qss=float(np.mean(actual[hold[-300:]]));rss=float(np.mean(reference[hold[-300:]]))
    amplitude=rss-r0;direction=np.sign(amplitude)
    tolerance=max(.02*abs(amplitude),.0015)
    outside=np.flatnonzero(abs(actual[hold]-qss)>tolerance)
    last_bad=int(outside[-1]) if len(outside) else -1
    settle=(float((a['control_steady_ns'][hold[last_bad+1]]-a['control_steady_ns'][hold[0]])*1e-9)
            if last_bad+1<len(hold) else None)
    return {'first_segment':sid,'coordinate':coordinate,'reference_change_rad':amplitude,
            'measured_final_change_rad':qss-q0,'steady_target_error_rad':rss-qss,
            'response_ratio':float((qss-q0)/amplitude) if abs(amplitude)>1e-9 else None,
            'overshoot_beyond_measured_final_rad':float(max(0.,np.max(direction*(actual[ix]-qss)))),
            'settle_from_hold_start_s':settle,'settle_band_halfwidth_rad':tolerance,
            'settle_definition':'Remain within 2% of requested change (minimum 0.0015 rad) around the final measured mean; not around the target. Final mean uses last 0.3 s of hold.'}


def main():
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    a,ma=read(args.baseline);b,mb=read(args.candidate)
    ca,cb=ma['controller'],mb['controller']
    changed={k:[ca.get(k),cb.get(k)] for k in set(ca)|set(cb) if ca.get(k)!=cb.get(k)}
    allowed={'pd_kp','pd_kd'}
    if set(changed)-allowed:
        raise ValueError(f'Confounded controller/trajectory change: {changed}')
    segments=np.unique(a['segment_id'])
    if not np.array_equal(segments,np.unique(b['segment_id'])):
        raise ValueError('Segment coverage differs; do not call this a complete same-plan A/B.')
    rows=[];shape_warnings=[];alignment_diagnostics=[]
    for sid in segments:
        ia=np.flatnonzero(a['segment_id']==sid);ib=np.flatnonzero(b['segment_id']==sid)
        ta=(a['control_steady_ns'][ia]-a['control_steady_ns'][ia[0]])*1e-9
        tb=(b['control_steady_ns'][ib]-b['control_steady_ns'][ib[0]])*1e-9
        ar=a['q_ref_model'][ia];br=b['q_ref_model'][ib]
        aligned=np.column_stack([np.interp(ta,tb,br[:,j]-br[0,j]) for j in (0,1)])
        mismatch=float(np.max(abs(aligned-(ar-ar[0]))))
        # Compare actual held target events, too: interpolation across a ZOH
        # edge can manufacture a mismatch from sub-millisecond sample jitter.
        ua=np.r_[0,np.flatnonzero(np.any(np.diff(ar,axis=0)!=0,axis=1))+1]
        ub=np.r_[0,np.flatnonzero(np.any(np.diff(br,axis=0)!=0,axis=1))+1]
        event_mismatch=(float(np.max(abs((ar[ua]-ar[0])-(br[ub]-br[0]))))
                        if len(ua)==len(ub) else None)
        detail={'segment':int(sid),'interpolated_reference_max_difference_rad':mismatch,
                'held_reference_events':[len(ua),len(ub)],
                'held_reference_event_max_difference_rad':event_mismatch,
                'segment_duration_difference_s':float(tb[-1]-ta[-1])}
        if (event_mismatch is None or event_mismatch>1e-8
                or abs(ta[-1]-tb[-1])>.01):
            shape_warnings.append(detail)
        if mismatch>.005:
            alignment_diagnostics.append(detail)
        rows.append({'baseline':metrics(a,sid),'candidate':metrics(b,sid),
                     'reference_shape_max_difference_rad':mismatch})
    args.output.mkdir(parents=True,exist_ok=True)
    step_cases=[(sid,coordinate) for offset in (0,78)
                for sid,coordinate in ((21+offset,'common'),(25+offset,'common'),
                                       (29+offset,'difference'),(33+offset,'difference'))]
    report={'baseline':str(args.baseline),'candidate':str(args.candidate),'changed':changed,
            'reference_shape_warnings':shape_warnings,
            'reference_alignment_diagnostics':alignment_diagnostics,
            'initial_active_angle_difference_rad':(b['q_model'][0]-a['q_model'][0]).tolist(),
            'overall':{'baseline':metrics(a),'candidate':metrics(b)},
            'recording_checks':{label:json.loads((run/'recording_summary.json').read_text())
                                for label,run in (('baseline',args.baseline),('candidate',args.candidate))},
            'segments':rows,
            'rounded_step_response':[
                {'baseline':step_response(a,sid,coord),'candidate':step_response(b,sid,coord)}
                for sid,coord in step_cases],
            'comparison_limitations':[
                'Same configured trajectory; initial measured pose differs and changes gravity loading.',
                'Held target event sequences are compared separately from interpolated ZOH edge discrepancies.',
                'Highpass velocity energy includes feedback and 50 Hz target-hold effects, not only mechanical vibration.'
            ],
            'automatic_promotion':False,
            'decision_rule':'Inspect measured tracking, residual oscillation and torque occupancy together. A larger motion amplitude alone is insufficient.'}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,
                         'axes.spines.top':False,'axes.spines.right':False})
    fig,ax=plt.subplots(3,2,figsize=(13,9),constrained_layout=True)
    labels=[f"旧 PD {ca['pd_kp'][0]:g}/{ca['pd_kd'][0]:g}",
            f"新 PD {cb['pd_kp'][0]:g}/{cb['pd_kd'][0]:g}"]
    for col,sid in enumerate((9,11)):
        for run,label,color in zip((a,b),labels,('#2363b1','#df682a')):
            ix=np.flatnonzero(run['segment_id']==sid)[::5]
            t=(run['control_steady_ns'][ix]-run['control_steady_ns'][ix[0]])*1e-9
            e=run['position_error_model'][ix]
            ax[0,col].plot(t,np.degrees(e.mean(axis=1)),label=label,color=color,lw=1)
            ax[1,col].plot(t,np.degrees(e[:,1]-e[:,0]),label=label,color=color,lw=1)
            ax[2,col].plot(t,np.max(abs(run['tau_cmd_api'][ix]),axis=1),label=label,color=color,lw=1)
        ax[0,col].set_title('整体摆动段' if col==0 else '相对运动段')
        ax[0,col].set_ylabel('平均角误差（°）');ax[1,col].set_ylabel('电机角差误差（°）')
        ax[2,col].set(ylabel='两轴请求力矩绝对值较大者（N·m）',xlabel='段内时间（s）')
    for panel in ax.flat:panel.grid(alpha=.18);panel.legend(fontsize=9)
    pose_delta_deg=float(np.degrees(np.mean(b['q_model'][0]-a['q_model'][0])))
    fig.suptitle(f'相同轨迹方案的 PD 实机对照｜初始平均角相差 {abs(pose_delta_deg):.2f}°',fontsize=15)
    fig.savefig(args.output/'measured_pd_comparison.png',dpi=150)
    fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
    for panel,(sid,coordinate) in zip(axes.flat,step_cases[:4]):
        for j,(run,label,color) in enumerate(zip((a,b),labels,('#2363b1','#df682a'))):
            ix=np.flatnonzero(np.isin(run['segment_id'],np.arange(sid,sid+4)))
            before=np.arange(ix[0]-300,ix[0]);ix=np.r_[before,ix]
            t=(run['control_steady_ns'][ix].astype(np.int64)-int(run['control_steady_ns'][ix[300]]))*1e-9
            project=(lambda x:x.mean(axis=1)) if coordinate=='common' else (lambda x:x[:,1]-x[:,0])
            ref=project(run['q_ref_model'][ix]);q=project(run['q_model'][ix]);origin=float(ref[:300].mean())
            if j==0:panel.plot(t[::2],np.degrees(ref[::2]-origin),color='#303945',ls='--',lw=1,label='目标')
            panel.plot(t[::2],np.degrees(q[::2]-origin),color=color,lw=1,label=label)
        panel.set(title=('平均角' if coordinate=='common' else '电机角差')+('正向' if sid in (21,29) else '反向')+'阶跃',
                  xlabel='距阶跃开始（s）',ylabel='相对阶跃前目标（°）')
        panel.grid(alpha=.18);panel.legend(fontsize=9)
    fig.suptitle('0.22 s 平滑阶跃及保持｜保留静态跟踪偏差',fontsize=15)
    fig.savefig(args.output/'step_response_comparison.png',dpi=150)
    print(json.dumps({'changed':changed,'reference_shape_warnings':shape_warnings},ensure_ascii=False))


if __name__=='__main__':main()
