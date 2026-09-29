"""Measured PD curves and a frozen, reduced common-mode forward validation.

The forward model is empirical, not a CAD closed-chain model. Measured state is
used only at each segment's initial condition; subsequent feedback is simulated.
No model or gain is selected using the previously reserved validation segments.
"""
from pathlib import Path
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent
plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False,
                     'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})
raw = dict(np.load(RUN / 'response_1khz.npz'))
a = {k: v[raw['phase'] == 2] for k, v in raw.items()}
qref = a['q_ref_model']
q = qref - a['position_error_model']
c, cr = q.mean(axis=1), qref.mean(axis=1)
d, dr = q[:, 1] - q[:, 0], qref[:, 1] - qref[:, 0]
clock = a['control_steady_ns']
t = (clock - clock[0]) * 1e-9
blue, orange = '#2363b1', '#db602b'

fig, ax = plt.subplots(2, 2, figsize=(13, 7.4), constrained_layout=True)
for col, segment in enumerate([1, 109]):
    ix = np.flatnonzero(a['segment_id'] == segment)[::10]
    ts = (clock[ix] - clock[ix[0]]) * 1e-9
    actual, ref = (c, cr) if col == 0 else (d, dr)
    zero = actual[ix[0]]
    ax[0, col].plot(ts, np.rad2deg(ref[ix] - zero), color=blue, label='目标', lw=1.6)
    ax[0, col].plot(ts, np.rad2deg(actual[ix] - zero), color=orange, label='实测', lw=1.5)
    ax[0, col].set_title('整体摆动：能跟随，仍有误差' if col == 0 else '髋膝相对运动：目标改变，实测变化很小', fontsize=13)
    ax[0, col].set_ylabel('两电机平均角度变化（°）' if col == 0 else '两电机角度差变化（°）')
    ax[1, col].plot(ts, a['tau_cmd_api'][ix, 0], color=blue, label='髋电机请求', lw=1.2)
    ax[1, col].plot(ts, a['tau_cmd_api'][ix, 1], color=orange, label='辅助膝电机请求', lw=1.2)
    ax[1, col].set(ylabel='请求力矩（N·m）', xlabel='段内时间（s）')
    ax[1, col].text(.02, .03, f'原始 bag 段号 {segment}；未对曲线平滑', transform=ax[1, col].transAxes,
                    fontsize=9, color='#555555', bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .8})
for panel in ax.flat:
    panel.legend(loc='upper right', fontsize=10, framealpha=.85)
    panel.grid(alpha=.17)
fig.suptitle('上次左腿 PD 实测：Kp=60，Kd=2｜目标 50 Hz，PD 1000 Hz\n车体固定，连杆、轮组、气弹簧及重力均保留', fontsize=15)
fig.savefig(ROOT / 'measured_tracking_and_torque.png', dpi=160)

# Both active motor axes, not an inferred physical knee angle.
fig, ax = plt.subplots(2, 2, figsize=(13, 7.2), constrained_layout=True)
for col, segment in enumerate([1, 109]):
    ix = np.flatnonzero(a['segment_id'] == segment)[::10]
    ts = (clock[ix] - clock[ix[0]]) * 1e-9
    for j in (0, 1):
        offset = q[ix[0], j]
        ax[j, col].plot(ts, np.rad2deg(qref[ix, j] - offset), color=blue, label='目标', lw=1.5)
        ax[j, col].plot(ts, np.rad2deg(q[ix, j] - offset), color=orange, label='实测', lw=1.4)
        ax[j, col].set_ylabel(('髋电机' if j == 0 else '辅助膝电机') + '角度变化（°）')
        ax[j, col].grid(alpha=.17)
        ax[j, col].legend(loc='upper right', fontsize=10)
    ax[0, col].set_title(f'段 {segment}：' + ('整体摆动扫频' if col == 0 else '相对运动扫频'))
    ax[1, col].set_xlabel('段内时间（s）')
fig.suptitle('两主动轴各自的目标与反馈｜各子图以该轴段首实测角为零', fontsize=15)
fig.savefig(ROOT / 'both_motor_tracking.png', dpi=160)

# Freeze the already fitted empirical model using development error only.
fit = json.loads((RUN / 'common_mode_inverse_fit.json').read_text())
selected = min(fit['models'], key=lambda model: model['rmse']['development'])
M, B, F, Gs, Gc, bias = selected['parameters']
eps = selected['eps']
velocity_initial = signal.savgol_filter(c, 41, 4, deriv=1, delta=.001)

def replay(ix, kp=60., kd=2.):
    # Same held target at every recorded nominal 1 ms control tick.
    position = float(c[ix[0]])
    velocity = float(velocity_initial[ix[0]])
    delta = float(d[ix[0]])
    prediction = np.empty(len(ix))
    output = np.empty(len(ix))
    dt = .001
    def accel(x, v, effort):
        return (effort - B*v - F*math.tanh(v/eps)
                - Gs*math.sin(x) - Gc*math.cos(x) - bias) / M
    for k, sample in enumerate(ix):
        prediction[k] = position
        u0 = np.clip(kp*(qref[sample, 0] - (position-delta/2)) - kd*velocity, -40, 40)
        u1 = np.clip(kp*(qref[sample, 1] - (position+delta/2)) - kd*velocity, -40, 40)
        effort = u0 + u1
        output[k] = effort
        # Midpoint integration: no measured q/dq after the initial condition.
        acc = accel(position, velocity, effort)
        vm = velocity + .5*dt*acc
        xm = position + .5*dt*velocity
        position += dt*vm
        velocity += dt*accel(xm, vm, effort)
    return prediction, output

metrics = []
replays = {}
for segment, partition in [(45, 'development'), (47, 'development'), (57, 'development'),
                           (59, 'development'), (127, 'development'), (129, 'development'),
                           (71, 'reserved_validation'), (81, 'reserved_validation')]:
    ix = np.flatnonzero(a['segment_id'] == segment)
    pred, effort = replay(ix)
    replays[segment] = (ix, pred)
    eval_ix = slice(1500, -1500)
    truth = c[ix][eval_ix]
    model_error = pred[eval_ix] - truth
    actual_tracking = cr[ix][eval_ix] - truth
    metrics.append(dict(segment=segment, partition=partition,
        common_prediction_rmse_rad=float(np.sqrt(np.mean(model_error**2))),
        common_measured_tracking_rmse_rad=float(np.sqrt(np.mean(actual_tracking**2))),
        observed_common_ptp_rad=float(np.ptp(truth)),
        simulated_total_effort_peak_nm=float(np.max(np.abs(effort)))))

# Fixed candidate comparison on development trajectories, not validation-based tuning.
counterfactual = []
for segment in (45, 57, 127):
    ix, baseline = replays[segment]
    candidate, effort = replay(ix, kp=120., kd=3.)
    sample = slice(1500, -1500)
    counterfactual.append(dict(segment=segment,
        simulated_60_2_tracking_rmse_rad=float(np.sqrt(np.mean((baseline[sample]-cr[ix][sample])**2))),
        simulated_120_3_tracking_rmse_rad=float(np.sqrt(np.mean((candidate[sample]-cr[ix][sample])**2))),
        simulated_120_3_total_effort_peak_nm=float(np.max(np.abs(effort)))))

report = dict(
    source_bag='left-pair-pd-20260928T050017Z',
    mcap_sha256='a60ae06cb0a4178068bdcf563038074157fb1268cf8738b91c9908784838f4c8',
    model_selection='eps chosen using existing development inverse-dynamics RMSE only; no refitting on held-out forward response',
    selected_empirical_common_model=selected,
    scope='Reduced common-mode inverse torque fit validated by independent free closed-loop replay; differential coordinate frozen to its initial value, ideal feedback/torque, nominal 1 ms. Not full closed-chain or actuator identification.',
    state_use='Recorded q/dq only at segment initial condition; feedback then simulated; recorded 50 Hz position targets retained',
    metrics=metrics,
    counterfactual_common_gain_comparison=counterfactual,
    counterfactual_scope="Same old reference, frozen reduced model, simulated common-mode tracking only; no empirical prediction of knee extension, new larger trajectory, or actuator stability",
    gain_decision='Do not optimize/release PD from this reduced model without adequate differential excitation and validated actuator/feedback dynamics.',
)
(ROOT / 'forward_validation.json').write_text(json.dumps(report, indent=2) + '\n')
fig, ax = plt.subplots(2, 1, figsize=(12, 6.5), constrained_layout=True)
for panel, segment in zip(ax, [71, 81]):
    ix, pred = replays[segment]
    stride = slice(None, None, 10)
    ts = (clock[ix] - clock[ix[0]]) * 1e-9
    origin = c[ix[0]]
    panel.plot(ts[stride], np.rad2deg(cr[ix][stride]-origin), color='#8c939c', label='预设目标', lw=1, alpha=.65)
    panel.plot(ts[stride], np.rad2deg(c[ix][stride]-origin), color=orange, label='实测', lw=1.5)
    panel.plot(ts[stride], np.rad2deg(pred[stride]-origin), color=blue, label='简化模型自由闭环回放', lw=1.2)
    metric = next(m for m in metrics if m['segment'] == segment)
    panel.set_title(f'独立多正弦段 {segment}：预测实测差异 RMSE={np.rad2deg(metric["common_prediction_rmse_rad"]):.2f}°')
    panel.set(ylabel='平均角度变化（°）', xlabel='段内时间（s）')
    panel.grid(alpha=.17)
    panel.legend(fontsize=9, loc='upper right')
fig.suptitle('拟合参数的回放检验｜只验证整体摆动，尚非完整电机/闭链模型', fontsize=14)
fig.savefig(ROOT / 'common_forward_validation.png', dpi=160)
print(json.dumps(report, ensure_ascii=False, indent=2))
