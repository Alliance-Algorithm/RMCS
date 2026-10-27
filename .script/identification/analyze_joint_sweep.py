#!/usr/bin/env python3
"""Audit wheel-leg sweep CSV; fit local coupled mechanics and host-to-current delay.

Fits are candidates in current-derived torque units. Local affine loads cannot
replace a full-turn closed-chain/gravity/gas-spring model in a simulator.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

S = "/wheel_leg/sweep/"


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        needed = [i for i, name in enumerate(header) if name in ("index", "/wheel_leg/dr16_fresh",
                  "/wheel_leg/enable_request", "/wheel_leg/feedback_fresh") or name.startswith(S)
                  or name.startswith("/wheel_leg/") and any(name.endswith("/" + suffix) for suffix in
                     ("angle", "velocity", "torque", "control_torque", "tau_frame_api", "tx_kind",
                      "tx_queued_steady_ns", "feedback_sequence", "feedback_steady_ns", "status_code"))]
        columns = {header[i]: [] for i in needed}
        incomplete = []
        for line, row in enumerate(reader, 2):
            if len(row) != len(header):
                incomplete.append({"line": line, "fields": len(row), "expected": len(header)})
                continue
            for i in needed:
                value = row[i]
                integer = header[i].endswith(("_ns", "feedback_sequence")) or header[i] == "index"
                columns[header[i]].append(int(value) if integer else float(value))
    return {key: np.asarray(value) for key, value in columns.items()}, incomplete


def metrics(y, prediction):
    rmse = float(np.sqrt(np.mean((y - prediction) ** 2)))
    scale = float(np.std(y))
    return {"rmse": rmse, "nrmse": rmse / max(scale, 1e-9),
            "r2": 1 - rmse * rmse / max(scale * scale, 1e-18)}


def previous_hold(t, values, query):
    return values[np.clip(np.searchsorted(t, query, side="right") - 1, 0, len(t) - 1)]


def fit_delay(command_t, command, feedback_t, measured, train, holdout, groups):
    """Fit a delayed first-order command response without differentiating torque.

    Segment-local filter initialization and a 0.25 s burn-in avoid connecting
    discontinuous runs. Both the command and response use host event timestamps.
    """
    from scipy.signal import lfilter

    best = None
    for delay_ms in range(0, 61, 2):
        for lag_s in (0, .002, .005, .01, .02, .04):
            response = np.empty_like(measured)
            usable = np.zeros(len(measured), dtype=bool)
            for indices in groups:
                t = feedback_t[indices]
                grid = np.arange(t[0], t[-1] + .001, .001)
                u = previous_hold(command_t, command, grid - delay_ms / 1000)
                alpha = np.exp(-.001 / lag_s) if lag_s else 0
                filtered = lfilter([1 - alpha], [1, -alpha], u, zi=[alpha * u[0]])[0]
                response[indices] = np.interp(t, grid, filtered)
                usable[indices] = t - t[0] > .25
            training, testing = train & usable, holdout & usable
            if np.count_nonzero(training) < 300 or np.count_nonzero(testing) < 300:
                return {"accepted": False, "reason": "need complete training and holdout dynamic segments"}
            x = np.column_stack((response, np.ones(len(response))))
            gain, bias = np.linalg.lstsq(x[training], measured[training], rcond=None)[0]
            if gain <= 0 or np.std(response[training]) < .1:
                continue
            prediction = x @ [gain, bias]
            training_metrics = metrics(measured[training], prediction[training])
            if best is None or training_metrics["rmse"] < best["train"]["rmse"]:
                best = {"delay_ms": delay_ms, "time_constant_ms": lag_s * 1000,
                        "gain": float(gain), "bias_nm": float(bias), "train": training_metrics,
                        "holdout": metrics(measured[testing], prediction[testing])}
    if best is None:
        return {"accepted": False, "reason": "no positive excited command-to-current response"}
    best["accepted"] = (best["holdout"]["nrmse"] < .35 and best["delay_ms"] < 60
                        and best["time_constant_ms"] < 40)
    best["scope"] = "host queue to reported current-based torque, including transport/measurement effects"
    return best


def design(acc, vel, q, center):
    n = len(q)
    x = np.zeros((n, 2, 14))
    x[:, 0, :2], x[:, 1, 1:3] = acc, acc
    x[:, 0, 3:5], x[:, 1, 4:6] = vel, vel
    x[:, 0, 6], x[:, 1, 7] = np.tanh(30 * vel[:, 0]), np.tanh(30 * vel[:, 1])
    pose = np.column_stack((np.ones(n), q - center))
    x[:, 0, 8:11], x[:, 1, 11:14] = pose, pose
    return x


def physical_parameters(p):
    result = p.copy()
    for offset, floor in ((0, 1e-6), (3, 0)):
        a, b, c = p[offset:offset + 3]
        factor = np.array([[np.exp(a), 0], [b, np.exp(c)]])
        matrix = factor @ factor.T + floor * np.eye(2)
        result[offset:offset + 3] = matrix[0, 0], matrix[0, 1], matrix[1, 1]
    result[6:8] = np.exp(p[6:8])
    return result


def fit_local(q, vel, acc, tau, holdout):
    from scipy.optimize import least_squares

    train = ~holdout
    if np.count_nonzero(train) < 500 or np.count_nonzero(holdout) < 500:
        return {"accepted": False, "reason": "too few complete local train/holdout observations"}
    center = np.mean(q[train], axis=0)
    x = design(acc, vel, q, center)
    matrix = x[train].reshape(-1, 14)
    norms = np.linalg.norm(matrix, axis=0)
    if np.any(norms < 1e-8):
        return {"accepted": False, "reason": "unexcited mechanical parameter"}
    singular = np.linalg.svd(matrix / norms, compute_uv=False)
    condition = float(singular[0] / singular[-1])
    if condition > 1e5:
        return {"accepted": False, "reason": "rank/conditioning insufficient", "condition": condition}
    initial = np.linalg.lstsq(matrix, tau[train].ravel(), rcond=None)[0]
    for offset, floor in ((0, 1e-5), (3, 1e-5)):
        a, b, c = initial[offset:offset + 3]
        value, vectors = np.linalg.eigh([[a, b], [b, c]])
        factor = np.linalg.cholesky((vectors * np.maximum(value, floor)) @ vectors.T)
        initial[offset:offset + 3] = np.log(factor[0, 0]), factor[1, 0], np.log(factor[1, 1])
    initial[6:8] = np.log(np.maximum(initial[6:8], .001))
    lower, upper = np.full(14, -np.inf), np.full(14, np.inf)
    lower[[0, 2, 3, 5, 6, 7]], upper[[0, 2, 3, 5, 6, 7]] = -15, 6
    result = least_squares(lambda p: matrix @ physical_parameters(p) - tau[train].ravel(),
                           initial, bounds=(lower, upper), loss="soft_l1", f_scale=.2,
                           max_nfev=500, x_scale="jac")
    parameters = physical_parameters(result.x)
    prediction = np.einsum("nij,j->ni", x, parameters)
    m = np.array([[parameters[0], parameters[1]], [parameters[1], parameters[2]]])
    b = np.array([[parameters[3], parameters[4]], [parameters[4], parameters[5]]])
    load = parameters[8:].reshape(2, 3)
    checks = [metrics(tau[holdout, j], prediction[holdout, j]) for j in range(2)]
    return {"accepted": bool(result.success and all(row["nrmse"] < .35 for row in checks)),
            "scope": "local fixed-base effective model in reported-torque units; not a global URDF damping fit",
            "M": m.tolist(), "B": b.tolist(), "coulomb": parameters[6:8].tolist(),
            "load_affine": load.tolist(), "q_center": center.tolist(), "condition": condition,
            "holdout_torque": checks, "optimizer_success": bool(result.success)}


def tune_pd(model, delays, torque_cap):
    """Search diagonal gains on the coupled model with fitted delay and actuator lag.

    Results are offline candidates. The nonlinear full-turn model and vehicle
    experiments still need separate validation before changing production gains.
    """
    m, b = np.asarray(model["M"]), np.asarray(model["B"])
    stiffness = np.asarray(model["load_affine"])[:, 1:]
    load_bias = np.asarray(model["load_affine"])[:, 0]
    friction = np.asarray(model["coulomb"])
    drive_gain = np.asarray([row["gain"] for row in delays])
    drive_bias = np.asarray([row["bias_nm"] for row in delays])
    inv_m = np.linalg.inv(m)
    candidates = []
    for hz in (2, 3, 4, 5, 6):
        for damping in (.8, 1, 1.2):
            omega = 2 * np.pi * hz
            kp = np.clip(np.diag(m) * omega ** 2 - np.diag(stiffness), 20, 320)
            kd = np.clip(2 * damping * omega * np.diag(m) - np.diag(b), .2, 16)
            try:
                equilibrium = np.linalg.solve(np.diag(drive_gain * kp) + stiffness, drive_bias - load_bias)
            except np.linalg.LinAlgError:
                continue
            initial_torque = load_bias + stiffness @ equilibrium
            if np.any(np.abs(equilibrium) > .15) or np.any(np.abs(kp * equilibrium) > torque_cap):
                continue
            worst, saturated = 0., 0
            valid = True
            for axis in (0, 1):
                dt, steps = .001, 2000
                q, v, tau = equilibrium.copy(), np.zeros(2), initial_torque.copy()
                target = np.eye(2)[axis] * .05
                queue = np.zeros((steps, 2))
                errors = []
                for tick in range(steps):
                    command = kp * (target - q) - kd * v
                    saturated += int(np.any(np.abs(command) > torque_cap))
                    queue[tick] = np.clip(command, -torque_cap, torque_cap)
                    for j in range(2):
                        past = tick - round(delays[j]["delay_ms"])
                        u = (queue[past, j] * drive_gain[j] + drive_bias[j]
                             if past >= 0 else initial_torque[j])
                        lag_s = delays[j]["time_constant_ms"] / 1000
                        alpha = np.exp(-dt / lag_s) if lag_s else 0
                        tau[j] = alpha * tau[j] + (1 - alpha) * u
                    v += dt * (inv_m @ (tau - load_bias - b @ v - stiffness @ q
                                       - friction * np.tanh(30 * v)))
                    q += dt * v
                    error = np.linalg.norm(target - q)
                    if not np.isfinite(error) or error > .3:
                        valid = False
                        break
                    errors.append(error)
                if not valid or np.linalg.norm(target - q) > .01:
                    valid = False
                    break
                worst = max(worst, float(np.sqrt(np.mean(np.square(errors)))))
            if valid:
                candidates.append({"kp": kp.tolist(), "kd": kd.tolist(), "step_rmse_rad": worst,
                                   "saturated_fraction": saturated / 4000})
    if not candidates:
        return {"available": False, "reason": "no stable delayed coupled-model step candidate"}
    best = min(candidates, key=lambda row: row["step_rmse_rad"] + .05 * row["saturated_fraction"])
    return {"available": True, "candidate": best, "auto_applied": False,
            "validation": "offline 0.05 rad steps of each axis, 1 kHz, fitted delay/lag and static-load PD error"}


def validate_rollouts(model, time, q, velocity, torque, holdout, stage):
    """Short free trajectories on untouched segments; never use measured states mid-window."""
    inv_m, b = np.linalg.inv(model["M"]), np.asarray(model["B"])
    load, center, friction = (np.asarray(model[key]) for key in ("load_affine", "q_center", "coulomb"))
    position_errors, speed_errors = [], []
    start = 0
    while start + 1 < len(time):
        if not holdout[start]:
            start += 1
            continue
        predicted_q, predicted_v = q[start].copy(), velocity[start].copy()
        end = start
        while end + 1 < len(time) and time[end + 1] - time[start] <= .15:
            dt = time[end + 1] - time[end]
            if not holdout[end + 1] or stage[end + 1] != stage[start] or dt <= 0 or dt > .01:
                break
            acceleration = inv_m @ (torque[end] - b @ predicted_v - friction * np.tanh(30 * predicted_v)
                                      - load[:, 0] - load[:, 1:] @ (predicted_q - center))
            predicted_v += dt * acceleration
            predicted_q += dt * predicted_v
            position_errors.append(predicted_q - q[end + 1])
            speed_errors.append(predicted_v - velocity[end + 1])
            end += 1
        start = max(start + 1, end + 1)
    if len(position_errors) < 300:
        return {"accepted": False, "reason": "too few continuous held-out rollout observations"}
    position_rmse = float(np.sqrt(np.mean(np.square(position_errors))))
    speed_rmse = float(np.sqrt(np.mean(np.square(speed_errors))))
    return {"accepted": bool(position_rmse < .02 and speed_rmse < .4),
            "q_rmse_rad": position_rmse, "velocity_rmse_rad_s": speed_rmse,
            "window_s": .15, "observations": len(position_errors)}


def analyze(data, incomplete, fit=False):
    state, active = data[S + "state"], data[S + "reference_active"]
    running = (state == 3) & (active == 1)
    report = {"complete_records": len(state), "incomplete_records": incomplete,
              "duration_s": float((int(data[S + "control_steady_ns"][-1])
                                    - int(data[S + "control_steady_ns"][0])) * 1e-9),
              "state_counts": {str(int(x)): int(np.sum(state == x)) for x in np.unique(state)},
              "stop_reason_counts": {str(int(x)): int(np.sum(data[S + "stop_reason"] == x))
                                     for x in np.unique(data[S + "stop_reason"])},
              "active_records": int(np.count_nonzero(running)), "real2sim_ready": False,
              "accepted": False, "local_models": [], "delay": [], "high_speed_delay": [], "axes": {}}
    report["remote_counts"] = {
        name: {str(int(x)): int(np.count_nonzero(data[name] == x)) for x in np.unique(data[name])}
        for name in ("/wheel_leg/dr16_fresh", "/wheel_leg/enable_request", S + "remote_left", S + "remote_right")
        if name in data}
    if not np.any(running):
        report["rejection_reasons"] = ["no running/active reference samples; do not fit damping, delay or PD"]
        return report
    sides = np.unique(data[S + "side"][running])
    if len(sides) != 1 or sides[0] not in (0, 1):
        raise ValueError("CSV must contain one selected leg")
    side = "left" if sides[0] == 0 else "right"
    axes = [f"{side}_{joint}_joint" for joint in ("hip", "knee")]
    report["side"] = side
    for axis in axes:
        q = data[S + axis + "/model_angle"]
        v = data[S + axis + "/model_velocity"]
        e = data[S + axis + "/tracking_error"]
        report["axes"][axis] = {"angle_span_rad": float(np.ptp(q[running])),
            "speed_abs_p95_rad_s": float(np.percentile(np.abs(v[running]), 95)),
            "tracking_rms_rad": float(np.sqrt(np.nanmean(e[running] ** 2))),
            "saturated_fraction": float(np.mean(data[S + axis + "/saturated"][running])),
            "repeated_feedback_fraction": float(np.mean(np.diff(data[f"/wheel_leg/{axis}/feedback_sequence"]) == 0))}
    if not fit:
        report["accepted"] = True
        report["fit_status"] = "audit only; use --fit-local on excitation-v3 mode 3 for candidate models"
        return report
    required = [S + name for name in ("excitation_revision", "stage_elapsed_s", "stage_duration_s")]
    if any(name not in data for name in required):
        report["rejection_reasons"] = ["missing excitation-v3 stage metadata and independent probe provenance"]
        return report
    # Reject partial segments and never let a holdout segment become training rows.
    eligible = np.zeros(len(state), dtype=bool)
    segment_masks = []
    for run, stage in np.unique(np.column_stack((data[S + "run_id"], data[S + "stage"])), axis=0):
        mask = running & (data[S + "run_id"] == run) & (data[S + "stage"] == stage)
        indices = np.flatnonzero(mask)
        if len(indices) < 500 or np.any(data[S + "excitation_revision"][mask] != 3):
            continue
        if not np.all(np.isin(data[S + "waveform"][mask], [4, 6])):
            continue
        elapsed, duration = data[S + "stage_elapsed_s"][mask], data[S + "stage_duration_s"][mask]
        if elapsed.min() > .025 or elapsed.max() < duration[0] - .025:
            continue
        if len(np.unique(data[S + "validation"][mask])) != 1:
            raise ValueError("validation label changes within one stage")
        eligible |= mask
        segment_masks.append(mask)
    if not segment_masks:
        report["rejection_reasons"] = ["need complete waveform=4/6 train/holdout segments"]
        return report
    ns0 = int(data[S + "control_steady_ns"][0])
    q = np.column_stack([data[S + axis + "/model_angle"] for axis in axes])
    v = np.column_stack([data[S + axis + "/model_velocity"] for axis in axes])
    signs = []
    for j, axis in enumerate(axes):
        api_velocity = data[f"/wheel_leg/{axis}/velocity"]
        moving = eligible & np.isfinite(api_velocity) & np.isfinite(v[:, j]) & (np.abs(api_velocity) > .05)
        if np.count_nonzero(moving) < 100:
            report["rejection_reasons"] = ["too little measured motion to verify API/model effort directions"]
            return report
        sign = 1 if np.median(v[moving, j] / api_velocity[moving]) > 0 else -1
        if not np.allclose(v[moving, j], sign * api_velocity[moving], atol=1e-7, rtol=1e-6):
            report["rejection_reasons"] = ["API/model velocity calibration is inconsistent"]
            return report
        signs.append(sign)
    report["model_sign"] = signs
    stamps = np.column_stack([data[f"/wheel_leg/{axis}/feedback_steady_ns"] for axis in axes])
    seq = np.column_stack([data[f"/wheel_leg/{axis}/feedback_sequence"] for axis in axes])
    tau = np.asarray(signs) * np.column_stack([data[f"/wheel_leg/{axis}/torque"] for axis in axes])
    saturation = np.column_stack([data[S + axis + "/saturated"] for axis in axes]).any(axis=1)
    controls = data[S + "control_steady_ns"]
    fresh = np.all((controls[:, None] >= stamps) & ((controls[:, None] - stamps) < 5_000_000), axis=1)
    finite = np.isfinite(q).all(axis=1) & np.isfinite(v).all(axis=1) & np.isfinite(tau).all(axis=1)
    good = eligible & ~saturation & fresh & finite & (np.abs(stamps[:, 0] - stamps[:, 1]) < 2_000_000)
    for axis_index, axis in enumerate(axes):
        motor, sweep = f"/wheel_leg/{axis}/", S + axis + "/"
        tx = data[motor + "tx_queued_steady_ns"]
        u = signs[axis_index] * data[motor + "tau_frame_api"]
        command_mask = (data[motor + "tx_kind"] == 1) & np.isfinite(u) & (tx > 0)
        ci = np.flatnonzero(command_mask)
        if len(ci):
            ci = ci[np.r_[True, np.diff(tx[ci]) > 0]]
        receive = data[motor + "feedback_steady_ns"]
        sequences = data[motor + "feedback_sequence"]
        for waveform, key in ((6, "delay"), (4, "high_speed_delay")):
            fi = np.flatnonzero(good & (data[S + "waveform"] == waveform)
                                & np.r_[False, np.diff(sequences) > 0])
            group_indices = [np.flatnonzero(mask[fi]) for mask in segment_masks]
            group_indices = [indices for indices in group_indices if len(indices)]
            if not len(ci) or not len(fi):
                report[key].append({"accepted": False, "reason": "no encoded torque frames/fresh responses"})
                continue
            if waveform == 6 and not np.any(data[sweep + "probe_torque_model"][fi] != 0):
                report[key].append({"accepted": False, "reason": "no independent torque probe"})
                continue
            if waveform == 4 and np.median(np.abs(data[sweep + "model_velocity"][fi])) < 2:
                report[key].append({"accepted": False, "reason": "actual high-speed motion was not achieved"})
                continue
            holdout = data[S + "validation"][fi] == 1
            # Timestamp on every frame is used, not a reference/position cross-correlation.
            report[key].append(fit_delay((tx[ci] - ns0) * 1e-9, u[ci], (receive[fi] - ns0) * 1e-9,
                                         signs[axis_index] * data[motor + "torque"][fi], ~holdout, holdout, group_indices))
    samples, accelerations = [], []
    for mask in segment_masks:
        if not np.all(data[S + "waveform"][mask] == 6):
            continue
        indices = np.flatnonzero(mask & good)
        paired = []
        for i in indices:
            if not paired or np.all(seq[i] > seq[paired[-1]]):
                paired.append(i)
        for n in range(2, len(paired) - 2):
            lo, mid, hi = paired[n - 2], paired[n], paired[n + 2]
            gaps = np.diff(stamps[paired[n - 2:n + 3]], axis=0)
            if np.any(gaps <= 0) or np.any(gaps > 5_000_000) or not np.all(good[lo:hi + 1]):
                continue
            dt = (stamps[hi] - stamps[lo]) * 1e-9
            samples.append(mid)
            accelerations.append((v[hi] - v[lo]) / dt)
    indices, acceleration = np.asarray(samples, dtype=int), np.asarray(accelerations)
    beta = np.rad2deg(data[S + f"{side}/beta_estimate"])
    for center in (50, 75, 100):
        selected = np.abs(beta[indices] - center) < 5
        ii, aa = indices[selected], acceleration[selected]
        if not len(ii) or np.any(np.ptp(q[ii], axis=0) > .5):
            report["local_models"].append({"beta_deg": center, "accepted": False,
                                            "reason": "missing local data or excursion outside local model"})
            continue
        model = fit_local(q[ii], v[ii], aa, tau[ii], data[S + "validation"][ii] == 1)
        model["beta_deg"] = center
        if model["accepted"]:
            time = (controls[ii] - ns0) * 1e-9
            model["holdout_rollout"] = validate_rollouts(model, time, q[ii], v[ii], tau[ii],
                data[S + "validation"][ii] == 1, data[S + "stage"][ii])
            model["accepted"] = model["holdout_rollout"]["accepted"]
        if model["accepted"] and all(row["accepted"] for row in report["delay"]):
            caps = np.asarray([np.min(data[S + f"host/{joint}_torque_limit_nm"][ii]) for joint in ("hip", "knee")])
            model["pd_search"] = tune_pd(model, report["delay"], caps)
        report["local_models"].append(model)
    report["accepted"] = bool(len(report["local_models"]) == 3 and all(x["accepted"] for x in report["local_models"])
                              and len(report["delay"]) == 2 and all(x["accepted"] for x in report["delay"])
                              and len(report["high_speed_delay"]) == 2 and all(x["accepted"] for x in report["high_speed_delay"]))
    report["real2sim_limits"] = ["effective torque scale lacks independent shaft-torque calibration",
        "local models do not identify full-turn gravity/gas-spring/Coriolis or grounded contact",
        "PD candidates require independent trajectory replay and vehicle validation before deployment"]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fit-local", action="store_true")
    args = parser.parse_args()
    data, incomplete = read_csv(args.csv)
    report = analyze(data, incomplete, args.fit_local)
    report["source_csv"] = str(args.csv.resolve())
    with args.csv.open("rb") as stream:
        report["source_sha256"] = hashlib.file_digest(stream, "sha256").hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
