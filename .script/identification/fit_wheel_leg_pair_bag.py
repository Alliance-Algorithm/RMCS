#!/usr/bin/env python3
"""Fit a local, coupled hip/aux plant from export_identification_bag NPZ + JSON.

The identified torque is *reported motor feedback* in calibrated model units,
not measured joint effort. The local pose load includes gravity, spring and
other static effects together; none of those contributions is separable here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    from scipy.optimize import least_squares
except ModuleNotFoundError:
    least_squares = None


NAMES = ("M00", "M01", "M11", "B00", "B01", "B11", "Fc0", "Fc1",
         "load0_bias", "load0_q0", "load0_q1", "load1_bias", "load1_q0", "load1_q1")


class UnidentifiableError(ValueError):
    def __init__(self, ranks: dict):
        super().__init__("coupled mass/friction/pose load not identifiable: rank or condition failed")
        self.ranks = ranks


def load_run(npz_path: Path, profile: Path) -> tuple[dict, dict, dict]:
    metadata = json.loads(npz_path.with_suffix(".json").read_text())
    if metadata.get("schema_version") not in (2, 3, 4) or metadata.get("side") not in ("left", "right"):
        raise ValueError("expected exporter schema 2, 3 or 4 with a left/right side")
    digest = hashlib.sha256(profile.read_bytes()).hexdigest()
    if digest != metadata.get("profile_sha256"):
        raise ValueError("saved run profile SHA256 differs from exporter metadata")
    root = yaml.safe_load(profile.read_text())
    if not isinstance(root, dict):
        raise ValueError("run YAML must contain recorder and controller parameters")
    controller = root.get("wheel_leg_pair_identification_controller", {}).get("ros__parameters", {})
    recorder = root.get("wheel_leg_identification_recorder", {}).get("ros__parameters", {})
    if controller.get("side") != metadata["side"] or recorder.get("side") != metadata["side"]:
        raise ValueError("both recorder and pair controller must select the exported side")
    if controller.get("probe_pattern") in ("rotation_chirp", "multiband_chirp"):
        raise ValueError(
            "Full-turn rotation_chirp/multiband_chirp requires fixed-base nonlinear closed-chain replay with "
            "gravity and gas springs; this local affine-load fitter is not valid for that run")
    gains = {}
    for name in ("kp_position", "kp_velocity", "ki_velocity", "max_speed", "max_torque", "integral_limit"):
        value = controller.get(name, [0.] * 4 if name == "ki_velocity" else None)
        vector = np.asarray(value, dtype=float) if value is not None else np.array([])
        if vector.shape != (4,) or not np.isfinite(vector).all() or np.any(vector < 0):
            raise ValueError(f"run YAML needs four finite nonnegative {name} gains/limits")
        if name in ("kp_position", "kp_velocity", "max_speed", "max_torque") and np.any(vector == 0):
            raise ValueError(f"run YAML {name} must be positive")
        gains[name] = vector.tolist()
    if np.any((np.asarray(gains["ki_velocity"]) > 0) & (np.asarray(gains["integral_limit"]) == 0)):
        raise ValueError("velocity integral requires a positive integral_limit")
    with np.load(npz_path, allow_pickle=False) as file:
        arrays = {key: file[key] for key in file.files}
    return arrays, metadata, gains


def checked_run(a: dict, meta: dict, max_pair_dt_s: float) -> tuple[list[np.ndarray], tuple[int, int]]:
    side = meta["side"]
    selected = (0, 1) if side == "left" else (2, 3)
    if (meta.get("selected_axes") != list(selected) or meta.get("insufficient_holdout")
        or meta.get("model_order") != ["left_hip", "left_aux", "right_hip", "right_aux"]):
        raise ValueError("selected axes or complete-segment holdout missing")
    required = {"pair_valid": (), "side_valid": (), "double_loop_observed": (), "pair_epoch": (),
                "segment_id": (), "segment_instance": (), "split": (), "actuation_scope": (),
                "selected_side": (), "phase": (), "tick": (), "control_steady_ns": (),
                "feedback_steady_ns": (6,), "feedback_sequence": (6,),
                "q_model": (4,), "dq_model": (4,), "torque_fb_model": (4,),
                "tau_frame_model": (4,), "q_ref_model": (4,), "dq_ref_model": (4,),
                "velocity_target_model": (4,), "tau_preclip_model": (4,),
                "tx_kind": (6,), "tx_queued_steady_ns": (6,), "torque_integral_model": (4,)}
    if "pair_valid" not in a or a["pair_valid"].ndim != 1:
        raise ValueError("missing one-dimensional pair_valid")
    n = len(a["pair_valid"])
    for key, shape in required.items():
        if key not in a or a[key].shape != (n, *shape):
            raise ValueError(f"missing or malformed exported {key}")
    for key in ("control_steady_ns", "feedback_steady_ns", "feedback_sequence", "tx_queued_steady_ns", "tick"):
        if a[key].dtype != np.uint64:
            raise ValueError(f"{key} must retain exact uint64 timestamps/sequences")
    pairs = np.flatnonzero(a["pair_valid"])
    if not len(pairs) or not np.all(a["split"][pairs] != 0):
        raise ValueError("no labeled paired observations")
    if not np.all(a["selected_side"] == (0 if side == "left" else 1)):
        raise ValueError("recorded side disagrees with run profile")
    if not np.all(a["double_loop_observed"][pairs]):
        raise ValueError("missing two-loop references or pure-torque frames on accepted pairs")
    if not np.all(a["side_valid"][pairs]) or not np.all(a["actuation_scope"][pairs] == 1):
        raise ValueError("accepted pairs require valid selected-side-only commands")
    if not np.all(a["phase"][pairs] == 2) or not np.all(a["segment_id"][pairs] >= 0):
        raise ValueError("accepted pairs must belong to running segments")
    if not np.isfinite(np.concatenate([a[key][pairs][:, selected].ravel() for key in
                                        ("q_model", "dq_model", "torque_fb_model", "tau_frame_model",
                                         "q_ref_model", "dq_ref_model", "velocity_target_model",
                                         "tau_preclip_model")])).all():
        raise ValueError("nonfinite model coordinates, torque or two-loop references on paired observations")
    for key in ("axis_encoder_ambiguous", "axis_sequence_gap", "pair_skew_fail", "pair_age_fail"):
        if key not in a:
            raise ValueError(f"missing exporter QC flag {key}")
    if np.any(a["feedback_steady_ns"][pairs][:, selected] == 0):
        raise ValueError("paired feedback without a CAN receive time")
    if np.any(a["feedback_steady_ns"][pairs][:, selected] > a["control_steady_ns"][pairs, None]):
        raise ValueError("future CAN receive time")
    segment_split = meta.get("segment_split", {})
    for i in pairs:
        label = "train" if a["split"][i] == 1 else "holdout" if a["split"][i] == 2 else None
        if label is None or segment_split.get(str(int(a["segment_id"][i]))) != label:
            raise ValueError("split must agree with exporter complete-segment labels")
    if len({int(a["segment_id"][i]) for i in pairs if a["split"][i] == 2}) == 0:
        raise ValueError("no held-out complete segment")

    # A repeated CAN value may be held while waiting for the other axis. It is
    # NEVER an additional observation. Bad raw ticks terminate the run, even
    # when a later accepted pair has the same segment ID.
    runs: list[list[int]] = []
    for i in pairs:
        start = runs[-1][-1] if runs else None
        contiguous = False
        if start is not None:
            path = slice(start + 1, i + 1)
            dt = (int(a["control_steady_ns"][i]) - int(a["control_steady_ns"][start])) * 1e-9
            old, new = a["feedback_steady_ns"][[start, i]][:, selected]
            seq0, seq1 = a["feedback_sequence"][[start, i]][:, selected]
            contiguous = (a["segment_id"][start] == a["segment_id"][i]
                          and a["segment_instance"][start] == a["segment_instance"][i]
                          and a["pair_epoch"][start] == a["pair_epoch"][i]
                          and a["split"][start] == a["split"][i]
                          and 0 < dt <= max_pair_dt_s
                          and np.all(seq1 > seq0) and np.all(new > old)
                          and all((int(new[j]) - int(old[j])) * 1e-9 <= max_pair_dt_s for j in range(2))
                          and np.all(a["side_valid"][path])
                          and np.all(a["actuation_scope"][path] == 1)
                          and not np.any(a["pair_skew_fail"][path])
                          and not np.any(a["pair_age_fail"][path])
                          and not np.any(a["axis_sequence_gap"][path][:, selected])
                          and not np.any(a["axis_encoder_ambiguous"][path][:, selected]))
        if contiguous:
            runs[-1].append(int(i))
        else:
            runs.append([int(i)])
    return [np.asarray(run, dtype=int) for run in runs], selected


def design(acc: np.ndarray, vel: np.ndarray, q: np.ndarray, center: np.ndarray,
           tanh_gain: float, damping: bool) -> np.ndarray:
    """Two equations per pair, symmetric M/B and per-axis local affine pose load."""
    n = len(q)
    x = np.zeros((2 * n, len(NAMES)))
    x[0::2, 0:2] = acc
    x[1::2, 1:3] = acc
    if damping:
        x[0::2, 3:5] = vel
        x[1::2, 4:6] = vel
    x[0::2, 6] = np.tanh(tanh_gain * vel[:, 0])
    x[1::2, 7] = np.tanh(tanh_gain * vel[:, 1])
    pose = np.column_stack((np.ones(n), q - center))
    x[0::2, 8:11] = pose
    x[1::2, 11:14] = pose
    return x if damping else x[:, [0, 1, 2, *range(6, 14)]]


def derivatives(a: dict, runs: list[np.ndarray], selected: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    indices, acceleration = [], []
    for run in runs:
        for prev, i, nxt in zip(run[:-2], run[1:-1], run[2:]):
            stamps = a["feedback_steady_ns"][[prev, nxt]][:, selected]
            dt = np.asarray([(int(stamps[1, j]) - int(stamps[0, j])) * 1e-9 for j in range(2)])
            if np.any(dt <= 0):
                continue
            indices.append(i)
            acceleration.append((a["dq_model"][nxt, selected] - a["dq_model"][prev, selected]) / dt)
    if not indices:
        raise ValueError("no consecutive triplets of valid fresh CAN pairs")
    return np.asarray(indices), np.asarray(acceleration)


def rank_report(x: np.ndarray, names: list[str]) -> dict:
    norm = np.linalg.norm(x, axis=0)
    if np.any(norm == 0):
        return {"rank": 0, "parameters": len(names), "condition": None, "singular_values": [],
                "names": names}
    singular = np.linalg.svd(x / norm, compute_uv=False)
    rank = int(np.count_nonzero(singular > singular[0] * 1e-8))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else None
    if condition is not None and not math.isfinite(condition):
        condition = None
    return {"rank": rank, "parameters": len(names), "condition": condition,
            "singular_values": singular.tolist(), "names": names}


def spd_initial(matrix: np.ndarray) -> np.ndarray:
    eig, vec = np.linalg.eigh(matrix)
    floor = max(1e-6, float(np.max(np.abs(eig))) * 1e-3)
    return np.linalg.cholesky((vec * np.maximum(eig, floor)) @ vec.T)


def fit_model(a: dict, runs: list[np.ndarray], selected: tuple[int, int],
              tanh_gain: float, max_condition: float, min_train: int) -> tuple[dict, dict, np.ndarray, np.ndarray]:
    if least_squares is None:
        raise ValueError("scipy is required for SPD-constrained paired least squares")
    indices, acc = derivatives(a, runs, selected)
    train = a["split"][indices] == 1
    if int(sum(train)) < min_train:
        raise ValueError("too few training triplets after asynchronous CAN QC")
    q = a["q_model"][indices][:, selected]
    vel = a["dq_model"][indices][:, selected]
    tau = a["torque_fb_model"][indices][:, selected]
    center = np.mean(q[train], axis=0)
    diagnostics = {}
    damping = True
    for with_damping in (True, False):
        names = list(NAMES if with_damping else NAMES[:3] + NAMES[6:])
        x = design(acc, vel, q, center, tanh_gain, with_damping)
        report = rank_report(x[np.repeat(train, 2)], names)
        diagnostics["with_damping" if with_damping else "without_damping"] = report
        if (report["rank"] == report["parameters"] and report["condition"] is not None
            and report["condition"] <= max_condition):
            damping = with_damping
            break
    else:
        raise UnidentifiableError(diagnostics)
    x_train = x[np.repeat(train, 2)]
    y_train = tau[train].reshape(-1)
    linear = np.linalg.lstsq(x_train, y_train, rcond=None)[0]
    mass = spd_initial(np.array([[linear[0], linear[1]], [linear[1], linear[2]]]))
    offset = 3
    if damping:
        viscous = spd_initial(np.array([[linear[3], linear[4]], [linear[4], linear[5]]]))
        offset = 6
    else:
        viscous = None
    # Cholesky makes M strictly SPD, B positive semidefinite; Fc is nonnegative.
    # The remaining six coefficients are an unconstrained *local* pose load.
    def unpack(p: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        lm = np.array([[math.exp(p[0]), 0.], [p[1], math.exp(p[2])]])
        m = lm @ lm.T
        k = 3
        if damping:
            lb = np.array([[p[k], 0.], [p[k+1], p[k+2]]])
            b = lb @ lb.T
            k += 3
        else:
            b = np.zeros((2, 2))
        return m, b, p[k:k+2], p[k+2:k+5], p[k+5:k+8]

    p0 = [math.log(mass[0, 0]), mass[1, 0], math.log(mass[1, 1])]
    if damping:
        p0 += [viscous[0, 0], viscous[1, 0], viscous[1, 1]]
    p0 += [*np.maximum(linear[offset:offset+2], 0), *linear[offset+2:]]
    low = np.full(len(p0), -np.inf)
    low[offset:offset+2] = 0.

    def predict(p: np.ndarray, qv: np.ndarray, vv: np.ndarray, av: np.ndarray) -> np.ndarray:
        m, b, fc, load0, load1 = unpack(p)
        pose = np.column_stack((np.ones(len(qv)), qv - center))
        load = np.column_stack((pose @ load0, pose @ load1))
        return av @ m + vv @ b + np.tanh(tanh_gain * vv) * fc + load

    scale = max(0.01, float(np.std(y_train)))
    opt = least_squares(lambda p: ((predict(p, q[train], vel[train], acc[train]) - tau[train]) / scale).ravel(),
                        p0, bounds=(low, np.full(len(p0), np.inf)), max_nfev=600)
    if not opt.success or not np.isfinite(opt.x).all():
        raise ValueError(f"constrained fit did not converge: {opt.message}")
    m, b, fc, load0, load1 = unpack(opt.x)
    prediction = predict(opt.x, q, vel, acc)
    model = {"mass_matrix_kg_m2_effective": m.tolist(),
             "damping_matrix_nm_s_rad_effective": b.tolist() if damping else None,
             "damping_observable": damping, "coulomb_nm_effective": fc.tolist(),
             "tanh_gain_s_rad": tanh_gain, "pose_center_rad": center.tolist(),
             "observed_training_q_range_rad": [q[train].min(axis=0).tolist(), q[train].max(axis=0).tolist()],
             "observed_training_dq_range_rad_s": [vel[train].min(axis=0).tolist(), vel[train].max(axis=0).tolist()],
             "static_pose_load_nm": {"hip": load0.tolist(), "aux": load1.tolist(),
                                     "basis": "[1, q_hip - center_hip, q_aux - center_aux]"},
             "input": "torque_fb_model (reported motor torque, not independent joint effort)",
             "equation": "tau_fb = M @ ddq + B @ dq + Fc * tanh(k*dq) + local_pose_load",
             "optimizer_nfev": int(opt.nfev)}
    return model, diagnostics, indices, prediction


def metrics(actual: np.ndarray, predicted: np.ndarray) -> dict:
    if not len(actual):
        return {"count": 0, "rmse": None, "mae": None, "r2": None}
    residual = actual - predicted
    var = float(np.sum((actual - actual.mean()) ** 2))
    return {"count": len(actual), "rmse": float(np.sqrt(np.mean(residual ** 2))),
            "mae": float(np.mean(np.abs(residual))),
            "r2": float(1 - np.sum(residual ** 2) / var) if var > 1e-16 else None}


def metrics_by_axis(actual: np.ndarray, predicted: np.ndarray) -> list[dict]:
    return [metrics(actual[:, j], predicted[:, j]) for j in range(2)]


def replay(a: dict, runs: list[np.ndarray], selected: tuple[int, int], model: dict,
           window_s: float) -> tuple[dict, dict]:
    """Causal local ZOH replay using only the *previous* observed feedback torque.

    Initial conditions are measured at the beginning of each bounded window;
    no state, torque or timestamp is interpolated over an invalid pair.
    """
    n = len(a["pair_valid"])
    q_pred, v_pred = np.full((n, 2), np.nan), np.full((n, 2), np.nan)
    window = np.full(n, -1, dtype=np.int32)
    scored = np.zeros(n, dtype=bool)
    m = np.asarray(model["mass_matrix_kg_m2_effective"])
    b = np.asarray(model["damping_matrix_nm_s_rad_effective"] if model["damping_observable"] else np.zeros((2, 2)))
    fc = np.asarray(model["coulomb_nm_effective"])
    center = np.asarray(model["pose_center_rad"])
    load = np.asarray([model["static_pose_load_nm"]["hip"], model["static_pose_load_nm"]["aux"]])
    k = model["tanh_gain_s_rad"]

    def acceleration(q: np.ndarray, v: np.ndarray, tau: np.ndarray) -> np.ndarray:
        force = tau - b @ v - fc * np.tanh(k * v) - load @ np.r_[1., q - center]
        return np.linalg.solve(m, force)

    count = 0
    for run in runs:
        first = 0
        while first < len(run) - 1:
            # Pair time is the later CAN receive time, never an invented control
            # sample. The older axis can be a few ms behind (exporter skew QC).
            origin = max(map(int, a["feedback_steady_ns"][run[first], selected]))
            last = first + 1
            while last + 1 < len(run) and (max(map(int, a["feedback_steady_ns"][run[last+1], selected]))
                                             - origin) * 1e-9 <= window_s:
                last += 1
            chunk = run[first:last+1]
            q = a["q_model"][chunk[0], selected].copy()
            v = a["dq_model"][chunk[0], selected].copy()
            q_pred[chunk[0]], v_pred[chunk[0]] = q, v
            window[chunk] = count
            for prev, cur in zip(chunk[:-1], chunk[1:]):
                dt = (max(map(int, a["feedback_steady_ns"][cur, selected]))
                      - max(map(int, a["feedback_steady_ns"][prev, selected]))) * 1e-9
                tau = a["torque_fb_model"][prev, selected]
                a0 = acceleration(q, v, tau)
                mid_v = v + .5 * dt * a0
                mid_q = q + .5 * dt * v
                v = v + dt * acceleration(mid_q, mid_v, tau)
                q = q + dt * mid_v
                q_pred[cur], v_pred[cur] = q, v
                scored[cur] = True
            count += 1
            first = last + 1
    scored_ids = np.flatnonzero(scored & np.isfinite(q_pred).all(axis=1))
    if not len(scored_ids):
        raise ValueError("no continuous pairs for causal local replay")
    curves = {"row": np.flatnonzero(window >= 0), "window_id": window[window >= 0],
              "control_steady_ns": a["control_steady_ns"][window >= 0],
              "feedback_steady_ns": a["feedback_steady_ns"][window >= 0][:, selected],
              "segment_id": a["segment_id"][window >= 0], "split": a["split"][window >= 0],
              "q_observed": a["q_model"][window >= 0][:, selected], "q_replayed": q_pred[window >= 0],
              "dq_observed": a["dq_model"][window >= 0][:, selected], "dq_replayed": v_pred[window >= 0],
              "q_residual": a["q_model"][window >= 0][:, selected] - q_pred[window >= 0],
              "dq_residual": a["dq_model"][window >= 0][:, selected] - v_pred[window >= 0]}
    results = {}
    for label, code in (("train", 1), ("holdout", 2)):
        chosen = scored_ids[a["split"][scored_ids] == code]
        results[label] = {"q_rad": metrics_by_axis(a["q_model"][chosen][:, selected], q_pred[chosen]),
                          "dq_rad_s": metrics_by_axis(a["dq_model"][chosen][:, selected], v_pred[chosen]),
                          "windows": len(np.unique(window[chosen])), "by_segment": {}}
        for segment in np.unique(a["segment_id"][chosen]):
            group = chosen[a["segment_id"][chosen] == segment]
            results[label]["by_segment"][str(int(segment))] = {
                "q_rad": metrics_by_axis(a["q_model"][group][:, selected], q_pred[group]),
                "dq_rad_s": metrics_by_axis(a["dq_model"][group][:, selected], v_pred[group])}
    return results, curves


def alignment(a: dict, runs: list[np.ndarray], selected: tuple[int, int], max_lag_ms: int) -> dict:
    """Diagnostic host-queue to CAN-receive delay; ZOH, same QC run only.

    Correlation has an arbitrary offset/scale nuisance removed. These are NOT
    motor torque constants, CAN acknowledgements, or motor execution times.
    """
    candidates = np.arange(max_lag_ms + 1, dtype=np.int64) * 1_000_000
    output = []
    for j, axis in enumerate(selected):
        train_scores = []
        records = []
        for lag in candidates:
            observations = {1: ([], []), 2: ([], [])}
            for run in runs:
                raw = np.arange(run[0], run[-1] + 1)
                raw = raw[(a["tx_kind"][raw, axis] == 1) & (a["tx_queued_steady_ns"][raw, axis] > 0)]
                if not len(raw):
                    continue
                times = a["tx_queued_steady_ns"][raw, axis]
                # Out-of-order submissions are not reconstructed.
                if np.any(times[1:] <= times[:-1]):
                    continue
                for i in run:
                    fb = int(a["feedback_steady_ns"][i, axis])
                    t = fb - int(lag)
                    if t < 0:
                        continue
                    p = int(np.searchsorted(times, t, side="right") - 1)
                    if p < 0 or not np.isfinite(a["tau_frame_model"][raw[p], axis]):
                        continue
                    code = int(a["split"][i])
                    observations[code][0].append(a["tau_frame_model"][raw[p], axis])
                    observations[code][1].append(a["torque_fb_model"][i, axis])
            records.append(observations)
            command, feedback = map(np.asarray, observations[1])
            if len(command) < 30 or np.std(command) < 1e-3 or np.std(feedback) < 1e-3:
                train_scores.append(float("inf"))
            else:
                # Centered normalized correlation permits differing *reported*
                # scales without claiming to identify a motor torque constant.
                correlation = float(np.corrcoef(command, feedback)[0, 1])
                train_scores.append(1 - correlation ** 2 if correlation > 0 else 1.)
        best = int(np.argmin(train_scores))
        if not np.isfinite(train_scores[best]) or np.sum(np.isclose(train_scores, train_scores[best], atol=1e-4)) > 2:
            output.append({"axis": j, "effective_lag_ms": None, "reason": "insufficient excitation or aliased lag grid"})
            continue
        train_x, train_y = map(np.asarray, records[best][1])
        held_x, held_y = map(np.asarray, records[best][2])
        if len(held_x) < 10:
            output.append({"axis": j, "effective_lag_ms": None, "reason": "insufficient held-out alignment"})
            continue
        slope = float(np.cov(train_x, train_y, bias=True)[0, 1] / np.var(train_x))
        offset = float(train_y.mean() - slope * train_x.mean())
        score = metrics(held_y, offset + slope * held_x)
        if (train_scores[best] > .2 or score["r2"] is None or score["r2"] < .5):
            output.append({"axis": j, "effective_lag_ms": None,
                           "reason": "held-out host-queue alignment failed", "holdout_reported_torque": score})
            continue
        output.append({"axis": j, "effective_lag_ms": int(best),
                       "train_correlation_loss": float(train_scores[best]),
                       "holdout_reported_torque": score,
                       "qualification": "host submission -> CAN receive alignment only; no execution acknowledgement"})
    return {"grid_ms": [0, max_lag_ms, 1], "per_axis": output}


def environment(a: dict, selected: tuple[int, int]) -> dict:
    pairs = a["pair_valid"]
    def extent(values: np.ndarray) -> dict:
        values = np.asarray(values).ravel()
        values = values[np.isfinite(values)]
        return {"count": len(values), "min": float(values.min()) if len(values) else None,
                "max": float(values.max()) if len(values) else None}
    return {"actuation_scope_codes_all": {str(k): int(np.sum(a["actuation_scope"] == k)) for k in (0, 1, 2)},
            "selected_command_only_not_physical_disable_proof": True,
            "temperature_c_selected_pairs": extent(a["temperature_c"][pairs][:, selected]) if "temperature_c" in a else None,
            "supply_voltage_v_selected_pairs": extent(a["supply_voltage_v"][pairs]) if "supply_voltage_v" in a else None}


def interval_qc(a: dict, selected: tuple[int, int]) -> dict:
    """Count rejected raw ticks between the first/last accepted pair in each instance."""
    active = np.zeros(len(a["pair_valid"]), dtype=bool)
    pairs = np.flatnonzero(a["pair_valid"])
    for instance in np.unique(a["segment_instance"][pairs]):
        indices = pairs[a["segment_instance"][pairs] == instance]
        active[indices[0]:indices[-1]+1] = True
    flags = {"side_invalid": ~a["side_valid"],
             "sequence_gap": np.any(a["axis_sequence_gap"][:, selected], axis=1),
             "encoder_ambiguous": np.any(a["axis_encoder_ambiguous"][:, selected], axis=1),
             "pair_skew_fail": a["pair_skew_fail"], "pair_age_fail": a["pair_age_fail"]}
    counts = {key: int(np.count_nonzero(active & flag)) for key, flag in flags.items()}
    return {"raw_ticks_between_accepted_pairs": int(active.sum()), "excluded_fault_ticks": counts}


def cascade_consistency(a: dict, selected: tuple[int, int], gains: dict) -> dict:
    """Check the logged 200 Hz PC cascade against the SHA-matched run gains.

    Check only update ticks, not a later held reference combined with a newer
    asynchronous CAN observation. Integral torque is *observed*, not inferred
    from a position reference or an invented controller state.
    """
    mask = ((a["tick"] % 5 == 0) & a["side_valid"] & a["double_loop_observed"]
            & (a["phase"] == 2) & (a["segment_id"] >= 0))
    indices = np.flatnonzero(mask)
    if not len(indices):
        return {"update_ticks": 0, "velocity_target_rmse_rad_s": None,
                "torque_preclip_rmse_nm": None}
    def sample(name: str) -> np.ndarray:
        return a[name][indices][:, selected]
    kp_p = np.asarray(gains["kp_position"])[list(selected)]
    kp_v = np.asarray(gains["kp_velocity"])[list(selected)]
    ki = np.asarray(gains["ki_velocity"])[list(selected)]
    integral_limit = np.asarray(gains["integral_limit"])[list(selected)]
    speed_max = np.asarray(gains["max_speed"])[list(selected)]
    target = np.clip(sample("dq_ref_model") + kp_p * (sample("q_ref_model") - sample("q_model")),
                     -speed_max, speed_max)
    integral = sample("torque_integral_model")
    expected_tau = kp_v * (sample("velocity_target_model") - sample("dq_model")) + integral
    finite = np.isfinite(target).all(axis=1) & np.isfinite(expected_tau).all(axis=1)
    finite &= np.isfinite(integral).all(axis=1)
    if not np.all(finite):
        return {"update_ticks": len(indices), "velocity_target_rmse_rad_s": None,
                "torque_preclip_rmse_nm": None}
    bounded_integral = bool(np.all(np.abs(integral) <= ki * integral_limit + 1e-5))
    return {"update_ticks": len(indices), "velocity_target_rmse_rad_s":
            [m["rmse"] for m in metrics_by_axis(sample("velocity_target_model"), target)],
            "torque_preclip_rmse_nm":
            [m["rmse"] for m in metrics_by_axis(sample("tau_preclip_model"), expected_tau)],
            "logged_integral_within_yaml_limits": bounded_integral}


def evaluate(npz_path: Path, profile: Path, output: Path, *, max_pair_dt_ms: float = 12.,
             window_ms: float = 200., tanh_gain: float = 30., max_lag_ms: int = 25,
             max_condition: float = 1e4, min_train: int = 40, min_holdout: int = 10,
             min_r2: float = .5, max_torque_rmse_nm: float = 1.5,
             max_q_rmse_rad: float = .10, max_dq_rmse_rad_s: float = 1.) -> dict:
    if (min_train < 3 or min_holdout < 3 or not 0 <= max_lag_ms <= 100 or window_ms < max_pair_dt_ms
        or not all(np.isfinite(x) and x > 0 for x in
                   (max_pair_dt_ms, window_ms, tanh_gain, max_condition, max_torque_rmse_nm,
                    max_q_rmse_rad, max_dq_rmse_rad_s)) or not -1 <= min_r2 <= 1):
        raise ValueError("invalid fitting or qualification thresholds")
    a, meta, gains = load_run(npz_path, profile)
    runs, selected = checked_run(a, meta, max_pair_dt_ms * 1e-3)
    cascade = cascade_consistency(a, selected, gains)
    qc = interval_qc(a, selected)
    thresholds = {"min_training_triplets": min_train, "min_holdout_triplets": min_holdout,
                  "min_torque_r2_per_axis": min_r2, "max_torque_rmse_nm_per_axis": max_torque_rmse_nm,
                  "max_local_q_rmse_rad_per_axis": max_q_rmse_rad,
                  "max_local_dq_rmse_rad_s_per_axis": max_dq_rmse_rad_s,
                  "max_scaled_design_condition": max_condition,
                  "max_logged_velocity_target_rmse_rad_s_per_axis": .10,
                  "max_logged_preclip_rmse_nm_per_axis": .4}
    try:
        model, ranks, ids, tau_prediction = fit_model(a, runs, selected, tanh_gain, max_condition, min_train)
    except UnidentifiableError as error:
        report = {"input_npz": str(npz_path), "profile_sha256": meta["profile_sha256"],
                  "calibration_sha256": meta["calibration_sha256"], "side": meta["side"],
                  "selected_axes": list(selected), "controller_from_saved_run_yaml": gains,
                  "logged_pc_cascade_consistency": cascade, "observability": error.ranks,
                  "train_only_fit": True, "hardware_scope_and_conditions": environment(a, selected),
                  "interval_qc": qc,
                  "qualification_thresholds": thresholds, "qualification_failures": [str(error)],
                  "curve_and_residual_npz": None, "candidate_simulator_only": None}
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        return report
    target = a["torque_fb_model"][ids][:, selected]
    torque = {}
    for label, code in (("train", 1), ("holdout", 2)):
        mask = a["split"][ids] == code
        torque[label] = {"per_axis": metrics_by_axis(target[mask], tau_prediction[mask]),
                         "segments": sorted(set(map(int, a["segment_id"][ids[mask]]))),
                         "by_segment": {}}
        for segment in torque[label]["segments"]:
            group = mask & (a["segment_id"][ids] == segment)
            torque[label]["by_segment"][str(segment)] = metrics_by_axis(target[group], tau_prediction[group])
    replays, curves = replay(a, runs, selected, model, window_ms * 1e-3)
    reasons = []
    raw_running = (a["phase"] == 2) & (a["segment_id"] >= 0)
    if np.any(a["actuation_scope"][raw_running] != 1):
        reasons.append("running ticks do not all have selected-side-only submitted commands")
    if any(qc["excluded_fault_ticks"].values()):
        reasons.append("QC failures between accepted pairs; no interpolation over these intervals")
    if (cascade["update_ticks"] < 10 or cascade["velocity_target_rmse_rad_s"] is None
        or cascade["torque_preclip_rmse_nm"] is None
        or not cascade.get("logged_integral_within_yaml_limits", False)
        or max(cascade["velocity_target_rmse_rad_s"] or [math.inf]) > .10
        or max(cascade["torque_preclip_rmse_nm"] or [math.inf]) > .4):
        reasons.append("recorded 200 Hz PC cascade disagrees with saved run YAML or lacks update observations")
    for label in ("train", "holdout"):
        required = min_train if label == "train" else min_holdout
        if min(m["count"] for m in torque[label]["per_axis"]) < required:
            reasons.append(f"{label}: insufficient derivative observations")
        for j in range(2):
            t = torque[label]["per_axis"][j]
            q = replays[label]["q_rad"][j]
            v = replays[label]["dq_rad_s"][j]
            if (t["r2"] is None or t["r2"] < min_r2 or t["rmse"] > max_torque_rmse_nm):
                reasons.append(f"{label} axis {j}: torque residual threshold failed")
            if q["count"] < required or q["rmse"] > max_q_rmse_rad or v["rmse"] > max_dq_rmse_rad_s:
                reasons.append(f"{label} axis {j}: causal replay threshold failed")
    # A strong held-out segment must not conceal a different failing one.
    for segment, label in meta["segment_split"].items():
        if label == "holdout" and segment not in torque["holdout"]["by_segment"]:
            reasons.append(f"holdout segment {segment}: no valid derivative/replay observations")
    for segment, per_axis in torque["holdout"]["by_segment"].items():
        replay_segment = replays["holdout"]["by_segment"].get(segment)
        for j, t in enumerate(per_axis):
            if (t["count"] < min_holdout or t["r2"] is None or t["r2"] < min_r2
                or t["rmse"] > max_torque_rmse_nm):
                reasons.append(f"holdout segment {segment} axis {j}: torque residual threshold failed")
            if (replay_segment is None or replay_segment["q_rad"][j]["count"] < min_holdout
                or replay_segment["q_rad"][j]["rmse"] > max_q_rmse_rad
                or replay_segment["dq_rad_s"][j]["rmse"] > max_dq_rmse_rad_s):
                reasons.append(f"holdout segment {segment} axis {j}: causal replay threshold failed")
    curves.update(torque_row=ids, torque_feedback_observed_nm=target,
                  torque_feedback_predicted_nm=tau_prediction, torque_residual_nm=target-tau_prediction)
    lag = alignment(a, runs, selected, max_lag_ms)
    curve_path = output.with_suffix(".replay.npz")
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(curve_path, **curves)
    report = {"input_npz": str(npz_path), "profile_sha256": meta["profile_sha256"],
              "calibration_sha256": meta["calibration_sha256"], "side": meta["side"],
              "selected_axes": list(selected), "controller_from_saved_run_yaml": gains,
              "model": model, "observability": ranks, "train_only_fit": True,
              "logged_pc_cascade_consistency": cascade,
              "torque_feedback_residuals": torque, "causal_local_open_loop_replay": replays,
              "replay_window_ms": window_ms, "curve_and_residual_npz": str(curve_path),
               "command_to_feedback_alignment": lag,
               "hardware_scope_and_conditions": environment(a, selected), "interval_qc": qc,
              "qualification_thresholds": thresholds, "qualification_failures": reasons,
               "candidate_simulator_only": ({**model,
                                              "command_to_feedback_effective_lag_ms":
                                              [item["effective_lag_ms"] for item in lag["per_axis"]],
                                              "requires": "PhysX and real hardware validation before manual use",
                                             "auto_update_training": False} if not reasons else None),
              "limitations": ["local effective closed-chain model around recorded pose; static load mixes spring, gravity and offsets",
                              "torque feedback is not independently measured joint torque or motor torque constant",
                              "CAN receive stamps are asynchronous and host TX queue is not motor execution time",
                              "local replay uses prior measured torque, not a closed-loop controller simulation; 200 Hz PC cascade telemetry is cross-checked against saved YAML gains"]}
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--npz", type=Path, required=True, help="export_identification_bag.py output")
    parser.add_argument("--profile", type=Path, required=True, help="saved, SHA-matched run profile YAML")
    parser.add_argument("--output", type=Path, required=True, help="JSON report (also writes .replay.npz)")
    parser.add_argument("--max-pair-dt-ms", type=float, default=12.)
    parser.add_argument("--replay-window-ms", type=float, default=200.)
    parser.add_argument("--tanh-gain", type=float, default=30.)
    parser.add_argument("--max-lag-ms", type=int, default=25)
    parser.add_argument("--max-condition", type=float, default=1e4)
    parser.add_argument("--min-train", type=int, default=40)
    parser.add_argument("--min-holdout", type=int, default=10)
    parser.add_argument("--min-r2", type=float, default=.5)
    parser.add_argument("--max-torque-rmse-nm", type=float, default=1.5)
    parser.add_argument("--max-q-rmse-rad", type=float, default=.10)
    parser.add_argument("--max-dq-rmse-rad-s", type=float, default=1.)
    args = parser.parse_args()
    if args.output.suffix != ".json" or args.npz.suffix != ".npz":
        parser.error("--npz needs .npz and --output needs .json")
    result = evaluate(args.npz, args.profile, args.output,
                      max_pair_dt_ms=args.max_pair_dt_ms, window_ms=args.replay_window_ms,
                      tanh_gain=args.tanh_gain, max_lag_ms=args.max_lag_ms,
                      max_condition=args.max_condition, min_train=args.min_train,
                      min_holdout=args.min_holdout, min_r2=args.min_r2,
                      max_torque_rmse_nm=args.max_torque_rmse_nm,
                      max_q_rmse_rad=args.max_q_rmse_rad, max_dq_rmse_rad_s=args.max_dq_rmse_rad_s)
    print(json.dumps({"report": str(args.output), "curves": result["curve_and_residual_npz"],
                      "candidate_simulator_only": result["candidate_simulator_only"] is not None,
                      "qualification_failures": result["qualification_failures"]}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
