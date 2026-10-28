#!/usr/bin/env python3
"""Fit effective LEFT-leg gas holding torque from horizontal, unloaded sweeps.

Training and validation half-cycles stay separate. An old-recording-only fit is
also transferred to the new P/P recording before that recording enters the final
candidate fit. The output is an offline motor-coordinate load map, not a unique
gas pressure/force model or a prediction of compensated closed-loop tracking.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pandas as pd

from analyze_wheel_leg_sweep_archive import sha256

S = "/wheel_leg/sweep/"
H, K = S + "left_hip_joint/", S + "left_knee_joint/"
BASE_KNOTS = np.array([60, 65, 70, 75, 80, 85, 90, 94, 96, 97, 98, 99, 100, 101.])
CANDIDATE_KNOTS = np.r_[56., 58., BASE_KNOTS]
NEEDED = [S + c for c in ("state", "reference_active", "run_id", "side", "mode",
                          "stage", "validation", "control_steady_ns", "control_dt_s",
                          "left/beta_estimate", "left/beta_valid")]
for prefix, axis in ((H, "left_hip_joint"), (K, "left_knee_joint")):
    NEEDED += [prefix + "model_velocity", prefix + "saturated",
               "/wheel_leg/" + axis + "/torque"]


def load_source(path: Path, cache: Path | None):
    """Use a source-hash-checked numeric cache or audit the original CSV first."""
    digest = sha256(path)
    if cache is not None:
        for item in cache.glob("*.json"):
            meta = json.loads(item.read_text())
            if meta.get("path") == str(path) and meta.get("sha256") == digest:
                frame = pd.read_pickle(item.with_suffix(".pkl"))
                assert len(frame) == meta["records"]
                return frame[NEEDED].copy(), {**meta, "reader": "source-hash-checked numeric cache"}
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        bad, rows = [], 0
        for number, row in enumerate(reader, 2):
            rows += 1
            if len(row) != len(header):
                bad.append((number, len(row)))
    # Interrupted recording may leave one truncated final line. Never silently
    # accept a malformed interior row or a row with extra shifted fields.
    assert not bad or (len(bad) == 1 and bad[0][0] == rows + 1 and bad[0][1] < len(header)), bad
    frame = pd.read_csv(path, usecols=NEEDED, skiprows=[n - 1 for n, _ in bad])
    assert len(frame) == rows - len(bad)
    return frame, dict(path=str(path), sha256=digest, bytes=path.stat().st_size,
                       columns=len(header), records=len(frame),
                       malformed_lines_excluded=[n for n, _ in bad], reader="audited source CSV")


def prepare(frame, stages):
    selected = frame[(frame[S + "state"] == 3) & (frame[S + "reference_active"] == 1)
                     & (frame[S + "run_id"] == 1) & frame[S + "stage"].isin(stages)]
    assert selected[S + "side"].eq(0).all() and selected[S + "mode"].eq(2).all()
    result = []
    for stage, group in selected.groupby(S + "stage"):
        time = group[S + "control_steady_ns"].to_numpy(float) * 1e-9
        assert np.all(np.diff(time) > 0)
        x = pd.DataFrame(dict(
            time_s=time, beta_deg=np.rad2deg(group[S + "left/beta_estimate"].to_numpy(float)),
            hip_v=group[H + "model_velocity"].to_numpy(float),
            knee_v=group[K + "model_velocity"].to_numpy(float),
            # Both source profiles use q_model=-q_API+offset. Power invariance
            # therefore gives tau_model=-tau_API, with no additional gear factor.
            hip_torque_nm=-group["/wheel_leg/left_hip_joint/torque"].to_numpy(float),
            knee_torque_nm=-group["/wheel_leg/left_knee_joint/torque"].to_numpy(float)))
        bad = (~group[S + "left/beta_valid"].eq(1).to_numpy()
               | group[H + "saturated"].ne(0).to_numpy()
               | group[K + "saturated"].ne(0).to_numpy())
        for gap in np.flatnonzero(np.diff(time) > .002) + 1:
            bad |= abs(time - time[gap]) < .12
        for column in ("beta_deg", "hip_v", "knee_v", "hip_torque_nm", "knee_torque_nm"):
            x[column] = x[column].rolling(51, center=True, min_periods=51).mean()
        x["relative_velocity_rad_s"] = x.knee_v - x.hip_v
        acceleration = np.gradient(x.relative_velocity_rad_s.to_numpy(), time)
        x["relative_acceleration_rad_s2"] = pd.Series(acceleration).rolling(
            101, center=True, min_periods=51).mean()
        x["relative_holding_torque_nm"] = (x.hip_torque_nm - x.knee_torque_nm) / 2
        x["common_torque_nm"] = (x.hip_torque_nm + x.knee_torque_nm) / 2
        x = x[~bad].dropna().iloc[::20].copy()
        x["stage"] = int(stage)
        x["validation"] = int(group[S + "validation"].iloc[0])
        result.append(x)
    return pd.concat(result, ignore_index=True)


def screen(frame, knots):
    return frame[frame.beta_deg.between(knots[0], knots[-1])
                 & frame.relative_velocity_rad_s.abs().gt(.06)
                 & frame.relative_acceleration_rad_s2.abs().lt(3.)
                 & frame.hip_v.abs().lt(.18)].copy()


def basis(beta, knots):
    beta = np.asarray(beta, float)
    assert np.isfinite(beta).all() and np.all((beta >= knots[0]) & (beta <= knots[-1]))
    interval = np.clip(np.searchsorted(knots, beta, side="right") - 1, 0, len(knots) - 2)
    fraction = (beta - knots[interval]) / (knots[interval + 1] - knots[interval])
    matrix = np.zeros((len(beta), len(knots)))
    matrix[np.arange(len(beta)), interval] = 1 - fraction
    matrix[np.arange(len(beta)), interval + 1] = fraction
    return matrix


def fit(training, knots):
    training = training.copy()
    assert training.validation.eq(0).all()
    training["angle_bin_deg"] = np.round(training.beta_deg * 2) / 2
    training["direction"] = np.sign(training.relative_velocity_rad_s)
    # First average within a half-cycle, then across repeated half-cycles. Each
    # source/angle/direction receives equal weight, regardless of dwell time.
    columns = ["beta_deg", "relative_holding_torque_nm"]
    passes = training.groupby(["source", "stage", "angle_bin_deg", "direction"])[columns].mean().reset_index()
    cells = passes.groupby(["source", "angle_bin_deg", "direction"])[columns].mean().reset_index()
    b = basis(cells.beta_deg, knots)
    matrix = np.column_stack((b, -b * cells.direction.to_numpy()[:, None]))
    coeff, _, rank, _ = np.linalg.lstsq(matrix, cells.relative_holding_torque_nm, rcond=None)
    assert rank == matrix.shape[1]
    gas, width = coeff[:len(knots)], coeff[len(knots):]
    assert np.all(gas > 0) and np.all(width >= 0), (gas, width)
    return dict(beta_deg=knots.tolist(), holding_torque_per_motor_model_nm=gas.tolist(),
                direction_half_width_nm=width.tolist(), rows=len(training),
                source_angle_direction_cells=len(cells), repeated_pass_cells=len(passes),
                matrix_condition_number=float(np.linalg.cond(matrix))), cells


def predict(frame, model):
    b = basis(frame.beta_deg, np.asarray(model["beta_deg"]))
    static = b @ np.asarray(model["holding_torque_per_motor_model_nm"])
    width = b @ np.asarray(model["direction_half_width_nm"])
    # delta=q_knee-q_hip; T=(tau_hip-tau_knee)/2. Positive relative
    # velocity lowers the required T because motion-opposing effort adds to knee.
    moving = static - width * np.sign(frame.relative_velocity_rad_s.to_numpy())
    return static, moving


def stats(values):
    values = np.asarray(values, float)
    assert np.isfinite(values).all() and len(values)
    return dict(rmse=float(np.sqrt(np.mean(values ** 2))), bias=float(values.mean()),
                mae=float(np.mean(abs(values))), p95_abs=float(np.quantile(abs(values), .95)),
                max_abs=float(abs(values).max()))


def evaluate(frame, model):
    static, moving = predict(frame, model)
    actual = frame.relative_holding_torque_nm.to_numpy()
    h, k = frame.hip_torque_nm.to_numpy(), frame.knee_torque_nm.to_numpy()
    return dict(rows=len(frame), stages=sorted(int(s) for s in frame.stage.unique()),
                beta_range_deg=[float(frame.beta_deg.min()), float(frame.beta_deg.max())],
                relative_holding_before_nm=stats(actual),
                relative_residual_static_only_nm=stats(actual - static),
                relative_residual_static_and_direction_nm=stats(actual - moving),
                common_torque_nm=stats(frame.common_torque_nm),
                joint_effort_before_nm=dict(hip=stats(h), knee=stats(k), pair=stats(np.r_[h, k])),
                joint_effort_minus_static_map_nm=dict(hip=stats(h-static), knee=stats(k+static),
                                                    pair=stats(np.r_[h-static, k+static])),
                joint_effort_minus_static_and_direction_nm=dict(
                    hip=stats(h-moving), knee=stats(k+moving), pair=stats(np.r_[h-moving, k+moving])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-csv", type=Path, required=True)
    parser.add_argument("--pp-csv", type=Path, required=True)
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    old, old_meta = load_source(args.old_csv, args.cache_dir)
    new, new_meta = load_source(args.pp_csv, args.cache_dir)
    old = prepare(old, range(1, 37))
    new = prepare(new, range(1, 12))  # Stage 12 was interrupted after 0.152 s.
    old["source"], new["source"] = "old_pd", "new_pp"
    old_slow = old[old.stage.le(12)]
    old_train = old_slow[old_slow.validation.eq(0)]
    old_val = old_slow[old_slow.validation.eq(1)]
    new_train = new[new.validation.eq(0)]
    new_val = new[new.validation.eq(1)]
    assert set(old_train.stage) == set(range(1, 7))
    assert set(old_val.stage) == set(range(7, 13))
    assert set(new_train.stage) == set(range(1, 7))
    assert set(new_val.stage) == set(range(7, 12))

    transfer, _ = fit(screen(old_train, BASE_KNOTS), BASE_KNOTS)
    # The transfer audit below uses no samples from the new recording for fitting.
    transfer_sets = {"old_slow_holdout": old_val, "new_pp_external": new,
                     "old_medium_screened": old[old.stage.between(13, 24)],
                     "old_fast_screened": old[old.stage.between(25, 36)]}
    transfer_metrics = {name: evaluate(screen(f, BASE_KNOTS), transfer)
                        for name, f in transfer_sets.items()}
    combined_train = pd.concat([old_train, new_train], ignore_index=True)
    candidate, cells = fit(screen(combined_train, CANDIDATE_KNOTS), CANDIDATE_KNOTS)
    val_sets = {"old_slow_holdout": old_val, "new_pp_holdout": new_val}
    candidate_metrics = {name: evaluate(screen(f, CANDIDATE_KNOTS), candidate)
                         for name, f in val_sets.items()}
    coverage = {}
    for name, f in {"old_slow": old_slow, "new_pp": new}.items():
        inside = f.beta_deg.between(CANDIDATE_KNOTS[0], CANDIDATE_KNOTS[-1])
        coverage[name] = dict(prepared_rows=len(f), angle_domain_fraction=float(inside.mean()),
                              low_dynamic_rows=len(screen(f, CANDIDATE_KNOTS)),
                              prepared_beta_range_deg=[float(f.beta_deg.min()), float(f.beta_deg.max())])
    result = dict(
        status="offline_effective_left_leg_holding_map_not_hardware_validated",
        sources={"old_pd": old_meta, "new_pp": new_meta},
        conditions="User confirms horizontal suspended leg; no external load or gravity torque.",
        coordinates=dict(beta="logged actual FK included angle, deg, current sweep calibration",
                         delta="q_knee_model-q_hip_model",
                         relative_holding="T=(tau_hip_model-tau_knee_model)/2",
                         static_feedforward_model="[+T(beta), -T(beta)] for LEFT [hip,knee]",
                         static_feedforward_api="[-T(beta), +T(beta)] for the logged model_sign=[-1,-1]",
                         direction="T_required=T_static-W(beta)*sign(delta_dot)",
                         physical_knee_torque="T_gas_beta=T_static*d(delta)/d(beta); radians in both derivatives"),
        processing=dict(smoothing="51-sample centered torque/velocity/angle mean (~51 ms)",
                        acceleration="gradient of smoothed relative velocity, 101-sample centered mean",
                        reduction="one in every 20 valid smoothed samples (~20 ms)",
                        gap_exclusion="exclude +/-120 ms about control gaps >2 ms",
                        screen=dict(min_relative_speed_rad_s=.06, max_abs_relative_acceleration_rad_s2=3.,
                                    max_abs_hip_speed_rad_s=.18),
                        fit="0.5-degree angle cells, equal source/angle/direction weights after pass averaging; linear interpolation",
                        use_torque="reported motor torque, not command or an independent torque sensor"),
        coverage=coverage, transfer_model=transfer, transfer_validation=transfer_metrics,
        candidate_model=candidate, candidate_validation=candidate_metrics,
        proposed_feedforward=dict(enabled=False, side="left", interpolation="piecewise linear",
                                  angle_domain_deg=[56., 101.], out_of_domain="unsupported; do not extrapolate",
                                  static_map_only=True, direction_compensation_enabled=False,
                                  per_motor_final_total_torque_limit_nm=40.),
        limitations=[
            "The mean of forward/reverse required torque estimates effective average spring load; inertia, thermal history, seal hysteresis and drivetrain friction are not uniquely separated.",
            "W is a direction-dependent motor-coordinate load half-width; it is not an independently identified pure gas seal or Coulomb friction coefficient.",
            "Centered smoothing and acceleration estimates are offline-only processing, not an implementation for a real-time observer.",
            "Residual effort subtraction evaluates recorded states. It does not simulate or measure tracking after feedforward is enabled.",
            "The candidate includes only stages1..6 of each recording; stages7+ remain held out. New recording has only 2.5 holdout cycles.",
            "The 56..101 degree interval is identification coverage, not the mechanical limit range or permission to extrapolate to 35..110 degrees.",
            "The high-angle steep load change is empirical; its mechanical cause and a catalogue force-pressure-stroke model are not identified.",
            "No right-leg dataset, loaded/grounded validation, compensated closed-loop experiment, accepted inertia/damping, or pure actuator delay is supplied by this fit.",
            "No runtime controller, motor command, RL actuator contract or simulator force model is changed by this script.",
        ])
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "model.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    table = pd.DataFrame(dict(beta_deg=candidate["beta_deg"],
                              holding_torque_per_motor_model_nm=candidate["holding_torque_per_motor_model_nm"],
                              direction_half_width_nm=candidate["direction_half_width_nm"]))
    table.to_csv(args.out / "left_gas_holding_map.csv", index=False)
    cells.to_csv(args.out / "training_angle_direction_cells.csv", index=False)
    held = screen(new_val, CANDIDATE_KNOTS)
    static, moving = predict(held, candidate)
    np.savez_compressed(args.out / "plot_data.npz", train_beta=cells.beta_deg,
                        train_torque=cells.relative_holding_torque_nm, train_direction=cells.direction,
                        val_time=held.time_s.to_numpy()-held.time_s.iloc[0],
                        val_actual=held.relative_holding_torque_nm, val_static=static, val_direction=moving)
    print(json.dumps(dict(table=table.round(4).to_dict("records"),
                          transfer_validation=transfer_metrics,
                          candidate_validation=candidate_metrics, coverage=coverage),
                     indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
