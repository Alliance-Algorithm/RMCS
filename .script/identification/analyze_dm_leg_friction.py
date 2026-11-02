#!/usr/bin/env python3
"""Identify moving DM output friction after subtracting gas-spring holding torque.

Preparation reads original CSVs with the bundled Pandas runtime. Analysis uses
the scientific Python runtime for SciPy/Matplotlib. No robot or training process
is started, and no controller/model default is changed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
S = "/wheel_leg/sweep/"
BASE = Path("/home/noir/Documents/workspace/wheel_leg扫频")
SOURCES = [
    dict(name="first", file="左腿整腿转动，覆盖低速和约 2 秒一圈.csv",
         sha256="cd6db89bc493d5f0ddfb84ef775c4f9468429e511a7b2cd7dee355d34c5a196c",
         excluded_lines=[245805]),
    dict(name="repeat", file="左腿整腿转动，覆盖低速和约 2 秒一圈（2）.csv",
         sha256="8f7e2c78639e917b8d7bb33956d904876598509ad18f9d2d20fe98170c6aff50",
         excluded_lines=[]),
    dict(name="angles", file="左腿关节不同夹角扫频.csv",
         sha256="da4a6f2e3286870f2646639dad9f665244a086d829863801cc8ad32b44766597",
         excluded_lines=[]),
]
PAIRS = {
    "speed_020": (1, 2, .2), "speed_060": (4, 5, .6),
    "speed_150": (7, 8, 1.5), "speed_314": (10, 11, np.pi),
}
ANGLE_PAIRS = {
    "angle_ref50": (4, 5, np.pi), "angle_ref75": (9, 10, np.pi),
    "angle_ref100": (14, 15, np.pi), "angle_ref60": (17, 18, np.pi),
    "angle_ref90": (20, 21, np.pi),
}


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def header_array(text, name, width):
    match = re.search(r"\b" + name + r"\{\{(.*?)\}\};", text, re.S)
    if match is None:
        raise ValueError(f"Cannot locate generated array {name}")
    rows = [[float(x.strip()) for x in row.split(",")]
            for row in re.findall(r"\{([^{}]+)\}", match[1])]
    result = np.array(rows)
    assert result.ndim == 2 and result.shape[1] == width
    assert np.all(np.diff(result[:, 0]) > 0)
    return result


def current_gas_model():
    folder = ROOT / "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg"
    header = folder / "full_vehicle_v202.hpp"
    calibration_path = ROOT / "rmcs_ws/src/rmcs_bringup/config/contracts/wheel_leg_calibration110.json"
    calibration = json.loads(calibration_path.read_text())
    text = header.read_text()
    geometry = header_array(text, "kLeft", 5)
    extra = np.array(calibration["gas_geometry_extension"]["left"])
    assert extra[0, 0] > geometry[-1, 0]
    geometry = np.vstack((geometry, extra))
    force = header_array(text, "kForce", 2)
    scalar = lambda name: float(re.search(name + r"\s*=\s*([^;]+);", text)[1])
    return dict(geometry=geometry.tolist(), force=force.tolist(),
                low_rate=scalar("kLowCompressionRate"), high_slope=scalar("kHighCompressionSlope"),
                offsets=calibration["model_offset_rad"][:2],
                sources={str(p): digest(p) for p in [header, calibration_path,
                    folder / "full_vehicle_calibration110.hpp", folder / "gas_spring_compensation.hpp"]})


def archive_gas_model():
    path = ROOT / ".script/identification/calibration/gas_spring_compensation_20261028.json"
    model = json.loads(path.read_text())
    side = model["sides"]["left"]
    geometry = np.column_stack((side["delta_rad"], side["beta_deg"], side["compression_m"],
                               side["length_jacobian_m_per_rad"]))
    return dict(geometry=geometry.tolist(),
                force=np.column_stack((model["compression_knots_m"], model["force_knots_n"])).tolist(),
                low_rate=model["force_extension"]["low_compression_rate_per_m"],
                high_slope=model["force_extension"]["high_compression_slope_n_per_m"],
                offsets=[1.6, 2.93], sources={str(path): digest(path)})


def gas_torque(model, delta):
    geometry, knots = np.asarray(model["geometry"]), np.asarray(model["force"])
    valid = (delta >= geometry[0, 0]) & (delta <= geometry[-1, 0])
    values = [np.interp(delta, geometry[:, 0], geometry[:, i]) for i in range(1, 5)]
    beta, compression, jh, jk = values
    force = np.interp(compression, knots[:, 0], knots[:, 1])
    low, high = compression < knots[0, 0], compression > knots[-1, 0]
    force[low] = knots[0, 1] * np.exp(model["low_rate"] * (compression[low] - knots[0, 0]))
    force[high] = knots[-1, 1] + model["high_slope"] * (compression[high] - knots[-1, 0])
    # Same holding-effort sign as gas_spring_compensation.hpp. Measured motor
    # effort minus this value leaves friction, inertia, and model error.
    return beta, np.where(valid, -force * jh, np.nan), np.where(valid, -force * jk, np.nan)


def prepare(out):
    import pandas as pd

    current, archive = current_gas_model(), archive_gas_model()
    write_json(out / "gas_models.json", dict(current=current, archive=archive))
    common = [S + x for x in ["state", "reference_active", "side", "run_id", "stage",
              "waveform", "control_steady_ns", "stage_elapsed_s", "stage_duration_s",
              "left/beta_estimate", "left/beta_valid", "drives_ready"]]
    common += ["/wheel_leg/feedback_fresh", "/wheel_leg/dm_feedback_fresh"]
    for axis in ("hip", "knee"):
        prefix = f"/wheel_leg/left_{axis}_joint/"
        common += [prefix + x for x in ["angle", "velocity", "torque", "feedback_sequence",
                   "feedback_steady_ns", "fault_code", "status_code"]]
        common += [S + f"left_{axis}_joint/" + x for x in ["model_angle", "model_velocity", "saturated"]]
    frames, provenance, excluded = [], [], []
    for source_id, source in enumerate(SOURCES):
        path = BASE / source["file"]
        actual_hash = digest(path)
        assert actual_hash == source["sha256"], f"Source changed: {path}"
        frame = pd.read_csv(path, usecols=common,
                            skiprows=[line - 1 for line in source["excluded_lines"]])
        records = len(frame)
        # Waveform enum changed between recording revisions: the early common
        # rotations are 2, and the later multi-angle common rotations are 4.
        waveform = 2 if source_id < 2 else 4
        active = ((frame[S+"state"] == 3) & (frame[S+"reference_active"] == 1)
                  & (frame[S+"side"] == 0) & (frame[S+"waveform"] == waveform))
        frame = frame[active].copy()
        pairs = PAIRS if source_id < 2 else ANGLE_PAIRS
        stage_counts = []
        for run, run_frame in frame.groupby(S+"run_id"):
            for case, (a, b, nominal_speed) in pairs.items():
                stages = [run_frame[run_frame[S+"stage"] == stage].copy() for stage in (a, b)]
                # The second multi-angle run ends in stage 20; do not treat an
                # incomplete one-way movement as a forward/reverse experiment.
                if any(len(g) < 100 or g[S+"stage_elapsed_s"].max() < .95 * g[S+"stage_duration_s"].iloc[0]
                       for g in stages):
                    excluded.append(dict(source=source["name"], run=int(run), case=case,
                                         reason="missing or incomplete forward/reverse stage"))
                    continue
                for group in stages:
                    t = group[S+"control_steady_ns"].to_numpy(float) * 1e-9
                    assert np.all(np.diff(t) > 0)
                    x = pd.DataFrame(dict(time_s=t, elapsed_s=group[S+"stage_elapsed_s"].to_numpy(float),
                        duration_s=group[S+"stage_duration_s"].to_numpy(float),
                        beta_log_deg=np.rad2deg(group[S+"left/beta_estimate"].to_numpy(float))))
                    bad = ((group[S+"left/beta_valid"].to_numpy() != 1)
                           | (group[S+"drives_ready"].to_numpy() != 1)
                           | (group["/wheel_leg/feedback_fresh"].to_numpy() != 1)
                           | (group["/wheel_leg/dm_feedback_fresh"].to_numpy() != 1))
                    api_angles = []
                    for axis, short in [("hip", "h"), ("knee", "k")]:
                        p, m = f"/wheel_leg/left_{axis}_joint/", S+f"left_{axis}_joint/"
                        x[f"q{short}"] = group[m+"model_angle"].to_numpy(float)
                        x[f"v{short}"] = group[m+"model_velocity"].to_numpy(float)
                        x[f"tau{short}"] = -group[p+"torque"].to_numpy(float)
                        api_angles.append(group[p+"angle"].to_numpy(float))
                        assert np.max(abs(x[f"v{short}"] + group[p+"velocity"].to_numpy(float))) < 1e-10
                        offset = archive["offsets"][0 if short == "h" else 1]
                        assert np.max(abs(x[f"q{short}"] + api_angles[-1] - offset)) < 1e-10
                        age = (group[S+"control_steady_ns"].to_numpy(float)
                               - group[p+"feedback_steady_ns"].to_numpy(float)) * 1e-9
                        bad |= (age < 0) | (age > .005) | (group[p+"fault_code"].to_numpy() != 0)
                        bad |= group[m+"saturated"].to_numpy() != 0
                        x[f"seq_{short}"] = group[p+"feedback_sequence"].to_numpy(float)
                    x["encoder_delta_rad"] = api_angles[1] - api_angles[0]
                    for label, model in [("archive", archive), ("current", current)]:
                        delta = -x["encoder_delta_rad"].to_numpy() + model["offsets"][1] - model["offsets"][0]
                        beta, hip, knee = gas_torque(model, delta)
                        x[f"beta_{label}_deg"] = beta
                        x[f"gas_{label}_h"], x[f"gas_{label}_k"] = hip, knee
                    for gap in np.flatnonzero(np.diff(t) > .002) + 1:
                        bad |= abs(t - t[gap]) < .12
                    smooth = [c for c in x if c not in ["time_s", "elapsed_s", "duration_s", "seq_h", "seq_k"]]
                    x[smooth] = x[smooth].rolling(51, center=True, min_periods=51).mean()
                    for short in ("h", "k"):
                        x[f"a{short}"] = pd.Series(np.gradient(x[f"v{short}"].to_numpy(), t)).rolling(
                            101, center=True, min_periods=101).mean()
                        for label in ("archive", "current"):
                            x[f"res_{label}_{short}"] = x[f"tau{short}"] - x[f"gas_{label}_{short}"]
                    x["q_common"] = (x.qh + x.qk) / 2
                    x["v_common"] = (x.vh + x.vk) / 2
                    x["dv"] = x.vk - x.vh
                    x["source"], x["source_name"] = source_id, source["name"]
                    x["run"], x["stage"] = int(run), int(group[S+"stage"].iloc[0])
                    x["case"], x["nominal_speed"] = case, nominal_speed
                    before = len(x)
                    x = x[~bad].dropna().copy()
                    # Decimation avoids treating repeated feedback snapshots as
                    # independent observations. Statistical units remain pairs,
                    # angle cells and recordings, not these controller rows.
                    x = x.loc[~((x.seq_h.diff() == 0) & (x.seq_k.diff() == 0))].iloc[::5]
                    stage_counts.append(dict(run=int(run), case=case, stage=int(group[S+"stage"].iloc[0]),
                                             raw_rows=before, prepared_rows=len(x)))
                    frames.append(x)
        provenance.append(dict(**source, path=str(path), bytes=path.stat().st_size, records=records,
                               waveform=f"{waveform}: recorded common leg rotation", stages=stage_counts))
        print("PREPARED", source["name"], sum(x["prepared_rows"] for x in stage_counts), flush=True)
    joined = pd.concat(frames, ignore_index=True)
    np.savez_compressed(out / "prepared.npz", **{c: joined[c].to_numpy(dtype=str if c in
        ("source_name", "case") else None) for c in joined.columns})
    write_json(out / "sources.json", dict(sources=provenance, excluded_stage_pairs=excluded,
        prepared_rows=len(joined), script_sha256=digest(__file__),
        condition="User confirmed: suspended, horizontal, no gravity or external load",
        torque_coordinate="active motor output model frame; tau_model=-tau_API, v_model=-v_API",
        gas_subtraction="residual=tau_feedback_model-tau_gas_holding_model",
        raw_files_modified=False,
        preprocessing="Per-stage centered 51-row means; gradient velocity then centered 101-row mean; "
        "fresh feedback age <=5ms, no faults/saturation; exclude +/-120ms around >2ms gaps; every fifth row"))


def load_prepared(out):
    archive = np.load(out / "prepared.npz", allow_pickle=False)
    return {k: archive[k] for k in archive.files}


def pairs_from_data(data, max_acceleration=1.0):
    # Constant actual speed and relative angle, away from startup/shutdown.
    v = data["v_common"]
    keep = ((data["elapsed_s"] > .6) & (data["elapsed_s"] < data["duration_s"] - .5)
            & (abs(v) > .1) & (data["vh"] * data["vk"] > 0)
            & (abs(data["dv"]) < .04)
            & (abs(data["ah"]) < max_acceleration) & (abs(data["ak"]) < max_acceleration)
            & (abs(abs(v) - data["nominal_speed"]) < np.maximum(.04, .05*data["nominal_speed"])))
    selected = {k: a[keep] for k, a in data.items()}
    qbin = np.rint(selected["q_common"] / .25).astype(int)
    keys = np.array([f"{s}:{r}:{case}:{b}" for s, r, case, b in zip(
        selected["source"], selected["run"], selected["case"], qbin)])
    fields = [k for k in selected if k not in ("source_name", "case")]
    records, rejected = [], dict(missing_direction=0, pose_mismatch=0, beta_mismatch=0)
    for key in np.unique(keys):
        same = keys == key
        pos, neg = same & (selected["v_common"] > 0), same & (selected["v_common"] < 0)
        if min(pos.sum(), neg.sum()) < 5:
            rejected["missing_direction"] += 1
            continue
        plus = {k: float(selected[k][pos].mean()) for k in fields}
        minus = {k: float(selected[k][neg].mean()) for k in fields}
        if abs(plus["q_common"] - minus["q_common"]) > .06:
            rejected["pose_mismatch"] += 1
            continue
        if abs(plus["beta_log_deg"] - minus["beta_log_deg"]) > .3:
            rejected["beta_mismatch"] += 1
            continue
        row = dict(source=int(plus["source"]), run=int(plus["run"]),
                   case=str(selected["case"][pos][0]), q_bin_rad=float(qbin[pos][0] * .25),
                   plus_rows=int(pos.sum()), minus_rows=int(neg.sum()))
        for field in fields:
            row[f"plus_{field}"] = plus[field]
            row[f"minus_{field}"] = minus[field]
        row["speed_rad_s"] = (plus["v_common"] - minus["v_common"]) / 2
        for short in ("h", "k"):
            row[f"speed_{short}_rad_s"] = (plus[f"v{short}"] - minus[f"v{short}"]) / 2
            row[f"raw_half_width_{short}_nm"] = (plus[f"tau{short}"] - minus[f"tau{short}"]) / 2
            for label in ("archive", "current"):
                row[f"friction_{label}_{short}_nm"] = (plus[f"res_{label}_{short}"] - minus[f"res_{label}_{short}"]) / 2
                row[f"bias_{label}_{short}_nm"] = (plus[f"res_{label}_{short}"] + minus[f"res_{label}_{short}"]) / 2
                row[f"gas_half_difference_{label}_{short}_nm"] = (plus[f"gas_{label}_{short}"] - minus[f"gas_{label}_{short}"]) / 2
        row["pair_friction_current_nm"] = row["friction_current_h_nm"] + row["friction_current_k_nm"]
        row["beta_log_deg"] = (plus["beta_log_deg"] + minus["beta_log_deg"]) / 2
        row["beta_current_deg"] = (plus["beta_current_deg"] + minus["beta_current_deg"]) / 2
        row["encoder_delta_rad"] = (plus["encoder_delta_rad"] + minus["encoder_delta_rad"]) / 2
        records.append(row)
    return records, dict(selected_rows=int(keep.sum()), paired_cells=len(records), rejected_cells=rejected)


def write_csv(path, rows):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def group_summary(records):
    groups = []
    keys = sorted({(r["source"], r["run"], r["case"]) for r in records})
    for source, run, case in keys:
        rows = [r for r in records if (r["source"], r["run"], r["case"]) == (source, run, case)]
        numeric = [k for k, v in rows[0].items() if isinstance(v, (int, float))]
        item = {k: float(np.mean([r[k] for r in rows])) for k in numeric}
        item.update(source=source, source_name=SOURCES[source]["name"], run=run, case=case, paired_cells=len(rows))
        for short in ("h", "k"):
            item[f"cell_std_{short}_nm"] = float(np.std([r[f"friction_current_{short}_nm"] for r in rows], ddof=1)) if len(rows) > 1 else 0.
        groups.append(item)
    return groups


def fit_speed_models(groups, short):
    from scipy.optimize import lsq_linear

    results = {}
    for train_id, test_id in [(0, 1), (1, 0)]:
        train = sorted([g for g in groups if g["source"] == train_id], key=lambda g: g["speed_rad_s"])
        test = sorted([g for g in groups if g["source"] == test_id], key=lambda g: g["speed_rad_s"])
        assert len(train) == len(test) == 4
        field = f"friction_current_{short}_nm"
        v, y = np.array([g[f"speed_{short}_rad_s"] for g in train]), np.array([g[field] for g in train])
        vt, yt = np.array([g[f"speed_{short}_rad_s"] for g in test]), np.array([g[field] for g in test])
        coeff = lsq_linear(np.column_stack((np.ones(4), v)), y, bounds=(0, np.inf), tol=1e-12)
        assert coeff.success
        linear = np.linalg.lstsq(np.column_stack((np.ones(4), v)), y, rcond=None)[0]
        coulomb = y.mean()
        pred_linear = coeff.x[0] + coeff.x[1] * vt
        pred_table = np.interp(vt, v, y)
        rms = lambda z: float(np.sqrt(np.mean((z - yt) ** 2)))
        results[f"train_{SOURCES[train_id]['name']}_validate_{SOURCES[test_id]['name']}"] = dict(
            coulomb_nm=float(coulomb), nonnegative_fc_nm=float(coeff.x[0]),
            nonnegative_b_nm_s_rad=float(coeff.x[1]),
            unconstrained_speed_slope_nm_s_rad=float(linear[1]),
            coulomb_validation_rms_nm=rms(np.full(4, coulomb)),
            coulomb_viscous_validation_rms_nm=rms(pred_linear),
            measured_speed_table_validation_rms_nm=rms(pred_table),
            validation_level_count=4,
            validation_max_endpoint_speed_extension_rad_s=float(max(0, v.min()-vt.min(), vt.max()-v.max())))
    return results


def analyze(out):
    data = load_prepared(out)
    records, selection = pairs_from_data(data)
    # Compare exactly the same absolute common-angle cells in both files and
    # all four speeds, so neither long slow records nor different poses dominate.
    base = [r for r in records if r["source"] < 2]
    keys = sorted({(r["source"], r["case"]) for r in base})
    assert len(keys) == 8
    common_bins = set.intersection(*[{r["q_bin_rad"] for r in base if (r["source"], r["case"]) == k} for k in keys])
    assert len(common_bins) >= 8, f"Too few common angle bins: {len(common_bins)}"
    base = [r for r in base if r["q_bin_rad"] in common_bins]
    baseline = group_summary(base)
    angle_groups = group_summary([r for r in records if r["source"] == 2])
    speed_table = []
    for case in PAIRS:
        groups = [g for g in baseline if g["case"] == case]
        assert len(groups) == 2
        row = dict(case=case, speed_rad_s=float(np.mean([g["speed_rad_s"] for g in groups])),
                   beta_log_deg=float(np.mean([g["beta_log_deg"] for g in groups])),
                   beta_current_deg=float(np.mean([g["beta_current_deg"] for g in groups])))
        for short in ("h", "k"):
            row[f"friction_{short}_nm"] = float(np.mean([g[f"friction_current_{short}_nm"] for g in groups]))
            row[f"recording_min_{short}_nm"] = float(min(g[f"friction_current_{short}_nm"] for g in groups))
            row[f"recording_max_{short}_nm"] = float(max(g[f"friction_current_{short}_nm"] for g in groups))
        speed_table.append(row)
    angle_table = []
    for case in ANGLE_PAIRS:
        groups = [g for g in angle_groups if g["case"] == case]
        if not groups:
            continue
        fields = ["speed_rad_s", "beta_log_deg", "beta_current_deg", "encoder_delta_rad",
                  "friction_current_h_nm", "friction_current_k_nm"]
        row = {key: float(np.mean([g[key] for g in groups])) for key in fields}
        row.update(case=case, recordings=len(groups),
                   paired_cells_total=sum(g["paired_cells"] for g in groups),
                   minimum_cells_per_recording=min(g["paired_cells"] for g in groups),
                   usage="diagnostic_only_sparse_constant_speed_pairs")
        angle_table.append(row)
    angle_table.sort(key=lambda r: r["beta_current_deg"])
    high_speed = [r for r in speed_table if r["speed_rad_s"] > .5]
    parameters = {}
    for axis, short in [("hip", "h"), ("knee", "k")]:
        candidate = float(np.mean([r[f"friction_{short}_nm"] for r in high_speed]))
        left = dict(derivation="left measured, gas-subtracted common-angle bidirectional steady motion",
            coulomb_plateau_candidate_nm=candidate,
            candidate_velocity_domain_rad_s=[float(high_speed[0]["speed_rad_s"]), float(high_speed[-1]["speed_rad_s"])],
            viscous_coefficient_nm_s_rad=None, viscous_identified=False,
            static_breakaway_nm=None, static_breakaway_identified=False,
            stribeck_velocity_rad_s=None, stribeck_identified=False,
            low_speed_moving_friction_nm=speed_table[0][f"friction_{short}_nm"],
            reference_beta_log_deg=float(np.mean([r["beta_log_deg"] for r in speed_table])),
            reference_beta_calibration110_deg=float(np.mean([r["beta_current_deg"] for r in speed_table])),
            speed_knots_rad_s=[r["speed_rad_s"] for r in speed_table],
            moving_friction_knots_nm=[r[f"friction_{short}_nm"] for r in speed_table],
            outside_measured_speed_and_posture_domain="not identified; no validated extrapolation rule",
            interpretation="effective installed motor/gear/bearing/cable moving friction, not isolated motor internals")
        parameters[f"left_{axis}"] = left
        parameters[f"right_{axis}"] = {**left,
            "derivation": "user-authorized left-to-right approximation, not independent right-leg identification",
            "symmetry_source": f"left_{axis}"}
    sensitivity = {}
    for threshold in (.5, 2.):
        rr, counts = pairs_from_data(data, threshold)
        bb = [r for r in rr if r["source"] < 2 and r["q_bin_rad"] in common_bins]
        gg = group_summary(bb)
        sensitivity[str(threshold)] = dict(selection=counts,
            group_values=[{k: g[k] for k in ["source_name", "case", "paired_cells", "friction_current_h_nm", "friction_current_k_nm"]} for g in gg])
    diagnostics = dict(
        common_gas_torque_sum_max_abs_nm=float(np.max(abs(data["gas_current_h"] + data["gas_current_k"]))),
        baseline_archive_current_friction_max_difference_nm=float(max(abs(
            g[f"friction_current_{s}_nm"] - g[f"friction_archive_{s}_nm"]) for g in baseline for s in ("h", "k"))),
        baseline_hip_bias_range_nm=[min(g["bias_current_h_nm"] for g in baseline), max(g["bias_current_h_nm"] for g in baseline)],
        baseline_knee_bias_range_nm=[min(g["bias_current_k_nm"] for g in baseline), max(g["bias_current_k_nm"] for g in baseline)],
        sparse_angle_hip_bias_max_nm=max(g["bias_current_h_nm"] for g in angle_groups),
        sparse_angle_knee_bias_min_nm=min(g["bias_current_k_nm"] for g in angle_groups))
    result = dict(schema="rmcs-dm-leg-moving-friction-analysis-v1", date="2026-10-31",
        gas_subtraction="DM measured torque in model frame minus positive commanded gas holding effort",
        main_estimator="(residual_positive_motion - residual_negative_motion)/2 at matching common-angle cells",
        selection=selection, gas_diagnostics=diagnostics, matched_common_angle_bins_rad=sorted(common_bins),
        baseline_groups=baseline, speed_table=speed_table, angle_groups=angle_groups, angle_table=angle_table,
        speed_models={axis: fit_speed_models(baseline, short) for axis, short in [("hip", "h"), ("knee", "k")]},
        acceleration_sensitivity=sensitivity, parameters=parameters,
        limitations=["Residual midpoints mix gas-model error, direction asymmetry and other loads; they are not uniquely attributed to gas or friction.",
            "Effective moving friction only; installed motor, reducer, bearings and cables are not separated.",
            "No zero-speed breakaway protocol; do not identify static friction from moving data.",
            "Only four ordered speed levels, no motor temperatures; Stribeck and physical viscous damping are not independently identifiable.",
            "Common-mode trajectories cannot identify a full coupled inertia matrix or relative-mode friction.",
            "Right-leg values mirror the left by explicit user instruction, without right-leg validation.",
            "Angles retain both historical log beta and current calibration110 beta; these labels are not interchangeable.",
            "No source recording, default controller, default Isaac model, robot motion or training changed."])
    write_json(out / "analysis.json", result)
    write_json(out / "parameters.json", dict(schema="rmcs-dm-friction-offline-candidate-v1",
        torque_units="Nm at DM active output", passive_friction_torque="-F(abs(v), angle)*sign(v)",
        compensation_torque="+F(abs(v), angle)*sign(v)",
        live_controller_applied=False, isaac_default_applied=False, right_symmetry_authorized=True,
        beta_reference="speed table is local to recorded beta about 84 degrees / calibration110 about 89 degrees",
        motors=parameters, speed_table=speed_table,
        angle_diagnostics=angle_table, angle_diagnostics_are_validated_model_knots=False,
        unmeasured_static_and_viscous_parameters_are_null=True))
    write_csv(out / "paired_cells.csv", records)
    write_csv(out / "speed_table.csv", speed_table)
    write_csv(out / "angle_table.csv", angle_table)
    make_plot(out, result)
    write_report(out, result)
    write_json(out / "verification.json", dict(
        original_source_sha256_matched=True,
        gas_sources_unchanged_after_preparation=all(digest(path) == sha for model in
            json.loads((out / "gas_models.json").read_text()).values() for path, sha in model["sources"].items()),
        gas_subtraction_identity_max_error_nm=float(max(
            np.max(abs(data[f"tau{s}"] - data[f"gas_current_{s}"] - data[f"res_current_{s}"])) for s in ("h", "k"))),
        gas_common_rotation_sum_max_abs_nm=float(np.max(abs(data["gas_current_h"] + data["gas_current_k"]))),
        corrected_pair_halfwidth_identity_max_error_nm=float(max(abs(
            r[f"friction_current_{s}_nm"] - r[f"raw_half_width_{s}_nm"]
            + r[f"gas_half_difference_current_{s}_nm"]) for r in records for s in ("h", "k"))),
        baseline_archive_vs_current_max_friction_difference_nm=float(max(abs(
            r[f"friction_current_{s}_nm"] - r[f"friction_archive_{s}_nm"]) for r in baseline for s in ("h", "k"))),
        all_paired_moving_friction_positive=bool(all(r[f"friction_current_{s}_nm"] > 0 for r in records for s in ("h", "k"))),
        complete_speed_levels_per_baseline_recording=4, common_pose_bin_count=len(common_bins),
        report_hashes={name: digest(out / name) for name in ["analysis.json", "parameters.json", "paired_cells.csv", "speed_table.csv", "angle_table.csv", "friction.png", "README.md"]},
        script_sha256=digest(__file__), hardware_motion_started=False, training_started=False))
    print(json.dumps(dict(speed_table=speed_table, angle_table=angle_table,
                         speed_models=result["speed_models"], parameters=parameters), ensure_ascii=False, indent=2))


def make_plot(out, result):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), constrained_layout=True)
    colors = ["#1676b8", "#d15d28"]
    for j, (axis, short) in enumerate([("Hip", "h"), ("Knee drive", "k")]):
        ax = axes[0, j]
        for source in (0, 1):
            rows = sorted([g for g in result["baseline_groups"] if g["source"] == source], key=lambda g: g["speed_rad_s"])
            ax.plot([g["speed_rad_s"] for g in rows], [g[f"friction_current_{short}_nm"] for g in rows],
                    "o-", color=colors[source], label=SOURCES[source]["name"])
        ax.set(title=f"{axis}: gas-subtracted moving friction", xlabel="Actual speed magnitude [rad/s]", ylabel="Direction half-width [Nm]")
        ax.legend()
        ax.grid(alpha=.25)
        ax = axes[1, j]
        for run, marker in [(1, "o"), (2, "s")]:
            rows = sorted([g for g in result["angle_groups"] if g["run"] == run], key=lambda g: g["beta_current_deg"])
            ax.plot([g["beta_current_deg"] for g in rows], [g[f"friction_current_{short}_nm"] for g in rows], marker, label=f"angle run {run}")
        ax.set(title=f"{axis}: sparse posture diagnostics (~3.1 rad/s)", xlabel="Knee included angle, calibration110 [deg]", ylabel="Direction half-width [Nm]")
        ax.legend()
        ax.grid(alpha=.25)
    fig.suptitle("Left DM joint moving friction; right values assumed symmetric", fontsize=14)
    fig.savefig(out / "friction.png", dpi=170)
    plt.close(fig)


def write_report(out, result):
    # Written after numerical inspection; keep this function the report's single
    # source of truth so rerunning cannot silently detach prose from the values.
    rows = "\n".join(f"| {r['speed_rad_s']:.3f} | {r['friction_h_nm']:.3f} | {r['friction_k_nm']:.3f} |" for r in result["speed_table"])
    angle_rows = "\n".join(f"| {r['beta_log_deg']:.2f} | {r['beta_current_deg']:.2f} | {r['friction_current_h_nm']:.3f} | {r['friction_current_k_nm']:.3f} | {r['recordings']} | {r['paired_cells_total']} |" for r in result["angle_table"])
    source_rows = "\n".join(f"- `{BASE / s['file']}`，SHA-256 `{s['sha256']}`" for s in SOURCES)
    model_rows = []
    for axis, values in result["speed_models"].items():
        for split, r in values.items():
            model_rows.append(f"| {axis} | {split} | {r['coulomb_validation_rms_nm']:.4f} | {r['coulomb_viscous_validation_rms_nm']:.4f} | {r['measured_speed_table_validation_rms_nm']:.4f} | {r['nonnegative_b_nm_s_rad']:.5g} |")
    h, k = result["parameters"]["left_hip"], result["parameters"]["left_knee"]
    diag = result["gas_diagnostics"]
    text = f"""# 四个 DM 电机摩擦辨识（2026-10-31）

使用左腿髋、膝电机同时等角度转动的原始记录，在实际 DM 反馈力矩中减去气簧的电机端保持力矩，再按相同整腿姿态配对正反方向。右腿髋、膝分别采用左腿对应电机的近似值，这是用户授权的对称假设。

![摩擦随速度及夹角变化](friction.png)

## 可获得的数值

下面是两份独立整腿转动记录的等权平均，单位是电机主动输出轴 N·m，不是地面摩擦系数。每个格使用相同的绝对整腿角度区间；两方向的完整分离为表中数值的两倍。

| 实测速度幅值 [rad/s] | hip 运动摩擦力矩 [N·m] | knee 驱动运动摩擦力矩 [N·m] |
|---:|---:|---:|
{rows}

约 0.6–3.1 rad/s 范围，局部简化库仑项候选为左/右 hip **{h['coulomb_plateau_candidate_nm']:.3f} N·m**、左/右 knee **{k['coulomb_plateau_candidate_nm']:.3f} N·m**。这是约 84° 历史日志夹角附近的装配状态动摩擦近似，不能无条件覆盖全部夹角、全部速度。低速仍在运动的力矩单独保留，不能叫静摩擦力矩。

`parameters.json` 保存四电机候选、四档速度表和夹角诊断。未知的物理黏性系数、静摩擦脱离阈值、Stribeck 特征速度均为 `null`，没有把缺乏辨识写成物理零。仿真摩擦施加方向为 `-F*sign(v)`，控制前馈补偿方向相反。

## 直接减去气簧之后还要检查什么

使用 `tau_model=-tau_API`、`v_model=-v_API`。已有补偿函数给的是电机为抵住气簧需要施加的保持力矩，所以残差为 `r=tau_feedback_model-tau_gas_holding_model`。不能把 API 力矩直接与 model 力矩相减，不能用闭环命令代替实际反馈。

在水平悬空、无外载条件下，残差仍包含惯性和气簧模型误差。本次排除启动/反向/停机段，并要求实际同向等速、低加速度。再取同一姿态的 `F=(r_positive-r_negative)/2`。`bias=(r_positive+r_negative)/2` 单独保留。它可能混合气簧误差、方向不对称和其他负载，不能唯一归因；对称摩擦模型不能直接把它也算作库仑项。每个角度格的原始反馈、气簧保持力矩、残差、速度、加速度、偏置都见 `paired_cells.csv`。

本批历史数据代入当前补偿后，基准夹角附近 hip 中点范围为 {diag['baseline_hip_bias_range_nm'][0]:+.2f}～{diag['baseline_hip_bias_range_nm'][1]:+.2f} N·m、knee 为 {diag['baseline_knee_bias_range_nm'][0]:+.2f}～{diag['baseline_knee_bias_range_nm'][1]:+.2f} N·m；部分大夹角诊断记录中 hip/knee 偏置可分别达到 {diag['sparse_angle_hip_bias_max_nm']:+.2f}/{diag['sparse_angle_knee_bias_min_nm']:+.2f} N·m。说明“反馈减气簧”还不能直接当成纯摩擦。这里报告的是正反向半差得到的对称有效动摩擦。历史/当前气簧模型替换对基准摩擦半差影响最大 {diag['baseline_archive_current_friction_max_difference_nm']:.4f} N·m，因为同姿态下方向无关的差异基本被配对抵消。这些历史残差不能在缺少编码器零点对应证据时直接证明当前实车补偿误差。

气簧长度不变并不意味着每个电机的气簧力矩为零；两个电机的气簧力矩在共同转动方向上基本抵消。本批处理轨迹的当前模型两电机气簧力矩之和绝对值最大 {diag['common_gas_torque_sum_max_abs_nm']:.6f} N·m。库仑平台候选之和约 **{h['coulomb_plateau_candidate_nm'] + k['coulomb_plateau_candidate_nm']:.3f} N·m**，比两个电机的分别归因更少受内部气簧载荷影响。实际夹角仍有微小波动，单电机结果依赖气簧模型和对称摩擦假设，不能宣称已完全隔离气簧密封摩擦。

## 不同夹角的数据

表中速度均约 3.1 rad/s。历史数据使用旧偏置 `q_model=-q_API+[1.6,2.93]`；当前分支 calibration110 使用 `[1.6,3.0282761689183353]`。分析从原始编码器差值分别重算，保留两种夹角标签，避免把同一编码器姿态误贴成同一个标定角。主要摩擦结果使用当前分支 `full_vehicle_v202.hpp` 与 calibration110 扩展几何，并继承当前气簧力曲线。旧模型作为敏感性对照。

| 历史日志 β [deg] | 当前 calibration110 β [deg] | hip [N·m] | knee 驱动 [N·m] | 有效往返记录数 | 配对姿态格总数 |
|---:|---:|---:|---:|---:|---:|
{angle_rows}

这组记录加减速/抖动较多，严格恒速筛选后每个夹角仅剩少量姿态格，末端部分条件只有 1 格。此表只作夹角依赖诊断，**不作为已经验证的全夹角插值模型**。夹角表表示该装配和气簧预载下的有效运动阻力，包含减速器、轴承、连接件及线束影响。不能进一步唯一分离各个内部部件；也不能用这条共同运动轨迹覆盖相对伸缩方向的全部摩擦。

## 筛选与留出验证

只使用已核对的整腿共同转动阶段。早期两份速度记录对应 waveform 2，后期多夹角记录对应 waveform 4，不能跨版本仅凭枚举值判断动作。要求状态运行、参考激活、左腿、反馈新鲜、反馈年龄 0–5 ms、无故障且无力矩饱和。每阶段 51 点居中均值，速度求导再 101 点均值；剔除 >2 ms 时钟空隙前后 120 ms，每 5 点取一次。滤波仅用于离线分析。

恒速条件：启动后 >0.6 s、距结束 >0.5 s，两个电机同向，速度差 <0.04 rad/s，实际速度距标称值 <max(0.04,5%标称速度)，两电机加速度均 <1 rad/s²。绝对整腿角度每 0.25 rad 一格，两方向各至少 5 个处理样本，格内均值位置差 <0.06 rad、历史 β 差 <0.3°。两份速度记录、全部四速度共用 **{len(result['matched_common_angle_bins_rad'])}** 个姿态格，格等权，不按慢速记录长度加权。

一份完整速度记录拟合、另一份验证，并交换角色。下面 RMS 评估四档稳态方向半宽，属于不同录制之间的离线验证，不是时域闭环或硬件验证。

| 轴 | 分割 | 库仑常数 RMS [N·m] | 库仑+非负黏性 RMS [N·m] | 实测速度表 RMS [N·m] | 拟合非负 B [N·m·s/rad] |
|---|---|---:|---:|---:|---:|
{chr(10).join(model_rows)}

速度按固定顺序执行，没有 DM 电机温度记录。低速动摩擦增加可作为经验速度依赖项保存，但不足以唯一确定静摩擦峰值、Stribeck 特征速度或物理黏性阻尼。小加速度余量与模型误差仍存在；`analysis.json` 包含 0.5/2 rad/s² 筛选敏感性对照。第二次多角度记录在最后一组中途结束，未把单向段作为完整往返。

## 来源与复现

{source_rows}

原始 CSV 全部 SHA-256 重新匹配，未修改。第一份文件被中断的末行 245805 沿用既有审计排除。`gas_models.json` 保存当次使用的曲线、几何、标定和源文件哈希。数据读取与拟合：

```bash
/home/noir/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 .script/identification/analyze_dm_leg_friction.py prepare --out reports/dm_leg_friction_20261031
/home/noir/miniconda3/envs/isaacsim6/bin/python .script/identification/analyze_dm_leg_friction.py analyze --out reports/dm_leg_friction_20261031
```

仅新增离线分析脚本及报告。没有修改实车控制器或 Isaac Sim 默认模型，没有启动实车或训练。
"""
    (out / "README.md").write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["prepare", "analyze"])
    parser.add_argument("--out", type=Path, default=ROOT / "reports/dm_leg_friction_20261031")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.phase == "prepare":
        prepare(args.out)
    else:
        analyze(args.out)


if __name__ == "__main__":
    main()
