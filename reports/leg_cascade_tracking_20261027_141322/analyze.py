#!/usr/bin/env python3
"""Read-only error audit of the first P/P leg sweep, with a matched slow baseline."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import subprocess
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SOURCE = Path("/home/noir/wheel_leg_joints_2026-10-27_14-13-22.csv")
OLD_SOURCE = Path("/home/noir/Documents/workspace/wheel_leg扫频/wheel_leg_膝关节气簧辨识数据.csv")
OLD_META = Path("/home/noir/.cache/rmcs-sweep-archive-20261027/e5ccbdff95ab.json")
S = "/wheel_leg/sweep/"
AXES = ("left_hip_joint", "left_knee_joint")


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return dict(n=len(a), bias=float(a.mean()), mae=float(abs(a).mean()),
                rmse=float(np.sqrt(np.mean(a*a))), p95_abs=float(np.quantile(abs(a), .95)),
                max_abs=float(abs(a).max()), std=float(a.std()),
                min=float(a.min()), max=float(a.max()))


def active(frame):
    return frame[(frame[S+"state"] == 3) & (frame[S+"reference_active"] == 1)]


def metrics(frame):
    out = {}
    for axis in AXES:
        p = S+axis+"/"
        out[axis] = {
            "position_error_ref_minus_actual_deg": stats(np.rad2deg(frame[p+"tracking_error"])),
            "trajectory_velocity_error_ref_minus_actual_rad_s": stats(frame[p+"reference_velocity"]-frame[p+"model_velocity"]),
            "torque_cmd_api_nm": stats(frame["/wheel_leg/"+axis+"/control_torque"]),
            "torque_feedback_api_nm": stats(frame["/wheel_leg/"+axis+"/torque"]),
            "torque_saturated_fraction": float(frame[p+"saturated"].mean()),
        }
        if p+"velocity_target" in frame:
            out[axis].update({
                "inner_velocity_error_target_minus_actual_rad_s": stats(frame[p+"velocity_error"]),
                "velocity_target_rad_s": stats(frame[p+"velocity_target"]),
                "velocity_target_limited_fraction": float(frame[p+"velocity_target_limited"].mean()),
                "controller_formula_api_residual_nm": stats(
                    frame[p+"torque_preclip"] + 4*frame[p+"velocity_error"] + frame[p+"probe_torque_model"]),
            })
    out["beta_error_actual_minus_ref_deg"] = stats(np.rad2deg(frame[S+"left/beta_estimate"]-frame[S+"beta_reference"]))
    out["beta_actual_range_deg"] = np.rad2deg(frame[S+"left/beta_estimate"]).agg(["min", "max"]).to_dict()
    return out


def main():
    # Reject malformed rows instead of silently shifting columns or dropping data.
    with SOURCE.open() as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = 0
        for number, row in enumerate(reader, 2):
            if len(row) != len(header):
                raise ValueError(f"Malformed source line {number}: {len(row)} fields")
            rows += 1
    frame = pd.read_csv(SOURCE)
    assert len(frame) == rows
    current_hash = sha256(SOURCE)
    # This cache has explicit source path and original-file hash provenance.
    old_meta = json.loads(OLD_META.read_text())
    assert old_meta["path"] == str(OLD_SOURCE)
    assert old_meta["sha256"] == sha256(OLD_SOURCE)
    old = pd.read_pickle(OLD_META.with_suffix(".pkl"))
    running = active(frame)
    assert running[S+"side"].eq(0).all()
    assert running[S+"mode"].eq(2).all()
    assert running[S+"run_id"].eq(1).all()
    # Eleven complete 4 s half-cycles: 3 train cycles and 2.5 holdout cycles.
    # Exclude the initial 5 s move and truncated stage 12 from the main metrics.
    main_run = running[running[S+"stage"].between(1, 11)]
    baseline = active(old)
    baseline = baseline[baseline[S+"stage"].between(1, 11)
                        & baseline[S+"run_id"].eq(1) & baseline[S+"mode"].eq(2)]
    for g in (main_run, baseline):
        assert set(g[S+"stage"].unique()) == set(range(1, 12))
        assert np.allclose(g[S+"stage_duration_s"], 4.0)
        assert abs(float(np.rad2deg(g[S+"beta_reference"]).min())-45) < 1e-5
        assert abs(float(np.rad2deg(g[S+"beta_reference"]).max())-100) < 1e-5
    current = metrics(main_run)
    before = metrics(baseline)
    comparison = []
    for axis in AXES+("beta",):
        if axis == "beta":
            a, b = before["beta_error_actual_minus_ref_deg"], current["beta_error_actual_minus_ref_deg"]
        else:
            a, b = before[axis]["position_error_ref_minus_actual_deg"], current[axis]["position_error_ref_minus_actual_deg"]
        comparison.append(dict(axis=axis, old_rmse_deg=a["rmse"], new_rmse_deg=b["rmse"],
                               reduction_percent=100*(1-b["rmse"]/a["rmse"])))
    stages = []
    for stage, g in running.groupby(S+"stage"):
        stages.append(dict(stage=int(stage), rows=len(g), validation=int(g[S+"validation"].iloc[0]),
                           duration_s=float((g[S+"control_steady_ns"].iloc[-1]-g[S+"control_steady_ns"].iloc[0])*1e-9),
                           stage_duration_s=float(g[S+"stage_duration_s"].iloc[-1]),
                           stage_elapsed_last_s=float(g[S+"stage_elapsed_s"].iloc[-1]), metrics=metrics(g)))
    beta_points = []
    ref_deg = np.rad2deg(main_run[S+"beta_reference"])
    for beta in (45, 50, 60, 75, 90, 100):
        g = main_run[abs(ref_deg-beta) < .5]
        point = dict(beta_reference_bin_center_deg=beta, bin_half_width_deg=.5, rows=len(g),
                     actual_mean_deg=float(np.rad2deg(g[S+"left/beta_estimate"]).mean()),
                     actual_p05_p95_deg=np.rad2deg(g[S+"left/beta_estimate"]).quantile([.05, .95]).tolist())
        for direction, mask in (("opening", g[S+"left_knee_joint/reference_velocity"]>.001),
                                ("closing", g[S+"left_knee_joint/reference_velocity"]<-.001)):
            h = g[mask]
            point[direction] = dict(rows=len(h), actual_mean_deg=float(np.rad2deg(h[S+"left/beta_estimate"]).mean()))
        beta_points.append(point)
    zero_time = float(frame[S+"control_steady_ns"].iloc[0])
    changes = (frame[S+"state"].ne(frame[S+"state"].shift())
               | frame[S+"mode"].ne(frame[S+"mode"].shift())
               | frame[S+"run_id"].ne(frame[S+"run_id"].shift()))
    timeline = [dict(t_s=float((r[S+"control_steady_ns"]-zero_time)*1e-9), state=int(r[S+"state"]),
                     mode=int(r[S+"mode"]), run=int(r[S+"run_id"]), stop=int(r[S+"stop_reason"]))
                for _,r in frame[changes].iterrows()]
    gap_rows = main_run[main_run[S+"control_dt_s"]>.002]
    gaps = []
    for idx,r in gap_rows.iterrows():
        prev = frame.iloc[idx-1]
        gaps.append(dict(t_s=float((r[S+"control_steady_ns"]-zero_time)*1e-9),
                         dt_ms=float(r[S+"control_dt_s"]*1000), stage=int(r[S+"stage"]),
                         previous_controller_duration_us=float((prev[S+"control_end_steady_ns"]-prev[S+"control_steady_ns"])*1e-3)))
    result = dict(source=str(SOURCE), source_sha256=current_hash, rows=rows, columns=len(header),
                  source_malformed_rows=0, active_rows=len(running), analysis_rows=len(main_run),
                  baseline_source=str(OLD_SOURCE), baseline_sha256=old_meta["sha256"], baseline_rows=len(baseline),
                  baseline_malformed_tail_excluded=old_meta["malformed_lines_excluded"],
                  selection="left/run1/mode2, complete stages1..11; 8 s round trip; exclude initial move and partial stage12",
                  gains={c.removeprefix(S+"host/"):frame[c].dropna().unique().tolist() for c in frame if c.startswith(S+"host/")},
                  current=current, baseline=before, comparison=comparison,
                  train=metrics(main_run[main_run[S+"validation"]==0]),
                  validation_partial_cycle=metrics(main_run[main_run[S+"validation"]==1]),
                  stages=stages, beta_bins=beta_points, timeline=timeline,
                  completed=bool(frame[S+"completed"].any()),
                  timing=dict(control_dt_ms_quantiles=main_run[S+"control_dt_s"].quantile([.5,.99,1]).mul(1000).to_dict(),
                              controller_duration_us_quantiles=((main_run[S+"control_end_steady_ns"]-main_run[S+"control_steady_ns"])*1e-3).quantile([.5,.99,1]).to_dict(),
                              gaps_over_2_ms=gaps),
                  limits=["beta is closed-chain FK from motor encoders, not an independent angle sensor",
                          "This file has no fast stage, no local chirp, no right-leg test and no completed run",
                          "The baseline is a separate recording under matched trajectory conditions, not a controlled randomized A/B experiment",
                          "Timestamp gaps are control scheduling gaps, not an identified actuator delay"])
    (OUT/"metrics.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n")

    x = (main_run[S+"control_steady_ns"].to_numpy(float)-main_run[S+"control_steady_ns"].iloc[0])*1e-9
    np.savez_compressed(OUT/"plot_data.npz", time=x,
                        beta_ref=np.rad2deg(main_run[S+"beta_reference"].to_numpy(float)),
                        beta_actual=np.rad2deg(main_run[S+"left/beta_estimate"].to_numpy(float)),
                        hip_error=np.rad2deg(main_run[S+AXES[0]+"/tracking_error"].to_numpy(float)),
                        knee_error=np.rad2deg(main_run[S+AXES[1]+"/tracking_error"].to_numpy(float)),
                        knee_vref=main_run[S+AXES[1]+"/reference_velocity"].to_numpy(float),
                        knee_v=main_run[S+AXES[1]+"/model_velocity"].to_numpy(float))
    # Matplotlib is absent from the bundled analysis runtime. Use the installed
    # system plotting runtime without modifying either dependency environment.
    subprocess.run(["/usr/bin/python3", str(OUT/"plot.py")], check=True)
    print(json.dumps(dict(comparison=comparison,current=current,rows=rows,analysis_rows=len(main_run)),indent=2,ensure_ascii=False))


if __name__ == "__main__":
    main()
