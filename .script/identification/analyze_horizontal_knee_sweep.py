#!/usr/bin/env python3
"""Audit a horizontal, suspended knee sweep without fitting unidentifiable parameters.

Uses measured motion and reported motor torque. Stiction candidates and quasi-static
loads include gas-spring seal/gear friction; neither is a pure gas-spring measurement.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

S = "/wheel_leg/sweep/"


def load(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        header = next(csv.reader(stream))
    selected = [name for name in header if name == "index" or name.startswith(S)
                or name.startswith("/wheel_leg/left_") and any(name.endswith("/" + field)
                    for field in ("velocity", "torque", "control_torque", "feedback_sequence",
                                  "feedback_steady_ns", "tau_frame_api", "tx_kind"))]
    # C parser avoids hundreds of MB of Python float objects on a long 1 kHz log.
    data = pd.read_csv(path, usecols=selected)
    digest, incomplete = hashlib.sha256(), []
    with path.open("rb") as stream:
        for number, line in enumerate(stream, 1):
            digest.update(line)
            # ValueCollector writes numeric, unquoted cells and unquoted signal names.
            fields = line.count(b",") + 1
            if fields != len(header):
                incomplete.append({"line": number, "fields": fields})
    if incomplete:
        raise ValueError(f"Incomplete CSV records require explicit cleaning: {incomplete[:5]}")
    return data, {"source_csv": str(path.resolve()), "source_sha256": digest.hexdigest(),
                  "columns": len(header), "records": len(data), "incomplete_records": incomplete}


def summarize(data):
    running = (data[S + "state"] == 3) & (data[S + "reference_active"] == 1)
    if not running.any() or set(data.loc[running, S + "side"]) != {0}:
        raise ValueError("Expected an active, left-leg knee sweep")
    t = (data[S + "control_steady_ns"] - data[S + "control_steady_ns"].iloc[0]) * 1e-9
    beta = np.rad2deg(data[S + "left/beta_estimate"])
    reference = np.rad2deg(data[S + "beta_reference"])
    report = {"condition": "horizontal suspended; gravity omitted by operator confirmation",
              "side": "left", "duration_s": float(t.iloc[-1]),
              "active_records": int(running.sum()),
              "state_counts": {str(int(k)): int(v) for k, v in data[S + "state"].value_counts().items()},
              "observed_beta_deg": [float(beta[running].min()), float(beta[running].max())],
              "beta_error_deg_p05_p50_p95": np.percentile((beta - reference)[running], [5, 50, 95]).tolist(),
              "completed": bool(data.loc[running, S + "completed"].any()) or bool(data[S + "completed"].iloc[-1]),
              "validation_records": int((running & (data[S + "validation"] == 1)).sum()),
              "axes": {}, "stages": [], "holds": [], "breakaway_candidates": [],
              "real2sim_ready": False, "inertia_and_damping_fitted": False}
    for joint in ("hip", "knee"):
        axis = "left_" + joint + "_joint"
        p, m = S + axis + "/", "/wheel_leg/" + axis + "/"
        velocity, api = data[p + "model_velocity"], data[m + "velocity"]
        moving = running & (api.abs() > .05)
        sign = float(np.sign(np.median(velocity[moving] / api[moving])))
        if not np.allclose(velocity[moving], sign * api[moving], atol=1e-7):
            raise ValueError("Inconsistent API/model velocity direction")
        error = data.loc[running, p + "tracking_error"]
        fresh = data[m + "feedback_sequence"].diff() > 0
        age = (data[S + "control_steady_ns"] - data[m + "feedback_steady_ns"]) * 1e-6
        report["axes"][joint] = {
            "model_sign": sign, "tracking_rms_deg": float(np.rad2deg(np.sqrt(np.mean(error ** 2)))),
            "saturated_fraction": float(data.loc[running, p + "saturated"].mean()),
            "peak_reported_torque_nm": float(data.loc[running, m + "torque"].abs().max()),
            "feedback_age_ms_p99": float(np.percentile(age[running], 99)),
            "repeated_feedback_fraction": float((~fresh[running]).mean()),
            "finite_encoded_torque_records": int((running & np.isfinite(data[m + "tau_frame_api"])).sum()),
            "independent_probe_records": int((running & (data[p + "probe_torque_model"] != 0)).sum())}
    previous_direction = 0
    for (run, stage), g in data[running].groupby([S + "run_id", S + "stage"], sort=False):
        first, last = g.index[0], g.index[-1]
        row = {"run": int(run), "stage": int(stage), "waveform": int(g[S + "waveform"].iloc[0]),
               "start_s": float(t[first]), "end_s": float(t[last]),
               "reference_range_deg": [float(reference[g.index].min()), float(reference[g.index].max())],
               "actual_range_deg": [float(beta[g.index].min()), float(beta[g.index].max())]}
        report["stages"].append(row)
        if row["waveform"] != 1:
            continue
        delta = reference[last] - reference[first]
        if abs(delta) > .1:
            previous_direction = int(np.sign(delta))
            if t[last] - t[first] > 2:
                report["breakaway_candidates"].extend(breakaway(g, t, beta, report["axes"]["knee"]["model_sign"]))
        elif t[last] - t[first] > .5:
            settled = g[t[g.index] >= t[last] - .3]
            row = {"run": int(run), "stage": int(stage), "direction": previous_direction,
                   "reference_beta_deg": float(reference[settled.index].median()),
                   "actual_beta_deg": float(beta[settled.index].median()),
                   "beta_jitter_span_deg": float(np.ptp(beta[settled.index]))}
            for joint in ("hip", "knee"):
                p = S + "left_" + joint + "_joint/"
                m = "/wheel_leg/left_" + joint + "_joint/torque"
                row[joint + "_model_torque_nm"] = float(settled[m].median() * report["axes"][joint]["model_sign"])
                row[joint + "_error_deg"] = float(np.rad2deg(settled[p + "tracking_error"].median()))
            report["holds"].append(row)
    report["limitations"] = [
        "Measured range is a loaded closed-loop trajectory, not a mechanical hard-stop calibration.",
        "PD error alone gives net balancing effort, not separate gas force, friction, damping or inertia.",
        "Breakaway torques include position-dependent spring load; no pure stiction coefficient is exported.",
        "Mode 2 does not independently identify the full coupled hip/knee inertia matrix.",
        "This audit exports no accepted inertia/damping or actuator delay parameters.",
        "Encoded frame events require time alignment and independent validation before estimating delay."]
    return report


def breakaway(g, t, beta, sign):
    """Conservative motion-onset candidates, deduplicated by feedback sequence.

    Require a >=50 ms near-stationary interval followed by >=20 ms motion and
    >1 mrad measured displacement. These thresholds exceed velocity quantization.
    """
    m, p = "/wheel_leg/left_knee_joint/", S + "left_knee_joint/"
    g = g[g[m + "feedback_sequence"].diff() > 0]
    time = t[g.index].to_numpy()
    velocity = g[p + "model_velocity"].rolling(21, center=True, min_periods=21).mean().to_numpy()
    q = g[p + "model_angle"].to_numpy()
    events, rest_start, release_start, last_event = [], None, None, -np.inf
    for i in range(1, len(g) - 21):
        if time[i] - time[i - 1] > .005:
            rest_start = release_start = None
        if abs(velocity[i]) < .025:
            rest_start = i if rest_start is None else rest_start
            release_start = None
            continue
        if rest_start is None or time[i] - time[rest_start] < .05:
            continue
        release_start = i if release_start is None else release_start
        end = np.searchsorted(time, time[i] + .020)
        if (end < len(g) and time[i] - last_event > .1 and end > i
                and np.all(np.abs(velocity[i:end]) > .08) and abs(q[end] - q[i]) > .001):
            events.append({"stage": int(g[S + "stage"].iloc[0]), "time_s": float(time[i]),
                           "actual_beta_deg": float(beta[g.index[i]]),
                           "load_plus_friction_model_nm": float(sign * g[m + "torque"].iloc[i])})
            last_event = time[i]
        # Allow up to 0.1 s for the filtered velocity to cross the motion threshold.
        if abs(velocity[i]) > .08 or time[i] - time[release_start] > .1:
            rest_start = release_start = None
    return events


def plot(data, report, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .2})
    t = (data[S + "control_steady_ns"] - data[S + "control_steady_ns"].iloc[0]) * 1e-9
    beta = np.rad2deg(data[S + "left/beta_estimate"])
    reference = np.rad2deg(data[S + "beta_reference"])
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    sample = np.arange(0, len(data), 20)
    ax = axes[0, 0]
    ax.plot(t.iloc[sample], reference.iloc[sample], label="Reference", lw=1)
    ax.plot(t.iloc[sample], beta.iloc[sample], label="Measured", lw=1)
    ax.set(xlabel="Time [s]", ylabel="Hip-knee included angle [deg]", title="A. Loaded trajectory")
    ax.legend()
    holds = pd.DataFrame(report["holds"])
    ax = axes[0, 1]
    for direction, label in ((1, "Opening"), (-1, "Closing")):
        part = holds[holds.direction == direction]
        ax.scatter(part.reference_beta_deg, part.actual_beta_deg, label=label, s=17)
    ax.plot([45, 105], [45, 105], color="gray", ls="--", label="Perfect tracking")
    ax.set(xlabel="Held reference [deg]", ylabel="Measured at end of hold [deg]", title="B. Static offset and direction dependence")
    ax.legend()
    ax = axes[1, 0]
    for joint, color in (("hip", "C0"), ("knee", "C1")):
        ax.scatter(holds.actual_beta_deg, holds[joint + "_model_torque_nm"], s=15, color=color, label=joint)
    ax.set(xlabel="Measured included angle [deg]", ylabel="Reported model-coordinate torque [Nm]", title="C. Combined spring + friction balancing load")
    ax.legend()
    ax = axes[1, 1]
    for stage, label in ((148, "Reference 50 deg"), (152, "Reference 75 deg")):
        g = data[data[S + "stage"] == stage]
        if len(g):
            ii = g.index[::5]
            ax.plot(t[ii] - t[g.index[0]], beta[ii] - beta[g.index].median(), label=label, lw=.8)
    ax.set(xlabel="Time within 3-10 Hz stage [s]", ylabel="Measured angle about its median [deg]", title="D. Small high-frequency motion and quantization")
    ax.legend()
    fig.suptitle("Horizontal suspended LEFT leg | PD 120 / 4 | 40 Nm cap | gravity omitted", fontsize=14)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    data, source = load(args.csv)
    report = {**source, **summarize(data)}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    plot(data, report, args.output_dir / "slow_sweep_analysis.png")
    print(json.dumps({key: report[key] for key in ("records", "duration_s", "observed_beta_deg", "axes", "completed", "validation_records")}, indent=2))
    print("Breakaway candidates:", len(report["breakaway_candidates"]))


if __name__ == "__main__":
    main()
