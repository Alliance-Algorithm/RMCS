#!/usr/bin/env python3
"""Audit the archived leg data for a conservative, non-integrating P/P start.

This evaluates commands on recorded states, not the response of a new controller.
It does not fit an accepted inertia/damping model or infer a new closed-loop
bandwidth. Raw CSVs are never modified. Cache lookup uses the archive reader's
source/hash validation, and --cache-dir is optional.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from analyze_wheel_leg_sweep_archive import load_frame, sha256


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.data_dir / "左腿关节不同夹角扫频.csv"
    frame = load_frame(source, args.cache_dir)
    sweep = "/wheel_leg/sweep/"
    active = frame[(frame[sweep + "reference_active"] == 1)
                   & frame[sweep + "run_id"].isin([1, 2])]
    position_p, velocity_p, velocity_cap, torque_cap = 40., 4., 8.5, 40.
    points = []
    # A complete training chirp and independent holdout at each pose in each run.
    for beta, stages in ((50, (2, 3)), (75, (7, 8)), (100, (12, 13))):
        group = active[active[sweep + "stage"].isin(stages)]
        row = {"beta_reference_deg": beta, "samples": len(group), "segments": []}
        for (run, stage), segment in group.groupby([sweep + "run_id", sweep + "stage"]):
            row["segments"].append({"run": int(run), "stage": int(stage),
                                    "samples": len(segment),
                                    "validation": int(segment[sweep + "validation"].median())})
        row["beta_actual_deg_median"] = float(np.rad2deg(group[sweep + "left/beta_estimate"].median()))
        for axis in ("left_hip_joint", "left_knee_joint"):
            prefix = sweep + axis + "/"
            error = group[prefix + "tracking_error"].to_numpy(float)
            velocity = group[prefix + "model_velocity"].to_numpy(float)
            reference_velocity = group[prefix + "reference_velocity"].to_numpy(float)
            torque = -group["/wheel_leg/" + axis + "/torque"].to_numpy(float)
            good = np.isfinite(error) & np.isfinite(velocity) & np.isfinite(torque) & np.isfinite(reference_velocity)
            error, velocity, torque = error[good], velocity[good], torque[good]
            reference_velocity = reference_velocity[good]
            raw_target = reference_velocity + position_p * error
            target = np.clip(raw_target, -velocity_cap, velocity_cap)
            requested = velocity_p * (target - velocity)
            row[axis] = {
                "model_torque_nm_median": float(np.median(torque)),
                "error_deg_median": float(np.rad2deg(np.median(error))),
                "error_deg_constant_load_estimate_at_k160": float(np.rad2deg(np.median(torque) / 160)),
                "velocity_target_clipped_fraction_on_recorded_states": float(np.mean(abs(raw_target) > velocity_cap)),
                "torque_clipped_fraction_on_recorded_states": float(np.mean(abs(requested) > torque_cap)),
            }
        points.append(row)
    rotation = active[active[sweep + "stage"].isin([4, 5, 9, 10, 14, 15])]
    fast = {}
    for axis in ("left_hip_joint", "left_knee_joint"):
        reference_v = rotation[sweep + axis + "/reference_velocity"].dropna().abs()
        actual_v = rotation[sweep + axis + "/model_velocity"].dropna().abs()
        fast[axis] = {"reference_speed_p99_rad_s": float(reference_v.quantile(.99)),
                      "actual_speed_p99_rad_s": float(actual_v.quantile(.99))}
    knee_source = args.data_dir / "wheel_leg_膝关节气簧辨识数据.csv"
    knee = load_frame(knee_source, args.cache_dir)
    knee_active = knee[(knee[sweep + "reference_active"] == 1) & (knee[sweep + "run_id"] == 1)]
    noise = {}
    for axis in ("left_hip_joint", "left_knee_joint"):
        differences = []
        for stage in (38, 39, 41, 42):
            segment = knee_active[knee_active[sweep + "stage"] == stage]
            t = segment[sweep + "control_steady_ns"].to_numpy(float) * 1e-9
            v = segment[sweep + axis + "/model_velocity"].to_numpy(float)
            dt, dv = np.diff(t), np.diff(v)
            good = np.isfinite(dv) & (dt > .0005) & (dt < .002)
            differences.append(dv[good] / dt[good])
        acceleration = np.concatenate(differences)
        noise[axis] = {"raw_velocity_difference_acceleration_abs_p99_rad_s2": float(np.quantile(abs(acceleration), .99))}
    result = {
        "status": "conservative_starting_gains_not_dynamic_optimum_or_hardware_validation",
        "source": str(source), "source_sha256": sha256(source),
        "knee_source": str(knee_source), "knee_source_sha256": sha256(knee_source),
        "coordinates": "model rad/rad_s/Nm; tau_model=-tau_api for model_sign=-1",
        "gains": {"position_p_per_s": position_p, "velocity_p_nm_s_per_rad": velocity_p,
                  "velocity_i": 0., "velocity_d": 0., "velocity_target_limit_rad_s": velocity_cap,
                  "torque_limit_nm": torque_cap, "equivalent_position_kp_nm_per_rad": 160.,
                  "equivalent_position_kd_nm_s_per_rad": 4.},
        "static_effective_load_and_recorded_state_command_audit": points,
        "rotation_speed": fast, "derivative_diagnostic": noise,
        "limitations": ["Recorded-state clipping fractions are not closed-loop replay or a performance prediction.",
                        "Constant-load estimates omit changing spring stiffness, hysteresis and friction.",
                        "No independently accepted J/B model or inner-loop bandwidth was obtained.",
                        "Right-leg gains are mirrored starting values; independent right-leg validation is still required.",
                        "Raw velocity differences include both real acceleration and quantization; they do not identify sensor noise alone."],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"gains": result["gains"], "rotation_speed": fast,
                      "derivative_diagnostic": noise}, indent=2))


if __name__ == "__main__":
    main()
