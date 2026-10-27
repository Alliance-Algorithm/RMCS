#!/usr/bin/env python3
"""Audit the wheel-leg sweep CSV archive and produce offline tuning evidence.

The logs are ValueCollector exports rather than the small CSV format consumed by
fit_friction_velocity_pid.py.  This script keeps the raw exports untouched and
uses the cached DataFrames made during the analysis when --cache-dir is given.
Without a cache it falls back to pandas.read_csv, so the command remains
reproducible on a clean checkout (at the cost of more memory and time).

The report intentionally distinguishes effective closed-loop quantities from a
validated open-loop mechanical model.  The leg sweep was run with position PD,
so static gas-spring load and position error are observable, while unique
inertia/damping parameters are not.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


ROOT = Path("/home/noir/Documents/workspace/wheel_leg扫频")
DEFAULT_OUT = Path("reports/wheel_leg_sweep_identification_20261027")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--data-dir", type=Path, default=ROOT)
    p.add_argument("--cache-dir", type=Path, default=None)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    return p.parse_args()


def json_default(value: object) -> object:
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"not JSON serializable: {type(value)!r}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(values: Iterable[float]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    return array[np.isfinite(array)]


def median(series: pd.Series) -> float | None:
    values = finite(series.to_numpy())
    return None if values.size == 0 else float(np.median(values))


def rms(series: pd.Series) -> float | None:
    values = finite(series.to_numpy())
    return None if values.size == 0 else float(np.sqrt(np.mean(values * values)))


def load_frame(path: Path, cache_dir: Path | None) -> pd.DataFrame:
    if cache_dir is not None:
        metadata = list(cache_dir.glob("*.json"))
        for item in metadata:
            try:
                info = json.loads(item.read_text())
            except json.JSONDecodeError:
                continue
            if info.get("path") == str(path):
                pickle_path = item.with_suffix(".pkl")
                if pickle_path.exists() and info.get("sha256") == sha256(path):
                    return pd.read_pickle(pickle_path)
    return pd.read_csv(path, low_memory=False)


def col(frame: pd.DataFrame, name: str) -> pd.Series:
    if name not in frame:
        return pd.Series(np.nan, index=frame.index, dtype=float)
    return frame[name]


def data_audit(data_dir: Path, cache_dir: Path | None) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for path in sorted(data_dir.glob("*.csv")):
        frame = load_frame(path, cache_dir)
        active_column = col(frame, "/wheel_leg/sweep/reference_active")
        state = col(frame, "/wheel_leg/sweep/state")
        active = active_column.eq(1)
        # run_elapsed/reference_elapsed reset at each repetition.  Prefer the
        # monotonic collector clock for the archive duration, then fall back to
        # the per-run elapsed fields for older exports.
        steady_ns = finite(col(frame, "/wheel_leg/sweep/control_steady_ns"))
        if steady_ns.size >= 2:
            duration_s = float((np.max(steady_ns) - np.min(steady_ns)) * 1e-9)
        else:
            run_elapsed = finite(col(frame, "/wheel_leg/sweep/run_elapsed_s"))
            if run_elapsed.size == 0:
                run_elapsed = finite(col(frame, "/wheel_leg/sweep/reference_elapsed_s"))
            duration_s = float(np.max(run_elapsed)) if run_elapsed.size else 0.0
        stages = finite(col(frame, "/wheel_leg/sweep/stage"))
        rows.append(
            {
                "file": path.name,
                "path": str(path),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
                "records": int(len(frame)),
                "duration_s": duration_s,
                "active_records": int(active.sum()),
                "state_counts": {str(k): int(v) for k, v in state.value_counts().items()},
                "stage_count": int(np.unique(stages[stages >= 0]).size),
                "validation_records": int(col(frame, "/wheel_leg/sweep/validation").eq(1).sum()),
                "mode_values": [int(v) for v in finite(col(frame, "/wheel_leg/sweep/mode")).astype(int).tolist()],
            }
        )
    # mode_values above is intentionally compacted below; a dataset can contain
    # hundreds of thousands of repeated values.
    for row in rows:
        row["mode_values"] = sorted(set(row["mode_values"]))
    return rows


def final_part(group: pd.DataFrame, fraction: float = 0.6) -> pd.DataFrame:
    elapsed = col(group, "/wheel_leg/sweep/stage_elapsed_s")
    if elapsed.notna().any():
        order = np.argsort(elapsed.to_numpy())
        start = max(0, int(len(order) * fraction))
        return group.iloc[order[start:]]
    return group.iloc[max(0, int(len(group) * fraction)) :]


def wheel_plateaus(frame: pd.DataFrame) -> list[dict[str, object]]:
    active = frame[
        col(frame, "/wheel_leg/sweep/reference_active").eq(1)
        & col(frame, "/wheel_leg/sweep/stage").ge(0)
        & col(frame, "/wheel_leg/sweep/run_id").isin([2, 3])
    ]
    records: list[dict[str, object]] = []
    groups = active.groupby(["/wheel_leg/sweep/run_id", "/wheel_leg/sweep/stage"])
    for (run, stage), group in groups:
        group = final_part(group)
        record: dict[str, object] = {"run": int(run), "stage": int(stage), "samples": int(len(group))}
        for side in ("left", "right"):
            prefix = f"/wheel_leg/{side}_wheel"
            sweep_prefix = f"/wheel_leg/sweep/{side}_wheel"
            record[f"{side}_reference_rad_s"] = median(col(group, f"{sweep_prefix}/reference_velocity"))
            record[f"{side}_velocity_rad_s"] = median(col(group, f"{prefix}/velocity"))
            record[f"{side}_error_rad_s"] = median(col(group, f"{sweep_prefix}/tracking_error"))
            record[f"{side}_command_nm"] = median(col(group, f"{sweep_prefix}/torque_preclip"))
            record[f"{side}_feedback_torque_nm"] = median(col(group, f"{prefix}/torque"))
            record[f"{side}_saturated_fraction"] = float(
                col(group, f"{sweep_prefix}/saturated").eq(1).mean()
            )
        records.append(record)
    return records


def wheel_tuning(plateaus: list[dict[str, object]]) -> dict[str, object]:
    operating: dict[str, list[dict[str, float]]] = {"left": [], "right": []}
    top_speed: dict[str, list[dict[str, float]]] = {"left": [], "right": []}
    for row in plateaus:
        for side in operating:
            reference = row.get(f"{side}_reference_rad_s")
            velocity = row.get(f"{side}_velocity_rad_s")
            error = row.get(f"{side}_error_rad_s")
            if reference is None or velocity is None or error is None:
                continue
            sample = {
                "run": float(row["run"]),
                "stage": float(row["stage"]),
                "reference_rad_s": float(reference),
                "velocity_rad_s": float(velocity),
                "error_rad_s": float(error),
            }
            if 30.0 <= abs(float(reference)) <= 58.0:
                operating[side].append(sample)
            if abs(float(reference)) >= 64.0:
                top_speed[side].append(sample)

    baseline_kp = 0.5
    candidate_kp = 0.75
    summary: dict[str, object] = {
        "baseline_kp": baseline_kp,
        "candidate_kp": candidate_kp,
        "ki": 0.0,
        "kd": 0.0,
        "operating_range_rad_s": [30.0, 58.0],
        "operating": {},
        "top_speed": {},
    }
    for side in operating:
        errors = np.asarray([abs(r["error_rad_s"]) for r in operating[side]], dtype=float)
        candidate = errors * baseline_kp / candidate_kp
        summary["operating"][side] = {
            "samples": int(errors.size),
            "baseline_mae_rad_s": float(np.mean(errors)) if errors.size else None,
            "candidate_static_mae_rad_s": float(np.mean(candidate)) if candidate.size else None,
            "baseline_max_abs_error_rad_s": float(np.max(errors)) if errors.size else None,
            "candidate_static_max_abs_error_rad_s": float(np.max(candidate)) if candidate.size else None,
        }
        summary["top_speed"][side] = {
            "samples": len(top_speed[side]),
            "max_measured_rad_s": max(
                (abs(r["velocity_rad_s"]) for r in top_speed[side]), default=None
            ),
            "target_rad_s": 65.0,
            "mean_abs_error_rad_s": (
                float(np.mean([abs(r["error_rad_s"]) for r in top_speed[side]]))
                if top_speed[side]
                else None
            ),
            "interpretation": "speed/voltage-limited; proportional gain scaling is not validated here",
        }
    return summary


def wheel_mechanical_fit(frame: pd.DataFrame, dataset: str) -> dict[str, object]:
    """Fit a deliberately small wheel model and retain its validation verdict.

    The model is an effective output-torque regression
    ``tau = J*ddq + B*dq + Fc*tanh(100*dq) + bias``.  It is useful for finding
    bad data or a speed-dependent regime, but it is not accepted as a motor
    plant unless the held-out R2 is strong and the parameters are physical.
    """

    active = (
        col(frame, "/wheel_leg/sweep/reference_active").eq(1)
        & col(frame, "/wheel_leg/sweep/state").eq(3)
        & col(frame, "/wheel_leg/sweep/stage").ge(0)
    )
    result: dict[str, object] = {"dataset": dataset, "axes": {}}
    for side in ("left", "right"):
        sweep = f"/wheel_leg/sweep/{side}_wheel"
        raw = f"/wheel_leg/{side}_wheel"
        selected = frame.loc[
            active,
            [
                f"{sweep}/model_velocity",
                f"{sweep}/model_acceleration",
                f"{raw}/torque",
                "/wheel_leg/sweep/validation",
            ],
        ].replace([np.inf, -np.inf], np.nan).dropna()
        if len(selected) < 100:
            result["axes"][side] = {"accepted": False, "reason": "too few finite samples"}
            continue
        velocity = selected[f"{sweep}/model_velocity"].to_numpy(float)
        acceleration = selected[f"{sweep}/model_acceleration"].to_numpy(float)
        torque = selected[f"{raw}/torque"].to_numpy(float)
        validation = selected["/wheel_leg/sweep/validation"].to_numpy(int)
        design = np.column_stack([acceleration, velocity, np.tanh(100.0 * velocity), np.ones_like(velocity)])
        train = validation == 0
        if int(train.sum()) < 50 or int((~train).sum()) < 50:
            result["axes"][side] = {"accepted": False, "reason": "missing independent validation"}
            continue
        coefficients = np.linalg.lstsq(design[train], torque[train], rcond=None)[0]
        prediction = design @ coefficients

        def metrics(mask: np.ndarray) -> dict[str, float]:
            residual = torque[mask] - prediction[mask]
            centered = torque[mask] - float(np.mean(torque[mask]))
            ss_total = float(centered @ centered)
            return {
                "samples": int(mask.sum()),
                "rmse_nm": float(np.sqrt(np.mean(residual * residual))),
                "r2": float(1.0 - (residual @ residual) / ss_total) if ss_total > 0 else float("nan"),
            }

        j, b, fc, bias = coefficients.tolist()
        val_metrics = metrics(~train)
        result["axes"][side] = {
            "J_kg_m2": float(j),
            "B_Nm_s_rad": float(b),
            "Fc_Nm": float(fc),
            "bias_nm": float(bias),
            "train": metrics(train),
            "validation": val_metrics,
            "accepted": bool(j > 0.0 and b >= 0.0 and fc >= 0.0 and val_metrics["r2"] >= 0.80),
            "model": "tau_fb = J*ddq + B*dq + Fc*tanh(100*dq) + bias",
        }
    return result


def fit_command_feedback_delay(
    frame: pd.DataFrame, axis: str, validation: int | None = None
) -> dict[str, object]:
    """Fit a first-order command-to-reported-torque effective delay.

    This is a telemetry-chain fit, not a pure CAN or drive latency measurement.
    Each stage is filtered independently so stage transitions do not create a
    false impulse.  The reported NRMSE is normalized by the feedback torque span.
    """

    active = col(frame, "/wheel_leg/sweep/reference_active").eq(1)
    if validation is not None:
        active &= col(frame, "/wheel_leg/sweep/validation").eq(validation)
    active &= col(frame, "/wheel_leg/sweep/stage").ge(0)
    source = col(frame, f"/wheel_leg/{axis}/control_torque").to_numpy(float)
    measured = col(frame, f"/wheel_leg/{axis}/torque").to_numpy(float)
    groups = frame.loc[active].groupby(
        ["/wheel_leg/sweep/run_id", "/wheel_leg/sweep/stage"], sort=False
    ).groups
    dt_values = finite(col(frame.loc[active], "/wheel_leg/sweep/control_dt_s"))
    dt = float(np.median(dt_values)) if dt_values.size else 0.001
    if not math.isfinite(dt) or dt <= 0:
        dt = 0.001

    def filtered(index: np.ndarray, delay_samples: int, tau: float) -> np.ndarray:
        u = source[index]
        z = np.empty_like(u)
        alpha = dt / (tau + dt)
        state = float(u[0])
        for i, value in enumerate(u):
            state += alpha * (value - state)
            z[i] = state
        if delay_samples:
            delayed = np.empty_like(z)
            delayed[:delay_samples] = z[0]
            delayed[delay_samples:] = z[:-delay_samples]
            return delayed
        return z

    indices = [np.asarray(index, dtype=int) for index in groups.values() if len(index) >= 20]
    if not indices:
        return {"accepted": False, "reason": "no active contiguous stages"}

    best: dict[str, object] | None = None
    for delay_samples in range(0, 6):
        for tau_ms in range(1, 21):
            tau = tau_ms / 1000.0
            all_z: list[np.ndarray] = []
            all_y: list[np.ndarray] = []
            for index in indices:
                z = filtered(index, delay_samples, tau)
                all_z.append(z)
                all_y.append(measured[index])
            z = np.concatenate(all_z)
            y = np.concatenate(all_y)
            ok = np.isfinite(z) & np.isfinite(y)
            if int(ok.sum()) < 50:
                continue
            design = np.column_stack([z[ok], np.ones(int(ok.sum()))])
            gain, bias = np.linalg.lstsq(design, y[ok], rcond=None)[0]
            residual = y[ok] - (gain * z[ok] + bias)
            span = max(float(np.percentile(np.abs(y[ok] - np.median(y[ok])), 95)), 1e-3)
            nrmse = float(np.sqrt(np.mean(residual * residual)) / span)
            candidate = {
                "delay_ms": delay_samples * dt * 1000.0,
                "time_constant_ms": float(tau_ms),
                "gain": float(gain),
                "bias_nm": float(bias),
                "nrmse": nrmse,
                "dt_s": dt,
                "samples": int(ok.sum()),
            }
            if best is None or nrmse < float(best["nrmse"]):
                best = candidate
    if best is None:
        return {"accepted": False, "reason": "fit failed"}
    best["accepted"] = bool(float(best["nrmse"]) <= 0.05)
    best["scope"] = "host command to reported current-based torque; transport/drive/measurement combined"
    return best


def leg_static_and_validation(frame: pd.DataFrame) -> dict[str, object]:
    active = (
        col(frame, "/wheel_leg/sweep/reference_active").eq(1)
        & col(frame, "/wheel_leg/sweep/run_id").isin([1, 2])
        & col(frame, "/wheel_leg/sweep/stage").ge(0)
    )
    selected = frame.loc[active].copy()
    selected["beta_ref_deg"] = np.rad2deg(col(selected, "/wheel_leg/sweep/beta_reference"))
    selected["beta_actual_deg"] = np.rad2deg(col(selected, "/wheel_leg/sweep/left/beta_estimate"))
    selected["beta_bin"] = (selected["beta_ref_deg"] / 25.0).round() * 25.0

    holds: list[dict[str, object]] = []
    for (run, stage), group in selected.groupby(["/wheel_leg/sweep/run_id", "/wheel_leg/sweep/stage"]):
        group = final_part(group)
        beta = median(group["beta_ref_deg"])
        if beta is None or min(abs(beta - target) for target in (50, 75, 100)) > 4:
            continue
        record: dict[str, object] = {
            "run": int(run),
            "stage": int(stage),
            "validation": int(col(group, "/wheel_leg/sweep/validation").median()),
            "beta_reference_deg": beta,
            "beta_actual_deg": median(group["beta_actual_deg"]),
        }
        for axis in ("left_hip_joint", "left_knee_joint"):
            prefix = f"/wheel_leg/{axis}"
            sweep_prefix = f"/wheel_leg/sweep/{axis}"
            record[f"{axis}_position_error_rad"] = median(col(group, f"{sweep_prefix}/tracking_error"))
            record[f"{axis}_position_error_rms_rad"] = rms(col(group, f"{sweep_prefix}/tracking_error"))
            record[f"{axis}_api_torque_nm"] = median(col(group, f"{prefix}/torque"))
            record[f"{axis}_model_torque_nm"] = -record[f"{axis}_api_torque_nm"] if record[f"{axis}_api_torque_nm"] is not None else None
            record[f"{axis}_model_velocity_rms_rad_s"] = rms(col(group, f"{sweep_prefix}/model_velocity"))
        holds.append(record)

    # Average repeated train/holdout points and predict the residual position
    # error after increasing Kp from the old 120 used for this archive to 160.
    points: dict[str, list[dict[str, float]]] = {"50": [], "75": [], "100": []}
    for row in holds:
        key = str(min((50, 75, 100), key=lambda target: abs(row["beta_reference_deg"] - target)))
        points[key].append(row)
    static: list[dict[str, object]] = []
    for beta_key, rows in points.items():
        if not rows:
            continue
        out: dict[str, object] = {"beta_reference_deg": int(beta_key), "samples": len(rows)}
        actual = [r["beta_actual_deg"] for r in rows if r["beta_actual_deg"] is not None]
        out["beta_actual_deg_median"] = float(np.median(actual)) if actual else None
        for axis in ("left_hip_joint", "left_knee_joint"):
            errors = np.asarray([r[f"{axis}_position_error_rad"] for r in rows], dtype=float)
            torques = np.asarray([r[f"{axis}_model_torque_nm"] for r in rows], dtype=float)
            errors = errors[np.isfinite(errors)]
            torques = torques[np.isfinite(torques)]
            out[f"{axis}_error_deg_baseline"] = float(np.rad2deg(np.median(errors))) if errors.size else None
            out[f"{axis}_error_deg_kp160_static_prediction"] = (
                float(np.rad2deg(np.median(errors)) * 120.0 / 160.0) if errors.size else None
            )
            out[f"{axis}_model_torque_nm_median"] = float(np.median(torques)) if torques.size else None
        static.append(out)

    # The old log's velocity signal is a signed model velocity.  It is close to
    # the numerical derivative of model angle in the held/validation windows;
    # report that as a diagnostic rather than a plant fit.
    velocity_diag: dict[str, object] = {}
    for axis in ("left_hip_joint", "left_knee_joint"):
        values: list[dict[str, float]] = []
        for (run, stage), group in selected.groupby(["/wheel_leg/sweep/run_id", "/wheel_leg/sweep/stage"]):
            group = final_part(group)
            t = col(group, "/wheel_leg/sweep/run_elapsed_s").to_numpy(float)
            q = col(group, f"/wheel_leg/sweep/{axis}/model_angle").to_numpy(float)
            v = col(group, f"/wheel_leg/sweep/{axis}/model_velocity").to_numpy(float)
            mask = np.isfinite(t) & np.isfinite(q) & np.isfinite(v)
            if int(mask.sum()) < 20:
                continue
            t, q, v = t[mask], q[mask], v[mask]
            order = np.argsort(t)
            t, q, v = t[order], q[order], v[order]
            unique = np.r_[True, np.diff(t) > 0]
            t, q, v = t[unique], q[unique], v[unique]
            if len(t) < 20:
                continue
            derivative = np.gradient(q, t)
            corr = np.corrcoef(derivative, v)[0, 1]
            values.append({"run": int(run), "stage": int(stage), "correlation": float(corr), "rmse_rad_s": float(np.sqrt(np.mean((derivative - v) ** 2)))})
        if values:
            velocity_diag[axis] = {
                "segments": len(values),
                "median_correlation": float(np.median([x["correlation"] for x in values])),
                "median_rmse_rad_s": float(np.median([x["rmse_rad_s"] for x in values])),
            }
    return {
        "controller_gains_in_archive": {"kp": 120.0, "kd": 4.0, "torque_limit_nm": 40.0},
        "hold_points": holds,
        "static_kp_scaling": static,
        "velocity_diagnostic": velocity_diag,
        "command_feedback_delay": {
            axis: fit_command_feedback_delay(frame, axis)
            for axis in ("left_hip_joint", "left_knee_joint")
        },
        "mechanical_model_status": "rejected_for_real2sim",
        "mechanical_model_reason": "The log is closed-loop position PD. Static gas-spring load dominates the torque; unique J/B/Fc parameters are not independently observable from these runs.",
    }


def main() -> int:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    files = {path.name: path for path in args.data_dir.glob("*.csv")}
    required = [
        "左腿关节不同夹角扫频.csv",
        "wheel_leg_轮子高速辨识数据.csv",
    ]
    missing = [name for name in required if name not in files]
    if missing:
        raise SystemExit(f"missing required data files: {missing}")

    audit = data_audit(args.data_dir, args.cache_dir)
    wheel_frame = load_frame(files["wheel_leg_轮子高速辨识数据.csv"], args.cache_dir)
    wheel_low_frame = load_frame(files["wheel_leg_轮子低速辨识数据.csv"], args.cache_dir)
    leg_frame = load_frame(files["左腿关节不同夹角扫频.csv"], args.cache_dir)
    plateaus = wheel_plateaus(wheel_frame)
    wheel = wheel_tuning(plateaus)
    wheel["mechanical_fit"] = {
        "low_speed": wheel_mechanical_fit(wheel_low_frame, "wheel_leg_轮子低速辨识数据.csv"),
        "high_speed": wheel_mechanical_fit(wheel_frame, "wheel_leg_轮子高速辨识数据.csv"),
        "status": "effective regression only; no wheel model accepted for real2sim",
    }
    leg = leg_static_and_validation(leg_frame)

    (args.out / "data_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2, default=json_default) + "\n")
    (args.out / "wheel_plateaus.json").write_text(json.dumps(plateaus, ensure_ascii=False, indent=2, default=json_default) + "\n")
    (args.out / "wheel_optimization.json").write_text(json.dumps(wheel, ensure_ascii=False, indent=2, default=json_default) + "\n")
    (args.out / "leg_identification.json").write_text(json.dumps(leg, ensure_ascii=False, indent=2, default=json_default) + "\n")
    print(json.dumps({"out": str(args.out), "datasets": len(audit), "wheel_plateaus": len(plateaus), "leg_hold_points": len(leg["hold_points"])}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
