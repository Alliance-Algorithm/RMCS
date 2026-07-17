#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    from scipy.optimize import least_squares
except ModuleNotFoundError:
    least_squares = None


@dataclass
class OdeSegment:
    dt: np.ndarray
    angle: np.ndarray
    velocity: np.ndarray
    tau: np.ndarray


@dataclass
class GrayboxFitResult:
    fit_mode: str
    signal_name: str
    residual_name: str
    gravity_mode: str
    sample_count: int
    dt: float
    window_length: int
    poly_order: int
    tanh_gain: float
    angle_min: float
    angle_max: float
    angle_span: float
    inertia: float
    viscous_damping: float
    coulomb_friction: float
    gravity_gain: float
    gravity_phase: float
    gravity_phase_deg: float
    gravity_zero_crossing: float
    gravity_zero_crossing_deg: float
    torque_bias: float
    rmse: float
    mae: float
    r2: float
    design_condition: float
    shooting_window: float
    segment_count: int
    optimizer_nfev: int
    optimizer_status: int
    optimizer_success: bool
    optimizer_message: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit the gray-box sweep model "
            "J * theta_dd + B * theta_d + Fc * tanh(k * theta_d) + G * sin(theta + phi) = tau_ff"
        )
    )
    parser.add_argument("csv_path", type=Path, help="Path to swept_frequency_controller CSV")
    parser.add_argument(
        "--target",
        type=str,
        default=None,
        help="Target prefix such as /gimbal/pitch. If omitted, infer from CSV headers.",
    )
    parser.add_argument(
        "--input-signal",
        choices=("control_torque", "torque"),
        default="torque",
        help="Signal used as tau_ff. Defaults to torque.",
    )
    parser.add_argument(
        "--fit-mode",
        choices=("linear", "ode-angle"),
        default="linear",
        help="linear uses differentiated-velocity regression; ode-angle integrates the ODE and fits angle only.",
    )
    parser.add_argument(
        "--window-length",
        type=int,
        default=31,
        help="Odd Savitzky-Golay window length for smoothing velocity. Defaults to 31.",
    )
    parser.add_argument(
        "--poly-order",
        type=int,
        default=3,
        help="Savitzky-Golay polynomial order for velocity smoothing and linear-mode differentiation.",
    )
    parser.add_argument(
        "--tanh-gain",
        type=float,
        default=100.0,
        help="Gain k used in Fc * tanh(k * theta_d). Defaults to 100.",
    )
    parser.add_argument(
        "--trim-start",
        type=float,
        default=0.0,
        help="Discard samples before this elapsed time in seconds.",
    )
    parser.add_argument(
        "--trim-end",
        type=float,
        default=0.0,
        help="Discard samples within this many seconds from the end of the file.",
    )
    parser.add_argument(
        "--shooting-window",
        type=float,
        default=0.5,
        help="Segment duration in seconds for ode-angle multiple shooting. Defaults to 0.5.",
    )
    parser.add_argument(
        "--ode-max-nfev",
        type=int,
        default=200,
        help="Maximum least-squares function evaluations for ode-angle mode. Defaults to 200.",
    )
    parser.add_argument(
        "--fixed-gravity-gain",
        type=float,
        default=None,
        help="If set together with --fixed-gravity-phase, hold gravity gain G fixed.",
    )
    parser.add_argument(
        "--fixed-gravity-phase",
        type=float,
        default=None,
        help="If set together with --fixed-gravity-gain, hold gravity phase phi fixed in radians.",
    )
    parser.add_argument(
        "--json-output",
        type=Path,
        default=None,
        help="Optional path to write fit summary as JSON.",
    )
    return parser.parse_args()


def infer_target(fieldnames: Iterable[str]) -> str:
    angle_targets = {name[: -len("/angle")] for name in fieldnames if name.endswith("/angle")}
    if len(angle_targets) != 1:
        raise ValueError(
            "Failed to infer target from CSV headers. Please pass --target explicitly."
        )
    return next(iter(angle_targets))


def read_rows(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)
        if reader.fieldnames is None:
            raise ValueError(f"CSV file has no header: {path}")
        return rows, reader.fieldnames


def require_columns(fieldnames: list[str], target: str) -> dict[str, str]:
    columns = {
        "elapsed_s": "elapsed_s",
        "control_torque": f"{target}/control_torque",
        "torque": f"{target}/torque",
        "velocity": f"{target}/velocity",
        "angle": f"{target}/angle",
    }

    missing = [column for column in columns.values() if column not in fieldnames]
    if missing:
        joined = ", ".join(missing)
        raise ValueError(f"Missing required CSV columns: {joined}")
    return columns


def validate_args(args: argparse.Namespace) -> None:
    if args.fit_mode == "ode-angle" and least_squares is None:
        raise ValueError(
            "ode-angle mode requires scipy. Install it in .venv or run with a Python environment that has scipy."
        )
    if args.window_length < 5 or args.window_length % 2 == 0:
        raise ValueError("window-length must be an odd integer >= 5")
    if args.poly_order < 2:
        raise ValueError("poly-order must be >= 2")
    if args.poly_order >= args.window_length:
        raise ValueError("poly-order must be smaller than window-length")
    if args.tanh_gain <= 0.0 or not math.isfinite(args.tanh_gain):
        raise ValueError("tanh-gain must be finite and positive")
    if args.trim_start < 0.0 or args.trim_end < 0.0:
        raise ValueError("trim-start and trim-end must be non-negative")
    if args.shooting_window <= 0.0 or not math.isfinite(args.shooting_window):
        raise ValueError("shooting-window must be finite and positive")
    if args.ode_max_nfev <= 0:
        raise ValueError("ode-max-nfev must be positive")
    fixed_gain_set = args.fixed_gravity_gain is not None
    fixed_phase_set = args.fixed_gravity_phase is not None
    if fixed_gain_set != fixed_phase_set:
        raise ValueError(
            "fixed-gravity-gain and fixed-gravity-phase must be provided together"
        )
    if fixed_gain_set and (
        not math.isfinite(args.fixed_gravity_gain)
        or not math.isfinite(args.fixed_gravity_phase)
        or args.fixed_gravity_gain < 0.0
    ):
        raise ValueError(
            "fixed gravity parameters must be finite and fixed-gravity-gain must be non-negative"
        )


def savitzky_golay(y: np.ndarray, dt: float, window_length: int, poly_order: int, deriv: int) -> np.ndarray:
    half_window = window_length // 2
    offsets = np.arange(-half_window, half_window + 1, dtype=float) * dt
    vandermonde = np.vstack([offsets**order for order in range(poly_order + 1)]).T
    coefficients = math.factorial(deriv) * np.linalg.pinv(vandermonde)[deriv]
    windows = np.lib.stride_tricks.sliding_window_view(y, window_length)
    return windows @ coefficients


def compute_metrics(measured: np.ndarray, prediction: np.ndarray) -> tuple[float, float, float]:
    residual = measured - prediction
    centered = measured - float(np.mean(measured))
    ss_res = float(residual @ residual)
    ss_tot = float(centered @ centered)
    rmse = math.sqrt(ss_res / measured.size)
    mae = float(np.mean(np.abs(residual)))
    r2 = float("nan") if ss_tot <= 0.0 else 1.0 - ss_res / ss_tot
    return rmse, mae, r2


def fit_linear_graybox(
    angle: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    tau_ff: np.ndarray,
    signal_name: str,
    dt: float,
    window_length: int,
    poly_order: int,
    tanh_gain: float,
) -> GrayboxFitResult:
    regressors = np.column_stack(
        [
            acceleration,
            velocity,
            np.tanh(tanh_gain * velocity),
            np.sin(angle),
            np.cos(angle),
        ]
    )

    coefficients, _, _, _ = np.linalg.lstsq(regressors, tau_ff, rcond=None)
    prediction = regressors @ coefficients

    inertia, viscous_damping, coulomb_friction, gravity_sin, gravity_cos = coefficients.tolist()
    gravity_gain = math.hypot(gravity_sin, gravity_cos)
    gravity_phase = math.atan2(gravity_cos, gravity_sin)

    rmse, mae, r2 = compute_metrics(tau_ff, prediction)

    return GrayboxFitResult(
        fit_mode="linear",
        signal_name=signal_name,
        residual_name=signal_name,
        gravity_mode="free",
        sample_count=int(tau_ff.size),
        dt=dt,
        window_length=window_length,
        poly_order=poly_order,
        tanh_gain=tanh_gain,
        angle_min=float(np.min(angle)),
        angle_max=float(np.max(angle)),
        angle_span=float(np.max(angle) - np.min(angle)),
        inertia=float(inertia),
        viscous_damping=float(viscous_damping),
        coulomb_friction=float(coulomb_friction),
        gravity_gain=float(gravity_gain),
        gravity_phase=float(gravity_phase),
        gravity_phase_deg=float(math.degrees(gravity_phase)),
        gravity_zero_crossing=float(-gravity_phase),
        gravity_zero_crossing_deg=float(math.degrees(-gravity_phase)),
        torque_bias=0.0,
        rmse=rmse,
        mae=mae,
        r2=r2,
        design_condition=float(np.linalg.cond(regressors)),
        shooting_window=float("nan"),
        segment_count=1,
        optimizer_nfev=0,
        optimizer_status=0,
        optimizer_success=True,
        optimizer_message="linear least squares",
    )


def fit_fixed_gravity_graybox(
    angle: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    tau_ff: np.ndarray,
    signal_name: str,
    dt: float,
    window_length: int,
    poly_order: int,
    tanh_gain: float,
    gravity_gain: float,
    gravity_phase: float,
) -> GrayboxFitResult:
    gravity_term = gravity_gain * np.sin(angle + gravity_phase)
    regressors = np.column_stack(
        [
            acceleration,
            velocity,
            np.tanh(tanh_gain * velocity),
        ]
    )

    coefficients, _, _, _ = np.linalg.lstsq(regressors, tau_ff - gravity_term, rcond=None)
    prediction = regressors @ coefficients + gravity_term

    inertia, viscous_damping, coulomb_friction = coefficients.tolist()

    rmse, mae, r2 = compute_metrics(tau_ff, prediction)

    return GrayboxFitResult(
        fit_mode="linear",
        signal_name=signal_name,
        residual_name=signal_name,
        gravity_mode="fixed",
        sample_count=int(tau_ff.size),
        dt=dt,
        window_length=window_length,
        poly_order=poly_order,
        tanh_gain=tanh_gain,
        angle_min=float(np.min(angle)),
        angle_max=float(np.max(angle)),
        angle_span=float(np.max(angle) - np.min(angle)),
        inertia=float(inertia),
        viscous_damping=float(viscous_damping),
        coulomb_friction=float(coulomb_friction),
        gravity_gain=float(gravity_gain),
        gravity_phase=float(gravity_phase),
        gravity_phase_deg=float(math.degrees(gravity_phase)),
        gravity_zero_crossing=float(-gravity_phase),
        gravity_zero_crossing_deg=float(math.degrees(-gravity_phase)),
        torque_bias=0.0,
        rmse=rmse,
        mae=mae,
        r2=r2,
        design_condition=float(np.linalg.cond(regressors)),
        shooting_window=float("nan"),
        segment_count=1,
        optimizer_nfev=0,
        optimizer_status=0,
        optimizer_success=True,
        optimizer_message="linear least squares",
    )


def build_ode_segments(
    elapsed: np.ndarray,
    angle: np.ndarray,
    velocity: np.ndarray,
    tau_ff: np.ndarray,
    shooting_window: float,
    dt: float,
) -> list[OdeSegment]:
    segment_samples = max(2, int(round(shooting_window / dt)) + 1)
    segments: list[OdeSegment] = []
    start = 0
    while start < angle.size - 1:
        stop = min(start + segment_samples, angle.size)
        if stop - start < 2:
            break
        elapsed_segment = elapsed[start:stop]
        segments.append(
            OdeSegment(
                dt=np.diff(elapsed_segment),
                angle=angle[start:stop],
                velocity=velocity[start:stop],
                tau=tau_ff[start:stop],
            )
        )
        if stop == angle.size:
            break
        start = stop - 1
    return segments


def build_ode_initial_guess(
    angle: np.ndarray,
    velocity: np.ndarray,
    tau_ff: np.ndarray,
    tanh_gain: float,
    fixed_gravity_gain: float | None,
    fixed_gravity_phase: float | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tanh_velocity = np.tanh(tanh_gain * velocity)
    tau_scale = max(1.0, float(np.percentile(np.abs(tau_ff), 95)))
    velocity_scale = max(1e-3, float(np.percentile(np.abs(velocity), 95)))
    damping_upper = max(10.0, 50.0 * tau_scale / velocity_scale)
    friction_upper = max(10.0, 10.0 * tau_scale)
    gravity_upper = max(10.0, 10.0 * tau_scale)
    bias_limit = max(10.0, 5.0 * tau_scale)

    if fixed_gravity_gain is None:
        regressors = np.column_stack(
            [
                velocity,
                tanh_velocity,
                np.sin(angle),
                np.cos(angle),
                np.ones_like(angle),
            ]
        )
        coefficients, _, _, _ = np.linalg.lstsq(regressors, tau_ff, rcond=None)
        damping0, friction0, gravity_sin0, gravity_cos0, bias0 = coefficients.tolist()
        gravity_gain0 = math.hypot(gravity_sin0, gravity_cos0)
        gravity_phase0 = math.atan2(gravity_cos0, gravity_sin0)
        x0 = np.array(
            [
                0.01,
                abs(damping0),
                abs(friction0),
                max(1e-6, gravity_gain0),
                gravity_phase0,
                bias0,
            ],
            dtype=float,
        )
        lower = np.array([1e-6, 0.0, 0.0, 0.0, -4.0 * math.pi, -bias_limit], dtype=float)
        upper = np.array(
            [10.0, damping_upper, friction_upper, gravity_upper, 4.0 * math.pi, bias_limit],
            dtype=float,
        )
        return np.clip(x0, lower, upper), lower, upper

    gravity_term = fixed_gravity_gain * np.sin(angle + fixed_gravity_phase)
    regressors = np.column_stack([velocity, tanh_velocity, np.ones_like(angle)])
    coefficients, _, _, _ = np.linalg.lstsq(regressors, tau_ff - gravity_term, rcond=None)
    damping0, friction0, bias0 = coefficients.tolist()
    x0 = np.array([0.01, abs(damping0), abs(friction0), bias0], dtype=float)
    lower = np.array([1e-6, 0.0, 0.0, -bias_limit], dtype=float)
    upper = np.array([10.0, damping_upper, friction_upper, bias_limit], dtype=float)
    return np.clip(x0, lower, upper), lower, upper


def unpack_ode_parameters(
    parameters: np.ndarray,
    fixed_gravity_gain: float | None,
    fixed_gravity_phase: float | None,
) -> tuple[float, float, float, float, float, float]:
    if fixed_gravity_gain is None:
        inertia, viscous_damping, coulomb_friction, gravity_gain, gravity_phase, torque_bias = (
            parameters.tolist()
        )
        return (
            float(inertia),
            float(viscous_damping),
            float(coulomb_friction),
            float(gravity_gain),
            float(gravity_phase),
            float(torque_bias),
        )

    inertia, viscous_damping, coulomb_friction, torque_bias = parameters.tolist()
    return (
        float(inertia),
        float(viscous_damping),
        float(coulomb_friction),
        float(fixed_gravity_gain),
        float(fixed_gravity_phase),
        float(torque_bias),
    )


def simulate_segment_angle(
    segment: OdeSegment,
    inertia: float,
    viscous_damping: float,
    coulomb_friction: float,
    gravity_gain: float,
    gravity_phase: float,
    tanh_gain: float,
    torque_bias: float,
) -> np.ndarray:
    def angular_acceleration(theta: float, omega: float, tau: float) -> float:
        return (
            tau
            + torque_bias
            - viscous_damping * omega
            - coulomb_friction * math.tanh(tanh_gain * omega)
            - gravity_gain * math.sin(theta + gravity_phase)
        ) / inertia

    predicted = np.empty_like(segment.angle)
    theta = float(segment.angle[0])
    omega = float(segment.velocity[0])
    predicted[0] = theta

    for index, delta_t in enumerate(segment.dt):
        tau0 = float(segment.tau[index])
        tau1 = float(segment.tau[index + 1])
        tau_mid = 0.5 * (tau0 + tau1)

        k1_theta = omega
        k1_omega = angular_acceleration(theta, omega, tau0)

        theta_k2 = theta + 0.5 * delta_t * k1_theta
        omega_k2 = omega + 0.5 * delta_t * k1_omega
        k2_theta = omega_k2
        k2_omega = angular_acceleration(theta_k2, omega_k2, tau_mid)

        theta_k3 = theta + 0.5 * delta_t * k2_theta
        omega_k3 = omega + 0.5 * delta_t * k2_omega
        k3_theta = omega_k3
        k3_omega = angular_acceleration(theta_k3, omega_k3, tau_mid)

        theta_k4 = theta + delta_t * k3_theta
        omega_k4 = omega + delta_t * k3_omega
        k4_theta = omega_k4
        k4_omega = angular_acceleration(theta_k4, omega_k4, tau1)

        theta += (delta_t / 6.0) * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta)
        omega += (delta_t / 6.0) * (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega)
        predicted[index + 1] = theta

    return predicted


def simulate_shooting_prediction(
    segments: list[OdeSegment],
    inertia: float,
    viscous_damping: float,
    coulomb_friction: float,
    gravity_gain: float,
    gravity_phase: float,
    tanh_gain: float,
    torque_bias: float,
) -> tuple[np.ndarray, np.ndarray]:
    measured_parts: list[np.ndarray] = []
    predicted_parts: list[np.ndarray] = []
    for segment in segments:
        predicted = simulate_segment_angle(
            segment,
            inertia,
            viscous_damping,
            coulomb_friction,
            gravity_gain,
            gravity_phase,
            tanh_gain,
            torque_bias,
        )
        measured_parts.append(segment.angle[1:])
        predicted_parts.append(predicted[1:])
    return np.concatenate(measured_parts), np.concatenate(predicted_parts)


def fit_ode_angle_graybox(
    elapsed: np.ndarray,
    angle: np.ndarray,
    velocity: np.ndarray,
    tau_ff: np.ndarray,
    signal_name: str,
    dt: float,
    window_length: int,
    poly_order: int,
    tanh_gain: float,
    shooting_window: float,
    ode_max_nfev: int,
    fixed_gravity_gain: float | None,
    fixed_gravity_phase: float | None,
) -> GrayboxFitResult:
    segments = build_ode_segments(elapsed, angle, velocity, tau_ff, shooting_window, dt)
    if not segments:
        raise ValueError("Too few samples remain to build ODE shooting segments.")

    x0, lower, upper = build_ode_initial_guess(
        angle, velocity, tau_ff, tanh_gain, fixed_gravity_gain, fixed_gravity_phase
    )

    def residual_function(parameters: np.ndarray) -> np.ndarray:
        (
            inertia,
            viscous_damping,
            coulomb_friction,
            gravity_gain,
            gravity_phase,
            torque_bias,
        ) = unpack_ode_parameters(parameters, fixed_gravity_gain, fixed_gravity_phase)
        measured, predicted = simulate_shooting_prediction(
            segments,
            inertia,
            viscous_damping,
            coulomb_friction,
            gravity_gain,
            gravity_phase,
            tanh_gain,
            torque_bias,
        )
        return predicted - measured

    optimization = least_squares(
        residual_function,
        x0,
        bounds=(lower, upper),
        method="trf",
        x_scale=np.maximum(np.abs(x0), 1e-3),
        max_nfev=ode_max_nfev,
    )

    (
        inertia,
        viscous_damping,
        coulomb_friction,
        gravity_gain,
        gravity_phase,
        torque_bias,
    ) = unpack_ode_parameters(
        optimization.x, fixed_gravity_gain, fixed_gravity_phase
    )
    measured_angle, predicted_angle = simulate_shooting_prediction(
        segments,
        inertia,
        viscous_damping,
        coulomb_friction,
        gravity_gain,
        gravity_phase,
        tanh_gain,
        torque_bias,
    )
    rmse, mae, r2 = compute_metrics(measured_angle, predicted_angle)

    return GrayboxFitResult(
        fit_mode="ode-angle",
        signal_name=signal_name,
        residual_name="angle",
        gravity_mode="fixed" if fixed_gravity_gain is not None else "free",
        sample_count=int(angle.size),
        dt=dt,
        window_length=window_length,
        poly_order=poly_order,
        tanh_gain=tanh_gain,
        angle_min=float(np.min(angle)),
        angle_max=float(np.max(angle)),
        angle_span=float(np.max(angle) - np.min(angle)),
        inertia=float(inertia),
        viscous_damping=float(viscous_damping),
        coulomb_friction=float(coulomb_friction),
        gravity_gain=float(gravity_gain),
        gravity_phase=float(gravity_phase),
        gravity_phase_deg=float(math.degrees(gravity_phase)),
        gravity_zero_crossing=float(-gravity_phase),
        gravity_zero_crossing_deg=float(math.degrees(-gravity_phase)),
        torque_bias=float(torque_bias),
        rmse=rmse,
        mae=mae,
        r2=r2,
        design_condition=float("nan"),
        shooting_window=shooting_window,
        segment_count=len(segments),
        optimizer_nfev=int(optimization.nfev),
        optimizer_status=int(optimization.status),
        optimizer_success=bool(optimization.success),
        optimizer_message=str(optimization.message),
    )


def print_fit(result: GrayboxFitResult, label: str) -> None:
    print(f"{label}:")
    print(f"  fit mode: {result.fit_mode}")
    print(f"  signal: {result.signal_name}")
    print(f"  residual: {result.residual_name}")
    print(f"  gravity mode: {result.gravity_mode}")
    print(f"  samples: {result.sample_count}")
    print(
        "  angle span: "
        f"[{result.angle_min:.6f}, {result.angle_max:.6f}] rad "
        f"({math.degrees(result.angle_span):.2f} deg)"
    )
    print(
        "  model: "
        f"{result.inertia:.6f} * theta_dd + {result.viscous_damping:.6f} * theta_d "
        f"+ {result.coulomb_friction:.6f} * tanh({result.tanh_gain:.1f} * theta_d) "
        f"+ {result.gravity_gain:.6f} * sin(theta {'+' if result.gravity_phase >= 0.0 else '-'} "
        f"{abs(result.gravity_phase):.6f}) = tau_ff"
    )
    print(f"  J: {result.inertia:.6f}")
    print(f"  B: {result.viscous_damping:.6f}")
    print(f"  Fc: {result.coulomb_friction:.6f}")
    print(f"  G: {result.gravity_gain:.6f}")
    print(
        f"  phi: {result.gravity_phase:.6f} rad ({result.gravity_phase_deg:.2f} deg)"
    )
    print(
        "  gravity zero crossing (-phi): "
        f"{result.gravity_zero_crossing:.6f} rad "
        f"({result.gravity_zero_crossing_deg:.2f} deg)"
    )
    if result.fit_mode == "ode-angle":
        print(
            f"  shooting window: {result.shooting_window:.3f} s "
            f"({result.segment_count} segments)"
        )
        print(f"  torque bias: {result.torque_bias:.6f}")
        print(
            "  optimizer: "
            f"success={result.optimizer_success}, "
            f"status={result.optimizer_status}, "
            f"nfev={result.optimizer_nfev}"
        )
        print(f"  Angle RMSE: {result.rmse:.6f}")
        print(f"  Angle MAE: {result.mae:.6f}")
    else:
        print(f"  RMSE: {result.rmse:.6f}")
        print(f"  MAE: {result.mae:.6f}")
    print(f"  R^2: {result.r2:.6f}")
    if math.isfinite(result.design_condition):
        print(f"  design condition: {result.design_condition:.3f}")


def print_warnings(result: GrayboxFitResult) -> None:
    warnings: list[str] = []
    if result.inertia <= 0.0:
        warnings.append("identified inertia J is non-positive")
    if result.viscous_damping < 0.0:
        warnings.append("identified viscous damping B is negative")
    if result.coulomb_friction < 0.0:
        warnings.append("identified Coulomb friction Fc is negative")
    if math.degrees(result.angle_span) < 90.0:
        warnings.append("angle coverage is below 90 deg, so gravity phase may be weakly observable")
    if result.fit_mode == "linear":
        if result.design_condition > 100.0:
            warnings.append("regression matrix is ill-conditioned; parameters may be sensitive to noise")
        if math.isfinite(result.r2) and result.r2 < 0.7:
            warnings.append("R^2 is modest; derivative noise or model mismatch may still dominate")
    else:
        if not result.optimizer_success:
            warnings.append(f"optimizer did not report success: {result.optimizer_message}")
        if abs(result.torque_bias) > max(0.1, 0.2 * result.gravity_gain):
            warnings.append("identified torque bias is large; input signal may have a non-zero offset")
        if math.isfinite(result.r2) and result.r2 < 0.7:
            warnings.append("angle-only fit is modest; excitation or gravity observability may still be weak")

    if not warnings:
        return

    print("Warnings:")
    for item in warnings:
        print(f"  - {item}")


def main() -> int:
    args = parse_args()
    validate_args(args)

    rows, fieldnames = read_rows(args.csv_path)
    if not rows:
        raise ValueError("CSV file is empty.")

    target = args.target if args.target is not None else infer_target(fieldnames)
    columns = require_columns(fieldnames, target)

    elapsed = np.array([float(row[columns["elapsed_s"]]) for row in rows], dtype=float)
    angle = np.array([float(row[columns["angle"]]) for row in rows], dtype=float)
    velocity = np.array([float(row[columns["velocity"]]) for row in rows], dtype=float)
    control_torque = np.array([float(row[columns["control_torque"]]) for row in rows], dtype=float)
    measured_torque = np.array([float(row[columns["torque"]]) for row in rows], dtype=float)

    if elapsed.size <= args.window_length:
        raise ValueError("Too few samples for the requested Savitzky-Golay window length.")

    dt = float(np.median(np.diff(elapsed)))
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("Failed to determine a valid time step from elapsed_s.")

    half_window = args.window_length // 2
    velocity_smooth = savitzky_golay(
        velocity, dt, args.window_length, args.poly_order, deriv=0
    )

    center_slice = slice(half_window, elapsed.size - half_window)
    elapsed_valid = elapsed[center_slice]
    angle_valid = np.unwrap(angle[center_slice])
    control_torque_valid = control_torque[center_slice]
    measured_torque_valid = measured_torque[center_slice]

    mask = np.ones_like(elapsed_valid, dtype=bool)
    if args.trim_start > 0.0:
        mask &= elapsed_valid >= args.trim_start
    if args.trim_end > 0.0:
        mask &= elapsed_valid <= elapsed_valid[-1] - args.trim_end

    if int(np.count_nonzero(mask)) < 10:
        raise ValueError("Too few samples remain after trimming.")

    elapsed_fit = elapsed_valid[mask]
    angle_fit = angle_valid[mask]
    velocity_fit = velocity_smooth[mask]
    control_torque_fit = control_torque_valid[mask]
    measured_torque_fit = measured_torque_valid[mask]

    primary_signal = (
        control_torque_fit if args.input_signal == "control_torque" else measured_torque_fit
    )
    primary_signal_name = columns[args.input_signal]
    reference_signal = (
        measured_torque_fit if args.input_signal == "control_torque" else control_torque_fit
    )
    reference_signal_name = (
        columns["torque"] if args.input_signal == "control_torque" else columns["control_torque"]
    )

    if args.fit_mode == "linear":
        acceleration = savitzky_golay(
            velocity, dt, args.window_length, args.poly_order, deriv=1
        )
        acceleration_fit = acceleration[mask]
        fit_function = fit_linear_graybox
        fit_kwargs: dict[str, float] = {}
        if args.fixed_gravity_gain is not None:
            fit_function = fit_fixed_gravity_graybox
            fit_kwargs = {
                "gravity_gain": args.fixed_gravity_gain,
                "gravity_phase": args.fixed_gravity_phase,
            }

        primary_fit = fit_function(
            angle_fit,
            velocity_fit,
            acceleration_fit,
            primary_signal,
            primary_signal_name,
            dt,
            args.window_length,
            args.poly_order,
            args.tanh_gain,
            **fit_kwargs,
        )
        reference_fit = fit_function(
            angle_fit,
            velocity_fit,
            acceleration_fit,
            reference_signal,
            reference_signal_name,
            dt,
            args.window_length,
            args.poly_order,
            args.tanh_gain,
            **fit_kwargs,
        )
    else:
        primary_fit = fit_ode_angle_graybox(
            elapsed_fit,
            angle_fit,
            velocity_fit,
            primary_signal,
            primary_signal_name,
            dt,
            args.window_length,
            args.poly_order,
            args.tanh_gain,
            args.shooting_window,
            args.ode_max_nfev,
            args.fixed_gravity_gain,
            args.fixed_gravity_phase,
        )
        reference_fit = fit_ode_angle_graybox(
            elapsed_fit,
            angle_fit,
            velocity_fit,
            reference_signal,
            reference_signal_name,
            dt,
            args.window_length,
            args.poly_order,
            args.tanh_gain,
            args.shooting_window,
            args.ode_max_nfev,
            args.fixed_gravity_gain,
            args.fixed_gravity_phase,
        )

    print(f"CSV: {args.csv_path}")
    print(f"Target: {target}")
    print(f"Rows loaded: {len(rows)}")
    print(f"Rows used: {primary_fit.sample_count}")
    print(f"Median dt: {dt:.6f} s")
    print(f"Fit mode: {args.fit_mode}")
    if args.fit_mode == "linear":
        print(
            f"Savitzky-Golay: window_length={args.window_length}, poly_order={args.poly_order}, "
            f"source={columns['velocity']}"
        )
    else:
        print(
            "Velocity smoothing: "
            f"window_length={args.window_length}, poly_order={args.poly_order}, "
            f"source={columns['velocity']}"
        )
        print(
            "ODE multiple shooting: "
            f"window={args.shooting_window:.3f} s, max_nfev={args.ode_max_nfev}"
        )
    if args.fixed_gravity_gain is not None:
        print(
            "Fixed gravity: "
            f"G={args.fixed_gravity_gain:.6f}, phi={args.fixed_gravity_phase:.6f} rad"
        )
    if args.trim_start > 0.0 or args.trim_end > 0.0:
        print(f"Trim: start={args.trim_start:.3f} s, end={args.trim_end:.3f} s")
    print_fit(primary_fit, "Primary fit")
    print_fit(reference_fit, "Reference fit")
    print_warnings(primary_fit)
    print("Suggested params:")
    print(f"  inertia: {primary_fit.inertia:.6f}")
    print(f"  viscous_damping: {primary_fit.viscous_damping:.6f}")
    print(f"  coulomb_friction: {primary_fit.coulomb_friction:.6f}")
    print(f"  gravity_gain: {primary_fit.gravity_gain:.6f}")
    print(f"  gravity_phase: {primary_fit.gravity_phase:.6f}")
    if args.fit_mode == "ode-angle":
        print("Nuisance params:")
        print(f"  torque_bias: {primary_fit.torque_bias:.6f}")

    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "csv_path": str(args.csv_path),
            "target": target,
            "input_signal": args.input_signal,
            "fit_mode": args.fit_mode,
            "rows_total": len(rows),
            "rows_used": primary_fit.sample_count,
            "dt": dt,
            "window_length": args.window_length,
            "poly_order": args.poly_order,
            "tanh_gain": args.tanh_gain,
            "trim_start": args.trim_start,
            "trim_end": args.trim_end,
            "shooting_window": args.shooting_window,
            "ode_max_nfev": args.ode_max_nfev,
            "fixed_gravity_gain": args.fixed_gravity_gain,
            "fixed_gravity_phase": args.fixed_gravity_phase,
            "primary_fit": asdict(primary_fit),
            "reference_fit": asdict(reference_fit),
        }
        args.json_output.write_text(json.dumps(payload, indent=2) + "\n")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
