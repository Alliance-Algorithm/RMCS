#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a first-order friction-wheel speed plant from SweptFrequencyController CSV logs "
            "and emit gains compatible with rmcs_core::controller::pid::PidController."
        )
    )
    parser.add_argument("csv_paths", nargs="+", type=Path, help="One or more sweep CSV files")
    parser.add_argument(
        "--input-signal",
        choices=("torque", "control_torque"),
        default="torque",
        help="Plant input column. Prefer measured torque when available.",
    )
    parser.add_argument(
        "--window-length",
        type=int,
        default=31,
        help="Odd Savitzky-Golay window length used on velocity before differentiation.",
    )
    parser.add_argument(
        "--poly-order",
        type=int,
        default=3,
        help="Savitzky-Golay polynomial order.",
    )
    parser.add_argument(
        "--trim-start",
        type=float,
        default=1.5,
        help="Discard the first N seconds to remove enable transients.",
    )
    parser.add_argument(
        "--trim-end",
        type=float,
        default=0.5,
        help="Discard the last N seconds to avoid sweep shutoff transients.",
    )
    parser.add_argument(
        "--fc-hz",
        type=float,
        default=None,
        help="Target closed-loop crossover for all wheels. Default: front=22Hz, back=18Hz.",
    )
    parser.add_argument(
        "--td-ratio",
        type=float,
        default=0.0,
        help="Derivative time as a fraction of the identified mechanical time constant. Keep 0 unless velocity is very clean.",
    )
    parser.add_argument(
        "--split-ratio",
        type=float,
        default=0.08,
        help="Integral action only inside +/- split_ratio * steady_speed.",
    )
    parser.add_argument(
        "--i-output-ratio",
        type=float,
        default=0.35,
        help="Clamp integral contribution to this fraction of the identified AC torque span.",
    )
    parser.add_argument(
        "--min-r2",
        type=float,
        default=0.80,
        help="Warn when the acceleration fit R^2 drops below this value.",
    )
    parser.add_argument(
        "--group-average",
        action="store_true",
        help="Also emit averaged gain-only blocks for front wheels and back wheels.",
    )
    return parser.parse_args()


@dataclass
class FitResult:
    csv_path: Path
    target: str
    controller_name: str
    dt: float
    samples: int
    steady_speed: float
    steady_torque: float
    inertia: float
    damping: float
    tau_mech: float
    fit_r2: float
    fc_hz: float
    kp: float
    ki: float
    kd: float
    integral_limit: float
    split_limit: float
    output_limit: float
    torque_span: float


def savitzky_golay(
    y: np.ndarray, dt: float, window_length: int, poly_order: int, deriv: int
) -> np.ndarray:
    half_window = window_length // 2
    offsets = np.arange(-half_window, half_window + 1, dtype=float) * dt
    vandermonde = np.vstack([offsets**order for order in range(poly_order + 1)]).T
    coefficients = math.factorial(deriv) * np.linalg.pinv(vandermonde)[deriv]
    windows = np.lib.stride_tricks.sliding_window_view(y, window_length)
    return windows @ coefficients


def infer_target(fieldnames: list[str]) -> str:
    candidates = {
        name[: -len("/velocity")]
        for name in fieldnames
        if name.endswith("/velocity") and name != "velocity"
    }
    if len(candidates) != 1:
        raise ValueError("failed to infer a unique target from CSV headers")
    return next(iter(candidates))


def require_columns(fieldnames: list[str], target: str) -> dict[str, str]:
    columns = {
        "elapsed_s": "elapsed_s",
        "control_torque": f"{target}/control_torque",
        "torque": f"{target}/torque",
        "velocity": f"{target}/velocity",
    }
    missing = [value for value in columns.values() if value not in fieldnames]
    if missing:
        raise ValueError(f"missing required columns: {', '.join(missing)}")
    return columns


def default_fc_hz(target: str) -> float:
    if "front" in target:
        return 22.0
    if "back" in target:
        return 18.0
    return 20.0


def controller_name_for(target: str) -> str:
    return target.rsplit("/", 1)[-1] + "_velocity_pid_controller"


def read_csv(path: Path, input_signal: str) -> tuple[str, np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        if not rows:
            raise ValueError(f"CSV has no data rows: {path}")
        target = infer_target(reader.fieldnames)
        columns = require_columns(reader.fieldnames, target)

    elapsed = np.array([float(row[columns["elapsed_s"]]) for row in rows], dtype=float)
    velocity = np.array([float(row[columns["velocity"]]) for row in rows], dtype=float)
    torque = np.array([float(row[columns[input_signal]]) for row in rows], dtype=float)
    return target, elapsed, velocity, torque


def fit_first_order_model(
    elapsed: np.ndarray, velocity: np.ndarray, torque: np.ndarray, args: argparse.Namespace
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    if args.window_length < 5 or args.window_length % 2 == 0:
        raise ValueError("window-length must be an odd integer >= 5")
    if args.poly_order < 2 or args.poly_order >= args.window_length:
        raise ValueError("poly-order must be >= 2 and smaller than window-length")
    if elapsed.size <= args.window_length:
        raise ValueError("too few samples for the requested Savitzky-Golay window")

    dt = float(np.median(np.diff(elapsed)))
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("failed to determine a valid dt from elapsed_s")

    velocity_smooth = savitzky_golay(velocity, dt, args.window_length, args.poly_order, deriv=0)
    acceleration_smooth = savitzky_golay(
        velocity, dt, args.window_length, args.poly_order, deriv=1
    )

    half_window = args.window_length // 2
    center = slice(half_window, elapsed.size - half_window)
    elapsed_fit = elapsed[center]
    torque_fit = torque[center]

    mask = np.isfinite(elapsed_fit)
    mask &= np.isfinite(velocity_smooth)
    mask &= np.isfinite(acceleration_smooth)
    mask &= np.isfinite(torque_fit)
    if args.trim_start > 0.0:
        mask &= elapsed_fit >= args.trim_start
    if args.trim_end > 0.0:
        mask &= elapsed_fit <= elapsed_fit[-1] - args.trim_end
    if int(np.count_nonzero(mask)) < 20:
        raise ValueError("too few usable samples remain after trimming")

    return velocity_smooth[mask], acceleration_smooth[mask], torque_fit[mask], dt


def normalize_sign(
    velocity: np.ndarray, acceleration: np.ndarray, torque: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if float(np.median(velocity)) >= 0.0:
        return velocity, acceleration, torque
    return -velocity, -acceleration, -torque


def fit_one(path: Path, args: argparse.Namespace) -> FitResult:
    target, elapsed, velocity, torque = read_csv(path, args.input_signal)
    velocity_fit, acceleration_fit, torque_fit, dt = fit_first_order_model(
        elapsed, velocity, torque, args
    )
    velocity_fit, acceleration_fit, torque_fit = normalize_sign(
        velocity_fit, acceleration_fit, torque_fit
    )

    steady_speed = float(np.median(velocity_fit))
    steady_torque = float(np.median(torque_fit))
    regressors = np.column_stack([velocity_fit - steady_speed, torque_fit - steady_torque])
    coefficients, _, _, _ = np.linalg.lstsq(regressors, acceleration_fit, rcond=None)
    a_vel, b_tau = coefficients.tolist()
    if not math.isfinite(a_vel) or not math.isfinite(b_tau):
        raise ValueError("least-squares result is not finite")
    if b_tau <= 0.0:
        raise ValueError(f"identified torque gain <= 0 (b={b_tau:.6e})")
    if a_vel >= 0.0:
        raise ValueError(f"identified damping slope >= 0 (a={a_vel:.6e})")

    prediction = regressors @ coefficients
    residual = acceleration_fit - prediction
    centered = acceleration_fit - float(np.mean(acceleration_fit))
    ss_res = float(residual @ residual)
    ss_tot = float(centered @ centered)
    fit_r2 = float("nan") if ss_tot <= 0.0 else 1.0 - ss_res / ss_tot

    inertia = 1.0 / b_tau
    damping = -a_vel / b_tau
    tau_mech = inertia / damping
    fc_hz = args.fc_hz if args.fc_hz is not None else default_fc_hz(target)
    wc = 2.0 * math.pi * fc_hz

    kp = wc * inertia
    ki = wc * damping * dt
    td = max(0.0, args.td_ratio) * tau_mech
    kd = 0.0 if td == 0.0 else (kp * td) / dt

    torque_span = float(np.percentile(np.abs(torque_fit - steady_torque), 95))
    max_i_output = max(0.05, args.i_output_ratio * torque_span)
    integral_limit = 0.0 if ki <= 0.0 else max_i_output / ki
    split_limit = max(5.0, args.split_ratio * abs(steady_speed))
    output_limit = max(
        0.1,
        abs(steady_torque) + 1.2 * torque_span,
        1.3 * float(np.percentile(np.abs(torque_fit), 99)),
    )

    return FitResult(
        csv_path=path,
        target=target,
        controller_name=controller_name_for(target),
        dt=dt,
        samples=int(velocity_fit.size),
        steady_speed=steady_speed,
        steady_torque=steady_torque,
        inertia=inertia,
        damping=damping,
        tau_mech=tau_mech,
        fit_r2=fit_r2,
        fc_hz=fc_hz,
        kp=kp,
        ki=ki,
        kd=kd,
        integral_limit=integral_limit,
        split_limit=split_limit,
        output_limit=output_limit,
        torque_span=torque_span,
    )


def format_yaml_block(result: FitResult, controller_name: str | None = None) -> str:
    controller_name = result.controller_name if controller_name is None else controller_name
    return "\n".join(
        [
            f"{controller_name}:",
            "  ros__parameters:",
            f"    measurement: {result.target}/velocity",
            f"    setpoint: {result.target}/control_velocity",
            f"    control: {result.target}/control_torque",
            f"    kp: {result.kp:.9f}",
            f"    ki: {result.ki:.9f}",
            f"    kd: {result.kd:.9f}",
            f"    integral_min: {-result.integral_limit:.6f}",
            f"    integral_max: {result.integral_limit:.6f}",
            f"    integral_split_min: {-result.split_limit:.6f}",
            f"    integral_split_max: {result.split_limit:.6f}",
            f"    output_min: {-result.output_limit:.6f}",
            f"    output_max: {result.output_limit:.6f}",
        ]
    )


def format_gain_only_block(name: str, result: FitResult) -> str:
    return "\n".join(
        [
            f"{name}:",
            "  ros__parameters:",
            f"    kp: {result.kp:.9f}",
            f"    ki: {result.ki:.9f}",
            f"    kd: {result.kd:.9f}",
            f"    integral_min: {-result.integral_limit:.6f}",
            f"    integral_max: {result.integral_limit:.6f}",
            f"    integral_split_min: {-result.split_limit:.6f}",
            f"    integral_split_max: {result.split_limit:.6f}",
            f"    output_min: {-result.output_limit:.6f}",
            f"    output_max: {result.output_limit:.6f}",
        ]
    )


def print_result(result: FitResult, min_r2: float) -> None:
    print(f"CSV: {result.csv_path}")
    print(f"  target: {result.target}")
    print(f"  samples: {result.samples}")
    print(f"  dt: {result.dt:.6f} s")
    print(f"  steady_speed: {result.steady_speed:.6f} rad/s")
    print(f"  steady_torque: {result.steady_torque:.6f} N*m")
    print(f"  J: {result.inertia:.9f}")
    print(f"  B: {result.damping:.9f}")
    print(f"  tau_mech: {result.tau_mech:.6f} s")
    print(f"  fit R^2: {result.fit_r2:.6f}")
    print(f"  chosen fc: {result.fc_hz:.2f} Hz")
    if math.isfinite(result.fit_r2) and result.fit_r2 < min_r2:
        print(
            "  warning: fit quality is weak; reduce sweep amplitude or clean the data before using this PID"
        )
    print(format_yaml_block(result))


def make_average_result(group_name: str, members: list[FitResult]) -> FitResult:
    template = members[0]
    return FitResult(
        csv_path=template.csv_path,
        target=template.target,
        controller_name=group_name,
        dt=statistics.mean(item.dt for item in members),
        samples=sum(item.samples for item in members),
        steady_speed=statistics.mean(item.steady_speed for item in members),
        steady_torque=statistics.mean(item.steady_torque for item in members),
        inertia=statistics.mean(item.inertia for item in members),
        damping=statistics.mean(item.damping for item in members),
        tau_mech=statistics.mean(item.tau_mech for item in members),
        fit_r2=statistics.mean(item.fit_r2 for item in members),
        fc_hz=statistics.mean(item.fc_hz for item in members),
        kp=statistics.mean(item.kp for item in members),
        ki=statistics.mean(item.ki for item in members),
        kd=statistics.mean(item.kd for item in members),
        integral_limit=statistics.mean(item.integral_limit for item in members),
        split_limit=statistics.mean(item.split_limit for item in members),
        output_limit=statistics.mean(item.output_limit for item in members),
        torque_span=statistics.mean(item.torque_span for item in members),
    )


def main() -> int:
    args = parse_args()
    results: list[FitResult] = []
    for path in args.csv_paths:
        results.append(fit_one(path, args))

    for index, result in enumerate(results):
        if index:
            print()
        print_result(result, args.min_r2)

    if args.group_average:
        front = [item for item in results if "front" in item.target]
        back = [item for item in results if "back" in item.target]
        if front:
            print()
            print("Front average:")
            average = make_average_result("front_friction_velocity_pid_average", front)
            print(format_gain_only_block("front_friction_velocity_pid_average", average))
        if back:
            print()
            print("Back average:")
            average = make_average_result("back_friction_velocity_pid_average", back)
            print(format_gain_only_block("back_friction_velocity_pid_average", average))

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
