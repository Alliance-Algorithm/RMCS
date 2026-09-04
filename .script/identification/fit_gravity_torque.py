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


def wrap_to_pi(angle: float) -> float:
    wrapped = math.remainder(angle, 2.0 * math.pi)
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def circular_mean(angles: list[float]) -> float:
    sin_sum = sum(math.sin(angle) for angle in angles)
    cos_sum = sum(math.cos(angle) for angle in angles)
    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return wrap_to_pi(angles[0])
    return wrap_to_pi(math.atan2(sin_sum, cos_sum))


@dataclass
class FitResult:
    signal_name: str
    sample_count: int
    angle_min: float
    angle_max: float
    angle_span: float
    gain: float
    phase: float
    phase_deg: float
    zero_crossing_angle: float
    zero_crossing_angle_deg: float
    peak_angle: float
    peak_angle_deg: float
    rmse: float
    mae: float
    r2: float
    design_condition: float


@dataclass
class PairingSummary:
    available_point_count: int
    paired_point_count: int
    dropped_point_count: int
    mean_control_friction: float
    mean_torque_friction: float


@dataclass
class AveragedSample:
    point_index: int
    angle: float
    setpoint: float
    control_torque: float
    torque: float
    control_friction: float
    torque_friction: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit the gray-box gravity torque model u_ff = G * sin(theta + phi) "
            "from static torque test CSV data."
        )
    )
    parser.add_argument("csv_path", type=Path, help="Path to static_torque_test_controller CSV")
    parser.add_argument(
        "--target",
        type=str,
        default=None,
        help="Target prefix such as /gimbal/pitch. If omitted, infer from CSV headers.",
    )
    parser.add_argument(
        "--signal",
        choices=("control_torque", "torque"),
        default="torque",
        help="Primary signal to fit. Defaults to torque.",
    )
    parser.add_argument(
        "--velocity-threshold",
        type=float,
        default=0.1,
        help=(
            "Discard rows with abs(velocity) larger than this threshold. "
            "Use a negative value to disable filtering."
        ),
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
        "update_count": "update_count",
        "elapsed_s": "elapsed_s",
        "control_torque": f"{target}/control_torque",
        "torque": f"{target}/torque",
        "velocity": f"{target}/velocity",
        "angle": f"{target}/angle",
        "point_index": "point_index",
        "approach_direction": "approach_direction",
        "setpoint": "setpoint",
    }

    missing = [
        column
        for key, column in columns.items()
        if key in {"update_count", "elapsed_s", "control_torque", "torque", "velocity", "angle"}
        and column not in fieldnames
    ]
    if missing:
        joined = ", ".join(missing)
        raise ValueError(f"Missing required CSV columns: {joined}")
    return columns


def select_static_rows(
    rows: list[dict[str, str]], columns: dict[str, str], velocity_threshold: float
) -> list[dict[str, str]]:
    if velocity_threshold < 0.0:
        return rows

    filtered_rows = []
    for row in rows:
        velocity = float(row[columns["velocity"]])
        if abs(velocity) <= velocity_threshold:
            filtered_rows.append(row)
    return filtered_rows


def has_bidirectional_columns(fieldnames: list[str], columns: dict[str, str]) -> bool:
    return all(
        columns[key] in fieldnames for key in ("point_index", "approach_direction", "setpoint")
    )


def fit_sine_model(samples: list[tuple[float, float]], signal_name: str) -> FitResult:
    if len(samples) < 3:
        raise ValueError("Need at least 3 samples to fit the sine model.")

    s_s = 0.0
    s_c = 0.0
    c_c = 0.0
    s_y = 0.0
    c_y = 0.0
    angles: list[float] = []
    values: list[float] = []

    for angle, value in samples:
        wrapped_angle = wrap_to_pi(angle)
        sin_theta = math.sin(wrapped_angle)
        cos_theta = math.cos(wrapped_angle)

        s_s += sin_theta * sin_theta
        s_c += sin_theta * cos_theta
        c_c += cos_theta * cos_theta
        s_y += sin_theta * value
        c_y += cos_theta * value

        angles.append(wrapped_angle)
        values.append(value)

    determinant = s_s * c_c - s_c * s_c
    if abs(determinant) < 1e-12:
        raise ValueError(
            "The regression design matrix is singular. The angle coverage is likely too narrow."
        )

    sin_gain = (s_y * c_c - c_y * s_c) / determinant
    cos_gain = (s_s * c_y - s_c * s_y) / determinant

    gain = math.hypot(sin_gain, cos_gain)
    phase = math.atan2(cos_gain, sin_gain)

    predictions = [gain * math.sin(angle + phase) for angle in angles]
    residuals = [value - prediction for value, prediction in zip(values, predictions)]

    mean_value = sum(values) / len(values)
    ss_res = sum(residual * residual for residual in residuals)
    ss_tot = sum((value - mean_value) ** 2 for value in values)
    rmse = math.sqrt(ss_res / len(values))
    mae = sum(abs(residual) for residual in residuals) / len(values)
    r2 = float("nan") if ss_tot <= 0.0 else 1.0 - ss_res / ss_tot

    trace = s_s + c_c
    disc = math.sqrt(max(0.0, (s_s - c_c) ** 2 + 4.0 * s_c * s_c))
    lambda_max = 0.5 * (trace + disc)
    lambda_min = 0.5 * (trace - disc)
    condition = float("inf") if lambda_min <= 0.0 else lambda_max / lambda_min

    angle_min = min(angles)
    angle_max = max(angles)

    zero_crossing_angle = wrap_to_pi(-phase)
    peak_angle = wrap_to_pi(math.pi * 0.5 - phase)

    return FitResult(
        signal_name=signal_name,
        sample_count=len(values),
        angle_min=angle_min,
        angle_max=angle_max,
        angle_span=angle_max - angle_min,
        gain=gain,
        phase=phase,
        phase_deg=math.degrees(phase),
        zero_crossing_angle=zero_crossing_angle,
        zero_crossing_angle_deg=math.degrees(zero_crossing_angle),
        peak_angle=peak_angle,
        peak_angle_deg=math.degrees(peak_angle),
        rmse=rmse,
        mae=mae,
        r2=r2,
        design_condition=condition,
    )


def pair_bidirectional_rows(
    rows: list[dict[str, str]], columns: dict[str, str]
) -> tuple[list[AveragedSample], PairingSummary]:
    grouped_rows: dict[int, dict[int, dict[str, str]]] = {}

    for row in rows:
        approach_direction = int(row[columns["approach_direction"]])
        if approach_direction not in (-1, +1):
            continue

        point_index = int(row[columns["point_index"]])
        grouped_rows.setdefault(point_index, {})[approach_direction] = row

    averaged_samples: list[AveragedSample] = []
    control_friction_values: list[float] = []
    torque_friction_values: list[float] = []

    for point_index in sorted(grouped_rows):
        paired_rows = grouped_rows[point_index]
        if -1 not in paired_rows or +1 not in paired_rows:
            continue

        negative_row = paired_rows[-1]
        positive_row = paired_rows[+1]

        angle = circular_mean(
            [
                float(negative_row[columns["angle"]]),
                float(positive_row[columns["angle"]]),
            ]
        )
        setpoint = circular_mean(
            [
                float(negative_row[columns["setpoint"]]),
                float(positive_row[columns["setpoint"]]),
            ]
        )

        control_negative = float(negative_row[columns["control_torque"]])
        control_positive = float(positive_row[columns["control_torque"]])
        torque_negative = float(negative_row[columns["torque"]])
        torque_positive = float(positive_row[columns["torque"]])

        control_friction = 0.5 * (control_positive - control_negative)
        torque_friction = 0.5 * (torque_positive - torque_negative)

        averaged_samples.append(
            AveragedSample(
                point_index=point_index,
                angle=angle,
                setpoint=setpoint,
                control_torque=0.5 * (control_positive + control_negative),
                torque=0.5 * (torque_positive + torque_negative),
                control_friction=control_friction,
                torque_friction=torque_friction,
            )
        )
        control_friction_values.append(abs(control_friction))
        torque_friction_values.append(abs(torque_friction))

    available_point_count = len(grouped_rows)
    paired_point_count = len(averaged_samples)
    dropped_point_count = available_point_count - paired_point_count

    summary = PairingSummary(
        available_point_count=available_point_count,
        paired_point_count=paired_point_count,
        dropped_point_count=dropped_point_count,
        mean_control_friction=(
            sum(control_friction_values) / len(control_friction_values)
            if control_friction_values
            else float("nan")
        ),
        mean_torque_friction=(
            sum(torque_friction_values) / len(torque_friction_values)
            if torque_friction_values
            else float("nan")
        ),
    )
    return averaged_samples, summary


def print_fit(result: FitResult, label: str) -> None:
    print(f"{label}:")
    print(f"  signal: {result.signal_name}")
    print(f"  samples: {result.sample_count}")
    print(
        "  angle span: "
        f"[{result.angle_min:.6f}, {result.angle_max:.6f}] rad "
        f"({math.degrees(result.angle_span):.2f} deg)"
    )
    print(f"  G: {result.gain:.6f}")
    print(f"  phi: {result.phase:.6f} rad ({result.phase_deg:.2f} deg)")
    print(
        "  model: "
        f"u_ff = {result.gain:.6f} * sin(theta {'+' if result.phase >= 0.0 else '-'} "
        f"{abs(result.phase):.6f})"
    )
    print(
        "  zero crossing angle (-phi): "
        f"{result.zero_crossing_angle:.6f} rad ({result.zero_crossing_angle_deg:.2f} deg)"
    )
    print(
        "  peak angle (pi/2 - phi): "
        f"{result.peak_angle:.6f} rad ({result.peak_angle_deg:.2f} deg)"
    )
    print(f"  RMSE: {result.rmse:.6f}")
    print(f"  MAE: {result.mae:.6f}")
    print(f"  R^2: {result.r2:.6f}")
    print(f"  design condition: {result.design_condition:.3f}")


def print_warnings(result: FitResult) -> None:
    warnings: list[str] = []
    if math.degrees(result.angle_span) < 90.0:
        warnings.append(
            "angle coverage is below 90 deg, so phase identification may be weak or biased"
        )
    if result.design_condition > 10.0:
        warnings.append(
            "sin/cos regressors are ill-conditioned for this dataset, so phase is sensitive to noise"
        )
    if math.isfinite(result.r2) and result.r2 < 0.8:
        warnings.append(
            "R^2 is low; gravity may be mixed with friction, backlash, or insufficient settling"
        )

    if not warnings:
        return

    print("Warnings:")
    for item in warnings:
        print(f"  - {item}")


def raw_samples_from_rows(
    rows: list[dict[str, str]], columns: dict[str, str], signal_key: str
) -> list[tuple[float, float]]:
    return [
        (float(row[columns["angle"]]), float(row[columns[signal_key]]))
        for row in rows
    ]


def averaged_samples_to_tuples(
    rows: list[AveragedSample], signal_key: str
) -> list[tuple[float, float]]:
    return [(row.angle, getattr(row, signal_key)) for row in rows]


def main() -> int:
    args = parse_args()

    rows, fieldnames = read_rows(args.csv_path)
    target = args.target if args.target is not None else infer_target(fieldnames)
    columns = require_columns(fieldnames, target)

    static_rows = select_static_rows(rows, columns, args.velocity_threshold)
    if not static_rows:
        raise ValueError("No rows remain after velocity filtering.")

    payload: dict[str, object] = {
        "csv_path": str(args.csv_path),
        "target": target,
        "velocity_threshold": args.velocity_threshold,
        "rows_total": len(rows),
        "rows_used": len(static_rows),
    }

    print(f"CSV: {args.csv_path}")
    print(f"Target: {target}")
    if args.velocity_threshold >= 0.0:
        print(f"Velocity filter: abs({columns['velocity']}) <= {args.velocity_threshold}")
    else:
        print("Velocity filter: disabled")
    print(f"Rows used: {len(static_rows)} / {len(rows)}")

    primary_signal_key = args.signal
    reference_signal_key = "torque" if primary_signal_key == "control_torque" else "control_torque"

    if has_bidirectional_columns(fieldnames, columns):
        averaged_samples, pairing_summary = pair_bidirectional_rows(static_rows, columns)
        if not averaged_samples:
            raise ValueError(
                "No bidirectional pairs were found. The CSV may be incomplete or the test did not reach both directions."
            )

        primary_fit = fit_sine_model(
            averaged_samples_to_tuples(averaged_samples, primary_signal_key),
            f"paired_avg:{columns[primary_signal_key]}",
        )
        reference_fit = fit_sine_model(
            averaged_samples_to_tuples(averaged_samples, reference_signal_key),
            f"paired_avg:{columns[reference_signal_key]}",
        )

        print("Mode: bidirectional pairing with friction cancellation")
        print(
            "Pairing: "
            f"{pairing_summary.paired_point_count} paired / {pairing_summary.available_point_count} available points "
            f"({pairing_summary.dropped_point_count} dropped)"
        )
        print(
            "Mean half-difference friction: "
            f"control={pairing_summary.mean_control_friction:.6f}, "
            f"torque={pairing_summary.mean_torque_friction:.6f}"
        )

        payload["pairing"] = asdict(pairing_summary)
        payload["mode"] = "bidirectional"
    else:
        primary_fit = fit_sine_model(
            raw_samples_from_rows(static_rows, columns, primary_signal_key),
            columns[primary_signal_key],
        )
        reference_fit = fit_sine_model(
            raw_samples_from_rows(static_rows, columns, reference_signal_key),
            columns[reference_signal_key],
        )
        print("Mode: legacy single-direction fit without friction cancellation")
        payload["mode"] = "legacy_single_direction"

    print_fit(primary_fit, "Primary fit")
    print_fit(reference_fit, "Reference fit")
    print_warnings(primary_fit)
    print("Suggested params:")
    print(f"  gravity_gain: {primary_fit.gain:.6f}")
    print(f"  gravity_phase: {primary_fit.phase:.6f}")

    payload["primary_fit"] = asdict(primary_fit)
    payload["reference_fit"] = asdict(reference_fit)

    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(payload, indent=2) + "\n")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
