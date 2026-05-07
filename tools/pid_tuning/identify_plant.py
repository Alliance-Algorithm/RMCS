#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np


REQUIRED_COLUMNS = {
    "timestamp_us",
    "wheel_id",
    "run_id",
    "sample_idx",
    "setpoint_velocity",
    "measured_velocity",
    "control_torque",
}


@dataclass(frozen=True)
class Sample:
    timestamp_us: int
    wheel_id: str
    run_id: int
    sample_idx: int
    setpoint_velocity: float
    measured_velocity: float
    control_torque: float


@dataclass(frozen=True)
class Segment:
    wheel_id: str
    run_id: int
    timestamp_us: np.ndarray
    sample_idx: np.ndarray
    setpoint_velocity: np.ndarray
    measured_velocity: np.ndarray
    control_torque: np.ndarray
    dropped_rows: int
    inferred_dt_s: float
    sample_jitter_ratio: float
    missing_samples: int


@dataclass(frozen=True)
class CandidateResult:
    delay_samples: int
    a: float
    b: float
    c: float
    rollout_rmse: float
    one_step_rmse: float
    mae: float
    fit_percent: float
    physically_plausible: bool
    predicted_velocity: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Estimate a first-order-plus-dead-time plant model for friction wheel velocity loops "
            "from recorder CSV."
        )
    )
    parser.add_argument("input_csv", help="CSV produced by FrictionWheelPidRecorder")
    parser.add_argument(
        "--output-dir",
        help="Directory for summary/model files. Default: /tmp/friction_pid_identify/<timestamp>",
    )
    parser.add_argument("--wheel-id", help="Only process one wheel_id")
    parser.add_argument("--run-id", type=int, help="Only process one run_id")
    parser.add_argument(
        "--sample-time-ms",
        type=float,
        default=1.0,
        help="Fallback sample time in milliseconds when timestamps are missing or invalid",
    )
    parser.add_argument(
        "--max-delay-ms",
        type=float,
        default=25.0,
        help="Maximum pure delay to scan in milliseconds",
    )
    parser.add_argument(
        "--max-delay-samples",
        type=int,
        help="Maximum pure delay to scan in samples. Overrides --max-delay-ms",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=80,
        help="Minimum samples required for one segment",
    )
    parser.add_argument(
        "--min-speed-span",
        type=float,
        default=30.0,
        help="Minimum measured velocity span required to consider a segment sufficiently excited",
    )
    parser.add_argument(
        "--min-torque-span",
        type=float,
        default=0.05,
        help="Minimum control torque span required to consider a segment sufficiently excited",
    )
    parser.add_argument(
        "--allow-unphysical",
        action="store_true",
        help="Allow selecting candidates outside the expected 0<a<1, b>0 range if needed",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Only print output paths and fatal errors",
    )
    return parser.parse_args()


def parse_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite number")
    return number


def parse_int(value: str) -> int:
    return int(value)


def default_output_dir() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("/tmp/friction_pid_identify") / stamp


def log(message: str, quiet: bool = False) -> None:
    if not quiet:
        print(message)


def load_samples(input_csv: Path, wheel_id: str | None, run_id: int | None) -> list[Sample]:
    with input_csv.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("input CSV has no header")

        missing = REQUIRED_COLUMNS.difference(reader.fieldnames)
        if missing:
            raise ValueError(f"input CSV is missing columns: {', '.join(sorted(missing))}")

        samples: list[Sample] = []
        for row_index, row in enumerate(reader, start=2):
            try:
                enabled_value = row.get("enabled", "1").strip()
                if enabled_value not in {"", "1", "true", "True", "TRUE"}:
                    continue

                sample = Sample(
                    timestamp_us=parse_int(row["timestamp_us"]),
                    wheel_id=row["wheel_id"].strip(),
                    run_id=parse_int(row["run_id"]),
                    sample_idx=parse_int(row["sample_idx"]),
                    setpoint_velocity=parse_float(row["setpoint_velocity"]),
                    measured_velocity=parse_float(row["measured_velocity"]),
                    control_torque=parse_float(row["control_torque"]),
                )
            except Exception as exc:
                raise ValueError(f"failed to parse row {row_index}: {exc}") from exc

            if wheel_id is not None and sample.wheel_id != wheel_id:
                continue
            if run_id is not None and sample.run_id != run_id:
                continue
            samples.append(sample)

    return samples


def group_samples(samples: Iterable[Sample]) -> dict[tuple[str, int], list[Sample]]:
    grouped: dict[tuple[str, int], list[Sample]] = {}
    for sample in samples:
        grouped.setdefault((sample.wheel_id, sample.run_id), []).append(sample)
    return grouped


def build_segment(
    wheel_id: str,
    run_id: int,
    rows: list[Sample],
    fallback_dt_s: float,
) -> Segment:
    ordered = sorted(rows, key=lambda item: (item.sample_idx, item.timestamp_us))

    filtered: list[Sample] = []
    dropped_rows = 0
    last_sample_idx: int | None = None
    last_timestamp_us: int | None = None
    for row in ordered:
        if last_sample_idx is not None and row.sample_idx <= last_sample_idx:
            dropped_rows += 1
            continue
        if last_timestamp_us is not None and row.timestamp_us <= last_timestamp_us:
            dropped_rows += 1
            continue
        filtered.append(row)
        last_sample_idx = row.sample_idx
        last_timestamp_us = row.timestamp_us

    if not filtered:
        raise ValueError(f"{wheel_id}/run{run_id}: no valid rows remain after ordering checks")

    timestamp_us = np.asarray([row.timestamp_us for row in filtered], dtype=np.int64)
    sample_idx = np.asarray([row.sample_idx for row in filtered], dtype=np.int64)
    setpoint_velocity = np.asarray([row.setpoint_velocity for row in filtered], dtype=np.float64)
    measured_velocity = np.asarray([row.measured_velocity for row in filtered], dtype=np.float64)
    control_torque = np.asarray([row.control_torque for row in filtered], dtype=np.float64)

    inferred_dt_s, sample_jitter_ratio = infer_sample_time(timestamp_us, fallback_dt_s)
    missing_samples = count_missing_samples(sample_idx)

    return Segment(
        wheel_id=wheel_id,
        run_id=run_id,
        timestamp_us=timestamp_us,
        sample_idx=sample_idx,
        setpoint_velocity=setpoint_velocity,
        measured_velocity=measured_velocity,
        control_torque=control_torque,
        dropped_rows=dropped_rows,
        inferred_dt_s=inferred_dt_s,
        sample_jitter_ratio=sample_jitter_ratio,
        missing_samples=missing_samples,
    )


def infer_sample_time(timestamp_us: np.ndarray, fallback_dt_s: float) -> tuple[float, float]:
    if timestamp_us.size < 2:
        return fallback_dt_s, 0.0

    dt_us = np.diff(timestamp_us).astype(np.float64)
    dt_us = dt_us[dt_us > 0.0]
    if dt_us.size == 0:
        return fallback_dt_s, 0.0

    median_dt_us = float(np.median(dt_us))
    if not math.isfinite(median_dt_us) or median_dt_us <= 0.0:
        return fallback_dt_s, 0.0

    mad_us = float(np.median(np.abs(dt_us - median_dt_us)))
    jitter_ratio = 0.0 if median_dt_us == 0.0 else mad_us / median_dt_us
    return median_dt_us / 1e6, jitter_ratio


def count_missing_samples(sample_idx: np.ndarray) -> int:
    if sample_idx.size < 2:
        return 0
    diffs = np.diff(sample_idx)
    return int(np.sum(np.clip(diffs - 1, 0, None)))


def excitation_ok(segment: Segment, min_speed_span: float, min_torque_span: float) -> tuple[bool, str]:
    speed_span = percentile_span(segment.measured_velocity)
    torque_span = percentile_span(segment.control_torque)
    if speed_span < min_speed_span:
        return False, f"measured velocity span too small ({speed_span:.3f} < {min_speed_span:.3f})"
    if torque_span < min_torque_span:
        return False, f"control torque span too small ({torque_span:.3f} < {min_torque_span:.3f})"
    return True, ""


def percentile_span(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.percentile(values, 95) - np.percentile(values, 5))


def estimate_segment(
    segment: Segment,
    max_delay_samples: int,
    allow_unphysical: bool,
) -> CandidateResult:
    if segment.measured_velocity.size < 3:
        raise ValueError(f"{segment.wheel_id}/run{segment.run_id}: too few samples to fit")

    best_result: CandidateResult | None = None
    best_unphysical: CandidateResult | None = None

    for delay_samples in range(max_delay_samples + 1):
        candidate = fit_delay_candidate(segment.measured_velocity, segment.control_torque, delay_samples)
        if candidate is None:
            continue

        if candidate.physically_plausible:
            if best_result is None or compare_candidates(candidate, best_result):
                best_result = candidate
        elif best_unphysical is None or compare_candidates(candidate, best_unphysical):
            best_unphysical = candidate

    if best_result is not None:
        return best_result
    if allow_unphysical and best_unphysical is not None:
        return best_unphysical
    if best_unphysical is not None:
        raise ValueError(
            f"{segment.wheel_id}/run{segment.run_id}: only unphysical candidates found; "
            "rerun with --allow-unphysical to inspect them"
        )
    raise ValueError(f"{segment.wheel_id}/run{segment.run_id}: no candidate could be fitted")


def fit_delay_candidate(y: np.ndarray, u: np.ndarray, delay_samples: int) -> CandidateResult | None:
    sample_count = y.size
    if sample_count - delay_samples < 3:
        return None

    k = np.arange(delay_samples, sample_count - 1, dtype=np.int64)
    if k.size < 3:
        return None

    x = np.column_stack((y[k], u[k - delay_samples], np.ones(k.size, dtype=np.float64)))
    target = y[k + 1]

    theta, *_ = np.linalg.lstsq(x, target, rcond=None)
    a, b, c = (float(theta[0]), float(theta[1]), float(theta[2]))
    if not all(math.isfinite(value) for value in (a, b, c)):
        return None

    one_step = x @ theta
    one_step_rmse = rmse(target, one_step)

    predicted = np.zeros_like(y)
    predicted[0] = y[0]
    for index in range(sample_count - 1):
        delayed_u = u[index - delay_samples] if index >= delay_samples else 0.0
        predicted[index + 1] = a * predicted[index] + b * delayed_u + c

    actual_tail = y[1:]
    predicted_tail = predicted[1:]
    if actual_tail.size == 0:
        return None

    rollout_rmse = rmse(actual_tail, predicted_tail)
    mae = float(np.mean(np.abs(actual_tail - predicted_tail)))
    fit_percent = normalized_fit_percent(actual_tail, predicted_tail)
    physically_plausible = (0.0 < a < 1.0) and (b > 0.0)

    return CandidateResult(
        delay_samples=delay_samples,
        a=a,
        b=b,
        c=c,
        rollout_rmse=rollout_rmse,
        one_step_rmse=one_step_rmse,
        mae=mae,
        fit_percent=fit_percent,
        physically_plausible=physically_plausible,
        predicted_velocity=predicted,
    )


def compare_candidates(left: CandidateResult, right: CandidateResult) -> bool:
    if left.rollout_rmse < right.rollout_rmse - 1e-9:
        return True
    if math.isclose(left.rollout_rmse, right.rollout_rmse, rel_tol=0.0, abs_tol=1e-9):
        if left.fit_percent > right.fit_percent + 1e-9:
            return True
        if math.isclose(left.fit_percent, right.fit_percent, rel_tol=0.0, abs_tol=1e-9):
            return left.delay_samples < right.delay_samples
    return False


def rmse(actual: np.ndarray, predicted: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(actual - predicted))))


def normalized_fit_percent(actual: np.ndarray, predicted: np.ndarray) -> float:
    centered = actual - np.mean(actual)
    denominator = float(np.linalg.norm(centered))
    if denominator <= 0.0:
        return 0.0
    numerator = float(np.linalg.norm(actual - predicted))
    return 100.0 * max(0.0, 1.0 - numerator / denominator)


def continuous_params(candidate: CandidateResult, dt_s: float) -> tuple[float | None, float | None, float | None]:
    a = candidate.a
    if not (0.0 < a < 1.0):
        return None, None, None

    one_minus_a = 1.0 - a
    if one_minus_a <= 0.0:
        return None, None, None

    time_constant_s = -dt_s / math.log(a)
    dc_gain = candidate.b / one_minus_a
    velocity_bias = candidate.c / one_minus_a
    return time_constant_s, dc_gain, velocity_bias


def sanitize_token(value: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return sanitized or "unknown"


def write_prediction_csv(
    output_path: Path,
    segment: Segment,
    candidate: CandidateResult,
) -> None:
    with output_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "timestamp_us",
                "sample_idx",
                "wheel_id",
                "run_id",
                "setpoint_velocity",
                "measured_velocity",
                "predicted_velocity",
                "control_torque",
                "residual",
            ]
        )
        residual = segment.measured_velocity - candidate.predicted_velocity
        for index in range(segment.measured_velocity.size):
            writer.writerow(
                [
                    int(segment.timestamp_us[index]),
                    int(segment.sample_idx[index]),
                    segment.wheel_id,
                    segment.run_id,
                    float(segment.setpoint_velocity[index]),
                    float(segment.measured_velocity[index]),
                    float(candidate.predicted_velocity[index]),
                    float(segment.control_torque[index]),
                    float(residual[index]),
                ]
            )


def write_model_json(
    output_path: Path,
    segment: Segment,
    candidate: CandidateResult,
    time_constant_s: float | None,
    dc_gain: float | None,
    velocity_bias: float | None,
    excitation_message: str,
) -> None:
    payload = {
        "wheel_id": segment.wheel_id,
        "run_id": segment.run_id,
        "sample_count": int(segment.measured_velocity.size),
        "dropped_rows": segment.dropped_rows,
        "missing_samples": segment.missing_samples,
        "inferred_dt_s": segment.inferred_dt_s,
        "sample_jitter_ratio": segment.sample_jitter_ratio,
        "excitation_warning": excitation_message or None,
        "model_type": "first_order_plus_dead_time_discrete",
        "discrete": {
            "a": candidate.a,
            "b": candidate.b,
            "c": candidate.c,
            "delay_samples": candidate.delay_samples,
        },
        "continuous": {
            "time_constant_s": time_constant_s,
            "dc_gain": dc_gain,
            "velocity_bias": velocity_bias,
            "delay_s": candidate.delay_samples * segment.inferred_dt_s,
        },
        "fit": {
            "rollout_rmse": candidate.rollout_rmse,
            "one_step_rmse": candidate.one_step_rmse,
            "mae": candidate.mae,
            "fit_percent": candidate.fit_percent,
            "physically_plausible": candidate.physically_plausible,
        },
    }

    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def write_summary_csv(output_path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "wheel_id",
        "run_id",
        "status",
        "message",
        "sample_count",
        "dropped_rows",
        "missing_samples",
        "inferred_dt_s",
        "sample_jitter_ratio",
        "speed_span",
        "torque_span",
        "delay_samples",
        "delay_s",
        "a",
        "b",
        "c",
        "time_constant_s",
        "dc_gain",
        "velocity_bias",
        "rollout_rmse",
        "one_step_rmse",
        "mae",
        "fit_percent",
        "model_json",
        "prediction_csv",
    ]

    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> int:
    args = parse_args()

    input_csv = Path(args.input_csv)
    if not input_csv.is_file():
        print(f"input CSV does not exist: {input_csv}", file=sys.stderr)
        return 2

    fallback_dt_s = args.sample_time_ms / 1000.0
    if fallback_dt_s <= 0.0:
        print("--sample-time-ms must be positive", file=sys.stderr)
        return 2

    output_dir = Path(args.output_dir) if args.output_dir else default_output_dir()
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        samples = load_samples(input_csv, args.wheel_id, args.run_id)
    except Exception as exc:
        print(f"failed to load CSV: {exc}", file=sys.stderr)
        return 2

    if not samples:
        print("no samples matched the selected wheel/run filters", file=sys.stderr)
        return 2

    grouped = group_samples(samples)
    summary_rows: list[dict[str, object]] = []
    success_count = 0

    for (wheel_id, run_id), rows in sorted(grouped.items()):
        base_summary = {
            "wheel_id": wheel_id,
            "run_id": run_id,
            "model_json": "",
            "prediction_csv": "",
        }

        try:
            segment = build_segment(wheel_id, run_id, rows, fallback_dt_s)
        except Exception as exc:
            summary_rows.append(
                {
                    **base_summary,
                    "status": "error",
                    "message": str(exc),
                }
            )
            continue

        if segment.measured_velocity.size < args.min_samples:
            summary_rows.append(
                {
                    **base_summary,
                    "status": "skipped",
                    "message": (
                        f"too few samples ({segment.measured_velocity.size} < {args.min_samples})"
                    ),
                    "sample_count": int(segment.measured_velocity.size),
                    "dropped_rows": segment.dropped_rows,
                    "missing_samples": segment.missing_samples,
                    "inferred_dt_s": segment.inferred_dt_s,
                    "sample_jitter_ratio": segment.sample_jitter_ratio,
                    "speed_span": percentile_span(segment.measured_velocity),
                    "torque_span": percentile_span(segment.control_torque),
                }
            )
            continue

        excitation_good, excitation_message = excitation_ok(
            segment,
            min_speed_span=args.min_speed_span,
            min_torque_span=args.min_torque_span,
        )
        if not excitation_good:
            summary_rows.append(
                {
                    **base_summary,
                    "status": "skipped",
                    "message": excitation_message,
                    "sample_count": int(segment.measured_velocity.size),
                    "dropped_rows": segment.dropped_rows,
                    "missing_samples": segment.missing_samples,
                    "inferred_dt_s": segment.inferred_dt_s,
                    "sample_jitter_ratio": segment.sample_jitter_ratio,
                    "speed_span": percentile_span(segment.measured_velocity),
                    "torque_span": percentile_span(segment.control_torque),
                }
            )
            continue

        if args.max_delay_samples is not None:
            max_delay_samples = max(0, args.max_delay_samples)
        else:
            max_delay_samples = max(0, int(round((args.max_delay_ms / 1000.0) / segment.inferred_dt_s)))

        try:
            candidate = estimate_segment(
                segment=segment,
                max_delay_samples=max_delay_samples,
                allow_unphysical=args.allow_unphysical,
            )
        except Exception as exc:
            summary_rows.append(
                {
                    **base_summary,
                    "status": "error",
                    "message": str(exc),
                    "sample_count": int(segment.measured_velocity.size),
                    "dropped_rows": segment.dropped_rows,
                    "missing_samples": segment.missing_samples,
                    "inferred_dt_s": segment.inferred_dt_s,
                    "sample_jitter_ratio": segment.sample_jitter_ratio,
                    "speed_span": percentile_span(segment.measured_velocity),
                    "torque_span": percentile_span(segment.control_torque),
                }
            )
            continue

        time_constant_s, dc_gain, velocity_bias = continuous_params(candidate, segment.inferred_dt_s)

        wheel_token = sanitize_token(wheel_id)
        run_token = f"run_{run_id}"
        model_json_path = output_dir / f"{wheel_token}_{run_token}_model.json"
        prediction_csv_path = output_dir / f"{wheel_token}_{run_token}_fit.csv"

        write_model_json(
            model_json_path,
            segment,
            candidate,
            time_constant_s,
            dc_gain,
            velocity_bias,
            excitation_message="" if excitation_good else excitation_message,
        )
        write_prediction_csv(prediction_csv_path, segment, candidate)

        speed_span = percentile_span(segment.measured_velocity)
        torque_span = percentile_span(segment.control_torque)
        summary_rows.append(
            {
                **base_summary,
                "status": "ok",
                "message": "" if candidate.physically_plausible else "selected unphysical candidate",
                "sample_count": int(segment.measured_velocity.size),
                "dropped_rows": segment.dropped_rows,
                "missing_samples": segment.missing_samples,
                "inferred_dt_s": segment.inferred_dt_s,
                "sample_jitter_ratio": segment.sample_jitter_ratio,
                "speed_span": speed_span,
                "torque_span": torque_span,
                "delay_samples": candidate.delay_samples,
                "delay_s": candidate.delay_samples * segment.inferred_dt_s,
                "a": candidate.a,
                "b": candidate.b,
                "c": candidate.c,
                "time_constant_s": time_constant_s,
                "dc_gain": dc_gain,
                "velocity_bias": velocity_bias,
                "rollout_rmse": candidate.rollout_rmse,
                "one_step_rmse": candidate.one_step_rmse,
                "mae": candidate.mae,
                "fit_percent": candidate.fit_percent,
                "model_json": str(model_json_path),
                "prediction_csv": str(prediction_csv_path),
            }
        )

        success_count += 1
        log(
            (
                f"[ok] {wheel_id}/run{run_id}: delay={candidate.delay_samples} samples, "
                f"tau={time_constant_s if time_constant_s is not None else float('nan'):.6f}s, "
                f"gain={dc_gain if dc_gain is not None else float('nan'):.6f}, "
                f"fit={candidate.fit_percent:.2f}%"
            ),
            quiet=args.quiet,
        )

    summary_path = output_dir / "identification_summary.csv"
    write_summary_csv(summary_path, summary_rows)

    if success_count == 0:
        print(f"no segment was successfully identified; summary: {summary_path}", file=sys.stderr)
        return 1

    log(f"summary: {summary_path}", quiet=args.quiet)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
