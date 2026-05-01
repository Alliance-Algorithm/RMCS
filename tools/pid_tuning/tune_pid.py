#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np

from identify_plant import build_segment, group_samples, load_samples, sanitize_token


CURRENT_DEFAULT_KP = 0.003436926
CURRENT_DEFAULT_KD = 0.009373434


@dataclass(frozen=True)
class PlantModel:
    wheel_id: str
    run_id: int
    dt_s: float
    a: float
    b: float
    c: float
    delay_samples: int
    source_path: Path
    fit_percent: float | None


@dataclass(frozen=True)
class TraceData:
    wheel_id: str
    run_id: int
    dt_s: float
    setpoint: np.ndarray
    initial_velocity: float
    observed_velocity: np.ndarray | None
    observed_control: np.ndarray | None
    source_name: str


@dataclass(frozen=True)
class SimulationResult:
    velocity: np.ndarray
    control: np.ndarray
    diverged: bool


@dataclass(frozen=True)
class Metrics:
    cost: float
    mae_norm: float
    rmse_norm: float
    itae_norm: float
    overshoot_norm: float
    steady_error_norm: float
    final_error_norm: float
    settle_ratio: float
    control_rms_norm: float
    control_tv_norm: float
    saturation_ratio: float


@dataclass(frozen=True)
class EvaluationRecord:
    kp: float
    kd: float
    cost: float
    mae_norm: float
    rmse_norm: float
    itae_norm: float
    overshoot_norm: float
    steady_error_norm: float
    final_error_norm: float
    settle_ratio: float
    control_rms_norm: float
    control_tv_norm: float
    saturation_ratio: float
    method: str
    evaluation_index: int


@dataclass(frozen=True)
class TuningOutcome:
    model: PlantModel
    trace_count: int
    output_min: float
    output_max: float
    best: EvaluationRecord
    baseline: EvaluationRecord
    best_simulation: SimulationResult
    best_trace: TraceData
    output_dir: Path


class ObjectiveEvaluator:
    def __init__(
        self,
        model: PlantModel,
        traces: list[TraceData],
        output_min: float,
        output_max: float,
        settle_band_ratio: float,
        quiet: bool,
    ) -> None:
        self.model = model
        self.traces = traces
        self.output_min = output_min
        self.output_max = output_max
        self.settle_band_ratio = settle_band_ratio
        self.quiet = quiet
        self.cache: dict[tuple[float, float], EvaluationRecord] = {}
        self.records: list[EvaluationRecord] = []
        self.best_record: EvaluationRecord | None = None

    def evaluate(self, kp: float, kd: float, method: str) -> EvaluationRecord:
        kp = float(kp)
        kd = float(kd)
        key = (round(kp, 15), round(kd, 15))
        cached = self.cache.get(key)
        if cached is not None:
            return cached

        trace_metrics: list[Metrics] = []
        for trace in self.traces:
            simulation = simulate_closed_loop(
                model=self.model,
                trace=trace,
                kp=kp,
                kd=kd,
                output_min=self.output_min,
                output_max=self.output_max,
            )
            metrics = compute_metrics(
                trace=trace,
                simulation=simulation,
                settle_band_ratio=self.settle_band_ratio,
                output_min=self.output_min,
                output_max=self.output_max,
            )
            trace_metrics.append(metrics)

        record = EvaluationRecord(
            kp=kp,
            kd=kd,
            cost=mean_metric(trace_metrics, "cost"),
            mae_norm=mean_metric(trace_metrics, "mae_norm"),
            rmse_norm=mean_metric(trace_metrics, "rmse_norm"),
            itae_norm=mean_metric(trace_metrics, "itae_norm"),
            overshoot_norm=mean_metric(trace_metrics, "overshoot_norm"),
            steady_error_norm=mean_metric(trace_metrics, "steady_error_norm"),
            final_error_norm=mean_metric(trace_metrics, "final_error_norm"),
            settle_ratio=mean_metric(trace_metrics, "settle_ratio"),
            control_rms_norm=mean_metric(trace_metrics, "control_rms_norm"),
            control_tv_norm=mean_metric(trace_metrics, "control_tv_norm"),
            saturation_ratio=mean_metric(trace_metrics, "saturation_ratio"),
            method=method,
            evaluation_index=len(self.records),
        )

        self.records.append(record)
        self.cache[key] = record
        if self.best_record is None or record.cost < self.best_record.cost:
            self.best_record = record
            if not self.quiet:
                print(
                    (
                        f"[best] {self.model.wheel_id}/run{self.model.run_id}: "
                        f"kp={record.kp:.9g}, kd={record.kd:.9g}, cost={record.cost:.6f}"
                    )
                )
        return record


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Tune friction wheel velocity PID gains offline from identified plant models. "
            "The discrete control law matches rmcs_core::controller::pid::PidCalculator "
            "with ki fixed to 0."
        )
    )
    parser.add_argument(
        "model_input",
        help="Either one *_model.json from identify_plant.py or identification_summary.csv",
    )
    parser.add_argument(
        "--trace-csv",
        help=(
            "Recorder CSV from FrictionWheelPidRecorder. If provided, the original setpoint trace "
            "is reused for closed-loop evaluation."
        ),
    )
    parser.add_argument("--wheel-id", help="Only tune one wheel_id")
    parser.add_argument("--run-id", type=int, help="Only tune one run_id")
    parser.add_argument(
        "--trace-scope",
        choices=("exact", "wheel"),
        default="exact",
        help="How to match trace segments to a model when --trace-csv is provided",
    )
    parser.add_argument(
        "--sample-time-ms",
        type=float,
        default=1.0,
        help="Fallback sample interval for trace CSV parsing when timestamps are invalid",
    )
    parser.add_argument(
        "--method",
        choices=("hybrid", "ga", "bo"),
        default="hybrid",
        help="Optimization method",
    )
    parser.add_argument("--kp-min", type=float, default=1e-6, help="Lower bound for kp")
    parser.add_argument("--kp-max", type=float, default=1.0, help="Upper bound for kp")
    parser.add_argument("--kd-min", type=float, default=1e-8, help="Lower bound for kd")
    parser.add_argument("--kd-max", type=float, default=1.0, help="Upper bound for kd")
    parser.add_argument(
        "--seed-kp",
        type=float,
        default=CURRENT_DEFAULT_KP,
        help="Initial/reference kp to compare against",
    )
    parser.add_argument(
        "--seed-kd",
        type=float,
        default=CURRENT_DEFAULT_KD,
        help="Initial/reference kd to compare against",
    )
    parser.add_argument(
        "--ga-population",
        type=int,
        default=36,
        help="GA population size",
    )
    parser.add_argument(
        "--ga-generations",
        type=int,
        default=20,
        help="GA generation count",
    )
    parser.add_argument(
        "--bo-initial",
        type=int,
        default=12,
        help="BO random warmup samples",
    )
    parser.add_argument(
        "--bo-iterations",
        type=int,
        default=24,
        help="BO surrogate iterations",
    )
    parser.add_argument(
        "--random-seed",
        type=int,
        default=42,
        help="Random seed for deterministic tuning runs",
    )
    parser.add_argument(
        "--output-limit",
        type=float,
        help="Symmetric control limit. Equivalent to --output-min=-x --output-max=x",
    )
    parser.add_argument("--output-min", type=float, help="Lower control clamp")
    parser.add_argument("--output-max", type=float, help="Upper control clamp")
    parser.add_argument(
        "--infer-output-percentile",
        type=float,
        default=99.5,
        help="Percentile used to infer |control_torque| limit from --trace-csv",
    )
    parser.add_argument(
        "--estimated-output-factor",
        type=float,
        default=2.5,
        help="Fallback multiplier when output limit is estimated from target speed and plant gain",
    )
    parser.add_argument(
        "--step-setpoint",
        type=float,
        help="Target speed for synthetic step evaluation when --trace-csv is not provided",
    )
    parser.add_argument(
        "--step-duration-ms",
        type=float,
        default=1200.0,
        help="Synthetic step hold duration in milliseconds",
    )
    parser.add_argument(
        "--pre-roll-ms",
        type=float,
        default=30.0,
        help="Synthetic zero-input lead-in before the step",
    )
    parser.add_argument(
        "--settle-band-ratio",
        type=float,
        default=0.05,
        help="Relative settling band used by the objective",
    )
    parser.add_argument(
        "--output-dir",
        help="Directory for tuning outputs. Default: /tmp/friction_pid_tuning/<timestamp>",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress progress output",
    )
    return parser.parse_args()


def default_output_dir() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("/tmp/friction_pid_tuning") / stamp


def load_models(model_input: Path, wheel_id: str | None, run_id: int | None) -> list[PlantModel]:
    if not model_input.is_file():
        raise ValueError(f"model input does not exist: {model_input}")

    if model_input.suffix.lower() == ".json":
        models = [load_model_json(model_input)]
    else:
        models = load_models_from_summary(model_input)

    filtered = [
        model
        for model in models
        if (wheel_id is None or model.wheel_id == wheel_id)
        and (run_id is None or model.run_id == run_id)
    ]
    if not filtered:
        raise ValueError("no model matched the selected wheel/run filters")
    return filtered


def load_model_json(path: Path) -> PlantModel:
    payload = json.loads(path.read_text())
    discrete = payload["discrete"]
    fit = payload.get("fit", {})
    dt_s = float(payload["inferred_dt_s"])
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError(f"{path}: invalid inferred_dt_s")

    return PlantModel(
        wheel_id=str(payload["wheel_id"]),
        run_id=int(payload["run_id"]),
        dt_s=dt_s,
        a=float(discrete["a"]),
        b=float(discrete["b"]),
        c=float(discrete["c"]),
        delay_samples=int(discrete["delay_samples"]),
        source_path=path,
        fit_percent=float(fit["fit_percent"]) if "fit_percent" in fit else None,
    )


def load_models_from_summary(path: Path) -> list[PlantModel]:
    models: list[PlantModel] = []
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("status") != "ok":
                continue
            model_json = (row.get("model_json") or "").strip()
            if not model_json:
                continue
            model_path = Path(model_json)
            if not model_path.is_absolute():
                model_path = (path.parent / model_path).resolve()
            models.append(load_model_json(model_path))

    if not models:
        raise ValueError(f"{path}: no usable model_json rows found in summary")
    return models


def load_trace_map(trace_csv: Path, fallback_dt_s: float) -> dict[tuple[str, int], TraceData]:
    samples = load_samples(trace_csv, None, None)
    grouped = group_samples(samples)
    trace_map: dict[tuple[str, int], TraceData] = {}
    for (wheel_id, run_id), rows in grouped.items():
        segment = build_segment(wheel_id, run_id, rows, fallback_dt_s)
        trace_map[(wheel_id, run_id)] = TraceData(
            wheel_id=wheel_id,
            run_id=run_id,
            dt_s=segment.inferred_dt_s,
            setpoint=segment.setpoint_velocity.copy(),
            initial_velocity=float(segment.measured_velocity[0]) if segment.measured_velocity.size else 0.0,
            observed_velocity=segment.measured_velocity.copy(),
            observed_control=segment.control_torque.copy(),
            source_name=f"{trace_csv}:{wheel_id}/run{run_id}",
        )
    return trace_map


def select_traces(
    model: PlantModel,
    trace_map: dict[tuple[str, int], TraceData] | None,
    trace_scope: str,
    step_setpoint: float | None,
    step_duration_ms: float,
    pre_roll_ms: float,
) -> list[TraceData]:
    if trace_map:
        exact = trace_map.get((model.wheel_id, model.run_id))
        if exact is not None:
            return [exact]
        if trace_scope == "wheel":
            same_wheel = [
                trace
                for trace in trace_map.values()
                if trace.wheel_id == model.wheel_id
            ]
            if same_wheel:
                return sorted(same_wheel, key=lambda trace: trace.run_id)
        raise ValueError(
            f"no trace matched {model.wheel_id}/run{model.run_id}; "
            "provide the corresponding recorder CSV or switch to synthetic step mode"
        )

    if step_setpoint is None:
        raise ValueError(
            f"{model.wheel_id}/run{model.run_id}: --step-setpoint is required when --trace-csv is absent"
        )
    return [make_step_trace(model, step_setpoint, step_duration_ms, pre_roll_ms)]


def make_step_trace(
    model: PlantModel,
    step_setpoint: float,
    step_duration_ms: float,
    pre_roll_ms: float,
) -> TraceData:
    pre_samples = max(1, int(round((pre_roll_ms / 1000.0) / model.dt_s)))
    hold_samples = max(10, int(round((step_duration_ms / 1000.0) / model.dt_s)))
    total = pre_samples + hold_samples
    setpoint = np.zeros(total, dtype=np.float64)
    setpoint[pre_samples:] = step_setpoint

    return TraceData(
        wheel_id=model.wheel_id,
        run_id=model.run_id,
        dt_s=model.dt_s,
        setpoint=setpoint,
        initial_velocity=0.0,
        observed_velocity=None,
        observed_control=None,
        source_name=f"synthetic_step:{model.wheel_id}/run{model.run_id}",
    )


def infer_output_bounds(
    model: PlantModel,
    traces: list[TraceData],
    output_limit: float | None,
    output_min: float | None,
    output_max: float | None,
    infer_output_percentile: float,
    estimated_output_factor: float,
) -> tuple[float, float]:
    if output_limit is not None:
        limit = abs(output_limit)
        if limit <= 0.0:
            raise ValueError("--output-limit must be positive")
        return -limit, limit

    if output_min is not None or output_max is not None:
        if output_min is None or output_max is None:
            raise ValueError("--output-min and --output-max must be set together")
        if output_min >= output_max:
            raise ValueError("--output-min must be smaller than --output-max")
        return float(output_min), float(output_max)

    observed_controls = [
        trace.observed_control
        for trace in traces
        if trace.observed_control is not None and trace.observed_control.size > 0
    ]
    if observed_controls:
        merged = np.concatenate(observed_controls)
        limit = float(np.percentile(np.abs(merged), infer_output_percentile))
        limit = max(limit, 1e-6)
        return -limit, limit

    dc_gain = plant_dc_gain(model)
    target_mag = max(trace_target_magnitude(trace) for trace in traces)
    if dc_gain is None or abs(dc_gain) < 1e-9:
        estimated_limit = 10.0
    else:
        estimated_hold = target_mag / abs(dc_gain)
        estimated_limit = max(estimated_output_factor * estimated_hold, 1.0)
    return -estimated_limit, estimated_limit


def plant_dc_gain(model: PlantModel) -> float | None:
    denominator = 1.0 - model.a
    if abs(denominator) < 1e-12:
        return None
    return model.b / denominator


def trace_target_magnitude(trace: TraceData) -> float:
    if trace.setpoint.size == 0:
        return 1.0
    return max(float(np.percentile(np.abs(trace.setpoint), 95)), 1.0)


def simulate_closed_loop(
    model: PlantModel,
    trace: TraceData,
    kp: float,
    kd: float,
    output_min: float,
    output_max: float,
) -> SimulationResult:
    sample_count = trace.setpoint.size
    velocity = np.zeros(sample_count, dtype=np.float64)
    control = np.zeros(sample_count, dtype=np.float64)
    velocity[0] = trace.initial_velocity

    diverged = False
    last_err = math.nan
    divergence_limit = max(50.0 * trace_target_magnitude(trace), 1e6)

    for index in range(sample_count):
        err = float(trace.setpoint[index] - velocity[index])
        command = kp * err
        if not math.isnan(last_err):
            command += kd * (err - last_err)
        last_err = err
        command = min(max(command, output_min), output_max)
        control[index] = command

        if index + 1 >= sample_count:
            continue

        delayed_u = control[index - model.delay_samples] if index >= model.delay_samples else 0.0
        next_velocity = model.a * velocity[index] + model.b * delayed_u + model.c
        if not math.isfinite(next_velocity) or abs(next_velocity) > divergence_limit:
            diverged = True
            velocity[index + 1 :] = divergence_limit * np.sign(next_velocity if math.isfinite(next_velocity) else 1.0)
            break
        velocity[index + 1] = next_velocity

    return SimulationResult(velocity=velocity, control=control, diverged=diverged)


def compute_metrics(
    trace: TraceData,
    simulation: SimulationResult,
    settle_band_ratio: float,
    output_min: float,
    output_max: float,
) -> Metrics:
    setpoint = trace.setpoint
    velocity = simulation.velocity
    control = simulation.control
    scale = trace_target_magnitude(trace)
    error = setpoint - velocity
    normalized_error = error / scale

    mae_norm = float(np.mean(np.abs(normalized_error)))
    rmse_norm = float(np.sqrt(np.mean(np.square(normalized_error))))

    time_vector = np.linspace(0.0, 1.0, setpoint.size, endpoint=True)
    itae_norm = float(np.mean(time_vector * np.abs(normalized_error)))

    active_mask = np.abs(setpoint) >= 0.1 * scale
    if not np.any(active_mask):
        active_mask = np.ones(setpoint.size, dtype=bool)
    active_indices = np.where(active_mask)[0]
    active_start = int(active_indices[0])

    plateau_mask = active_mask & (
        np.abs(np.diff(setpoint, prepend=setpoint[0])) <= 0.005 * scale
    )
    if np.count_nonzero(plateau_mask) < max(5, setpoint.size // 20):
        plateau_mask = np.zeros(setpoint.size, dtype=bool)
        plateau_mask[max(active_start, int(setpoint.size * 0.8)) :] = True
    plateau_indices = np.where(plateau_mask)[0]

    if plateau_indices.size > 0:
        steady_error_norm = float(np.mean(np.abs(normalized_error[plateau_indices])))
        overshoot_norm = (
            float(np.max(np.maximum(0.0, np.abs(velocity[plateau_indices]) - np.abs(setpoint[plateau_indices]))))
            / scale
        )
    else:
        steady_error_norm = float(np.mean(np.abs(normalized_error[-max(1, setpoint.size // 10) :])))
        overshoot_norm = (
            float(np.max(np.maximum(0.0, np.abs(velocity[active_mask]) - np.abs(setpoint[active_mask]))))
            / scale
        )

    band = np.maximum(settle_band_ratio * np.abs(setpoint), settle_band_ratio * scale)
    outside = np.where(active_mask & (np.abs(error) > band))[0]
    if outside.size == 0:
        settle_ratio = 0.0
    else:
        denominator = max(1, (setpoint.size - 1) - active_start)
        settle_ratio = float((outside[-1] - active_start) / denominator)

    final_error_norm = float(abs(error[-1]) / scale)

    finite_limits = math.isfinite(output_min) and math.isfinite(output_max)
    if finite_limits:
        control_scale = max(abs(output_min), abs(output_max), 1.0)
        saturation_margin = 0.01 * control_scale
        saturation_ratio = float(
            np.mean((control <= output_min + saturation_margin) | (control >= output_max - saturation_margin))
        )
    else:
        control_scale = max(float(np.percentile(np.abs(control), 95)), 1.0)
        saturation_ratio = 0.0

    control_rms_norm = float(np.sqrt(np.mean(np.square(control / control_scale))))
    if control.size >= 2:
        control_tv_norm = float(np.mean(np.abs(np.diff(control))) / control_scale)
    else:
        control_tv_norm = 0.0

    divergence_penalty = 50.0 if simulation.diverged else 0.0
    cost = (
        4.0 * mae_norm
        + 2.5 * rmse_norm
        + 2.0 * itae_norm
        + 3.5 * overshoot_norm
        + 2.5 * steady_error_norm
        + 2.0 * final_error_norm
        + 1.2 * settle_ratio
        + 0.25 * control_rms_norm
        + 0.6 * control_tv_norm
        + 1.5 * saturation_ratio
        + divergence_penalty
    )

    return Metrics(
        cost=cost,
        mae_norm=mae_norm,
        rmse_norm=rmse_norm,
        itae_norm=itae_norm,
        overshoot_norm=overshoot_norm,
        steady_error_norm=steady_error_norm,
        final_error_norm=final_error_norm,
        settle_ratio=settle_ratio,
        control_rms_norm=control_rms_norm,
        control_tv_norm=control_tv_norm,
        saturation_ratio=saturation_ratio,
    )


def mean_metric(metrics: Iterable[Metrics], field_name: str) -> float:
    values = [getattr(metric, field_name) for metric in metrics]
    return float(sum(values) / len(values))


def encode_params(kp: float, kd: float) -> np.ndarray:
    return np.asarray([math.log10(kp), math.log10(kd)], dtype=np.float64)


def decode_params(encoded: np.ndarray) -> tuple[float, float]:
    return 10.0 ** float(encoded[0]), 10.0 ** float(encoded[1])


def sample_encoded(bounds: tuple[np.ndarray, np.ndarray], rng: random.Random) -> np.ndarray:
    lower, upper = bounds
    return np.asarray(
        [rng.uniform(float(lower[0]), float(upper[0])), rng.uniform(float(lower[1]), float(upper[1]))],
        dtype=np.float64,
    )


def clamp_encoded(encoded: np.ndarray, bounds: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    lower, upper = bounds
    return np.minimum(np.maximum(encoded, lower), upper)


def build_seed_points(
    bounds: tuple[np.ndarray, np.ndarray],
    seed_kp: float,
    seed_kd: float,
    model: PlantModel,
    traces: list[TraceData],
    output_min: float,
    output_max: float,
) -> list[np.ndarray]:
    lower, upper = bounds
    seeds: list[np.ndarray] = []

    def append_point(kp: float, kd: float) -> None:
        kp = min(max(kp, 10.0 ** float(lower[0])), 10.0 ** float(upper[0]))
        kd = min(max(kd, 10.0 ** float(lower[1])), 10.0 ** float(upper[1]))
        encoded = encode_params(kp, kd)
        for existing in seeds:
            if np.allclose(existing, encoded, atol=1e-12, rtol=0.0):
                return
        seeds.append(encoded)

    append_point(seed_kp, seed_kd)
    append_point(seed_kp * 0.5, seed_kd * 0.5)
    append_point(seed_kp * 2.0, seed_kd * 2.0)
    append_point(seed_kp * 0.5, seed_kd * 2.0)
    append_point(seed_kp * 2.0, max(seed_kd * 0.5, 1e-12))

    dc_gain = plant_dc_gain(model)
    target_mag = max(trace_target_magnitude(trace) for trace in traces)
    output_limit = max(abs(output_min), abs(output_max), 1.0)
    if dc_gain is not None and abs(dc_gain) > 1e-9:
        proportional_guess = min(max(0.35 * output_limit / target_mag, 1e-12), 10.0 ** float(upper[0]))
        derivative_guess = min(max(proportional_guess * max(1.0, model.delay_samples), 1e-12), 10.0 ** float(upper[1]))
        append_point(proportional_guess, derivative_guess)
        append_point(0.75 * proportional_guess, 0.5 * derivative_guess)

    append_point(10.0 ** float(lower[0]), 10.0 ** float(lower[1]))
    return seeds


def run_ga(
    evaluator: ObjectiveEvaluator,
    bounds: tuple[np.ndarray, np.ndarray],
    population_size: int,
    generations: int,
    seed_points: list[np.ndarray],
    rng: random.Random,
    quiet: bool,
) -> None:
    lower, upper = bounds
    population: list[np.ndarray] = [point.copy() for point in seed_points[:population_size]]
    while len(population) < population_size:
        population.append(sample_encoded(bounds, rng))

    for generation in range(generations):
        scored: list[tuple[EvaluationRecord, np.ndarray]] = []
        for encoded in population:
            kp, kd = decode_params(encoded)
            scored.append((evaluator.evaluate(kp, kd, method="ga"), encoded))
        scored.sort(key=lambda item: item[0].cost)

        if not quiet:
            best_record = scored[0][0]
            print(
                (
                    f"[ga] generation={generation + 1}/{generations} "
                    f"best_cost={best_record.cost:.6f} "
                    f"kp={best_record.kp:.9g} kd={best_record.kd:.9g}"
                )
            )

        elite_count = max(2, population_size // 5)
        survivors = [encoded.copy() for _, encoded in scored[:elite_count]]

        next_population: list[np.ndarray] = survivors[:]
        mutation_sigma = max(0.03, 0.18 * (1.0 - generation / max(1, generations)))
        tournament_pool = [encoded for _, encoded in scored[: max(elite_count * 2, 4)]]

        while len(next_population) < population_size:
            parent_a = rng.choice(tournament_pool)
            parent_b = rng.choice(tournament_pool)
            alpha = rng.uniform(0.2, 0.8)
            child = alpha * parent_a + (1.0 - alpha) * parent_b
            if rng.random() < 0.85:
                child = child + np.asarray(
                    [rng.gauss(0.0, mutation_sigma), rng.gauss(0.0, mutation_sigma)],
                    dtype=np.float64,
                )
            if rng.random() < 0.12:
                child = sample_encoded(bounds, rng)
            child = clamp_encoded(child, bounds)
            next_population.append(child)

        population = next_population

    # Evaluate the final population once more so the final offspring enter the archive.
    for encoded in population:
        kp, kd = decode_params(encoded)
        evaluator.evaluate(kp, kd, method="ga")


def kernel_rbf(x1: np.ndarray, x2: np.ndarray, length_scale: float) -> np.ndarray:
    diff = x1[:, None, :] - x2[None, :, :]
    dist_sq = np.sum(np.square(diff), axis=2)
    return np.exp(-0.5 * dist_sq / (length_scale * length_scale))


def normalize_encoded(encoded: np.ndarray, bounds: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    lower, upper = bounds
    return (encoded - lower) / np.maximum(upper - lower, 1e-12)


def run_bo(
    evaluator: ObjectiveEvaluator,
    bounds: tuple[np.ndarray, np.ndarray],
    initial_samples: int,
    iterations: int,
    seed_points: list[np.ndarray],
    rng: random.Random,
    quiet: bool,
) -> None:
    archive_encoded: list[np.ndarray] = []
    archive_costs: list[float] = []

    def register_point(encoded: np.ndarray, method: str) -> None:
        kp, kd = decode_params(encoded)
        record = evaluator.evaluate(kp, kd, method=method)
        archive_encoded.append(encoded.copy())
        archive_costs.append(record.cost)

    seen: set[tuple[float, float]] = set()

    def add_if_new(encoded: np.ndarray, method: str) -> None:
        key = (round(float(encoded[0]), 12), round(float(encoded[1]), 12))
        if key in seen:
            return
        seen.add(key)
        register_point(encoded, method)

    for encoded in seed_points[:initial_samples]:
        add_if_new(clamp_encoded(encoded, bounds), "bo")

    while len(archive_encoded) < initial_samples:
        add_if_new(sample_encoded(bounds, rng), "bo")

    for iteration in range(iterations):
        x_train = np.asarray(archive_encoded, dtype=np.float64)
        y_train = np.asarray(archive_costs, dtype=np.float64)
        x_norm = normalize_encoded(x_train, bounds)

        length_scale = 0.22
        jitter = 1e-6
        k_xx = kernel_rbf(x_norm, x_norm, length_scale) + jitter * np.eye(x_norm.shape[0])
        centered = y_train - np.mean(y_train)
        alpha = np.linalg.solve(k_xx, centered)
        inverse_k = np.linalg.inv(k_xx)

        candidate_pool = [sample_encoded(bounds, rng) for _ in range(2048)]
        if evaluator.best_record is not None:
            best_encoded = encode_params(evaluator.best_record.kp, evaluator.best_record.kd)
            for _ in range(256):
                local = best_encoded + np.asarray(
                    [rng.gauss(0.0, 0.06), rng.gauss(0.0, 0.06)],
                    dtype=np.float64,
                )
                candidate_pool.append(clamp_encoded(local, bounds))

        candidate_matrix = np.asarray(candidate_pool, dtype=np.float64)
        candidate_norm = normalize_encoded(candidate_matrix, bounds)

        k_star = kernel_rbf(x_norm, candidate_norm, length_scale)
        mean = np.mean(y_train) + k_star.T @ alpha
        variance = np.maximum(0.0, 1.0 - np.sum((k_star.T @ inverse_k) * k_star.T, axis=1))
        sigma = np.sqrt(variance)
        beta = max(0.35, 1.75 - 1.2 * (iteration / max(1, iterations)))
        acquisition = mean - beta * sigma

        ordering = np.argsort(acquisition)
        chosen: np.ndarray | None = None
        for idx in ordering:
            encoded = candidate_matrix[int(idx)]
            key = (round(float(encoded[0]), 12), round(float(encoded[1]), 12))
            if key not in seen:
                chosen = encoded
                break
        if chosen is None:
            chosen = sample_encoded(bounds, rng)

        add_if_new(chosen, "bo")

        if not quiet and evaluator.best_record is not None:
            best = evaluator.best_record
            print(
                (
                    f"[bo] iteration={iteration + 1}/{iterations} "
                    f"best_cost={best.cost:.6f} "
                    f"kp={best.kp:.9g} kd={best.kd:.9g}"
                )
            )


def tune_one_model(
    model: PlantModel,
    traces: list[TraceData],
    args: argparse.Namespace,
    rng: random.Random,
    output_dir: Path,
) -> TuningOutcome:
    output_min, output_max = infer_output_bounds(
        model=model,
        traces=traces,
        output_limit=args.output_limit,
        output_min=args.output_min,
        output_max=args.output_max,
        infer_output_percentile=args.infer_output_percentile,
        estimated_output_factor=args.estimated_output_factor,
    )

    if args.kp_min <= 0.0 or args.kd_min <= 0.0:
        raise ValueError("--kp-min and --kd-min must be strictly positive for log-scale search")
    if args.kp_min >= args.kp_max or args.kd_min >= args.kd_max:
        raise ValueError("kp/kd bounds must satisfy min < max")

    bounds = (
        np.asarray([math.log10(args.kp_min), math.log10(args.kd_min)], dtype=np.float64),
        np.asarray([math.log10(args.kp_max), math.log10(args.kd_max)], dtype=np.float64),
    )

    evaluator = ObjectiveEvaluator(
        model=model,
        traces=traces,
        output_min=output_min,
        output_max=output_max,
        settle_band_ratio=args.settle_band_ratio,
        quiet=args.quiet,
    )

    seeds = build_seed_points(
        bounds=bounds,
        seed_kp=args.seed_kp,
        seed_kd=args.seed_kd,
        model=model,
        traces=traces,
        output_min=output_min,
        output_max=output_max,
    )

    baseline = evaluator.evaluate(args.seed_kp, args.seed_kd, method="baseline")

    if args.method in {"ga", "hybrid"}:
        run_ga(
            evaluator=evaluator,
            bounds=bounds,
            population_size=args.ga_population,
            generations=args.ga_generations,
            seed_points=seeds,
            rng=rng,
            quiet=args.quiet,
        )

    if args.method in {"bo", "hybrid"}:
        if args.method == "bo":
            bo_seeds = seeds
        else:
            sorted_records = sorted(evaluator.records, key=lambda record: record.cost)
            bo_seeds = [encode_params(record.kp, record.kd) for record in sorted_records[: max(8, args.bo_initial)]]
            bo_seeds.extend(seeds)

        run_bo(
            evaluator=evaluator,
            bounds=bounds,
            initial_samples=args.bo_initial,
            iterations=args.bo_iterations,
            seed_points=bo_seeds,
            rng=rng,
            quiet=args.quiet,
        )

    if evaluator.best_record is None:
        raise RuntimeError(f"{model.wheel_id}/run{model.run_id}: no candidate was evaluated")

    best = evaluator.best_record
    best_trace = traces[0]
    best_simulation = simulate_closed_loop(
        model=model,
        trace=best_trace,
        kp=best.kp,
        kd=best.kd,
        output_min=output_min,
        output_max=output_max,
    )

    tune_output_dir = output_dir / f"{sanitize_token(model.wheel_id)}_run_{model.run_id}"
    tune_output_dir.mkdir(parents=True, exist_ok=True)

    write_candidates_csv(tune_output_dir / "candidates.csv", evaluator.records)
    write_response_csv(
        tune_output_dir / "best_response.csv",
        trace=best_trace,
        simulation=best_simulation,
        output_min=output_min,
        output_max=output_max,
    )
    write_tuning_json(
        tune_output_dir / "tuning_result.json",
        model=model,
        trace_count=len(traces),
        output_min=output_min,
        output_max=output_max,
        baseline=baseline,
        best=best,
        method=args.method,
    )

    return TuningOutcome(
        model=model,
        trace_count=len(traces),
        output_min=output_min,
        output_max=output_max,
        best=best,
        baseline=baseline,
        best_simulation=best_simulation,
        best_trace=best_trace,
        output_dir=tune_output_dir,
    )


def write_candidates_csv(output_path: Path, records: list[EvaluationRecord]) -> None:
    fieldnames = [
        "evaluation_index",
        "method",
        "kp",
        "kd",
        "cost",
        "mae_norm",
        "rmse_norm",
        "itae_norm",
        "overshoot_norm",
        "steady_error_norm",
        "final_error_norm",
        "settle_ratio",
        "control_rms_norm",
        "control_tv_norm",
        "saturation_ratio",
    ]
    ordered = sorted(records, key=lambda record: (record.cost, record.evaluation_index))
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in ordered:
            writer.writerow(
                {
                    "evaluation_index": record.evaluation_index,
                    "method": record.method,
                    "kp": record.kp,
                    "kd": record.kd,
                    "cost": record.cost,
                    "mae_norm": record.mae_norm,
                    "rmse_norm": record.rmse_norm,
                    "itae_norm": record.itae_norm,
                    "overshoot_norm": record.overshoot_norm,
                    "steady_error_norm": record.steady_error_norm,
                    "final_error_norm": record.final_error_norm,
                    "settle_ratio": record.settle_ratio,
                    "control_rms_norm": record.control_rms_norm,
                    "control_tv_norm": record.control_tv_norm,
                    "saturation_ratio": record.saturation_ratio,
                }
            )


def write_response_csv(
    output_path: Path,
    trace: TraceData,
    simulation: SimulationResult,
    output_min: float,
    output_max: float,
) -> None:
    saturation_margin = 0.01 * max(abs(output_min), abs(output_max), 1.0)
    with output_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "sample_idx",
                "time_s",
                "setpoint_velocity",
                "simulated_velocity",
                "simulated_control",
                "saturated",
                "observed_velocity",
                "observed_control",
            ]
        )
        for index in range(trace.setpoint.size):
            simulated_control = float(simulation.control[index])
            saturated = int(
                (simulated_control <= output_min + saturation_margin)
                or (simulated_control >= output_max - saturation_margin)
            )
            writer.writerow(
                [
                    index,
                    index * trace.dt_s,
                    float(trace.setpoint[index]),
                    float(simulation.velocity[index]),
                    simulated_control,
                    saturated,
                    "" if trace.observed_velocity is None else float(trace.observed_velocity[index]),
                    "" if trace.observed_control is None else float(trace.observed_control[index]),
                ]
            )


def write_tuning_json(
    output_path: Path,
    model: PlantModel,
    trace_count: int,
    output_min: float,
    output_max: float,
    baseline: EvaluationRecord,
    best: EvaluationRecord,
    method: str,
) -> None:
    payload = {
        "wheel_id": model.wheel_id,
        "run_id": model.run_id,
        "trace_count": trace_count,
        "method": method,
        "ki": 0.0,
        "output_min": output_min,
        "output_max": output_max,
        "plant_model": {
            "source_path": str(model.source_path),
            "dt_s": model.dt_s,
            "a": model.a,
            "b": model.b,
            "c": model.c,
            "delay_samples": model.delay_samples,
            "fit_percent": model.fit_percent,
        },
        "baseline": record_to_dict(baseline),
        "best": record_to_dict(best),
        "improvement": {
            "absolute_cost_reduction": baseline.cost - best.cost,
            "relative_cost_reduction": 0.0 if baseline.cost == 0.0 else (baseline.cost - best.cost) / baseline.cost,
        },
    }
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def record_to_dict(record: EvaluationRecord) -> dict[str, float | str | int]:
    return {
        "kp": record.kp,
        "ki": 0.0,
        "kd": record.kd,
        "cost": record.cost,
        "mae_norm": record.mae_norm,
        "rmse_norm": record.rmse_norm,
        "itae_norm": record.itae_norm,
        "overshoot_norm": record.overshoot_norm,
        "steady_error_norm": record.steady_error_norm,
        "final_error_norm": record.final_error_norm,
        "settle_ratio": record.settle_ratio,
        "control_rms_norm": record.control_rms_norm,
        "control_tv_norm": record.control_tv_norm,
        "saturation_ratio": record.saturation_ratio,
        "method": record.method,
        "evaluation_index": record.evaluation_index,
    }


def write_summary_csv(output_path: Path, outcomes: list[TuningOutcome]) -> None:
    fieldnames = [
        "wheel_id",
        "run_id",
        "trace_count",
        "method",
        "seed_kp",
        "seed_kd",
        "best_kp",
        "best_kd",
        "baseline_cost",
        "best_cost",
        "absolute_cost_reduction",
        "relative_cost_reduction",
        "output_min",
        "output_max",
        "result_dir",
    ]
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for outcome in outcomes:
            baseline = outcome.baseline
            best = outcome.best
            writer.writerow(
                {
                    "wheel_id": outcome.model.wheel_id,
                    "run_id": outcome.model.run_id,
                    "trace_count": outcome.trace_count,
                    "method": best.method,
                    "seed_kp": baseline.kp,
                    "seed_kd": baseline.kd,
                    "best_kp": best.kp,
                    "best_kd": best.kd,
                    "baseline_cost": baseline.cost,
                    "best_cost": best.cost,
                    "absolute_cost_reduction": baseline.cost - best.cost,
                    "relative_cost_reduction": 0.0 if baseline.cost == 0.0 else (baseline.cost - best.cost) / baseline.cost,
                    "output_min": outcome.output_min,
                    "output_max": outcome.output_max,
                    "result_dir": str(outcome.output_dir),
                }
            )


def main() -> int:
    args = parse_args()
    model_input = Path(args.model_input)
    output_dir = Path(args.output_dir) if args.output_dir else default_output_dir()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.sample_time_ms <= 0.0:
        print("--sample-time-ms must be positive", file=sys.stderr)
        return 2
    if args.step_duration_ms <= 0.0 or args.pre_roll_ms < 0.0:
        print("--step-duration-ms must be positive and --pre-roll-ms must be non-negative", file=sys.stderr)
        return 2

    try:
        models = load_models(model_input, args.wheel_id, args.run_id)
    except Exception as exc:
        print(f"failed to load model input: {exc}", file=sys.stderr)
        return 2

    trace_map = None
    if args.trace_csv:
        try:
            trace_map = load_trace_map(Path(args.trace_csv), args.sample_time_ms / 1000.0)
        except Exception as exc:
            print(f"failed to load trace CSV: {exc}", file=sys.stderr)
            return 2

    rng = random.Random(args.random_seed)
    outcomes: list[TuningOutcome] = []

    for model in models:
        try:
            traces = select_traces(
                model=model,
                trace_map=trace_map,
                trace_scope=args.trace_scope,
                step_setpoint=args.step_setpoint,
                step_duration_ms=args.step_duration_ms,
                pre_roll_ms=args.pre_roll_ms,
            )
            outcome = tune_one_model(
                model=model,
                traces=traces,
                args=args,
                rng=rng,
                output_dir=output_dir,
            )
        except Exception as exc:
            print(f"[error] {model.wheel_id}/run{model.run_id}: {exc}", file=sys.stderr)
            continue

        outcomes.append(outcome)
        if not args.quiet:
            print(
                (
                    f"[ok] {model.wheel_id}/run{model.run_id}: "
                    f"kp={outcome.best.kp:.9g}, kd={outcome.best.kd:.9g}, "
                    f"cost={outcome.best.cost:.6f}, "
                    f"baseline={outcome.baseline.cost:.6f}, "
                    f"dir={outcome.output_dir}"
                )
            )

    if not outcomes:
        print("no model was successfully tuned", file=sys.stderr)
        return 1

    summary_path = output_dir / "tuning_summary.csv"
    write_summary_csv(summary_path, outcomes)
    if not args.quiet:
        print(f"summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
