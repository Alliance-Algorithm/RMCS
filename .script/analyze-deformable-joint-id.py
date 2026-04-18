#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
from pathlib import Path


def is_finite(value):
    return math.isfinite(value)


def fit_sine_coefficients(samples, value_key):
    ss = 0.0
    cc = 0.0
    sc = 0.0
    ys = 0.0
    yc = 0.0

    for row in samples:
        phase = row["phase_rad"]
        value = row[value_key]
        s = math.sin(phase)
        c = math.cos(phase)
        ss += s * s
        cc += c * c
        sc += s * c
        ys += value * s
        yc += value * c

    det = ss * cc - sc * sc
    if abs(det) < 1e-9:
        return None

    a = (ys * cc - yc * sc) / det
    b = (yc * ss - ys * sc) / det
    amplitude = math.hypot(a, b)
    phase = math.atan2(b, a)
    return amplitude, phase


def logspace(start, stop, count):
    if count <= 1:
        return [start]
    log_start = math.log(start)
    log_stop = math.log(stop)
    step = (log_stop - log_start) / (count - 1)
    return [math.exp(log_start + step * i) for i in range(count)]


def wrap_phase_deg(value):
    while value <= -180.0:
        value += 360.0
    while value > 180.0:
        value -= 360.0
    return value


def load_rows(path):
    rows = []
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for raw in reader:
            try:
                row = {
                    "update_count": int(raw["update_count"]),
                    "elapsed_s": float(raw["elapsed_s"]),
                    "segment_index": int(raw["segment_index"]),
                    "working_point_angle_deg": float(raw["working_point_angle_deg"]),
                    "target_angle": float(raw["target_angle"]),
                    "excitation_offset": float(raw["excitation_offset"]),
                    "excitation_velocity": float(raw["excitation_velocity"]),
                    "actual_angle": float(raw["actual_angle"]),
                    "actual_velocity": float(raw["actual_velocity"]),
                    "control_torque": float(raw["control_torque"]),
                    "measured_torque": float(raw["measured_torque"]),
                    "frequency_hz": float(raw["frequency_hz"]),
                    "phase_rad": float(raw["phase_rad"]),
                }
            except (KeyError, ValueError):
                continue

            if not all(is_finite(row[key]) for key in row if key not in {"update_count", "segment_index"}):
                continue

            row["working_point_angle_rad"] = math.radians(row["working_point_angle_deg"])
            row["actual_angle_offset"] = row["actual_angle"] - row["working_point_angle_rad"]
            rows.append(row)

    return rows


def estimate_frf(rows, bin_count, min_samples):
    active_rows = [
        row for row in rows
        if row["frequency_hz"] > 0.0 and abs(row["excitation_offset"]) > 1e-9
    ]
    if not active_rows:
        return []

    min_frequency = min(row["frequency_hz"] for row in active_rows)
    max_frequency = max(row["frequency_hz"] for row in active_rows)
    if min_frequency <= 0.0 or max_frequency <= min_frequency:
        return []

    centers = logspace(min_frequency, max_frequency, bin_count)
    ratio = (max_frequency / min_frequency) ** (1.0 / max(bin_count - 1, 1))
    half_ratio = math.sqrt(ratio)

    frf_rows = []
    for center in centers:
        lower = center / half_ratio
        upper = center * half_ratio
        band_rows = [
            row for row in active_rows
            if lower <= row["frequency_hz"] <= upper
        ]
        if len(band_rows) < min_samples:
            continue

        input_fit = fit_sine_coefficients(band_rows, "excitation_offset")
        output_fit = fit_sine_coefficients(band_rows, "actual_angle_offset")
        if input_fit is None or output_fit is None:
            continue

        input_amplitude, input_phase = input_fit
        output_amplitude, output_phase = output_fit
        if input_amplitude <= 1e-9:
            continue

        gain = output_amplitude / input_amplitude
        gain_db = 20.0 * math.log10(max(gain, 1e-9))
        phase_deg = wrap_phase_deg(math.degrees(output_phase - input_phase))
        frf_rows.append(
            {
                "frequency_hz": center,
                "gain_db": gain_db,
                "phase_deg": phase_deg,
                "input_amplitude": input_amplitude,
                "output_amplitude": output_amplitude,
                "sample_count": len(band_rows),
            }
        )

    return frf_rows


def estimate_metrics(frf_rows):
    if not frf_rows:
        return None

    low_band = frf_rows[: min(3, len(frf_rows))]
    low_gain_db = statistics.mean(row["gain_db"] for row in low_band)
    low_gain_linear = 10.0 ** (low_gain_db / 20.0)
    peak_gain_db = max(row["gain_db"] for row in frf_rows)
    peak_delta_db = peak_gain_db - low_gain_db

    bandwidth_hz = None
    phase_at_bandwidth_deg = None
    for row in frf_rows:
        if row["gain_db"] <= low_gain_db - 3.0:
            bandwidth_hz = row["frequency_hz"]
            phase_at_bandwidth_deg = row["phase_deg"]
            break

    return {
        "low_gain_db": low_gain_db,
        "low_gain_linear": low_gain_linear,
        "peak_gain_db": peak_gain_db,
        "peak_delta_db": peak_delta_db,
        "bandwidth_hz": bandwidth_hz,
        "phase_at_bandwidth_deg": phase_at_bandwidth_deg,
    }


def suggest_tuning(metrics, current_angle_kp, current_velocity_kp, current_velocity_kd, target_bw):
    angle_scale = 1.0
    velocity_scale = 1.0
    suggested_velocity_kd = current_velocity_kd

    if metrics["low_gain_linear"] < 0.95:
        angle_scale = max(angle_scale, min(1.0 / max(metrics["low_gain_linear"], 0.5), 1.5))

    if metrics["bandwidth_hz"] is not None and metrics["bandwidth_hz"] < target_bw:
        angle_scale = max(
            angle_scale,
            min(target_bw / max(metrics["bandwidth_hz"], 0.25), 1.5),
        )

    if metrics["peak_delta_db"] > 6.0:
        velocity_scale *= 0.8
        suggested_velocity_kd = current_velocity_kd + max(0.2, 0.02 * current_velocity_kp)
    elif metrics["peak_delta_db"] > 3.0:
        velocity_scale *= 0.9
        suggested_velocity_kd = current_velocity_kd + max(0.1, 0.01 * current_velocity_kp)
    elif metrics["bandwidth_hz"] is not None and metrics["bandwidth_hz"] < target_bw:
        velocity_scale *= min(target_bw / max(metrics["bandwidth_hz"], 0.25), 1.25)

    if metrics["phase_at_bandwidth_deg"] is not None and metrics["phase_at_bandwidth_deg"] < -135.0:
        velocity_scale *= 0.9
        suggested_velocity_kd = max(suggested_velocity_kd, current_velocity_kd + 0.1)

    return {
        "suggested_angle_kp": current_angle_kp * angle_scale,
        "suggested_velocity_kp": current_velocity_kp * velocity_scale,
        "suggested_velocity_kd": suggested_velocity_kd,
    }


def write_frf_csv(path, frf_rows):
    if not frf_rows:
        return

    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "frequency_hz",
                "gain_db",
                "phase_deg",
                "input_amplitude",
                "output_amplitude",
                "sample_count",
            ],
        )
        writer.writeheader()
        writer.writerows(frf_rows)


def collect_csv_files(input_path):
    if input_path.is_file():
        return [input_path]
    return sorted(
        path
        for path in input_path.glob("*.csv")
        if path.is_file() and not path.name.endswith(".frf.csv")
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", help="CSV file or directory containing CSV files")
    parser.add_argument("--current-angle-kp", type=float, default=12.0)
    parser.add_argument("--current-velocity-kp", type=float, default=25.0)
    parser.add_argument("--current-velocity-kd", type=float, default=0.0)
    parser.add_argument("--target-bandwidth-hz", type=float, default=6.0)
    parser.add_argument("--bin-count", type=int, default=24)
    parser.add_argument("--min-samples-per-bin", type=int, default=40)
    args = parser.parse_args()

    input_path = Path(args.input_path)
    csv_files = collect_csv_files(input_path)
    if not csv_files:
        raise SystemExit("No CSV files found")

    per_file_suggestions = []
    print("file,working_point_deg,low_gain_db,peak_delta_db,bandwidth_hz,suggested_angle_kp,suggested_velocity_kp,suggested_velocity_kd")

    for csv_file in csv_files:
        rows = load_rows(csv_file)
        frf_rows = estimate_frf(rows, args.bin_count, args.min_samples_per_bin)
        if not frf_rows:
            continue

        metrics = estimate_metrics(frf_rows)
        if metrics is None:
            continue

        suggestion = suggest_tuning(
            metrics,
            args.current_angle_kp,
            args.current_velocity_kp,
            args.current_velocity_kd,
            args.target_bandwidth_hz,
        )
        per_file_suggestions.append(suggestion)

        frf_path = csv_file.with_suffix(".frf.csv")
        write_frf_csv(frf_path, frf_rows)

        working_point_deg = rows[0]["working_point_angle_deg"] if rows else math.nan
        bandwidth = metrics["bandwidth_hz"]
        bandwidth_text = f"{bandwidth:.3f}" if bandwidth is not None else "nan"
        print(
            f"{csv_file.name},{working_point_deg:.3f},{metrics['low_gain_db']:.3f},"
            f"{metrics['peak_delta_db']:.3f},{bandwidth_text},"
            f"{suggestion['suggested_angle_kp']:.3f},"
            f"{suggestion['suggested_velocity_kp']:.3f},"
            f"{suggestion['suggested_velocity_kd']:.3f}"
        )

    if not per_file_suggestions:
        raise SystemExit("No valid FRF could be estimated from the provided CSV data")

    overall_angle_kp = statistics.median(
        item["suggested_angle_kp"] for item in per_file_suggestions)
    overall_velocity_kp = min(
        item["suggested_velocity_kp"] for item in per_file_suggestions)
    overall_velocity_kd = max(
        item["suggested_velocity_kd"] for item in per_file_suggestions)

    print("\nSuggested YAML snippet:")
    print(f"angle_kp: {overall_angle_kp:.3f}")
    print(f"velocity_kp: {overall_velocity_kp:.3f}")
    print(f"velocity_kd: {overall_velocity_kd:.3f}")


if __name__ == "__main__":
    main()
