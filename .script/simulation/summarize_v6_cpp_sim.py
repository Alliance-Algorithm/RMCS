#!/usr/bin/env python3
"""Plot a production-C++ V6 Isaac run without changing its original reports.

Usage: python .script/simulation/summarize_v6_cpp_sim.py RUN_DIRECTORY
Cached inference/PD duration samples are not counted as separate invocations.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


STATES = {0: "INIT", 1: "IDLE", 2: "PREPARE", 3: "RL"}
STATE_COLORS = {0: "#dddddd", 1: "#f3c8bd", 2: "#f7df9a", 3: "#c7e2d2"}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def statistics(values):
    values = np.asarray([float(v) for v in values if finite(v)], dtype=float)
    if not values.size:
        return {"count": 0, "min": None, "mean": None, "median": None,
                "p95": None, "p99": None, "max": None}
    return {"count": int(values.size), "min": float(values.min()), "mean": float(values.mean()),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(values.max())}


def source_horizons():
    """Read the sibling runner's literal case table without importing Isaac."""
    path = Path(__file__).with_name("play_v6_cpp_isaac.py")
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == "CASES" for target in node.targets):
            cases = ast.literal_eval(node.value)
            return {name: float(values[0]) for name, values in cases.items()}
    raise ValueError(f"No literal CASES table in {path}")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scalar(row, field, index=None):
    value = row.get(field)
    if index is not None:
        value = value[index] if isinstance(value, list) and len(value) > index else None
    return float(value) if finite(value) else np.nan


def load_cases(root, report):
    entries = report.get("cases", [])
    if not isinstance(entries, list):
        raise ValueError("report.cases must be a list")
    declared = {entry["case"]: entry for entry in entries
                if isinstance(entry, dict) and isinstance(entry.get("case"), str)}
    names = list(declared)
    for name in report.get("requested_cases", []):
        if isinstance(name, str) and name not in names:
            names.append(name)
    traces, errors = {}, {}
    for path in sorted(root.glob("*.json")):
        if path.name in ("report.json", "summary.json"):
            continue
        try:
            value = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            if path.stem in names:
                errors[path.stem] = str(error)
            continue
        if isinstance(value, list) and (not value or all(isinstance(row, dict) for row in value)):
            if value and not any("time_s" in row for row in value) and path.stem not in names:
                continue
            traces[path.stem] = value
            if path.stem not in names:
                names.append(path.stem)
    return names, declared, traces, errors


def summarize_case(name, entry, rows, horizon, report, error=None):
    warnings = []
    times = [row.get("time_s") for row in rows]
    valid_times = all(finite(value) for value in times)
    monotonic = valid_times and all(b > a for a, b in zip(times, times[1:]))
    invalid_signals = sum(not all(np.isfinite(scalar(row, field, index)) for field, index in (
        ("velocity", 0), ("velocity", 2), ("reference", 0), ("reference", 2),
        ("height_m", None), ("height_reference_m", None))) or row.get("state") not in STATES
        for row in rows)
    if rows and not monotonic:
        warnings.append("Nonfinite or nonincreasing simulation timestamps")
    if error:
        warnings.append("Trace read error: " + error)
    if invalid_signals:
        warnings.append(f"Missing or invalid plotted signals/state in {invalid_signals} trace rows")
    actual = float(times[-1]) if times and finite(times[-1]) else None
    dt = report.get("physics_dt_s", .005)
    dt = float(dt) if finite(dt) and dt > 0 else .005
    cadence_valid = bool(valid_times and (not rows or (
        abs(times[0] - dt) <= dt * 1e-4 and
        all(abs((b - a) - dt) <= dt * 1e-4 for a, b in zip(times, times[1:])))))
    if rows and not cadence_valid:
        warnings.append("Trace timestamps do not contain every declared physics step from the first step")
    fault_rows = [row for row in rows if row.get("fault") is True]
    mechanism_failures = sum(row.get("mechanism_valid") is False for row in rows)
    reported_complete = entry.get("completed") is True
    horizon_reached = horizon is not None and actual is not None and actual >= horizon - dt / 2
    complete = bool(rows and monotonic and cadence_valid and reported_complete and horizon_reached
                    and not error and not invalid_signals)
    if horizon is None:
        warnings.append("Unknown case horizon; completion cannot be independently verified")
    if entry.get("samples") is not None and entry["samples"] != len(rows):
        warnings.append("Report sample count differs from trace rows")
        complete = False
    failed_checks = [key for key, value in entry.get("checks", {}).items() if value is not True]
    observed_failure = bool(fault_rows or mechanism_failures)
    source_pass = entry.get("passed")
    if error or invalid_signals or (rows and (not monotonic or not cadence_valid)):
        outcome, reason = "INVALID", "invalid_trace"
    elif not rows:
        outcome, reason = "MISSING", "no_trace_samples"
    elif horizon is None:
        outcome, reason = "UNASSESSED", "unknown_case_horizon"
    elif not complete:
        outcome = "INCOMPLETE"
        reason = "controller_fault" if fault_rows else "mechanism_failure" if mechanism_failures else "early_stop_or_short_run"
    elif observed_failure or source_pass is False or failed_checks:
        outcome, reason = "FAIL", "observed_failure" if observed_failure else "reported_checks_failed"
    elif source_pass is True and entry.get("checks"):
        outcome, reason = "PASS", "complete_reported_checks_passed"
    else:
        outcome, reason = "UNASSESSED", "no_complete_scoring_evidence"
    if source_pass is True and outcome != "PASS":
        warnings.append("Source reported passed=true; independent completion/evidence checks do not confirm it")
    wall = [row.get("wall_s") for row in rows]
    wall_end = float(wall[-1]) if wall and finite(wall[-1]) else None
    wall_delta = [1000 * (b - a) for a, b in zip(wall, wall[1:])
                  if finite(a) and finite(b) and b >= a]
    direct_intervals = [row.get("wall_sample_interval_ms") for row in rows
                        if finite(row.get("wall_sample_interval_ms")) and row["wall_sample_interval_ms"] > 0]
    intervals = direct_intervals or wall_delta
    interval_source = "bridge_wall_sample_interval_ms" if direct_intervals else "estimated_from_trace_wall_s_differences"
    sample_stamps = [row.get("sensor_steady_ns") for row in rows if finite(row.get("sensor_steady_ns"))]
    sequences = [row.get("sensor_sequence") for row in rows if finite(row.get("sensor_sequence"))]
    inference = [row.get("inference_us") for row in rows
                 if finite(row.get("inference_us")) and row["inference_us"] > 0]
    pd = [row.get("pd_us") for row in rows if finite(row.get("pd_us")) and row["pd_us"] > 0]
    state_counts = Counter(STATES.get(row.get("state"), "UNKNOWN") for row in rows)
    transitions = []
    previous = object()
    for row in rows:
        if row.get("state") != previous:
            previous = row.get("state")
            transitions.append({"time_s": row.get("time_s"), "state": previous,
                                "name": STATES.get(previous, "UNKNOWN")})
    return dict(case=name, outcome=outcome, passed=outcome == "PASS", reason=reason,
        reported_passed=source_pass, reported_completed=reported_complete, completed=complete,
        expected_simulated_seconds=horizon, observed_simulated_seconds=actual,
        trace_samples=len(rows), source_report=entry, failed_checks=failed_checks,
        first_fault=fault_rows[0] if fault_rows else None, mechanism_failure_samples=mechanism_failures,
        state_sample_counts=dict(state_counts), state_transitions=transitions, warnings=warnings,
        timing=dict(wall_seconds=wall_end,
            simulation_to_wall_ratio=actual / wall_end if actual is not None and wall_end and wall_end > 0 else None,
            simulation_sample_interval_ms=statistics([1000 * (b - a) for a, b in zip(times, times[1:])
                                                     if finite(a) and finite(b)]),
            wall_sample_interval_ms=statistics(intervals), wall_sample_interval_source=interval_source,
            wall_sample_intervals_over_20ms=sum(value > 20 for value in intervals),
            bridge_step_duration_ms=statistics([row.get("wall_step_duration_ms") for row in rows]),
            distinct_sensor_timestamps=len(set(sample_stamps)) if sample_stamps else None,
            distinct_sensor_sequences=len(set(sequences)) if sequences else None,
            inference_us=statistics(inference), pd_us=statistics(pd), inference_invocations=None,
            duration_sample_scope="Cached last-call durations sampled per sensor frame; not invocation counts"),
        sensors=dict(issue_sample_counts=dict(Counter(str(row.get("sensor_issue")) for row in rows)),
            invalid_mask_samples=sum(finite(row.get("sensor_mask")) and row["sensor_mask"] != 0 for row in rows),
            invalid_status_samples=sum(row.get("sensors_valid") is False for row in rows),
            motor_age_ms=statistics([row.get("motor_age_ms") for row in rows]),
            imu_age_ms=statistics([row.get("imu_age_ms") for row in rows]),
            acceleration_age_ms=statistics([row.get("acceleration_age_ms") for row in rows])))


def plot_cases(destination, results, traces, dpi):
    count = max(len(results), 1)
    figure, axes = plt.subplots(count, 4, figsize=(16, max(3.2 * count, 4.)), squeeze=False,
                               layout="constrained")
    if not results:
        for axis in axes.flat:
            axis.set_axis_off()
        axes[0, 0].text(.05, .5, "NO CASE DATA\nRun is incomplete or has no saved traces.", transform=axes[0, 0].transAxes)
    for row_index, result in enumerate(results):
        name, outcome = result["case"], result["outcome"]
        rows = traces.get(name, [])
        times = np.asarray([scalar(row, "time_s") for row in rows])
        actual = result["observed_simulated_seconds"] or 0.
        expected = result["expected_simulated_seconds"]
        end = max(actual, expected or actual, .005)
        label = name.encode("ascii", "backslashreplace").decode()
        spans = []
        if rows:
            start = rows[0].get("time_s")
            state = rows[0].get("state")
            for row in rows[1:]:
                if row.get("state") != state:
                    spans.append((start, row.get("time_s"), state))
                    start, state = row.get("time_s"), row.get("state")
            spans.append((start, rows[-1].get("time_s"), state))
        for column, (field, index, reference, ref_index, title, unit) in enumerate((
                ("velocity", 0, "reference", 0, "Forward velocity", "m/s"),
                ("velocity", 2, "reference", 2, "Yaw rate", "rad/s"),
                ("height_m", None, "height_reference_m", None, "Base height", "m"))):
            axis = axes[row_index, column]
            if rows:
                axis.plot(times, [scalar(row, field, index) for row in rows], color="#2874a6", label="Measured", linewidth=1.2)
                axis.step(times, [scalar(row, reference, ref_index) for row in rows], where="post",
                          color="#d46d27", label="C++ reference", linestyle="--", linewidth=1.1)
            axis.set(title=title if row_index == 0 else None, xlabel="Simulation time / s", ylabel=unit, xlim=(0., end))
            axis.grid(alpha=.22)
            for start, stop, state in spans:
                if finite(start) and finite(stop) and stop > start:
                    axis.axvspan(start, stop, color=STATE_COLORS.get(state, "#e6c8ef"), alpha=.22, linewidth=0)
            if expected is not None and actual < expected:
                axis.axvspan(actual, expected, color="#c0c0c0", alpha=.18, hatch="//", linewidth=0)
            if outcome != "PASS" and rows:
                axis.axvline(actual, color="#b03a2e", linewidth=1., linestyle=":")
            if row_index == 0:
                axis.legend(fontsize=8, loc="best")
        state_axis = axes[row_index, 3]
        if rows:
            state_axis.step(times, [scalar(row, "state") for row in rows], where="post", color="#444444", label="State")
        state_axis.set(yticks=list(STATES), yticklabels=list(STATES.values()), ylim=(-.3, 3.4),
                       xlabel="Simulation time / s", xlim=(0., end), title="Controller state" if row_index == 0 else None)
        state_axis.grid(alpha=.22)
        color = "#23764a" if outcome == "PASS" else "#b03a2e"
        state_axis.text(.02, .97, f"{label}: {outcome}\nObserved {actual:.3f}s" +
                        (f" / {expected:g}s" if expected is not None else " / unknown horizon"),
                        transform=state_axis.transAxes, va="top", fontsize=9, color=color,
                        bbox=dict(facecolor="white", edgecolor="none", alpha=.9, pad=1.))
        if result["first_fault"]:
            stamp = result["first_fault"].get("time_s")
            if finite(stamp):
                state_axis.axvline(stamp, color="#b03a2e", linestyle="--", linewidth=1.)
        if not rows:
            state_axis.text(.5, .45, "NO TRACE DATA", transform=state_axis.transAxes, ha="center", color=color)
    figure.suptitle("Production RMCS C++ | V6 free-base Isaac | shaded states, hatched unobserved time", fontsize=13)
    figure.savefig(destination / "trajectory.png", dpi=dpi)
    plt.close(figure)


def write_markdown(destination, summary):
    lines = ["# V6 C++ dynamic simulation summary", "", f"Source run status: `{summary['source_status']}`. "
             f"Confirmed passes: {summary['passed_cases']}/{summary['case_count']}.", "",
             "![Trajectories](trajectory.png)", "",
             "| Case | Outcome | Observed / expected sim s | Wall s | Wall sample p95 ms | Last inference p95 us |",
             "| --- | --- | ---: | ---: | ---: | ---: |"]
    def fmt(value):
        return f"{value:.3f}" if finite(value) else "n/a"
    for case in summary["cases"]:
        timing = case["timing"]
        lines.append(f"| {case['case']} | {case['outcome']} | {fmt(case['observed_simulated_seconds'])} / "
                     f"{fmt(case['expected_simulated_seconds'])} | {fmt(timing['wall_seconds'])} | "
                     f"{fmt(timing['wall_sample_interval_ms']['p95'])} | {fmt(timing['inference_us']['p95'])} |")
    lines += ["", "Inference and PD timings are cached last-call durations sampled in trace rows; "
              "their row counts are not invocation counts. Wall intervals use bridge values when saved, "
              "otherwise explicitly labeled estimates from wall_s differences.", "",
              "Incomplete, missing, unknown-horizon and invalid cases cannot pass. The original report and "
              "case JSON are unchanged. This nominal simulation does not qualify hardware, CAN/USB or BMI088 EKF.", ""]
    for case in summary["cases"]:
        if case["outcome"] != "PASS" or case["warnings"]:
            detail = "; ".join([case["reason"], *case["warnings"]])
            lines.append(f"- `{case['case']}`: {detail}")
    (destination / "summary.md").write_text("\n".join(lines).rstrip() + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Output directory containing report.json and case JSON traces")
    parser.add_argument("--output", type=Path, help="Summary destination; defaults to the run directory")
    parser.add_argument("--dpi", type=int, default=140)
    parser.add_argument("--no-markdown", action="store_true")
    args = parser.parse_args()
    if not 40 <= args.dpi <= 300:
        parser.error("dpi must be in [40,300]")
    root = args.run.resolve()
    destination = (args.output or root).resolve()
    report_file = root / "report.json"
    report = json.loads(report_file.read_text())
    horizons = source_horizons()
    names, declared, traces, errors = load_cases(root, report)
    results = []
    for name in names:
        entry = declared.get(name, {})
        recorded_horizon = entry.get("expected_duration_s")
        use_recorded = finite(recorded_horizon) and recorded_horizon > 0
        horizon = float(recorded_horizon) if use_recorded else horizons.get(name)
        result = summarize_case(name, entry, traces.get(name, []), horizon, report, errors.get(name))
        result["horizon_source"] = "source_report.expected_duration_s" if use_recorded else "current_runner.CASES"
        results.append(result)
    summary = dict(schema="rmcs-v6-cpp-isaac-summary-v1", generated_at=datetime.now(timezone.utc).isoformat(),
        source_directory=str(root), source_report_sha256=digest(report_file),
        summarizer_sha256=digest(Path(__file__)), source_status=report.get("status", "unknown"),
        source_error=report.get("error"), policy_sha256=report.get("policy_sha256"),
        contract_sha256=report.get("contract_sha256"), asset_manifest_sha256=report.get("asset_manifest_sha256"),
        timing_scope=report.get("timing_scope"), sensor_source=report.get("sensor_source"),
        physics_dt_s=report.get("physics_dt_s"), executor_rate_hz=report.get("executor_rate_hz"),
        policy_rate_hz=report.get("policy_rate_hz"), pd_rate_hz=report.get("pd_rate_hz"),
        case_count=len(results), passed_cases=sum(case["passed"] for case in results),
        complete=bool(results) and report.get("status") == "complete" and all(case["completed"] for case in results),
        hardware_ready=False, domain_qualified=None, cases=results,
        source_trace_sha256={name: digest(root / f"{name}.json") for name in traces},
        artifacts=dict(trajectory="trajectory.png", summary="summary.json"))
    summary["passed"] = summary["complete"] and summary["passed_cases"] == summary["case_count"]
    destination.mkdir(parents=True, exist_ok=True)
    plot_cases(destination, results, traces, args.dpi)
    (destination / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    if not args.no_markdown:
        write_markdown(destination, summary)
    print(json.dumps({"source_status": summary["source_status"], "complete": summary["complete"],
                      "passed_cases": summary["passed_cases"], "case_count": summary["case_count"],
                      "trajectory": str(destination / "trajectory.png"),
                      "summary": str(destination / "summary.json")}, indent=2))


if __name__ == "__main__":
    main()
