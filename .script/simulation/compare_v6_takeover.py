#!/usr/bin/env python3
"""Compare direct, 100 ms and 200 ms production-C++ V6 upright takeover runs.

Usage: python .script/simulation/compare_v6_takeover.py DIRECT RUN_100MS RUN_200MS
       --output COMPARISON_DIRECTORY

Only episode-reset engineering endpoint stress is compared. No full fallen-pose
recovery trajectory or hardware acceptance is inferred. Original runs are read
only; comparison JSON and a plot are written separately.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


VARIANTS = (("direct", 0., "#2874a6"), ("100ms", .1, "#d46d27"),
            ("200ms", .2, "#23764a"))
METRICS = ("tilt_peak_first_2s_deg", "planar_speed_peak_first_2s_m_s",
           "entry_effort_jump_max_nm", "effort_step_peak_first_2s_nm")
IDENTITY_FIELDS = ("schema", "controller", "policy_sha256", "contract_sha256",
                   "asset_manifest_sha256", "physics_dt_s", "executor_rate_hz",
                   "policy_rate_hz", "pd_rate_hz", "sensor_source", "reset_jitter",
                   "timing_scope", "fixed_base", "external_guide",
                   "state_writes_between_resets", "initial_profiles",
                   "initial_profile_scope", "requested_cases", "policy_joint_ids")
BRIDGE_FIELDS = ("protocol", "simulation_only", "profile_path", "model_path",
                 "model_sha256", "policy_profile", "axis_order", "executor_hz",
                 "physics_hz", "inference_hz", "pd_hz",
                 "v6_takeover_blend_supported")
REQUIRED_SOURCES = (
    ".script/simulation/play_v6_cpp_isaac.py",
    ".script/simulation/v6_component_sim_bridge.cpp",
    "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml",
    "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg_chassis_controller.cpp",
    "rmcs_ws/src/rmcs_rl/src/rl_controller.cpp",
    "rmcs_ws/src/rmcs_rl/src/rl_controller.hpp",
    "rmcs_ws/src/rmcs_rl/src/controller_recovery.cpp",
    "rmcs_ws/src/rmcs_rl/src/observation.cpp",
    "rmcs_ws/src/rmcs_rl/src/action.cpp",
    "rmcs_ws/src/rmcs_rl/src/configuration.cpp",
    "rmcs_ws/src/rmcs_rl/src/policy.hpp",
)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_value(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def vector(value, length):
    return isinstance(value, list) and len(value) == length and all(finite(v) for v in value)


def close(a, b):
    return (a is None and b is None) or (finite(a) and finite(b) and
        math.isclose(a, b, rel_tol=1e-7, abs_tol=1e-7))


def read_json(path):
    # JSON's permissive NaN/Infinity extensions are not valid evidence.
    def invalid_constant(value):
        raise ValueError("Nonfinite JSON constant: " + value)
    return json.loads(path.read_text(), parse_constant=invalid_constant)


def bridge_configuration(report):
    bridge = report.get("bridge", {})
    if not isinstance(bridge, dict):
        return None
    result = {key: bridge.get(key) for key in BRIDGE_FIELDS}
    parameters = deepcopy(bridge.get("parameters"))
    if isinstance(parameters, dict) and isinstance(parameters.get("rl_controller"), dict):
        parameters["rl_controller"].pop("v6_takeover_blend_seconds", None)
    result["parameters_except_blend"] = parameters
    return result


def compare_metadata(runs):
    errors = []
    baseline = runs[0]["report"]
    for run in runs:
        label, report = run["label"], run["report"]
        for key in IDENTITY_FIELDS:
            if key not in report:
                errors.append(f"{label}: missing report.{key}")
            elif key in baseline and report[key] != baseline[key]:
                errors.append(f"{label}: report.{key} differs from direct")
        for key in ("policy_sha256", "contract_sha256", "asset_manifest_sha256"):
            if not sha256_value(report.get(key)):
                errors.append(f"{label}: invalid report.{key}")
        sources = report.get("source_sha256")
        if not isinstance(sources, dict) or not sources:
            errors.append(f"{label}: missing recorded source_sha256")
        else:
            for path in REQUIRED_SOURCES:
                if not sha256_value(sources.get(path)):
                    errors.append(f"{label}: missing or invalid source hash: {path}")
            if sources != baseline.get("source_sha256"):
                errors.append(f"{label}: recorded source_sha256 differs from direct")
        for key, expected in (("physics_dt_s", .005), ("executor_rate_hz", 1000),
                              ("policy_rate_hz", 50), ("pd_rate_hz", 200)):
            if not close(report.get(key), expected):
                errors.append(f"{label}: unexpected report.{key}")
        if report.get("fixed_base") is not False or report.get("external_guide") is not False:
            errors.append(f"{label}: free-base/unguided configuration is not confirmed")
        if report.get("state_writes_between_resets") != 0:
            errors.append(f"{label}: reset-only state writes are not confirmed")
        profiles = report.get("initial_profiles")
        if not isinstance(profiles, dict) or not profiles:
            errors.append(f"{label}: no declared initial profiles")
        if not isinstance(report.get("initial_profile_scope"), str) or not report["initial_profile_scope"]:
            errors.append(f"{label}: missing engineering endpoint scope")
        if not isinstance(report.get("requested_cases"), list) or not report["requested_cases"]:
            errors.append(f"{label}: no requested cases")
        bridge = report.get("bridge", {})
        if not isinstance(bridge, dict):
            errors.append(f"{label}: bridge metadata is not an object")
            continue
        for key in BRIDGE_FIELDS:
            if key not in bridge:
                errors.append(f"{label}: missing bridge.{key}")
        if bridge.get("model_sha256") != report.get("policy_sha256"):
            errors.append(f"{label}: bridge model hash does not match report policy")
        if bridge.get("simulation_only") is not True:
            errors.append(f"{label}: simulation-only bridge is not confirmed")
        if bridge.get("v6_takeover_blend_supported") is not True:
            errors.append(f"{label}: public takeover blend diagnostic is unavailable")
        parameters = bridge.get("parameters", {})
        controller = parameters.get("rl_controller", {}) if isinstance(parameters, dict) else {}
        duration = controller.get("v6_takeover_blend_seconds") if isinstance(controller, dict) else None
        run["observed_blend_seconds"] = duration
        if not close(duration, run["expected_blend_seconds"]):
            errors.append(f"{label}: expected blend {run['expected_blend_seconds']:g} s, got {duration!r}")
        if bridge_configuration(report) != bridge_configuration(baseline):
            errors.append(f"{label}: bridge configuration differs beyond the blend duration")
    return errors


def recompute_takeover(rows):
    early = [row for row in rows if finite(row.get("time_s")) and row["time_s"] <= 2.]
    entry = next((i for i, row in enumerate(rows) if row.get("state") == 3), None)
    valid_early = bool(early) and all(finite(row.get("tilt_deg")) and
        vector(row.get("velocity"), 3) and vector(row.get("torque_api"), 6) for row in early)
    values = {key: None for key in METRICS}
    if valid_early:
        values[METRICS[0]] = max(row["tilt_deg"] for row in early)
        values[METRICS[1]] = max(math.hypot(*row["velocity"][:2]) for row in early)
        values[METRICS[3]] = max((float(np.max(np.abs(np.array(b["torque_api"]) -
            np.array(a["torque_api"])))) for a, b in zip(early, early[1:])), default=None)
    if entry is not None and entry > 0 and all(vector(rows[i].get("torque_api"), 6)
                                              for i in (entry - 1, entry)):
        values[METRICS[2]] = float(np.max(np.abs(np.array(rows[entry]["torque_api"]) -
                                                   np.array(rows[entry - 1]["torque_api"]))))
    return values, entry


def analyze_case(run, key, entry):
    errors = []
    root = run["root"]
    path = root / (key + ".json")
    rows = []
    if Path(key).name != key or key in (".", ".."):
        errors.append("Unsafe case trace filename")
    else:
        try:
            rows = read_json(path)
            if not isinstance(rows, list) or not all(isinstance(row, dict) for row in rows):
                rows = []
                errors.append("Trace must be a list of objects")
        except (OSError, ValueError) as error:
            errors.append("Cannot read trace: " + str(error))
    if not entry:
        errors.append("Case has no source report entry")
    times = [row.get("time_s") for row in rows]
    dt = run["report"].get("physics_dt_s")
    cadence_valid = bool(rows) and finite(dt) and dt > 0 and all(finite(t) for t in times)
    if cadence_valid:
        cadence_valid = abs(times[0] - dt) <= dt * 1e-4 and all(
            abs(b - a - dt) <= dt * 1e-4 for a, b in zip(times, times[1:]))
    if rows and not cadence_valid:
        errors.append("Trace does not contain every declared physics tick")
    signals_valid = bool(rows) and all(finite(row.get("tilt_deg")) and
        vector(row.get("velocity"), 3) and vector(row.get("torque_api"), 6) and
        row.get("state") in (0, 1, 2, 3) for row in rows)
    if rows and not signals_valid:
        errors.append("Missing/nonfinite tilt, velocity, six-axis effort or controller state")
    if entry.get("samples") != len(rows):
        errors.append("Source sample count differs from trace")
    observed = float(times[-1]) if times and finite(times[-1]) else None
    expected, requested = entry.get("expected_duration_s"), entry.get("requested_duration_s")
    complete = bool(cadence_valid and signals_valid and finite(expected) and expected > 0 and
        observed >= expected - dt / 2 and entry.get("completed") is True and not errors)
    requested_complete = bool(cadence_valid and finite(requested) and requested > 0 and
        observed >= requested - dt / 2)
    metrics, entry_index = recompute_takeover(rows)
    reported_metrics = entry.get("takeover", {})
    if not isinstance(reported_metrics, dict):
        reported_metrics = {}
    metric_agreement = {name: name in reported_metrics and close(reported_metrics.get(name), value)
                        for name, value in metrics.items()}
    if not all(metric_agreement.values()):
        errors.append("Source takeover metrics differ from trace or are missing")
    rl_entry = rows[entry_index]["time_s"] if entry_index is not None else None
    if not close(entry.get("rl_entered_at_s"), rl_entry):
        errors.append("Source RL entry timestamp differs from trace")
    faults = [row for row in rows if row.get("fault") is True or
              row.get("mechanism_valid") is False or bool(row.get("physics_failure"))]
    checks = entry.get("checks", {})
    checks_valid = isinstance(checks, dict) and bool(checks) and all(value is True for value in checks.values())
    active_after_entry = rows[entry_index:] if entry_index is not None else []
    if entry.get("command_case") == "disable":
        active_after_entry = [row for row in active_after_entry if row["time_s"] < 5.005 - 1e-9]
    sustained = bool(active_after_entry) and all(row.get("state") == 3 for row in active_after_entry)
    if errors:
        outcome = "INVALID" if rows else "MISSING"
    elif not complete:
        outcome = "INCOMPLETE"
    elif faults or entry.get("passed") is not True or not checks_valid or not sustained:
        outcome = "FAIL"
    else:
        outcome = "PASS"
    result = dict(case=key, command_case=entry.get("command_case"),
        initial_profile=entry.get("initial_profile"), outcome=outcome,
        source_passed=entry.get("passed"), source_completed=entry.get("completed"),
        complete=complete, requested_window_complete=requested_complete,
        expected_duration_s=expected, requested_duration_s=requested,
        observed_duration_s=observed, trace_samples=len(rows), cadence_valid=cadence_valid,
        signals_valid=signals_valid, first_2s_observed=bool(cadence_valid and observed >= 2. - dt / 2),
        rl_entered_at_s=rl_entry, source_rl_entered_at_s=entry.get("rl_entered_at_s"),
        sustained_rl_after_entry=sustained, end_state=rows[-1].get("state") if rows else None,
        end_effort_api_nm=rows[-1].get("torque_api") if rows else None,
        steady_samples=entry.get("steady_samples"), source_steady_checks=checks,
        source_steady_checks_all_pass=checks_valid, reported_takeover=reported_metrics,
        recomputed_takeover=metrics, takeover_metric_agreement=metric_agreement,
        observed_failure_samples=len(faults), first_failure_time_s=faults[0].get("time_s") if faults else None,
        errors=errors, trace_sha256=digest(path) if path.is_file() and Path(key).name == key else None)
    return result, rows


def plot(destination, runs, profiles, command_case, window, valid, dpi):
    figure, axes = plt.subplots(max(1, len(profiles)), 3,
        figsize=(14.5, max(3.1 * len(profiles), 3.7)), squeeze=False, layout="constrained")
    for row_index, profile in enumerate(profiles):
        profile_label = profile.encode("ascii", "backslashreplace").decode()
        for run in runs:
            matches = [(key, result) for key, result in run["cases"].items()
                       if result["initial_profile"] == profile and result["command_case"] == command_case]
            if len(matches) != 1:
                axes[row_index, 0].text(.02, .92 - .10 * run["index"],
                    f"{run['label']}: MISSING/AMBIGUOUS", transform=axes[row_index, 0].transAxes,
                    color=run["color"], fontsize=8)
                continue
            key, result = matches[0]
            rows = [row for row in run["traces"].get(key, []) if finite(row.get("time_s")) and
                    row["time_s"] <= window]
            times = [row["time_s"] for row in rows]
            for column, field, index in ((0, "tilt_deg", None), (1, "torque_api", 0), (2, "torque_api", 4)):
                values = []
                for row in rows:
                    value = row.get(field)
                    if index is not None:
                        value = value[index] if isinstance(value, list) and len(value) > index else None
                    values.append(float(value) if finite(value) else np.nan)
                axis = axes[row_index, column]
                axis.plot(times, values, label=run["label"], color=run["color"], linewidth=1.25)
                rl_entry = result["rl_entered_at_s"]
                if finite(rl_entry) and rl_entry <= window:
                    axis.axvline(rl_entry, color=run["color"], alpha=.5, linestyle=":", linewidth=.8)
                actual = result["observed_duration_s"]
                if finite(actual) and actual < window:
                    axis.axvline(actual, color=run["color"], linestyle="--", linewidth=.9)
            text = f"{run['label']}: {result['outcome']} | RL " + (
                f"{result['rl_entered_at_s']:.3f}s" if finite(result["rl_entered_at_s"]) else "NOT ENTERED")
            axes[row_index, 0].text(.02, .96 - .10 * run["index"], text,
                transform=axes[row_index, 0].transAxes, va="top", color=run["color"], fontsize=8,
                bbox=dict(facecolor="white", edgecolor="none", alpha=.75, pad=.5))
        for column, (title, unit) in enumerate((("Actual tilt", "deg"), ("Left hip API effort", "Nm"),
                                               ("Left wheel API effort", "Nm"))):
            axis = axes[row_index, column]
            axis.set(title=title if row_index == 0 else None, xlabel="Simulation time / s",
                     ylabel=(profile_label + "\n" if column == 0 else "") + unit, xlim=(0., window))
            axis.grid(alpha=.22)
            if row_index == 0:
                handles, labels = axis.get_legend_handles_labels()
                if handles:
                    axis.legend(handles, labels, fontsize=9, loc="best")
    figure.suptitle(("INVALID comparison | " if not valid else "") +
        f"V6 upright endpoint takeover | {command_case} | full recovery NOT exercised\n"
        "Dotted lines: RL entry; dashed lines: early trace end", fontsize=12)
    figure.savefig(destination / "takeover_comparison.png", dpi=dpi)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("direct", type=Path, help="Run with v6_takeover_blend_seconds=0")
    parser.add_argument("blend100", type=Path, help="Run with v6_takeover_blend_seconds=0.1")
    parser.add_argument("blend200", type=Path, help="Run with v6_takeover_blend_seconds=0.2")
    parser.add_argument("--output", type=Path, help="Separate comparison directory; default: sibling takeover_comparison")
    parser.add_argument("--case", help="Command case to plot; default stand, otherwise first recorded command case")
    parser.add_argument("--window-seconds", type=float, default=2., help="Plot first 1 to 2 seconds; default 2")
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()
    if not 1 <= args.window_seconds <= 2:
        parser.error("window-seconds must be in [1,2]")
    if not 40 <= args.dpi <= 300:
        parser.error("dpi must be in [40,300]")
    roots = [path.resolve() for path in (args.direct, args.blend100, args.blend200)]
    if len(set(roots)) != 3:
        parser.error("Three distinct run directories are required")
    destination = (args.output or roots[0].parent / "takeover_comparison").resolve()
    if destination in roots:
        parser.error("output must be separate from all source run directories")
    runs, input_errors = [], []
    for index, (root, (label, duration, color)) in enumerate(zip(roots, VARIANTS)):
        path = root / "report.json"
        try:
            report = read_json(path)
            if not isinstance(report, dict):
                raise ValueError("report must be an object")
        except (OSError, ValueError) as error:
            report = {}
            input_errors.append(f"{label}: cannot read report: {error}")
        runs.append(dict(index=index, label=label, root=root, color=color,
            expected_blend_seconds=duration, report=report, cases={}, traces={},
            source_report_sha256=digest(path) if path.is_file() else None))
    metadata_errors = input_errors + compare_metadata(runs)
    requested = runs[0]["report"].get("requested_cases", [])
    if not isinstance(requested, list):
        requested = []
    keys = list(dict.fromkeys(key for key in requested if isinstance(key, str)))
    for run in runs:
        entries = run["report"].get("cases", [])
        if not isinstance(entries, list):
            entries = []
            metadata_errors.append(f"{run['label']}: report.cases is not a list")
        declared = {}
        for entry in entries:
            if not isinstance(entry, dict) or not isinstance(entry.get("case"), str):
                metadata_errors.append(f"{run['label']}: invalid case entry")
                continue
            key = entry["case"]
            if key in declared:
                metadata_errors.append(f"{run['label']}: duplicate case {key}")
            declared[key] = entry
            if key not in keys:
                keys.append(key)
        run["declared"] = declared
    for run in runs:
        for key in keys:
            result, rows = analyze_case(run, key, run["declared"].get(key, {}))
            run["cases"][key], run["traces"][key] = result, rows
    for key in keys:
        first = runs[0]["cases"][key]
        for run in runs[1:]:
            other = run["cases"][key]
            for field in ("command_case", "initial_profile", "expected_duration_s", "requested_duration_s"):
                if other[field] != first[field]:
                    metadata_errors.append(f"{run['label']}: {key}.{field} differs from direct")
    profiles = runs[0]["report"].get("initial_profiles", {})
    profiles = list(profiles) if isinstance(profiles, dict) else []
    command_cases = list(dict.fromkeys(result["command_case"] for run in runs for result in run["cases"].values()
                                     if isinstance(result["command_case"], str)))
    command_case = args.case or ("stand" if "stand" in command_cases else command_cases[0] if command_cases else "stand")
    if command_case not in command_cases:
        metadata_errors.append("Requested plot command case has no recorded data: " + command_case)
    if not profiles:
        profiles = ["NO_INITIAL_PROFILE_DATA"]
    for run in runs:
        for profile in profiles:
            matches = [result for result in run["cases"].values()
                if result["initial_profile"] == profile and result["command_case"] == command_case]
            if len(matches) != 1:
                metadata_errors.append(f"{run['label']}: expected one {command_case}/{profile} plot trace")
    trace_errors = [f"{run['label']}/{key}: {error}" for run in runs for key, result in run["cases"].items()
                    for error in result["errors"]]
    valid = not metadata_errors and not trace_errors
    output = dict(schema="rmcs-v6-upright-takeover-comparison-v1",
        generated_at=datetime.now(timezone.utc).isoformat(), comparison_valid=valid,
        all_runs_complete=bool(keys) and all(run["report"].get("status") == "complete" and
            all(result["complete"] for result in run["cases"].values()) for run in runs),
        all_runs_passed=valid and bool(keys) and all(run["report"].get("status") == "complete" and
            all(result["outcome"] == "PASS" for result in run["cases"].values()) for run in runs),
        validation=dict(metadata_matches=not metadata_errors, trace_evidence_valid=not trace_errors,
                        metadata_errors=metadata_errors, trace_errors=trace_errors),
        scope="Engineering endpoint/reset stress comparing normal upright C++ capture; no full fallen-pose recovery executed",
        full_recovery_exercised=False, recovery_success_rate=None, hardware_ready=False,
        metric_scope="Early-window metrics are descriptive; full-horizon source steady checks are recorded separately",
        plot=dict(file="takeover_comparison.png", command_case=command_case,
                  initial_profiles=profiles, window_seconds=args.window_seconds,
                  axis_order="policy/API order; left hip index 0; left wheel index 4"),
        comparator_sha256=digest(Path(__file__)), runs=[dict(
            label=run["label"], source_directory=str(run["root"]),
            source_report_sha256=run["source_report_sha256"],
            source_status=run["report"].get("status"), source_error=run["report"].get("error"),
            expected_blend_seconds=run["expected_blend_seconds"],
            observed_blend_seconds=run.get("observed_blend_seconds"),
            policy_sha256=run["report"].get("policy_sha256"),
            contract_sha256=run["report"].get("contract_sha256"),
            asset_manifest_sha256=run["report"].get("asset_manifest_sha256"),
            source_sha256=run["report"].get("source_sha256"),
            bridge_configuration_except_blend=bridge_configuration(run["report"]),
            initial_profiles=run["report"].get("initial_profiles"),
            all_cases_complete=bool(run["cases"]) and all(result["complete"] for result in run["cases"].values()),
            confirmed_passes=sum(result["outcome"] == "PASS" for result in run["cases"].values()),
            cases=list(run["cases"].values())) for run in runs])
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "takeover_comparison.json").write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    plot(destination, runs, profiles, command_case, args.window_seconds, valid, args.dpi)
    print(json.dumps(dict(comparison_valid=valid, full_recovery_exercised=False,
        output=str(destination), metadata_errors=len(metadata_errors), trace_errors=len(trace_errors),
        case_outcomes={run["label"]: {key: result["outcome"] for key, result in run["cases"].items()}
                       for run in runs}), indent=2))
    return 0 if valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
