#!/usr/bin/env python3
"""Inspect a real MCAP immediately after a single-side PC torque-cascade run.

This checks recording and controller telemetry, not physical model accuracy.
It needs no URDF zero offsets or SciPy, so it can run inside the RMCS container.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml

from export_identification_bag import read_bag


def inspect(rows: list[dict], side: str, kind: str = "pair",
            require_validation: bool = False) -> dict:
    if side not in ("left", "right") or not rows:
        raise ValueError("expected a nonempty left/right identification bag")
    if kind == "wheel":
        return inspect_wheels(rows, side)
    if kind != "pair":
        raise ValueError("unknown experiment kind")
    index = 0 if side == "left" else 2
    other = 2 if side == "left" else 0
    expected = 0 if side == "left" else 1
    tick_gaps = 0
    age_ms = []
    for previous, current in zip(rows, rows[1:]):
        tick_gaps += int(current["tick"] != previous["tick"] + 1)
    running = [r for r in rows if r["phase"] == 2]
    active = [r for r in running if r["actuation_scope"] == 1]
    heartbeat = [r for r in running if r["actuation_scope"] == 0
                 and all(r["tx_kind"][axis] == 2 for axis in (index, index + 1))
                 and all(r["dm_status"][axis] == 1 for axis in (index, index + 1))
                 and all(r["dm_status"][axis] == 0 for axis in (other, other + 1))]
    capture_only = (not running and all(not r["enable_requested"]
                                        and r["actuation_scope"] == 0
                                        and all(status == 0 for status in r["dm_status"])
                                        for r in rows))
    complete = 0
    for row in active:
        values = [*row["q_ref_model"][index:index + 2],
                  *row["velocity_target_model"][index:index + 2],
                  *row["tau_preclip_model"][index:index + 2]]
        complete += int(all(math.isfinite(v) for v in values))
        for axis in (index, index + 1):
            if row["feedback_steady_ns"][axis]:
                age_ms.append((row["control_steady_ns"] - row["feedback_steady_ns"][axis]) / 1e6)
    reasons = []
    if any(row["selected_side"] != expected for row in rows):
        reasons.append("bag selected_side disagrees with run profile")
    if not running and not capture_only:
        reasons.append("no phase=2 control samples")
    if len(active) + len(heartbeat) != len(running):
        reasons.append("some running samples do not confirm a selected-only torque-mode pair")
    if complete != len(active):
        reasons.append("outer position/velocity target or inner torque telemetry missing")
    if rows[-1]["dropped_samples"] or tick_gaps:
        reasons.append("recorder reports dropped messages or nonconsecutive executor ticks")
    if any(r["dm_status"][other] != 0 or r["dm_status"][other + 1] != 0 for r in running):
        reasons.append("inactive DM pair did not report disabled")
    if any(age < 0 for age in age_ms):
        reasons.append("motor CAN receive time after controller sample")
    if require_validation and running and not any(r.get("segment_role") == 1 for r in active):
        reasons.append("independent pair validation segment missing")
    status = "inspect_before_fitting" if reasons else (
        "passive_capture" if capture_only else "recording_complete")
    def bounds(field: str, axis: int) -> dict:
        values = [r[field][axis] for r in active if math.isfinite(r[field][axis])]
        return {"min": min(values, default=None), "max": max(values, default=None),
                "range": max(values) - min(values) if values else None}

    axis_summary = {
        name: {"q_api": bounds("q_api", axis),
               "dq_api": bounds("dq_api", axis),
               "torque_fb_api": bounds("torque_fb_api", axis),
               "q_ref_model": bounds("q_ref_model", axis),
               "tau_preclip_api": bounds("tau_preclip_api", axis),
               "tau_cmd_api": bounds("tau_cmd_api", axis),
               "tau_frame_api": bounds("tau_frame_api", axis),
               "torque_limited_samples": sum(bool(r["torque_limited"][axis]) for r in active)}
        for name, axis in (("hip", index), ("knee", index + 1))
    } if active and "tau_preclip_api" in active[0] else {}
    segment_summary = []
    for segment in sorted({r["segment_id"] for r in active}):
        segment_rows = [r for r in active if r["segment_id"] == segment]
        first, last = segment_rows[0], segment_rows[-1]
        def pair(field: str, row: dict) -> list[float]:
            return [row[field][index], row[field][index + 1]]
        segment_summary.append({
            "id": segment, "waveform": first.get("segment_waveform"),
            "samples": len(segment_rows), "validation": first.get("segment_role") == 1,
            "q_start_api": pair("q_api", first), "q_end_api": pair("q_api", last),
            "q_ref_end_model": pair("q_ref_model", last),
            "motor_difference_start_rad": first["q_api"][index + 1] - first["q_api"][index],
            "motor_difference_end_rad": last["q_api"][index + 1] - last["q_api"][index],
            "peak_abs_frame_torque_nm": [max(abs(r["tau_frame_api"][axis]) for r in segment_rows)
                                         for axis in (index, index + 1)],
        })
    timing_summary = []
    for name, stamp_index in (("left_hip", 0), ("left_knee", 1), ("right_hip", 2),
                              ("right_knee", 3), ("left_wheel", 4), ("right_wheel", 5)):
        values = [(r["control_steady_ns"] - r["feedback_steady_ns"][stamp_index]) / 1e6
                  for r in rows if r["feedback_steady_ns"][stamp_index]]
        timing_summary.append({"axis": name, "max_age_ms": max(values, default=None)})
    gyro_age = [(r["control_steady_ns"] - r["imu_last_ns"]) / 1e6 for r in rows
                if r["imu_last_ns"]]
    failures = [r for r in rows if r["phase"] == -1]
    terminal = failures[0] if failures else None
    return {"status": status,
             "side": side, "samples": len(rows), "running_samples": len(running),
             "selected_only_torque_samples": len(active), "complete_two_loop_samples": complete,
             "selected_system_heartbeat_samples": len(heartbeat),
             "segment_ids": sorted({r["segment_id"] for r in active}),
             "validation_samples": sum(r.get("segment_role") == 1 for r in active),
             "axis_summary": axis_summary,
             "segment_summary": segment_summary,
             "all_axis_feedback_age_ms": timing_summary,
             "imu_gyro_age_ms_max": max(gyro_age, default=None),
             "failure_terminal": ({
                 "failure_reason": terminal.get("failure_reason"),
                 "feedback_age_ms": [(terminal["control_steady_ns"] - stamp) / 1e6 if stamp else None
                                      for stamp in terminal["feedback_steady_ns"]],
                 "imu_age_ms": (terminal["control_steady_ns"] - terminal["imu_last_ns"]) / 1e6
                                if terminal["imu_last_ns"] else None,
             } if terminal is not None else None),
            "dropped_samples": rows[-1]["dropped_samples"], "tick_gaps": tick_gaps,
            "selected_can_age_ms_max": max(age_ms, default=None), "reasons": reasons}


def inspect_wheels(rows: list[dict], side: str) -> dict:
    running = [row for row in rows if row["phase"] == 2]
    active = [row for row in running if row["actuation_scope"] == 3]
    servo = [row for row in active if row["wheel_mode"] == 1]
    coast = [row for row in active if row["wheel_mode"] == 2]
    gaps = sum(current["tick"] != previous["tick"] + 1 for previous, current in zip(rows, rows[1:]))
    reasons = []
    if not running or len(active) != len(running):
        reasons.append("missing wheel-only running samples or parked DM status 0")
    if not all(row["selected_side"] == (0 if side == "left" else 1)
               and row["experiment_kind"] == 1 for row in rows):
        reasons.append("bag profile and wheel experiment kind disagree")
    if any(not all(math.isfinite(value) for value in (
            *row["wheel_velocity_target_api"], *row["dq_api"][4:6],
            *row["tau_preclip_api"][4:6], *row["tau_frame_api"][4:6])) for row in active):
        reasons.append("wheel target, feedback or actual frame effort missing")
    targets = [row["wheel_velocity_target_api"][0] for row in servo]
    if not any(value > .2 for value in targets) or not any(value < -.2 for value in targets):
        reasons.append("wheel run lacks both rotation directions")
    if not coast or any(abs(value) > .03 for row in coast for value in row["tau_frame_api"][4:6]):
        reasons.append("enabled zero-current coast is missing or not neutral")
    if not any(row["segment_role"] == 1 for row in active):
        reasons.append("independent validation segment missing")
    if rows[-1]["dropped_samples"] or gaps:
        reasons.append("recorder dropped samples or executor tick gap")
    return {"status": "wheel_recording_complete" if not reasons else "inspect_before_fitting",
            "side": side, "samples": len(rows), "running_samples": len(running),
            "wheel_current_samples": len(active), "coast_samples": len(coast),
            "dropped_samples": rows[-1]["dropped_samples"], "tick_gaps": gaps,
            "reasons": reasons}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--profile", required=True, type=Path)
    args = parser.parse_args()
    invalid_run = args.bag.resolve().parent / "RUN_INVALID.json"
    if invalid_run.exists():
        print(json.dumps({"status": "invalid_profile_provenance", "marker": str(invalid_run),
                          "reasons": ["Applied experiment disagrees with the archived contract"]}, indent=2))
        return 2
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    side = profile["wheel_leg_identification_recorder"]["ros__parameters"]["side"]
    kind = profile["wheel_leg_identification_recorder"]["ros__parameters"].get("experiment_kind", "pair")
    controller = profile.get("wheel_leg_pair_identification_controller", {}).get("ros__parameters", {})
    report = inspect(read_bag(args.bag), side, kind, controller.get("experiment_stage") == "probe")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["status"] in ("recording_complete", "passive_capture", "wheel_recording_complete") else 2


if __name__ == "__main__":
    raise SystemExit(main())
