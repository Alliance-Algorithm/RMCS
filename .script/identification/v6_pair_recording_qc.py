"""Actual V6 recording coverage and transient summaries; no resampling."""
from __future__ import annotations

import numpy as np


def summarize_recording(a: dict, recording: dict) -> dict:
    n = len(a["control_steady_ns"])
    time = a["control_steady_ns"]
    # Reject gaps rather than assigning their unknown interval to a cached pose.
    dt = np.zeros(n)
    for i in range(n-1):
        delta = int(time[i+1]) - int(time[i])
        if 0 < delta <= 20_000_000:
            dt[i] = delta * 1e-9
    active = (a["phase"] == 2) & (a["actuation_scope"] == 1) & (a["feedback_fresh"] != 0)
    beta = a["inner_knee_fk_deg"]
    tolerance = recording["recording_beta_tolerance_deg"]
    requested = a["inner_knee_requested_deg"]
    side = 0 if recording["side"] == "left" else 2
    coverage = {}
    for center in (45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 102):
        actual_bin = active & np.isfinite(beta) & (np.abs(beta-center) <= tolerance)
        requested_bin = active & np.isfinite(requested) & (np.abs(requested-center) <= tolerance)
        coverage[str(center)] = {"actual_fk_seconds": float(dt[actual_bin].sum()),
                                 "requested_seconds": float(dt[requested_bin].sum()),
                                 "admitted_command": recording["recording_beta_min_deg"] <= center <= recording["recording_beta_max_deg"]}
    skipped = sorted(set(map(int, a["skipped_arrival_segment"])) - {-1})
    qualified = sorted(set(map(int, a["qualified_arrival_segment"])) - {-1})
    changes = []
    last = None
    for i in np.flatnonzero(active):
        center = float(a["reference_center_delta_rad"][i])
        if not np.isfinite(center):
            continue
        if last is None or abs(center-last) > 1e-10:
            changes.append({"control_steady_ns": int(time[i]), "tick": int(a["tick"][i]),
                            "reference_update_tick": int(a["reference_update_tick"][i]),
                            "segment": int(a["segment_id"][i]), "delta_rad": center})
            last = center
    phases = []
    for cycle in sorted(set(map(int, a["jump_cycle_id"])) - {-1}):
        for phase in range(1, 9):
            mask = active & (a["jump_cycle_id"] == cycle) & (a["jump_phase"] == phase)
            if not np.any(mask):
                phases.append({"cycle": cycle, "phase": phase, "seconds": 0., "covered": False})
                continue
            error = beta[mask] - requested[mask]
            finite = np.isfinite(error)
            phases.append({"cycle": cycle, "phase": phase, "seconds": float(dt[mask].sum()),
                           "covered": bool(np.any(finite)),
                           "beta_rmse_deg": float(np.sqrt(np.mean(error[finite]**2))) if finite.any() else None,
                           "saturation_fraction": float(np.mean(a["axis_saturated"][mask, side:side+2]))})
    axes = []
    for axis in (side, side+1):
        tau, dq = a["torque_fb_model"][:, axis], a["dq_model"][:, axis]
        valid = active & np.isfinite(tau) & np.isfinite(dq)
        power = tau * dq
        effort_bins = {f"{low}-{high}": float(dt[valid & (np.abs(tau) >= low) & (np.abs(tau) < high)].sum())
                       for low, high in ((0, 10), (10, 20), (20, 30), (30, 40))}
        effort_bins[">=40"] = float(dt[valid & (np.abs(tau) >= 40)].sum())
        quadrants = {f"tau{t:+d}_dq{v:+d}": float(dt[valid & (tau*t > .1) & (dq*v > .01)].sum())
                     for t in (-1, 1) for v in (-1, 1)}
        longest = current = 0.
        for limited, seconds in zip(active & a["axis_saturated"][:, axis], dt):
            current = current + seconds if limited else 0.
            longest = max(longest, current)
        axes.append({"axis": axis, "feedback_torque_bin_seconds_nm": effort_bins,
                     "feedback_torque_velocity_quadrant_seconds": quadrants,
                     "positive_work_estimate_j": float(np.sum(np.maximum(power[valid], 0)*dt[valid])),
                     "negative_work_estimate_j": float(np.sum(np.minimum(power[valid], 0)*dt[valid])),
                     "longest_pc_limit_interval_s": float(longest)})
    return {"run": recording["recording_run"], "revision": recording["trajectory_revision"],
            "split_unit": "entire run; L03/R03/LJ02/RJ02 are independent holdouts",
            "beta_source": "versioned calibrated FK estimate, not an independent angle measurement",
            "coverage": coverage, "uncovered_arrival_segments": skipped, "qualified_arrival_segments": qualified,
            "reference_center_history": changes, "jump_phases": phases, "axes": axes,
            "limitations": ["Airborne phases do not establish jump height or actual ground contact",
                            "State duration uses raw executor intervals and cached latest feedback; not independent sample count",
                            "Feedback torque and mechanical work are drive estimates; negative work is not measured electrical regeneration"]}
