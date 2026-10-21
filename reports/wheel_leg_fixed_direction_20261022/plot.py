#!/usr/bin/env python3
"""Plot recorded failure and the measured failed-start closed-chain probe."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
recorded = np.load(ROOT / "csv_derived.npz")
samples = json.loads((ROOT / "isaac_final/csv_failed_suspended_samples.json").read_text())
report = json.loads((ROOT / "isaac_final/report.json").read_text())
entry = next(row for row in report["results"] if row["case"] == "csv_failed_suspended")
mask = (recorded["index"] >= 5025) & (recorded["index"] <= 5268)
old_t = (recorded["index"][mask] - 5025) * .005
new_t = np.array([s["time_s"] for s in samples])
new_q = np.array([s["active_q_rad"] for s in samples])
new_ref = np.array([s["reference_q_rad"] for s in samples])
new_knee = np.array([s["knee_deg"] for s in samples])
old_q = recorded["q"][mask]
old_ref = recorded["reference"][mask]
handover = entry["handover"]["time_s"]
fig, axes = plt.subplots(2, 2, figsize=(11.8, 7), layout="constrained")
for side, label in enumerate(["Left", "Right"]):
    ax = axes[0, side]
    ax.plot(old_t, old_ref[:, side * 2:side * 2 + 2].mean(axis=1), "--", color="#bb4238", label="Old CSV reference")
    ax.plot(old_t, old_q[:, side * 2:side * 2 + 2].mean(axis=1), color="#bb4238", label="Old CSV feedback")
    ax.plot(new_t, new_ref[:, side * 2:side * 2 + 2].mean(axis=1), "--", color="#197c9c", label="New reference")
    ax.plot(new_t, new_q[:, side * 2:side * 2 + 2].mean(axis=1), color="#197c9c", label="New PhysX feedback")
    ax.set_title(f"{label} motor pair mean angle")
    ax.set_ylabel("Model angle / rad")
    ax.axvline(handover, color="#418552", ls=":", label="RL phase event")
    ax = axes[1, side]
    ax.plot(old_t, recorded["knees"][mask, side], color="#bb4238", label="Old CSV FK knee")
    ax.plot(new_t, new_knee[:, side], color="#197c9c", label="New PhysX knee")
    ax.axhline(40.5, color="#555555", ls="--", lw=1, label="40.5 deg target")
    ax.axvline(handover, color="#418552", ls=":", label="RL phase event")
    ax.set_title(f"{label} inner knee angle")
    ax.set_ylabel("Angle / deg")
for ax in axes.flat:
    ax.set_xlim(0, 2.4)
    ax.set_xlabel("Elapsed time / s")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8, loc="best")
fig.suptitle("Failed CSV start: fixed reverse direction at 3 rad/s\n"
             "Fixed-attitude closed-chain fixture; PD holds final legs after RL event, no actor", fontsize=12)
fig.savefig(ROOT / "fixed_direction_evidence.png", dpi=160)
