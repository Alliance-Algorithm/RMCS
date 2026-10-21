#!/usr/bin/env python3
"""Read-only analysis of the supplied CSV; exclude its incomplete final row."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
source = ROOT / "input.csv"
with source.open() as handle:
    reader = csv.reader(handle)
    header = next(reader)
    raw = list(reader)
data = np.array([[float(v) for v in row] for row in raw if len(row) == len(header)])


def col(path):
    return header.index("/wheel_leg/rl/" + path)


state = data[:, col("state")].astype(int)
phase = data[:, col("recovery/native_phase")].astype(int)
names = ["left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"]
q_idx = [col(f"model/{name}/angle") for name in names]
ref_idx = [col(f"recovery/{name}/reference") for name in names]
goal_idx = [col(f"recovery/{name}/tuck_target") for name in names]
gravity = data[:, [col("input/projected_gravity/" + axis) for axis in "xyz"]]
tilt = np.rad2deg(np.arccos(np.clip(-gravity[:, 2], -1., 1.)))
episodes = []
starts = np.flatnonzero((state == 2) & (phase == 1) & np.r_[True, phase[:-1] != 1])
for first in starts:
    last = next((i for i in range(first + 1, len(data)) if state[i] != 2), len(data))
    prepare = next(i for i in range(first, last) if phase[i] == 7)
    stop = last - 1
    mean = data[np.ix_([prepare, stop], ref_idx)].reshape(2, 2, 2).mean(axis=2)
    q = data[stop, q_idx]
    episodes.append(dict(start_index=int(data[first, 0]), prepare_index=int(data[prepare, 0]),
        last_recovering_index=int(data[stop, 0]),
        fold_duration_s=float((data[prepare, 0] - data[first, 0]) * .005),
        rotation_until_fault_s=float((data[last, 0] - data[prepare, 0]) * .005),
        start_q_rad=data[first, q_idx].tolist(), start_gravity=gravity[first].tolist(),
        final_q_rad=q.tolist(), final_reference_rad=data[stop, ref_idx].tolist(),
        final_knee_deg=data[stop, [col("recovery/tuck/left/knee_deg"),
                                   col("recovery/tuck/right/knee_deg")]].tolist(),
        final_hinge_rad=[float(q[1] - q[0]), float(q[2] - q[3])],
        reference_pair_rotation_rad=(mean[1] - mean[0]).tolist(),
        remaining_rad=float(data[stop, col("recovery/trajectory_remaining_rad")]),
        max_tilt_deg=float(tilt[first:last].max()),
        next_failure=int(data[last, col("recovery/failure")]),
        native_phases=sorted(set(phase[first:last].tolist()))))
result = dict(source_original="/home/noir/wheel_leg_rl_2026-10-21_17-18-21.csv",
    source_observed_later="/home/noir/.local/share/Trash/files/wheel_leg_rl_2026-10-21_17-18-21.csv",
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    valid_rows=len(data), incomplete_rows=len(raw) - len(data), columns=len(header),
    sample_period_s=.005, rl_state_3_rows=int((state == 3).sum()), episodes=episodes,
    findings=["Four FOLD/PREPARE attempts, no RL ownership in this recording.",
        "Failed rotation increases the left pair and decreases the right pair.",
        "Pasted log reports right measured hinge -0.1734 and -0.1719 rad below -0.1706 rad.",
        "Last complete CSV samples precede the exact logged limit crossing.",
        "This CSV cannot establish what the actor does after handover."])
(ROOT / "analysis.json").write_text(json.dumps(result, indent=2) + "\n")
np.savez_compressed(ROOT / "csv_derived.npz", index=data[:, 0], state=state, phase=phase,
    q=data[:, q_idx], reference=data[:, ref_idx], goal=data[:, goal_idx], tilt_deg=tilt,
    knees=data[:, [169, 170]], remaining=data[:, col("recovery/trajectory_remaining_rad")])
print(json.dumps(result, indent=2))
