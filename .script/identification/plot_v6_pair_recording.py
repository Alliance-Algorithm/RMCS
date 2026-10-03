#!/usr/bin/env python3
"""Plot nominal 50 Hz reference previews, without suggesting measured coverage."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def plot(csv_path: Path, output: Path, run: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = np.genfromtxt(csv_path, names=True, delimiter=",")
    jump = a["cycle"] >= 0
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), layout="constrained")
    axes[0].step(a["time_s"], a["beta_requested_deg"], where="post", color="#287c8e", lw=.9)
    axes[0].set_ylabel("Inner knee beta [deg]")
    axes[0].set_title(f"{run}: nominal targets at 50 Hz; no measured feedback")
    for i, label in (("q_hip_rad", "Hip"), ("q_aux_rad", "Auxiliary")):
        axes[1].step(a["time_s"], a[i], where="post", lw=.8, label=label)
    axes[1].set_ylabel("V6 active axis [rad]")
    axes[1].legend(loc="upper right")
    if jump.any():
        selection = jump & (a["cycle"] < 3)
        axes[2].step(a["time_s"][selection], a["beta_requested_deg"][selection], where="post", lw=1.2)
        phases = ("none", "compress", "loaded", "extend", "tuck", "reach", "buffer", "settle", "gap")
        for phase in range(1, 9):
            indices = np.flatnonzero(selection & (a["jump_phase"] == phase))
            for block in np.split(indices, np.flatnonzero(np.diff(indices) != 1)+1):
                if block.size:
                    axes[2].axvspan(a["time_s"][block[0]], a["time_s"][block[-1]]+.02,
                                   color=plt.get_cmap("tab10")(phase), alpha=.15)
                    if a["cycle"][block[0]] == 0:
                        axes[2].text(a["time_s"][block[0]], 102., phases[phase],
                                     fontsize=8, rotation=60, va="bottom")
        axes[2].set_ylim(62, 114)
        axes[2].set_ylabel("First 3 airborne cycles [deg]")
    else:
        for key, label in (("dq_hip_rad_s", "Hip"), ("dq_aux_rad_s", "Auxiliary")):
            axes[2].plot(a["time_s"], a[key], lw=.7, label=label)
        axes[2].set_ylabel("Smooth trajectory dq [rad/s]")
        axes[2].legend(loc="upper right")
    for axis in axes:
        axis.set_xlabel("Nominal time [s]")
        axis.grid(alpha=.2)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--run", required=True)
    args = parser.parse_args()
    plot(args.csv, args.output, args.run)


if __name__ == "__main__":
    main()
