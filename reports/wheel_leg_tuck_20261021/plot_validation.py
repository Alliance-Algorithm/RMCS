"""Render the recorded, independent PhysX measurements; does not run simulation."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
results = json.loads((root / "isaac_final/report.json").read_text())
labels = {
    "front": "Legs forward",
    "back": "Legs backward",
    "asymmetric": "Opposite headings",
    "pitch_forward": "Body pitch +20 deg",
    "pitch_backward": "Body pitch -20 deg",
    "ground_supported": "Wheels supporting weight",
    "ground_no_integral": "Support, integral disabled",
}
colors = plt.get_cmap("tab10")
plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.dpi": 150})
fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), layout="constrained")
for i, row in enumerate(results["results"]):
    name = row["case"]
    samples = json.loads((root / f"isaac_final/{name}_samples.json").read_text())
    t = np.array([s["time_s"] for s in samples])
    knee = np.array([s["knee_deg"] for s in samples])
    error_mm = np.abs([s["wheel_com_error_m"] for s in samples]).max(axis=1) * 1000
    negative = row["integral_gain"] == 0
    color, style = ("0.4", "--") if negative else (colors(i), "-")
    for j in range(2):
        axes[0, 0].plot(t, knee[:, j], color=color, ls=style,
                        lw=1.3 if j == 0 else .7, alpha=1 if j == 0 else .6,
                        label=labels[name] if j == 0 else None)
        mask = t >= 5.5
        axes[0, 1].plot(t[mask], knee[mask, j], color=color, ls=style, lw=1.2)
    axes[1, 0].plot(t, error_mm, color=color, ls=style, lw=1.2)
    if name == "ground_supported":
        support = np.array([s["wheel_support_n"] for s in samples])
        axes[1, 1].plot(t, support[:, 0], color=colors(0), label="Left wheel")
        axes[1, 1].plot(t, support[:, 1], color=colors(1), ls="--", label="Right wheel")
        axes[1, 1].plot(t, [s["nonwheel_support_n"] for s in samples],
                        color="0.4", label="All other bodies")

for ax in axes[0]:
    ax.axhspan(40, 41, color="#7aa68b", alpha=.16)
    ax.axhline(40.5, color="#446f54", lw=1, ls=":")
    ax.set_ylabel("Actual thigh/calf angle (deg)")
axes[0, 0].set(title="A  Passive knee measurements", ylim=(39.5, 58))
axes[0, 0].legend(fontsize=8, loc="upper right")
axes[0, 1].set(title="B  Final 0.5 s / transfer window", ylim=(39.95, 41.15))
axes[0, 1].text(5.52, 40.12, "Transfer range: 40 to 41 deg", color="#446f54", fontsize=9)
axes[1, 0].axhline(5, color="#aa4b38", lw=1, ls=":", label="5 mm transfer limit")
axes[1, 0].set(title="C  Wheel alignment to whole-robot COM", yscale="log",
               ylim=(.01, 300), ylabel="Max absolute fore/aft error (mm)")
axes[1, 0].legend(fontsize=8)
axes[1, 1].axhline(13.886 * 9.81 / 2, color="#446f54", ls=":", label="Half of robot weight")
axes[1, 1].set(title="D  Ground contact / vertically guided body", ylim=(-3, 100),
               ylabel="Vertical contact force (N)")
axes[1, 1].legend(fontsize=8, loc="center right")
for ax in axes.flat:
    ax.set_xlabel("Time (s)")
    ax.grid(alpha=.2)
fig.suptitle("Wheel-leg final tuck | Isaac Sim 6.0.0.1 | dt = 1 ms\n"
             "Leg fixtures with closed chains and gas springs; body attitude constrained", fontsize=13)
fig.savefig(root / "tuck_validation.png")
print(root / "tuck_validation.png")
