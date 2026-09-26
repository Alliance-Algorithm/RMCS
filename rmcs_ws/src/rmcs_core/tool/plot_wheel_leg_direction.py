#!/usr/bin/env python3
"""Plot the measured PhysX trace from --direction-comparison."""
import argparse
import json
import os
from pathlib import Path
import subprocess

os.environ.setdefault("MPLCONFIGDIR", "/tmp/rmcs-matplotlib")
import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("report", type=Path)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
report = json.loads(args.report.read_text())
record = np.load(args.report.with_suffix(".trace.npz"))
trace = record["trace"]
time = record["time_s"] - report["phases"][-1]["start_time_s"]
selected = time >= -1.e-8
time, trace = time[selected], trace[selected]
font_path = subprocess.check_output(["fc-match", "-f", "%{file}", "Noto Sans CJK SC"], text=True).strip()
font_manager.fontManager.addfont(font_path)
font_name = font_manager.FontProperties(fname=font_path).get_name()
plt.rcParams.update({"font.family": font_name, "axes.unicode_minus": False,
                     "font.size": 10, "axes.titlesize": 12})
fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
blue, red = "#1668ad", "#c04a30"

ax = axes[0, 0]
for env, color, label in [(0, blue, "联合选弧"), (1, red, "独立短弧")]:
    for joint, style, name in [(0, "-", "髋"), (1, "--", "膝")]:
        ax.plot(time, trace[:, env, 3, joint], color=color, ls=style,
                label=f"{label} · {name}", lw=2)
ax.axhline(0, color="gray", lw=.7)
ax.set(xlim=(0, .65), ylabel="URDF 方向速度指令 (rad/s)", title="1  目标切换后：两种选弧给出了不同方向")
ax.legend(fontsize=9, ncol=2)

ax = axes[0, 1]
for env, color, label in [(0, blue, "联合选弧"), (1, red, "独立短弧")]:
    difference = trace[:, env, 0, 0] - trace[:, env, 0, 1]
    ax.plot(time, difference, color=color, label=label, lw=2)
ax.axhline(report["parameters"]["max_motor_difference"], color=red, ls=":", label="120° 膝内角对应边界")
ax.axhline(0, color="gray", ls=":", label="目标开合量")
ax.set(ylabel="闭链开合量 h−k (rad)", title="2  独立短弧停在边界，联合选弧到达目标")
ax.legend(fontsize=9)

ax = axes[1, 0]
actual_knee_phase = (trace[:, 0, 0, 1] + np.pi) % (2 * np.pi) - np.pi
ax.plot(time, trace[:, 0, 5, 1], color="#7559a6", label="DM 原始角度")
ax.plot(time, trace[:, 0, 1, 1], color=blue, lw=2, label="RMCS 解码相位")
ax.plot(time, actual_knee_phase, color="#222222", ls=":", label="PhysX 真实角度取相位")
ax.set(xlim=(0, 2.5), ylabel="左膝电机角度 (rad)", title="3  原始角度过零回绕，解码仍与真实相位吻合")
ax.legend(fontsize=9)

ax = axes[1, 1]
for env, color, label in [(0, blue, "联合选弧"), (1, red, "独立短弧")]:
    error = (trace[:, env, 0] - trace[:, env, 2] + np.pi) % (2 * np.pi) - np.pi
    ax.plot(time, np.rad2deg(np.abs(error).max(axis=1)), color=color, label=label, lw=2)
final = np.rad2deg(report["phases"][-1]["per_environment_max_error_rad"])
ax.text(.97, .7, f"独立短弧残差 {final[1]:.2f}°\n联合选弧残差 {final[0]:.3f}°",
        transform=ax.transAxes, ha="right", va="top", bbox=dict(fc="white", ec="#dddddd", pad=6))
ax.set(ylabel="四电机最大角度误差 (°)", title="4  相同速度环下的到位结果")
ax.legend(fontsize=9, loc="upper right")
for ax in axes.flat:
    ax.set_xlabel("切换至 [0, 0, 0, 0] 后的时间 (s)")
    ax.grid(alpha=.2)
fig.suptitle("Isaac Sim V5 闭链方向对照：相同电机参数，只改变角度选弧\n"
             "初始左髋/左膝 −2.100 / −3.430 rad；保留实机 offset，经过实际 DM 驱动与 VEL 帧", fontsize=14)
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output, dpi=160)
plt.close(fig)
