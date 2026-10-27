#!/usr/bin/env python3
"""Replot the limit CSV using the controller's calibrated PairGeometry inverse."""

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "rmcs_ws/src/rmcs_bringup/config/wheel-leg-joint-limit-probe.yaml"
SWEEP = "/wheel_leg/sweep/"


class PairGeometry:
    """Vectorized port of wheel_leg_pair_recording_plan.hpp, including endpoint slopes."""

    def __init__(self, beta_deg, delta):
        self.x = np.deg2rad(np.asarray(beta_deg, dtype=float))
        self.y = np.asarray(delta, dtype=float)
        h = np.diff(self.x)
        d = np.diff(self.y) / h
        if len(self.x) < 3 or not np.all(h > 0) or not np.all(d * d[0] > 0):
            raise ValueError("Expected a strictly monotone beta/delta calibration")
        w1, w2 = 2 * h[1:] + h[:-1], h[1:] + 2 * h[:-1]
        slope = np.r_[d[0], (w1 + w2) / (w1 / d[:-1] + w2 / d[1:]), d[-1]]
        self.h = h
        self.a = 2 * self.y[:-1] - 2 * self.y[1:] + h * (slope[:-1] + slope[1:])
        self.b = -3 * self.y[:-1] + 3 * self.y[1:] - h * (2 * slope[:-1] + slope[1:])
        self.c = h * slope[:-1]

    def at(self, beta):
        beta = np.asarray(beta)
        if np.any((beta < self.x[0] - 1e-10) | (beta > self.x[-1] + 1e-10)):
            raise ValueError("Extrapolation outside the geometry table is forbidden")
        beta = np.clip(beta, self.x[0], self.x[-1])
        i = np.clip(np.searchsorted(self.x, beta, side="right") - 1, 0, len(self.x) - 2)
        t = (beta - self.x[i]) / self.h[i]
        return ((self.a[i] * t + self.b[i]) * t + self.c[i]) * t + self.y[i]

    def beta(self, delta):
        delta = np.asarray(delta, dtype=float)
        valid = np.isfinite(delta) & (delta >= min(self.y)) & (delta <= max(self.y))
        result = np.full(delta.shape, np.nan)
        lo = np.full(np.count_nonzero(valid), self.x[0])
        hi = np.full_like(lo, self.x[-1])
        for _ in range(48):
            middle = (lo + hi) / 2
            select_lo = (self.at(middle) < delta[valid]) == (self.y[-1] > self.y[0])
            lo = np.where(select_lo, middle, lo)
            hi = np.where(select_lo, hi, middle)
        result[valid] = (lo + hi) / 2
        return result


def calibration(path):
    # These named parameters are numeric inline arrays in the controller YAML.
    # Parse just those arrays; this script does not rewrite or interpret other YAML.
    content = path.read_text()
    names = ["model_sign", "model_offset", "geometry_beta_deg", "geometry_left_delta",
             "geometry_right_delta"]
    result = {}
    for name in names:
        matches = re.findall(rf"^    {name}:\s*(\[[^\n]+\])\s*$", content, re.MULTILINE)
        if len(matches) != 1:
            raise ValueError(f"Expected one numeric inline array for {name}")
        result[name] = json.loads(matches[0])
    return result


def read_columns(path, columns):
    values = []
    incomplete = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        indices = [header.index(name) for name in columns]
        for line, row in enumerate(reader, start=2):
            if len(row) != len(header):
                incomplete.append({"csv_line": line, "index": row[0] if row else None,
                                   "fields": len(row), "expected_fields": len(header)})
                continue
            values.append([float(row[i]) for i in indices])
    return dict(zip(columns, np.asarray(values).T)), incomplete


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "reports/leg_sweep_debug")
    args = parser.parse_args()
    columns = ["index", SWEEP + "control_steady_ns", SWEEP + "state",
               SWEEP + "probe/low/result", SWEEP + "probe/high/result"]
    for side in ("left", "right"):
        columns.extend([SWEEP + side + "/beta_estimate", SWEEP + side + "/beta_valid"])
        for joint in ("hip", "knee"):
            axis = f"{side}_{joint}_joint"
            columns.extend([SWEEP + axis + "/model_angle", f"/wheel_leg/{axis}/angle",
                            f"/wheel_leg/{axis}/feedback_sequence"])
    data, incomplete = read_columns(args.csv, columns)
    params = calibration(args.config)
    clock = data[SWEEP + "control_steady_ns"]
    if not np.all(np.isfinite(clock)) or not np.all(np.diff(clock) > 0):
        raise ValueError("Complete records need finite, increasing control timestamps")
    time = (clock - clock[0]) * 1e-9
    summary = {
        "source_csv": str(args.csv.resolve()), "source_sha256": digest(args.csv),
        "calibration_yaml": str(args.config.resolve()), "calibration_sha256": digest(args.config),
        "geometry_source": "v6_pair_upstream_zero_geometry_v1 @ 8149afda",
        "method": "PairGeometry Hermite interpolation with 48-step inverse bisection",
        "reference_beta_deg": [40, 110], "geometry_table_beta_deg": [30, 120],
        "complete_records": len(time), "incomplete_records_excluded": incomplete,
        "duration_s": float(time[-1]), "sides": {},
        "probe_low_result_max": float(np.nanmax(data[SWEEP + "probe/low/result"])),
        "probe_high_result_max": float(np.nanmax(data[SWEEP + "probe/high/result"])),
    }
    series = {}
    for n, side in enumerate(("left", "right")):
        geom = PairGeometry(params["geometry_beta_deg"], params[f"geometry_{side}_delta"])
        hip, knee = [data[SWEEP + f"{side}_{j}_joint/model_angle"] for j in ("hip", "knee")]
        ready = np.isfinite(hip) & np.isfinite(knee)
        phase_residual = []
        for joint, model in zip(("hip", "knee"), (hip, knee)):
            axis = f"{side}_{joint}_joint"
            i = 2 * n + int(joint == "knee")
            received = data[f"/wheel_leg/{axis}/feedback_sequence"] > 0
            ready &= received
            phase = params["model_sign"][i] * data[f"/wheel_leg/{axis}/angle"]
            phase += params["model_offset"][i]
            residual = (model - phase + np.pi) % (2 * np.pi) - np.pi
            phase_residual.append(np.max(np.abs(residual[received])))
        delta = np.where(ready, knee - hip, np.nan)
        beta_rad = geom.beta(delta)
        logged = data[SWEEP + side + "/beta_estimate"]
        logged_valid = data[SWEEP + side + "/beta_valid"] == 1
        if not np.array_equal(np.isfinite(beta_rad), logged_valid):
            raise ValueError(f"{side}: reconstructed beta validity differs from CSV")
        if not np.allclose(beta_rad[logged_valid], logged[logged_valid], atol=1e-11, rtol=0):
            raise ValueError(f"{side}: reconstructed beta differs from CSV")
        beta = np.rad2deg(beta_rad)
        details = {
            "valid_records": int(np.count_nonzero(np.isfinite(beta))),
            "feedback_unavailable_records": int(np.count_nonzero(~ready)),
            "out_of_table_records": int(np.count_nonzero(ready & ~np.isfinite(beta))),
            "delta_definition": "q_model_knee - q_model_hip",
            "delta_reference_at_40_110_rad": geom.at(np.deg2rad([40, 110])).tolist(),
            "delta_min_max_rad": [float(np.nanmin(delta)), float(np.nanmax(delta))],
            "below_40_records": int(np.count_nonzero(beta < 40)),
            "above_110_records": int(np.count_nonzero(beta > 110)),
            "max_abs_beta_csv_error_rad": float(np.max(np.abs(beta_rad[logged_valid] - logged[logged_valid]))),
            "max_abs_model_phase_error_rad": float(max(phase_residual)),
        }
        for label, i in (("min", np.nanargmin(beta)), ("max", np.nanargmax(beta))):
            details[label] = {"beta_deg": float(beta[i]), "time_s": float(time[i]),
                              "index": int(data["index"][i]), "delta_rad": float(delta[i]),
                              "state": int(data[SWEEP + "state"][i])}
        summary["sides"][side] = details
        series[side] = (np.where(ready, hip, np.nan), np.where(ready, knee, np.nan), delta, beta)

    font_path = Path("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc")
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font_path).get_name()
    plt.rcParams.update({"font.size": 11, "axes.unicode_minus": False,
                         "svg.fonttype": "none", "axes.spines.top": False,
                         "axes.spines.right": False, "axes.labelcolor": "#334155",
                         "xtick.color": "#475569", "ytick.color": "#475569"})
    fig, axes = plt.subplots(3, 2, figsize=(16, 10.5), sharex=True,
                             gridspec_kw={"height_ratios": [1.3, 1, 1]})
    for col, (side, title) in enumerate((("left", "左腿"), ("right", "右腿"))):
        hip, knee, delta, beta = series[side]
        result = summary["sides"][side]
        ax = axes[0, col]
        ax.set_title(f"{title}：大小腿机械夹角 β", loc="left", fontsize=14, fontweight="bold", pad=12)
        ax.axhspan(40, 110, color="#ecf5ee", label="参考范围 40–110°")
        ax.axhline(40, color="#b36b17", linestyle="--", linewidth=1)
        ax.axhline(110, color="#b36b17", linestyle="--", linewidth=1)
        ax.plot(time, beta, color="#0f766e", linewidth=1.4, label="代码几何换算 β")
        ax.fill_between(time, beta, 40, where=beta < 40, color="#efb4a8", alpha=.8)
        ax.set_ylim(28, 126)
        ax.set_yticks([30, 40, 60, 80, 100, 110, 120])
        ax.set_ylabel("β（°）")
        ax.legend(loc="upper left", frameon=False, fontsize=9)
        for label, offset in (("min", (12, 14)), ("max", (12, -45))):
            point = result[label]
            ax.scatter(point["time_s"], point["beta_deg"], color="#0f766e", s=25, zorder=4)
            ax.annotate(f"{point['beta_deg']:.2f}°", (point["time_s"], point["beta_deg"]),
                        xytext=offset, textcoords="offset points", fontsize=11,
                        bbox={"facecolor": "white", "edgecolor": "none", "alpha": .85},
                        arrowprops={"arrowstyle": "-", "color": "#64748b"})
        ax = axes[1, col]
        ax.set_title("标定后的电机差值 Δ = q_knee − q_hip", loc="left", fontsize=12)
        bounds = result["delta_reference_at_40_110_rad"]
        ax.axhspan(min(bounds), max(bounds), color="#ecf5ee")
        for degree, bound in zip((40, 110), bounds):
            ax.axhline(bound, color="#b36b17", linestyle="--", linewidth=1)
            ax.text(time[-1] - 1, bound + .035, f"β={degree}°：{bound:+.6f} rad",
                    ha="right", fontsize=9, color="#865316")
        ax.plot(time, delta, color="#7253a3", linewidth=1.2)
        ax.set_ylabel("Δ（rad）")
        ax.set_ylim(-.55, 1.72) if side == "left" else ax.set_ylim(-1.72, .55)
        ax = axes[2, col]
        ax.set_title("连续模型角（原 fig1 的角度坐标）", loc="left", fontsize=12)
        ax.plot(time, hip, color="#2563eb", label="hip", linewidth=1.1)
        ax.plot(time, knee, color="#d95c30", label="knee", linewidth=1.1)
        ax.set_ylabel("q_model（rad）")
        ax.set_xlabel("记录时间（s）")
        ax.legend(loc="upper left", frameon=False, fontsize=10, ncol=2)
    for ax in axes.flat:
        ax.set_xlim(0, time[-1])
        ax.grid(axis="both", color="#dbe2e9", linewidth=.6, alpha=.7)
        ax.set_axisbelow(True)
    fig.suptitle("Fig.1  髋、膝电机角差换算大小腿夹角", x=.065, ha="left", y=.982,
                 fontsize=19, fontweight="bold")
    fig.text(.065, .944, "q_model = sign × angle + offset；Δ 经各侧 PairGeometry 反解为 β。图中均为反馈换算值。",
             fontsize=11, color="#475569")
    fig.text(.065, .022, "β 为当前标定下的闭链几何估计；40–110°为参考范围。曲线极值尚不能直接认定为实车机械硬限位。",
             fontsize=11, color="#475569")
    fig.subplots_adjust(left=.065, right=.98, bottom=.083, top=.885, hspace=.40, wspace=.18)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "svg"):
        output = args.output_dir / f"fig1_limits_overview.{suffix}"
        fig.savefig(output, dpi=140)
        if suffix == "svg":
            output.write_text("\n".join(line.rstrip() for line in output.read_text().splitlines()) + "\n")
    plt.close(fig)
    (args.output_dir / "limits_mechanical_angle_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
