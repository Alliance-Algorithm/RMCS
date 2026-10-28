"""Scientific plot of the offline gas holding-map fit and held-out residuals."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
result = json.loads((OUT / "model.json").read_text())
data = np.load(OUT / "plot_data.npz")
font = "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(font)
plt.rcParams.update({"font.family": fm.FontProperties(fname=font).get_name(),
                     "font.size": 10.5, "axes.unicode_minus": False,
                     "figure.facecolor": "white"})
model = result["candidate_model"]
beta = np.array(model["beta_deg"])
torque = np.array(model["holding_torque_per_motor_model_nm"])
width = np.array(model["direction_half_width_nm"])
fig, axes = plt.subplots(2, 2, figsize=(14, 8.6), layout="constrained")

ax = axes[0, 0]
for sign, color, label in ((1, "#2563EB", "开腿：训练角度单元"),
                           (-1, "#D97706", "收腿：训练角度单元")):
    mask = data["train_direction"] == sign
    ax.scatter(data["train_beta"][mask], data["train_torque"][mask], s=10,
               color=color, alpha=.45, label=label)
ax.plot(beta, torque, lw=2.3, color="#0F766E", label="平均保持力矩 T")
ax.fill_between(beta, torque-width, torque+width, color="#64748B", alpha=.12,
                 label="方向差异 T±W")
ax.set(title="按实际夹角拟合；56–101° 内插", xlabel="实际大小腿夹角 / °",
       ylabel="每台电机的差动力矩幅值 / N·m", ylim=(0, 24))
ax.legend(fontsize=9, loc="lower left", ncol=2)

ax = axes[0, 1]
gaps = np.r_[False, np.diff(data["val_time"]) > .1]
def val_line(key):
    values = data[key].copy()
    values[gaps] = np.nan
    return values
ax.plot(data["val_time"], val_line("val_actual"), color="#475569", lw=1.1, label="验证段反馈 T")
ax.plot(data["val_time"], val_line("val_static"), color="#0F766E", ls="--", lw=1.5,
        label="平均气簧曲线")
ax.plot(data["val_time"], val_line("val_direction"), color="#2563EB", lw=1.3,
        label="加方向差异的拟合")
ax.set(title="新 P/P 留出段：差动力矩拟合 RMS 0.425 N·m", xlabel="验证段时间 / s",
       ylabel="T = (髋力矩 − 膝力矩) / 2，N·m", ylim=(0, 25))
ax.legend(fontsize=9, loc="lower left")

ax = axes[1, 0]
names = ["旧记录留出", "新 P/P 外部验证", "中速筛选段", "快速筛选段"]
keys = list(result["transfer_validation"])
x = np.arange(len(keys))
w = .34
values = result["transfer_validation"]
ax.bar(x-w/2, [values[k]["relative_residual_static_only_nm"]["rmse"] for k in keys], w,
       color="#94A3B8", label="只扣平均负载")
bars = ax.bar(x+w/2, [values[k]["relative_residual_static_and_direction_nm"]["rmse"] for k in keys], w,
              color="#2563EB", label="再考虑方向差异")
ax.bar_label(bars, fmt="%.2f", padding=3, fontsize=9)
ax.set_xticks(x, names, fontsize=9)
ax.set(title="仅用旧慢速训练段拟合的跨记录验证\n中/快速仅比较低加速度样本", ylabel="差动力矩残差 RMS / N·m", ylim=(0, 2.8))
ax.legend(fontsize=9, loc="upper right")

ax = axes[1, 1]
v = result["candidate_validation"]["new_pp_holdout"]
x = np.arange(2)
before = [v["joint_effort_before_nm"][a]["rmse"] for a in ("hip", "knee")]
after = [v["joint_effort_minus_static_map_nm"][a]["rmse"] for a in ("hip", "knee")]
ax.bar(x-w/2, before, w, color="#CBD5E1", label="记录中的力矩")
bars = ax.bar(x+w/2, after, w, color="#0F766E", label="离线扣除平均曲线后的残差")
ax.bar_label(bars, fmt="%.2f", padding=3)
ax.set_xticks(x, ["髋电机", "膝电机"])
ax.set(title="新留出段：剩余膝负载仍需反馈与摩擦模型", ylabel="电机侧力矩 RMS / N·m", ylim=(0, 20))
ax.legend(fontsize=9, loc="upper left")

for ax in axes.flat:
    ax.grid(axis="y", alpha=.18)
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)
fig.suptitle("水平悬空左腿：气簧平均负载与方向差异建模\n离线力矩拟合；补偿后的闭环误差尚需实测", fontsize=15)
fig.savefig(OUT / "gas_spring_fit.png", dpi=170)
plt.close(fig)
