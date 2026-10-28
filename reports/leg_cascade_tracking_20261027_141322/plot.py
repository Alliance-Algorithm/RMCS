"""Render exported numeric results using the host's installed Matplotlib."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
data = np.load(OUT/"plot_data.npz")
result = json.loads((OUT/"metrics.json").read_text())
comparison = result["comparison"]
font_path = Path("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc")
if font_path.exists():
    fm.fontManager.addfont(str(font_path))
    plt.rcParams["font.family"] = fm.FontProperties(fname=str(font_path)).get_name()
plt.rcParams.update({"font.size":11,"axes.unicode_minus":False,"figure.facecolor":"white"})
fig, axes = plt.subplots(2,2,figsize=(14,8.2),layout="constrained")
sl = slice(None,None,8)
xx = data["time"][sl]
ax=axes[0,0]
ax.plot(xx,data["beta_ref"][sl],label="参考夹角",color="#4B5563",lw=1.6,ls="--")
ax.plot(xx,data["beta_actual"][sl],label="实际夹角（FK）",color="#2563EB",lw=1.6)
ax.set(title="机械夹角跟踪：参考 45–100°，实际约 54.9–101.4°",xlabel="慢速段时间 / s",ylabel="大小腿夹角 / °")
ax.legend(loc="upper right",ncol=2,fontsize=10)
ax=axes[0,1]
for column,label,color in zip(("hip_error","knee_error"),("髋：参考−实际","膝：参考−实际"),("#0F766E","#D97706")):
    ax.plot(xx,data[column][sl],label=label,color=color,lw=1.1)
ax.axhline(0,color="#9CA3AF",lw=.8)
ax.set(title="电机模型角误差：方向固定，随气簧姿态变化",xlabel="慢速段时间 / s",ylabel="角度误差 / °")
ax.legend(loc="upper right",ncol=2,fontsize=10)
ax=axes[1,0]
ax.plot(xx,data["knee_vref"][sl],color="#4B5563",ls="--",lw=1.4,label="轨迹参考速度")
ax.plot(xx,data["knee_v"][sl],color="#2563EB",lw=.9,alpha=.85,label="实测速度")
ax.set(title="膝轨迹速度误差 RMS 0.134 rad/s",xlabel="慢速段时间 / s",ylabel="速度 / rad/s")
ax.legend(loc="upper right",ncol=2,fontsize=10)
ax=axes[1,1]
names=("髋位置","膝位置","大小腿夹角")
ix=np.arange(3);w=.34
ax.bar(ix-w/2,[r["old_rmse_deg"] for r in comparison],w,color="#CBD5E1",label="原位置 PD 120/4")
bars=ax.bar(ix+w/2,[r["new_rmse_deg"] for r in comparison],w,color="#2563EB",label="级联 P/P 40 × 4")
ax.bar_label(bars,fmt="%.2f°",padding=3)
ax.set_xticks(ix,names)
ax.set(title="同为 8 s 往返，stage 1–11 的误差 RMS",ylabel="RMS / °",ylim=(0,13))
ax.legend(loc="upper left",fontsize=10)
for ax in axes.flat:
    ax.grid(alpha=.18,axis="y")
    ax.set_axisbelow(True)
    ax.spines[["top","right"]].set_visible(False)
fig.suptitle("左腿级联 P/P 闭环误差：慢速阶段有改善，气簧负载偏差仍明显",fontsize=16)
fig.savefig(OUT/"tracking_error.png",dpi=170)
plt.close(fig)
