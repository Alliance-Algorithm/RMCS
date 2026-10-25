import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

cands = ["Noto Sans CJK SC","Noto Sans CJK JP","Source Han Sans CN","Noto Sans CJK TC","DejaVu Sans"]
avail = {f.name for f in fm.fontManager.ttflist}
use = next((c for c in cands if c in avail), "DejaVu Sans")
print("font:", use)
plt.rcParams["font.sans-serif"] = [use]
plt.rcParams["axes.unicode_minus"] = False

T = np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
              9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
Ibus = np.array([0.09,0.10,0.11,0.14,0.16,0.21,0.25,0.31,0.36,0.43,0.50,
                 0.58,0.69,0.78,0.88,1.03,1.11,1.19,1.36,1.46,1.55])
cmd = np.arange(50,1051,50); Iq = cmd/100.0
def linfit(x,y):
    A=np.vstack([x,np.ones_like(x)]).T;(k,b),*_=np.linalg.lstsq(A,y,rcond=None)
    return k,b,1-np.sum((y-(k*x+b))**2)/np.sum((y-y.mean())**2)
k,b,r2 = linfit(Iq,T)
k1,_,_ = linfit(Iq[Iq<=6],T[Iq<=6]); k2,_,_ = linfit(Iq[Iq>=6.5],T[Iq>=6.5])
A=np.vstack([np.ones_like(T),T**2]).T;(a,bb),*_=np.linalg.lstsq(A,Ibus,rcond=None)

fig, ax = plt.subplots(1,3,figsize=(13.2,3.9),dpi=160)
fig.subplots_adjust(wspace=0.32,left=0.06,right=0.985,top=0.86,bottom=0.15)

xx=np.linspace(0,11,200)
ax[0].scatter(Iq,T,s=26,c="#1f4e79",zorder=3,label="实测点")
ax[0].plot(xx,k*xx+b,"--",c="#c00000",lw=1.6,label=f"整段线性拟合 Kt={k:.2f}")
ax[0].plot(xx,1.548*xx+0.0224*xx**2,"-",c="#2e8b57",lw=1.6,label="二次拟合（略上弯）")
ax[0].set_xlabel("相电流 / q 轴电流 (A)"); ax[0].set_ylabel("输出扭矩 (N·m)")
ax[0].set_title("① 斜率才是 Kt，比值不是",fontsize=11)
ax[0].legend(fontsize=8,loc="upper left"); ax[0].grid(alpha=.25)
ax[0].annotate(f"局部斜率\n2–6 A: {k1:.2f}\n6.5–10.5 A: {k2:.2f}",xy=(4.8,6.5),fontsize=8.5,
               bbox=dict(fc="#fff8e1",ec="#d4a017",lw=.8))

ax[1].scatter(Iq,T/Iq,s=26,c="#1f4e79",zorder=3)
ax[1].axhline(k,ls="--",c="#c00000",lw=1.4,label=f"斜率 Kt={k:.2f}")
ax[1].set_ylim(1.3,1.9); ax[1].set_xlabel("相电流 (A)"); ax[1].set_ylabel("逐点 T / I")
ax[1].set_title("② 逐点 T/I：1.48 → 1.79（±9%）",fontsize=11)
ax[1].legend(fontsize=8,loc="lower right"); ax[1].grid(alpha=.25)

tt=np.linspace(0.5,19,300)
ax[2].scatter(T,T/Ibus,s=26,c="#1f4e79",zorder=3,label="实测 T / 母线电流")
ax[2].plot(tt,tt/(a+bb*tt**2),"-",c="#c00000",lw=1.4,label="堵转模型 I$_{bus}$=a+bT²")
pk=np.sqrt(a/bb)
ax[2].axvline(pk,ls=":",c="gray",lw=1)
ax[2].annotate(f"峰值 ~{pk:.1f} N·m\n只是损耗结构，不是 Kt",xy=(pk,24),xytext=(8.0,23.2),fontsize=8.5,
               arrowprops=dict(arrowstyle="->",lw=.9))
ax[2].set_xlabel("输出扭矩 (N·m)"); ax[2].set_ylabel("T / 母线电流 (N·m/A)")
ax[2].set_title("③ 用母线电流算：驼峰形，8.4–24",fontsize=11)
ax[2].legend(fontsize=8,loc="upper right"); ax[2].grid(alpha=.25)

fig.suptitle("LK5010i36 堵转实测：为什么“转矩常数”看起来不是定值",fontsize=12.5,y=0.975)
fig.savefig(".analysis/lk5010_kt.png")
print("saved", k, b, r2, k1, k2, a, bb, pk)
