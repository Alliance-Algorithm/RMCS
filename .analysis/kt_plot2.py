import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

cands=["Noto Sans CJK SC","Noto Sans CJK JP","Source Han Sans CN","DejaVu Sans"]
avail={f.name for f in fm.fontManager.ttflist}
use=next(c for c in cands if c in avail)
plt.rcParams["font.sans-serif"]=[use]; plt.rcParams["axes.unicode_minus"]=False

T=np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
            9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
Ibus=np.array([0.09,0.10,0.11,0.14,0.16,0.21,0.25,0.31,0.36,0.43,0.50,
               0.58,0.69,0.78,0.88,1.03,1.11,1.19,1.36,1.46,1.55])
I=np.arange(50,1051,50)/100.0
c2,c1,c0=np.polyfit(I,T,2)
A=np.vstack([np.ones_like(T),T**2]).T;(a,bb),*_=np.linalg.lstsq(A,Ibus,rcond=None)
A2=np.vstack([I**2,np.ones_like(I)]).T;(m,b2),*_=np.linalg.lstsq(A2,Ibus,rcond=None)
r2b=1-np.sum((Ibus-(m*I**2+b2))**2)/np.sum((Ibus-Ibus.mean())**2)
tang=lambda x: 2*c2*x+c1
rat=lambda x: c2*x+c1
print(f"quad R2={1-np.sum((T-(c2*I**2+c1*I+c0))**2)/np.sum((T-T.mean())**2):.4f}  resid={np.std(T-(c2*I**2+c1*I+c0)):.3f}")
print(f"Ibus-I2 fit R2={r2b:.4f} R={m*24/2:.4f} P0={b2*24:.2f}")

fig,ax=plt.subplots(1,3,figsize=(13.4,4.0),dpi=160)
fig.subplots_adjust(wspace=0.34,left=0.055,right=0.985,top=0.84,bottom=0.16)
xx=np.linspace(0,11,300)

# 1
ax[0].scatter(I,T,s=26,c="#1f4e79",zorder=4,label="实测点")
ax[0].plot(xx,c2*xx**2+c1*xx+c0,"-",c="#2e8b57",lw=1.8,label="二次拟合（过原点，略上凸）")
ax[0].plot([0,7.5],[0,12.75],":",c="#555555",lw=1.3,label="割线 T/I = 1.70")
xs=np.linspace(6.0,9.0,10)
ax[0].plot(xs,12.75+tang(7.5)*(xs-7.5),"--",c="#d2691e",lw=1.5,label="切线 dT/dI = 1.88")
ax[0].set_xlabel("相电流 / q 轴电流 (A)");ax[0].set_ylabel("输出扭矩 (N·m)")
ax[0].set_title("① Kt 是切线斜率，不是 T/I",fontsize=11)
ax[0].legend(fontsize=7.6,loc="upper left");ax[0].grid(alpha=.25)
ax[0].annotate("额定点\n7.5 A / 12.75 N·m",xy=(7.5,12.75),xytext=(8.4,9.0),fontsize=8,
               arrowprops=dict(arrowstyle="->",lw=.9),color="#333333")

# 2
tt=np.linspace(0.4,19,300)
ax[1].scatter(T,T/Ibus,s=26,c="#1f4e79",zorder=4,label="实测 T / 母线电流")
ax[1].plot(tt,tt/(a+bb*tt**2),"-",c="#c00000",lw=1.5,label="堵转模型 P$_0$/V + (2R/V)I$^2$")
pk=np.sqrt(a/bb);ax[1].axvline(pk,ls=":",c="gray",lw=1)
ax[1].annotate(f"峰值 ≈{pk:.1f} N·m：固定损耗被摊薄 +\n铜耗上升，与 Kt 无关",xy=(pk,22.2),
               xytext=(8.6,21.4),fontsize=8,bbox=dict(fc="#fff8e1",ec="#d4a017",lw=.8))
ax[1].set_xlabel("输出扭矩 (N·m)");ax[1].set_ylabel("T / 母线电流 (N·m/A)")
ax[1].set_title("② 用母线电流算：8.4 → 24 → 12",fontsize=11)
ax[1].legend(fontsize=7.6,loc="lower center");ax[1].grid(alpha=.25)

# 3
ax[2].scatter(I,T/I,s=26,c="#1f4e79",zorder=4,label="逐点 T/I")
ax[2].plot(xx,rat(xx),"-",c="#c00000",lw=1.5,label="同一条凸曲线的 T/I")
ax[2].set_xlabel("相电流 (A)");ax[2].set_ylabel("逐点 T / I (N·m/A)")
ax[2].set_title("③ 逐点比值：1.52 → 1.77，仍在爬",fontsize=11)
ax[2].set_ylim(1.30,1.90);ax[2].legend(fontsize=7.6,loc="lower right");ax[2].grid(alpha=.25)
ax[2].annotate("等效减速器效率\n≈0.77 → 0.90",xy=(6.8,1.60),xytext=(2.2,1.34),fontsize=8,
               arrowprops=dict(arrowstyle="->",lw=.9),color="#444444")

fig.suptitle("LK5010i36 堵转实测：为什么“转矩常数”看起来不是定值",fontsize=12.5,y=0.965)
fig.savefig(".analysis/lk5010_kt.png")
print("ok")
