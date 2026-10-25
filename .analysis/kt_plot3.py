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
n=np.arange(50,1051,50); I=n*66/4096
Ibus=np.array([0.09,0.10,0.11,0.14,0.16,0.21,0.25,0.31,0.36,0.43,0.50,
               0.58,0.69,0.78,0.88,1.03,1.11,1.19,1.36,1.46,1.55])
a,b,c=np.polyfit(I,T,2)
A=np.vstack([np.ones_like(T),T**2]).T;(p0,p1),*_=np.linalg.lstsq(A,Ibus,rcond=None)
Ir=I[14];Tr=T[14]
print("rated",Ir,Tr,Tr/Ir,2*a*Ir+b)

fig,ax=plt.subplots(1,3,figsize=(13.4,4.0),dpi=160)
fig.subplots_adjust(wspace=0.34,left=0.055,right=0.985,top=0.83,bottom=0.16)
xx=np.linspace(0,17.2,300)
ax[0].scatter(I,T,s=26,c="#1f4e79",zorder=4,label="实测点")
ax[0].plot(xx,a*xx**2+b*xx+c,"-",c="#2e8b57",lw=1.8,label="二次拟合")
ax[0].plot([0,Ir],[0,Tr],":",c="#555555",lw=1.3,label="割线 = 1.055")
xs=np.linspace(10.2,14.0,10)
ax[0].plot(xs,Tr+(2*a*Ir+b)*(xs-Ir),"--",c="#d2691e",lw=1.5,label="切线 = 1.169")
ax[0].set_xlabel("iq (A)   [MG: 66/4096 A/LSB]");ax[0].set_ylabel("关节输出扭矩 (N·m)")
ax[0].set_title("① Kt 是切线，不是 T/I",fontsize=11)
ax[0].legend(fontsize=7.6,loc="upper left");ax[0].grid(alpha=.25)
ax[0].annotate("额定点 750 counts\n= 12.08 A / 12.75 N·m",xy=(Ir,Tr),xytext=(12.6,8.4),fontsize=8,
               arrowprops=dict(arrowstyle="->",lw=.9),color="#333333")

tt=np.linspace(0.4,19,300)
ax[1].scatter(T,T/Ibus,s=26,c="#1f4e79",zorder=4,label="实测 T / 母线电流")
ax[1].plot(tt,tt/(p0+p1*tt**2),"-",c="#c00000",lw=1.5,label="堵转模型 P0/V + (1.5R/V)iq²")
pk=np.sqrt(p0/p1);ax[1].axvline(pk,ls=":",c="gray",lw=1)
ax[1].annotate("峰值 ≈5.2 N·m：固定损耗被摊薄 +\n铜耗上升，与 Kt 无关",xy=(pk,22.2),
               xytext=(8.6,21.4),fontsize=8,bbox=dict(fc="#fff8e1",ec="#d4a017",lw=.8))
ax[1].set_xlabel("关节输出扭矩 (N·m)");ax[1].set_ylabel("T / 母线电流 (N·m/A)")
ax[1].set_title("② 用母线电流算：8.4 → 24 → 12",fontsize=11)
ax[1].legend(fontsize=7.6,loc="lower center");ax[1].grid(alpha=.25)

ax[2].scatter(I,T/I,s=26,c="#1f4e79",zorder=4,label="逐点 T/I")
ax[2].plot(xx,a*xx+b,"-",c="#c00000",lw=1.5,label="同一条凸曲线的 T/I")
ax[2].set_xlabel("iq (A)");ax[2].set_ylabel("逐点 T / iq (N·m/A)")
ax[2].set_title("③ 逐点比值：0.92 → 1.11",fontsize=11)
ax[2].set_ylim(0.85,1.20);ax[2].legend(fontsize=7.6,loc="lower right");ax[2].grid(alpha=.25)
ax[2].annotate("轻载效率低、重载效率高\n（传动效率随载荷爬）",xy=(9.0,1.03),xytext=(1.2,1.14),fontsize=8,
               arrowprops=dict(arrowstyle="->",lw=.9),color="#444444")

fig.suptitle("MG5010-i36 堵转实测（iq = 66/4096 A/LSB，满量程 16.92 A）",fontsize=12.5,y=0.965)
fig.savefig(".analysis/lk5010_kt_protocol.png")
print("ok")
