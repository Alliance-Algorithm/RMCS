import numpy as np
from scipy.linalg import solve_continuous_are
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import step, StateSpace

g=9.81; M_B,M_W,R_W,J_W=11.62,0.40,0.06,2*0.0047; I_B=0.10
def model(L,I_b=I_B,J_w=J_W,m_b=M_B,m_w=M_W,r=R_W):
    M=m_w+J_w/r**2+m_b; Jb=I_b+m_b*L**2; mL=m_b*L; det=M*Jb-mL**2
    A=np.zeros((4,4)); B=np.zeros((4,1)); A[0,1]=1; A[2,3]=1
    A[1,0]=M*m_b*g*L/det; A[3,0]=-mL*m_b*g*L/det
    B[1,0]=(-M-mL/r)/det;  B[3,0]=(Jb/r+mL)/det
    return A,B
th,thd,v,x,u=0.10,1.0,0.60,0.20,3.69
Q=np.diag([1/th**2,1/thd**2,1/v**2,1/x**2]); R=np.array([[1/u**2]])
def K_of(A,B):
    P=solve_continuous_are(A,B,Q,R); return (np.linalg.inv(R)@B.T@P).ravel()

A,B=model(0.26)
G0=np.array([-17.6,-3.6,-3.0,0.0]); G1=np.array([-17.6,-3.6,-3.0,-2.2])
fig,ax=plt.subplots(1,3,figsize=(14.5,4.0))

for Gv,nm,ls in ((G0,"gain_w = 0","--"),(G1,"gain_w = -2.2","-")):
    s=StateSpace(A-np.outer(B.ravel(),Gv),B,np.eye(4),np.zeros((4,1)))
    t,y=step(s,N=1500)
    ax[0].plot(t,y[:,3],ls,label=nm)
ax[0].axhline(0,color="k",lw=.6); ax[0].set_ylim(-0.05,0.25)
ax[0].set_title("0.1 Nm step -> position"); ax[0].set_xlabel("t [s]"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)

for Gv,nm,mk in ((G0,"gain_w = 0","o"),(G1,"gain_w = -2.2","s")):
    p=np.linalg.eigvals(A-np.outer(B.ravel(),Gv))
    ax[1].scatter(p.real,p.imag,marker=mk,label=nm)
ax[1].axvline(0,color="r",lw=.8,ls=":"); ax[1].set_title("closed-loop poles")
ax[1].set_xlabel("Re"); ax[1].set_ylabel("Im"); ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)

hs=np.linspace(0.22,0.32,21); Ks=np.array([K_of(*model(h)) for h in hs])
ax2=ax[2]
for i,nm,c in ((0,r"$K_\theta$","tab:blue"),(1,r"$K_{\dot\theta}$","tab:orange")):
    ax2.plot(hs,Ks[:,i],marker=".",color=c,label=nm)
ax2.set_xlabel("h [m]"); ax2.set_ylabel("pitch gains"); ax2.grid(alpha=.3)
ax3=ax2.twinx()
for i,nm,c in ((2,r"$K_v$","tab:green"),(3,r"$K_x$","tab:red")):
    ax3.plot(hs,Ks[:,i],marker="x",color=c,label=nm)
ax3.set_ylabel("velocity / position gains")
h1,l1=ax2.get_legend_handles_labels(); h2,l2=ax3.get_legend_handles_labels()
ax2.legend(h1+h2,l1+l2,fontsize=8,loc="center right")
ax2.set_title("same Q/R, gains vs CoM height")
plt.tight_layout(); plt.savefig(".scratch/lqr/lqr.png",dpi=130)
print("theta gain change %.1f%%, v gain change %.2f%%"%(100*(Ks[-1,0]/Ks[0,0]-1),100*(Ks[-1,2]/Ks[0,2]-1)))
