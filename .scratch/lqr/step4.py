import numpy as np
from scipy.linalg import solve_continuous_are
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import step, StateSpace

g=9.81; M_B,M_W,R_W,J_W = 11.62,0.40,0.06,2*0.0047; I_B=0.10
def model(L,I_b=I_B,J_w=J_W,m_b=M_B,m_w=M_W,r=R_W):
    M=m_w+J_w/r**2+m_b; Jb=I_b+m_b*L**2; mL=m_b*L; det=M*Jb-mL**2
    A=np.zeros((4,4)); B=np.zeros((4,1)); A[0,1]=1; A[2,3]=1
    A[1,0]=M*m_b*g*L/det; A[3,0]=-mL*m_b*g*L/det
    B[1,0]=(-M-mL/r)/det;  B[3,0]=(Jb/r+mL)/det
    return A,B

# Bryson: "acceptable" deviations -> Q, "acceptable" effort -> R
th, thd, v, x, u = 0.10, 1.0, 0.60, 0.20, 3.69
Q = np.diag([1/th**2, 1/thd**2, 1/v**2, 1/x**2]); R = np.array([[1/u**2]])
def K_of(A,B,Q=Q,R=R):
    P = solve_continuous_are(A,B,Q,R); return (np.linalg.inv(R)@B.T@P).ravel()

A,B = model(0.26)
Kb = K_of(A,B)
print("Bryson Q diag:", np.diag(Q), " R:", R[0,0])
print("K (Bryson) =", np.array2string(Kb, precision=2))
print("shipped    =", np.array2string(np.array([-17.6,-3.6,-3.0,-2.2]), precision=2))
print("Bryson/shipped ratio:", np.array2string(Kb/np.array([-17.6,-3.6,-3.0,-2.2]), precision=2))

hs=np.linspace(0.22,0.32,21); Ks=np.array([K_of(*model(h)) for h in hs])
print("\nK @h=0.22:", np.array2string(Ks[0],precision=2))
print("K @h=0.32:", np.array2string(Ks[-1],precision=2))
print("change [%]:", np.array2string(100*(Ks[-1]/Ks[0]-1),precision=1))
# also: how does the unstable pole move with h
print("open-loop lambda vs h:", np.array2string(np.array([np.max(np.linalg.eigvals(model(h)[0]).real) for h in hs[::5]]), precision=2))

fig,ax=plt.subplots(1,2,figsize=(11.5,4.2))
for Gv,nm,ls in ((np.array([-17.6,-3.6,-3.0,0.0]),"gain_w = 0","--"),(np.array([-17.6,-3.6,-3.0,-2.2]),"gain_w = -2.2","-")):
    s=StateSpace(A-np.outer(B.ravel(),Gv),B,np.eye(4),np.zeros((4,1)))
    t,y=step(s,N=2500)
    ax[0].plot(t,y[:,3],ls,label=f"x [m], {nm}")
    ax[0].plot(t,np.degrees(y[:,0]),ls,alpha=.5,label=f"pitch [deg], {nm}")
ax[0].set_title("0.1 Nm step, shipped gains"); ax[0].set_xlabel("t [s]"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=7)

for i,nm in enumerate([r"$K_\theta$",r"$K_{\dot\theta}$",r"$K_v$",r"$K_x$"]):
    ax[1].plot(hs,Ks[:,i],marker=".",label=nm)
ax[1].set_title("Bryson LQR: gains vs CoM height h"); ax[1].set_xlabel("h [m]")
ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
plt.tight_layout(); plt.savefig(".scratch/lqr/lqr.png",dpi=130)
print("\nsaved")
