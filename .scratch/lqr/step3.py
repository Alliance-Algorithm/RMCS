import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.optimize import minimize
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import step, StateSpace

g = 9.81
M_B, M_W, R_W, J_W = 11.62, 0.40, 0.06, 2*0.0047
I_B = 0.10

def model(L, I_b=I_B, J_w=J_W, m_b=M_B, m_w=M_W, r=R_W):
    M = m_w + J_w/r**2 + m_b; Jb = I_b + m_b*L**2; mL = m_b*L
    det = M*Jb - mL**2
    A = np.zeros((4,4)); B = np.zeros((4,1))
    A[0,1]=1; A[2,3]=1
    A[1,0] = M*m_b*g*L/det
    A[3,0] = -mL*m_b*g*L/det
    B[1,0] = (-M - mL/r)/det
    B[3,0] = (Jb/r + mL)/det
    return A,B

A,B = model(0.26)
def K_of(A,B,q):
    P = solve_continuous_are(A, B, np.diag(q), np.array([[1.0]]))
    return B.T @ P

TARGET = np.array([-17.6,-3.6,-3.0,-2.2])
def cost(p):
    q = np.exp(p)
    K = K_of(A,B,q).ravel()
    if not np.all(np.isfinite(K)) or np.any(K*TARGET <= 0): return 1e6
    return float(np.sum((np.log(np.abs(K/TARGET)))**2))
best = min((minimize(cost, np.random.default_rng(s).uniform(-6, 6, 4), method="Nelder-Mead",
                     options={"maxiter":60000,"fatol":1e-14,"xatol":1e-13}) for s in range(10)),
           key=lambda r: r.fun)
Q = np.exp(best.x); Kfit = K_of(A,B,Q).ravel()
print("fitted Q diag =", np.array2string(Q, precision=4))
print("K fit  =", np.array2string(Kfit, precision=2), " shipped =", TARGET)
print("relative error:", np.array2string(Kfit/TARGET - 1, precision=3))
print("poles with fitted K:", np.array2string(np.sort_complex(np.linalg.eigvals(A - np.outer(B.ravel(), Kfit))), precision=3))

hs = np.linspace(0.22, 0.32, 21); Ks = []
for h in hs:
    Ah, Bh = model(h)
    Ks.append(K_of(Ah, Bh, Q).ravel())
Ks = np.array(Ks)
print("\nK @ h=0.22:", np.array2string(Ks[0], precision=2))
print("K @ h=0.32:", np.array2string(Ks[-1], precision=2))
print("change [%]:", np.array2string(100*(Ks[-1]/Ks[0]-1), precision=1))

fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.2))
G0 = np.array([-17.6,-3.6,-3.0,0.0]); G1 = np.array([-17.6,-3.6,-3.0,-2.2])
for Gv, nm, ls in ((G0,"gain_w = 0","--"), (G1,"gain_w = -2.2","-")):
    sysobj = StateSpace(A - np.outer(B.ravel(), Gv), B, np.eye(4), np.zeros((4,1)))
    t, y = step(sysobj, N=2000)
    ax[0].plot(t, y[:,3], ls, label=f"position, {nm}")
    ax[0].plot(t, np.degrees(y[:,0]), ls, alpha=.55, label=f"pitch [deg], {nm}")
ax[0].set_title("0.1 Nm step: position term on vs off")
ax[0].set_xlabel("t [s]"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=7)

for i, nm in enumerate([r"$K_\theta$", r"$K_{\dot\theta}$", r"$K_v$", r"$K_x$"]):
    ax[1].plot(hs, Ks[:,i], marker=".", label=nm)
ax[1].set_title("same Q/R, gains vs CoM height"); ax[1].set_xlabel("h [m]")
ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
plt.tight_layout(); plt.savefig(".scratch/lqr/lqr.png", dpi=130)
print("saved")
