import numpy as np
from scipy.linalg import solve_continuous_are
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import step

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
G   = np.array([-17.6,-3.6,-3.0,-2.2])
G0  = np.array([-17.6,-3.6,-3.0, 0.0])

def poles(Gv, sign):
    return np.sort_complex(np.linalg.eigvals(A + sign*np.outer(B.ravel(), Gv)))

print("convention u = +G x  (code literal):", np.array2string(poles(G, +1), precision=3))
print("convention u = -G x  (mirrored frame):", np.array2string(poles(G, -1), precision=3))
print()
print("gain_w=0, u=-Gx :", np.array2string(poles(G0, -1), precision=4))
print("gain_w=-2.2,u=-Gx:", np.array2string(poles(G, -1), precision=4))

# --- fit Q diagonals with R=1 (scale-free: LQR K only depends on Q/R) ---
from scipy.optimize import minimize
def K_of(p):
    Q = np.diag(np.exp(p)); R = np.array([[1.0]])
    P = solve_continuous_are(A, B, Q, R)
    return (np.linalg.inv(R) @ B.T @ P).ravel()

TARGET = np.array([-17.6,-3.6,-3.0,-2.2])
def cost(p):
    K = K_of(p)
    if not np.all(np.isfinite(K)) or np.any(K*TARGET <= 0): return 1e6
    return float(np.sum((np.log(np.abs(K/TARGET)))**2))
best = min((minimize(cost, np.random.default_rng(s).normal(size=4), method="Nelder-Mead",
                     options={"maxiter":40000,"fatol":1e-14,"xatol":1e-12}) for s in range(8)),
           key=lambda r: r.fun)
print("\n=== Q/R that reproduces the shipped gains (log-err %.3f) ===" % best.fun)
print("Q diag =", np.array2string(np.exp(best.x), precision=3), " R = 1")
print("K      =", np.array2string(K_of(best.x), precision=2), "  shipped =", TARGET)

# --- figure: (a) step response of pitch, (b) K vs CoM height ---
fig, ax = plt.subplots(1, 2, figsize=(11, 4))
Acl = A - np.outer(B.ravel(), G)
sysm = __import__("scipy.signal", fromlist=["StateSpace"]).StateSpace(Acl, B, np.eye(4), np.zeros((4,1)))
t, y = step(sysm, N=1500)
ax[0].plot(t, np.degrees(y[:,0]), label="pitch (deg) for +0.1 Nm step")
ax[0].plot(t, y[:,2], label="velocity (m/s)")
ax[0].plot(t, y[:,3], label="position (m)")
ax[0].set_title("closed loop, shipped gains, u=-Gx"); ax[0].set_xlabel("t [s]")
ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)

hs = np.linspace(0.22, 0.32, 21); Ks = []
for h in hs:
    Ah, Bh = model(h)
    Ks.append(K_of(best.x))
Ks = np.array(Ks)
for i, nm in enumerate(["K_theta","K_thetadot","K_v","K_x"]):
    ax[1].plot(hs, Ks[:,i], label=nm)
ax[1].set_title("LQR gains vs CoM height h (same Q/R)"); ax[1].set_xlabel("h [m]")
ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
plt.tight_layout(); plt.savefig(".scratch/lqr/lqr.png", dpi=130)
print("\nsaved .scratch/lqr/lqr.png")
print("K at h=0.22:", np.array2string(Ks[0], precision=2))
print("K at h=0.32:", np.array2string(Ks[-1], precision=2))
