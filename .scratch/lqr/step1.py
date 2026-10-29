import numpy as np
from scipy.linalg import solve_continuous_are, eigvals
from scipy.optimize import minimize

g = 9.81
M_B, M_W = 11.62, 0.40
R_W, J_W = 0.06, 2 * 0.0047
L_COM, I_B = 0.26, 0.10

def model(L=L_COM, I_b=I_B, J_w=J_W, m_b=M_B, m_w=M_W, r=R_W):
    """State x = [theta, theta_dot, v, x], input u = wheel torque.
       Reduced model: wheel (rolling, reflected inertia) + rigid body pivoting about the axle."""
    M = m_w + J_w / r**2 + m_b
    Jb = I_b + m_b * L**2
    mL = m_b * L
    det = M * Jb - mL**2
    A = np.zeros((4, 4)); B = np.zeros((4, 1))
    A[0, 1] = 1.0; A[2, 3] = 1.0
    A[1, 0] = M * m_b * g * L / det
    A[3, 0] = -mL * m_b * g * L / det
    B[1, 0] = (-M - mL / r) / det
    B[3, 0] = (Jb / r + mL) / det
    return A, B

def lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.inv(R) @ B.T @ P

A, B = model()
print("=== reduced model ===")
print("A =\n", np.array2string(A, precision=3, suppress_small=True))
print("B =\n", B.ravel())
ev = np.linalg.eigvals(A)
print("open-loop poles:", np.array2string(np.sort_complex(ev), precision=3))
print("unstable root lambda = %.2f rad/s  (doc quotes ~11.6)" % max(abs(ev)))

TARGET = np.array([-17.6, -3.6, -3.0, -2.2])
print("\n=== Bryson-style Q/R (state scales: 0.1 rad, 1 rad/s, 1 m/s, 1 m) ===")
Q = np.diag([1/0.1**2, 1/1.0**2, 1/1.0**2, 1/1.0**2]); R = np.array([[1/3.0**2]])
K = lqr(A, B, Q, R)
print("K =", np.array2string(K.ravel(), precision=2))
print("target =", TARGET, " ratio =", np.array2string(K.ravel()/TARGET, precision=2))

# fit Q=diag(exp p[:4]), R=exp p[4] to reproduce the shipped gains
def make(p):
    Q = np.diag(np.exp(p[:4])); R = np.array([[np.exp(p[4])]])
    return Q, R
def cost(p):
    try:
        Q, R = make(p)
        K = lqr(A, B, Q, R).ravel()
        return float(np.sum(np.log(np.abs(K / TARGET))**2))
    except Exception:
        return 1e9
best = None
for seed in range(6):
    rng = np.random.default_rng(seed)
    p0 = rng.normal(size=5)
    res = minimize(cost, p0, method="Nelder-Mead",
                   options={"maxiter": 20000, "xatol": 1e-10, "fatol": 1e-12})
    if best is None or res.fun < best.fun: best = res
Q, R = make(best.x); Kfit = lqr(A, B, Q, R).ravel()
print("\n=== fitted Q/R reproducing the shipped gains (log-error %.4f) ===" % best.fun)
print("Q diag =", np.array2string(np.diag(Q), precision=4))
print("R = %.3f" % R[0, 0])
print("K =", np.array2string(Kfit, precision=2), "  target =", TARGET)
