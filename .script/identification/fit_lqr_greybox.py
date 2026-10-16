#!/usr/bin/env python3
"""Grey-box identification of the wheel-leg balance plant -> LQR design.

Model (state x = [theta, theta_dot, x_dot], input u = (tau_l+tau_r)/2):
    theta_dot_dot = a1*theta + a2*theta_dot + a3*x_dot + b*u
    x_dot_dot     = c1*theta + c2*theta_dot + c3*x_dot + d*u
    theta               row is the exact kinematic (0,1,0)

Input: a 1000 Hz CSV. Required columns (overridable via CLI):
    t (or time/timestamp[/t_ns]), pitch, wheel_vel_l, wheel_vel_r, tau_l, tau_r
Optional: pitch_rate (gyro) -> used directly; if absent, theta_dot is the
Savitzky-Golay derivative of pitch.

IMPORTANT (closed-loop identifiability): if the recorded wheel torque is a
*linear* function of the state (e.g. a linear balance loop), the plant (A) is
NOT separable from the controller -- only A-BK and B are. The RL policy is
nonlinear and provides the necessary excitation. A steady/symmetric drive with
no excitation is unidentifiable: vary the speed / wobble / chirp while logging.

Output: (A,B), open-loop poles, LQR designs, and <csv>.lqr.json.
Pure numpy (no scipy)."""
import argparse, json, math
import numpy as np


def sg_coeffs(m, n, deriv):
    j = np.arange(-m, m + 1, dtype=float)
    A = np.vander(j, n + 1, increasing=True)
    pinv = np.linalg.pinv(A)
    return math.factorial(deriv) * pinv[deriv]      # len 2m+1


def sg_deriv(y, m, n, deriv, dt):
    c = sg_coeffs(m, n, deriv)
    yc = np.convolve(y, c[::-1], mode="same") / dt ** deriv
    if len(yc) > m:
        yc[:m] = yc[m]
        yc[-m:] = yc[-m - 1]
    return yc


def lqr(A, B, Q, R):
    n = A.shape[0]
    H = np.block([[A, -B @ np.linalg.inv(R) @ B.T], [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    idx = w.real < 0
    P = np.real(V[n:, idx] @ np.linalg.inv(V[:n, idx]))
    return np.linalg.inv(R) @ B.T @ P


def lqr_design(A, B, K):
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--delim", default=",")
    ap.add_argument("--t-col", default=None)
    ap.add_argument("--pitch", default="pitch")
    ap.add_argument("--pitch-rate", default="pitch_rate")
    ap.add_argument("--wvl", default="wheel_vel_l")
    ap.add_argument("--wvr", default="wheel_vel_r")
    ap.add_argument("--tl", default="tau_l")
    ap.add_argument("--tr", default="tau_r")
    ap.add_argument("--wheel-radius", type=float, default=0.06)
    ap.add_argument("--no-header", action="store_true")
    ap.add_argument("--sg-m", type=int, default=8, help="SG half window (samples)")
    ap.add_argument("--sg-n", type=int, default=3, help="SG poly order")
    args = ap.parse_args()

    raw = np.genfromtxt(args.csv, delimiter=args.delim, names=not args.no_header,
                        dtype=float, invalid_raise=False)
    has_hdr = bool(raw.dtype.names)
    names = list(raw.dtype.names) if has_hdr else [str(i) for i in range(np.atleast_2d(raw).shape[1])]

    def col(name):
        if has_hdr:
            return np.asarray(raw[name], float)
        return np.asarray(raw[:, int(name)], float)

    print("columns:", names)
    t = None
    for cand in ([args.t_col] if args.t_col else []) + ["t", "time", "timestamp", "t_ns"]:
        if cand and cand in names:
            t = col(cand)
            if cand.endswith("ns"):
                t = t * 1e-9
            break
    if t is None:
        dt = 0.001
        print("no time column; assuming dt = 1 ms")
    else:
        dt = float(np.median(np.diff(t)))
        print(f"dt = {dt*1000:.4f} ms ({1/dt:.0f} Hz)")
    if not np.isfinite(dt) or dt <= 0:
        dt = 0.001

    theta = col(args.pitch)
    wl, wr = col(args.wvl), col(args.wvr)
    tl_, tr_ = col(args.tl), col(args.tr)
    x_dot = args.wheel_radius * 0.5 * (wl + wr)
    u = 0.5 * (tl_ + tr_)
    n = len(theta)
    print(f"samples = {n}")

    if args.pitch_rate in names:
        theta_dot = col(args.pitch_rate)
        theta_ddot = np.gradient(theta_dot, dt)
        print("theta_dot from gyro column")
    else:
        theta_dot = sg_deriv(theta, args.sg_m, args.sg_n, 1, dt)
        theta_ddot = sg_deriv(theta, args.sg_m, args.sg_n, 2, dt)
        print("theta_dot/ddot from Savitzky-Golay")
    x_ddot = sg_deriv(x_dot, args.sg_m, args.sg_n, 1, dt)

    trim = args.sg_m + 3
    if n > 2 * trim + 200:
        sl = slice(trim, n - trim)
        theta, theta_dot, theta_ddot = theta[sl], theta_dot[sl], theta_ddot[sl]
        x_dot, x_ddot, u = x_dot[sl], x_ddot[sl], u[sl]
    M = np.column_stack([theta, theta_dot, x_dot, u])
    s1, *_ = np.linalg.lstsq(M, theta_ddot, rcond=None)   # [a1 a2 a3 b]
    s2, *_ = np.linalg.lstsq(M, x_ddot, rcond=None)       # [c1 c2 c3 d]
    A = np.array([[0.0, 1.0, 0.0], s1[:3], s2[:3]])
    B = np.array([[0.0], [s1[3]], [s2[3]]])

    print("\nidentified A =\n", np.round(A, 3))
    print("identified B =\n", np.round(B.ravel(), 4))
    print("open-loop poles:", np.round(np.sort_complex(np.linalg.eigvals(A)), 3))

    # one-step residual (accel prediction RMSE)
    r1 = theta_ddot - M @ s1
    r2 = x_ddot - M @ s2
    print(f"accel fit RMSE: theta_ddot={r1.std():.3f} rad/s^2  "
          f"theta_ddot_signal={theta_ddot.std():.3f}  x_ddot={r2.std():.3f}")

    print("\nLQR designs (state [theta, theta_dot, x_dot], u = common Nm):")
    out = {"A": A.tolist(), "B": B.ravel().tolist(), "dt": dt}
    for tag, qth, qx, qxd in [("soft", 50, 5, 1), ("mid", 200, 20, 1), ("hard", 600, 50, 2)]:
        Q = np.diag([qth, 1, qxd]); R = np.array([[1.0]])
        K = lqr(A, B, Q, R)[0]
        P = np.linalg.eigvals(A - B @ K.reshape(1, 3))
        print(f"  {tag:4s} K=[{K[0]:7.2f} {K[1]:6.2f} {K[2]:6.2f}]  closed poles="
              f"{np.round(np.sort_complex(P),2)}  u@1deg={abs(K[0]*np.deg2rad(1)):.2f}Nm")
        out[tag] = {"K": K.tolist(), "poles": [str(p) for p in np.sort_complex(P)]}
    with open(args.csv + ".lqr.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nsaved:", args.csv + ".lqr.json")


if __name__ == "__main__":
    main()
