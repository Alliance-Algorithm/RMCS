import numpy as np

T = np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
              9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
Ibus = np.array([0.09,0.10,0.11,0.14,0.16,0.21,0.25,0.31,0.36,0.43,0.50,
                 0.58,0.69,0.78,0.88,1.03,1.11,1.19,1.36,1.46,1.55])
cmd = np.arange(50,1051,50)
Iq = cmd/100.0

def linfit(x,y):
    A = np.vstack([x, np.ones_like(x)]).T
    (k,b), *_ = np.linalg.lstsq(A,y,rcond=None)
    yh = k*x+b
    ss_res = np.sum((y-yh)**2); ss_tot = np.sum((y-y.mean())**2)
    return k,b,1-ss_res/ss_tot,yh

k,b,r2,yh = linfit(Iq,T)
print(f"ALL   Kt={k:.4f} T0={b:+.4f} R2={r2:.5f} resid_std={np.std(T-yh):.3f} maxres={np.max(np.abs(T-yh)):.3f}")
for lo,hi,name in [(0,5.5,"0.5-5.5A"),(6,10.5,"6-10.5A"),(2,6,"2-6A"),(6.5,10.5,"6.5-10.5A")]:
    m=(Iq>=lo)&(Iq<=hi); kk,bb,rr,yy=linfit(Iq[m],T[m])
    print(f"{name:10s} Kt={kk:.4f} T0={bb:+.4f} R2={rr:.4f} n={m.sum()}")
p = np.polyfit(Iq,T,2); print("quad:",p)
print("per-point:")
for c,i,t in zip(cmd,Iq,T):
    print(f"{c:5d} {i:5.2f} {t:6.2f} {t/i:6.3f} {t/Ibus[list(cmd).index(c)]:6.2f}")
print("ratio T/Iq mean",np.mean(T/Iq),"std",np.std(T/Iq),"min",np.min(T/Iq),"max",np.max(T/Iq))

A = np.vstack([np.ones_like(T), T**2]).T
(a,bb), *_ = np.linalg.lstsq(A,Ibus,rcond=None)
Ih=a+bb*T**2; r2b=1-np.sum((Ibus-Ih)**2)/np.sum((Ibus-Ibus.mean())**2)
print(f"\nIbus = {a:.5f} + {bb:.6f} T^2  R2={r2b:.4f}")
R = bb*24*k*k/2
print(f"implied R_phase={R:.4f} ohm, P0={a*24:.2f} W, peak T/Ibus at T={np.sqrt(a/bb):.2f}")
print("T/Ibus:", np.array2string(T/Ibus, precision=2))
print("T/Iq :", np.array2string(T/Iq, precision=3))
print("Kt_motor(eta .85):",k/36/0.85,"Kv:",9.549/(k/36/0.85))
print("Kt_motor(no eta):",k/36,"Kv:",9.549/(k/36))
