import numpy as np
T = np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
              9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
Ibus = np.array([0.09,0.10,0.11,0.14,0.16,0.21,0.25,0.31,0.36,0.43,0.50,
                 0.58,0.69,0.78,0.88,1.03,1.11,1.19,1.36,1.46,1.55])
I = np.arange(50,1051,50)/100.0
c2,c1,c0 = np.polyfit(I,T,2)
print(f"quad: {c2:.5f} I^2 + {c1:.5f} I + {c0:.4f}")
for x in [0.5,2,5,7.5,10.5]:
    print(f"I={x:4.1f}  secant={np.interp(x,I,T)/x:.3f}  tangent={2*c2*x+c1:.3f}")
A=np.vstack([I**2,np.ones_like(I)]).T
(m,b),*_=np.linalg.lstsq(A,Ibus,rcond=None)
fit=m*I**2+b; r2=1-np.sum((Ibus-fit)**2)/np.sum((Ibus-Ibus.mean())**2)
print(f"Ibus={m:.5f} I^2 + {b:.5f}, R2={r2:.4f}, R2ph={m*24/2:.4f} ohm, P0={b*24:.2f} W")
print("resid Ibus:", np.round(Ibus-fit,3))
print("pred@10.5A:", m*110.25+b)
