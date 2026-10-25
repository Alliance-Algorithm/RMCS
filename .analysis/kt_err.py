import numpy as np
T=np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
            9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
I=np.arange(50,1051,50)/100.0
print("candidate single K: max|err| Nm, %FS, worst rel%")
for K in [1.50,1.558,1.645,1.701,1.759,1.79,1.88]:
    e=K*I-T
    j=np.argmax(abs(e))
    print(f"K={K:5.3f}  max|e|={abs(e).max():5.2f} Nm  ({abs(e).max()/18.6*100:4.1f}% FS)  worst rel {e[j]/T[j]*100:+5.1f}% @ {I[j]:.1f}A  rms={np.sqrt(np.mean(e**2)):.2f}")
print()
# piecewise secant by range
for lo,hi in [(0.5,2),(2,5),(5,8),(8,10.5)]:
    m=(I>=lo)&(I<=hi)
    print(f"{lo}-{hi}A: sumT/sumI = {T[m].sum()/I[m].sum():.3f}")
print()
a,b=0.02241,1.54801
def inv(Tq):
    s=np.sign(Tq); x=abs(Tq)
    return s*(-b+np.sqrt(b*b+4*a*x))/(2*a)
for tq in [0.76,3.08,8.25,12.75,18.6,-12.75]:
    print(f"T={tq:6.2f} -> I={inv(tq):6.2f} A")
print()
# validation: leave-out / residual of quadratic
Tf=a*I**2+b*I
print("quad max resid", np.max(np.abs(Tf-T)), "rms", np.sqrt(np.mean((Tf-T)**2)))
