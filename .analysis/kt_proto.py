import numpy as np
T=np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
            9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
n=np.arange(50,1051,50)
Ibus=np.array([0.09,0.10,0.11,0.14,0.16,0.21,0.25,0.31,0.36,0.43,0.50,
               0.58,0.69,0.78,0.88,1.03,1.11,1.19,1.36,1.46,1.55])
I=n*66.0/4096.0
print("Imax=%.4f A, step=%.4f A"%(I[-1],I[1]-I[0]))
sec=T/I
print("secant min/max/mean: %.3f %.3f %.3f"%(sec.min(),sec.max(),sec.mean()))
a,b,c=np.polyfit(I,T,2)
print("quad: %.6f I^2 + %.6f I + %.6f"%(a,b,c))
yh=a*I**2+b*I+c
print("R2=%.5f resid_max=%.3f"%(1-np.sum((T-yh)**2)/np.sum((T-T.mean())**2),np.max(np.abs(T-yh))))
print("tangent@rated(%.2fA)=%.4f  @max=%.4f"%(I[14],2*a*I[14]+b,2*a*I[-1]+b))
print("secant@rated=%.4f @max=%.4f"%(T[14]/I[14],T[-1]/I[-1]))
best=None
for K in np.arange(0.90,1.20,0.0005):
    e=np.max(np.abs(K*I-T))
    if best is None or e<best[1]: best=(K,e)
print("minimax K=%.4f maxerr=%.3f (%.1f%%FS)"%(best[0],best[1],best[1]/18.6*100))
Krms=min(np.arange(0.90,1.20,0.0005),key=lambda K:np.sqrt(np.mean((K*I-T)**2)))
print("min-rms K=%.4f rms=%.3f"%(Krms,np.sqrt(np.mean((Krms*I-T)**2))))
for lo,hi in [(50,200),(200,500),(500,800),(800,1050)]:
    m=(n>=lo)&(n<=hi)
    print(f"n {lo}-{hi} ({lo*66/4096:.2f}-{hi*66/4096:.2f} A): secant={T[m].sum()/I[m].sum():.4f}")
R=0.082
Pcu=1.5*I[-1]**2*R
print("cu loss @max=%.1f W, Ibus=%.3f A (table 1.55)"%(Pcu,(Pcu+2.1)/24))
print("R needed under old 10.5A assumption: %.3f ohm"%(35.1/(1.5*10.5**2)))
Kt_m=1.169/(36*0.9); print("Kt_motor(eta .9)=%.4f -> Kv=%.1f"%(Kt_m,9.549/Kt_m))
Kt_m2=1.055/(36*0.9); print("Kt_motor from secant=%.4f -> Kv=%.1f"%(Kt_m2,9.549/Kt_m2))
def inv(t):
    s=np.sign(t); return s*(-b+np.sqrt(b*b+4*a*abs(t)))/(2*a)
for t in [0.76,3.08,8.25,12.75,18.6]: print("T=%.2f -> I=%.2f A -> n=%.0f"%(t,inv(t),inv(t)*4096/66))
print("raw form: T=%.4e n^2 + %.6f n"%(a*(66/4096)**2,b*66/4096))
