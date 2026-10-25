import numpy as np
T=np.array([0.76,1.48,2.47,3.08,3.85,4.96,5.95,6.65,7.55,8.25,9.20,
            9.75,11.17,11.80,12.75,13.50,15.20,15.75,16.70,17.90,18.60])
I=np.arange(50,1051,50)/100.0
a,b=0.02241,1.54801
for K in [1.558,1.70,1.79,1.88]:
    e=K*I-T
    rel=e/T*100
    print(f"K={K:4.2f}: max|e|={abs(e).max():.2f} Nm ({abs(e).max()/18.6*100:.1f}%FS), max rel={rel[np.argmax(abs(rel))]:+.1f}% @{I[np.argmax(abs(rel))]:.1f}A")
    print("   err@0.5,2,5,7.5,10.5:", np.round(np.interp([0.5,2,5,7.5,10.5],I,e),2))
e=(a*I**2+b*I)-T
print(f"quad: max|e|={abs(e).max():.2f} Nm ({abs(e).max()/18.6*100:.1f}%FS), rms={np.sqrt(np.mean(e**2)):.2f}")
print("   err:", np.round(e,2))
