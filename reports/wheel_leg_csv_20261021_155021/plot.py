"""Plot the observed failed transitions and replayed compensation gate."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
a=np.load(root/'derived.npz')
p=np.load(root/'replay.npz')['output']
summary=json.loads((root/'analysis.json').read_text())
plt.rcParams.update({'font.size':10,'figure.dpi':150,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(4,2,figsize=(12,10.5),layout='constrained')
for col,episode in enumerate([2,4]):
    e=summary['episodes'][episode-1]
    ids=np.arange(e['start'],e['end']+1)
    ids=ids[a['phase'][ids]>=0]
    t=a['t'][ids]-a['t'][ids[0]]
    for leg in [0,1]:
        axes[0,col].plot(t,a['knee'][ids,leg],label=['Left leg','Right leg'][leg])
        axes[1,col].plot(t,a['error'][ids,leg]*1000)
    axes[0,col].axhspan(40,41,color='#77a086',alpha=.25,label='40 to 41 deg transfer window')
    axes[0,col].set_ylim(38,110)
    axes[0,col].set_ylabel('Reconstructed knee angle (deg)')
    axes[0,col].legend(fontsize=8)
    axes[1,col].axhspan(-5,5,color='#77a086',alpha=.25)
    axes[1,col].axhline(0,color='0.4',lw=.8)
    axes[1,col].set_ylabel('Wheel fore/aft error to COM (mm)')
    axes[2,col].plot(t,p[ids,24],color='#a04036',label='Max final-target joint error')
    axes[2,col].axhline(.15,color='#446c50',ls=':',label='Integral enable bound: 0.15 rad')
    axes[2,col].set(ylabel='Max joint error (rad)',ylim=(0,1.7))
    axes[2,col].legend(fontsize=8,loc='upper right')
    axes[3,col].plot(t,a['pitch'][ids],color='#61558c',label='Body pitch')
    axes[3,col].plot(t,a['tilt'][ids],color='#ba923f',ls='--',label='Total body tilt')
    axes[3,col].axhline(30,color='#a04036',ls=':',label='Transfer tilt bound')
    axes[3,col].axhline(-30,color='#a04036',ls=':')
    axes[3,col].set_ylabel('Body angle (deg)')
    axes[3,col].legend(fontsize=8,loc='upper left')
    for ax in axes[:,col]:
        ax.set_xlim(t[0],t[-1])
        ax.set_xlabel('Time from native-script start (s)')
        ax.grid(alpha=.2)
    axes[0,col].set_title('Attempt 2: FOLD stalls, then 8 s timeout' if col==0 else 'Attempt 4: PREPARE cannot reach transfer pose')
    axes[2,col].text(.03,.07,'Tuck compensation inactive in FOLD' if col==0 else 'Minimum error 0.160 rad > 0.150 rad; integral = 0 Nm',
                     transform=axes[2,col].transAxes,fontsize=8,color='#a04036')
fig.suptitle('Vehicle CSV diagnosis | closed-chain reconstruction + production C++ replay\n'
             'Measured API encoders converted with current calibration; no physical motion',fontsize=13)
fig.savefig(root/'takeover_blockers.png')
print(root/'takeover_blockers.png')
