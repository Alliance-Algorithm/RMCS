"""Plot actual closed-chain fixture traces; post-event holding is not RL."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root = Path(__file__).resolve().parent
report = json.loads((root/'isaac_extended/report.json').read_text())
rows={x['case']:x for x in report['results']}
fig, axes=plt.subplots(2,2,figsize=(13,7.2),sharex=True,layout='constrained')
for row, names in enumerate((('extended_front','extended_back'),('ground_front','ground_back'))):
    for name,color in zip(names,('#1565c0','#d05a12')):
        samples=json.loads((root/'isaac_extended'/f'{name}_samples.json').read_text())
        t=np.array([s['time_s'] for s in samples])
        knees=np.array([s['knee_deg'] for s in samples])
        errors=np.array([s['wheel_com_error_m'] for s in samples])*1000
        for axis,values in zip(axes[row],(knees,errors)):
            axis.plot(t,values[:,0],color=color,label=name.replace('_',' '),lw=1.7)
            axis.plot(t,values[:,1],color=color,ls='--',lw=1)
            axis.axvline(rows[name]['handover']['time_s'],color=color,ls=':',alpha=.75,lw=1.1)
    axes[row,0].axhline(40.5,color='#345542',lw=1,ls='-.',label='40.5 deg target')
    axes[row,1].axhline(0,color='#345542',lw=1,ls='-.')
    axes[row,0].set_ylabel('Inner knee angle (deg)')
    axes[row,1].set_ylabel('Wheel - COM fore/aft (mm)')
    for axis in axes[row]:
        axis.set_xlim(0,3)
        axis.grid(alpha=.18)
        axis.legend(loc='best',fontsize=8)
axes[0,0].set_title('104 deg starts: fold and rotate')
axes[0,1].set_title('Wheel alignment from PhysX body poses')
axes[1,0].set_title('Ground starts: legs initially forward / backward')
axes[1,1].set_title('Grounded wheel alignment')
for axis in axes[-1]: axis.set_xlabel('Time (s)')
fig.suptitle('Production recovery trajectory in Isaac Sim 6 closed-chain fixtures',fontsize=14)
fig.supxlabel('Body attitude fixed. Solid / dashed: left / right. Dotted: RL-phase event; fixture PD holds legs afterward, no actor.',fontsize=9)
fig.savefig(root/'trajectory_evidence.png',dpi=150)
print(root/'trajectory_evidence.png')
