import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
data=json.loads((root/'hardware_plot_data.json').read_text())
hardware=json.loads((root/'hardware_events.json').read_text())
latest=data['latest']
simroot=root/'isaac_baseline_v2'
normal=[r for r in json.loads((simroot/'standing_305_samples.json').read_text()) if not r['post_terminal']]
firstsim=[r for r in json.loads((simroot/'csv_first_upright_samples.json').read_text()) if not r['post_terminal']]
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.titleweight':'bold'})
fig,axes=plt.subplots(2,2,figsize=(13.5,8.2),layout='constrained')
blue='#1765a6';orange='#bd5b13';red='#b22d36';green='#34734d'
t=np.array([r['time_s'] for r in latest])
ax=axes[0,0]
ax.plot(t,[r['roll_deg'] for r in latest],color=blue,label='Measured roll')
ax.set(ylabel='Roll (deg)',xlabel='Time after script start (s)',title='A. Hardware: a valid IK goal is rejected')
ay=ax.twinx();ay.spines['right'].set_visible(True)
ay.plot(t,[r['target_below_mm'][0] for r in latest],color=orange,label='Left wheel below COM at IK goal')
ay.axhline(40,color=red,ls='--',lw=1.2,label='Hard readiness threshold: 40 mm')
ay.set_ylabel('Gravity projection at target (mm)',color=orange)
ax.tick_params(axis='y',labelcolor=blue);ay.tick_params(axis='y',labelcolor=orange)
ax.axvline(t[-1],color=red,ls=':',lw=1.3)
ax.annotate('Fault: roll 17.26°\ntarget distance 39.86 mm',xy=(t[-1],17.26),xytext=(.017,14.5),
            arrowprops={'arrowstyle':'->','color':red},color=red,fontsize=10)
lines=ax.lines[:1]+ay.lines
ay.legend(lines,[l.get_label() for l in lines],loc='center left',bbox_to_anchor=(0.,.23),fontsize=8,framealpha=1.)
ax=axes[0,1]
for side,label,color in [(0,'Left measured knee',blue),(1,'Right measured knee',green)]:
    ax.plot(t,[r['knee_deg'][side] for r in latest],label=label,color=color)
ax.axhline(40.5,color=orange,ls='--',label='Requested goal: 40.5°')
ax.set(ylabel='Thigh / shank angle (deg)',xlabel='Time after script start (s)',
       title='B. Hardware: folding ends before the legs arrive',ylim=(35,112))
ax.annotate('At fault: 54.69° / 52.85°',xy=(t[-2],55.1),xytext=(.012,62),
            arrowprops={'arrowstyle':'->','color':red},color=red)
ax.legend(fontsize=9,loc='upper right')
ax=axes[1,0]
st=np.array([r['time_s'] for r in normal])
ref=np.array([r['native_output'][:4] for r in normal]);reference=(ref[:,0]+ref[:,1])/2
ax.plot(st,-np.degrees(reference-reference[0]),color=orange,label='Leg reference rotation magnitude')
ax.set(ylabel='Commanded common rotation (deg)',xlabel='Simulation time (s)',
       title='C. Balanced 305 mm start: an unnecessary full turn')
ay=ax.twinx();ay.spines['right'].set_visible(True)
ay.plot(st,[r['tilt_deg'] for r in normal],color=blue,label='Body tilt')
ay.set_ylabel('Body tilt (deg)',color=blue);ay.set_ylim(0,45)
ax.tick_params(axis='y',labelcolor=orange);ay.tick_params(axis='y',labelcolor=blue)
ax.annotate('371°',xy=(st[-1],-np.degrees(reference[-1]-reference[0])),
            xytext=(1.8,385),arrowprops={'arrowstyle':'->','color':red},color=red)
ax.set_ylim(-10,410)
lines=ax.lines+ay.lines
ax.legend(lines,[l.get_label() for l in lines],loc='upper left',fontsize=8)
ax=axes[1,1]
first=data['first']
ax.plot([r['time_s'] for r in first],[r['tilt_deg'] for r in first],color=red,label='Hardware, first upright start')
ax.plot([r['time_s'] for r in firstsim],[r['tilt_deg'] for r in firstsim],color=blue,label='Same start geometry in simulation')
ax.axhline(90,color='gray',ls=':',lw=1)
ax.set(ylabel='Body tilt (deg)',xlabel='Time after script start (s)',title='D. Upright-start hardware tipped past 90°')
ax.text(.04,.96,'Full 180° flip was not reproduced in simulation.',transform=ax.transAxes,
        va='top',fontsize=9,color='#4b4b4b')
ax.legend(fontsize=9,loc='center left')
for ax in axes.flat:ax.grid(alpha=.15)
fig.suptitle('Wheel-leg recovery: hardware replay and unrestrained Isaac Sim 6.0',fontsize=15,fontweight='bold')
fig.supxlabel('Simulation curves stop at the first RL ownership event. No actor is run; no policy success is claimed.',fontsize=9)
fig.savefig(root/'recovery_diagnosis.png',dpi=165)
print('Hardware state counts:',hardware['state_counts'])
print('Figure:',root/'recovery_diagnosis.png')
