#!/usr/bin/env python3
"""Plot only exported measurements; no simulated or interpolated trajectories."""
import csv, json, math, sys, os
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(os.environ.get('LAB4_EVIDENCE_OUTPUT',str(Path(__file__).resolve().parents[1]/'evidence')))
runs=sys.argv[1:] or ['qualify61_%d_01'%i for i in (1,2,3)]
fig,(route,timing)=plt.subplots(1,2,figsize=(13,6),gridspec_kw={'width_ratios':[1.65,1]})
colors=['#007f9d','#df7834','#7a52ad']
for index,run in enumerate(runs):
    folder=root/run
    rows=list(csv.DictReader((folder/'trajectory_samples.csv').open()))
    gates=list(csv.DictReader((folder/'analysis/gate_crossings.csv').open()))
    start=float(gates[0]['time_s']);end=float(gates[-1]['time_s'])
    actual=[r for r in rows if r['kind']=='actual' and start-.03<=float(r['source_stamp_s'])<=end+.03]
    route.plot([float(r['x']) for r in actual],[float(r['y']) for r in actual],
               color=colors[index],lw=1.6,alpha=.9,label='Actual cold start %d'%(index+1))
    if index==0:
        reference=[r for r in rows if r['kind']=='reference' and start-.03<=float(r['source_stamp_s'])<=end+.03]
        route.plot([float(r['x']) for r in reference],[float(r['y']) for r in reference],
                   color='#29343a',ls='--',lw=1.2,label='Published reference')
        official=list(csv.DictReader((folder/'gates.csv').open()))
        for gate_number,g in enumerate(official,1):
            x,y,yaw=[float(g[k]) for k in ('x','y','yaw_raw_rad')]
            dx,dy=-1.2*math.sin(yaw),1.2*math.cos(yaw)
            route.plot([x-dx,x+dx],[y-dy,y+dy],color='#b22b2b',lw=2.5)
            route.text(x+.35,y+.45,str(gate_number),fontsize=8,color='#742626')
    elapsed=[float(g['time_s'])-start for g in gates]
    timing.plot(range(1,13),elapsed,'o-',color=colors[index],ms=4,label='Cold start %d'%(index+1))
route.set(xlabel='World x (m)',ylabel='World y (m)',title='Measured route through the original 12 gates')
route.set_aspect('equal',adjustable='datalim');route.grid(alpha=.2);route.legend(fontsize=8,loc='lower left')
timing.axhline(7,color='#bd3333',ls='--',lw=1,label='7-second goal')
timing.set(xlabel='Actual ordered gate crossing',ylabel='Time since gate 1 (s)',
           title='First-to-last gate timing',xticks=list(range(1,13)),ylim=(-.1,7.4))
timing.grid(alpha=.2);timing.legend(fontsize=8)
mode='Unity headless' if json.loads((root/runs[0]/'run_metadata.json').read_text()).get('simulator_headless') else 'Unity rendered 3D window'
fig.suptitle('TESSE physical flights | '+mode+' | collisions enabled | clock rate unchanged',fontsize=11)
fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(root/'race_comparison.png',dpi=170)
fig.savefig(root/'race_comparison.pdf')
plt.close(fig)
fig,ax=plt.subplots(figsize=(10,5))
targets=list(csv.DictReader((root/runs[0]/'analysis/selected_gate_targets.csv').open()))
gate_ids=list(range(1,13));centres=[float(t['centre_z']) for t in targets]
ax.fill_between(gate_ids,[z-1.02 for z in centres],[z+1.02 for z in centres],color='#9ea6ad',alpha=.2,label='Original gate aperture after 0.18 m body margin')
ax.plot(gate_ids,centres,'x--',color='#606770',label='Physical gate centre')
ax.plot(gate_ids,[float(t['selected_z']) for t in targets],'s--',color='#20262b',label='Selected reference point')
for i,run in enumerate(runs):
 crossings=list(csv.DictReader((root/run/'analysis/gate_crossings.csv').open()))
 ax.plot(gate_ids,[float(t['world_z_m']) for t in crossings],'o-',color=colors[i],ms=4,label='Actual flight %d'%(i+1))
ax.set(xlabel='Original gate number',ylabel='World height (m)',xticks=gate_ids,title='Height planning within unchanged physical gate apertures')
ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper left');fig.tight_layout()
fig.savefig(root/'gate_height_comparison.png',dpi=170);fig.savefig(root/'gate_height_comparison.pdf');plt.close(fig)
print('Wrote evidence/race_comparison.png and .pdf')
