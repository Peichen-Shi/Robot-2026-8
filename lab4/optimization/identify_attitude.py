"""Estimate controller-model inertia and command lag from existing diagnostic bags.

Exclude samples near Unity's angular-speed saturation; do not modify physics.
The linear fit includes unknown per-axis damping to expose timing mismatch.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('directories',type=Path,nargs='+')
parser.add_argument('--output',type=Path)
args=parser.parse_args()
streams=[]
for directory in args.directories:
    metadata=json.loads((directory/'run_metadata.json').read_text())
    rows=[]
    with rosbag.Bag(str(directory/(metadata['run_id']+'.bag'))) as bag:
        for _,m,_ in bag.read_messages(topics=['/controller_diagnostics']):
            if m.control_fault:continue
            vector=lambda v:np.array([v.x,v.y,v.z])
            rows.append((m.header.stamp.to_sec(),vector(m.body_angular_velocity),vector(m.moment_command)))
    streams.append(rows)
fits=[]
for lag in (0,1,2):
    x=[];y=[]
    for rows in streams:
        for i in range(2,len(rows)-1):
            t,w,_=rows[i];dt=rows[i+1][0]-t
            if not .019<dt<.021 or max(np.linalg.norm(w),np.linalg.norm(rows[i+1][1]))>6.:continue
            dx,dy,dz=(rows[i+1][1]-w)/dt
            wx,wy,wz=w
            x.extend([[dx,-wy*wz,wy*wz,wx,0,0],
                      [wx*wz,dy,-wx*wz,0,wy,0],
                      [-wx*wy,wx*wy,dz,0,0,wz]])
            y.extend(rows[i-lag][2])
    x=np.array(x);y=np.array(y)
    fitted=np.linalg.lstsq(x,y,rcond=None)[0]
    fits.append(dict(lag_samples=lag,nominal_lag_seconds=.02*lag,
                     observed_intervals=len(y)//3,inertia_kg_m2=fitted[:3].tolist(),
                     damping=fitted[3:].tolist(),moment_rmse_Nm=float(np.sqrt(np.mean((x@fitted-y)**2)))))
report=dict(source='actual diagnostic messages; approximate model fit',
            input_runs=[str(p) for p in args.directories],fits=fits)
print(json.dumps(report,indent=2))
if args.output:
    with args.output.open('x') as output:json.dump(report,output,indent=2)
