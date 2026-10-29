"""Exploratory per-gate plane intersections, never a substitute for acceptance."""
from pathlib import Path
import sys,csv,json
import rosbag,numpy as np,yaml
r=Path(sys.argv[1]);meta=json.loads((r/'run_metadata.json').read_text())
rows=[]
with rosbag.Bag(str(r/(meta['run_id']+'.bag'))) as bag:
    for _,m,_ in bag.read_messages(topics=['/current_state']):
        p=m.pose.pose.position;rows.append([m.header.stamp.to_sec(),p.x,p.y,p.z])
a=np.array(rows)
gates=list(csv.DictReader((r/'gates.csv').open()))
times=yaml.safe_load((r/'parameters.yaml').read_text())['trajectory_generation_node']['segment_durations']
first=float(list(csv.DictReader((r/'analysis/gate_crossings.csv').open()))[0]['time_s'])
expected=first+np.r_[0,np.cumsum(times[1:-1])]
for i,g in enumerate(gates):
    yaw=float(g['yaw_raw_rad']);n=np.array([np.cos(yaw),np.sin(yaw),0.])
    lateral=np.array([-np.sin(yaw),np.cos(yaw),0.])
    centre=np.array([float(g[k]) for k in ('x','y','z')])
    signed=(a[:,1:]-centre)@n
    intersections=[]
    for j in np.where((signed[:-1]<0)&(signed[1:]>=0))[0]:
        fraction=-signed[j]/(signed[j+1]-signed[j])
        sample=a[j]+fraction*(a[j+1]-a[j]);delta=sample[1:]-centre
        intersections.append((abs(sample[0]-expected[i]),sample[0],delta@lateral,delta[2]))
    if intersections:
        error,t,y,z=min(intersections)
        print(json.dumps(dict(gate=i+1,time_s=t,time_from_first_s=t-first,
                              nearest_expected_time_error_s=error,local_y_m=y,local_z_m=z,
                              clearance_m=float(1.02-max(abs(y),abs(z))))))
    else:print(json.dumps(dict(gate=i+1,intersection=None)))
