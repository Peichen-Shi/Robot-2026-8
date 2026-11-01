#!/usr/bin/env python3
"""Reconstruct a recorded race and independently check its spline constraints."""
import argparse,csv,json,math,subprocess
from pathlib import Path
import rosbag,yaml
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
d=a.directory;meta=json.loads((d/'run_metadata.json').read_text())
metrics=json.loads((d/'analysis/metrics.json').read_text());T=metrics['planned_trajectory_duration_s']
initial=None;references=[];started=False
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
with rosbag.Bag(str(d/(meta['run_id']+'.bag'))) as bag:
 for _,m,_ in bag.read_messages(topics=['/desired_state_stamped']):
  r=m.points[0];tr=r.transforms[0];q=tr.rotation;tau=r.time_from_start.to_sec()
  point=[tr.translation.x,tr.translation.y,tr.translation.z,yaw(q)]
  if tau==0 and not started:initial=point
  if 0<tau<T-1e-6:
   started=True;v=r.velocities[0].linear;acc=r.accelerations[0].linear
   references.append([tau]+point+[v.x,v.y,v.z,acc.x,acc.y,acc.z])
if initial is None or not references:raise RuntimeError('Missing recorded initial state or trajectory')
references=references[::max(1,len(references)//100)]
gates=list(csv.DictReader((d/'gates.csv').open()));f=d/'analysis/numerics_input.txt'
params=yaml.safe_load((d/'parameters.yaml').read_text())['trajectory_generation_node']
lateral=params.get('gate_lateral_offsets') or [0.]*len(gates)
vertical=params.get('gate_vertical_offsets') or [0.]*len(gates)
if len(lateral)!=len(gates) or len(vertical)!=len(gates):raise RuntimeError('Invalid gate target count')
if any(not math.isfinite(v) or abs(v)>.70 for v in lateral+vertical):raise RuntimeError('Aperture offset exceeds configured bound')
selected_rows=[]
with f.open('w') as out:
 out.write('%d %d\n'%(len(gates),len(references)))
 out.write(' '.join(map(repr,initial+[T]))+'\n')
 for index,g in enumerate(gates):
  x,y,z,w=[float(g['q'+k]) for k in ('x','y','z','w')]
  normal=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)]
  local_y=[2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)]
  local_z=[2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)]
  selected=[float(g[k])+lateral[index]*local_y[j]+vertical[index]*local_z[j] for j,k in enumerate(('x','y','z'))]
  selected_rows.append([index+1]+[float(g[k]) for k in ('x','y','z')]+[lateral[index],vertical[index]]+selected+[1.02-abs(lateral[index]),1.02-abs(vertical[index])])
  out.write(' '.join(map(repr,selected+[float(g['yaw_raw_rad'])]+normal))+'\n')
 for row in references:out.write(' '.join(map(repr,row))+'\n')
with (d/'analysis/selected_gate_targets.csv').open('w',newline='') as target_file:
 writer=csv.writer(target_file)
 writer.writerow(['gate','centre_x','centre_y','centre_z','lateral_offset_m','vertical_offset_m','selected_x','selected_y','selected_z','nominal_remaining_y_m','nominal_remaining_z_m'])
 writer.writerows(selected_rows)
args=['rosrun','trajectory_generation','trajectory_numerics_audit',str(f),'--fast-allocation',str(params.get('terminal_segment_time',0))]
if params.get('segment_durations'):
 times=d/'analysis/recorded_segment_times.txt';times.write_text(' '.join(map(repr,params['segment_durations']))+'\n')
 args+=['--segment-times',str(times)]
result=subprocess.run(args,text=True,stdout=subprocess.PIPE,check=False)
audit=json.loads(result.stdout)
audit.update(gate_constraint_targets='selected points within original apertures' if any(lateral+vertical) else 'original gate centres',
             maximum_selected_lateral_offset_m=max(map(abs,lateral)),
             maximum_selected_vertical_offset_m=max(map(abs,vertical)),
             minimum_nominal_gate_clearance_m=1.2-.18-max(map(abs,lateral+vertical)))
(d/'analysis/numerics_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
print(json.dumps(audit,indent=2));raise SystemExit(result.returncode)
