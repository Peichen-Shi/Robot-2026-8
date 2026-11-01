#!/usr/bin/env python3
"""Reconstruct the actual optimized references with the same vendored library."""
import argparse
import csv
import json
import math
import os
import subprocess
import rosbag
p=argparse.ArgumentParser(); p.add_argument('results'); a=p.parse_args()
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for run in ('stable_run_01','stable_run_04','stable_run_05'):
    directory=os.path.join(a.results,run)
    with open(directory+'/analysis/metrics.json') as f: metrics=json.load(f)
    initial=None; references=[]
    with rosbag.Bag(directory+'/'+run+'.bag') as bag:
        for _,m,t in bag.read_messages(topics=['/desired_state']):
            transform=m.transforms[0]; point=transform.translation
            if metrics['mission_start_source_s']+1 < t.to_sec() < metrics['mission_start_source_s']+4 and m.time_from_start.to_sec()==0:
                initial=[point.x,point.y,point.z,yaw(transform.rotation)]
            if m.time_from_start.to_sec()>0:
                v=m.velocities[0].linear; acc=m.accelerations[0].linear
                references.append([m.time_from_start.to_sec(),point.x,point.y,point.z,
                                   yaw(transform.rotation),v.x,v.y,v.z,acc.x,acc.y,acc.z])
    if initial is None: raise RuntimeError('missing recorded READY_HOLD reference')
    references=references[::max(1,len(references)//80)]
    with open(directory+'/gates.csv') as f: gates=list(csv.DictReader(f))
    input_path=directory+'/analysis/numerics_input.txt'
    with open(input_path,'w') as f:
        f.write('%d %d\n'%(len(gates),len(references)))
        f.write(' '.join(map(repr,initial+[metrics['planned_trajectory_duration_s']]))+'\n')
        for g in gates:
            x,y,z,w=[float(g['q'+k]) for k in ('x','y','z','w')]
            normal=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)]
            values=[float(g[k]) for k in ('x','y','z','yaw_raw_rad')]+normal
            f.write(' '.join(map(repr,values))+'\n')
        for row in references: f.write(' '.join(map(repr,row))+'\n')
    with open(directory+'/analysis/numerics_audit.json','w') as output:
        subprocess.check_call(['rosrun','trajectory_generation','trajectory_numerics_audit',input_path],stdout=output)
    print(run,'numerical reconstruction/continuity/conditioning audit PASS')
