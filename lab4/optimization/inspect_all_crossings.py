#!/usr/bin/env python3
"""Diagnose every physical gate, even after a miss invalidates the race.

Nearest expected-time crossings are diagnostic only; they do not replace the
unchanged ordered gate validator or count failed flights as complete races.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag
import yaml


def inspect(directory):
    directory=Path(directory)
    metadata=json.loads((directory/'run_metadata.json').read_text())
    parameters=yaml.safe_load((directory/'parameters.yaml').read_text())['trajectory_generation_node']
    rows=[];start=None
    with rosbag.Bag(str(directory/(metadata['run_id']+'.bag'))) as bag:
        for topic,m,_ in bag.read_messages(topics=['/current_state','/trajectory_status']):
            if topic=='/current_state':
                p=m.pose.pose.position
                rows.append([m.header.stamp.to_sec(),p.x,p.y,p.z])
            elif m.phase=='TRACKING' and start is None:
                start=m.header.stamp.to_sec()-m.trajectory_elapsed_s
    if start is None or len(rows)<2:
        raise ValueError('No recorded tracking interval')
    actual=np.array(rows);times=actual[:,0];points=actual[:,1:]
    expected=start+np.cumsum(parameters['segment_durations'])[:-1]
    result=[]
    with (directory/'gates.csv').open() as stream:
        gates=list(csv.DictReader(stream))
    for index,gate in enumerate(gates):
        x,y,z,w=(float(gate['q'+k]) for k in ('x','y','z','w'))
        rotation=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                           [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                           [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
        centre=np.array([float(gate[k]) for k in ('x','y','z')])
        local=(points-centre)@rotation
        possible=np.where((local[:-1,0]<0)&(local[1:,0]>=0)&(times[:-1]>start))[0]
        crossings=[]
        for k in possible:
            fraction=-local[k,0]/(local[k+1,0]-local[k,0])
            time=times[k]+fraction*(times[k+1]-times[k])
            yz=local[k,1:]+fraction*(local[k+1,1:]-local[k,1:])
            crossings.append((abs(time-expected[index]),time,yz))
        if not crossings:
            result.append({'gate':index+1,'found':False});continue
        _,time,yz=min(crossings,key=lambda item:item[0])
        result.append({'gate':index+1,'found':True,'relative_time_s':float(time-start),
                       'nominal_time_s':float(expected[index]-start),
                       'local_y_m':float(yz[0]),'local_z_m':float(yz[1]),
                       'inside_aperture':bool(max(abs(yz))<=1.02),
                       'remaining_clearance_m':float(1.02-max(abs(yz))),
                       'planned_lateral_m':float(parameters['gate_lateral_offsets'][index])})
    return {'run_id':metadata['run_id'],'kind':'diagnostic_not_ordered_race_validation','crossings':result}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    report=inspect(args.directory)
    (args.directory/'analysis/all_gate_diagnostics.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
