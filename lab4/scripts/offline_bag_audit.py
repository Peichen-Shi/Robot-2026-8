#!/usr/bin/env python3
"""Export original messages for report plots and audit their provenance; no flight."""
import argparse
import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
import rosbag

p = argparse.ArgumentParser()
p.add_argument('bag', type=Path)
p.add_argument('out', type=Path)
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
rows = defaultdict(list)
windows = {}
events = []
def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def norm(v):
    return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)

with rosbag.Bag(str(a.bag)) as bag:
    info = bag.get_type_and_topic_info()
    topics = {k: {'type':v.msg_type, 'messages':v.message_count,
                  'connections':v.connections} for k,v in info.topics.items()}
    for topic,m,t,header in bag.read_messages(return_connection_header=True):
        caller = str(header.get('callerid', '?'))
        key = topic+' | '+caller
        item = windows.setdefault(key, {'first_bag_s':t.to_sec(),'last_bag_s':t.to_sec(),'messages':0})
        item['last_bag_s'] = t.to_sec()
        item['messages'] += 1
        if topic == '/current_state':
            stamp = m.header.stamp.to_sec() or t.to_sec()
            pt = m.pose.pose.position
            rows['actual'].append([stamp,t.to_sec(),pt.x,pt.y,pt.z,yaw(m.pose.pose.orientation),norm(m.twist.twist.linear),norm(m.twist.twist.angular)])
        elif topic == '/desired_state_stamped' and m.points and m.points[0].transforms:
            pt = m.points[0]; tr = pt.transforms[0]
            rows['reference'].append([m.header.stamp.to_sec(),t.to_sec(),tr.translation.x,tr.translation.y,tr.translation.z,yaw(tr.rotation),norm(pt.velocities[0].linear),norm(pt.accelerations[0].linear),pt.time_from_start.to_sec()])
        elif topic == '/trajectory_status':
            rows['status'].append([m.header.stamp.to_sec(),t.to_sec(),m.run_id,m.trajectory_id,m.phase,m.planned_duration_s,m.trajectory_elapsed_s,m.mission_elapsed_s,m.sample_tau_s,m.reference_complete,m.fault])
        elif topic == '/tesse/rotor_watchdog_event':
            events.append({'bag_t':t.to_sec(),'data':m.data})
        elif topic == '/controller_diagnostics' and m.control_fault:
            events.append({'bag_t':t.to_sec(),'fault':m.fault_reason,'phase':m.task_phase_name})

headers = {
    'actual':['source_t','bag_t','x','y','z','yaw_rad','speed_mps','angular_speed_rad_s'],
    'reference':['source_t','bag_t','x','y','z','yaw_rad','speed_mps','acceleration_mps2','tau_s'],
    'status':['source_t','bag_t','run_id','trajectory_id','phase','T_s','trajectory_elapsed_s','mission_elapsed_s','tau_s','complete','fault']}
for name, values in rows.items():
    with (a.out/(name+'.csv')).open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(headers[name]);writer.writerows(values)
audit={'bag_sha256':hashlib.sha256(a.bag.read_bytes()).hexdigest(),'topics':topics,
       'connection_windows':windows,'events':events,'rows':{k:len(v) for k,v in rows.items()}}
for name in ('actual','reference'):
    ts=[r[0] for r in rows[name]]
    audit[name+'_duplicate_timestamps']=sum(b==c for b,c in zip(ts,ts[1:]))
    audit[name+'_clock_rollbacks']=sum(c<b for b,c in zip(ts,ts[1:]))
(a.out/'bag_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
print(json.dumps(audit,indent=2))
