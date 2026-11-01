#!/usr/bin/env python3
"""Export physical-flight evidence without changing acceptance results."""
import csv,json,math,shutil,subprocess,sys,os
from pathlib import Path
import rosbag
ROOT=Path(__file__).resolve().parents[1];out=Path(os.environ.get('LAB4_EVIDENCE_OUTPUT',str(ROOT/'evidence')));out.mkdir(exist_ok=True)
runs=sys.argv[1:] or (['baseline_105_01']+['clipped_cold_%d_01'%i for i in (1,2,3)])
summary=[]
for run in runs:
 d=ROOT/'results'/run
 if not (d/'race_acceptance.json').exists():continue
 subprocess.run([sys.executable,str(ROOT/'scripts/audit_race_numerics.py'),str(d)],check=True,stdout=subprocess.DEVNULL)
 dest=out/run;dest.mkdir(exist_ok=True)
 for rel in ['race_acceptance.json','run_metadata.json','parameters.yaml','gates.csv',run+'.bag',
             'analysis/metrics.json','analysis/gate_crossings.csv','analysis/numerics_audit.json','analysis/selected_gate_targets.csv']:
  src=d/rel;target=dest/rel;target.parent.mkdir(exist_ok=True,parents=True);shutil.copy2(src,target)
 m=json.loads((d/'analysis/metrics.json').read_text());a=json.loads((d/'race_acceptance.json').read_text())
 gaps=[];rows=[]
 with rosbag.Bag(str(d/(run+'.bag'))) as bag:
  for topic,msg,b in bag.read_messages(topics=['/current_state','/desired_state_stamped','/controller_diagnostics']):
   if topic=='/current_state':
    p=msg.pose.pose.position;v=msg.twist.twist.linear
    rows.append(['actual',msg.header.stamp.to_sec(),p.x,p.y,p.z,math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)])
   elif topic=='/desired_state_stamped':
    p=msg.points[0].transforms[0].translation;v=msg.points[0].velocities[0].linear
    rows.append(['reference',msg.header.stamp.to_sec(),p.x,p.y,p.z,math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)])
   elif not msg.control_fault:gaps.append(msg.state_interval_wall_s)
 with (dest/'trajectory_samples.csv').open('w',newline='') as f:
  writer=csv.writer(f);writer.writerow(['kind','source_stamp_s','x','y','z','speed_mps']);writer.writerows(rows)
 summary.append(dict(run_id=run,passed=a['passed'],gates=a['passed_gates'],
                     race_time_s=a['first_gate_to_last_gate_ms']/1000. if a['passed'] else None,
                     tracking_start_to_last_gate_s=m.get('trajectory_start_to_last_gate_s'),
                     minimum_clearance_m=a['minimum_clearance_m'],
                     peak_speed_mps=a['actual_speed_peak_mps'],
                     maximum_state_arrival_interval_wall_s=max(gaps,default=None),
                     racing_metrics=a['racing_metrics']))
(out/'verification_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
subprocess.run([sys.executable,str(ROOT/'optimization/collect_results.py')],check=True,stdout=subprocess.DEVNULL)
for name in ['trial_summary.json','trial_summary.csv']:
 shutil.copy2(ROOT/'results'/name,out/name)
print(json.dumps(summary,indent=2))
