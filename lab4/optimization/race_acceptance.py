#!/usr/bin/env python3
"""First-to-last racing acceptance; never changes legacy stability/P95 results."""
import csv
import json
import math
from pathlib import Path
import numpy as np
import rosbag

def evaluate_run(directory):
    directory=Path(directory)
    m=json.loads((directory/'analysis/metrics.json').read_text())
    meta=json.loads((directory/'run_metadata.json').read_text())
    gate_rows=list(csv.DictReader((directory/'gates.csv').open()))
    official=list(csv.DictReader((Path(__file__).parent/'official_gates.csv').open()))
    same_gates=len(gate_rows)==len(official)==12 and all(
        abs(float(a[k])-float(b[k]))<1e-6 for a,b in zip(gate_rows,official) for k in ('x','y','z','yaw_raw_rad'))
    forbidden=[r for r in m['completion_verification']['failure_reasons'] if r not in (
        'no independently verified reference completion',
        'less than 60 seconds continuous terminal hold after settling',
        'whole-run position limits exceeded')]
    statuses=[]
    with rosbag.Bag(str(directory/(meta['run_id']+'.bag'))) as bag:
        for _,s,t in bag.read_messages(topics=['/trajectory_status']):
            if s.trajectory_id>0:statuses.append(s)
    finished=any(s.phase=='FINISHED' and s.reference_complete and s.sample_tau_s>=s.planned_duration_s-1e-6 for s in statuses)
    ids={(s.run_id,s.trajectory_id) for s in statuses}
    crossings=list(csv.DictReader((directory/'analysis/gate_crossings.csv').open()))
    all_crossed=len(crossings)==12 and all(r['passed']=='True' for r in crossings)
    finite=[m.get('first_gate_to_last_gate_s'),m.get('actual_speed_peak_mps')]
    checks=dict(original_twelve_gates_unchanged=same_gates,all_ordered_forward_gates=all_crossed,
        positive_finite_actual_time=all(v is not None and math.isfinite(v) and v>0 for v in finite),
        complete_reference_and_finished=finished,single_trial=len(ids)==1,
        no_fault_clock_reset_or_long_gap=not forbidden,
        collision_telemetry_enabled=meta.get('collision_telemetry_enabled') is True,
        no_reported_mission_collision=not m.get('collision_events_during_mission'))
    return dict(passed=all(checks.values()),checks=checks,run_id=meta['run_id'],
        first_gate_to_last_gate_ms=1000*m['first_gate_to_last_gate_s'] if m.get('first_gate_to_last_gate_s') is not None else None,
        passed_gates=m['passed_gates'],gate_width_m=2.4,gate_height_m=2.4,body_margin_m=.18,
        ignored_legacy_criteria=['whole-record position aggregate','60-second terminal observation'],
        legacy_stability_result=m['completion_verification'],failure_reasons=forbidden,
        minimum_clearance_m=min(min(float(r['remaining_clearance_y_m']),float(r['remaining_clearance_z_m'])) for r in crossings) if all_crossed else None,
        racing_metrics=m.get('segments',{}).get('course_through_last_crossed_gate'),
        actual_speed_peak_mps=m['actual_speed_peak_mps'])

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args()
    print(json.dumps(evaluate_run(a.directory),indent=2))
