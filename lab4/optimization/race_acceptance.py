#!/usr/bin/env python3
"""First-to-last racing acceptance; never changes legacy stability/P95 results."""
import csv
import json
import math
from pathlib import Path

def stable_terminal_window(samples, duration=1.0, maximum_gap=0.10):
    """Require a full, ordered second of finite, continuously stable telemetry."""
    if len(samples) < 2 or not all(math.isfinite(v) for row in samples for v in row):
        return False
    # Reject rollback anywhere in the terminal stream; duplicate epochs do not
    # add observation time and are checked for stability like every other row.
    if any(b[0] < a[0] for a, b in zip(samples, samples[1:])):
        return False
    cutoff = samples[-1][0] - duration
    starts = [i for i, row in enumerate(samples) if row[0] <= cutoff + 1e-9]
    if not starts:
        return False
    start = starts[-1]
    while start > 0 and samples[start - 1][0] == samples[start][0]:
        start -= 1
    tail = samples[start:]
    return (tail[-1][0] - tail[0][0] >= duration - 1e-9
            and all(b[0] - a[0] <= maximum_gap + 1e-9
                    for a, b in zip(tail, tail[1:]))
            and all(0 <= r[1] <= .3 and 0 <= r[2] <= .2
                    and 0 <= r[3] <= .5 for r in tail))

def evaluate_run(directory, bag_factory=None):
    if bag_factory is None:
        import rosbag
        bag_factory = rosbag.Bag
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
    statuses=[];terminal=[]
    with bag_factory(str(directory/(meta['run_id']+'.bag'))) as bag:
        for topic,s,t in bag.read_messages(topics=['/trajectory_status','/controller_diagnostics']):
            if topic=='/trajectory_status' and s.trajectory_id>0:statuses.append(s)
            elif topic=='/controller_diagnostics' and s.task_phase_name in ('POST_RACE_HOLD','FINISHED'):
                norm=lambda v:math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                terminal.append((s.header.stamp.to_sec(),norm(s.position_error),norm(s.velocity),norm(s.body_angular_velocity)))
    finished=any(s.phase in ('POST_RACE_HOLD','FINISHED') and s.reference_complete and s.sample_tau_s>=s.planned_duration_s-1e-6 for s in statuses)
    terminal_ok=stable_terminal_window(terminal)
    ids={(s.run_id,s.trajectory_id) for s in statuses}
    crossings=list(csv.DictReader((directory/'analysis/gate_crossings.csv').open()))
    all_crossed=len(crossings)==12 and all(r['passed']=='True' for r in crossings)
    finite=[m.get('first_gate_to_last_gate_s'),m.get('actual_speed_peak_mps')]
    checks=dict(original_twelve_gates_unchanged=same_gates,all_ordered_forward_gates=all_crossed,
        positive_finite_actual_time=all(v is not None and math.isfinite(v) and v>0 for v in finite),
        complete_published_reference=finished,
        single_trial=len(ids)==1 and all(run_id == meta['run_id'] for run_id, _ in ids),
        stable_exit_last_second=terminal_ok,
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
