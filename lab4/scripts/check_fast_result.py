#!/usr/bin/env python3
"""Additional speed acceptance; never replaces the original stability checks."""
import argparse
import json
import math
import sys
from pathlib import Path

def evaluate(m, limit=20.):
    course=m.get('segments',{}).get('course_through_last_crossed_gate') or {}
    end=m.get('mission_start_to_last_gate_s')
    settled=m.get('mission_start_to_settled_finish_s')
    checks=dict(
        original_physical_acceptance=bool(m['actual_stable_full_route']),
        all_twelve_gates=m['passed_gates']==12,
        actual_mission_to_last_gate_under_limit=end is not None and 0<end<limit,
        actual_settled_finish_under_limit=settled is not None and 0<settled<limit,
        complete_reference_under_limit=m['planned_trajectory_duration_s']+.5<limit,
        racing_rmse_unchanged_limit=course.get('position_rmse_m',math.inf) is not None and course.get('position_rmse_m',math.inf)<=.75,
        racing_p95_unchanged_limit=course.get('position_p95_m',math.inf) is not None and course.get('position_p95_m',math.inf)<=1.25,
        no_reported_collisions=not m.get('collision_events_during_mission',[]))
    return dict(passed=all(checks.values()),checks=checks,limit_s=limit,
        actual_mission_to_last_gate_s=end,
        actual_tracking_to_last_gate_s=m.get('trajectory_start_to_last_gate_s'),
        actual_mission_to_settled_finish_s=settled,
        measured_wall_to_settled_s_estimate=m.get('mission_to_settled_wall_s_estimate'),
        planned_reference_duration_s=m['planned_trajectory_duration_s'],
        racing_metrics=course,terminal_observation_separate=True,
        terminal_check=m['completion_verification'])

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path);p.add_argument('--limit',type=float,default=20.)
    a=p.parse_args()
    result=evaluate(json.loads((a.run/'analysis/metrics.json').read_text()),a.limit)
    (a.run/'analysis/fast_acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    sys.exit(0 if result['passed'] else 1)
