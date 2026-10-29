"""Qualify real recorded flights, including a continuous sixty-second hold."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import rosbag

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'optimization'))
from race_acceptance import stable_terminal_window


def check(directory):
    directory=Path(directory)
    race=json.loads((directory/'race_acceptance.json').read_text())
    metadata=json.loads((directory/'run_metadata.json').read_text())
    terminal=[]
    with rosbag.Bag(str(directory/(metadata['run_id']+'.bag'))) as bag:
        for _,m,_ in bag.read_messages(topics=['/controller_diagnostics']):
            if m.task_phase_name not in ('POST_RACE_HOLD','FINISHED'):continue
            norm=lambda v:math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            terminal.append((m.header.stamp.to_sec(),norm(m.position_error),
                             norm(m.velocity),norm(m.body_angular_velocity)))
    hold=stable_terminal_window(terminal,duration=60.)
    tail=[s for s in terminal if s[0]>=terminal[-1][0]-60.] if terminal else []
    startup=json.loads((directory.parent/(metadata['run_id'][:-3]+'_startup')/'startup.json').read_text())
    numeric_path=directory/'analysis/numerics_audit.json'
    numeric=json.loads(numeric_path.read_text()) if numeric_path.exists() else {}
    result=dict(run_id=metadata['run_id'],race_passed=race['passed'],
                actual_race_ms=race['first_gate_to_last_gate_ms'],passed_gates=race['passed_gates'],
                minimum_clearance_m=race['minimum_clearance_m'],stable_terminal_60s=hold,
                fresh_visible_simulator=startup.get('headless') is False and startup.get('live_state_and_clock') is True,
                numeric_reference_audit=numeric.get('passed') is True,
                terminal_tail_position_error_peak_m=max((s[1] for s in tail),default=None),
                terminal_tail_speed_peak_mps=max((s[2] for s in tail),default=None),
                terminal_tail_body_rate_peak_rad_s=max((s[3] for s in tail),default=None))
    result['qualified']=bool(result['race_passed'] and result['passed_gates']==12
                             and result['actual_race_ms']<4000 and hold
                             and result['fresh_visible_simulator'] and result['numeric_reference_audit'])
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    result=check(args.directory)
    print(json.dumps(result,indent=2))
    (args.directory/'sub4_qualification.json').write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(0 if result['qualified'] else 1)
