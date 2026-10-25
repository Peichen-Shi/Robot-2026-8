#!/usr/bin/env python3
"""Real independent TESSE attempts: accept 2/2 early, otherwise require 2/3."""
import argparse
import json
import os
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('profile', type=Path)
p.add_argument('duration', type=float)
p.add_argument('prefix')
p.add_argument('--height', type=float, default=2.67)
a = p.parse_args()
here = Path(__file__).resolve().parent
root = Path(os.environ.get('LAB4_SPEED_RESULTS', '/home/joe/lab4_speed5_20261025/results'))
root.mkdir(parents=True, exist_ok=True)
summary_path = root / (a.prefix + '_qualification.json')
if summary_path.exists():
    raise SystemExit('Refusing to overwrite qualification history: ' + str(summary_path))
summary = {'profile': str(a.profile), 'target_duration_s': a.duration,
           'start_height_m': a.height, 'rule': '2/2 early acceptance, otherwise 2/3',
           'attempts': [], 'qualified': False}
for attempt in range(1, 4):
    prefix = a.prefix + '_q%02d' % attempt
    directory = root / (prefix + '_01')
    if directory.exists():
        raise SystemExit('Refusing to reuse recorded attempt: ' + str(directory))
    with (root / (prefix + '_qualification.log')).open('w') as log:
        result = subprocess.run(['bash', str(here / 'run_trial.sh'),
                                 str(a.profile.resolve()), str(a.duration), prefix,
                                 str(a.height), '75'], stdout=log, stderr=subprocess.STDOUT)
    evidence_path = directory / 'race_acceptance.json'
    evidence = json.loads(evidence_path.read_text()) if evidence_path.exists() else {}
    passed = result.returncode == 0 and evidence.get('passed') is True and evidence.get('passed_gates') == 12
    summary['attempts'].append({'run_id': directory.name, 'passed': passed,
                               'return_code': result.returncode,
                               'first_gate_to_last_gate_ms': evidence.get('first_gate_to_last_gate_ms'),
                               'failure_reasons': evidence.get('failure_reasons', ['no acceptance evidence'])})
    successes = sum(row['passed'] for row in summary['attempts'])
    summary['qualified'] = successes >= 2
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary['attempts'][-1]), flush=True)
    if summary['qualified'] or attempt - successes >= 2:
        break
print('QUALIFIED' if summary['qualified'] else 'NOT QUALIFIED', flush=True)
raise SystemExit(0 if summary['qualified'] else 1)
