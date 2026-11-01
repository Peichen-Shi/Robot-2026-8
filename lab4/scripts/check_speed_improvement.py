#!/usr/bin/env python3
"""Apply all existing Sub4 checks AND an actual 500 ms improvement threshold."""
import argparse
import json
from pathlib import Path

from check_sub4_qualification import check


def qualify(directory, baseline_ms=3894.973, improvement_ms=500.):
    if baseline_ms <= 0 or improvement_ms <= 0 or improvement_ms >= baseline_ms:
        raise ValueError('Invalid baseline or requested improvement')
    result = check(directory)
    elapsed = result.get('actual_race_ms')
    result['baseline_ms'] = baseline_ms
    result['requested_improvement_ms'] = improvement_ms
    result['maximum_race_time_ms'] = baseline_ms-improvement_ms
    complete = result.get('passed_gates') == 12 and elapsed is not None
    result['actual_improvement_ms'] = baseline_ms-elapsed if complete else None
    result['qualified'] = bool(result['qualified'] and elapsed is not None
                               and elapsed <= baseline_ms-improvement_ms)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--baseline-ms',type=float,default=3894.973,
                        help='Conservative baseline: fastest of the three saved qualified runs')
    parser.add_argument('--improvement-ms',type=float,default=500.)
    args = parser.parse_args()
    result = qualify(args.directory,args.baseline_ms,args.improvement_ms)
    (args.directory/'speed_improvement_qualification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['qualified'] else 1)
