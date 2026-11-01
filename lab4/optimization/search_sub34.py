#!/usr/bin/env python3
"""Generate unqualified faster candidates without changing the proven baseline.

The cost discourages rapid changes in thrust direction and extreme forces.
It is a reference-screening surrogate only, not an identified Unity simulator.
"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
import yaml

from trajectory_screen import ROOT, evaluate, reconstruct, screen


def unpack(vector, baseline, target):
    profile = copy.deepcopy(baseline)
    weights = np.exp(vector[:11] - np.max(vector[:11]))
    times = .1 + (target - 1.1) * weights / sum(weights)
    profile['segment_durations'] = [baseline['segment_durations'][0], *map(float,times), baseline['segment_durations'][-1]]
    profile['gate_lateral_offsets'] = vector[11:23].tolist()
    return profile


def objective(vector, baseline, target):
    profile = unpack(vector,baseline,target)
    coefficients, durations = reconstruct(profile)
    # A fixed sample count per segment avoids changes in discretisation as time
    # allocation varies. Include neighbours of the racing interval for braking.
    starts = np.r_[0.,np.cumsum(durations)[:-1]]
    times = np.concatenate([start + duration*np.linspace(.01,.99,30)
                            for start,duration in zip(starts,durations)])
    acceleration = evaluate(coefficients,durations,times,2)[:,:3]
    jerk = evaluate(coefficients,durations,times,3)[:,:3]
    force = acceleration + np.array([0.,0.,9.81])
    force[:,2] = np.maximum(force[:,2],1.)
    angular_rate = np.linalg.norm(np.cross(force,jerk),axis=1) / np.maximum(np.sum(force*force,axis=1),1e-6)
    excess = np.maximum(angular_rate-6.,0.)
    force_excess = np.maximum(np.linalg.norm(force,axis=1)-160.,0.)
    # Integral excess rate is a proxy for the simulator's finite bank-reversal
    # response. Peak terms avoid trading one severe corner for many easy ones.
    return (np.mean(excess**2) + .12*np.max(excess)**2
            + .001*np.mean(np.sum(acceleration**2,axis=1))
            + .000005*np.mean(np.sum(jerk**2,axis=1))
            + .03*np.mean(force_excess**2)
            + .01*np.sum(vector[11:]**2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target',type=float,default=3.38)
    parser.add_argument('--iterations',type=int,default=100)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    baseline = yaml.safe_load((ROOT/'optimization/planner_profiles/sub4.yaml').read_text())
    controller = yaml.safe_load((ROOT/'optimization/profiles/sub4.yaml').read_text())
    original = np.array(baseline['segment_durations'][1:-1])
    scaled = original / sum(original) * args.target
    weights = np.maximum(scaled-.1,1e-6)
    vector = np.r_[np.log(weights),baseline['gate_lateral_offsets']]
    bounds = [(-5.,1.)]*11 + [(-.7,.7)]*12
    print('Screening reference; no flight result is being claimed.',flush=True)
    print('Initial surrogate cost',objective(vector,baseline,args.target),flush=True)
    result = minimize(objective,vector,args=(baseline,args.target),method='L-BFGS-B',
                      bounds=bounds,options={'maxiter':args.iterations,'ftol':1e-7,'maxls':25})
    candidates = {'scaled':unpack(vector,baseline,args.target),
                  'screened':unpack(result.x,baseline,args.target)}
    report = {'status':'NOT_FLIGHT_QUALIFIED','baseline':screen(baseline),
              'optimizer':{'success':bool(result.success),'message':str(result.message),
                           'iterations':int(result.nit),'initial_cost':float(objective(vector,baseline,args.target)),
                           'final_cost':float(result.fun)},'candidates':{}}
    ratio = float(args.target / sum(original))
    for name,profile in candidates.items():
        (args.output/(name+'_planner.yaml')).write_text(yaml.safe_dump(profile,sort_keys=False))
        report['candidates'][name] = screen(profile)
    # These thresholds are referenced to trajectory start, not race start.
    # Move learned final-turn compensation to the same relative course phase.
    for key in ('corner_force_bias_after','braking_lead_after'):
        controller[key] = baseline['segment_durations'][0] + (controller[key]-baseline['segment_durations'][0])*ratio
    for yaw_weight in (.2,.1,.0):
        trial = dict(controller,yaw_weight=yaw_weight)
        (args.output/('controller_yaw'+str(yaw_weight)+'.yaml')).write_text(yaml.safe_dump(trial,sort_keys=False))
    (args.output/'reference_screen.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__ == '__main__':
    main()
