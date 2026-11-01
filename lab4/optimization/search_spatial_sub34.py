#!/usr/bin/env python3
"""Offline 3D reference search; candidates still require recorded Unity flights."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
import yaml

from trajectory_screen import evaluate, reconstruct, screen


def unpack(x, seed, target, lateral):
    p = copy.deepcopy(seed)
    w = np.exp(x[:11]-max(x[:11]))
    p['segment_durations'] = [float(x[23]), *map(float, .1+(target-1.1)*w/sum(w)), 1.8]
    p['gate_vertical_offsets'] = x[11:23].tolist()
    if lateral:
        p['gate_lateral_offsets'] = x[24:36].tolist()
    return p


def cost(x, seed, target, lateral):
    p = unpack(x, seed, target, lateral)
    coefficients, durations = reconstruct(p)
    starts = np.r_[0., np.cumsum(durations)[:-1]]
    t = np.concatenate([s+d*np.linspace(.01, .99, 40) for s, d in zip(starts, durations)])
    a = evaluate(coefficients, durations, t, 2)[:, :3]
    j = evaluate(coefficients, durations, t, 3)[:, :3]
    f = a+np.array([0., 0., 9.81])
    f[:, 2] = np.maximum(f[:, 2], 1.)
    rate = np.linalg.norm(np.cross(f, j), axis=1)/np.maximum((f*f).sum(axis=1), 1e-8)
    excess = np.maximum(rate-5.5, 0.)
    return (np.mean(excess**2)+.3*max(excess)**2
            +.00015*np.mean((a*a).sum(axis=1))
            +.0000005*np.mean((j*j).sum(axis=1))
            +.005*np.mean(np.maximum(np.linalg.norm(f, axis=1)-150., 0.)**2)
            +.005*np.sum(x[11:23]**2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('seed', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', type=float, default=3.38)
    parser.add_argument('--iterations', type=int, default=180)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    p = yaml.safe_load(args.seed.read_text())
    x = np.r_[np.log(np.array(p['segment_durations'][1:-1])-.1), p['gate_vertical_offsets'], p['segment_durations'][0]]
    bounds = [(-5., 1.)]*11+[(-.7, .7)]*12+[(.85, 1.4)]
    reports = {}
    for lateral, name in [(False, 'vertical'), (True, 'spatial')]:
        if lateral:
            x = np.r_[x, p['gate_lateral_offsets']]
            bounds += [(-.65, .65)]*12
            x = np.maximum(np.array(bounds)[:, 0], np.minimum(np.array(bounds)[:, 1], x))
        print('Searching', name, 'initial cost', cost(x, p, args.target, lateral), flush=True)
        result = minimize(cost, x, args=(p, args.target, lateral), method='L-BFGS-B', bounds=bounds,
                          options={'maxiter': args.iterations, 'ftol': 1e-8, 'maxls': 25})
        x = result.x
        profile = unpack(x, p, args.target, lateral)
        (args.output/(name+'_planner.yaml')).write_text(yaml.safe_dump(profile, sort_keys=False))
        reports[name] = dict(screen=screen(profile), cost=float(result.fun), iterations=int(result.nit),
                             optimizer_success=bool(result.success), status=str(result.message))
        (args.output/'offline_screen.json').write_text(json.dumps(reports, indent=2)+'\n')
        print(name, json.dumps(reports[name]), flush=True)


if __name__ == '__main__':
    main()
