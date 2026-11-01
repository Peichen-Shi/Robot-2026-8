#!/usr/bin/env python3
"""Offline minimum-snap screening; this is NOT a flight or race-time validator.

Reconstruct the same degree-nine, C4 spline as the planner from endpoint
derivatives, in normalised segment time. Keep physical gates, constraints and
the certified sub4 configuration untouched. ROS/Unity qualification is still
required for every candidate.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def basis(t, derivative=0):
    t = np.asarray(t)
    result = np.zeros(t.shape + (10,))
    for power in range(derivative, 10):
        factor = math.factorial(power) / math.factorial(power - derivative)
        result[..., power] = factor * t ** (power - derivative)
    return result


_ENDPOINT = np.vstack([basis(t, d) for t in (0., 1.) for d in range(5)])
_INVERSE = np.linalg.inv(_ENDPOINT)
_Q = np.zeros((10, 10))
for _i in range(4, 10):
    for _j in range(4, 10):
        _Q[_i, _j] = (math.factorial(_i) / math.factorial(_i - 4)
                      * math.factorial(_j) / math.factorial(_j - 4)
                      / (_i + _j - 7))
_R = _INVERSE.T @ _Q @ _INVERSE
_R = (_R + _R.T) / 2


def minimum_snap(points, durations):
    """Position at all vertices, zero derivatives 1..4 at both ends."""
    points = np.asarray(points, dtype=float)
    durations = np.asarray(durations, dtype=float)
    if len(points) != len(durations) + 1 or np.any(durations <= 0):
        raise ValueError('Inconsistent vertices or non-positive duration')
    n = len(points)
    hessian = np.zeros((5 * n, 5 * n))
    derivatives = np.zeros((5 * n, points.shape[1]))
    derivatives[::5] = points
    for i, duration in enumerate(durations):
        scale = duration ** np.tile(np.arange(5), 2)
        local = _R * scale[:, None] * scale[None, :] / duration ** 7
        hessian[5*i:5*i+10, 5*i:5*i+10] += local
    free = np.array([5*i+d for i in range(1, n-1) for d in range(1, 5)])
    fixed = np.setdiff1d(np.arange(5*n), free)
    hf = hessian[np.ix_(free, free)]
    # Diagonal scaling avoids mixing position-derived units at short durations.
    scale = np.sqrt(np.diag(hf))
    rhs = -hessian[np.ix_(free, fixed)] @ derivatives[fixed]
    derivatives[free] = np.linalg.solve(hf / scale[:, None] / scale[None, :],
                                       rhs / scale[:, None]) / scale[:, None]
    coefficients = []
    for i, duration in enumerate(durations):
        scale = duration ** np.tile(np.arange(5), 2)
        coefficients.append(_INVERSE @ (derivatives[5*i:5*i+10] * scale[:, None]))
    return np.array(coefficients)


def evaluate(coefficients, durations, times, derivative=0):
    durations = np.asarray(durations)
    times = np.asarray(times)
    ends = np.cumsum(durations)
    indices = np.minimum(np.searchsorted(ends, times, side='right'), len(ends)-1)
    local = (times - np.r_[0., ends[:-1]][indices]) / durations[indices]
    return np.einsum('ij,ijk->ik', basis(local, derivative), coefficients[indices]) / durations[indices, None] ** derivative


def gate_geometry():
    with (ROOT / 'optimization/official_gates.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    centres = np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
    rotations = []
    for row in rows:
        x,y,z,w = (float(row['q'+k]) for k in ('x','y','z','w'))
        rotations.append([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                          [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                          [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
    return centres, np.array(rotations), np.array([float(r['yaw_unwrapped_rad']) for r in rows])


def reconstruct(profile, initial=(.4, 8., 2.67, -1.5707963267948966)):
    centres, rotations, yaws = gate_geometry()
    lateral = np.array(profile['gate_lateral_offsets'])
    vertical = np.array(profile['gate_vertical_offsets'])
    durations = np.array(profile['segment_durations'])
    if lateral.shape != (12,) or vertical.shape != (12,) or durations.shape != (13,):
        raise ValueError('Expected twelve offsets and thirteen durations')
    if not np.all(np.isfinite(np.r_[lateral, vertical, durations])) or np.max(np.abs(np.r_[lateral, vertical])) > .700000001 or min(durations) < .1:
        raise ValueError('Candidate violates existing offset/time bounds')
    targets = centres + lateral[:,None] * rotations[:,:,1] + vertical[:,None] * rotations[:,:,2]
    exit_point = targets[-1] + rotations[-1,:,0]
    points = np.column_stack([np.vstack([initial[:3], targets, exit_point]), np.r_[initial[3], yaws, yaws[-1]]])
    return minimum_snap(points, durations), durations


def screen(profile):
    coefficients, durations = reconstruct(profile)
    # Include exactly each gate boundary as well as dense samples.
    times = np.unique(np.r_[np.arange(0., sum(durations), .005), np.cumsum(durations)])
    acceleration = evaluate(coefficients, durations, times, 2)[:,:3]
    jerk = evaluate(coefficients, durations, times, 3)[:,:3]
    force = acceleration + np.array([0.,0.,9.81])
    rate = np.linalg.norm(np.cross(force, jerk), axis=1) / np.maximum(np.sum(force*force, axis=1), 1e-8)
    race = (times >= durations[0]) & (times <= sum(durations[:-1]))
    velocity_at_gates = evaluate(coefficients, durations, np.cumsum(durations)[:-1], 1)[:,:3]
    normals = gate_geometry()[1][:,:,0]
    forward_speed = np.einsum('ij,ij->i', velocity_at_gates, normals)
    return dict(kind='offline_reference_only_not_flown',
                planned_gate_interval_s=float(sum(durations[1:-1])),
                speed_peak_mps=float(np.linalg.norm(evaluate(coefficients,durations,times,1)[:,:3],axis=1).max()),
                acceleration_peak_mps2=float(np.linalg.norm(acceleration,axis=1).max()),
                race_thrust_direction_rate_peak_rad_s=float(rate[race].max()),
                race_thrust_direction_rate_p95_rad_s=float(np.percentile(rate[race],95)),
                all_reference_gate_crossings_forward=bool(np.all(forward_speed > 0)),
                minimum_forward_gate_speed_mps=float(min(forward_speed)),
                nominal_minimum_gate_clearance_m=float(1.02-np.max(np.abs(np.r_[profile['gate_lateral_offsets'],profile['gate_vertical_offsets']]))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', type=Path)
    args = parser.parse_args()
    print(json.dumps(screen(yaml.safe_load(args.profile.read_text())), indent=2))


if __name__ == '__main__':
    main()
