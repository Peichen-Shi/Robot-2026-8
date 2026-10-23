#!/usr/bin/env python3
"""Numerical helpers for LAB4 bag analysis (independent of ROS imports)."""

from dataclasses import dataclass
from typing import Iterable, List, Sequence

import numpy as np


def evaluate_completion(statuses, terminal, passed_gates, gate_count, rmse, p95,
                        invalid_reasons=(), rmse_limit=0.75, p95_limit=1.25):
    """Fail closed using independent planner metadata and a continuous hold.

    statuses are dictionaries in recorded order. terminal contains timestamp,
    position error, speed, yaw error and angular speed for POST_RACE_HOLD and
    FINISHED only. Thresholds are repair-regression criteria, not MIT grading.
    """
    reasons = list(invalid_reasons)
    active = [s for s in statuses if s['trajectory_id'] > 0]
    ids = {s['trajectory_id'] for s in active}
    reference_complete = False
    hold_duration = 0.0
    if len(ids) != 1:
        reasons.append('missing or multiple trajectory IDs')
    elif active:
        times = np.array([s['time'] for s in active])
        durations = np.array([s['planned_duration_s'] for s in active])
        elapsed = np.array([s['trajectory_elapsed_s'] for s in active])
        tau = np.array([s['sample_tau_s'] for s in active])
        run_ids = {s.get('run_id','') for s in active}
        if (not np.all(np.isfinite(durations)) or np.min(durations) <= 0 or
                np.ptp(durations) > 1e-6 or np.any(np.diff(times) < -1e-9) or
                not np.all(np.isfinite(elapsed)) or not np.all(np.isfinite(tau)) or
                np.any(np.diff(elapsed) < -1e-6) or np.any(np.diff(tau) < -1e-6) or len(run_ids)>1):
            reasons.append('invalid independent plan or clock rollback')
        else:
            total = float(durations[0])
            reference_complete = any(
                s['phase'] == 'FINISHED' and s['reference_complete'] and
                s['sample_tau_s'] >= total - 1e-3 and
                s['trajectory_elapsed_s'] >= total + 60.0 - 1e-3
                for s in active)
        if any(s['fault'] or s['phase'] == 'FAULT' for s in active):
            reasons.append('planner fault')
    if not reference_complete:
        reasons.append('no independently verified reference completion')
    if gate_count != 12 or passed_gates != gate_count:
        reasons.append('incomplete sequential gate crossings')
    if not np.isfinite(rmse) or not np.isfinite(p95) or rmse > rmse_limit or p95 > p95_limit:
        reasons.append('whole-run position limits exceeded')
    samples = np.asarray(terminal, dtype=float)
    hold_starts = [s['time'] for s in active if s['phase'] == 'POST_RACE_HOLD']
    if not hold_starts:
        reasons.append('missing independent terminal phase start')
    if samples.ndim == 2 and samples.shape[0] >= 2 and samples.shape[1] == 5:
        # A fixed 10-second settling allowance is declared before testing.
        hold_start = min(hold_starts) if hold_starts else samples[0, 0]
        samples = samples[samples[:, 0] >= hold_start + 10.0]
        if len(samples) >= 2:
            hold_duration = float(samples[-1, 0] - samples[0, 0])
            if (not np.all(np.isfinite(samples)) or
                    np.any(np.diff(samples[:, 0]) <= 0) or
                    np.max(np.diff(samples[:, 0])) > 0.5 or
                    np.any(samples[:, 1] > 0.30) or
                    np.any(samples[:, 2] > 0.20) or
                    np.any(np.abs(samples[:, 3]) > np.deg2rad(10)) or
                    np.any(samples[:, 4] > 0.5)):
                reasons.append('terminal stability or continuity limits exceeded')
    if hold_duration < 60.0 - 0.05:
        reasons.append('less than 60 seconds continuous terminal hold after settling')
    return {'reference_full_route': reference_complete,
            'actual_stable_full_route': not reasons,
            'terminal_hold_duration_s': hold_duration,
            'failure_reasons': sorted(set(reasons))}


@dataclass
class GateCrossing:
    gate_index: int
    sample_index: int
    time_s: float
    local_y_m: float
    local_z_m: float
    passed: bool
    reason: str


def quaternion_matrix_xyzw(quaternion: Sequence[float]) -> np.ndarray:
    """Return the local-to-world rotation matrix for an xyzw quaternion."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def last_monotonic_epoch(times: Iterable[float], values: np.ndarray):
    """Keep the last monotonic epoch and remove duplicate timestamps.

    A decreasing timestamp denotes a simulator reset.  For duplicates, the
    newest sample replaces the older one, matching ROS's latest-state meaning.
    """
    times = np.asarray(list(times), dtype=float)
    values = np.asarray(values)
    if len(times) != len(values):
        raise ValueError("time/value length mismatch")
    if len(times) == 0:
        return times, values
    rollback = np.flatnonzero(np.diff(times) < -1e-9)
    start = int(rollback[-1] + 1) if rollback.size else 0
    times = times[start:]
    values = values[start:]
    unique_times: List[float] = []
    unique_values: List[np.ndarray] = []
    for stamp, value in zip(times, values):
        if not np.isfinite(stamp):
            continue
        if unique_times and abs(stamp - unique_times[-1]) <= 1e-9:
            unique_values[-1] = value
        elif not unique_times or stamp > unique_times[-1]:
            unique_times.append(float(stamp))
            unique_values.append(value)
    return np.asarray(unique_times), np.asarray(unique_values)


def find_sequential_gate_crossings(
    times: Sequence[float],
    positions: np.ndarray,
    gate_positions: np.ndarray,
    gate_quaternions: np.ndarray,
    gate_width_m: float = 2.4,
    gate_height_m: float = 2.4,
    margin_m: float = 0.18,
) -> List[GateCrossing]:
    """Check forward crossings in gate order using each gate's local frame.

    The TESSE race gate lies in local y-z; local +x is the legal forward
    normal.  A pass requires a negative-to-nonnegative local-x sign change and
    an interpolated crossing point inside the aperture after applying margin.
    """
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    gate_positions = np.asarray(gate_positions, dtype=float)
    gate_quaternions = np.asarray(gate_quaternions, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("positions must be N x 3")
    if len(times) != len(positions):
        raise ValueError("time/position length mismatch")
    half_width = max(0.0, gate_width_m / 2.0 - margin_m)
    half_height = max(0.0, gate_height_m / 2.0 - margin_m)
    search_start = 0
    results: List[GateCrossing] = []
    for gate_index, (centre, quaternion) in enumerate(
        zip(gate_positions, gate_quaternions)
    ):
        rotation = quaternion_matrix_xyzw(quaternion)
        local = (positions - centre) @ rotation
        crossing = None
        for sample_index in range(max(1, search_start + 1), len(times)):
            x0, x1 = local[sample_index - 1, 0], local[sample_index, 0]
            if x0 <= 0.0 < x1 and x1 - x0 > 1e-9:
                alpha = -x0 / (x1 - x0)
                point = local[sample_index - 1] + alpha * (
                    local[sample_index] - local[sample_index - 1]
                )
                stamp = times[sample_index - 1] + alpha * (
                    times[sample_index] - times[sample_index - 1]
                )
                inside = abs(point[1]) <= half_width and abs(point[2]) <= half_height
                crossing = GateCrossing(
                    gate_index=gate_index,
                    sample_index=sample_index,
                    time_s=float(stamp),
                    local_y_m=float(point[1]),
                    local_z_m=float(point[2]),
                    passed=bool(inside),
                    reason="inside aperture" if inside else "plane crossed outside aperture",
                )
                if inside:
                    search_start = sample_index
                break
        if crossing is None:
            crossing = GateCrossing(
                gate_index=gate_index,
                sample_index=-1,
                time_s=float("nan"),
                local_y_m=float("nan"),
                local_z_m=float("nan"),
                passed=False,
                reason="no forward plane crossing",
            )
        results.append(crossing)
        if not crossing.passed:
            break
    return results


def aligned_position_errors(
    actual_times: Sequence[float],
    actual_positions: np.ndarray,
    desired_times: Sequence[float],
    desired_positions: np.ndarray,
) -> np.ndarray:
    actual_times = np.asarray(actual_times, dtype=float)
    desired_times = np.asarray(desired_times, dtype=float)
    actual_positions = np.asarray(actual_positions, dtype=float)
    desired_positions = np.asarray(desired_positions, dtype=float)
    if len(desired_times) < 2 or len(actual_times) < 1:
        return np.asarray([], dtype=float)
    valid = (actual_times >= desired_times[0]) & (actual_times <= desired_times[-1])
    if not np.any(valid):
        return np.asarray([], dtype=float)
    reference = np.column_stack(
        [np.interp(actual_times[valid], desired_times, desired_positions[:, axis])
         for axis in range(3)]
    )
    return np.linalg.norm(actual_positions[valid] - reference, axis=1)
