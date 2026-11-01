#!/usr/bin/env python3
"""Generate reproducible LAB4 metrics and gate-crossing evidence from a bag."""

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np
import rosbag

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lab4_metrics import (  # noqa: E402
    aligned_position_errors,
    find_sequential_gate_crossings,
    last_monotonic_epoch,
    evaluate_completion,
    quaternion_matrix_xyzw,
)


def stamp_seconds(message, bag_stamp):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0.0:
        return stamp.to_sec()
    return bag_stamp.to_sec()


def point_from_desired(message):
    transform = message.transforms[0]
    return [transform.translation.x, transform.translation.y, transform.translation.z]


def yaw_from_quaternion(quaternion):
    """Return ROS yaw for any object exposing x, y, z and w."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def time_weighted_rmse(times, values):
    if len(times) < 2:
        return math.nan
    duration = float(times[-1] - times[0])
    if duration <= 0.0:
        return math.nan
    return float(np.sqrt(np.trapz(np.asarray(values) ** 2, times) / duration))


def json_safe(value):
    """Keep invalid recordings explicitly failed, with null undefined metrics."""
    if isinstance(value, dict):
        return {k: json_safe(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, (float,np.floating)) and not np.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--gate-width", type=float, default=2.4)
    parser.add_argument("--gate-height", type=float, default=2.4)
    parser.add_argument("--margin", type=float, default=0.18)
    parser.add_argument("--rmse-limit", type=float, default=0.75)
    parser.add_argument("--p95-limit", type=float, default=1.25)
    parser.add_argument("--gate-index-offset", type=int, default=0)
    args = parser.parse_args()

    actual_times, actual_samples = [], []
    desired_times, desired_samples, desired_tau = [], [], []
    gates = None
    statuses, terminal, invalid_reasons = [], [], []
    watchdog_events = []
    source_references = []
    controller_faults = []
    stamped_references, collision_events = [], []
    wall_intervals = []
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(
            topics=["/current_state", "/desired_state", "/desired_traj_vertices",
                    "/trajectory_status", "/controller_diagnostics",
                    "/tesse/rotor_watchdog_event", "/desired_state_stamped", "/tesse/collision"]
        ):
            if topic == "/current_state":
                position = message.pose.pose.position
                actual_times.append(stamp_seconds(message, bag_stamp))
                twist = message.twist.twist.linear
                angular = message.twist.twist.angular
                actual_samples.append([
                    position.x, position.y, position.z,
                    yaw_from_quaternion(message.pose.pose.orientation),
                    math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z),
                    math.sqrt(angular.x**2 + angular.y**2 + angular.z**2),
                ])
            elif topic == "/desired_state" and message.transforms:
                desired_times.append(bag_stamp.to_sec())
                point = point_from_desired(message)
                velocity = message.velocities[0].linear if message.velocities else None
                acceleration = (message.accelerations[0].linear
                                if message.accelerations else None)
                desired_samples.append([
                    point[0], point[1], point[2],
                    yaw_from_quaternion(message.transforms[0].rotation),
                    (math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                     if velocity else math.nan),
                    (math.sqrt(acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2)
                     if acceleration else math.nan),
                ])
                desired_tau.append(message.time_from_start.to_sec())
            elif topic == "/desired_traj_vertices" and message.poses:
                gates = message
            elif topic == "/trajectory_status":
                statuses.append(dict(
                    time=stamp_seconds(message, bag_stamp),
                    trajectory_id=message.trajectory_id, run_id=message.run_id,
                    phase=message.phase, planned_duration_s=message.planned_duration_s,
                    trajectory_elapsed_s=message.trajectory_elapsed_s,
                    mission_elapsed_s=message.mission_elapsed_s,
                    sample_tau_s=message.sample_tau_s,
                    reference_complete=message.reference_complete, fault=message.fault))
            elif topic == "/controller_diagnostics":
                t = stamp_seconds(message, bag_stamp)
                if message.state_interval_wall_s>=0:
                    wall_intervals.append((t,message.state_interval_wall_s))
                source_references.append((bag_stamp.to_sec(), message.reference_source_ros_s))
                if message.control_fault and message.task_phase_name not in ('WAITING', ''):
                    controller_faults.append((t, message.fault_reason))
                if message.task_phase_name in ('POST_RACE_HOLD', 'FINISHED'):
                    q = message.attitude
                    yaw_error = math.atan2(
                        math.sin(yaw_from_quaternion(q) - message.desired_yaw),
                        math.cos(yaw_from_quaternion(q) - message.desired_yaw))
                    norm = lambda v: math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
                    terminal.append([t, norm(message.position_error), norm(message.velocity),
                                     yaw_error, norm(message.body_angular_velocity)])
            elif topic == "/tesse/rotor_watchdog_event":
                watchdog_events.append((bag_stamp.to_sec(), message.data))
            elif topic == '/desired_state_stamped' and message.points:
                p=message.points[0]
                if message.header.stamp.to_sec()>0 and p.transforms:
                    v,a=p.velocities[0].linear,p.accelerations[0].linear
                    stamped_references.append((message.header.stamp.to_sec(),[
                        *point_from_desired(p),yaw_from_quaternion(p.transforms[0].rotation),
                        math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                        math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)],p.time_from_start.to_sec()))
            elif topic == '/tesse/collision' and message.is_collision:
                collision_events.append(dict(time=stamp_seconds(message,bag_stamp),object=message.object_name))

    mission_origins=[s['time']-s['mission_elapsed_s'] for s in statuses
                     if s['trajectory_id']>0 and np.isfinite(s['mission_elapsed_s'])]
    mission_start = float(np.median(mission_origins)) if mission_origins else None
    if mission_start is not None:
        terminal = [r for r in terminal if r[0] >= mission_start]
    for t, reason in controller_faults:
        if mission_start is None or t >= mission_start:
            invalid_reasons.append('controller fault: ' + reason)
    alignment_method = 'legacy desired_state receipt timestamps; no independent source metadata'
    if stamped_references:
        desired_times=[r[0] for r in stamped_references]
        desired_samples=[r[1] for r in stamped_references]
        desired_tau=[r[2] for r in stamped_references]
        alignment_method='explicit stamped reference header; no fitted time shift'
    elif statuses and desired_times:
        # The planner publishes a stamped status in the same callback as each
        # headerless reference. Match tau and choose the nearest recorded pair;
        # repeated terminal tau is disambiguated by receipt proximity.
        by_tau = defaultdict(list)
        for s in statuses:
            by_tau[round(s['sample_tau_s'], 6)].append(s['time'])
        aligned_reference_times = []
        for received, tau in zip(desired_times, desired_tau):
            if mission_start is not None and received < mission_start:
                aligned_reference_times.append(received)
                continue  # recorded preflight is explicitly reported below
            candidates = by_tau.get(round(tau, 6), [])
            if not candidates:
                invalid_reasons.append('reference without matching source metadata')
                aligned_reference_times.append(received)
                continue
            nearest = min(candidates, key=lambda t: abs(t-received))
            if abs(nearest-received) > .5:
                invalid_reasons.append('reference/source metadata pairing exceeds 0.5 s')
            aligned_reference_times.append(nearest)
        desired_times = aligned_reference_times
        alignment_method = 'desired tau paired with stamped same-callback trajectory_status'
    racing_collisions=[e for e in collision_events if mission_start is not None and e['time']>=mission_start]
    if racing_collisions:
        invalid_reasons.append('simulator reported collision during mission')

    # Never conceal a reset by selecting a later monotonic suffix.
    for name, times, samples in [('state', actual_times, actual_samples),
                                  ('reference', desired_times, desired_samples)]:
        if np.any(np.diff(times) < -1e-9):
            invalid_reasons.append(name + ' clock rollback')
        if not np.all(np.isfinite(samples)):
            invalid_reasons.append(name + ' nonfinite samples')
        if len(times) > 1 and np.max(np.diff(times)) > 0.5:
            invalid_reasons.append(name + ' missing for more than 0.5 seconds')
    if desired_times:
        for t, event in watchdog_events:
            if desired_times[0] <= t <= desired_times[-1]:
                try:
                    payload = json.loads(event)
                    event_source = payload.get('ros_time_s', t)
                    if mission_start is not None and event_source < mission_start:
                        continue  # latched event from before this new mission
                    if payload.get('event') not in ('STREAM_RESTORED', 'released', 'release'):
                        invalid_reasons.append('bridge watchdog event: ' + event)
                except (ValueError, TypeError):
                    invalid_reasons.append('unparsed bridge watchdog event: ' + event)

    actual_times, actual_samples = last_monotonic_epoch(
        actual_times, np.asarray(actual_samples, dtype=float)
    )
    desired_times, desired_samples = last_monotonic_epoch(
        desired_times, np.asarray(desired_samples, dtype=float)
    )
    if gates is None or len(actual_times) < 2 or len(desired_times) < 2:
        raise RuntimeError("bag lacks gates, current_state, or desired_state samples")

    actual_positions = actual_samples[:, :3]
    desired_positions = desired_samples[:, :3]

    gate_positions = np.array(
        [[pose.position.x, pose.position.y, pose.position.z] for pose in gates.poses]
    )
    gate_quaternions = np.array(
        [[pose.orientation.x, pose.orientation.y, pose.orientation.z,
          pose.orientation.w] for pose in gates.poses]
    )
    crossings = find_sequential_gate_crossings(
        actual_times,
        actual_positions,
        gate_positions,
        gate_quaternions,
        args.gate_width,
        args.gate_height,
        args.margin,
    )
    valid = (actual_times >= desired_times[0]) & (actual_times <= desired_times[-1])
    aligned_times = actual_times[valid]
    desired_interpolated = np.column_stack([
        np.interp(aligned_times, desired_times, desired_positions[:, axis])
        for axis in range(3)
    ])
    axis_errors = actual_positions[valid] - desired_interpolated
    errors = np.linalg.norm(axis_errors, axis=1)
    rmse = time_weighted_rmse(aligned_times, errors)
    axis_rmse = [time_weighted_rmse(aligned_times, axis_errors[:, axis])
                 for axis in range(3)]
    p95 = float(np.percentile(errors, 95)) if errors.size else math.nan
    max_error = float(np.max(errors)) if errors.size else math.nan

    actual_yaw = np.unwrap(actual_samples[:, 3])
    desired_yaw = np.unwrap(desired_samples[:, 3])
    desired_yaw_interpolated = np.interp(aligned_times, desired_times, desired_yaw)
    yaw_error = np.arctan2(
        np.sin(actual_yaw[valid] - desired_yaw_interpolated),
        np.cos(actual_yaw[valid] - desired_yaw_interpolated),
    )
    yaw_rmse = time_weighted_rmse(aligned_times, yaw_error)
    passed_gates = sum(crossing.passed for crossing in crossings)
    completion = evaluate_completion(statuses, terminal, passed_gates,
                                     len(gates.poses), rmse, p95, invalid_reasons,
                                     args.rmse_limit, args.p95_limit)
    reference_full_route = completion['reference_full_route']
    stable_full_route = completion['actual_stable_full_route']
    path_length = float(np.sum(np.linalg.norm(np.diff(actual_positions, axis=0), axis=1)))
    passed_crossings = [crossing for crossing in crossings if crossing.passed]
    first_to_last_gate_time = math.nan
    execution_to_last_gate_time = math.nan
    pre_last_gate_metrics = None
    if passed_crossings:
        execution_to_last_gate_time = float(
            passed_crossings[-1].time_s - desired_times[0]
        )
        if len(passed_crossings) > 1:
            first_to_last_gate_time = float(
                passed_crossings[-1].time_s - passed_crossings[0].time_s
            )
        before_last = aligned_times <= passed_crossings[-1].time_s
        if np.count_nonzero(before_last) >= 2:
            pre_errors = errors[before_last]
            pre_times = aligned_times[before_last]
            pre_last_gate_metrics = {
                "position_rmse_m": time_weighted_rmse(pre_times, pre_errors),
                "position_p95_m": float(np.percentile(pre_errors, 95)),
                "position_max_m": float(np.max(pre_errors)),
            }

    tracking_starts = [s['time']-s['trajectory_elapsed_s'] for s in statuses
                       if s['phase'] == 'TRACKING' and s['trajectory_id'] > 0]
    independent_tracking_start = (float(np.median(tracking_starts))
                                  if tracking_starts else None)
    independent_time_to_last_gate = (
        float(passed_crossings[-1].time_s-independent_tracking_start)
        if passed_crossings and independent_tracking_start is not None else None)

    actual_deltas = np.diff(actual_times)
    desired_deltas = np.diff(desired_times)

    post_start = min((s['time'] for s in statuses if s['phase'] == 'POST_RACE_HOLD'),
                     default=None)
    last_gate = passed_crossings[-1].time_s if passed_crossings else None
    settled_source = None
    # Earliest continuously stable terminal suffix (at least five seconds),
    # in addition to the unchanged independent 60 s post-settling regression.
    if post_start is not None and terminal:
        rows=np.array([r for r in terminal if r[0]>=post_start],dtype=float)
        if len(rows):
            good=(rows[:,1]<=.30)&(rows[:,2]<=.20)&(np.abs(rows[:,3])<=math.radians(10.))&(rows[:,4]<=.50)
            bad=np.flatnonzero(~good)
            first=(int(bad[-1])+1) if len(bad) else 0
            if first<len(rows) and rows[-1,0]-rows[first,0]>=5.:
                if np.max(np.diff(rows[first:,0]))<=.5:
                    settled_source=float(rows[first,0])
    segment_metrics = {}
    boundaries = [('whole_record', float(aligned_times[0]), float(aligned_times[-1]))]
    if last_gate is not None:
        boundaries.append(('course_through_last_crossed_gate',
                           mission_start if mission_start is not None else aligned_times[0], last_gate))
    if post_start is not None:
        if last_gate is not None:
            boundaries.append(('exit_deceleration', last_gate, post_start))
        boundaries.append(('terminal_hold', post_start, float(aligned_times[-1])))
        boundaries.append(('terminal_after_10s_settling', post_start+10., float(aligned_times[-1])))
    for name, begin, end in boundaries:
        mask = (aligned_times >= begin) & (aligned_times <= end)
        if np.count_nonzero(mask) < 2:
            segment_metrics[name] = None
            continue
        segment_metrics[name] = dict(
            duration_s=float(aligned_times[mask][-1]-aligned_times[mask][0]),
            position_rmse_m=time_weighted_rmse(aligned_times[mask], errors[mask]),
            position_p95_m=float(np.percentile(errors[mask],95)),
            position_max_m=float(errors[mask].max()),
            speed_max_mps=float(actual_samples[valid,4][mask].max()),
            yaw_error_max_deg=float(np.rad2deg(np.abs(yaw_error[mask])).max()),
            angular_speed_max_rad_s=float(actual_samples[valid,5][mask].max()))

    os.makedirs(args.output_dir, exist_ok=True)
    with open(os.path.join(args.output_dir, "gate_crossings.csv"), "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "gate_index", "sample_index", "time_s", "local_y_m", "local_z_m",
            "passed", "reason",
            "original_gate_index", "direction", "world_x_m", "world_y_m", "world_z_m",
            "remaining_clearance_y_m", "remaining_clearance_z_m",
        ])
        for crossing in crossings:
            intersection = gate_positions[crossing.gate_index] + quaternion_matrix_xyzw(
                gate_quaternions[crossing.gate_index]) @ np.array(
                    [0., crossing.local_y_m, crossing.local_z_m])
            writer.writerow([
                crossing.gate_index,
                crossing.sample_index,
                crossing.time_s,
                crossing.local_y_m,
                crossing.local_z_m,
                crossing.passed,
                crossing.reason,
                crossing.gate_index+args.gate_index_offset, 'forward local +x',
                *intersection.tolist(),
                args.gate_width/2-args.margin-abs(crossing.local_y_m),
                args.gate_height/2-args.margin-abs(crossing.local_z_m),
            ])

    metrics = {
        "bag": os.path.abspath(args.bag),
        "mission_start_source_s": mission_start,
        "recorded_preplan_duration_s": (max(0., mission_start-actual_times[0])
                                         if mission_start is not None else None),
        "actual_samples": int(len(actual_times)),
        "desired_samples": int(len(desired_times)),
        "gate_count": int(len(gates.poses)),
        "passed_gates": int(passed_gates),
        "path_length_m": path_length,
        "position_rmse_m": rmse,
        "position_axis_rmse_m": {"x": axis_rmse[0], "y": axis_rmse[1], "z": axis_rmse[2]},
        "position_p95_m": p95,
        "position_max_m": max_error,
        "yaw_rmse_rad": yaw_rmse,
        "yaw_rmse_deg": math.degrees(yaw_rmse),
        "actual_speed_peak_mps": float(np.nanmax(actual_samples[:, 4])),
        "desired_speed_peak_mps": float(np.nanmax(desired_samples[:, 4])),
        "desired_acceleration_peak_mps2": float(np.nanmax(desired_samples[:, 5])),
        "actual_record_frequency_hz": float((len(actual_times) - 1) /
                                             (actual_times[-1] - actual_times[0])),
        "desired_record_frequency_hz": float((len(desired_times) - 1) /
                                              (desired_times[-1] - desired_times[0])),
        "actual_interval_p95_s": float(np.percentile(actual_deltas, 95)),
        "actual_interval_max_s": float(np.max(actual_deltas)),
        "desired_interval_p95_s": float(np.percentile(desired_deltas, 95)),
        "desired_interval_max_s": float(np.max(desired_deltas)),
        "planned_trajectory_duration_s": next((s['planned_duration_s'] for s in statuses
                                                if s['trajectory_id'] > 0), None),
        "recorded_tau_max_s": float(max(desired_tau)) if desired_tau else None,
        "completion_verification": completion,
        "segments": segment_metrics,
        "execution_start_to_last_gate_s": execution_to_last_gate_time,
        "legacy_reference_receipt_to_last_gate_s": execution_to_last_gate_time,
        "trajectory_start_source_s": independent_tracking_start,
        "trajectory_start_to_last_gate_s": independent_time_to_last_gate,
        "mission_start_to_last_gate_s": (
            float(passed_crossings[-1].time_s-mission_start)
            if passed_crossings and mission_start is not None else None),
        "first_gate_to_last_gate_s": first_to_last_gate_time,
        "pre_last_gate": pre_last_gate_metrics,
        "reference_full_route": reference_full_route,
        "actual_stable_full_route": stable_full_route,
        "stable_rule": {
            "sequential_forward_gate_crossings": int(len(gates.poses)),
            "gate_plane": "local y-z; forward normal local +x",
            "gate_margin_m": args.margin,
            "rmse_limit_m": args.rmse_limit,
            "p95_limit_m": args.p95_limit,
            "terminal_settling_allowance_s": 10.0,
            "terminal_continuous_required_s": 60.0,
            "terminal_position_limit_m": 0.30,
            "terminal_speed_limit_mps": 0.20,
            "terminal_yaw_limit_deg": 10.0,
            "terminal_angular_speed_limit_rad_s": 0.50,
        },
        "time_alignment": (
            "current_state header stamp when valid, otherwise bag stamp; "
            + alignment_method + '; '
            "independent trajectory_status T and FINISHED required; clock rollbacks "
            "invalidate the complete recording before suffix diagnostics; duplicate stamps replaced; desired "
            "state linearly interpolated at actual-state stamps without phase shift; "
            "RMSE is trapezoidally time weighted"
        ),
        "collision_events_during_mission": racing_collisions,
        "terminal_stable_since_source_s": settled_source,
        "mission_start_to_settled_finish_s": (settled_source-mission_start
            if settled_source is not None and mission_start is not None else None),
        "tracking_start_to_settled_finish_s": (settled_source-independent_tracking_start
            if settled_source is not None and independent_tracking_start is not None else None),
    }
    if len(wall_intervals)>2 and mission_start is not None:
        wall_rows=np.asarray(wall_intervals,dtype=float)
        wall_cumulative=np.cumsum(wall_rows[:,1])
        def wall_duration(end):
            if end is None:return None
            return float(np.interp(end,wall_rows[:,0],wall_cumulative)-
                         np.interp(mission_start,wall_rows[:,0],wall_cumulative))
        metrics['mission_to_last_gate_wall_s_estimate']=wall_duration(last_gate)
        metrics['mission_to_settled_wall_s_estimate']=wall_duration(settled_source)
        metrics['wall_time_method']='integrated recorded controller state-arrival wall intervals, interpolated at source-time boundaries; approximate, not a fitted phase shift'
    metrics = json_safe(metrics)
    with open(os.path.join(args.output_dir, "metrics.json"), "w") as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True, allow_nan=False)
    print(json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
