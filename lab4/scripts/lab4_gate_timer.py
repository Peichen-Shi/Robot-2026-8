#!/usr/bin/env python3
"""Read-only first-to-last actual gate timer. Never publishes control commands."""
import argparse
import json
import math
import os
import threading
import time
from pathlib import Path
import numpy as np
from lab4_metrics import find_sequential_gate_crossings


class GateTimer:
    def __init__(self, max_gap=.5):
        self.max_gap = max_gap
        self.gates = None
        self.identity = None
        self.reset()

    def reset(self):
        self.previous = None
        self.start = self.finish = None
        self.crossings = []
        self.invalid = ''
        self.armed = False

    def set_gates(self, poses):
        gates = np.asarray(poses, dtype=float)
        if gates.ndim != 2 or gates.shape[1] != 7 or len(gates) < 2 or not np.isfinite(gates).all():
            self.invalid = 'invalid gate geometry'
            return
        if self.gates is not None and not np.array_equal(self.gates, gates):
            self.invalid = 'gate geometry changed; restart timer for a new course'
            return
        self.gates = gates.copy()

    def status(self, run_id, trajectory_id, phase, fault=False):
        if trajectory_id <= 0:
            return
        identity = (run_id, trajectory_id)
        if identity != self.identity:
            self.reset()
            self.identity = identity
        if fault or phase == 'FAULT':
            self.invalid = 'planner fault'
        if phase in ('READY_HOLD', 'TRACKING'):
            self.armed = True

    def state(self, stamp, position):
        position = np.asarray(position, dtype=float)
        if not math.isfinite(stamp) or position.shape != (3,) or not np.isfinite(position).all():
            self.invalid = 'nonfinite odometry'
            return None
        if self.previous is not None:
            old_t, old_p = self.previous
            if stamp < old_t:
                self.invalid = 'ROS time rollback; restart timer after reset'
            if stamp == old_t:
                self.previous = (stamp, position)
                return None
            if self.armed and not self.finish and stamp-old_t > self.max_gap:
                self.invalid = 'odometry gap exceeds %.3f s' % self.max_gap
        event = None
        if (self.previous is not None and self.armed and self.gates is not None
                and not self.invalid and self.finish is None):
            old_t, old_p = self.previous
            index = len(self.crossings)
            gate = self.gates[index:index+1]
            result = find_sequential_gate_crossings(
                [old_t,stamp], np.asarray([old_p,position]), gate[:,:3],gate[:,3:])
            crossing = result[0]
            if crossing.sample_index >= 0:
                if not crossing.passed:
                    self.invalid = 'gate %d plane crossed outside aperture' % (index+1)
                else:
                    self.crossings.append(crossing.time_s)
                    event = 'GATE %d/%d' % (index+1,len(self.gates))
                    if self.start is None:
                        self.start = crossing.time_s
                        event = 'START at gate 1/%d' % len(self.gates)
                    if len(self.crossings) == len(self.gates):
                        self.finish = crossing.time_s
                        event = 'FINISH at gate %d/%d' % (len(self.gates),len(self.gates))
        self.previous = (stamp,position)
        return event

    def result(self):
        end = self.finish if self.finish is not None else (self.previous[0] if self.previous else None)
        elapsed = (end-self.start)*1000 if self.start is not None and end is not None else 0.
        phase = ('INVALID' if self.invalid else 'FINISHED' if self.finish is not None
                 else 'RUNNING' if self.start is not None else 'WAIT_FIRST_GATE')
        crossing_elapsed = (self.crossings[-1]-self.start)*1000 if self.crossings else 0.
        return dict(phase=phase,elapsed_ms=elapsed,crossing_elapsed_ms=crossing_elapsed,passed_gates=len(self.crossings),
                    total_gates=len(self.gates) if self.gates is not None else 0,
                    first_gate_ros_s=self.start,last_gate_ros_s=self.finish,
                    crossing_ros_s=list(self.crossings),run_id=self.identity[0] if self.identity else '',
                    reason=self.invalid,clock='odometry header / ROS simulation time',
                    rule='Sequential forward local +x crossings inside 2.4 m aperture with 0.18 m body margin; first gate to last gate only.')


def gate_poses(message):
    return [[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,
             p.orientation.z,p.orientation.w] for p in message.poses]


def consume(timer, topic, message, fallback):
    if topic == '/desired_traj_vertices':
        timer.set_gates(gate_poses(message))
    elif topic == '/trajectory_status':
        timer.status(message.run_id,message.trajectory_id,message.phase,message.fault)
    elif topic == '/current_state':
        p = message.pose.pose.position
        return timer.state(message.header.stamp.to_sec() or fallback,[p.x,p.y,p.z])
    return None


def save_result(path, result):
    if not path:
        return
    path = Path(path).resolve()
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    os.replace(str(temporary),str(path))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',help='Save result JSON on finish, invalidation or shutdown')
    parser.add_argument('--bag',help='Offline validation only: read existing bag, no live control')
    args,_=parser.parse_known_args()
    timer=GateTimer()
    if args.bag:
        import rosbag
        with rosbag.Bag(args.bag) as bag:
            for topic,m,t in bag.read_messages(topics=['/desired_traj_vertices','/trajectory_status','/current_state']):
                event=consume(timer,topic,m,t.to_sec())
                if event:
                    print('[GATE TIMER] %s | %.0f ms' % (event,timer.result()['crossing_elapsed_ms']),flush=True)
        result=timer.result();save_result(args.output,result)
        print(json.dumps(result,indent=2))
        return
    import rospy
    from geometry_msgs.msg import PoseArray
    from nav_msgs.msg import Odometry
    from trajectory_generation.msg import TrajectoryStatus
    rospy.init_node('lab4_gate_timer',anonymous=True)
    lock=threading.Lock()
    last_arrival=[None]
    def callback(topic,m):
        with lock:
            event=consume(timer,topic,m,rospy.Time.now().to_sec())
            if topic == '/current_state':
                last_arrival[0]=time.monotonic()
            if event:
                result=timer.result()
                print('[GATE TIMER] %s | %.0f ms' % (event,result['crossing_elapsed_ms']),flush=True)
                if timer.finish is not None:
                    save_result(args.output,result)
    rospy.Subscriber('/desired_traj_vertices',PoseArray,lambda m:callback('/desired_traj_vertices',m),queue_size=10)
    rospy.Subscriber('/trajectory_status',TrajectoryStatus,lambda m:callback('/trajectory_status',m),queue_size=100)
    rospy.Subscriber('/current_state',Odometry,lambda m:callback('/current_state',m),queue_size=100)
    print('[GATE TIMER] READY | WAIT_FIRST_GATE | 0 ms | ROS simulation time; crossing times interpolated from measured odometry.',flush=True)
    try:
        while not rospy.is_shutdown():
            with lock:
                result=timer.result()
                stale=last_arrival[0] is None or time.monotonic()-last_arrival[0]>.5
                print('[GATE TIMER] %s | %.0f ms | gates %d/%d%s%s' % (
                    result['phase'],result['elapsed_ms'],result['passed_gates'],result['total_gates'],
                    ' | waiting for fresh state' if stale else '',
                    ' | '+result['reason'] if result['reason'] else ''),flush=True)
            time.sleep(.2)  # Wall-paced output continues while simulator time is paused.
    finally:
        with lock:
            save_result(args.output,timer.result())


if __name__ == '__main__':
    main()
