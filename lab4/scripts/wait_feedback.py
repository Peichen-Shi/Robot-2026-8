#!/usr/bin/env python3
"""Check live state AND clock using wall time, even when ROS time is frozen."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

parser = argparse.ArgumentParser()
parser.add_argument('--timeout', type=float, default=20)
parser.add_argument('--output')
args = parser.parse_args()
rospy.init_node('lab4_feedback_check', anonymous=True, disable_signals=True)
lock = threading.Lock()
samples = {'state': [], 'clock': []}


def record(name, stamp):
    now = time.monotonic()
    with lock:
        history = samples[name]
        if history and (stamp < history[-1][1] or now - history[-1][0] > 1):
            history.clear()
        if not history or stamp > history[-1][1]:
            history.append((now, stamp))
            del history[:-100]


def state(message):
    p = message.pose.pose.position
    if all(math.isfinite(value) for value in (p.x, p.y, p.z)):
        record('state', message.header.stamp.to_sec())


subs = [rospy.Subscriber('/current_state', Odometry, state, queue_size=10),
        rospy.Subscriber('/clock', Clock, lambda m: record('clock', m.clock.to_sec()), queue_size=10)]
deadline = time.monotonic() + args.timeout
ready = False
while time.monotonic() < deadline and not rospy.is_shutdown():
    with lock:
        now = time.monotonic()
        ready = all(len(h) >= 5 and h[-1][1] - h[0][1] >= .2 and now - h[-1][0] < .75
                    for h in samples.values())
    if ready:
        break
    time.sleep(.025)
with lock:
    result = {'ready': ready, 'topics': {name: {
        'advancing_samples': len(h), 'first_stamp': h[0][1] if h else None,
        'last_stamp': h[-1][1] if h else None} for name, h in samples.items()}}
for sub in subs:
    sub.unregister()
if args.output:
    Path(args.output).write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps(result), flush=True)
raise SystemExit(0 if ready else 1)
