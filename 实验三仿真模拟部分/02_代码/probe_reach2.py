# -*- coding: utf-8 -*-
"""probe_reach2.py —— 精简版：车体尺寸 + 机械臂伸展映射（快速、带刷新）"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math, sys

client = RemoteAPIClient(host="localhost", port=23000)
sim = client.require("sim")

def find(bare):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None

def P(*a): print(*a, flush=True)

robot = find("RoboMaster")
s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")

def pos(h): return sim.getObjectPosition(h, sim.handle_world)
def set_deg(h, d): sim.setJointTargetPosition(h, math.radians(d))

def finger_mid():
    l = pos(left5); r = pos(right5)
    return [(l[0]+r[0])/2, (l[1]+r[1])/2, (l[2]+r[2])/2]

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(1.0)
P("sim running")

# 车体尺寸
parts = ["front_left_wheel_link_visual", "front_right_wheel_link_visual",
         "rear_left_wheel_link_visual", "rear_right_wheel_link_visual",
         "front_hit_sensor_link_visual", "rear_hit_sensor_link_visual",
         "left_hit_sensor_link_visual", "right_hit_sensor_link_visual",
         "base_link_visual", "extension_base_link_visual",
         "intelligent_controller_link_visual"]
b0 = pos(robot)
xs, ys = [], []
P("车体基座 (%.3f, %.3f, %.3f)" % tuple(b0))
for p in parts:
    h = find(p)
    if h is None:
        continue
    v = pos(h)
    dx, dy = v[0]-b0[0], v[1]-b0[1]
    xs.append(dx); ys.append(dy)
    P("  %-32s dx=%+.3f dy=%+.3f dz=%+.3f" % (p, dx, dy, v[2]-b0[2]))
if xs:
    P("车体部件范围: dx %.3f..%.3f   dy %.3f..%.3f  (实际外形再各外扩约 0.03)" %
      (min(xs), max(xs), min(ys), max(ys)))

# 夹爪闭合，与搬运状态一致
sim.setIntProperty(grp, "signal.target", 2)
t0 = time.time()
while time.time() - t0 < 6:
    if sim.getIntProperty(grp, "signal.state") == 2:
        break
    time.sleep(0.05)

P("\n姿态 -> 手指中点相对车体偏移:")
P("%6s %6s | %8s %8s %8s | %7s" % ("s0", "s1", "dx", "dy", "dz", "midZ"))
cands = []
for a0 in [-20, 0, 20]:
    for a1 in [40, 60, 80]:
        set_deg(s0, a0); set_deg(s1, a1)
        time.sleep(1.0)
        m = finger_mid(); b = pos(robot)
        dx, dy, dz = m[0]-b[0], m[1]-b[1], m[2]-b[2]
        P("%6.0f %6.0f | %+8.3f %+8.3f %+8.3f | %7.3f" % (a0, a1, dx, dy, dz, m[2]))
        cands.append((dx, dz, a0, a1, m[2]))

P("\n满足 dx>=0.26 的候选:")
for dx, dz, a0, a1, mz in cands:
    if dx >= 0.26:
        P("   s0=%+.0f s1=%+.0f -> dx=%.3f midZ=%.3f" % (a0, a1, dx, mz))

set_deg(s0, 0); set_deg(s1, 0); time.sleep(0.8)
sim.stopSimulation()
P("done")
