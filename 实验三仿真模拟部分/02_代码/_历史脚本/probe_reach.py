# -*- coding: utf-8 -*-
"""probe_reach.py —— 量机械臂伸展范围 + 车体尺寸（用于"车在箱外、手臂伸进去放"）"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time, math

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

robot = find("RoboMaster")
s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")

def pos(h): return sim.getObjectPosition(h, sim.handle_world)
def set_deg(h, d): sim.setJointTargetPosition(h, math.radians(d))
def jdeg(h): return math.degrees(sim.getJointPosition(h))

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

# 夹爪闭合（与搬运状态一致）
sim.setIntProperty(grp, "signal.target", 2)
t0 = time.time()
while time.time() - t0 < 6:
    if sim.getIntProperty(grp, "signal.state") == 2:
        break
    time.sleep(0.05)

# ---- 车体尺寸（各部件世界坐标相对车体）----
parts = ["front_left_wheel_link_visual", "front_right_wheel_link_visual",
         "rear_left_wheel_link_visual", "rear_right_wheel_link_visual",
         "front_hit_sensor_link_visual", "rear_hit_sensor_link_visual",
         "left_hit_sensor_link_visual", "right_hit_sensor_link_visual",
         "base_link_visual", "intelligent_controller_link_visual",
         "extension_base_link_visual"]
b0 = pos(robot)
print("车体中心 = (%.3f, %.3f, %.3f)" % tuple(b0))
xs, ys = [], []
for p in parts:
    h = find(p)
    if h is None:
        continue
    v = pos(h)
    xs.append(v[0] - b0[0]); ys.append(v[1] - b0[1])
    print("  %-34s d=(%+.3f, %+.3f, %+.3f)" % (p, v[0]-b0[0], v[1]-b0[1], v[2]-b0[2]))
print("部件范围: dx %.3f..%.3f   dy %.3f..%.3f" % (min(xs), max(xs), min(ys), max(ys)))

# ---- 机械臂伸展映射 ----
print("\n机械臂姿态 -> 手指中点相对车体偏移(dx, dy, dz), 及中点世界 Z")
print("%6s %6s | %8s %8s %8s | %8s" % ("s0", "s1", "dx", "dy", "dz", "midZ"))
best = []
for a0 in [-30, -20, -10, 0, 10, 20, 30, 40]:
    for a1 in [0, 20, 40, 60, 80]:
        set_deg(s0, a0); set_deg(s1, a1)
        time.sleep(1.1)
        m = finger_mid(); b = pos(robot)
        dx, dy, dz = m[0]-b[0], m[1]-b[1], m[2]-b[2]
        print("%6.0f %6.0f | %+8.3f %+8.3f %+8.3f | %8.3f" % (a0, a1, dx, dy, dz, m[2]))
        best.append((dx, dz, a0, a1, m[2]))

# 目标：dx≈0.26~0.30 且 中点 z 尽量接近 0.22 的组合
print("\n可用候选（dx>=0.24 且 midZ 在 0.19~0.26）:")
for dx, dz, a0, a1, mz in best:
    if dx >= 0.24 and 0.19 <= mz <= 0.26:
        print("   s0=%+.0f s1=%+.0f -> dx=%.3f midZ=%.3f" % (a0, a1, dx, mz))

# 收尾归位
set_deg(s0, 0); set_deg(s1, 0); time.sleep(1.0)
sim.stopSimulation()
print("done")
