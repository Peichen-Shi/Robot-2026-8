# -*- coding: utf-8 -*-
"""probe_binclear.py —— 用碰撞检测找出"车体/手臂开始碰到料盒"的临界 x，确定安全停靠位置"""
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

BIN_PARTS = ["LeftBin_bottom", "LeftBin_front", "LeftBin_back", "LeftBin_left", "LeftBin_right",
             "RightBin_bottom", "RightBin_front", "RightBin_back", "RightBin_left", "RightBin_right"]
walls = [(n, find(n)) for n in BIN_PARTS]

def set_deg(h, d): sim.setJointTargetPosition(h, math.radians(d))
def jdeg(h): return math.degrees(sim.getJointPosition(h))
def pos(h): return sim.getObjectPosition(h, sim.handle_world)

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(1.0)

def wait_arm(a0, a1, t=12):
    t0 = time.time()
    while time.time() - t0 < t:
        if abs(jdeg(s0) - a0) < 1.0 and abs(jdeg(s1) - a1) < 1.0:
            return
        time.sleep(0.05)

def collisions(tag):
    hit = []
    for n, h in walls:
        if h is None:
            continue
        try:
            r = sim.checkCollision(robot, h)
            # ZMQ 返回可能是 int 或 (result, handles)
            val = r[0] if isinstance(r, (list, tuple)) else r
        except Exception as e:
            val = "ERR:%s" % e
        if val not in (0, "0"):
            hit.append((n, val))
    b = pos(robot)
    print("  %-34s 车体 x=%.4f  →  碰撞: %s" % (tag, b[0], ("无 ✅" if not hit else hit)))
    return hit

print("== 1) 搬运姿态（手臂收起）在不同车体 x 下的碰撞 ==")
set_deg(s0, -25); set_deg(s1, 0); wait_arm(-25, 0)
for x in [0.058, 0.045, 0.030, 0.015, 0.000, -0.020]:
    sim.setObjectPosition(robot, sim.handle_world, [x, -0.90 + 0.0167, pos(robot)[2]])
    time.sleep(0.5)
    collisions("lift 姿态  x=%.3f" % x)

print("\n== 2) 伸入姿态（手臂伸向箱内）在不同车体 x 下的碰撞 ==")
set_deg(s0, -20); set_deg(s1, 60); wait_arm(-20, 60)
for x in [0.058, 0.045, 0.030, 0.015, 0.000, -0.020]:
    sim.setObjectPosition(robot, sim.handle_world, [x, -0.90 + 0.0167, pos(robot)[2]])
    time.sleep(0.6)
    collisions("insert 姿态(s0=-20,s1=60) x=%.3f" % x)

print("\n== 3) 参考：车体在 x=0.03 时，机械臂各关节高度（看手臂是否低于箱壁顶 0.08） ==")
sim.setObjectPosition(robot, sim.handle_world, [0.03, -0.90 + 0.0167, pos(robot)[2]])
time.sleep(0.6)
pjpos = pos(pj)
print("  Prismatic_joint 世界坐标 = (%.3f, %.3f, %.3f)" % tuple(pjpos))
l5 = find("left_gripper_5_respondable"); r5 = find("right_gripper_5_respondable")
if l5 and r5:
    pl, pr = pos(l5), pos(r5)
    print("  手指接触带 z = %.3f / %.3f （箱壁顶约 0.08）" % (pl[2], pr[2]))

sim.stopSimulation()
print("done")
