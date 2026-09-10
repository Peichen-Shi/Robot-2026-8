# -*- coding: utf-8 -*-
"""
demo.py —— 连贯演示（请在 CoppeliaSim 窗口观看）
1) 夹爪 OPEN / CLOSE / OPEN（看手指真实开合）
2) 机械臂 home -> 抬升 -> 下降 -> home（看臂升降）
3) Grid1 摆到手指正前方 -> 跟随"抓起" -> 抬升 -> 前伸搬运 -> 放回原位
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

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

def pos(h):
    return sim.getObjectPosition(h, sim.handle_world)

def setpos(h, xyz):
    sim.setObjectPosition(h, sim.handle_world, xyz)

s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
grid1 = find("Grid1")

GRID_ORIGIN = [0.38, -0.5, 0.11]

def jdeg(h):
    return math.degrees(sim.getJointPosition(h))

def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))

def wait_arm(a0, a1, tol=1.0, timeout=12.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.05)
    return False

def gripper(target, timeout=10.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.05)
    return st

def finger_mid():
    l = pos(left5); r = pos(right5)
    return [(l[0]+r[0])/2, (l[1]+r[1])/2, (l[2]+r[2])/2]

def finger_gap():
    l = pos(left5); r = pos(right5)
    return ((l[0]-r[0])**2 + (l[1]-r[1])**2 + (l[2]-r[2])**2) ** 0.5

def title(s):
    print("\n" + "=" * 60)
    print(s)
    print("=" * 60)

def pause(sec=1.0):
    time.sleep(sec)

# ============================================================
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.2)

# ---- 演示 1：夹爪开合 ----
title("演示 1：夹爪真实开合（看手指）")
set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
gripper(1)
print("张开：手指间距 = %.4f m" % finger_gap()); pause(1.2)
gripper(2)
print("闭合：手指间距 = %.4f m" % finger_gap()); pause(1.2)
gripper(1)
print("再次张开 = %.4f m" % finger_gap()); pause(1.0)

# ---- 演示 2：机械臂升降 ----
title("演示 2：机械臂 抬升/下降（看臂）")
set_deg(s0, -25); wait_arm(-25, 0)
print("抬升到位：夹爪Z = %.4f m" % pos(tip)[2]); pause(1.2)
set_deg(s0, 30); wait_arm(30, 0)
print("下降到位：夹爪Z = %.4f m" % pos(tip)[2]); pause(1.2)
set_deg(s0, 0); wait_arm(0, 0)
print("回 home：夹爪Z = %.4f m" % pos(tip)[2]); pause(0.8)

# ---- 演示 3：Grid1 抓起搬运 ----
title("演示 3：Grid1 跟随抓取 -> 抬升 -> 搬运 -> 放回")
# 先把物体设为静态不可碰撞，并摆到手指正前方
try:
    sim.setObjectInt32Param(grid1, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(grid1, sim.shapeintparam_respondable, 0)
except Exception:
    pass
set_deg(s0, 30); set_deg(s1, 0); wait_arm(30, 0)
time.sleep(0.3)
m = finger_mid()
setpos(grid1, [m[0], m[1], 0.11])
time.sleep(0.6)
print("Grid1 已放到手指正前方 (%.3f, %.3f)" % (m[0], m[1])); pause(1.0)

# 抓取：记录 offset
g = pos(grid1)
offset = [g[0]-m[0], g[1]-m[1], g[2]-m[2]]
held = True
print("抓取！offset=(%.4f,%.4f,%.4f)" % tuple(offset)); pause(0.8)

def follow():
    mm = finger_mid()
    setpos(grid1, [mm[0]+offset[0], mm[1]+offset[1], mm[2]+offset[2]])

# 抬升（携带）
set_deg(s0, -25)
while abs(jdeg(s0)+25) > 1.0:
    follow(); time.sleep(0.03)
follow()
print("抬升完成：Grid1 Z = %.3f" % pos(grid1)[2]); pause(1.5)

# 前伸搬运
set_deg(s1, 45)
while abs(jdeg(s1)-45) > 1.0:
    follow(); time.sleep(0.03)
follow()
print("前伸搬运：Grid1 X = %.3f" % pos(grid1)[0]); pause(1.5)

# 收回
set_deg(s1, 0)
while abs(jdeg(s1)) > 1.0:
    follow(); time.sleep(0.03)
follow()
print("收回：Grid1 (%.3f, %.3f, %.3f)" % tuple(pos(grid1))); pause(1.0)

# 放下并放回原位
held = False
setpos(grid1, GRID_ORIGIN)
set_deg(s0, 0)
while abs(jdeg(s0)) > 1.0:
    time.sleep(0.03)
gripper(1)
print("已把 Grid1 放回原位 (0.38, -0.50)"); pause(1.0)

title("演示结束")
try:
    sim.stopSimulation()
    print("仿真已停止。")
except Exception:
    pass
