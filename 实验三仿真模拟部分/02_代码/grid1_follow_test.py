# -*- coding: utf-8 -*-
"""
grid1_follow_test.py
============================================================
Grid1 “跟随式抓取” 搬运测试（Lua 成功方案移植）
============================================================
背景：
- Grid1 宽 0.12m > 夹爪张开间距 ~0.117m，夹爪物理夹不住，
  故按 Lua 成功方案用“物体跟随夹爪中心 + offset”实现仿真抓取。
- 机械臂控制（本场景实测）：
    s0=servo_motor_0 主升降（负角抬、正角降）直接 setJointTargetPosition
    s1=servo_motor_1 前伸/微调
- 夹爪开合走模型控制器信号 signal.target (1=open, 2=close)
- 本轮不动小车：把 Grid1 摆到夹爪正前方验证 抓->抬->运->放。

流程：
  A. home + OPEN
  B. 下降姿态 s0=+30 下测手指中点；把 Grid1 放到手指中点（Y 对齐，Z=0.11）
  C. “抓取”：记录 offset = Grid1中心 - 手指中点；之后每步 Grid1 = 手指中点 + offset
  D. 抬升 s0=-25（携带中保持跟随）
  E. 前伸搬运 s1=+45 再收回
  F. “放下”：解除跟随，Grid1 落到目标高度并停在原地
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

HOST = "localhost"
PORT = 23000

ST_OPEN = 1
ST_CLOSE = 2
APPROACH_S0 = 30.0
LIFT_S0 = -25.0
CARRY_S1 = 45.0
GRID_Z = 0.11


client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.require("sim")
print("连接成功（ZMQ %d）" % PORT)


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
if None in (s0, s1, tip, grp, left5, right5, grid1):
    raise RuntimeError("对象查找失败")

held = False
offset = None


def jdeg(h):
    return math.degrees(sim.getJointPosition(h))


def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))


def wait_arm(a0, a1, tol=1.0, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.1)
    return False


def gripper(target, timeout=8.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.1)
    return st


def finger_mid():
    l = pos(left5)
    r = pos(right5)
    return [(l[0] + r[0]) / 2, (l[1] + r[1]) / 2, (l[2] + r[2]) / 2]


def attach():
    """抓取：记录 offset（物体已经是静态不可碰撞，纯跟随）"""
    global held, offset
    g = pos(grid1)
    m = finger_mid()
    offset = [g[0] - m[0], g[1] - m[1], g[2] - m[2]]
    held = True
    print("   抓取：offset=(%.4f, %.4f, %.4f)" % (offset[0], offset[1], offset[2]))


def follow():
    """每个循环调用：Grid1 = 手指中点 + offset"""
    if not held:
        return
    m = finger_mid()
    setpos(grid1, [m[0] + offset[0], m[1] + offset[1], m[2] + offset[2]])


def release(drop_z=GRID_Z):
    global held, offset
    if held:
        # 放到指定高度（落到目标 Z），解除跟随
        p = pos(grid1)
        p[2] = drop_z
        setpos(grid1, p)
    held = False
    offset = None


def snap(tag):
    p = pos(grid1)
    print("  %-22s Grid1=(%.4f, %.4f, %.4f)  夹爪Z=%.4f s0=%+.1f s1=%+.1f" %
          (tag, p[0], p[1], p[2], pos(tip)[2], jdeg(s0), jdeg(s1)))


print("\n启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

# A. home + OPEN
set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
gripper(ST_OPEN)
print("① home + OPEN")

# B. 摆位（先把 Grid1 强制改为 静态+不可碰撞，否则它会掉/被撞飞）
try:
    sim.setObjectInt32Param(grid1, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(grid1, sim.shapeintparam_respondable, 0)
    print("   Grid1 -> static=1 respondable=0 (静态不可碰撞，可精确定位)")
except Exception as e:
    print("   改 Grid1 属性失败:", e)
set_deg(s0, APPROACH_S0); set_deg(s1, 0)
wait_arm(APPROACH_S0, 0)
time.sleep(0.5)
m = finger_mid()
print("② 下降到位，手指中点=(%.4f, %.4f, %.4f)" % (m[0], m[1], m[2]))
setpos(grid1, [m[0], m[1], GRID_Z])
time.sleep(0.5)
snap("Grid1 就位")

# C. 抓取（记录 offset）
print("③ 抓取（跟随式）")
attach()

# D. 抬升
set_deg(s0, LIFT_S0); set_deg(s1, 0)
while not (abs(jdeg(s0) - LIFT_S0) <= 1.0 and abs(jdeg(s1) - 0) <= 1.0):
    follow()
    time.sleep(0.05)
follow()
print("④ 抬升完成（携带 Grid1）")
snap("抬升后")

# E. 前伸搬运 -> 收回
set_deg(s1, CARRY_S1)
while abs(jdeg(s1) - CARRY_S1) > 1.0:
    follow(); time.sleep(0.05)
follow()
snap("前伸搬运")
set_deg(s1, 0)
while abs(jdeg(s1) - 0) > 1.0:
    follow(); time.sleep(0.05)
follow()
snap("收回")

# F. 放下
print("⑤ 放下")
release()
set_deg(s0, 0)
while abs(jdeg(s0) - 0) > 1.0:
    time.sleep(0.05)
gripper(ST_OPEN)
snap("放置完成")

print("\n完成。")
try:
    sim.stopSimulation()
    print("仿真已停止。")
except Exception:
    pass
