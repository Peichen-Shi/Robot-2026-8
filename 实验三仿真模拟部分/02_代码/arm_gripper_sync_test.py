# -*- coding: utf-8 -*-
"""
arm_gripper_sync_test.py
============================================================
RoboMaster EP 机械臂 + 夹爪协同测试
============================================================
流程（交接清单第 3 步）：
1. home：s0=0, s1=0，夹爪 OPEN
2. 下降：s0=+30°（夹爪随臂降低）
3. 夹爪 CLOSE（真实闭合）
4. 抬升：s0=-25°（夹爪随臂抬高，模拟“抓起”）
5. 夹爪 OPEN（模拟“放下”）
6. 回 home

控制要点（本场景实测）：
- 机械臂 servo_motor_0/1：位置环开着，直接 setJointTargetPosition
- 夹爪：走模型控制器信号 signal.target（1=open 2=close），轮询 signal.state
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

HOST = "localhost"
PORT = 23000

ST_OPEN = 1
ST_CLOSE = 2

LOWER_DEG = 30.0
LIFT_DEG = -25.0


# ============================================================
# 连接 & 找对象
# ============================================================
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


s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
if None in (s0, s1, tip, grp, left5, right5):
    raise RuntimeError("对象查找失败")


# ============================================================
# 工具
# ============================================================
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))


def tip_z():
    return sim.getObjectPosition(tip, sim.handle_world)[2]


def finger_gap():
    lp = sim.getObjectPosition(left5, sim.handle_world)
    rp = sim.getObjectPosition(right5, sim.handle_world)
    return ((lp[0] - rp[0]) ** 2 +
            (lp[1] - rp[1]) ** 2 +
            (lp[2] - rp[2]) ** 2) ** 0.5


def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))


def wait_arm(a0, a1, tol=1.0, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.1)
    return False


def gripper(target, label, timeout=8.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            break
        time.sleep(0.1)
    return st


def state_line(tag):
    return ("  %-20s s0=%+6.2f  s1=%+6.2f  夹爪Z=%.4f  手指间距=%.4f  Prism=%.5f" %
            (tag, jdeg(s0), jdeg(s1), tip_z(), finger_gap(),
             sim.getJointPosition(pj)))


# ============================================================
# 主流程
# ============================================================
print("\n启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

print("\n" + "=" * 64)
print("协同测试：home -> 降 -> 闭合 -> 升 -> 张开 -> home")
print("=" * 64)

# 1. home + 张开
set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
gripper(ST_OPEN, "open")
print(state_line("① home + OPEN"))

# 2. 下降（夹爪保持张开）
set_deg(s0, LOWER_DEG); set_deg(s1, 0)
ok = wait_arm(LOWER_DEG, 0)
print(state_line("② 下降 s0=%d" % LOWER_DEG) + (" 到位" if ok else " !!超时"))

# 3. 夹爪闭合（夹空气，验证真实闭合）
st = gripper(ST_CLOSE, "close")
print(state_line("③ 夹爪 CLOSE") + (" state=%s" % st))

# 4. 抬升（夹爪保持闭合，模拟搬运）
set_deg(s0, LIFT_DEG); set_deg(s1, 0)
ok = wait_arm(LIFT_DEG, 0)
print(state_line("④ 抬升 s0=%d" % LIFT_DEG) + (" 到位" if ok else " !!超时"))

# 5. 夹爪张开（模拟放下）
st = gripper(ST_OPEN, "open")
print(state_line("⑤ 夹爪 OPEN") + (" state=%s" % st))

# 6. 回 home
set_deg(s0, 0); set_deg(s1, 0)
ok = wait_arm(0, 0)
print(state_line("⑥ 回 home") + (" 到位" if ok else " !!超时"))

print("\n全部步骤结束。若 ②/④ 夹爪Z有明显升降、③ 手指间距明显变小，则协同 OK。")

try:
    sim.stopSimulation()
    print("仿真已停止。")
except Exception:
    pass
