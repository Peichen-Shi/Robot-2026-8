# -*- coding: utf-8 -*-
"""diag_drive.py —— 轮子驱动底盘实测（前进 / 旋转）"""
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

robot = find("RoboMaster")
wj = {}
for w in ["front_left_wheel_joint", "front_right_wheel_joint",
          "rear_left_wheel_joint", "rear_right_wheel_joint"]:
    wj[w] = find(w)
print("robot h=%d  轮子: %s" % (robot, {k: v for k, v in wj.items()}))

def base_state():
    p = sim.getObjectPosition(robot, sim.handle_world)
    # yaw 从姿态矩阵提取
    m = sim.getObjectMatrix(robot, sim.handle_world)
    # CoppeliaSim 矩阵列主序 12 元素: 前3列是旋转
    # 近似用 atan2(-m[2], m[0])? 按行: rot[0]=m[0],m[1],m[2] 是 X 轴方向
    yaw = math.atan2(m[1], m[0])
    return p, yaw

def report(tag):
    p, yaw = base_state()
    print("  %-26s base=(%.4f, %.4f, %.4f) yaw=%+.1f°" %
          (tag, p[0], p[1], p[2], math.degrees(yaw)))

def wheels(v_fl, v_fr, v_rl, v_rr):
    sim.setJointTargetVelocity(wj["front_left_wheel_joint"], v_fl)
    sim.setJointTargetVelocity(wj["front_right_wheel_joint"], v_fr)
    sim.setJointTargetVelocity(wj["rear_left_wheel_joint"], v_rl)
    sim.setJointTargetVelocity(wj["rear_right_wheel_joint"], v_rr)

def stopw():
    wheels(0, 0, 0, 0)

def wheel_speed():
    return [sim.getJointVelocity(wj[w]) for w in wj]

print("\n启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)
report("初始")

print("\n== 尝试 A：4轮同向 +3 rad/s（前进？）==")
wheels(3, 3, 3, 3)
for i in range(4):
    time.sleep(0.75)
    print("  +%.1fs  vel=%s" % (0.75*(i+1), ["%+.2f" % v for v in wheel_speed()]))
report("4轮同向3秒后")
stopw()
time.sleep(0.5)
report("停止")

print("\n== 尝试 B：4轮同向 -3（后退？）==")
wheels(-3, -3, -3, -3)
time.sleep(2.0)
report("4轮反向2秒后")
stopw()
time.sleep(0.3)

print("\n== 尝试 C：左+右- 原地转 == ")
wheels(3, -3, 3, -3)
time.sleep(2.0)
report("左+右- 2秒后")
stopw()
time.sleep(0.3)

print("\n== 尝试 D：左-右+ 反向转 ==")
wheels(-3, 3, -3, 3)
time.sleep(2.0)
report("左-右+ 2秒后")
stopw()

report("最终")
try:
    sim.stopSimulation()
except Exception:
    pass
print("done")
