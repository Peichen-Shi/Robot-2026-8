# -*- coding: utf-8 -*-
"""diag_arm2.py —— 关节-末端运动学映射: (s0,s1) 组合 -> 夹爪世界 X/Y/Z"""
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

s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")

def deg(h):
    return math.degrees(sim.getJointPosition(h))

def tip_pos():
    return sim.getObjectPosition(tip, sim.handle_world)

def setd(h, d):
    sim.setJointTargetPosition(h, math.radians(d))   # 只用位置环，不硬写

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

setd(s0, 0); setd(s1, 0)
time.sleep(2.5)

def snap(a0, a1):
    setd(s0, a0); setd(s1, a1)
    time.sleep(2.8)
    p = tip_pos()
    print("  (s0=%+6.1f, s1=%+6.1f)  ->  X=%+.4f  Y=%+.4f  Z=%+.4f" %
          (deg(s0), deg(s1), p[0], p[1], p[2]))

print("== 只动 s1（怀疑偏航/微调）==")
snap(0, 0)
snap(0, 30)
snap(0, -30)
print("== 只动 s0（怀疑俯仰/升降）==")
snap(0, 0)
snap(30, 0)
snap(-30, 0)
print("== 联动（同向/反向）==")
snap(20, 20)
snap(0, 0)
snap(-20, -20)
snap(0, 0)
snap(15, -25)
snap(0, 0)
snap(-15, 25)
print("== 回到 home ==")
snap(0, 0)

try:
    sim.stopSimulation()
except Exception:
    pass
print("done")
