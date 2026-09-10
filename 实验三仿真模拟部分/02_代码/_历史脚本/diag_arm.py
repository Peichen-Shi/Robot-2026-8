# -*- coding: utf-8 -*-
"""
diag_arm.py —— 机械臂 servo_motor_0 / servo_motor_1 上下运动诊断
流程:
A. 查找关节 + 打印参数(mode/motor/ctrl/脚本)
B. 仿真停止: setJointPosition 直接写, 读回
C. 仿真运行: 位置插值方式驱动 servo0/servo1, 记录关节角 + 夹爪高度
D. 验证方向: servo1 +0.5(降?) 与 -0.5(升?), servo0 旋转
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

client = RemoteAPIClient(host="localhost", port=23000)
sim = client.require("sim")

def alias_of(h):
    try:
        return sim.getObjectAlias(h, 0)
    except Exception:
        return "?"

def find(bare):
    objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    for h in objs:
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None

s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")     # 夹爪根部(随臂运动)
print("servo_motor_0 =", s0, " servo_motor_1 =", s1, " gripper_link =", tip)

def info(h, tag):
    print("\n-- %s (h=%d) --" % (tag, h))
    for p in ["jointintparam_motor_enabled", "jointintparam_ctrl_enabled",
              "jointintparam_dynctrlmode", "jointintparam_velocity_lock"]:
        try:
            print("  %-30s = %s" % (p, sim.getObjectInt32Param(h, getattr(sim, p))))
        except Exception as e:
            print("  %-30s ERR %s" % (p, e))
    try:
        print("  getJointMode      =", sim.getJointMode(h))
    except Exception as e:
        print("  getJointMode ERR", e)
    try:
        print("  getJointInterval  =", sim.getJointInterval(h))
    except Exception as e:
        print("  interval ERR", e)
    try:
        sc = sim.getScript(sim.scripttype_childscript, h)
        print("  关节自带子脚本    =", sc)
    except Exception as e:
        print("  getScript ERR", e)

info(s0, "servo_motor_0")
info(s1, "servo_motor_1")

def pos_deg(h):
    return math.degrees(sim.getJointPosition(h))

def tip_z():
    p = sim.getObjectPosition(tip, sim.handle_world)
    return p[2]

print("\n== B. 仿真停止: 直接写位置读回 ==")
for h, tag in [(s0, "s0"), (s1, "s1")]:
    for v in [0.3, -0.3, 0.0]:
        try:
            sim.setJointPosition(h, v)
        except Exception as e:
            print(tag, "setJointPosition ERR", e)
        time.sleep(0.25)
        print("  %s 写 %+.3f -> 读 %+.3f deg" % (tag, v, pos_deg(h)))

print("\n== C. 仿真运行: 位置驱动 ==")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

def set_deg(h, deg):
    rad = math.radians(deg)
    try:
        sim.setJointTargetPosition(h, rad)
    except Exception as e:
        print("setJointTargetPosition ERR", e)
    try:
        sim.setJointPosition(h, rad)
    except Exception as e:
        pass

def home():
    set_deg(s0, 0); set_deg(s1, 0)
    time.sleep(2.0)

def go(a0, a1, wait=3.0, label=""):
    set_deg(s0, a0); set_deg(s1, a1)
    time.sleep(wait)
    print("  %-28s s0=%+7.2f deg  s1=%+7.2f deg  夹爪高度=%.4f m" %
          (label, pos_deg(s0), pos_deg(s1), tip_z()))

print("回 home ...")
home()
print("  %-28s s0=%+7.2f deg  s1=%+7.2f deg  夹爪高度=%.4f m" %
      ("home", pos_deg(s0), pos_deg(s1), tip_z()))

go(0, 5, label="s1 = +5 deg (Lua: 下降?)")
go(0, -5, label="s1 = -5 deg (Lua: 抬升?)")
go(0, 0, label="回 home")
go(15, 0, label="s0 = +15 deg (旋转?)")
go(-15, 0, label="s0 = -15 deg")
go(0, 0, label="回 home")

try:
    sim.stopSimulation()
except Exception:
    pass
print("\ndone")
