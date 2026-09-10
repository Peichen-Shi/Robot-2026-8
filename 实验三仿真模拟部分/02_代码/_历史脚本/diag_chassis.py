# -*- coding: utf-8 -*-
"""diag_chassis.py —— 底盘/轮子摸底"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

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

def chain(h, n=3):
    out = []
    c = h
    while c >= 0 and len(out) < n:
        try:
            out.append(sim.getObjectAlias(c, 0))
        except Exception:
            out.append("?")
        c = sim.getObjectParent(c)
    return "/".join(reversed(out))

robot = find("RoboMaster")
wheels = ["front_left_wheel_joint", "front_right_wheel_joint",
          "rear_left_wheel_joint", "rear_right_wheel_joint"]
wh = {w: find(w) for w in wheels}
print("robot h=", robot)
for w in wheels:
    h = wh[w]
    print("\n-- %s h=%d 链:%s --" % (w, h, chain(h)))
    if h is None:
        continue
    for p in ["jointintparam_motor_enabled", "jointintparam_ctrl_enabled",
              "jointintparam_dynctrlmode", "jointintparam_velocity_lock"]:
        try:
            print("  %-30s = %s" % (p, sim.getObjectInt32Param(h, getattr(sim, p))))
        except Exception as e:
            print("  %-30s ERR" % p)
    try:
        print("  getJointMode =", sim.getJointMode(h))
    except Exception:
        pass
    try:
        print("  getJointInterval =", sim.getJointInterval(h))
    except Exception:
        pass
    try:
        sc = sim.getScript(sim.scripttype_childscript, h)
        print("  wheel自带脚本 =", sc)
        if sc and sc >= 0:
            txt = sim.getScriptStringParam(sc, sim.scriptstringparam_text) or ""
            open(r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\_script_wheel.lua", "w", encoding="utf-8").write(txt)
            print("  已导出到 _script_wheel.lua, %d 字符" % len(txt))
    except Exception as e:
        print("  script err", e)

# 机器人整体动态属性
try:
    print("\nrobot isDynamicallyEnabled:", sim.isDynamicallyEnabled(robot))
except Exception as e:
    print("isDyn err", e)

# 各轮子的子关节(inner) 数量
for w in wheels:
    h = wh[w]
    kids = sim.getObjectChildren(h, sim.handle_all, 0) or []
    for k in kids:
        try:
            if sim.getObjectType(k) == sim.object_joint_type:
                print("  %s 内层关节 %s h=%d" % (w, chain(k, 2), k))
        except Exception:
            pass
