# -*- coding: utf-8 -*-
"""probe2.py —— Grid1 状态 + setObjectPosition 行为"""
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

g = find("Grid1")
print("Grid1 h=", g)

def state(tag):
    p = sim.getObjectPosition(g, sim.handle_world)
    try:
        st = sim.getObjectInt32Param(g, sim.shapeintparam_static)
        rp = sim.getObjectInt32Param(g, sim.shapeintparam_respondable)
    except Exception as e:
        st = rp = "?"
    try:
        par = sim.getObjectParent(g)
        palias = sim.getObjectAlias(par, 0) if par >= 0 else "(none)"
    except Exception:
        palias = "?"
    try:
        dyn = sim.isDynamicallyEnabled(g)
    except Exception as e:
        dyn = "err %s" % e
    print("  %-28s pos=(%.4f, %.4f, %.4f) static=%s resp=%s parent=%s dyn=%s" %
          (tag, p[0], p[1], p[2], st, rp, palias, dyn))

print("== 仿真当前状态:", sim.getSimulationState(), "==")
state("停止态初始")

# 停止态 setpos 测试
sim.setObjectPosition(g, sim.handle_world, [0.20, 0.05, 0.11])
time.sleep(0.3)
state("停止态 setpos (0.20,0.05,0.11)")

# 恢复
sim.setObjectPosition(g, sim.handle_world, [0.38, -0.5, 0.11])

print("\n== 启动仿真测试 setpos ==")
sim.startSimulation()
time.sleep(0.8)
state("运行0.8s")
sim.setObjectPosition(g, sim.handle_world, [0.20, 0.05, 0.11])
time.sleep(0.3)
state("运行中 setpos (0.20,0.05,0.11)")
time.sleep(1.0)
state("setpos后1s")
sim.setObjectPosition(g, sim.handle_world, [0.38, -0.5, 0.11])
time.sleep(0.5)
state("复位 (0.38,-0.5,0.11)")
sim.stopSimulation()
time.sleep(0.5)
state("停止后")
print("done")
