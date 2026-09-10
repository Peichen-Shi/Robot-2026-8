# -*- coding: utf-8 -*-
"""probe_grid1.py —— 测 Grid1/机器人/夹爪几何与物理属性"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

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

def bbox(h):
    return sim.getObjectBoundingBox(h, sim.handle_world)

def show(tag, h):
    p = pos(h)
    print("%-16s h=%d  pos=(%.4f, %.4f, %.4f)" % (tag, h, p[0], p[1], p[2]))
    try:
        b = bbox(h)
        print("  bbox min=(%.4f,%.4f,%.4f) max=(%.4f,%.4f,%.4f)" %
              (b[0][0], b[0][1], b[0][2], b[1][0], b[1][1], b[1][2]))
    except Exception as e:
        print("  bbox err", e)

robot = find("RoboMaster")
grid1 = find("Grid1")
tip = find("gripper_link_respondable")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
tipvL = find("left_gripper_7_visual")
tipvR = find("right_gripper_7_visual")
attach = find("attachPoint")

show("RoboMaster", robot)
show("Grid1", grid1)
show("gripper_link(tip)", tip)
show("left5", left5)
show("right5", right5)
show("attachPoint", attach)

for name, h in [("Grid1", grid1), ("left5", left5), ("right5", right5)]:
    try:
        st = sim.getObjectInt32Param(h, sim.shapeintparam_static)
        rp = sim.getObjectInt32Param(h, sim.shapeintparam_respondable)
        print("%-8s static=%s respondable=%s" % (name, st, rp))
    except Exception as e:
        print(name, "param err", e)

# 夹爪张开/闭合时两个 tip visual 的位置
def gap():
    a = pos(tipvL); b = pos(tipvR)
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2) ** 0.5
print("手指视觉末端间距 = %.4f" % gap())
