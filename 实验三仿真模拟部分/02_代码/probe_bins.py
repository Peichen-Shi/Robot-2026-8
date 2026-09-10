# -*- coding: utf-8 -*-
"""probe_bins.py —— 量分类箱/Grid 布局，给完整流程定坐标"""
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

names = ["RoboMaster"] + ["Grid%d" % i for i in range(1, 7)] + \
        ["LeftBin_%s" % s for s in ["bottom", "front", "back", "left", "right"]] + \
        ["RightBin_%s" % s for s in ["bottom", "front", "back", "left", "right"]]

print("%-16s %-6s %-28s %s" % ("name", "handle", "pos(world)", "static/respondable"))
for n in names:
    h = find(n)
    if h is None:
        print("%-16s MISSING" % n)
        continue
    p = pos(h)
    try:
        st = sim.getObjectInt32Param(h, sim.shapeintparam_static)
        rp = sim.getObjectInt32Param(h, sim.shapeintparam_respondable)
        flags = "static=%d resp=%d" % (st, rp)
    except Exception as e:
        flags = "flags err"
    print("%-16s %-6d (%+.4f, %+.4f, %+.4f)   %s" % (n, h, p[0], p[1], p[2], flags))

# 机器人/夹爪参考
robot = find("RoboMaster")
tip = find("gripper_link_respondable")
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
print("\nRoboMaster base:", ["%+.4f" % v for v in pos(robot)])
print("gripper_link   :", ["%+.4f" % v for v in pos(tip)])
l = pos(left5); r = pos(right5)
print("finger mid     : (%+.4f, %+.4f, %+.4f)" % ((l[0]+r[0])/2, (l[1]+r[1])/2, (l[2]+r[2])/2))
print("finger mid - base 偏移: (%+.4f, %+.4f, %+.4f)" %
      ((l[0]+r[0])/2 - pos(robot)[0], (l[1]+r[1])/2 - pos(robot)[1], (l[2]+r[2])/2 - pos(robot)[2]))
