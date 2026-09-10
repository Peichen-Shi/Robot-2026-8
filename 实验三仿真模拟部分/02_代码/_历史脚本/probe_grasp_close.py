# -*- coding: utf-8 -*-
"""probe_grasp_close.py —— 测量夹爪闭合到“刚好贴住物体”的位置（避免手指穿过物体）
方法：把物体放到抓取位，逐步闭合夹爪，记录：
   - 两指接触带中心间距 gap
   - 左/右指与物体是否发生碰撞（sim.checkCollision）
同时测出“完全闭合”时的 gap，用于推算指垫厚度。
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

sim = RemoteAPIClient(host="localhost", port=23000).require("sim")

def find(b):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == b:
                return h
        except Exception:
            pass
    return None

robot = find("RoboMaster")
s0, s1 = find("servo_motor_0"), find("servo_motor_1")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
l5, r5 = find("left_gripper_5_respondable"), find("right_gripper_5_respondable")
grid = find("Grid1")

def pos(h): return sim.getObjectPosition(h, sim.handle_world)
def setpos(h, p): sim.setObjectPosition(h, sim.handle_world, p)
def gap():
    a, b = pos(l5), pos(r5)
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(1.0)

# 抓取姿态 + 开到 Grid1 抓取位
sim.setJointTargetPosition(s0, math.radians(30)); sim.setJointTargetPosition(s1, 0)
time.sleep(2.2)
m = [(pos(l5)[i] + pos(r5)[i]) / 2 for i in range(3)]
b = pos(robot); gp = pos(grid)
setpos(robot, [gp[0] - (m[0] - b[0]), gp[1] - (m[1] - b[1]), b[2]])
time.sleep(1.0)
# 物体设为静态不可碰撞？——为了测碰撞，这里临时让物体可碰撞
for h, name in ((grid, "Grid1"), (l5, "left5"), (r5, "right5")):
    try:
        print("%-8s static=%s respondable=%s" % (name,
              sim.getObjectInt32Param(h, sim.shapeintparam_static),
              sim.getObjectInt32Param(h, sim.shapeintparam_respondable)))
    except Exception as e:
        print(name, "param err", e)

print("\n打开夹爪，并让物体位于两指之间（跟随位置）")
sim.setIntProperty(grp, "signal.target", 1)      # OPEN
t0 = time.time()
while time.time() - t0 < 6 and sim.getIntProperty(grp, "signal.state") != 1:
    time.sleep(0.05)
time.sleep(0.3)
m = [(pos(l5)[i] + pos(r5)[i]) / 2 for i in range(3)]
setpos(grid, [m[0], m[1], 0.11])
time.sleep(0.5)
print("张开时 gap = %.4f m，物体中心 = (%.3f, %.3f, %.3f)" % (gap(), *pos(grid)))

print("\n逐步闭合（每步 0.3s），记录 gap 与碰撞：")
sim.setIntProperty(grp, "signal.target", 2)      # CLOSE
t0 = time.time()
last = None
while time.time() - t0 < 10:
    g = gap()
    try:
        cl = sim.checkCollision(l5, grid)
        cr = sim.checkCollision(r5, grid)
        cl = cl[0] if isinstance(cl, (list, tuple)) else cl
        cr = cr[0] if isinstance(cr, (list, tuple)) else cr
    except Exception as e:
        cl = cr = "ERR:%s" % e
    st = sim.getIntProperty(grp, "signal.state")
    print("  t=%.2fs  gap=%.4f  left-hit=%s right-hit=%s  state=%s" % (time.time()-t0, g, cl, cr, st))
    if st == 2:
        break
    time.sleep(0.3)
sim.setIntProperty(grp, "signal.target", 0)      # PAUSE
time.sleep(0.3)
print("\n完全闭合后 gap = %.4f m" % gap())
try:
    print("checkCollision 返回示例:", sim.checkCollision(l5, grid))
except Exception as e:
    print("checkCollision 不可用:", e)
sim.stopSimulation()
print("done")
