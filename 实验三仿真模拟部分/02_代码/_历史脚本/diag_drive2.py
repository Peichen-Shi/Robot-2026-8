# -*- coding: utf-8 -*-
"""diag_drive2.py —— 底盘控制策略确认
1) 直接传送 base 是否稳定（Lua 兜底方案）
2) 用差速旋转把车摆正(yaw->0)
3) yaw=0 下测：4轮同向±、对角组合 的世界运动方向(前/后/横移映射)
"""
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
names = ["front_left_wheel_joint", "front_right_wheel_joint",
         "rear_left_wheel_joint", "rear_right_wheel_joint"]
wj = {n: find(n) for n in names}
FL, FR, RL, RR = [wj[n] for n in names]

def state():
    p = sim.getObjectPosition(robot, sim.handle_world)
    m = sim.getObjectMatrix(robot, sim.handle_world)
    yaw = math.atan2(m[1], m[0])
    return p, yaw

def report(tag, p0=None, y0=None):
    p, y = state()
    if p0 is not None:
        print("  %-30s pos=(%.4f, %.4f) yaw=%+7.1f°  dPos=(%+.4f,%+.4f) dYaw=%+7.1f°" %
              (tag, p[0], p[1], math.degrees(y),
               p[0]-p0[0], p[1]-p0[1], math.degrees(y-y0)))
    else:
        print("  %-30s pos=(%.4f, %.4f) yaw=%+7.1f°" % (tag, p[0], p[1], math.degrees(y)))

def wheels(a, b, c, d):
    sim.setJointTargetVelocity(FL, a); sim.setJointTargetVelocity(FR, b)
    sim.setJointTargetVelocity(RL, c); sim.setJointTargetVelocity(RR, d)

def stopw():
    wheels(0, 0, 0, 0)

print("启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(0.8)
p0, y0 = state()
report("初始", p0, y0)

# 1) 传送测试
print("\n== 1) 直接 setObjectPosition 传送 ==")
sim.setObjectPosition(robot, sim.handle_world, [p0[0]+0.3, p0[1], p0[2]])
try:
    sim.resetDynamicObject(robot)
except Exception as e:
    print("  resetDynamicObject:", e)
time.sleep(1.2)
p1, y1 = state()
report("传送0.3m后", p0, y0)
# 送回原位
sim.setObjectPosition(robot, sim.handle_world, [p0[0], p0[1], p0[2]])
try:
    sim.resetDynamicObject(robot)
except Exception:
    pass
time.sleep(0.8)
report("传送回原位")

# 2) 差速转正到 yaw≈0
print("\n== 2) 差速旋转把 yaw 转到 0 ==")
max_t = 20.0
t0 = time.time()
while time.time() - t0 < max_t:
    _, y = state()
    e = math.degrees(y)
    if e > 180: e -= 360
    if e < -180: e += 360
    if abs(e) < 1.0:
        break
    s = 2.0 if e > 0 else -2.0
    wheels(s, -s, s, -s)
    time.sleep(0.05)
stopw()
p2, y2 = state()
report("转正后", None, None)

# 3) yaw≈0 映射测试：每组跑 1.5s
def test_move(name, a, b, c, d, sec=1.5):
    pa, ya = state()
    wheels(a, b, c, d)
    time.sleep(sec)
    stopw()
    time.sleep(0.3)
    report(name, pa, ya)

print("\n== 3) yaw≈0 下的方向映射 ==")
test_move("4轮同向+2", 2, 2, 2, 2)
test_move("4轮同向-2", -2, -2, -2, -2)
test_move("FL,RR+2 / FR,RL-2 (对角)", 2, -2, -2, 2)
test_move("FL,RR-2 / FR,RL+2", -2, 2, 2, -2)
test_move("原地转 左+右-", 2, -2, 2, -2)

stopw()
try:
    sim.stopSimulation()
except Exception:
    pass
print("done")
