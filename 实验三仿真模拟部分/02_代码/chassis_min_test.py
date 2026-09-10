# -*- coding: utf-8 -*-
"""
chassis_min_test.py  (v2 混合驱动版)
============================================================
RoboMaster EP 小车移动最小测试
============================================================
实测结论：
- 平移：小步 setObjectPosition + resetDynamicObject = 精确、无漂移
  （传送会保持朝向，位置分毫不差）
- 转向：setObjectOrientation 传送会把车“蹭”跑；但车轮差速转向很纯
  （实测转 170° 位移 < 0.5cm）。因此转向用 车轮闭环 P 控制。
- 轮速->角速度经验关系：dθ/dt ≈ -33°/s 每单位轮速(rad/s)，故
  轮速 = -Kp * 误差角(°) / 33，限幅 ±3 rad/s。

流程：旋转到目标朝向 -> 平移(传送) -> 再转 -> 再平移 -> 回原点
============================================================
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
w = {n: find(n) for n in ["front_left_wheel_joint", "front_right_wheel_joint",
                          "rear_left_wheel_joint", "rear_right_wheel_joint"]}
FL, FR, RL, RR = w["front_left_wheel_joint"], w["front_right_wheel_joint"], \
                 w["rear_left_wheel_joint"], w["rear_right_wheel_joint"]

def yaw_deg():
    m = sim.getObjectMatrix(robot, sim.handle_world)
    return math.degrees(math.atan2(m[1], m[0]))

def base():
    p = sim.getObjectPosition(robot, sim.handle_world)
    return p, yaw_deg()

def show(tag):
    p, y = base()
    print("  %-26s pos=(%.4f, %.4f)  yaw=%+7.1f°" %
          (tag, p[0], p[1], y))

def wheels_speed(a, b, c, d):
    sim.setJointTargetVelocity(FL, a)
    sim.setJointTargetVelocity(FR, b)
    sim.setJointTargetVelocity(RL, c)
    sim.setJointTargetVelocity(RR, d)

def rotate_to(target_deg, tol=1.5, max_t=15.0):
    """车轮差速 开关式闭环转到目标 yaw（实测转向非线性，避免线性增益）
    实测：轮速 s>0 (FL,RL=+s, FR,RR=-s) -> yaw 减小；s<0 -> yaw 增大。
    """
    t0 = time.time()
    last = None
    while time.time() - t0 < max_t:
        _, y = base()
        e = (target_deg - y + 180) % 360 - 180   # [-180,180)
        if abs(e) < tol:
            break
        s = -2.0 if e > 0 else 2.0        # e>0 需要 yaw 增大 -> s<0
        if abs(e) < 8.0:
            s = -0.6 if e > 0 else 0.6    # 接近时减速
        wheels_speed(s, -s, s, -s)
        time.sleep(0.05)
        last = e
    wheels_speed(0, 0, 0, 0)
    if last is not None and abs(last) >= tol:
        print("   !! 转向可能未完全到位 (e=%+.1f°)" % last)

def move_xy(dx, dy, steps=40):
    """小步传送平移（精确，保持朝向）"""
    p, _ = base()
    x0, y0, z0 = p[0], p[1], p[2]
    for i in range(1, steps + 1):
        t = i / steps
        sim.setObjectPosition(robot, sim.handle_world,
                              [x0 + dx * t, y0 + dy * t, z0])
        try:
            sim.resetDynamicObject(robot)
        except Exception:
            pass
        time.sleep(0.015)
    wheels_speed(0, 0, 0, 0)
    time.sleep(1.0)   # 停稳再转/继续

print("启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(0.8)

p0, y0 = base()
show("初始")

print("\n== 1) 车轮转向到 yaw=90° ==")
rotate_to(90)
show("转到90°")

print("\n== 2) 平移前进 1.0 m ==")
move_xy(1.0, 0)
show("前进1m")

print("\n== 3) 转向到 yaw=180° ==")
rotate_to(180)
show("转到180°")

print("\n== 4) 平移 (0, +0.6) ==")
move_xy(0, 0.6)
show("平移后")

print("\n== 5) 转回初始朝向并回初始位置 ==")
rotate_to(y0)
move_xy(p0[0] - base()[0][0], p0[1] - base()[0][1])
show("回到起点")

print("\n完成。")
try:
    sim.stopSimulation()
    print("仿真已停止。")
except Exception:
    pass
