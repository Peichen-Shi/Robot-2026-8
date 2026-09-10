# -*- coding: utf-8 -*-
"""probe_cam.py —— 验证内部相机取图与画面内容（能否看到蓝/红物体、是否有运动）"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math, time

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

old = find("demo_cam")
if old is not None:
    try:
        sim.removeObjects([old])
    except Exception:
        pass

cam = sim.createVisionSensor(6, [1280, 720, 0, 0], [0.1, 10.0, math.radians(60), 0.05, 0, 0, 0, 0, 0, 0, 0])
sim.setObjectAlias(cam, "demo_cam")
sim.setObjectPosition(cam, sim.handle_world, [0.38, 0.0, 4.0])
sim.setObjectOrientation(cam, sim.handle_world, [math.pi, 0.0, 0.0])
print("camera handle:", cam)

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(1.2)

def stat(tag, prev=None):
    img, res = sim.getVisionSensorImg(cam)
    a = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3).astype(np.int16)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    blue = ((b > r + 30) & (b > 100)).mean() * 100
    red = ((r > b + 30) & (r > 100)).mean() * 100
    print("%-16s mean=%.1f std=%.1f  蓝色像素=%.2f%%  红色像素=%.2f%%" %
          (tag, a.mean(), a.std(), blue, red))
    return a

a1 = stat("t=1.2s")
# 动一下机械臂，看画面是否变化（说明机器人可见）
s0 = find("servo_motor_0")
sim.setJointTargetPosition(s0, math.radians(-25))
time.sleep(2.0)
a2 = stat("抬臂后")
print("画面变化(平均绝对差) = %.2f" % np.abs(a1 - a2).mean())

sim.stopSimulation()
try:
    sim.removeObjects([cam])
    print("已删除临时相机")
except Exception as e:
    print("删除相机失败:", e)
print("done")
