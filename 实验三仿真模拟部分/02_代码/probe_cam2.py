# -*- coding: utf-8 -*-
"""probe_cam2.py —— 找一个"物体够大且能看全一排 Grid + 两个分类箱"的机位"""
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

def sub(a, b):
    return [a[i] - b[i] for i in range(3)]

def norm(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]

def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def look_at_matrix(px, target, up=(0, 0, 1)):
    """CoppeliaSim 12 元素行主序矩阵: [Xx,Yx,Zx,px, Xy,Yy,Zy,py, Xz,Yz,Zz,pz]"""
    z = norm(sub(target, px))            # 相机 +Z 指向目标
    x = norm(cross(up, z))
    y = cross(z, x)
    return [x[0], y[0], z[0], px[0],
            x[1], y[1], z[1], px[1],
            x[2], y[2], z[2], px[2]]

def kill_cam():
    c = find("demo_cam")
    if c is not None:
        try:
            sim.removeObjects([c])
        except Exception:
            pass

def make_cam(px, target, fov_deg=58, res=(1280, 720)):
    kill_cam()
    cam = sim.createVisionSensor(7, [res[0], res[1], 0, 0],
                                 [0.05, 8.0, math.radians(fov_deg), 0.05, 0, 0, 0, 0, 0, 0, 0])
    sim.setObjectAlias(cam, "demo_cam")
    sim.setObjectMatrix(cam, sim.handle_world, look_at_matrix(px, target))
    return cam

def grab(cam):
    sim.handleVisionSensor(cam)
    img, res = sim.getVisionSensorImg(cam)
    return np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3).astype(np.int16)

def stats(a, tag):
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    blue = ((b > r + 30) & (b > 100)).mean() * 100
    red = ((r > b + 30) & (r > 100)).mean() * 100
    # 画面四周各 8% 区域是否被物体占据（判断是否被裁切）
    h, w = a.shape[:2]
    def edge_energy(sl):
        seg = a[sl]
        return float(seg.std())
    print("%-22s mean=%6.1f std=%5.1f 蓝=%5.2f%% 红=%5.2f%% | 边带std L%.0f R%.0f T%.0f B%.0f" %
          (tag, a.mean(), a.std(), blue, red,
           edge_energy((slice(None), slice(0, w // 12))),
           edge_energy((slice(None), slice(w - w // 12, None))),
           edge_energy((slice(0, h // 12), slice(None))),
           edge_energy((slice(h - h // 12, None), slice(None)))))
    return a

robot = find("RoboMaster")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

CANDIDATES = [
    ("A 侧视(-1.6,0,0.85)", (-1.6, 0.0, 0.85), (0.38, 0.0, 0.12)),
    ("B 侧视高一点",        (-1.2, 0.0, 1.25), (0.38, 0.0, 0.12)),
    ("C 顶视 z=4",          (0.38, 0.0, 4.0),  (0.38, 0.0, 0.0)),
]

sim.startSimulation()
time.sleep(1.0)

for name, px, tg in CANDIDATES:
    cam = make_cam(px, tg)
    time.sleep(0.4)
    a1 = grab(cam)
    stats(a1, name)
    p0 = sim.getObjectPosition(robot, sim.handle_world)
    sim.setObjectPosition(robot, sim.handle_world, [p0[0], p0[1] + 0.4, p0[2]])
    time.sleep(0.5)
    a2 = grab(cam)
    print("   机器人移动0.4m 画面差 = %.2f" % np.abs(a1 - a2).mean())
    sim.setObjectPosition(robot, sim.handle_world, p0)
    time.sleep(0.3)

sim.stopSimulation()
kill_cam()
print("done")
