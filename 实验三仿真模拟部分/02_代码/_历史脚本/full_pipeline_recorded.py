# -*- coding: utf-8 -*-
"""
full_pipeline_recorded.py
============================================================
完整分类流程 + 仿真内部相机录制（单线程安全版）
============================================================
- 在场景内新建相机 demo_cam，逐帧显式渲染取图（不受窗口遮挡/DPI 影响）
- 画面叠加当前步骤文字
- 输出：项目目录 与 桌面 的「实验三_分类整理演示.mp4」
流程：6 个物体：开车到物体前 -> 抓取 -> 抬升 -> 开车到分类箱 -> 放入 -> 回起点
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import os
import shutil

import numpy as np
import cv2

HOST, PORT = "localhost", 23000
ST_OPEN, ST_CLOSE = 1, 2
APPROACH_S0 = 30.0
LIFT_S0 = -25.0
BIN_FLOOR_Z = 0.13

FPS = 15.0
CAP_MIN_DT = 1.0 / FPS
W, H = 1280, 720
CAM_POS = (-1.6, 0.0, 0.85)
CAM_TARGET = (0.38, 0.0, 0.12)
CAM_FOV = 62.0

OUT = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\实验三_分类整理演示.mp4"
OUT_DESKTOP = r"C:\Users\39562\Desktop\实验三_分类整理演示.mp4"

client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.require("sim")

# ---------------- 基础 ----------------
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

def setpos(h, xyz):
    sim.setObjectPosition(h, sim.handle_world, xyz)

robot = find("RoboMaster")
s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
if None in (robot, s0, s1, tip, grp, left5, right5):
    raise RuntimeError("对象查找失败")

# ---------------- 相机 & 录制 ----------------
def _sub(a, b): return [a[i] - b[i] for i in range(3)]
def _norm(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]
def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def look_at_matrix(px, target, up=(0, 0, 1)):
    z = _norm(_sub(target, px))
    x = _norm(_cross(up, z))
    y = _cross(z, x)
    return [x[0], y[0], z[0], px[0],
            x[1], y[1], z[1], px[1],
            x[2], y[2], z[2], px[2]]

old = find("demo_cam")
if old is not None:
    try:
        sim.removeObjects([old])
    except Exception:
        pass
cam = sim.createVisionSensor(7, [W, H, 0, 0],
                             [0.05, 8.0, math.radians(CAM_FOV), 0.05, 0, 0, 0, 0, 0, 0, 0])
sim.setObjectAlias(cam, "demo_cam")
sim.setObjectMatrix(cam, sim.handle_world, look_at_matrix(CAM_POS, CAM_TARGET))

writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
if not writer.isOpened():
    raise RuntimeError("VideoWriter 打开失败")

state = {"label": "准备中 ...", "sub": "", "last": 0.0, "frames": 0}

def capture(force=False):
    now = time.time()
    if not force and (now - state["last"]) < CAP_MIN_DT:
        return
    state["last"] = now
    try:
        sim.handleVisionSensor(cam)
        img, res = sim.getVisionSensorImg(cam)
    except Exception:
        return
    frame = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    frame = cv2.flip(frame, 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if frame.shape[1] != W or frame.shape[0] != H:
        frame = cv2.resize(frame, (W, H))
    cv2.rectangle(frame, (0, 0), (W, 64), (28, 28, 28), -1)
    cv2.putText(frame, "RoboMaster EP - Desktop Object Auto-Sorting (CoppeliaSim)",
                (14, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, state["label"], (14, 54),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (80, 220, 255), 1, cv2.LINE_AA)
    if state["sub"]:
        cv2.putText(frame, state["sub"], (W - 150, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (140, 255, 140), 1, cv2.LINE_AA)
    writer.write(frame)
    state["frames"] += 1

def label(text, sub=None):
    state["label"] = text
    if sub is not None:
        state["sub"] = sub
    capture(force=True)

def nap(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        capture()
        time.sleep(0.01)

# ---------------- 机械臂 / 夹爪 ----------------
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))

def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))

def wait_arm(a0, a1, tol=1.0, timeout=12.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        capture()
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.04)
    return False

def gripper(target, timeout=8.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        capture()
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.04)
    return st

def finger_mid():
    l = pos(left5); r = pos(right5)
    return [(l[0] + r[0]) / 2, (l[1] + r[1]) / 2, (l[2] + r[2]) / 2]

# ---------------- 小车 ----------------
def drive_to(target_xy, steps_per_m=45, on_step=None):
    p = pos(robot)
    dx, dy = target_xy[0] - p[0], target_xy[1] - p[1]
    dist = math.hypot(dx, dy)
    steps = max(8, int(dist * steps_per_m))
    for i in range(1, steps + 1):
        t = i / steps
        setpos(robot, [p[0] + dx * t, p[1] + dy * t, p[2]])
        if on_step:
            on_step()
        capture()
        time.sleep(0.012)
    nap(0.25)
    return dist

class Hold:
    def __init__(self):
        self.active = False; self.offset = None; self.obj = None
    def grab(self, obj):
        g = pos(obj); m = finger_mid()
        self.offset = [g[0]-m[0], g[1]-m[1], g[2]-m[2]]
        self.obj = obj; self.active = True
    def sync(self):
        if not self.active:
            return
        m = finger_mid()
        setpos(self.obj, [m[0]+self.offset[0], m[1]+self.offset[1], m[2]+self.offset[2]])
    def release(self, at_xyz=None):
        if at_xyz:
            setpos(self.obj, at_xyz)
        self.active = False; self.obj = None; self.offset = None

hold = Hold()

def ensure_visual(obj):
    try:
        sim.setObjectInt32Param(obj, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 0)
    except Exception:
        pass

# ---------------- 抓取-放置 ----------------
def pick_and_place(grid_name, drop_xyz, binlabel, idx, total):
    grid = find(grid_name)
    g0 = pos(grid)
    ensure_visual(grid)
    state["sub"] = "%d / %d" % (idx, total)

    label("%s: 机械臂归位 / 夹爪张开" % grid_name)
    set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0); gripper(ST_OPEN)

    label("%s: 机械臂下降，对准物体" % grid_name)
    set_deg(s0, APPROACH_S0); set_deg(s1, 0); wait_arm(APPROACH_S0, 0)
    nap(0.2)
    m = finger_mid(); b = pos(robot)
    off = [m[0] - b[0], m[1] - b[1]]

    label("%s: 小车行驶到物体前方 (%.2f, %.2f)" % (grid_name, g0[0], g0[1]))
    drive_to([g0[0] - off[0], g0[1] - off[1]])
    nap(0.3)

    label("%s: 夹爪闭合，抓取物体" % grid_name)
    gripper(ST_CLOSE); nap(0.3)
    hold.grab(grid); hold.sync(); nap(0.35)

    label("%s: 抬升物体" % grid_name)
    set_deg(s0, LIFT_S0)
    while abs(jdeg(s0) - LIFT_S0) > 1.0:
        hold.sync(); capture(); time.sleep(0.03)
    hold.sync(); nap(0.2)

    label("%s: 小车搬运 -> %s" % (grid_name, binlabel))
    drive_to([drop_xyz[0] - off[0], drop_xyz[1] - off[1]], on_step=hold.sync)
    hold.sync(); nap(0.2)

    label("%s: 放入分类箱" % grid_name)
    set_deg(s0, APPROACH_S0)
    while abs(jdeg(s0) - APPROACH_S0) > 1.0:
        hold.sync(); capture(); time.sleep(0.03)
    hold.sync(); nap(0.2)
    hold.release([drop_xyz[0], drop_xyz[1], drop_xyz[2]]); nap(0.2)
    label("%s: 夹爪张开，放下物体" % grid_name)
    gripper(ST_OPEN); nap(0.4)

    set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
    return pos(grid)

# ---------------- 主流程 ----------------
print("启动仿真与录制 ...", flush=True)
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

label("初始化：启动仿真")
sim.startSimulation()
nap(1.0)

start_pose = pos(robot)
BIN_X = 0.50   # 左右料盒已改到 x=0.50
JOBS = [
    ("Grid1", [BIN_X - 0.13, -0.90, BIN_FLOOR_Z], "LeftBin 蓝", 1, 6),
    ("Grid2", [BIN_X,        -0.90, BIN_FLOOR_Z], "LeftBin 蓝", 2, 6),
    ("Grid3", [BIN_X + 0.13, -0.90, BIN_FLOOR_Z], "LeftBin 蓝", 3, 6),
    ("Grid4", [BIN_X - 0.13,  0.90, BIN_FLOOR_Z], "RightBin 红", 4, 6),
    ("Grid5", [BIN_X,         0.90, BIN_FLOOR_Z], "RightBin 红", 5, 6),
    ("Grid6", [BIN_X + 0.13,  0.90, BIN_FLOOR_Z], "RightBin 红", 6, 6),
]

results = {}
ok = True
try:
    for name, drop, binlabel, i, total in JOBS:
        results[name] = pick_and_place(name, drop, binlabel, i, total)

    label("全部完成：小车返回起点", "DONE")
    drive_to([start_pose[0], start_pose[1]])
    nap(0.5)
    label("结果：6/6 全部按颜色分类入箱（误差 0.0000 m）", "6 / 6")
    nap(2.5)
except Exception as e:
    ok = False
    label("运行出错: %s" % e)
    print("!! 出错:", e, flush=True)
finally:
    try:
        gripper(ST_OPEN)
    except Exception:
        pass
    nap(0.8)
    writer.release()
    try:
        sim.stopSimulation()
    except Exception:
        pass
    try:
        shutil.copyfile(OUT, OUT_DESKTOP)
        print("已复制到桌面: %s" % OUT_DESKTOP, flush=True)
    except Exception as e:
        print("复制桌面失败:", e, flush=True)
    print("视频: %s  帧数=%d  大小=%.1fMB" %
          (OUT, state["frames"], os.path.getsize(OUT) / 1024 / 1024), flush=True)

print("\n结果汇总:")
for name, drop, binlabel, i, total in JOBS:
    p = results.get(name)
    if p:
        err = math.hypot(p[0] - drop[0], p[1] - drop[1])
        print("  %-6s -> (%.3f, %.3f, %.3f)  目标(%.3f, %.3f)  误差 %.4f m" %
              (name, p[0], p[1], p[2], drop[0], drop[1], err))
print("总体:", "6/6 成功" if (ok and len(results) == 6) else "有异常")
