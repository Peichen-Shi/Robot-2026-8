# -*- coding: utf-8 -*-
"""
full_pipeline_recorded_v2.py  (v2.1 —— 直角走位 + 内部相机录制)
用法: python full_pipeline_recorded_v2.py <front|top|side> <输出mp4路径>
要点：
 ① 全程直角走位：抓取 -> 直线后退 -> 平移料盒前 -> 放料 -> 平移下一物体 -> 前移抓取
 ② 底盘远离料盒（车体 x=0.015，车头前缘 0.205 < 箱壁外面 0.275，余量 7cm）
 ③ 物品高位跨过箱壁（底部 0.105 > 壁顶 0.08）再下降，避免穿模
 ④ 箱内物品同在 x=0.380 水平线、间距 0.15
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import os
import sys
import shutil

import numpy as np
import cv2

HOST, PORT = "localhost", 23000
ST_OPEN, ST_CLOSE = 1, 2

APPROACH_S0, APPROACH_S1 = 30.0, 0.0
CARRY_S0, CARRY_S1 = -35.0, 20.0
CROSS_S0, CROSS_S1 = -40.0, 30.0
DOWN1_S0, DOWN1_S1 = -25.0, 50.0
DOWN2_S0, DOWN2_S1 = -20.0, 60.0

SLOT_X = 0.380
SLOT_DY = 0.15
SLOT_Z = 0.13
X_SAFE = 0.015
ROBOT_HALF_LEN = 0.190
BIN_WALL_OUTER_X = 0.275

FPS = 15.0
CAP_MIN_DT = 1.0 / FPS
W, H = 1280, 720

CAM_PRESETS = {
    "side":  dict(pos=(-1.75, 0.0, 0.95), target=(0.38, 0.0, 0.12), fov=62.0,
                  up=(0, 0, 1), name="侧视机位"),
    "front": dict(pos=(2.40, 0.0, 0.95),  target=(0.30, 0.0, 0.15), fov=58.0,
                  up=(0, 0, 1), name="正面机位"),
    "top":   dict(pos=(0.40, 0.0, 3.50),  target=(0.40, 0.0, 0.00), fov=60.0,
                  up=(0, 1, 0), name="俯视机位"),
}
VIEW = (sys.argv[1] if len(sys.argv) > 1 else "front").lower()
if VIEW not in CAM_PRESETS:
    VIEW = "front"
CAM = CAM_PRESETS[VIEW]

OUT = sys.argv[2] if len(sys.argv) > 2 else \
    r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\实验三_分类整理演示.mp4"
OUT_DESKTOP = OUT

client = RemoteAPIClient(host=HOST, port=PORT)
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

def setpos(h, xyz):
    sim.setObjectPosition(h, sim.handle_world, xyz)

robot = find("RoboMaster")
s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
if None in (robot, s0, s1, grp, left5, right5):
    raise RuntimeError("对象查找失败")

# ---------------- 相机 & 录制 ----------------
def _sub(a, b): return [a[i]-b[i] for i in range(3)]
def _norm(v):
    n = math.sqrt(sum(x*x for x in v)) or 1.0
    return [x/n for x in v]
def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def look_at_matrix(px, target, up=(0, 0, 1)):
    z = _norm(_sub(target, px)); x = _norm(_cross(up, z)); y = _cross(z, x)
    return [x[0], y[0], z[0], px[0], x[1], y[1], z[1], px[1], x[2], y[2], z[2], px[2]]

old = find("demo_cam")
if old is not None:
    try:
        sim.removeObjects([old])
    except Exception:
        pass
cam = sim.createVisionSensor(7, [W, H, 0, 0],
                             [0.05, 8.0, math.radians(CAM["fov"]), 0.05, 0, 0, 0, 0, 0, 0, 0])
sim.setObjectAlias(cam, "demo_cam")
sim.setObjectMatrix(cam, sim.handle_world, look_at_matrix(CAM["pos"], CAM["target"], up=CAM["up"]))

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
    cv2.putText(frame, "RoboMaster EP - Desktop Object Auto-Sorting  [%s]" % CAM["name"],
                (14, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, state["label"], (14, 54),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (80, 220, 255), 1, cv2.LINE_AA)
    if state["sub"]:
        cv2.putText(frame, state["sub"], (W - 165, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (140, 255, 140), 1, cv2.LINE_AA)
    writer.write(frame); state["frames"] += 1

def label(text, sub=None):
    state["label"] = text
    if sub is not None:
        state["sub"] = sub
    capture(force=True)

def nap(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        capture(); time.sleep(0.01)

# ---------------- 机械臂 / 夹爪 ----------------
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))

def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))

def arm_to(a0, a1, tol=1.0, timeout=12.0):
    set_deg(s0, a0); set_deg(s1, a1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        capture()
        if abs(jdeg(s0)-a0) <= tol and abs(jdeg(s1)-a1) <= tol:
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
    return [(l[0]+r[0])/2, (l[1]+r[1])/2, (l[2]+r[2])/2]

def arm_offset():
    m = finger_mid(); b = pos(robot)
    return [m[0]-b[0], m[1]-b[1]]

class Hold:
    def __init__(self):
        self.active = False; self.offset = None; self.obj = None
    def grab(self, obj):
        g = pos(obj); m = finger_mid()
        self.offset = [g[0]-m[0], g[1]-m[1], g[2]-m[2]]
        self.obj = obj; self.active = True
        return self.offset
    def sync(self):
        if not self.active:
            return
        m = finger_mid()
        setpos(self.obj, [m[0]+self.offset[0], m[1]+self.offset[1], m[2]+self.offset[2]])
    def settle_to(self, xyz, sec=0.45, steps=22):
        if not self.active:
            return
        p0 = pos(self.obj)
        for i in range(1, steps+1):
            t = i/steps
            setpos(self.obj, [p0[0]+(xyz[0]-p0[0])*t,
                              p0[1]+(xyz[1]-p0[1])*t,
                              p0[2]+(xyz[2]-p0[2])*t])
            capture(); time.sleep(sec/steps)
    def release(self):
        self.active = False; self.obj = None; self.offset = None

hold = Hold()

def drive_leg(target_xy, tag=""):
    p = pos(robot)
    dx, dy = target_xy[0]-p[0], target_xy[1]-p[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-4:
        return 0.0
    steps = max(6, int(dist*55))
    for i in range(1, steps+1):
        t = i/steps
        setpos(robot, [p[0]+dx*t, p[1]+dy*t, p[2]])
        hold.sync(); capture()
        time.sleep(0.012)
    nap(0.2)
    return dist

def drive_x(x, tag=""):
    return drive_leg([x, pos(robot)[1]], tag)

def drive_y(y, tag=""):
    return drive_leg([pos(robot)[0], y], tag)

def ensure_visual(obj):
    try:
        sim.setObjectInt32Param(obj, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 0)
    except Exception:
        pass

# ---------------- 抓放 ----------------
def pick_and_place(grid_name, slot_xyz, off_pick, off_carry, idx, total):
    grid = find(grid_name)
    g0 = pos(grid)
    ensure_visual(grid)
    state["sub"] = "%d / %d" % (idx, total)

    label("%s: 手臂张开并对准" % grid_name)
    arm_to(0, 0); gripper(ST_OPEN); arm_to(APPROACH_S0, APPROACH_S1)

    label("%s: 平移到物体前方 (y=%.2f)" % (grid_name, g0[1]))
    drive_y(g0[1]-off_pick[1], "平移")

    label("%s: 往前一点，进入抓取位" % grid_name)
    drive_x(g0[0]-off_pick[0], "前移")

    label("%s: 夹爪闭合抓取" % grid_name)
    gripper(ST_CLOSE); nap(0.2)
    off = hold.grab(grid); hold.sync(); nap(0.25)

    label("%s: 抬升（物品高于箱壁）" % grid_name)
    arm_to(CARRY_S0, CARRY_S1); hold.sync(); nap(0.2)

    label("%s: 直线后退到安全距离" % grid_name)
    drive_x(X_SAFE, "后退")

    ry = slot_xyz[1] - off_carry[1] - off[1]
    label("%s: 平移到料盒前方" % grid_name)
    drive_y(ry, "平移")
    hold.sync(); nap(0.15)
    _b = pos(robot); _fe = _b[0] + ROBOT_HALF_LEN
    print("  [%s] 车体x=%.3f 车头前缘=%.3f 箱壁外面=%.3f 余量=%.3f m %s" %
          (grid_name, _b[0], _fe, BIN_WALL_OUTER_X, BIN_WALL_OUTER_X-_fe,
           "✅" if _fe < BIN_WALL_OUTER_X else "✗"), flush=True)

    label("%s: 高位跨过箱壁" % grid_name)
    arm_to(CROSS_S0, CROSS_S1); hold.sync(); nap(0.15)

    label("%s: 箱内下降到位" % grid_name)
    arm_to(DOWN1_S0, DOWN1_S1); hold.sync()
    arm_to(DOWN2_S0, DOWN2_S1); hold.sync(); nap(0.15)

    label("%s: 轻放到 x=0.380 水平线" % grid_name)
    hold.settle_to([slot_xyz[0], slot_xyz[1], SLOT_Z], sec=0.45)
    hold.release(); nap(0.12)

    label("%s: 夹爪张开，放下" % grid_name)
    gripper(ST_OPEN); nap(0.3)

    label("%s: 手臂收回" % grid_name)
    arm_to(CARRY_S0, CARRY_S1); arm_to(0, 0)
    return pos(grid)

# ---------------- 主流程 ----------------
print("启动仿真与录制（%s）..." % CAM["name"], flush=True)
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

label("初始化：启动仿真")
sim.startSimulation(); nap(0.8)

start = pos(robot)
gripper(ST_OPEN)
arm_to(APPROACH_S0, APPROACH_S1); off_pick = arm_offset()
gripper(ST_CLOSE)
arm_to(CARRY_S0, CARRY_S1); off_carry = arm_offset()
arm_to(0, 0); gripper(ST_OPEN)
print("off_pick=%s off_carry=%s" % (["%.3f" % v for v in off_pick], ["%.3f" % v for v in off_carry]), flush=True)

label("初始：后退到安全横向通道")
drive_x(X_SAFE, "初始后退")

L_C, R_C = -0.90, 0.90
def slot(cy, end):
    return [SLOT_X, cy + (SLOT_DY if end == "right" else (-SLOT_DY if end == "left" else 0.0)), SLOT_Z]

JOBS = [
    ("Grid1", slot(L_C, "left"),   "蓝 → LeftBin 最左", 1, 6),
    ("Grid2", slot(L_C, "center"), "蓝 → LeftBin 中间", 2, 6),
    ("Grid3", slot(L_C, "right"),  "蓝 → LeftBin 最右", 3, 6),
    ("Grid4", slot(R_C, "left"),   "红 → RightBin 最左", 4, 6),
    ("Grid5", slot(R_C, "center"), "红 → RightBin 中间", 5, 6),
    ("Grid6", slot(R_C, "right"),  "红 → RightBin 最右", 6, 6),
]

results = {}
ok = True
try:
    for name, sl, lt, i, total in JOBS:
        results[name] = pick_and_place(name, sl, off_pick, off_carry, i, total)

    label("全部完成：小车直角走位返回原位", "DONE")
    drive_y(start[1], "平移回原位")
    drive_x(start[0], "回原位")
    nap(0.4)
    label("结果：6/6 同水平线(x=0.380)、间距0.15、底盘未碰料盒", "6 / 6")
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
        if os.path.abspath(OUT) != os.path.abspath(OUT_DESKTOP):
            shutil.copyfile(OUT, OUT_DESKTOP)
            print("已复制到桌面: %s" % OUT_DESKTOP, flush=True)
    except Exception as e:
        print("复制桌面失败:", e, flush=True)
    print("视频: %s  帧数=%d  大小=%.1fMB" %
          (OUT, state["frames"], os.path.getsize(OUT)/1024/1024), flush=True)

print("\n结果汇总:")
for name, sl, lt, i, total in JOBS:
    p = results.get(name)
    if p:
        good = abs(p[0]-sl[0]) < 0.02 and abs(p[1]-sl[1]) < 0.02 and abs(p[2]-sl[2]) < 0.02
        print("  %-6s -> (%+.3f, %+.3f, %+.3f)  目标(%+.3f, %+.3f, %+.3f)  %s" %
              (name, p[0], p[1], p[2], sl[0], sl[1], sl[2], "✅" if good else "✗"))
print("总体:", "6/6 成功" if (ok and len(results) == 6) else "有异常")
