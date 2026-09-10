# -*- coding: utf-8 -*-
"""
vision_pipeline_recorded.py  (v3)
============================================================
视觉分类 + 夹爪摄像头（左上全程实时 / 右上抓取前识别） 的录制主程序
视频画面布局：
  ┌──────────────────────────────────────────────┐
  │ 标题：实验三 · 演示内容 ｜ 视角                 │
  │ ┌───────┐                         ┌───────┐  │
  │ │夹爪摄像头│  ← 全程实时显示            │夹爪摄像头│  ← 抓取前叠加识别结果
  │ └───────┘                         └───────┘  │     （类别+置信度），抓取成功后关闭
  │   ① 步骤说明（编号）…                物体 3/6  │
  └──────────────────────────────────────────────┘
要点：
  · 所有中文用 PIL + 微软雅黑绘制（不会出现问号/乱码）
  · 抓取：夹爪闭合到“手指刚接触物体”即停（checkCollision 判定）→ 手指不穿过物体
  · 放置：由机械臂前伸把物体送到箱内水平线（不再让物体自己“飘”过去）
用法：python vision_pipeline_recorded.py <front|top|side> <输出mp4>
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import os
import sys
import json

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

import vision_config as VC
import camera_node as CN
import detector_node as DN
import gripper_camera as GC
from sim_logger import SimLogger
from safety_monitor import SafetyMonitor

# ---------------- 运动参数 ----------------
ST_OPEN, ST_CLOSE, ST_PAUSE = 1, 2, 0
APPROACH_S0, APPROACH_S1 = 30.0, 0.0
CARRY_S0, CARRY_S1 = -35.0, 20.0
CROSS_S0, CROSS_S1 = -40.0, 30.0
DOWN1_S0, DOWN1_S1 = -25.0, 50.0
DOWN2_S0, DOWN2_S1 = -20.0, 60.0
SLOT_X, SLOT_DY, SLOT_Z = 0.380, 0.15, 0.13
X_SAFE = 0.015
ROBOT_HALF_LEN, BIN_WALL_OUTER_X = 0.190, 0.275

_HERE_DIR = os.path.dirname(os.path.abspath(__file__))
_DEMO_ROOT = os.path.dirname(_HERE_DIR) if os.path.basename(_HERE_DIR).lower() == "code" else _HERE_DIR
LOGDIR = os.path.join(_DEMO_ROOT, "logs")        # 运行日志统一放在演示文件夹的 logs/ 下
GRASP_FWD_CLEAR = 0.008      # 抓取位再后退 8mm：物体在夹爪中略靠前，腕部不接触物体
FPS = 12.0
CAP_MIN_DT = 1.0 / FPS
W, H = 1280, 720
PIP_W, PIP_H = 380, 214          # 右上：夹爪视角
OV_W, OV_H = 380, 214            # 左上：夹爪摄像头（全程实时）
PIP_REFRESH = 0.40
OV_REFRESH = 1.00

VIEW_LABEL = {"front": "正面视角", "top": "俯视视角", "side": "侧视（侧面）视角"}
VIEW_LABEL_EN = {"front": "Front view", "top": "Top view", "side": "Side view"}
DEMO_NAME = "视觉分类抓取与放置"
DEMO_NAME_EN = "Vision-Guided Classification Pick & Place"

# ---------------- 语言（视频内文字；命令行第 3 个参数给定 zh / en） ----------------
LANG = (sys.argv[3] if len(sys.argv) > 3 else "zh").lower()
ZH = {
    "title": "实验三 · 桌面物体自动分类整理",
    "subtitle": "演示：%s　｜　视角：%s",
    "cam_live": "夹爪摄像头（全程实时）",
    "cam_detect": "夹爪摄像头 · 识别结果（夹取判断）",
    "cls_a": "A类（蓝色棱柱）", "cls_b": "B类（红色圆柱）", "cls_none": "未检出物体",
    "obj_line": "物体：%s", "conf_line": "置信度：%.2f", "grasp_yes": "夹取：是（执行抓取）",
    "grasp_no": "夹取：否（无物体，跳过）",
    "note_cam": "夹爪相机：正前方目标",
    "s_start": "① 启动：仿真与夹爪相机就绪", "s_mode": "② 判断方式：只用夹爪相机逐个位置判断",
    "s_safe": "③ 后退：进入安全横向通道 x=%.3f", "s_align": "④ 对准：平移到 位置%d 前方",
    "s_approach": "⑤ 就位：前移到抓取位（位置%d）",
    "s_check": "⑥ 夹爪相机判断：位置%d 是否有物体",
    "s_skip": "位置%d：未看到物体 → 跳过不抓取，前往下一个位置",
    "s_grasp": "⑦ 抓取：夹爪闭合至接触物体",
    "s_lift": "⑧ 抓取完成：抬升物体",
    "s_back": "⑨ 后退：直线退回安全通道",
    "s_carry": "⑩ 搬运：平移到料盒前方",
    "s_insert": "⑪ 伸入：小车前移，机械臂前伸送料",
    "s_place": "⑫ 放置：把物体放到箱内水平线 x=0.380",
    "s_release": "⑬ 松开：夹爪张开",
    "s_retract": "⑭ 收回：手臂退出箱体",
    "s_done": "⑮ 全部完成：返回起始位置",
    "s_abort": "⑮ 任务中止：检测到异常（详见错误日志），返回起始位置",
    "r_abort": "结果：已放置 %d 个，任务因异常中止（本次计划 %d 个位置）",
    "r_sum": "结果：A类→左料盒　B类→右料盒　已放置 %d 个，跳过 %d 个",
    "prog": "位置 %d/%d · 已放置 %d", "prog_done": "已放置 %d/%d",
}
EN = {
    "title": "Experiment 3 - Desktop Object Auto-Sorting",
    "subtitle": "Demo: %s  |  View: %s",
    "cam_live": "Gripper camera (live)",
    "cam_detect": "Gripper camera - detection & grasp decision",
    "cls_a": "Class A (blue prism)", "cls_b": "Class B (red cylinder)", "cls_none": "no object",
    "obj_line": "Object: %s", "conf_line": "Confidence: %.2f", "grasp_yes": "Grasp: YES (pick it)",
    "grasp_no": "Grasp: NO (no object - skip)",
    "note_cam": "Gripper camera: target ahead",
    "s_start": "1. Start: simulation and gripper camera ready",
    "s_mode": "2. Decision: use the gripper camera only, slot by slot",
    "s_safe": "3. Retreat: go to the safe lateral channel x=%.3f",
    "s_align": "4. Align: translate to slot %d",
    "s_approach": "5. Approach: move forward to the grasp pose (slot %d)",
    "s_check": "6. Gripper camera: is there an object at slot %d ?",
    "s_skip": "Slot %d: no object seen -> SKIP, go to the next slot",
    "s_grasp": "7. Grasp: close until the fingers touch the object",
    "s_lift": "8. Grasped: lift the object",
    "s_back": "9. Retreat: straight back to the safe channel",
    "s_carry": "10. Carry: translate in front of the bin",
    "s_insert": "11. Insert: move forward, extend the arm over the bin",
    "s_place": "12. Place: put the object on the x=0.380 line",
    "s_release": "13. Release: open the gripper",
    "s_retract": "14. Retract: arm leaves the bin",
    "s_done": "15. Done: return to the start pose",
    "s_abort": "15. TASK ABORTED: anomaly detected (see error log), returning to start",
    "r_abort": "Result: placed %d, task aborted by error (%d slots planned)",
    "r_sum": "Result: Class A -> Left bin, Class B -> Right bin | placed %d, skipped %d",
    "prog": "slot %d/%d - placed %d", "prog_done": "placed %d/%d",
}
T = EN if LANG == "en" else ZH
VIEW_TXT = VIEW_LABEL_EN if LANG == "en" else VIEW_LABEL
DEMO_TXT = DEMO_NAME_EN if LANG == "en" else DEMO_NAME

CAM_PRESETS = {
    "front": dict(pos=(2.40, 0.0, 0.95), target=(0.30, 0.0, 0.15), fov=58.0, up=(0, 0, 1)),
    "top":   dict(pos=(0.40, 0.0, 3.50), target=(0.40, 0.0, 0.00), fov=60.0, up=(0, 1, 0)),
    "side":  dict(pos=(-1.75, 0.0, 0.95), target=(0.38, 0.0, 0.12), fov=62.0, up=(0, 0, 1)),
}
VIEW = (sys.argv[1] if len(sys.argv) > 1 else "front").lower()
if VIEW not in CAM_PRESETS:
    VIEW = "front"
CAM = CAM_PRESETS[VIEW]
_VIDEO_DIR = os.path.join(_DEMO_ROOT, "videos")
_out_arg = sys.argv[2] if len(sys.argv) > 2 else "auto"
if _out_arg.strip().lower() in ("", "auto"):
    _out_arg = os.path.join(_VIDEO_DIR, "demo_%s_%s.mp4" % (VIEW, time.strftime("%m%d_%H%M%S")))
OUT = _out_arg
os.makedirs(os.path.dirname(os.path.abspath(OUT)) or ".", exist_ok=True)   # 保证输出目录存在

# ---------------- 中文字体 ----------------
def _font(size, bold=False):
    cands = ([r"C:\Windows\Fonts\msyhbd.ttc"] if bold else []) + \
            [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
             r"C:\Windows\Fonts\simsun.ttc"]
    for p in cands:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()

F_TITLE = _font(22, True)
F_LABEL = _font(19)
F_SMALL = _font(15)
F_TINY = _font(13)

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

def setpos(h, xyz):
    sim.setObjectPosition(h, sim.handle_world, xyz)

robot = find("RoboMaster")
s0, s1 = find("servo_motor_0"), find("servo_motor_1")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5, right5 = find("left_gripper_5_respondable"), find("right_gripper_5_respondable")
if None in (robot, s0, s1, grp, left5, right5):
    raise RuntimeError("对象查找失败")

# ---------------- 通用绘图（PIL，中文正常） ----------------
def pil_from_bgr(frame):
    return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

def bgr_from_pil(img):
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

def paste_bgr(dst_pil, frame_bgr, xy, size, border=(0, 200, 255, 255), title=None):
    """把一路 BGR 画面缩放进 PIL 画布（带边框与标题）"""
    thumb = pil_from_bgr(frame_bgr).resize(size)
    x, y = xy
    d = ImageDraw.Draw(dst_pil)
    d.rectangle([x - 4, y - 4, x + size[0] + 4, y + size[1] + 4], fill=(30, 30, 30, 255),
                outline=border, width=2)
    dst_pil.paste(thumb, (x, y))
    if title:
        d.text((x, y + size[1] + 6), title, font=F_SMALL, fill=border[:3])

def annotate_cn(frame_bgr, dets, box_note=None):
    """在 BGR 图上画检测框 + 类别/置信度标签（PIL 绘制，按语言输出）"""
    img = pil_from_bgr(frame_bgr)
    d = ImageDraw.Draw(img)
    for det in dets:
        x, y, w, h = det["bbox"]
        col = (80, 160, 255) if det["class"] == VC.CLASS_A else (255, 90, 90)
        d.rectangle([x, y, x + w, y + h], outline=col, width=2)
        short = T["cls_a"] if det["class"] == VC.CLASS_A else T["cls_b"]
        txt = "%s  %.0f%%" % (short, det["confidence"] * 100)
        tw = d.textlength(txt, font=F_SMALL)
        ty = max(2, y - 20)
        d.rectangle([x, ty, x + tw + 8, ty + 18], fill=(30, 30, 30))
        d.text((x + 4, ty + 1), txt, font=F_SMALL, fill=col)
    if box_note:
        d.rectangle([0, 0, frame_bgr.shape[1], 26], fill=(30, 30, 30))
        d.text((6, 4), box_note, font=F_TINY, fill=(200, 255, 200))
    return bgr_from_pil(img)

# ---------------- 相机 & 录制 ----------------
def _sub(a, b): return [a[i] - b[i] for i in range(3)]
def _norm(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]
def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def look_at_matrix(px, target, up=(0, 0, 1)):
    z = _norm(_sub(target, px)); x = _norm(_cross(up, z)); y = _cross(z, x)
    return [x[0], y[0], z[0], px[0], x[1], y[1], z[1], px[1], x[2], y[2], z[2], px[2]]

def make_record_camera(sim):
    old = find("demo_cam")
    if old is not None:
        try:
            sim.removeObjects([old])
        except Exception:
            pass
    cam = sim.createVisionSensor(7, [W, H, 0, 0],
                                 [0.05, 30.0, math.radians(CAM["fov"]), 0.05, 0, 0, 0, 0, 0, 0, 0])
    sim.setObjectAlias(cam, "demo_cam")
    sim.setObjectMatrix(cam, sim.handle_world,
                        look_at_matrix(CAM["pos"], CAM["target"], up=CAM["up"]))
    return cam

rec_cam = None
LOG = None             # 轨迹/结果/错误日志
SAFE = None            # 安全监控
CURRENT_TARGET = None   # 当前抓取目标（允许夹爪与其接触）
SCENE_OBJS = []        # 场景中的物体（用于碰撞检查）
gcam = None            # 夹爪摄像头句柄（在抓取姿态下创建）
ov_cam = None          # 俯视识别相机句柄（仅用于类别识别）
writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
if not writer.isOpened():
    raise RuntimeError("VideoWriter 打开失败")

state = {"label": "ready ...", "sub": "", "last": 0.0, "frames": 0,
         "pip_lines": [],
         "pip": None, "pip_text": "", "pip_last": 0.0,
         "gcam": None, "gcam_last": 0.0}

def _draw_frame(frame_bgr):
    img = pil_from_bgr(frame_bgr)
    d = ImageDraw.Draw(img)
    # 标题栏（演示内容 + 视角）
    d.rectangle([0, 0, W, 66], fill=(24, 24, 28))
    d.text((14, 8), T["title"], font=F_TITLE, fill=(255, 255, 255))
    d.text((14, 38), T["subtitle"] % (DEMO_TXT, VIEW_TXT[VIEW]),
           font=F_SMALL, fill=(150, 200, 255))
    d.text((W - 190, 38), state["sub"], font=F_LABEL, fill=(140, 255, 140))
    # 步骤说明
    d.rectangle([0, 66, W, 100], fill=(45, 45, 50))
    d.text((14, 72), state["label"], font=F_LABEL, fill=(90, 220, 255))
    # 左上：夹爪摄像头（全程实时）
    if state["gcam"] is not None:
        paste_bgr(img, state["gcam"], (18, 110), (OV_W, OV_H), title=T["cam_live"])
    # 右上：夹爪摄像头识别结果 + 类别/置信度/是否夹取
    if state["pip"] is not None:
        x0, y0 = W - PIP_W - 18, 110
        paste_bgr(img, state["pip"], (x0, y0), (PIP_W, PIP_H))
        d.rectangle([x0 - 4, y0 - 4, x0 + PIP_W + 4, y0 + PIP_H + 78], outline=(0, 200, 255), width=2)
        d.text((x0, y0 - 22), T["cam_detect"], font=F_SMALL, fill=(0, 200, 255))
        yy = y0 + PIP_H + 6
        for text, col in state["pip_lines"]:
            d.rectangle([x0 - 3, yy - 1, x0 + PIP_W + 3, yy + 19], fill=(30, 30, 30))
            d.text((x0 + 4, yy + 1), text, font=F_SMALL, fill=col)
            yy += 21
    return bgr_from_pil(img)

def capture(force=False):
    now = time.time()
    if not force and (now - state["last"]) < CAP_MIN_DT:
        return
    state["last"] = now
    try:
        sim.handleVisionSensor(rec_cam)
        img, res = sim.getVisionSensorImg(rec_cam)
    except Exception:
        return
    frame = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    frame = cv2.flip(frame, 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if frame.shape[1] != W or frame.shape[0] != H:
        frame = cv2.resize(frame, (W, H))
    writer.write(_draw_frame(frame))
    state["frames"] += 1
    _log_traj()

def label(text, sub=None):
    state["label"] = text
    if sub is not None:
        state["sub"] = sub
    capture(force=True)
    phase_check(text)


_last_traj = [0.0]


def _log_traj():
    """记录机械臂轨迹（约 10Hz）"""
    if LOG is None:
        return
    now = time.time()
    if now - _last_traj[0] < 0.1:
        return
    _last_traj[0] = now
    try:
        b = pos(robot)
        LOG.traj(sim.getSimulationTime(), state["label"][:28],
                 jdeg(s0), jdeg(s1), sim.getJointPosition(pj),
                 sim.getIntProperty(grp, "signal.state"), (b[0], b[1]),
                 tuple(pos(hold.obj)) if hold.obj is not None else None)
    except Exception:
        pass


def phase_check(phase):
    """每个阶段做一次安全检查（关节限位 / 碰撞），记录违规"""
    if SAFE is None or LOG is None:
        return
    try:
        SAFE.set_allow(hold.obj if hold.active else None, CURRENT_TARGET)
        for kind, detail in SAFE.check(SCENE_OBJS, phase):
            LOG.violation(kind, "%s @ %s" % (detail, phase))
            print("    [SAFETY] %s: %s" % (kind, detail), flush=True)
    except Exception:
        pass

def nap(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        capture(); gcam_refresh()
        time.sleep(0.01)

# ---------------- 夹爪摄像头（全程实时 + 抓取时叠加识别结果） ----------------
def gcam_refresh(force=False):
    """抓取夹爪摄像头画面：左上角全程显示；抓取窗口内同时在右上角显示识别结果"""
    if gcam is None:
        return
    now = time.time()
    if not force and (now - state["gcam_last"]) < 0.18:
        return
    state["gcam_last"] = now
    try:
        frame = GC.capture(sim, gcam)
        state["gcam"] = frame
        if state["pip"] is not None:          # 抓取窗口：右上角刷新带识别的画面
            dets = DN.detect_local(frame)
            state["pip"] = annotate_cn(frame, dets, box_note=T["note_cam"])
            state["pip_last"] = now
    except Exception:
        pass

# ---------------- 右上：夹爪第一视角 ----------------
def pip_on():
    frame = GC.capture(sim, gcam)
    dets = DN.detect_local(frame)
    VC.ensure_bus()
    with open(GC.GRI_DETECTIONS, "w", encoding="utf-8") as f:
        json.dump({"stamp": time.time(), "source": "gripper_cam",
                   "count": len(dets), "detections": dets}, f, ensure_ascii=False, indent=2)
    ann = annotate_cn(frame, dets, box_note=T["note_cam"])
    cv2.imwrite(GC.GRI_ANNOTATED, ann)
    state["pip"] = ann
    state["gcam"] = frame
    state["pip_last"] = time.time()
    if dets:
        cx, cy = GC.GRI_CAM_W / 2.0, GC.GRI_CAM_H / 2.0
        prim = min(dets, key=lambda d: (d["bbox_center_px"][0]-cx)**2 + (d["bbox_center_px"][1]-cy)**2)
        short = T["cls_a"] if prim["class"] == VC.CLASS_A else T["cls_b"]
        state["pip_text"] = "%s  conf %.2f" % (short, prim["confidence"])
        # 右上角信息：类别 / 置信度 / 是否夹取
        state["pip_lines"] = [
            (T["obj_line"] % short, (255, 255, 255)),
            (T["conf_line"] % prim["confidence"], (140, 255, 140)),
            (T["grasp_yes"], (140, 255, 140)),
        ]
        print("    [gripper cam] target=%s bbox=%s conf=%.3f -> GRASP" %
              (short, prim["bbox"], prim["confidence"]), flush=True)
    else:
        state["pip_text"] = T["cls_none"]
        state["pip_lines"] = [
            (T["obj_line"] % T["cls_none"], (255, 180, 120)),
            (T["conf_line"] % 0.0, (255, 180, 120)),
            (T["grasp_no"], (255, 140, 120)),
        ]
        print("    [gripper cam] no object -> SKIP", flush=True)
    return dets

def pip_no_object():
    """判断为“无物体”时，右上角也显示一行明确的结论（随后关闭）"""
    state["pip_lines"] = [
        (T["obj_line"] % T["cls_none"], (255, 180, 120)),
        (T["conf_line"] % 0.0, (255, 180, 120)),
        (T["grasp_no"], (255, 140, 120)),
    ]

def pip_refresh():
    if state["pip"] is None or gcam is None:
        return
    if time.time() - state["pip_last"] >= PIP_REFRESH:
        try:
            frame = GC.capture(sim, gcam)
            dets = DN.detect_local(frame)
            state["pip"] = annotate_cn(frame, dets, box_note=T["note_cam"])
            state["pip_last"] = time.time()
        except Exception:
            pass

def pip_off():
    state["pip"] = None
    state["pip_text"] = ""
    state["pip_lines"] = []
    capture(force=True)

# ---------------- 机械臂 / 夹爪 / 小车 ----------------
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))

def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))

def arm_to(a0, a1, tol=1.0, timeout=12.0):
    set_deg(s0, a0); set_deg(s1, a1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        capture(); gcam_refresh()
        if abs(jdeg(s0)-a0) <= tol and abs(jdeg(s1)-a1) <= tol:
            return True
        time.sleep(0.04)
    return False

def gripper(target, timeout=8.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        capture(); gcam_refresh()
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.04)
    return st

def _hits(a, b):
    try:
        r = sim.checkCollision(a, b)
        v = r[0] if isinstance(r, (list, tuple)) else r
        return bool(v)
    except Exception:
        return False

def gripper_close_until_touch(obj, timeout=6.0):
    """闭合夹爪，直到手指刚接触物体就停（避免手指穿过物体造成穿模）"""
    sim.setIntProperty(grp, "signal.target", ST_CLOSE)
    t0 = time.time()
    touched = False
    while time.time() - t0 < timeout:
        capture(); gcam_refresh()
        if _hits(left5, obj) or _hits(right5, obj):
            touched = True
            break
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == ST_CLOSE:
            break
        time.sleep(0.02)
    sim.setIntProperty(grp, "signal.target", ST_PAUSE)     # 停止闭合（保持夹住）
    time.sleep(0.15)
    return touched

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
    def release(self):
        self.active = False; self.obj = None; self.offset = None

hold = Hold()

def drive_leg(target_xy):
    p = pos(robot)
    dx, dy = target_xy[0]-p[0], target_xy[1]-p[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-4:
        return 0.0
    steps = max(6, int(dist*55))
    for i in range(1, steps+1):
        t = i/steps
        setpos(robot, [p[0]+dx*t, p[1]+dy*t, p[2]])
        hold.sync(); capture(); gcam_refresh()
        time.sleep(0.012)
    nap(0.2)
    return dist

def drive_x(x): return drive_leg([x, pos(robot)[1]])
def drive_y(y): return drive_leg([pos(robot)[0], y])

def ensure_visual(obj):
    try:
        sim.setObjectInt32Param(obj, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 0)
    except Exception:
        pass

# ---------------- 主流程 ----------------
print("启动仿真与录制（%s / %s）..." % (DEMO_NAME, VIEW_LABEL[VIEW]), flush=True)
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

rec_cam = make_record_camera(sim)
SCENE_OBJS = [h for h in (find("Grid%d" % i) for i in range(1, VC.GRID_N + 1)) if h is not None]
LOG = SimLogger(LOGDIR, "%s_%s" % (VIEW, LANG))
SAFE = SafetyMonitor(sim, find)
print("日志: %s\n安全监控: 关节限位 + 碰撞检查（桌面/非目标物/自碰撞）" % LOGDIR, flush=True)

label(T["s_start"], T["prog"] % (0, VC.GRID_N, 0))

# ---- 验收演示开关：把 6 号物体放到夹爪前方但超出可达范围（触发“不可达目标”保护） ----
UNREACHABLE_DEMO = "--unreachable" in sys.argv
if UNREACHABLE_DEMO:
    g6 = find("Grid6")
    if g6 is not None:
        setpos(g6, [VC.GRID_X + 0.17, VC.GRID_Y0 + 5 * VC.GRID_DY, 0.11])
        print("\n[验收演示] 已将 6 号物体移到 x=%.3f（夹爪前方 17cm，超出夹取范围）" %
              (VC.GRID_X + 0.17), flush=True)
        print("           预期：夹爪相机会看到物体，但夹爪无法夹到 → 判定不可达 → 中止任务并报错", flush=True)

# ---- 演示开关 ②：快速演示（只处理指定位置，例如 --slots=1 或 --slots=1,2）----
SLOT_FILTER = None
for _a in sys.argv:
    if _a.startswith("--slots="):
        SLOT_FILTER = [int(x) for x in _a.split("=", 1)[1].replace("，", ",").split(",") if x.strip().isdigit()]
        SLOT_FILTER = SLOT_FILTER or None
if SLOT_FILTER:
    print("\n[快速演示] 只处理位置 %s（其余位置本次不处理）" % SLOT_FILTER, flush=True)

# ---- 演示开关 ③：桌上没有物体（所有位置都会判定为“无物体” → 全部跳过）----
if "--noobj" in sys.argv:
    _moved = 0
    for _i in range(1, VC.GRID_N + 1):
        _h = find("Grid%d" % _i)
        if _h is not None:
            _p = pos(_h)
            setpos(_h, [_p[0], _p[1], _p[2] - 2.0])
            _moved += 1
    print("\n[特殊情况] 已把 %d 个物体移出桌面 → 预期：所有位置都判定为“无物体”，全部跳过不抓取" % _moved, flush=True)

# ---- 演示开关 ④：料盒容量（--bincap=1 表示每个料盒只能放 1 个 → 之后触发“料盒已满”报错）----
BIN_CAP = None
for _a in sys.argv:
    if _a.startswith("--bincap="):
        try:
            BIN_CAP = max(1, int(_a.split("=", 1)[1]))
        except Exception:
            BIN_CAP = None
if BIN_CAP:
    print("\n[特殊情况] 演示用：把每个料盒容量限制为 %d 个 → 放满后再放会触发“料盒已满”错误并中止" % BIN_CAP, flush=True)

# ---- 只用夹爪相机判断：每个位置到抓取位后用夹爪相机看有没有物体 ----
label(T["s_mode"], T["prog"] % (0, VC.GRID_N, 0))
nap(1.0)

start = pos(robot)
gripper(ST_OPEN)
arm_to(APPROACH_S0, APPROACH_S1)
off_pick = arm_offset()
gcam = GC.ensure_gripper_camera(sim)          # 抓取姿态下创建夹爪相机
time.sleep(0.4)
gripper(ST_CLOSE); arm_to(CARRY_S0, CARRY_S1); off_carry = arm_offset()
arm_to(DOWN2_S0, DOWN2_S1); off_insert = arm_offset()     # 放置姿态偏移（用于精确落点）
delta_insert = [off_insert[0] - off_carry[0], off_insert[1] - off_carry[1]]   # 位姿差（与夹爪状态无关）
arm_to(0, 0); gripper(ST_OPEN)
print("偏移：抓取(%+.3f,%+.3f) 搬运(%+.3f,%+.3f) 放置(%+.3f,%+.3f) 位姿差(%+.3f,%+.3f)" %
      (off_pick[0], off_pick[1], off_carry[0], off_carry[1],
       off_insert[0], off_insert[1], delta_insert[0], delta_insert[1]), flush=True)

label(T["s_safe"] % X_SAFE, T["prog"] % (0, VC.GRID_N, 0))
drive_x(X_SAFE)

bin_slots = {"LeftBin": [VC.BIN_Y["LeftBin"] - VC.SLOT_DY, VC.BIN_Y["LeftBin"], VC.BIN_Y["LeftBin"] + VC.SLOT_DY],
             "RightBin": [VC.BIN_Y["RightBin"] - VC.SLOT_DY, VC.BIN_Y["RightBin"], VC.BIN_Y["RightBin"] + VC.SLOT_DY]}
bin_used = {"LeftBin": 0, "RightBin": 0}
if BIN_CAP:
    for _k in list(bin_slots):
        bin_slots[_k] = bin_slots[_k][:BIN_CAP]
results = {}

def primary_det(dets):
    """判断“夹爪正前方”是否有物体：
    只有检测框中心接近画面中央（正前方）的检测才被采纳，
    否则可能是相邻位置的物体出现在视野边缘，不能算当前位置有物体。"""
    if not dets:
        return None
    w, h = GC.GRI_CAM_W, GC.GRI_CAM_H
    cands, rejected = [], []
    for d in dets:
        cx, cy = d["bbox_center_px"]
        dx, dy = (cx - w / 2.0) / w, (cy - h / 2.0) / h
        if abs(dx) <= 0.20 and -0.30 <= dy <= 0.45:
            cands.append((abs(dx) + abs(dy) * 0.3, d))
        else:
            rejected.append((d["class"], round(dx, 2), round(dy, 2)))
    if rejected:
        print("      （忽略视野边缘的 %d 个检测，可能是相邻物体：%s）" % (len(rejected), rejected), flush=True)
    if not cands:
        return None
    cands.sort(key=lambda t: t[0])
    return cands[0][1]


def main_loop():
    """只用夹爪相机判断：逐个位置开到抓取位 → 夹爪相机看有没有物体 → 有则按类别抓放，无则跳过"""
    slot_list = [g for g in range(1, VC.GRID_N + 1) if (not SLOT_FILTER or g in SLOT_FILTER)]
    total = len(slot_list)
    done = 0
    aborted = False
    skipped = []
    for g in slot_list:
        sub = T["prog"] % (g, total, done)
        grid = find("Grid%d" % g)
        label(T["s_align"] % g, sub)
        arm_to(0, 0); gripper(ST_OPEN); arm_to(APPROACH_S0, APPROACH_S1)
        gy = VC.GRID_Y0 + (g - 1) * VC.GRID_DY
        drive_y(gy - off_pick[1])
        label(T["s_approach"] % g, sub)
        drive_x(VC.GRID_X - off_pick[0] - GRASP_FWD_CLEAR)

        # ⑥ 只用夹爪相机判断：正前方有没有物体（右上角同步显示识别结果）
        label(T["s_check"] % g, sub)
        dets = pip_on()
        prim = primary_det(dets)
        if prim is None:
            skipped.append(g)
            print("    ⤵ 位置%d：夹爪相机未看到物体 → 跳过，不抓取" % g, flush=True)
            label(T["s_skip"] % g, sub)
            nap(1.6)
            pip_no_object(); nap(0.6)
            state["pip"] = None
            capture(force=True)
            drive_x(X_SAFE)
            continue
        global CURRENT_TARGET
        CURRENT_TARGET = grid
        cls = prim["class"]
        bin_name = VC.CLASS_BIN[cls]
        if bin_used[bin_name] >= len(bin_slots[bin_name]):
            msg = ("cannot place object at slot %d: bin %s is full (%d/%d slots used) "
                   "- task aborted" % (g, bin_name, bin_used[bin_name], len(bin_slots[bin_name])))
            LOG.error(msg, "place")
            label("ERROR: " + msg, sub)
            nap(2.5)
            aborted = True
            break
        slot_y = bin_slots[bin_name][bin_used[bin_name]]
        bin_used[bin_name] += 1
        print("\n=== 位置%d：夹爪相机判定 %s（置信度 %.3f）→ %s ===" %
              (g, prim["class_name"], prim["confidence"], bin_name), flush=True)

        nap(1.2)
        label(T["s_grasp"], sub)
        touched = gripper_close_until_touch(grid)
        if not touched:
            print("    第 1 次闭合未接触物体 → 重试一次", flush=True)
            gripper(ST_OPEN); time.sleep(0.3)
            touched = gripper_close_until_touch(grid)
        if not touched:
            msg = ("target at slot %d is unreachable: gripper never touched an object "
                   "after 2 attempts - task aborted" % g)
            LOG.error(msg, "grasp")
            label("ERROR: unreachable target at slot %d - task aborted" % g, sub)
            nap(3.0)
            aborted = True
            break
        off = hold.grab(grid); hold.sync()
        dev = math.dist(pos(grid), [finger_mid()[i] + off[i] for i in range(3)])
        print("    夹爪闭合：%s；抓取校验偏离 %.4f m %s" %
              ("手指已接触物体" if touched else "到达行程末端", dev, "✅" if dev < 0.05 else "✗"), flush=True)
        nap(0.2)
        pip_off()
        label(T["s_lift"], sub)

        arm_to(CARRY_S0, CARRY_S1); hold.sync()
        label(T["s_back"], sub)
        drive_x(X_SAFE)
        ry_guess = slot_y - off_carry[1] - delta_insert[1] - off[1]
        label(T["s_carry"], sub)
        drive_y(ry_guess)
        # 到位后用“当前实际手指中点”重算精确落点（补偿部分闭合带来的微小差异）
        live = arm_offset()
        rx = SLOT_X - live[0] - delta_insert[0] - off[0]
        ry = slot_y - live[1] - delta_insert[1] - off[1]
        label(T["s_insert"], sub)
        drive_y(ry)
        drive_x(rx)
        _b = pos(robot); _fe = _b[0] + ROBOT_HALF_LEN
        print("    料盒停靠：车体x=%.3f 车头前缘=%.3f 箱壁外面=%.3f 余量=%.3f m %s" %
              (_b[0], _fe, BIN_WALL_OUTER_X, BIN_WALL_OUTER_X-_fe,
               "✅" if _fe < BIN_WALL_OUTER_X else "✗"), flush=True)
        arm_to(CROSS_S0, CROSS_S1); hold.sync()
        arm_to(DOWN1_S0, DOWN1_S1); hold.sync()
        label(T["s_place"], sub)
        arm_to(DOWN2_S0, DOWN2_S1); hold.sync(); nap(0.15)
        p_before = pos(grid)
        print("    机械臂送入后物体位置=(%.3f, %.3f, %.3f)  目标=(%.3f, %.3f, %.3f)" %
              (p_before[0], p_before[1], p_before[2], SLOT_X, slot_y, SLOT_Z), flush=True)
        hold.release(); nap(0.1)
        label(T["s_release"], sub)
        gripper(ST_OPEN); nap(0.25)
        label(T["s_retract"], sub)
        arm_to(CARRY_S0, CARRY_S1); arm_to(0, 0)
        CURRENT_TARGET = None
        results["Grid%d" % g] = (pos(grid), bin_name, cls, slot_y)
        _p = pos(grid)
        _err = [abs(_p[0] - SLOT_X), abs(_p[1] - slot_y), abs(_p[2] - SLOT_Z)]
        LOG.result(slot=g, grid="Grid%d" % g, cls=cls, class_name=prim["class_name"],
                   confidence=round(prim["confidence"], 3), bin=bin_name,
                   target=[SLOT_X, slot_y, SLOT_Z], actual=[round(v, 4) for v in _p],
                   error=[round(v, 4) for v in _err], success=max(_err) < 0.02)
        done += 1

    if aborted:
        label(T["s_abort"], T["prog_done"] % (done, total))
    else:
        label(T["s_done"], T["prog_done"] % (done, total))
    drive_y(start[1]); drive_x(start[0]); nap(0.4)
    if aborted:
        label(T["r_abort"] % (done, total), "%d / %d" % (done, total))
    else:
        label(T["r_sum"] % (done, total - done), "%d / %d" % (done, total))
    nap(2.0)
    return done, skipped, aborted


try:
    done_cnt, skipped_pos, was_aborted = main_loop()
    print("\n跳过的位置：%s（视觉未检测到物体）" % (skipped_pos if skipped_pos else "无"), flush=True)
except Exception as e:
    label("ERROR: %s" % e)
    print("!! 出错:", e, flush=True)
finally:
    try:
        gripper(ST_OPEN)
    except Exception:
        pass
    nap(0.6)
    writer.release()
    try:
        sim.stopSimulation()
    except Exception:
        pass
    print("视频: %s  帧数=%d  大小=%.1fMB" %
          (OUT, state["frames"], os.path.getsize(OUT)/1024/1024), flush=True)
    if LOG is not None:
        try:
            summ = LOG.close({"view": VIEW, "lang": LANG, "video": os.path.basename(OUT)})
            print("\n=== 执行结果汇总 ===", flush=True)
            print("  抓取次数=%d  成功=%d  成功率=%.0f%%  错误=%d  安全违规=%d"
                  % (summ["grasp_attempts"], summ["grasp_success"],
                     summ["success_rate"] * 100, summ["errors"], summ["violations"]), flush=True)
            print("  轨迹: %s" % summ["files"]["trajectory"], flush=True)
            print("  结果: %s" % summ["files"]["results"], flush=True)
            print("  错误: %s" % summ["files"]["errors"], flush=True)
            print("  汇总: %s" % summ["files"]["summary"], flush=True)
        except Exception as e:
            print("日志汇总写入失败:", e, flush=True)

print("\n最终结果：")
for k, (p, b, c, sy) in results.items():
    ex = abs(p[0]-SLOT_X); ey = abs(p[1]-sy); ez = abs(p[2]-SLOT_Z)
    print("  %-6s %s → %-9s 落点=(%+.3f, %+.3f, %+.3f) 误差=(%.4f, %.4f, %.4f) %s" %
          (k, c, b, p[0], p[1], p[2], ex, ey, ez,
           "✅" if max(ex, ey, ez) < 0.02 else "✗"))
