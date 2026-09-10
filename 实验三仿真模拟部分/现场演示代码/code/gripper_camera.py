# -*- coding: utf-8 -*-
"""
gripper_camera.py —— 机械臂夹爪第一视角相机（eye-in-hand）
- 相机挂在夹爪手腕（gripper_link_respondable）上，随手臂运动，朝夹爪前方看
- 采集图像并发布到消息总线：gripper_latest.jpg / gripper_camera_meta.json
- 识别节点用 detect_local() 对这张图做近距识别（类别/检测框/置信度）
"""
import math
import json
import time
import os

import numpy as np
import cv2

import vision_config as C

GRI_CAM_NAME = "gripper_cam"
GRI_LATEST = os.path.join(C.BUS_DIR, "gripper_latest.jpg")
GRI_META = os.path.join(C.BUS_DIR, "gripper_camera_meta.json")
GRI_DETECTIONS = os.path.join(C.BUS_DIR, "gripper_detections.json")
GRI_ANNOTATED = os.path.join(C.BUS_DIR, "gripper_annotated.jpg")
GRI_CAM_W, GRI_CAM_H = 320, 180
GRI_FOV = 75.0                                   # 稍广视角，便于看清前方物体
GRI_BACK_OFFSET = 0.075                          # 相机相对“手指带中点”向后偏移（机械臂在相机身后，不入镜）
GRI_UP_OFFSET = 0.085                            # 向上偏移（位于夹爪上方的中心位置）
GRI_AIM_AHEAD = 0.120                            # 视线在手指带前方 12cm 处对准物体
GRI_AIM_OBJ_Z = 0.11                             # 物体中心高度（抓取时物体中心 z）


def _find(sim, bare):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None


def _norm(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]


def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _look_at_matrix(px, target, up):
    z = _norm([target[i] - px[i] for i in range(3)])
    x = _norm(_cross(up, z))
    y = _cross(z, x)
    return [x[0], y[0], z[0], px[0], x[1], y[1], z[1], px[1], x[2], y[2], z[2], px[2]]


def ensure_gripper_camera(sim):
    """创建/重建夹爪相机：装在【夹爪前端中心、朝前看】，并挂到夹爪手腕上随臂运动。
    注意：请在机械臂处于“抓取姿态”时调用，这样相机的相对朝向才是抓取时的正前方。"""
    old = _find(sim, GRI_CAM_NAME)
    if old is not None:
        try:
            sim.removeObjects([old])
        except Exception:
            pass
    cam = sim.createVisionSensor(7, [GRI_CAM_W, GRI_CAM_H, 0, 0],
                                 [0.02, 4.0, math.radians(GRI_FOV), 0.03, 0, 0, 0, 0, 0, 0, 0])
    sim.setObjectAlias(cam, GRI_CAM_NAME)

    wrist = _find(sim, "gripper_link_respondable")
    l5 = _find(sim, "left_gripper_5_respondable")
    r5 = _find(sim, "right_gripper_5_respondable")
    if l5 is not None and r5 is not None:
        pl = sim.getObjectPosition(l5, sim.handle_world)
        pr = sim.getObjectPosition(r5, sim.handle_world)
        anchor = [(pl[i] + pr[i]) / 2.0 for i in range(3)]      # 手指接触带中点
    else:
        anchor = sim.getObjectPosition(wrist, sim.handle_world)
    # 相机位于手指带中点【后方偏上】→ 机械臂在相机身后，不入镜；视线朝前下方看向物体中心
    cam_pos = [anchor[0] - GRI_BACK_OFFSET, anchor[1], anchor[2] + GRI_UP_OFFSET]
    aim = [anchor[0] + GRI_AIM_AHEAD, anchor[1], GRI_AIM_OBJ_Z]
    sim.setObjectMatrix(cam, sim.handle_world, _look_at_matrix(cam_pos, aim, [0.0, 0.0, 1.0]))
    if wrist is not None:
        sim.setObjectParent(cam, wrist, True)                  # 随手臂运动
    return cam


def capture(sim, cam):
    sim.handleVisionSensor(cam)
    img, res = sim.getVisionSensorImg(cam)
    frame = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    frame = cv2.flip(frame, 0)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def publish(sim, cam, frame=None):
    C.ensure_bus()
    if frame is None:
        frame = capture(sim, cam)
    cv2.imwrite(GRI_LATEST, frame)
    meta = {
        "stamp": time.time(),
        "resolution": [GRI_CAM_W, GRI_CAM_H],
        "fov_deg": GRI_FOV,
        "camera_matrix": sim.getObjectMatrix(cam, sim.handle_world),
        "camera_pos": sim.getObjectPosition(cam, sim.handle_world),
        "note": "夹爪第一视角（eye-in-hand），仅做近距分类识别，不做世界映射",
    }
    with open(GRI_META, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return frame, meta
