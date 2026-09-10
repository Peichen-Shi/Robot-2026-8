# -*- coding: utf-8 -*-
"""
camera_node.py —— 仿真相机节点（发布图像）
职责：在 CoppeliaSim 中创建/使用俯视相机，抓取一帧图像并发布到消息总线：
        vision_bus/latest.jpg          最新图像
        vision_bus/frames/frame_XXXX.jpg 历史帧
        vision_bus/camera_meta.json    相机标定信息（位姿矩阵、FOV、分辨率）
可独立运行（调试用）：python camera_node.py <帧数> <间隔秒>
"""
import time
import math
import json
import os
import sys

import numpy as np
import cv2

import vision_config as C


# ---------------- 相机管理 ----------------
def _vec(a, b): return [a[i] - b[i] for i in range(3)]
def _norm(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]
def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def look_at_matrix(px, target, up):
    """CoppeliaSim 12 元素矩阵：[Xx,Yx,Zx,px, Xy,Yy,Zy,py, Xz,Yz,Zz,pz]"""
    z = _norm(_vec(target, px)); x = _norm(_cross(up, z)); y = _cross(z, x)
    return [x[0], y[0], z[0], px[0], x[1], y[1], z[1], px[1], x[2], y[2], z[2], px[2]]

def find(sim, bare):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None

def ensure_camera(sim):
    """创建（或复用）俯视相机，返回句柄"""
    cam = find(sim, C.CAM_NAME)
    if cam is not None:
        try:
            sim.removeObjects([cam])
        except Exception:
            pass
    cam = sim.createVisionSensor(7, [C.CAM_W, C.CAM_H, 0, 0],
                                 [C.CAM_NEAR, C.CAM_FAR, math.radians(C.CAM_FOV), 0.05,
                                  0, 0, 0, 0, 0, 0, 0])
    sim.setObjectAlias(cam, C.CAM_NAME)
    sim.setObjectMatrix(cam, sim.handle_world,
                        look_at_matrix(C.CAM_POS, C.CAM_TARGET, C.CAM_UP))
    return cam


# ---------------- 采集并发布 ----------------
def grab_frame(sim, cam):
    sim.handleVisionSensor(cam)
    img, res = sim.getVisionSensorImg(cam)
    frame = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    frame = cv2.flip(frame, 0)                      # CoppeliaSim 图像为上下翻转
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

def publish_frame(sim, cam, save_history=True):
    """抓一帧并发布（图像 + 相机标定）"""
    C.ensure_bus()
    frame = grab_frame(sim, cam)
    cv2.imwrite(C.TOPIC_LATEST, frame)
    if save_history:
        idx = len([f for f in os.listdir(C.FRAME_DIR) if f.startswith("frame_")])
        cv2.imwrite(os.path.join(C.FRAME_DIR, "frame_%04d.jpg" % idx), frame)
    meta = {
        "stamp": time.time(),
        "resolution": [C.CAM_W, C.CAM_H],
        "fov_deg": C.CAM_FOV,
        "camera_matrix": sim.getObjectMatrix(cam, sim.handle_world),
        "camera_pos": sim.getObjectPosition(cam, sim.handle_world),
        "note": "俯视相机；像素->世界反投影见 detector_node.pixel_to_world",
    }
    with open(C.TOPIC_CAM_META, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return frame, meta


# ---------------- 独立运行（调试） ----------------
def main():
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    dt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    sim = RemoteAPIClient(host="localhost", port=23000).require("sim")
    try:
        sim.startSimulation()
    except Exception:
        pass
    time.sleep(0.8)
    cam = ensure_camera(sim)
    print("相机就绪 h=%d，开始发布 %d 帧（间隔 %.2fs）" % (cam, n, dt))
    for i in range(n):
        publish_frame(sim, cam)
        print("  发布 frame %d -> %s" % (i, C.TOPIC_LATEST))
        time.sleep(dt)
    print("相机节点结束（图像已发布到 vision_bus/）")


if __name__ == "__main__":
    main()
