# -*- coding: utf-8 -*-
"""
detector_node.py —— 识别节点（发布 物体类别 / 检测框 / 置信度）
输入：vision_bus/latest.jpg + camera_meta.json（相机节点发布）
输出：vision_bus/detections.json      每个目标的 类别/检测框/置信度/世界坐标/网格号
      vision_bus/annotated_latest.jpg 标注图
      C:/Users/39562/Desktop/实验三_视觉识别结果.jpg  （便于查看）

识别原理（面向本实验的轻量视觉）：
  1) 颜色分割（HSV）：蓝色 → A 类候选；红色 → B 类候选
  2) 轮廓分析：圆度 circularity = 4πA/P²
       圆柱(圆) ≈ 1.0   棱柱(方) ≈ 0.78
  3) 类别判定 = 颜色 + 形状 双条件（A=蓝色棱柱 / B=红色圆柱）；
     两个条件不一致时判为“低置信”，降低置信度
  4) 检测框中心 →（按物体顶面高度校正的）反投影 → 世界坐标 → 网格号(1~6)
可独立运行：python detector_node.py [帧数]
"""
import json
import os
import shutil
import sys
import time

import numpy as np
import cv2

import vision_config as C


# ---------------- 反投影：像素 → 世界 ----------------
_CALIB_CACHE = {"mtime": None, "data": None}

def load_calibration():
    """读取相机标定（vision_bus/calibration.json），没有则返回 None"""
    try:
        mt = os.path.getmtime(C.CALIB_PATH)
    except OSError:
        return None
    if _CALIB_CACHE["mtime"] != mt:
        with open(C.CALIB_PATH, "r", encoding="utf-8") as f:
            _CALIB_CACHE["data"] = json.load(f)
        _CALIB_CACHE["mtime"] = mt
    return _CALIB_CACHE["data"]


def pixel_to_world(meta, u, v, obj_top_z=C.OBJ_TOP_Z):
    """像素 → 世界坐标
    优先使用标定结果（线性映射 x=a*u+b, y=c*v+d）；
    无标定时退回解析反投影（俯视相机 + 顶面高度校正）"""
    cal = load_calibration()
    if cal is not None:
        au, bu = cal["u_to_x"]; cvv, dv = cal["v_to_y"]
        return float(au * u + bu), float(cvv * v + dv)
    m = meta["camera_matrix"]
    Xc = np.array([m[0], m[4], m[8]])
    Yc = np.array([m[1], m[5], m[9]])
    P = np.array([m[3], m[7], m[11]])
    W, H = meta["resolution"]
    cam_z = P[2]
    span_x = 2.0 * cam_z * np.tan(np.radians(meta["fov_deg"]) / 2.0)
    s = span_x / float(W)
    k = 1.0 - obj_top_z / cam_z                     # 顶面高度校正（相似变换）
    off_right = (u - W / 2.0) * s * k
    off_up = (H / 2.0 - v) * s * k
    xy = P[:2] + off_right * Xc[:2] + off_up * Yc[:2]
    return float(xy[0]), float(xy[1])


def world_to_grid(x, y):
    """检测框中心 → 网格号(1~6)；不在物体行区间内返回 None"""
    if abs(x - C.GRID_X) > C.ROW_X_TOL:
        return None
    idx = int(round((y - C.GRID_Y0) / C.GRID_DY)) + 1
    if 1 <= idx <= C.GRID_N:
        yc = C.GRID_Y0 + (idx - 1) * C.GRID_DY
        if abs(y - yc) <= C.GRID_Y_TOL:
            return idx
    return None


# ---------------- 颜色分割 ----------------
def color_masks(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    def build(ranges):
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        return m
    blue = build(C.HSV_BLUE)
    red = build(C.HSV_RED)
    kernel = np.ones((5, 5), np.uint8)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    for m in (blue, red):
        cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, dst=m)
    return blue, red


# ---------------- 检测 ----------------
def detect(img_bgr, meta, map_world=True):
    blue, red = color_masks(img_bgr)
    cal = load_calibration()
    m_per_px = cal["m_per_px"] if cal else 0.0018
    dets = []
    for mask, color_name, expect_class in ((blue, "蓝", C.CLASS_A), (red, "红", C.CLASS_B)):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < C.MIN_AREA:
                continue
            per = cv2.arcLength(c, True) or 1e-6
            circ = float(min(1.0, 4.0 * np.pi * area / (per * per)))     # 圆度
            x, y, w, h = cv2.boundingRect(c)
            u, v = x + w / 2.0, y + h / 2.0
            if map_world and meta is not None:
                wx, wy = pixel_to_world(meta, u, v)
                grid = world_to_grid(wx, wy)
            else:
                wx = wy = None
                grid = None
            # 形状判别：长焦俯视下用圆度最稳（棱柱≈0.78，圆柱≈0.92）
            top_w_m = w * m_per_px
            round_shape = circ > C.CIRC_ROUND
            shape_name = "圆柱(圆)" if round_shape else "棱柱(方)"
            shape_class = C.CLASS_B if round_shape else C.CLASS_A
            # 颜色质量：检测框内该颜色像素占轮廓面积的比例（斑块实心度）
            roi = mask[y:y+h, x:x+w]
            purity = min(1.0, float(roi.sum() / 255.0) / max(1.0, area))
            agree = (shape_class == expect_class)
            circ_ref = 0.92 if expect_class == C.CLASS_B else 0.78
            circ_score = max(0.0, 1.0 - abs(circ - circ_ref) / 0.30)
            shape_score = max(0.0, 1.0 - abs(top_w_m - 0.06) / 0.04)
            conf = 0.40 * purity + 0.35 * shape_score + 0.25 * circ_score
            if not agree:
                conf *= 0.50
            cls = expect_class       # 颜色为主特征，形状用于一致性校验
            dets.append({
                "class": cls,
                "class_name": C.CLASS_NAME.get(cls, "未知"),
                "confidence": round(float(conf), 3),
                "color": color_name,
                "shape": shape_name,
                "top_width_m": round(float(top_w_m), 4),
                "circularity": round(circ, 3),
                "consistent": bool(agree),
                "bbox": [int(x), int(y), int(w), int(h)],
                "bbox_center_px": [round(u, 1), round(v, 1)],
                "world_xy": [round(wx, 4), round(wy, 4)] if wx is not None else None,
                "grid_index": grid,
                "bin": C.CLASS_BIN.get(cls) if grid else None,
            })
    dets.sort(key=lambda d: (d["grid_index"] is None, d["grid_index"] or 99, -d["confidence"]))
    return dets


def detect_local(img_bgr):
    """夹爪第一视角的近距识别：只发布 类别/检测框/置信度（不做世界映射与网格判定）
    近距透视图下"圆度"不再可靠（顶面被压缩），因此置信度按
    颜色纯度 + 斑块尺寸合理性 计算，形状仅作提示。"""
    dets = detect(img_bgr, None, map_world=False)
    h, w = img_bgr.shape[:2]
    frame_area = float(w * h)
    blue, red = color_masks(img_bgr)
    masks = {C.CLASS_A: blue, C.CLASS_B: red}
    for d in dets:
        x, y, bw, bh = d["bbox"]
        area = float(bw * bh)
        purity = min(1.0, float(masks[d["class"]][y:y+bh, x:x+bw].sum() / 255.0) / max(1.0, area))
        frac = area / frame_area
        size_score = 1.0 if 0.001 <= frac <= 0.25 else max(0.2, 1.0 - abs(frac - 0.05) * 4.0)
        d["confidence"] = round(min(0.99, 0.6 * purity + 0.4 * size_score), 3)
        d["shape"] = d["shape"] + "(近距提示)"
        d["note"] = "近距视角：形状不作判据，类别由颜色决定"
    dets.sort(key=lambda d: -d["confidence"])
    return dets


# ---------------- 发布 ----------------
def annotate(img_bgr, dets):
    img = img_bgr.copy()
    for d in dets:
        x, y, w, h = d["bbox"]
        col = (200, 120, 40) if d["class"] == C.CLASS_A else (60, 60, 230)   # BGR
        cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
        g = d["grid_index"]
        tag = ("位置%d " % g if g else "位置? ") + "%s %.0f%%" % (d["class"], d["confidence"] * 100)
        cv2.putText(img, tag, (x, max(24, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, col, 2, cv2.LINE_AA)
        cv2.circle(img, (int(d["bbox_center_px"][0]), int(d["bbox_center_px"][1])), 4, col, -1)
        if g:
            cv2.putText(img, "-> %s" % d["bin"], (x, y + h + 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2, cv2.LINE_AA)
    nA = sum(1 for d in dets if d["class"] == C.CLASS_A)
    nB = sum(1 for d in dets if d["class"] == C.CLASS_B)
    cv2.rectangle(img, (0, 0), (img.shape[1], 42), (30, 30, 30), -1)
    cv2.putText(img, "Vision: %d objects  |  A(blue prism)=%d -> LeftBin   B(red cylinder)=%d -> RightBin"
                % (len(dets), nA, nB), (14, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.68,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


def publish(dets, img_bgr, copy_desktop=True):
    C.ensure_bus()
    payload = {
        "stamp": time.time(),
        "count": len(dets),
        "class_counts": {
            C.CLASS_A: sum(1 for d in dets if d["class"] == C.CLASS_A),
            C.CLASS_B: sum(1 for d in dets if d["class"] == C.CLASS_B),
        },
        "detections": dets,
    }
    with open(C.TOPIC_DETECTIONS, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    ann = annotate(img_bgr, dets)
    cv2.imwrite(C.TOPIC_ANNOTATED, ann)
    if copy_desktop:
        try:
            shutil.copyfile(C.TOPIC_ANNOTATED, C.DESKTOP_ANNOTATED)
        except Exception:
            pass
    return payload, ann


def process_latest():
    """读取总线上的最新图像 → 检测 → 发布结果"""
    if not (os.path.exists(C.TOPIC_LATEST) and os.path.exists(C.TOPIC_CAM_META)):
        raise RuntimeError("总线上还没有图像，请先运行 camera_node.py 发布一帧")
    img = cv2.imread(C.TOPIC_LATEST)
    with open(C.TOPIC_CAM_META, "r", encoding="utf-8") as f:
        meta = json.load(f)
    dets = detect(img, meta)
    payload, _ = publish(dets, img)
    return payload


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    for i in range(n):
        p = process_latest()
        print("识别节点：第 %d 次，检出 %d 个目标（A=%d, B=%d）" %
              (i + 1, p["count"], p["class_counts"][C.CLASS_A], p["class_counts"][C.CLASS_B]))
        for d in p["detections"]:
            print("   位置%-4s %s  bbox=%s  置信度=%.3f  世界=(%.3f, %.3f)  料盒=%s" %
                  (d["grid_index"], d["class_name"], d["bbox"], d["confidence"],
                   d["world_xy"][0], d["world_xy"][1], d["bin"]))
        time.sleep(0.4)


if __name__ == "__main__":
    main()
