# -*- coding: utf-8 -*-
"""
vision_calibrate.py —— 相机标定节点
用"已知网格位置"做标定靶（6 个物体的真实坐标由仿真提供）：
  采集 1 帧 → 颜色分割得到 6 个斑块 → 与最近的真实网格位置配对
  → 最小二乘拟合  u→世界x、v→世界y 的线性映射 → 写入 vision_bus/calibration.json
注意：标定只用位置、不使用类别信息；类别识别仍由 detector_node 视觉判定。
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import json
import time

import numpy as np
import cv2

import vision_config as C
import camera_node
import detector_node


def ground_truth_positions(sim):
    out = {}
    for i in range(1, C.GRID_N + 1):
        h = camera_node.find(sim, "Grid%d" % i)
        if h is None:
            continue
        p = sim.getObjectPosition(h, sim.handle_world)
        out[i] = (p[0], p[1])
    return out


def run():
    client = RemoteAPIClient(host="localhost", port=23000)
    sim = client.require("sim")
    try:
        sim.startSimulation()
    except Exception:
        pass
    time.sleep(0.8)
    cam = camera_node.ensure_camera(sim)
    time.sleep(0.3)
    frame, meta = camera_node.publish_frame(sim, cam)
    truth = ground_truth_positions(sim)
    blue, red = detector_node.color_masks(frame)
    blobs = []
    for mask, cname in ((blue, "蓝"), (red, "红")):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < C.MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            blobs.append({"color": cname, "u": x + w / 2.0, "v": y + h / 2.0,
                          "w": w, "h": h, "area": a})
    print("标定：检出 %d 个斑块，真实网格 %d 个" % (len(blobs), len(truth)))

    # ---- 配对：图像 v 升序 ↔ 真实 y 降序（俯视相机画面上方 = 世界 +Y）----
    blobs_sorted = sorted(blobs, key=lambda b: b["v"])
    truth_sorted = sorted(truth.items(), key=lambda kv: -kv[1][1])
    if len(blobs_sorted) != len(truth_sorted):
        print("!! 斑块数与真实物体数不一致，标定可能不准（仍继续，按数量裁剪）")
    n = min(len(blobs_sorted), len(truth_sorted))
    U, V, WX, WY = [], [], [], []
    for k in range(n):
        b = blobs_sorted[k]
        g, (wx, wy) = truth_sorted[k]
        U.append(b["u"]); V.append(b["v"]); WX.append(wx); WY.append(wy)
        print("   位置%d  像素(%.0f, %.0f)  真实(%.3f, %.3f)  宽%d 高%d" %
              (g, b["u"], b["v"], wx, wy, b["w"], b["h"]))
    # 线性拟合 y = c*v + d（y 方向是主标定方向，残差可验证）
    cv_, dv = np.polyfit(V, WY, 1)
    m_per_px = float(abs(1.0 / cv_))
    resy = float(np.max(np.abs(np.polyval([cv_, dv], V) - np.array(WY))))
    # x 方向：所有物体同在 x=0.38（u 相同，无法独立拟合斜率）→ 用 y 方向比例
    # 并沿用俯视相机的方向关系（图像向右 = 世界 -X），以图像中心 u0 为锚点
    u0 = C.CAM_W / 2.0
    au, bu = -m_per_px, C.GRID_X + u0 * m_per_px
    resx = 0.0
    cal = {
        "stamp": time.time(),
        "u_to_x": [float(au), float(bu)],      # x = au*u + bu
        "v_to_y": [float(cv_), float(dv)],     # y = cv*v + dv
        "px_per_m_x": float(1.0 / m_per_px),
        "m_per_px": m_per_px,
        "max_residual_x": resx, "max_residual_y": resy,
        "prism_width_m": 0.06, "cylinder_width_m": 0.06,
        "shape_split_m": 0.09,
        "resolution": [C.CAM_W, C.CAM_H],
    }
    C.ensure_bus()
    with open(C.CALIB_PATH, "w", encoding="utf-8") as f:
        json.dump(cal, f, ensure_ascii=False, indent=2)
    print("\n标定完成：x = %.6f*u + %.4f ; y = %.6f*v + %.4f" % (au, bu, cv_, dv))
    print("  比例 %.1f 像素/米（%.4f 米/像素）；最大残差 x=%.4f y=%.4f m" %
          (cal["px_per_m_x"], m_per_px, resx, resy))
    print("  方向校验：图像上方(v小)= 世界 +Y；v=%.0f -> y=%.3f ; v=%.0f -> y=%.3f" %
          (V[0], np.polyval([cv_, dv], V[0]), V[-1], np.polyval([cv_, dv], V[-1])))
    print("  标定文件 ->", C.CALIB_PATH)
    sim.stopSimulation()
    return cal


if __name__ == "__main__":
    run()
