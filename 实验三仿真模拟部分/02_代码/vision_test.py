# -*- coding: utf-8 -*-
"""vision_test.py —— 单测视觉链路：发布一帧 → 识别 → 与真值比对"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

import vision_config as C
import camera_node
import detector_node

client = RemoteAPIClient(host="localhost", port=23000)
sim = client.require("sim")

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)
sim.startSimulation()
time.sleep(1.0)

cam = camera_node.ensure_camera(sim)
time.sleep(0.3)
camera_node.publish_frame(sim, cam)
print("相机节点：已发布 1 帧 ->", C.TOPIC_LATEST)

payload = detector_node.process_latest()
print("识别节点：检出 %d 个目标（A=%d, B=%d）\n" %
      (payload["count"], payload["class_counts"][C.CLASS_A], payload["class_counts"][C.CLASS_B]))

print("%-6s %-14s %-22s %-7s %-8s %-22s %-12s %s" %
      ("网格", "类别", "检测框(x,y,w,h)", "置信度", "圆度", "世界坐标(反投影)", "真值", "判定料盒"))
max_err = 0.0
for d in payload["detections"]:
    g = d["grid_index"]
    truth = (C.GRID_X, C.GRID_Y0 + (g - 1) * C.GRID_DY) if g else (None, None)
    err = None
    if g:
        err = max(abs(d["world_xy"][0] - truth[0]), abs(d["world_xy"][1] - truth[1]))
        max_err = max(max_err, err)
    print("%-6s %-14s %-22s %-7.3f %-8.3f (%6.3f, %6.3f)   %-12s %s  %s" %
          (g, d["class_name"], str(d["bbox"]), d["confidence"], d["circularity"],
           d["world_xy"][0], d["world_xy"][1],
           "(%.2f,%.2f)" % (truth[0], truth[1]) if g else "-",
           d["bin"] or "-",
           ("误差%.3fm" % err) if err is not None else "未落入网格"))

print("\n反投影最大误差 = %.4f m" % max_err)
print("标注图 ->", C.TOPIC_ANNOTATED)
print("桌面副本 ->", C.DESKTOP_ANNOTATED)
sim.stopSimulation()
print("done")
