# -*- coding: utf-8 -*-
"""verify_vision_video.py —— 验收视觉演示视频：时长/帧数/画中画出现比例/帧间差"""
import cv2
import numpy as np
import os
import shutil

P = r"C:\Users\39562\Desktop\实验三_视觉分类演示.mp4"
cap = cv2.VideoCapture(P)
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = cap.get(cv2.CAP_PROP_FPS)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("视频: %.1fs  %d帧  %dx%d  %.1fMB" % (n / fps, n, w, h, os.path.getsize(P) / 1024 / 1024))

border = np.array([0, 200, 255])       # 画中画边框颜色(BGR)
pip_frames = 0
checked = 0
prev = None
diffs = []
for i in range(0, n, 5):
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ok, fr = cap.read()
    if not ok:
        continue
    checked += 1
    roi = fr[66:320, w - 430:w - 6]
    d = np.abs(roi.astype(np.int16) - border).sum(axis=2)
    if (d < 150).sum() > 200:
        pip_frames += 1
    if prev is not None:
        diffs.append(np.abs(fr.astype(np.int16) - prev).mean())
    prev = fr
cap.release()
print("抽检 %d 帧: 出现画中画的帧 = %d (%.1f%%)  [6 次抓取窗口，预期约占 10%%]" %
      (checked, pip_frames, pip_frames / max(1, checked) * 100))
print("帧间差: 最小 %.2f  平均 %.2f" % (min(diffs), sum(diffs) / len(diffs)))

src = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\vision_bus\gripper_annotated.jpg"
dst = r"C:\Users\39562\Desktop\实验三_夹爪视角识别结果.jpg"
if os.path.exists(src):
    shutil.copyfile(src, dst)
    print("夹爪视角识别图 ->", dst)
print("俯视识别图存在:", os.path.exists(r"C:\Users\39562\Desktop\实验三_视觉识别结果.jpg"))
