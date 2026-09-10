# -*- coding: utf-8 -*-
"""verify_three_views.py —— 验收三个视角的视觉分类演示视频（画中画出现比例、帧间差）"""
import cv2
import numpy as np
import os

DIR = r"C:\Users\39562\Desktop\实验三_演示视频"
FILES = ["实验三_视觉分类演示_正面.mp4", "实验三_视觉分类演示_俯视.mp4", "实验三_视觉分类演示_侧视.mp4"]
border = np.array([0, 200, 255])       # 画中画边框（BGR）

for name in FILES:
    p = os.path.join(DIR, name)
    cap = cv2.VideoCapture(p)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 1
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    pip = 0
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
            pip += 1
        if prev is not None:
            diffs.append(np.abs(fr.astype(np.int16) - prev).mean())
        prev = fr
    cap.release()
    print("%-34s %5.1fs %4d帧 %dx%d %5.1fMB | 画中画 %4.1f%% | 帧间差 %.2f~%.2f" %
          (name, n / fps, n, w, h, os.path.getsize(p) / 1024 / 1024,
           pip / max(1, checked) * 100, min(diffs), max(diffs)))
