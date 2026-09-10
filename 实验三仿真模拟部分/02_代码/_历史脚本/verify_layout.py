# -*- coding: utf-8 -*-
"""verify_layout.py —— 验收三个视角视频的画面组成：
   ① 左上角全程有“识别相机”画面（区域非纯色、且随时间变化）
   ② 右上角抓取时出现“夹爪视角”画中画（黄色边框）
   ③ 标题栏/步骤栏有中文字（用白色像素墨量判断是否真的绘制成功）
"""
import cv2
import numpy as np
import os

DIR = r"C:\Users\39562\Desktop\实验三_演示视频"
FILES = ["实验三_视觉分类演示_正面.mp4", "实验三_视觉分类演示_俯视.mp4", "实验三_视觉分类演示_侧面.mp4"]
BORDER = np.array([255, 200, 0])       # PIL 里写的 RGB(0,200,255) → BGR 是 (255,200,0)
LABEL_C = np.array([255, 220, 90])     # 步骤文字 RGB(90,220,255) → BGR

for name in FILES:
    p = os.path.join(DIR, name)
    if not os.path.exists(p):
        print("缺少:", name); continue
    cap = cv2.VideoCapture(p)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps = cap.get(cv2.CAP_PROP_FPS) or 1
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ov_ok = ov_change = pip_ok = title_ink = label_ink = checked = 0
    prev_ov = None
    for i in range(0, n, 6):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i); ok, fr = cap.read()
        if not ok: continue
        checked += 1
        ov = fr[110:110+214, 18:18+380]                      # 左上识别相机
        if ov.std() > 12: ov_ok += 1
        g = cv2.cvtColor(ov, cv2.COLOR_BGR2GRAY)
        if prev_ov is not None and np.abs(g.astype(int) - prev_ov.astype(int)).mean() > 1.0:
            ov_change += 1
        prev_ov = g
        pip = fr[110:110+214, w-380-18:w-18]                 # 右上夹爪视角
        d = np.abs(pip.astype(np.int16) - BORDER).sum(axis=2)
        if (d < 150).sum() > 60: pip_ok += 1
        title = fr[6:32, 10:900]                             # 标题文字区
        label = fr[70:98, 10:1100]                           # 步骤文字区
        title_ink += int((title.min(axis=2) > 170).sum() > 400)
        dl = np.abs(label.astype(np.int16) - LABEL_C).sum(axis=2)
        label_ink += int((dl < 120).sum() > 300)
    cap.release()
    print("%-30s %5.1fs %4d帧 %dx%d %5.1fMB" % (name, n/fps, n, w, h, os.path.getsize(p)/1024/1024))
    print("    左上识别相机出现率 %.0f%%  画面在变化 %.0f%% ｜ 右上夹爪画中画 %.1f%% ｜ 标题文字 %d/%d ｜ 步骤文字 %d/%d" %
          (ov_ok/max(1,checked)*100, ov_change/max(1,checked)*100, pip_ok/max(1,checked)*100,
           title_ink, checked, label_ink, checked))
