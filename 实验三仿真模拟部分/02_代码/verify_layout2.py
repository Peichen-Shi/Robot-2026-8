# -*- coding: utf-8 -*-
"""verify_layout2.py —— 验收新版视频：
   ① 左上角全程有画面（夹爪摄像头）
   ② 左上画面随时间变化（相机随机械臂运动 → 画面明显变化）
   ③ 右上角抓取时出现识别画中画（黄色边框）
   ④ 抓取时 左上(原始) 与 右上(带识别标注) 高度相似 → 证明两处都是夹爪摄像头
   ⑤ 标题/步骤中文字已绘制
"""
import cv2
import numpy as np
import os

DIR = r"C:\Users\39562\Desktop\实验三_演示视频"
FILES = ["实验三_视觉分类演示_正面.mp4", "实验三_视觉分类演示_俯视.mp4", "实验三_视觉分类演示_侧面.mp4"]
BORDER = np.array([255, 200, 0])     # PIL 写的 RGB(0,200,255) → BGR
LABEL_C = np.array([255, 220, 90])

for name in FILES:
    p = os.path.join(DIR, name)
    if not os.path.exists(p):
        print("缺少:", name); continue
    cap = cv2.VideoCapture(p)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps = cap.get(cv2.CAP_PROP_FPS) or 1
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ok_cnt = chg = pip_cnt = checked = title_ok = label_ok = 0
    sims = []
    prev = None
    for i in range(0, n, 5):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i); ok, fr = cap.read()
        if not ok: continue
        checked += 1
        tl = fr[110:110+214, 18:18+380]
        tr = fr[110:110+214, w-380-18:w-18]
        g = cv2.cvtColor(tl, cv2.COLOR_BGR2GRAY)
        if g.std() > 12: ok_cnt += 1
        if prev is not None and np.abs(g.astype(int)-prev.astype(int)).mean() > 1.5: chg += 1
        prev = g
        d = np.abs(tr.astype(np.int16) - BORDER).sum(axis=2)
        is_pip = (d < 150).sum() > 60
        if is_pip:
            pip_cnt += 1
            a = cv2.cvtColor(tl, cv2.COLOR_BGR2GRAY).astype(np.float32)
            b = cv2.cvtColor(tr, cv2.COLOR_BGR2GRAY).astype(np.float32)
            a = (a - a.mean()) / (a.std() + 1e-6); b = (b - b.mean()) / (b.std() + 1e-6)
            sims.append(float((a * b).mean()))
        t = fr[6:32, 10:900]
        title_ok += int((t.min(axis=2) > 170).sum() > 400)
        lb = fr[70:98, 10:1100]
        label_ok += int((np.abs(lb.astype(np.int16) - LABEL_C).sum(axis=2) < 120).sum() > 300)
    cap.release()
    print("%-30s %5.1fs %4d帧 %5.1fMB" % (name, n/fps, n, os.path.getsize(p)/1024/1024))
    print("    左上画面 出现%.0f%% / 变化%.0f%% ｜ 右上识别画中画 %.1f%% ｜ 两处画面相似度 %.2f ｜ 中文 标题%d 步骤%d /%d" %
          (ok_cnt/max(1,checked)*100, chg/max(1,checked)*100, pip_cnt/max(1,checked)*100,
           (sum(sims)/len(sims)) if sims else -1, title_ok, label_ok, checked))
