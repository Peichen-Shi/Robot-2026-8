# -*- coding: utf-8 -*-
"""check_english.py —— ① 静态检查：画面绘制调用里是否还残留中文
                      ② 视频检查：右上角信息区是否有“类别/置信度/是否夹取”三行文字"""
import re
import io
import os
import cv2
import numpy as np

# ---------- ① 静态检查 ----------
P = "vision_pipeline_recorded.py"
src = io.open(P, encoding="utf-8").read().splitlines()
CJK = re.compile(r"[\u4e00-\u9fff]")
DRAW_KEYS = ("d.text(", "label(", "paste_bgr(", "box_note=", "pip_lines", "state[\"label\"]", "txt =")
bad = []
for i, ln in enumerate(src, 1):
    if any(k in ln for k in DRAW_KEYS) and CJK.search(ln):
        # 允许：注释行；以及 ZH 字典里的中文（画面文字的中文版）
        if ln.strip().startswith("#"):
            continue
        if re.match(r'\s*"[a-z_]+":', ln) or ln.strip().startswith('"'):
            continue
        bad.append((i, ln.strip()))
print("① 画面文字静态检查：", "全部为英文 ✅" if not bad else "发现中文残留 %d 处" % len(bad))
for i, ln in bad[:10]:
    print("   %d: %s" % (i, ln))

# ---------- ② 视频检查 ----------
V = r"C:\Users\39562\Desktop\实验三_演示视频\实验三_视觉分类演示_正面_位置4无物体_自动跳过_EN.mp4"
cap = cv2.VideoCapture(V)
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps = cap.get(cv2.CAP_PROP_FPS) or 1
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
info_frames = white_ink = green_ink = 0
checked = 0
for i in range(0, n, 4):
    cap.set(cv2.CAP_PROP_POS_FRAMES, i); ok, fr = cap.read()
    if not ok:
        continue
    checked += 1
    pip_area = fr[110:110+214, w-380-18:w-18]
    d = np.abs(pip_area.astype(np.int16) - np.array([255, 200, 0])).sum(axis=2)
    if (d < 150).sum() > 60:                       # 画中画出现
        info_frames += 1
        info = fr[330:400, w-380-18:w-18]          # 画中画下方的信息区
        white_ink += int((info.min(axis=2) > 200).sum() > 200)
        g = np.abs(info.astype(np.int16) - np.array([140, 255, 140])).sum(axis=2)
        green_ink += int((g < 120).sum() > 200)
cap.release()
print("② 视频：%.1fs %d帧 %dx%d" % (n/fps, n, w, int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if False else fr.shape[0]))
print("   画中画出现 %d 帧；其中信息区含白色文字 %d 帧、含绿色(置信度/夹取)文字 %d 帧" %
      (info_frames, white_ink, green_ink))
