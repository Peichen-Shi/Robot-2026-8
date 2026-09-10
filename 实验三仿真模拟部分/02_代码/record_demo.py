# -*- coding: utf-8 -*-
"""
record_demo.py —— 录制 CoppeliaSim 窗口画面成 MP4
用法:
    python -u record_demo.py <输出mp4路径> [停止标志文件] [最长秒数] [fps]
停止: 出现停止标志文件 / 超时 / Ctrl+C
"""
import sys
import os
import time
import ctypes
from ctypes import wintypes

import numpy as np
import cv2
from PIL import ImageGrab

OUT = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\39562\Desktop\demo.mp4"
STOP_FILE = sys.argv[2] if len(sys.argv) > 2 else r"C:\Users\39562\AppData\Local\Temp\stop_record.flag"
MAX_SEC = float(sys.argv[3]) if len(sys.argv) > 3 else 420.0
FPS = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0
TARGET_W = 1280

user32 = ctypes.windll.user32
user32.SetProcessDPIAware()

def find_window(substr="CoppeliaSim"):
    result = []
    EnumWindowsProc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)

    def cb(hwnd, lparam):
        if not user32.IsWindowVisible(hwnd):
            return True
        n = user32.GetWindowTextLengthW(hwnd)
        if n == 0:
            return True
        buf = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(hwnd, buf, n + 1)
        if substr.lower() in buf.value.lower():
            result.append((hwnd, buf.value))
        return True

    user32.EnumWindows(EnumWindowsProc(cb), 0)
    return result

wins = find_window("CoppeliaSim")
if not wins:
    print("!! 找不到 CoppeliaSim 窗口"); sys.exit(2)
hwnd, title = wins[0]
print("窗口:", title, "hwnd=", hwnd, flush=True)

# 还原并提到前台
SW_RESTORE = 9
user32.ShowWindow(hwnd, SW_RESTORE)
try:
    user32.SetForegroundWindow(hwnd)
except Exception as e:
    print("置前失败(继续):", e, flush=True)

# 录制期间置顶，避免被其它窗口遮挡
HWND_TOPMOST, HWND_NOTOPMOST = -1, -2
SWP_NOMOVE, SWP_NOSIZE = 0x0002, 0x0001
try:
    user32.SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE)
    print("已置顶 CoppeliaSim 窗口", flush=True)
except Exception as e:
    print("置顶失败(继续):", e, flush=True)
time.sleep(1.0)

class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                ("right", ctypes.c_long), ("bottom", ctypes.c_long)]

rc = RECT()
user32.GetWindowRect(hwnd, ctypes.byref(rc))
# 去掉标题栏/边框（约 30px 顶、8px 边）
left, top = rc.left + 8, rc.top + 31
right, bottom = rc.right - 8, rc.bottom - 8
w, h = right - left, bottom - top
print("捕获区域: (%d,%d)-(%d,%d)  %dx%d" % (left, top, right, bottom, w, h), flush=True)

scale = min(1.0, TARGET_W / float(w))
ow, oh = int(w * scale) // 2 * 2, int(h * scale) // 2 * 2
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
vw = cv2.VideoWriter(OUT, fourcc, FPS, (ow, oh))
if not vw.isOpened():
    print("!! VideoWriter 打开失败"); sys.exit(3)

t0 = time.time()
frames = 0
interval = 1.0 / FPS
try:
    while True:
        if os.path.exists(STOP_FILE):
            print("检测到停止标志，结束录制", flush=True)
            break
        if time.time() - t0 > MAX_SEC:
            print("达到最长录制时间", flush=True)
            break
        tick = time.time()
        img = ImageGrab.grab(bbox=(left, top, right, bottom))
        frame = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        if scale != 1.0:
            frame = cv2.resize(frame, (ow, oh), interpolation=cv2.INTER_AREA)
        vw.write(frame)
        frames += 1
        if frames % int(FPS * 10) == 0:
            print("已录 %.0fs / %d 帧" % (time.time() - t0, frames), flush=True)
        dt = time.time() - tick
        if dt < interval:
            time.sleep(interval - dt)
finally:
    vw.release()
    try:
        user32.SetWindowPos(hwnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE)
    except Exception:
        pass
    print("录制结束: %s  共 %d 帧  时长 %.1fs" % (OUT, frames, time.time() - t0), flush=True)
