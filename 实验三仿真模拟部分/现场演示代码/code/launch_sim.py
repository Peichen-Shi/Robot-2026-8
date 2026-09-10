# -*- coding: utf-8 -*-
"""
launch_sim.py —— 【现场演示版】实验三 完整仿真 一键启动（Launch 文件）
=====================================================================
它做四件事（老师/评委看这一段就能明白系统构成）：
  1) 启动 CoppeliaSim 并载入本文件夹内的场景 scene/second_imitation.ttt
  2) 等待 ZMQ Remote API 就绪（端口 23000）
  3) 检查相机标定数据 code/vision_bus/calibration.json，缺失则自动标定
  4) 运行视觉分类抓取主流程（code/vision_pipeline_recorded.py），并把结果汇总打印出来

用法：
  python launch_sim.py <视角> [视频输出|auto] [界面语言] [开关...]
      视角    : front（正面）/ top（俯视）/ side（侧面）
      视频输出: auto = 自动命名放到 videos/ 目录
      界面语言: zh（中文）/ en（英文）
      开关    :
        --fresh      先关闭已有的 CoppeliaSim，再冷启动（推荐，保证载入正确场景）
        --no-start   不启动 CoppeliaSim，复用已经打开的实例
        --close      跑完自动关闭 CoppeliaSim
        --unreachable     特殊情况：把 6 号物体放到抓不到的位置 → 触发“不可达”报错中止
        --noobj           特殊情况：把桌上物体全部移走 → 所有位置都判定“无物体”并跳过
        --bincap=N        特殊情况：把每个料盒的容量设为 N（演示“料盒已满 → 报错中止”）
        --slots=1,2       快速演示：只处理指定位置（例如只抓第 1 个物体）

示例：
  python launch_sim.py front auto zh --fresh
  python launch_sim.py front auto zh --fresh --unreachable
  python launch_sim.py top   auto zh --fresh --slots=1,2
=====================================================================
"""
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))              # .../实验三_现场演示代码/code
ROOT = os.path.dirname(HERE) if os.path.basename(HERE).lower() == "code" else HERE
PY = sys.executable
PORT = 23000

SCENE_CANDIDATES = [
    os.path.join(ROOT, "scene", "second_imitation.ttt"),
    os.path.join(HERE, "second_imitation.ttt"),
    os.path.join(ROOT, "second_imitation.ttt"),
]
VIDEO_DIR = os.path.join(ROOT, "videos")
LOG_DIR = os.path.join(ROOT, "logs")

COPPELIA_CANDIDATES = [
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe",
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimPro\coppeliaSim.exe",
    r"C:\Program Files (x86)\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe",
]

# launch_sim 自己消费的开关；其余 "--xxx" 一律透传给主流程
OWN_FLAGS = {"--fresh", "--no-start", "--close"}


def log(msg, fh=None):
    line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    if fh:
        fh.write(line + "\n")
        fh.flush()


def port_open(host="localhost", port=PORT, timeout=0.4):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def wait_port(want_open=True, timeout=90.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if port_open() == want_open:
            return True
        time.sleep(0.6)
    return False


def find_coppelia():
    for p in COPPELIA_CANDIDATES:
        if os.path.exists(p):
            return p
    try:
        out = subprocess.run(["where", "coppeliaSim.exe"], capture_output=True, text=True).stdout
        for line in (out or "").splitlines():
            line = line.strip()
            if line and os.path.exists(line):
                return line
    except Exception:
        pass
    return None


def kill_coppelia(fh):
    """关闭正在运行的 CoppeliaSim（连同其子进程）——只杀该进程树，不影响本脚本"""
    try:
        r = subprocess.run(["tasklist", "/FI", "IMAGENAME eq coppeliaSim.exe", "/FO", "CSV", "/NH"],
                           capture_output=True, text=True)
        pids = []
        for line in (r.stdout or "").splitlines():
            parts = [x.strip('"') for x in line.split('","')]
            if len(parts) >= 2 and parts[0].lower().startswith("coppeliasim"):
                pids.append(parts[1])
        for pid in pids:
            log("   关闭已有的 CoppeliaSim (PID=%s)" % pid, fh)
            subprocess.run(["taskkill", "/PID", pid, "/T", "/F"], capture_output=True, text=True)
        if pids:
            wait_port(want_open=False, timeout=20.0)
            time.sleep(1.0)
    except Exception as e:
        log("   （关闭 CoppeliaSim 时出现问题：%s）" % e, fh)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    passthrough = [f for f in flags if f not in OWN_FLAGS]
    fresh = "--fresh" in flags
    reuse = "--no-start" in flags

    view = (args[0] if len(args) > 0 else "front").lower()
    if view not in ("front", "top", "side"):
        view = "front"
    out = args[1] if len(args) > 1 else "auto"
    lang = (args[2] if len(args) > 2 else "zh").lower()
    if lang not in ("zh", "en"):
        lang = "zh"

    os.makedirs(VIDEO_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    logf = os.path.join(LOG_DIR, "launch_%s_%s.log" % (view, time.strftime("%Y%m%d_%H%M%S")))
    fh = open(logf, "w", encoding="utf-8")

    view_cn = {"front": "正面视角", "top": "俯视视角", "side": "侧面视角"}[view]
    scene = next((p for p in SCENE_CANDIDATES if os.path.exists(p)), None)
    coppelia = find_coppelia()

    log("=" * 66, fh)
    log("实验三 · 桌面物体自动分类整理 —— 完整仿真一键启动（现场演示版）", fh)
    log("视角=%s（%s）  界面语言=%s  额外开关=%s" % (view, view_cn, lang, passthrough or "无"), fh)
    log("场景文件: %s" % (scene or "!! 未找到 scene/second_imitation.ttt"), fh)
    log("仿真软件: %s" % (coppelia or "!! 未找到 coppeliaSim.exe"), fh)
    log("=" * 66, fh)

    if scene is None:
        log("!! 缺少场景文件，无法演示。请确认 scene\\second_imitation.ttt 存在。", fh)
        fh.close()
        return 2

    # ---------------- ① 启动 CoppeliaSim ----------------
    if fresh:
        kill_coppelia(fh)
    if port_open():
        log("① 检测到仿真接口已在监听（端口 %d）：复用当前 CoppeliaSim 实例" % PORT, fh)
        proc = None
    elif reuse:
        log("!! 指定了 --no-start，但端口 %d 没有在监听；请先打开 CoppeliaSim 并载入场景" % PORT, fh)
        fh.close()
        return 3
    else:
        if coppelia is None:
            log("!! 未找到 coppeliaSim.exe，请手动打开 CoppeliaSim 并载入：%s" % scene, fh)
            fh.close()
            return 4
        log("① 启动 CoppeliaSim 并载入场景 ...", fh)
        # 把仿真器的输出单独写到文件：既让演示窗口保持干净，也避免仿真进程占住本进程的输出管道
        try:
            cop_out = open(os.path.join(LOG_DIR, "coppelia_output.txt"), "w",
                           encoding="utf-8", errors="replace")
        except Exception:
            cop_out = subprocess.DEVNULL
        proc = subprocess.Popen([coppelia, scene], stdout=cop_out, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL)
        log("   PID=%d（仿真器输出见 logs\\coppelia_output.txt）" % proc.pid, fh)

    # ---------------- ② 等待 ZMQ 接口 ----------------
    log("② 等待 ZMQ Remote API（端口 %d）...", fh)
    if not wait_port(want_open=True, timeout=90.0):
        log("!! 仿真接口 90 秒内未就绪；启动失败", fh)
        fh.close()
        return 5
    log("   接口就绪", fh)

    # ---------------- ③ 相机标定 ----------------
    calib = os.path.join(HERE, "vision_bus", "calibration.json")
    if os.path.exists(calib):
        log("③ 相机标定数据已存在：code/vision_bus/calibration.json", fh)
    else:
        log("③ 缺少相机标定，自动执行 vision_calibrate.py ...", fh)
        r = subprocess.run([PY, "-u", "vision_calibrate.py"], cwd=HERE,
                           capture_output=True, text=True, encoding="utf-8", errors="replace")
        fh.write(r.stdout or "")
        log("   标定完成（返回码 %d）" % r.returncode, fh)

    # ---------------- ④ 运行主流程 ----------------
    cmd = [PY, "-u", "vision_pipeline_recorded.py", view, out, lang] + passthrough
    log("④ 启动视觉分类抓取主流程：", fh)
    log("   %s" % " ".join('"%s"' % c if " " in c else c for c in cmd), fh)
    t0 = time.time()
    r = subprocess.run(cmd, cwd=HERE, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    dur = time.time() - t0
    out_txt = r.stdout or ""
    fh.write(out_txt)
    if r.stderr:
        fh.write("[stderr]\n" + (r.stderr or ""))
    # 把主流程的输出原样显示给现场观众
    print(out_txt, end="", flush=True)
    if r.stderr:
        print(r.stderr, end="", flush=True)

    # ---------------- ⑤ 汇总 ----------------
    log("⑤ 主流程结束：返回码 %d，用时 %.1f 秒" % (r.returncode, dur), fh)
    summ = None
    try:
        files = sorted((os.path.join(LOG_DIR, f) for f in os.listdir(LOG_DIR)
                        if f.startswith("summary_") and f.endswith(".json")),
                       key=os.path.getmtime)
        if files:
            with open(files[-1], encoding="utf-8") as f2:
                summ = json.load(f2)
    except Exception:
        pass
    if summ:
        log("   本次结果：抓取 %s 次 / 成功 %s 次 / 成功率 %.0f%% / 错误 %s / 安全违规 %s"
            % (summ.get("grasp_attempts"), summ.get("grasp_success"),
               (summ.get("success_rate") or 0) * 100, summ.get("errors"), summ.get("violations")), fh)
        vid = summ.get("video")
        if vid:
            log("   演示视频：%s" % os.path.join(VIDEO_DIR, vid), fh)
    log("   运行日志（轨迹/结果/错误/汇总）：%s" % LOG_DIR, fh)
    log("   启动日志：%s" % logf, fh)
    log("=" * 66, fh)
    fh.close()

    if "--close" in flags and proc is not None:
        log("按要求关闭 CoppeliaSim")
        proc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
