# -*- coding: utf-8 -*-
"""
launch_sim.py —— 【Launch 文件】一键启动完整仿真系统
============================================================
流程：
  ① 启动 CoppeliaSim 并载入场景 second_imitation.ttt
  ② 等待 ZMQ Remote API（默认端口 23000）
  ③ 若缺少相机标定文件则自动运行 vision_calibrate.py
  ④ 运行视觉分类抓取主流程（可选录制视频）
  ⑤ 全程日志写入 logs/launch_<时间戳>.log，并在结束时输出结果汇总

用法：
    python launch_sim.py                          # 默认：正面视角录制，中文界面
    python launch_sim.py top  out.mp4 en          # 俯视视角、英文界面
    python launch_sim.py front none en            # 只跑不录（none=不录制）
    python launch_sim.py --no-start               # 复用已打开的 CoppeliaSim
============================================================
"""
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\second_imitation.ttt"
COPPELIA = r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe"
PORT = 23000
LOGDIR = os.path.join(HERE, "logs")
PY = sys.executable


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


def wait_zmq(fh, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if port_open():
            return True
        time.sleep(1.0)
    return False


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    view = args[0] if len(args) > 0 else "front"
    out = args[1] if len(args) > 1 else os.path.join(
        r"C:\Users\39562\Desktop\实验三_演示视频", "实验三_视觉分类演示_正面_最终版.mp4")
    lang = args[2] if len(args) > 2 else "zh"
    reuse = "--no-start" in flags

    os.makedirs(LOGDIR, exist_ok=True)
    logf = os.path.join(LOGDIR, "launch_%s.log" % time.strftime("%Y%m%d_%H%M%S"))
    fh = open(logf, "w", encoding="utf-8")
    log("=== 实验三 完整仿真系统启动 (Launch) ===", fh)
    log("场景: %s" % SCENE, fh)
    log("视角=%s  输出=%s  界面语言=%s" % (view, out, lang), fh)

    proc = None
    already = port_open()
    if already:
        log("检测到 ZMQ 端口 %d 已在监听：复用当前 CoppeliaSim 实例" % PORT, fh)
    elif reuse:
        log("!! 指定 --no-start 但端口 %d 未监听，退出" % PORT, fh)
        fh.close()
        return 2
    else:
        log("① 启动 CoppeliaSim ...", fh)
        proc = subprocess.Popen([COPPELIA, SCENE])
        log("   PID=%d" % proc.pid, fh)

    log("② 等待 ZMQ Remote API (端口 %d) ..." % PORT, fh)
    if not wait_zmq(fh):
        log("!! ZMQ 未在 90s 内就绪，启动失败", fh)
        fh.close()
        return 3
    log("   ZMQ 就绪", fh)

    calib = os.path.join(HERE, "vision_bus", "calibration.json")
    if not os.path.exists(calib):
        log("③ 缺少相机标定，运行 vision_calibrate.py ...", fh)
        r = subprocess.run([PY, "-u", "vision_calibrate.py"], cwd=HERE,
                           capture_output=True, text=True, encoding="utf-8", errors="replace")
        fh.write(r.stdout or "")
        log("   标定完成（返回码 %d）" % r.returncode, fh)
    else:
        log("③ 相机标定已存在：%s" % os.path.basename(calib), fh)

    log("④ 启动视觉分类抓取主流程 ...", fh)
    cmd = [PY, "-u", "vision_pipeline_recorded.py", view]
    if out and out.lower() != "none":
        cmd.append(out)
    else:
        cmd.append(os.path.join(HERE, "_no_record.mp4"))
    cmd.append(lang)
    log("   命令: %s" % " ".join('"%s"' % c if " " in c else c for c in cmd), fh)
    t0 = time.time()
    r = subprocess.run(cmd, cwd=HERE, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    dur = time.time() - t0
    if r.stdout:
        fh.write(r.stdout)
    if r.stderr:
        fh.write("[stderr]\n" + r.stderr)
    log("⑤ 主流程结束：返回码 %d，用时 %.1f s" % (r.returncode, dur), fh)

    summ = os.path.join(LOGDIR, "summary_%s_%s.json" % (view, lang))
    log("   轨迹/结果/错误日志见 %s" % LOGDIR, fh)
    log("   启动日志: %s" % logf, fh)
    log("=== 启动流程结束 ===", fh)
    fh.close()
    if proc is not None and "--close" in flags:
        log("按要求关闭 CoppeliaSim", None)
        proc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
