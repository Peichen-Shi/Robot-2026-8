# -*- coding: utf-8 -*-
"""patch_acceptance.py —— 把 日志 / 安全监控 / 不可达处理 接入 vision_pipeline_recorded.py"""
import io

P = "vision_pipeline_recorded.py"
s = io.open(P, encoding="utf-8").read()

REP = [
    # 1) 导入
    ("import gripper_camera as GC\n",
     "import gripper_camera as GC\nfrom sim_logger import SimLogger\nfrom safety_monitor import SafetyMonitor\n"),
    # 2) 常量：日志目录
    ("FPS = 12.0\n",
     "LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), \"logs\")\nFPS = 12.0\n"),
    # 3) 全局句柄
    ("rec_cam = None\ngcam = None            # 夹爪摄像头句柄（在抓取姿态下创建）",
     "rec_cam = None\nLOG = None             # 轨迹/结果/错误日志\nSAFE = None            # 安全监控\nSCENE_OBJS = []        # 场景中的物体（用于碰撞检查）\ngcam = None            # 夹爪摄像头句柄（在抓取姿态下创建）"),
    # 4) 轨迹记录（在 capture 里节流记录）
    ("    writer.write(_draw_frame(frame))\n    state[\"frames\"] += 1\n",
     "    writer.write(_draw_frame(frame))\n    state[\"frames\"] += 1\n    _log_traj()\n"),
    # 5) 新增：轨迹记录 + 阶段安全检查
    ("def label(text, sub=None):\n    state[\"label\"] = text\n    if sub is not None:\n        state[\"sub\"] = sub\n    capture(force=True)\n",
     "def label(text, sub=None):\n    state[\"label\"] = text\n    if sub is not None:\n        state[\"sub\"] = sub\n    capture(force=True)\n    phase_check(text)\n\n\n_last_traj = [0.0]\n\n\ndef _log_traj():\n    \"\"\"记录机械臂轨迹（约 10Hz）\"\"\"\n    if LOG is None:\n        return\n    now = time.time()\n    if now - _last_traj[0] < 0.1:\n        return\n    _last_traj[0] = now\n    try:\n        b = pos(robot)\n        LOG.traj(sim.getSimulationTime(), state[\"label\"][:28],\n                 jdeg(s0), jdeg(s1), sim.getJointPosition(pj),\n                 sim.getIntProperty(grp, \"signal.state\"), (b[0], b[1]),\n                 tuple(pos(hold.obj)) if hold.obj is not None else None)\n    except Exception:\n        pass\n\n\ndef phase_check(phase):\n    \"\"\"每个阶段做一次安全检查（关节限位 / 碰撞），记录违规\"\"\"\n    if SAFE is None or LOG is None:\n        return\n    try:\n        SAFE.set_allow(hold.obj if hold.active else None)\n        for kind, detail in SAFE.check(SCENE_OBJS, phase):\n            LOG.violation(kind, \"%s @ %s\" % (detail, phase))\n            print(\"    [SAFETY] %s: %s\" % (kind, detail), flush=True)\n    except Exception:\n        pass\n"),
    # 6) 初始化 LOG / SAFE（在创建录制相机之后）
    ("rec_cam = make_record_camera(sim)\n",
     "rec_cam = make_record_camera(sim)\nSCENE_OBJS = [h for h in (find(\"Grid%d\" % i) for i in range(1, VC.GRID_N + 1)) if h is not None]\nLOG = SimLogger(LOGDIR, \"%s_%s\" % (VIEW, LANG))\nSAFE = SafetyMonitor(sim, find)\nprint(\"日志: %s\\n安全监控: 关节限位 + 碰撞检查（桌面/非目标物/自碰撞）\" % LOGDIR, flush=True)\n"),
]

n = 0
for a, b in REP:
    if a in s:
        s = s.replace(a, b, 1)
        n += 1
    else:
        print("未匹配:", a.splitlines()[0][:70])
io.open(P, "w", encoding="utf-8").write(s)
print("阶段一替换：%d / %d" % (n, len(REP)))
