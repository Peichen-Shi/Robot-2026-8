# -*- coding: utf-8 -*-
"""patch_acceptance3.py —— 抓取时物体前移 8mm（腕部避让）+ 安全监控允许接触当前目标"""
import io

P = "vision_pipeline_recorded.py"
s = io.open(P, encoding="utf-8").read()

REP = [
    # 常量：抓取位再退 8mm，让物体在夹爪中略靠前，腕部不碰物体
    ("LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), \"logs\")\n",
     "LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), \"logs\")\n"
     "GRASP_FWD_CLEAR = 0.008      # 抓取位再后退 8mm：物体在夹爪中略靠前，腕部不接触物体\n"),
    # 全局：当前目标
    ("SAFE = None            # 安全监控\n",
     "SAFE = None            # 安全监控\nCURRENT_TARGET = None   # 当前抓取目标（允许夹爪与其接触）\n"),
    # 安全检查允许接触当前目标
    ("        SAFE.set_allow(hold.obj if hold.active else None)\n",
     "        SAFE.set_allow(hold.obj if hold.active else None, CURRENT_TARGET)\n"),
    # 抓取位后退 8mm
    ("        drive_x(VC.GRID_X - off_pick[0])\n",
     "        drive_x(VC.GRID_X - off_pick[0] - GRASP_FWD_CLEAR)\n"),
    # 设置/清除当前目标
    ("        cls = prim[\"class\"]\n        bin_name = VC.CLASS_BIN[cls]\n",
     "        global CURRENT_TARGET\n        CURRENT_TARGET = grid\n        cls = prim[\"class\"]\n        bin_name = VC.CLASS_BIN[cls]\n"),
    ("        arm_to(CARRY_S0, CARRY_S1); arm_to(0, 0)\n        results[\"Grid%d\" % g] = (pos(grid), bin_name, cls, slot_y)",
     "        arm_to(CARRY_S0, CARRY_S1); arm_to(0, 0)\n        CURRENT_TARGET = None\n        results[\"Grid%d\" % g] = (pos(grid), bin_name, cls, slot_y)"),
]

n = 0
for a, b in REP:
    if a in s:
        s = s.replace(a, b, 1)
        n += 1
    else:
        print("未匹配:", a.splitlines()[0][:70])
io.open(P, "w", encoding="utf-8").write(s)
print("阶段三替换：%d / %d" % (n, len(REP)))
