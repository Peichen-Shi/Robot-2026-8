# -*- coding: utf-8 -*-
"""
diag_gripper3.py  —— 用模型原生接口真实开合夹爪
接口：gripper_link_respondable 的 int 属性
    signal.target: 0=pause 1=open 2=close
    signal.state : 当前状态（控制器完成动作后写入）
通过轮询 signal.state 确认动作完成；同时测量指尖形状间距验证真实运动。
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient(host="localhost", port=23000)
sim = client.require("sim")

def alias_of(h):
    try:
        return sim.getObjectAlias(h, 0)
    except Exception:
        return "?"

def find(bare):
    objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    for h in objs:
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None

ROBOT = find("RoboMaster")
PJ = sim.getObject("/Prismatic_joint")  # 唯一别名
GRP = sim.getObjectParent(PJ)           # gripper_link_respondable

LEFT5 = find("left_gripper_5_respondable")
RIGHT5 = find("right_gripper_5_respondable")
LEFTTIP = find("left_gripper_7_visual")
RIGHTTIP = find("right_gripper_7_visual")

print("PJ=%d  GRP=%d(%s)  left5=%s right5=%s tipL=%s tipR=%s" %
      (PJ, GRP, alias_of(GRP), LEFT5, RIGHT5, LEFTTIP, RIGHTTIP))

def dist(a, b):
    if a is None or b is None:
        return None
    pa = sim.getObjectPosition(a, sim.handle_world)
    pb = sim.getObjectPosition(b, sim.handle_world)
    return ((pa[0]-pb[0])**2 + (pa[1]-pb[1])**2 + (pa[2]-pb[2])**2) ** 0.5

def get_state():
    try:
        return sim.getIntProperty(GRP, "signal.state")
    except Exception as e:
        return "err:" + str(e)

def get_target():
    try:
        return sim.getIntProperty(GRP, "signal.target")
    except Exception:
        return "?"

def snap(tag):
    p = sim.getJointPosition(PJ)
    print("  %-28s joint=%.5f  state=%s target=%s  gapL5R5=%s  gapTip=%s" %
          (tag, p, get_state(), get_target(),
           ("%.5f" % dist(LEFT5, RIGHT5) if dist(LEFT5, RIGHT5) is not None else "?"),
           ("%.5f" % dist(LEFTTIP, RIGHTTIP) if dist(LEFTTIP, RIGHTTIP) is not None else "?")))

def cmd(target, wait_s=6.0, label=""):
    sim.setIntProperty(GRP, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < wait_s:
        s = get_state()
        if isinstance(s, int) and s == target:
            break
        time.sleep(0.1)
    snap(label + " (state=%s)" % get_state())

print("\n== 停止仿真，脚本应全部启用 ==")
# 确认夹爪控制器脚本可用状态（仅打印）
try:
    print("simulation state:", sim.getSimulationState())
except Exception as e:
    print(e)

print("\n== 启动仿真 ==")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(0.8)   # 等脚本 sysCall_init 跑完（create_ep + gripper init）

snap("启动后初始")

print("\n== STEP 1: 命令 OPEN (target=1) ==")
cmd(1, label="open 完成")

print("\n== STEP 2: 命令 CLOSE (target=2) ==")
cmd(2, label="close 完成")

print("\n== STEP 3: 再命令 OPEN (target=1) ==")
cmd(1, label="再 open 完成")

print("\n== 结束，停止仿真 ==")
sim.stopSimulation()
print("done")
