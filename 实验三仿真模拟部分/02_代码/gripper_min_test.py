# -*- coding: utf-8 -*-
"""
gripper_min_test.py
============================================================
RoboMaster EP 夹爪最小真实开合测试（已通过验证）
============================================================
【为什么之前 Python 控制不动？】
场景中 /RoboMaster/.../gripper_link_respondable 上挂着一个
夹爪原厂控制器脚本（CoppeliaSim 模型自带）。它每个仿真步用
sim.setJointTargetVelocity 驱动 Prismatic_joint：
  - Prismatic_joint 是动态关节（电机开、无位置控制环）
  - 因此 sim.setJointPosition / setJointTargetPosition
    一写入就被动力学和该脚本顶回去，位置一直显示 ~0.0007
    （0.0007≈0.001 正是控制器里定义的 open 上限 0.00098）。

【正确控制方式（本文件采用）】
不抢关节，而是给控制器发状态命令。控制器挂在
gripper_link_respondable 上的 int 属性：
    signal.target : 0=pause(停)  1=open(张开)  2=close(闭合)
    signal.state  : 控制器完成动作后写入的当前状态
发命令后轮询 signal.state 即可知道动作是否完成。
注意：控制器依赖模型脚本运行，请保持场景中脚本处于启用状态。

【本测试流程】
1. 连接 CoppeliaSim（ZMQ, 默认端口 23000）
2. 启动仿真
3. OPEN  -> 等待完成
4. CLOSE -> 等待完成
5. OPEN  -> 等待完成
6. 每步打印 Prismatic_joint 位置 + 左右手指间距（验证真实动作）
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

HOST = "localhost"
PORT = 23000

# 状态常量（与夹爪控制器脚本一致）
ST_PAUSE = 0
ST_OPEN = 1
ST_CLOSE = 2

WAIT_TIMEOUT = 8.0   # 单个动作最长等待秒数
POLL_STEP = 0.1


# ============================================================
# 连接
# ============================================================
print()
print("=" * 60)
print("         RoboMaster EP 夹爪最小真实开合测试")
print("=" * 60)

client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.require("sim")
print("连接成功（ZMQ %d）" % PORT)


# ============================================================
# 找对象（按 alias 全场景搜索，不依赖路径层级）
# ============================================================
def find(bare):
    objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    for h in objs:
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None


def need(bare):
    h = find(bare)
    if h is None:
        raise RuntimeError("找不到对象: " + bare)
    return h


pj = sim.getObject("/Prismatic_joint")     # 夹爪主关节
grp = sim.getObjectParent(pj)              # gripper_link_respondable（控制器所在对象）
left5 = need("left_gripper_5_respondable")
right5 = need("right_gripper_5_respondable")

print("Prismatic_joint handle      =", pj)
print("gripper_link_respondable    =", grp)
print("left/right_gripper_5        =", left5, right5)
print()


# ============================================================
# 夹爪工具函数
# ============================================================
def finger_gap():
    """左右夹爪 respondable 质心间距，用于确认真实开合。"""
    lp = sim.getObjectPosition(left5, sim.handle_world)
    rp = sim.getObjectPosition(right5, sim.handle_world)
    return ((lp[0] - rp[0]) ** 2 +
            (lp[1] - rp[1]) ** 2 +
            (lp[2] - rp[2]) ** 2) ** 0.5


def get_state():
    return sim.getIntProperty(grp, "signal.state")


def command_gripper(target, label):
    """发命令并等待控制器完成。"""
    print()
    print("----------------------------------------------")
    print("%s  (signal.target=%d)" % (label, target))
    print("----------------------------------------------")

    sim.setIntProperty(grp, "signal.target", target)

    t0 = time.time()
    while time.time() - t0 < WAIT_TIMEOUT:
        s = get_state()
        if isinstance(s, int) and s == target:
            break
        time.sleep(POLL_STEP)
    else:
        print("!! 超时：控制器未进入目标状态 %d（当前 state=%s）" %
              (target, get_state()))
        raise RuntimeError("gripper timeout")

    pos = sim.getJointPosition(pj)
    gap = finger_gap()
    print("完成：Prismatic_joint = %.5f   手指间距 = %.5f m" % (pos, gap))


# ============================================================
# 主流程
# ============================================================
print("=" * 60)
print("启动仿真 ...")
print("=" * 60)

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)

sim.startSimulation()
time.sleep(0.8)   # 等待所有模型脚本 sysCall_init 完成

# 确认控制器已初始化
state0 = get_state()
print("控制器初始 state =", state0)

print()
print("=" * 60)
print("测试：OPEN -> CLOSE -> OPEN")
print("=" * 60)

command_gripper(ST_OPEN, "STEP 1：夹爪张开 (OPEN)")
command_gripper(ST_CLOSE, "STEP 2：夹爪闭合 (CLOSE)")
command_gripper(ST_OPEN, "STEP 3：再次张开 (OPEN)")

print()
print("=" * 60)
print("结果判断")
print("=" * 60)
print("张开时手指间距应明显大于闭合时（例如 0.10+ vs 0.03-），")
print("并且 Prismatic_joint 应分别接近 +0.001（张）与 -0.023（合）。")
print()

try:
    sim.stopSimulation()
    print("仿真已停止。测试结束。")
except Exception:
    pass
