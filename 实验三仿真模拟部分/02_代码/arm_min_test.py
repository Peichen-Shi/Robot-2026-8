# -*- coding: utf-8 -*-
"""
arm_min_test.py
============================================================
RoboMaster EP 机械臂最小升降测试（本场景实测版）
============================================================
【本场景结论（实测，与 Lua 参考场景不同！）】
- servo_motor_0 (s0) 是主升降：负角 -> 抬升，正角 -> 下降
    实测：home(Z=0.209)  s0=-30° -> Z≈0.269(抬)   s0=+30° -> Z≈0.163(降)
- servo_motor_1 (s1) 前伸/俯仰微调：正角前伸并略降，区间约 [-15.7°, +95°]
- 两个关节都是 电机+位置控制环(crtl_enabled=1)，直接 setJointTargetPosition 即可，
  不需要像夹爪那样走信号接口。
- 关节有物理区间限制，超界会被截断：s1 < -15.7° 无效果。
【流程】
1. 连接并启动仿真
2. 回到 home (s0=0, s1=0)
3. 抬升：s0=-25°（夹爪 Z 应明显上升）
4. 下降：s0=+30°（夹爪 Z 应明显下降）
5. 回到 home
每步“等待关节到达目标”后打印 关节角 + 夹爪(Z 用于判断真实动作)。
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

HOST = "localhost"
PORT = 23000

ARM_LIFT_DEG = -25.0   # 抬升（本场景 s0 负角抬升）
ARM_LOWER_DEG = 30.0   # 下降
SETTLE_TOL_DEG = 1.0   # 到位容差
SETTLE_WAIT = 10.0     # 最长等待


# ============================================================
# 连接 & 找对象
# ============================================================
client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.require("sim")
print("连接成功（ZMQ %d）" % PORT)


def find(bare):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == bare:
                return h
        except Exception:
            pass
    return None


s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")
if s0 is None or s1 is None or tip is None:
    raise RuntimeError("找不到伺服关节/夹爪")

print("servo_motor_0=%d  servo_motor_1=%d  夹爪=%d" % (s0, s1, tip))


# ============================================================
# 工具
# ============================================================
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))


def tip_z():
    return sim.getObjectPosition(tip, sim.handle_world)[2]


def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))


def wait_settle(a0_deg, a1_deg, tol=SETTLE_TOL_DEG, timeout=SETTLE_WAIT):
    """等待两个关节都到目标附近。返回是否到位。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        e0 = abs(jdeg(s0) - a0_deg)
        e1 = abs(jdeg(s1) - a1_deg)
        if e0 <= tol and e1 <= tol:
            return True
        time.sleep(0.1)
    return False


def go(a0, a1, label):
    set_deg(s0, a0)
    set_deg(s1, a1)
    ok = wait_settle(a0, a1)
    print("  %-18s s0=%+7.2f  s1=%+7.2f  夹爪Z=%.4f m   %s" %
          (label, jdeg(s0), jdeg(s1), tip_z(),
           "到位" if ok else "!!超时未到位"))


# ============================================================
# 主流程
# ============================================================
print("\n启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

print("\n" + "=" * 60)
print("测试：home -> 抬升 -> 下降 -> home")
print("=" * 60)
go(0.0, 0.0, "home")
go(ARM_LIFT_DEG, 0.0, "抬升 s0=%d" % ARM_LIFT_DEG)
go(ARM_LOWER_DEG, 0.0, "下降 s0=%d" % ARM_LOWER_DEG)
go(0.0, 0.0, "回 home")

z_home = tip_z()
print()
print("=" * 60)
print("结果：夹爪 Z 从 %.3f 抬升/下降应有明显变化（>0.03 m 即真实动作）" % z_home)
print("=" * 60)

try:
    sim.stopSimulation()
    print("仿真已停止。测试结束。")
except Exception:
    pass
