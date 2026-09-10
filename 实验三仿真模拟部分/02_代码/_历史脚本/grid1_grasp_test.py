# -*- coding: utf-8 -*-
"""
grid1_grasp_test.py
============================================================
RoboMaster EP 用“模型原生自动抓取”验证 Grid1 抓取-搬运-放下
============================================================
原理（基于夹爪控制器脚本源码，脚本在 gripper_link_respondable 上）：
  夹爪 CLOSE 过程中，控制器每步扫描场景里的“非静态 + 可碰撞”形状：
  当接近传感器命中且左右手指都碰到该形状时，执行
  setObjectParent(形状, attachPoint) 自动挂到机械臂上（打印 Attached shape），
  随后夹爪继续收紧，state -> close。

前提/注意：
  1) Grid1 需临时置为 非静态(static=0) + 可碰撞(respondable=1)
     （否则不会触发自动抓取；本场景 Grid1 存盘时是 static=1 respondable=0）
  2) 本轮不移动小车：先把 Grid1 摆到夹爪正前方（量手指实际位置后摆），
     验证“下降->闭合->自动挂接->抬升->移走->张开放下”全链路。
  3) 夹爪开/合走 signal 接口；机械臂走 setJointTargetPosition。

流程：
  A. 启动仿真, home + OPEN
  B. 摆位姿态 s0=+30（降）→ 实测左右手指位置 → Grid1 放到手指中点、Y 对齐、Z=0.11
  C. 把 Grid1 置为动态可碰撞
  D. CLOSE -> 轮询: Grid1 的父对象是否变成 attachPoint（=挂接成功）
  E. 抬升 s0=-25（Grid1 应随臂上升）
  F. 前伸 s1=+35 模拟搬运到旁边
  G. OPEN -> 放下, 记录 Grid1 落点
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

HOST = "localhost"
PORT = 23000

ST_OPEN = 1
ST_CLOSE = 2
GRASP_S0 = 30.0     # 下降姿态
LIFT_S0 = -25.0
CARRY_S1 = 35.0
GRID_Z = 0.11       # Grid1 中心高度（物体高约 0.22m，放在地板上）


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


def pos(h):
    return sim.getObjectPosition(h, sim.handle_world)


def setpos(h, xyz):
    sim.setObjectPosition(h, sim.handle_world, xyz)


s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
grid1 = find("Grid1")
attach = find("attachPoint")
if None in (s0, s1, tip, grp, left5, right5, grid1, attach):
    raise RuntimeError("对象查找失败")

print("attachPoint h=%d  Grid1 h=%d" % (attach, grid1))


def jdeg(h):
    return math.degrees(sim.getJointPosition(h))


def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))


def wait_arm(a0, a1, tol=1.0, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.1)
    return False


def gripper(target, label, timeout=10.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.1)
    return st


def grid_parent_is_attach():
    return sim.getObjectParent(grid1) == attach


def snap(tag):
    p = pos(grid1)
    print("  %-22s Grid1=(%.4f, %.4f, %.4f)  夹爪Z=%.4f  s0=%+.1f s1=%+.1f" %
          (tag, p[0], p[1], p[2],
           pos(tip)[2], jdeg(s0), jdeg(s1)))


print("\n启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

# A. home + OPEN
set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
gripper(ST_OPEN, "open")
print("\n① home + OPEN")

# B. 摆位：下降姿态，实测手指位置，把 Grid1 放到两指中点
set_deg(s0, GRASP_S0); set_deg(s1, 0)
ok = wait_arm(GRASP_S0, 0)
print("② 下降 s0=%d %s" % (GRASP_S0, "到位" if ok else "!!超时"))
time.sleep(0.5)

lp = pos(left5); rp = pos(right5)
mid = [(lp[0] + rp[0]) / 2, (lp[1] + rp[1]) / 2, GRID_Z]
print("   左指=(%.4f,%.4f,%.4f)  右指=(%.4f,%.4f,%.4f)" %
      (lp[0], lp[1], lp[2], rp[0], rp[1], rp[2]))
print("   手指中点=(%.4f, %.4f)  -> 把 Grid1 摆到这里" % (mid[0], mid[1]))
print("   手指 Z 均值=%.4f (夹爪接触带高度)" % ((lp[2] + rp[2]) / 2))

# Grid1 先置可碰撞动态，再放到位（避免和机械臂碰撞爆开）
try:
    sim.setObjectInt32Param(grid1, sim.shapeintparam_static, 0)
    sim.setObjectInt32Param(grid1, sim.shapeintparam_respondable, 1)
    print("   Grid1 -> static=0 respondable=1 (动态可碰撞)")
except Exception as e:
    print("   改 Grid1 属性失败:", e)
setpos(grid1, mid)
time.sleep(1.2)
snap("Grid1 就位")

# C. CLOSE —— 期待自动挂接
print("③ 夹爪 CLOSE（等待自动挂接）...")
st = gripper(ST_CLOSE, "close", timeout=15.0)
attached = grid_parent_is_attach()
print("   state=%s   Grid1父对象==attachPoint? %s" % (st, attached))
if not attached:
    # 试一次微调重试：重新张开，检查是否因手指没包住
    gripper(ST_OPEN, "open")
    time.sleep(1.0)
    lp = pos(left5); rp = pos(right5)
    mid = [(lp[0] + rp[0]) / 2, (lp[1] + rp[1]) / 2, GRID_Z]
    setpos(grid1, mid)
    time.sleep(1.0)
    st = gripper(ST_CLOSE, "close retry", timeout=15.0)
    attached = grid_parent_is_attach()
    print("   [retry] state=%s  attached=%s" % (st, attached))

if not attached:
    print("\n!! 未挂接成功。Grid1 目前位置:")
    snap("失败状态")
    sim.stopSimulation()
    raise SystemExit(2)

print("\n✅ 挂接成功：Grid1 已挂到 attachPoint")

# D. 抬升
set_deg(s0, LIFT_S0); set_deg(s1, 0)
wait_arm(LIFT_S0, 0)
snap("④ 抬升(携带Grid1)")

# E. 前伸搬运
set_deg(s1, CARRY_S1)
wait_arm(LIFT_S0, CARRY_S1)
snap("⑤ 前伸搬运")

# F. 张开放下
gripper(ST_OPEN, "open")
time.sleep(1.5)
snap("⑥ 张开放下")

print("\n完成。")
try:
    sim.stopSimulation()
    print("仿真已停止。")
except Exception:
    pass
