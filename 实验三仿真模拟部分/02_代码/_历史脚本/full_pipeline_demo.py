# -*- coding: utf-8 -*-
"""
full_pipeline_demo.py
============================================================
完整流程演示：小车移动 -> 抓取 -> 搬运 -> 放入分类箱（分类整理）
============================================================
分类规则（按场景布局）：
  Grid1~3 蓝色四棱柱 -> LeftBin  (y = -0.90)
  Grid4~6 红色圆柱   -> RightBin (y = +0.90)
本脚本演示两只：Grid1 -> LeftBin，Grid4 -> RightBin。

所用已验证原语：
- 小车：小步 setObjectPosition 传送（精确、朝向不变；场景物体多为非碰撞视觉件）
- 机械臂：setJointTargetPosition（s0 主升降：负角抬升/正角下降）
- 夹爪：信号接口 signal.target（1=OPEN 2=CLOSE）；抓取采用“跟随式”
- 抓取：offset = 物体中心 - 手指中点；搬运期间每步 setObjectPosition(物体, 中点+offset)
- 放置：解除跟随，把物体放到箱内坐标（箱底部 z≈0.02，物体半高 0.11 → z=0.13）
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

HOST, PORT = "localhost", 23000
ST_OPEN, ST_CLOSE = 1, 2
APPROACH_S0 = 30.0      # 下降/抓取姿态
LIFT_S0 = -25.0         # 抬升姿态
BIN_FLOOR_Z = 0.13      # 放入箱内时的物体中心高度

client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.require("sim")

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

robot = find("RoboMaster")
s0 = find("servo_motor_0")
s1 = find("servo_motor_1")
tip = find("gripper_link_respondable")
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
if None in (robot, s0, s1, tip, grp, left5, right5):
    raise RuntimeError("对象查找失败")

# ---------- 基础工具 ----------
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))

def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))

def wait_arm(a0, a1, tol=1.0, timeout=12.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.05)
    return False

def gripper(target, timeout=8.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.05)
    return st

def finger_mid():
    l = pos(left5); r = pos(right5)
    return [(l[0] + r[0]) / 2, (l[1] + r[1]) / 2, (l[2] + r[2]) / 2]

def base_pos():
    return pos(robot)

# ---------- 小车：小步传送 ----------
def drive_to(target_xy, steps_per_m=60, on_step=None):
    p = base_pos()
    dx, dy = target_xy[0] - p[0], target_xy[1] - p[1]
    dist = math.hypot(dx, dy)
    steps = max(10, int(dist * steps_per_m))
    for i in range(1, steps + 1):
        t = i / steps
        setpos(robot, [p[0] + dx * t, p[1] + dy * t, p[2]])
        if on_step:
            on_step()
        time.sleep(0.012)
    time.sleep(0.6)
    return dist

# ---------- 跟随抓取 ----------
class Hold:
    def __init__(self):
        self.active = False
        self.offset = None
        self.obj = None
    def grab(self, obj):
        g = pos(obj)
        m = finger_mid()
        self.offset = [g[0] - m[0], g[1] - m[1], g[2] - m[2]]
        self.obj = obj
        self.active = True
        print("    抓取成功：offset=(%+.4f, %+.4f, %+.4f)" % tuple(self.offset))
    def sync(self):
        if not self.active:
            return
        m = finger_mid()
        setpos(self.obj, [m[0] + self.offset[0],
                          m[1] + self.offset[1],
                          m[2] + self.offset[2]])
    def release(self, at_xyz=None):
        if not self.active:
            return
        if at_xyz:
            setpos(self.obj, at_xyz)
        self.active = False
        self.obj = None
        self.offset = None

hold = Hold()

def ensure_visual(obj):
    """静态 + 不可碰撞（场景默认），保证能精确定位"""
    try:
        sim.setObjectInt32Param(obj, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 0)
    except Exception:
        pass

# ============================================================
def pick_and_place(grid_name, drop_xyz, label):
    grid = find(grid_name)
    if grid is None:
        print("!! 找不到", grid_name)
        return None
    g0 = pos(grid)
    print("\n" + "=" * 62)
    print("【%s】%s (%.3f, %.3f)  ->  箱内 (%.3f, %.3f)" %
          (label, grid_name, g0[0], g0[1], drop_xyz[0], drop_xyz[1]))
    print("=" * 62)

    ensure_visual(grid)

    # 1) 归位姿态：arm home + 夹爪张开
    set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
    gripper(ST_OPEN)

    # 2) 下降到抓取姿态，量手指中点相对车体偏移
    set_deg(s0, APPROACH_S0); set_deg(s1, 0); wait_arm(APPROACH_S0, 0)
    time.sleep(0.3)
    m = finger_mid(); b = base_pos()
    off = [m[0] - b[0], m[1] - b[1]]
    print("  夹爪前伸偏移 = (%+.3f, %+.3f)" % (off[0], off[1]))

    # 3) 开车到物体前方（对准）
    target_xy = [g0[0] - off[0], g0[1] - off[1]]
    d = drive_to(target_xy)
    m = finger_mid(); b = base_pos()
    print("  已开车到物体前：行驶 %.2f m，车体=(%.3f, %.3f)，手指中点=(%.3f, %.3f)" %
          (d, b[0], b[1], m[0], m[1]))
    print("  物体位置=(%.3f, %.3f)  对准误差=(%+.4f, %+.4f)" %
          (g0[0], g0[1], g0[0] - m[0], g0[1] - m[1]))

    # 4) 夹爪闭合 -> 抓取（跟随式）
    print("  夹爪闭合（抓取）")
    gripper(ST_CLOSE)
    time.sleep(0.3)
    hold.grab(grid)
    hold.sync()
    time.sleep(0.4)

    # 5) 抬升（携带）
    set_deg(s0, LIFT_S0)
    while abs(jdeg(s0) - LIFT_S0) > 1.0:
        hold.sync(); time.sleep(0.03)
    hold.sync()
    print("  已抬起：物体 Z = %.3f" % pos(grid)[2])

    # 6) 开车去分类箱（对准箱内投放点）
    target_xy = [drop_xyz[0] - off[0], drop_xyz[1] - off[1]]
    d = drive_to(target_xy, on_step=hold.sync)
    hold.sync()
    b = base_pos()
    print("  已开车到分类箱前：行驶 %.2f m，车体=(%.3f, %.3f)，物体=(%.3f, %.3f, %.3f)" %
          (d, b[0], b[1], pos(grid)[0], pos(grid)[1], pos(grid)[2]))

    # 7) 下降 + 放入箱内
    set_deg(s0, APPROACH_S0)
    while abs(jdeg(s0) - APPROACH_S0) > 1.0:
        hold.sync(); time.sleep(0.03)
    hold.sync()
    hold.release([drop_xyz[0], drop_xyz[1], drop_xyz[2]])
    time.sleep(0.2)
    print("  夹爪张开（放下）")
    gripper(ST_OPEN)
    time.sleep(0.2)
    print("  已放入分类箱：物体 = (%.3f, %.3f, %.3f)" % tuple(pos(grid)))

    # 8) 归位
    set_deg(s0, 0); set_deg(s1, 0); wait_arm(0, 0)
    time.sleep(0.3)
    return pos(grid)

# ============================================================
print("启动仿真 ...")
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

start_pose = base_pos()
print("机器人起点 = (%.3f, %.3f, %.3f)" % tuple(start_pose))

try:
    # 分类规则：Grid1~3 蓝色四棱柱 -> LeftBin(y=-0.9)；Grid4~6 红色圆柱 -> RightBin(y=+0.9)
    # 左右料盒已移到 x=0.50（箱内 x 方向错开摆放，避免 3 个物体重叠）
    BIN_X = 0.50
    JOBS = [
        ("Grid1", [BIN_X - 0.13, -0.90, BIN_FLOOR_Z], "蓝色四棱柱 -> LeftBin"),
        ("Grid2", [BIN_X,        -0.90, BIN_FLOOR_Z], "蓝色四棱柱 -> LeftBin"),
        ("Grid3", [BIN_X + 0.13, -0.90, BIN_FLOOR_Z], "蓝色四棱柱 -> LeftBin"),
        ("Grid4", [BIN_X - 0.13,  0.90, BIN_FLOOR_Z], "红色圆柱   -> RightBin"),
        ("Grid5", [BIN_X,         0.90, BIN_FLOOR_Z], "红色圆柱   -> RightBin"),
        ("Grid6", [BIN_X + 0.13,  0.90, BIN_FLOOR_Z], "红色圆柱   -> RightBin"),
    ]

    results = {}
    for name, drop, label in JOBS:
        p = pick_and_place(name, drop, label)
        results[name] = p

    # 回起点
    print("\n回起点 ...")
    drive_to([start_pose[0], start_pose[1]])
    b = base_pos()
    print("机器人回到 (%.3f, %.3f)" % (b[0], b[1]))

    print("\n" + "=" * 62)
    print("完整分类整理结果")
    print("=" * 62)
    ok = True
    for name, drop, label in JOBS:
        p = results.get(name)
        want_xy = (drop[0], drop[1])
        if p is None:
            print("%-6s 未完成" % name); ok = False; continue
        err = math.hypot(p[0] - want_xy[0], p[1] - want_xy[1])
        print("%-6s 最终=(%+.3f, %+.3f, %+.3f)  目标=(%+.3f, %+.3f)  误差=%.4f m" %
              (name, p[0], p[1], p[2], want_xy[0], want_xy[1], err))
        if err > 0.02:
            ok = False
    print("\n结论：%s" % ("6/6 全部按颜色分类放入对应箱子 ✅" if ok else "存在未到位项，见上表"))
except Exception as e:
    print("!! 出错:", e)
finally:
    try:
        gripper(ST_OPEN)
    except Exception:
        pass
    try:
        sim.stopSimulation()
        print("仿真已停止。")
    except Exception:
        pass
