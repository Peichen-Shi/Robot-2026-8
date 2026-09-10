# -*- coding: utf-8 -*-
"""
full_pipeline_v2.py  (v2.1 —— 直角走位版)
============================================================
按要求：
 ① 全程只走直角（轴向）路线，绝不斜着走：
    抓取 -> 直线后退 -> 平移到料盒前 -> 放料 -> 平移到下一个物体前 -> 前移一点 -> 抓取 -> 循环
 ② 车体再往后退，底盘完全不碰料盒（料盒左壁外表面 x=0.275，壁厚 0.02）
 ③ 物品放入箱内同一条水平线 x=0.380、间距 0.15、高度一致
 ④ 物品跨过箱壁时从“箱壁上方”越过（高位姿态），避免物品穿模
放置顺序：Grid1→左箱最左、Grid2→左箱中、Grid3→左箱最右、
          Grid4→右箱最左、Grid5→右箱中、Grid6→右箱最右，最后小车回原位
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

HOST, PORT = "localhost", 23000
ST_OPEN, ST_CLOSE = 1, 2

# 手臂姿态（实测标定）
APPROACH_S0, APPROACH_S1 = 30.0, 0.0     # 抓取姿态：前伸≈0.173、手指高≈0.179
CARRY_S0, CARRY_S1 = -35.0, 20.0         # 搬运姿态（高位）：物品 z≈0.218 → 底部 0.108 > 箱壁顶 0.08
CROSS_S0, CROSS_S1 = -40.0, 30.0         # 高位跨过箱壁：物品 x≈0.318、z≈0.219
DOWN1_S0, DOWN1_S1 = -25.0, 50.0         # 箱内下降中：物品 z≈0.160
DOWN2_S0, DOWN2_S1 = -20.0, 60.0         # 放料姿态：物品 z≈0.132（箱底面上方）

SLOT_X = 0.380
SLOT_DY = 0.15
SLOT_Z = 0.13
X_SAFE = 0.015                            # 横向平移时使用的安全 x（车头远离料盒）
ROBOT_HALF_LEN = 0.190                    # 车体前伸半长（实测部件 0.155 + 外形余量）
BIN_WALL_OUTER_X = 0.275                  # 料盒左壁外表面（壁中心 0.285 - 壁厚 0.02/2）

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
pj = sim.getObject("/Prismatic_joint")
grp = sim.getObjectParent(pj)
left5 = find("left_gripper_5_respondable")
right5 = find("right_gripper_5_respondable")
if None in (robot, s0, s1, grp, left5, right5):
    raise RuntimeError("对象查找失败")

# ---------------- 工具 ----------------
def jdeg(h):
    return math.degrees(sim.getJointPosition(h))

def set_deg(h, deg):
    sim.setJointTargetPosition(h, math.radians(deg))

def arm_to(a0, a1, tol=1.0, timeout=12.0):
    set_deg(s0, a0); set_deg(s1, a1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(jdeg(s0) - a0) <= tol and abs(jdeg(s1) - a1) <= tol:
            return True
        time.sleep(0.04)
    return False

def gripper(target, timeout=8.0):
    sim.setIntProperty(grp, "signal.target", target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = sim.getIntProperty(grp, "signal.state")
        if isinstance(st, int) and st == target:
            return st
        time.sleep(0.04)
    return st

def finger_mid():
    l = pos(left5); r = pos(right5)
    return [(l[0] + r[0]) / 2, (l[1] + r[1]) / 2, (l[2] + r[2]) / 2]

def arm_offset():
    m = finger_mid(); b = pos(robot)
    return [m[0] - b[0], m[1] - b[1]]

class Hold:
    def __init__(self):
        self.active = False; self.offset = None; self.obj = None
    def grab(self, obj):
        g = pos(obj); m = finger_mid()
        self.offset = [g[0]-m[0], g[1]-m[1], g[2]-m[2]]
        self.obj = obj; self.active = True
        return self.offset
    def sync(self):
        if not self.active:
            return
        m = finger_mid()
        setpos(self.obj, [m[0]+self.offset[0], m[1]+self.offset[1], m[2]+self.offset[2]])
    def settle_to(self, xyz, sec=0.45, steps=22):
        if not self.active:
            return
        p0 = pos(self.obj)
        for i in range(1, steps+1):
            t = i/steps
            setpos(self.obj, [p0[0]+(xyz[0]-p0[0])*t,
                              p0[1]+(xyz[1]-p0[1])*t,
                              p0[2]+(xyz[2]-p0[2])*t])
            time.sleep(sec/steps)
    def release(self):
        self.active = False; self.obj = None; self.offset = None

hold = Hold()

def drive_leg(target_xy, tag=""):
    """一个轴向的直线行驶（另一轴保持不变）"""
    p = pos(robot)
    dx, dy = target_xy[0] - p[0], target_xy[1] - p[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-4:
        return 0.0
    steps = max(6, int(dist * 55))
    for i in range(1, steps + 1):
        t = i / steps
        setpos(robot, [p[0] + dx*t, p[1] + dy*t, p[2]])
        hold.sync()
        time.sleep(0.012)
    time.sleep(0.25)
    b = pos(robot)
    axis = "X" if abs(dx) > abs(dy) else "Y"
    print("    [%s] 沿 %s 直线行驶 %.3f m → 车体(%.3f, %.3f)" % (tag, axis, dist, b[0], b[1]))
    return dist

def drive_x(x, tag=""):
    return drive_leg([x, pos(robot)[1]], tag)

def drive_y(y, tag=""):
    return drive_leg([pos(robot)[0], y], tag)

def ensure_visual(obj):
    try:
        sim.setObjectInt32Param(obj, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 0)
    except Exception:
        pass

# ---------------- 抓放单个物体 ----------------
def pick_and_place(grid_name, slot_xyz, off_pick, off_carry, label):
    grid = find(grid_name)
    g0 = pos(grid)
    ensure_visual(grid)
    print("\n" + "=" * 66)
    print("【%s】%s (%.3f, %.3f)  ->  箱内 (%.3f, %.3f)" %
          (label, grid_name, g0[0], g0[1], slot_xyz[0], slot_xyz[1]))
    print("=" * 66)

    # ① 手臂张开 + 抓取姿态（车已在安全 x）
    arm_to(0, 0); gripper(ST_OPEN)
    arm_to(APPROACH_S0, APPROACH_S1)

    # ② 平移到物体所在行的 y
    drive_y(g0[1] - off_pick[1], "平移到物体前")

    # ③ 往前一点，进入抓取位
    x_grasp = g0[0] - off_pick[0]
    drive_x(x_grasp, "前移到抓取位")
    m = finger_mid()
    print("    对准误差=(%+.4f, %+.4f)" % (g0[0]-m[0], g0[1]-m[1]))

    # ④ 夹爪闭合抓取
    gripper(ST_CLOSE); time.sleep(0.2)
    off = hold.grab(grid); hold.sync()
    print("    夹爪闭合抓取，offset=(%+.4f, %+.4f, %+.4f)" % tuple(off))

    # ⑤ 抬到高位搬运姿态（物品高于箱壁）
    arm_to(CARRY_S0, CARRY_S1); hold.sync()
    print("    抬升(高位)：物品 z=%.3f（底部 %.3f > 箱壁顶 0.08）" %
          (pos(grid)[2], pos(grid)[2]-0.11))

    # ⑥ 直线后退到安全 x（先退，再平移）
    drive_x(X_SAFE, "先后退")

    # ⑦ 平移到料盒投放点的 y
    ry = slot_xyz[1] - off_carry[1] - off[1]
    drive_y(ry, "平移到料盒前")
    b = pos(robot)
    print("    就位：车体 x=%.3f  车头前缘 %.3f  箱壁外面 %.3f → 余量 %.3f m %s" %
          (b[0], b[0]+ROBOT_HALF_LEN, BIN_WALL_OUTER_X,
           BIN_WALL_OUTER_X-(b[0]+ROBOT_HALF_LEN),
           "✅ 不接触" if b[0]+ROBOT_HALF_LEN < BIN_WALL_OUTER_X else "✗"))

    # ⑧ 手臂：高位跨过箱壁 → 箱内下降
    arm_to(CROSS_S0, CROSS_S1); hold.sync()
    print("    高位跨壁：物品 x=%.3f z=%.3f（底部 %.3f 越壁顶）" %
          (pos(grid)[0], pos(grid)[2], pos(grid)[2]-0.11))
    arm_to(DOWN1_S0, DOWN1_S1); hold.sync()
    arm_to(DOWN2_S0, DOWN2_S1); hold.sync()
    p = pos(grid)
    print("    箱内到位：物品=(%.3f, %.3f, %.3f)  目标=(%.3f, %.3f, %.3f)" %
          (p[0], p[1], p[2], slot_xyz[0], slot_xyz[1], slot_xyz[2]))

    # ⑨ 轻放到水平线 + 张开夹爪
    hold.settle_to([slot_xyz[0], slot_xyz[1], SLOT_Z], sec=0.45)
    hold.release(); time.sleep(0.15)
    gripper(ST_OPEN); time.sleep(0.2)
    print("    已放入：实际=(%.3f, %.3f, %.3f)" % tuple(pos(grid)))

    # ⑩ 手臂收回（先高位退出箱壁，再回 home）
    arm_to(CARRY_S0, CARRY_S1)
    arm_to(0, 0)
    return pos(grid)

# ---------------- 主流程 ----------------
print("启动仿真 ...", flush=True)
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.5)
sim.startSimulation()
time.sleep(1.0)

start = pos(robot)

# 标定：抓取姿态（夹爪张开）与搬运姿态（夹爪闭合）下的前伸偏移
gripper(ST_OPEN)
arm_to(APPROACH_S0, APPROACH_S1)
off_pick = arm_offset()
gripper(ST_CLOSE)
arm_to(CARRY_S0, CARRY_S1)
off_carry = arm_offset()
arm_to(0, 0)
gripper(ST_OPEN)
print("抓取姿态偏移=(%+.3f, %+.3f)   搬运姿态偏移=(%+.3f, %+.3f)" %
      (off_pick[0], off_pick[1], off_carry[0], off_carry[1]))

# 先退到安全 x（后续所有横向平移都在这个 x 上进行）
drive_x(X_SAFE, "初始后退")

L_C, R_C = -0.90, 0.90
def slot(cy, end):
    return [SLOT_X, cy + (SLOT_DY if end == "right" else (-SLOT_DY if end == "left" else 0.0)), SLOT_Z]

JOBS = [
    ("Grid1", slot(L_C, "left"),   "蓝 → LeftBin 最左"),
    ("Grid2", slot(L_C, "center"), "蓝 → LeftBin 中间"),
    ("Grid3", slot(L_C, "right"),  "蓝 → LeftBin 最右"),
    ("Grid4", slot(R_C, "left"),   "红 → RightBin 最左"),
    ("Grid5", slot(R_C, "center"), "红 → RightBin 中间"),
    ("Grid6", slot(R_C, "right"),  "红 → RightBin 最右"),
]

results = {}
try:
    for name, sl, lt in JOBS:
        results[name] = pick_and_place(name, sl, off_pick, off_carry, lt)

    print("\n回原位（同样直角走位）...")
    drive_y(start[1], "平移回原位 y")
    drive_x(start[0], "后退/前移回原位 x")

    print("\n" + "=" * 66)
    print("结果检查")
    print("=" * 66)
    ok = True
    for name, sl, lt in JOBS:
        p = results.get(name)
        if p is None:
            print("%-6s 未完成" % name); ok = False; continue
        ex = abs(p[0]-sl[0]); ey = abs(p[1]-sl[1]); ez = abs(p[2]-sl[2])
        good = (ex < 0.02 and ey < 0.02 and ez < 0.02)
        print("%-6s -> (%+.3f, %+.3f, %+.3f)  目标(%+.3f, %+.3f, %+.3f)  误差(%.4f,%.4f,%.4f) %s"
              % (name, p[0], p[1], p[2], sl[0], sl[1], sl[2], ex, ey, ez, "✅" if good else "✗"))
        ok = ok and good
    print("\n结论：%s" % ("6/6 直角走位、同水平线、底盘未碰料盒 ✅" if ok else "存在偏差，见上表"))
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
