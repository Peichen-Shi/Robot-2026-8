# -*- coding: utf-8 -*-
"""
diag_gripper.py
============================================================
最小诊断：Prismatic_joint 为什么不动
============================================================
A. 场景对象清单（顶层 + 全部 joint）
B. 定位 /Prismatic_joint 及其父链，读取关节属性
C. 仿真未运行：直接 setJointPosition，读回
D. 启动仿真：持续 setJointTargetPosition，读回（检测被覆盖）
E. 尝试不同控制方式（position / target / 关闭 motor 等）

只做诊断，不抓取 Grid1。
============================================================
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

HOST = "localhost"
PORT = 23000

client = RemoteAPIClient(host=HOST, port=PORT)
sim = client.require("sim")

def alias_of(h):
    try:
        return sim.getObjectAlias(h, 0)
    except Exception:
        try:
            return sim.getObjectAlias(h, -1)
        except Exception:
            return "?"

def parent_chain(h):
    chain = []
    cur = h
    seen = 0
    while cur >= 0 and seen < 50:
        chain.append(alias_of(cur))
        cur = sim.getObjectParent(cur)
        seen += 1
    chain.reverse()
    if len(chain) > 4:
        return ".../" + "/".join(chain[-4:])
    return "/" + "/".join(chain)

def jtype(h):
    try:
        t = sim.getJointType(h)
        names = {0: "revolute", 1: "prismatic", 2: "screw"}
        return names.get(t, t)
    except Exception as e:
        return "err %s" % e

def safe(fn, default=None):
    try:
        return fn()
    except Exception as e:
        return "ERR(%s)" % e

print("=" * 70)
print("A. 顶层对象 / 全部关节清单")
print("=" * 70)
print("仿真状态:", sim.getSimulationState())

top = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
print("\n顶层对象（scene 直接子级）:")
roots = sim.getObjectsInTree(sim.handle_scene, sim.handle_scene, 0) or []
# 列出所有对象，标记 joint
all_joints = []
for h in top:
    try:
        otype = sim.getObjectType(h)
        alias = alias_of(h)
        if otype == getattr(sim, "object_joint_type", 7):
            all_joints.append(h)
            print("  [JOINT] %-6d %s" % (h, parent_chain(h)))
    except Exception:
        pass

print("\n顶层节点树略去（B 节已含全部关节路径）")

print("\n" + "=" * 70)
print("B. 定位 Prismatic_joint")
print("=" * 70)
targets = {}
for name in ["Prismatic_joint", "prismatic_joint"]:
    try:
        h = sim.getObject("/" + name)
        targets[name] = h
        print("getObject('/%s') -> handle %d" % (name, h))
    except Exception as e:
        print("getObject('/%s') 失败: %s" % (name, e))

# 按 alias 在全部对象里找 prismatic 相关 joint
print("\n全部名称含 'prismatic' 或 'gripper' 的对象:")
for h in top:
    a = alias_of(h)
    al = a.lower()
    if "prismatic" in al or "gripper" in al or "finger" in al:
        try:
            otype = sim.getObjectType(h)
        except Exception:
            otype = -1
        print("  h=%-6d type=%-3d %s" % (h, otype, parent_chain(h)))

# 选择诊断目标：别名等于 Prismatic_joint 的第一个
gripper = None
for name, h in targets.items():
    if h is not None and h >= 0:
        gripper = h
        break
if gripper is None:
    for h in all_joints:
        if alias_of(h) == "Prismatic_joint":
            gripper = h
            break

if gripper is None:
    print("没有找到任何 Prismatic_joint，停止。")
    raise SystemExit(1)

print("\n诊断目标 joint =", gripper)
print("路径          =", parent_chain(gripper))
print("joint type    =", jtype(gripper))

print("\n-- 关节属性 --")
print("position          =", safe(lambda: sim.getJointPosition(gripper)))
for fnname, desc in [
    ("getJointMode", "joint mode(options,mode)"),
    ("getJointInterval", "interval"),
    ("getJointTargetPosition", "target position"),
    ("getJointTargetVelocity", "target velocity"),
    ("getJointVelocity", "velocity"),
]:
    if hasattr(sim, fnname):
        try:
            r = getattr(sim, fnname)(gripper)
            print("%-24s = %s   (%s)" % (fnname, r, desc))
        except Exception as e:
            print("%-24s ERR %s" % (fnname, e))

for constname in ["jointmode_passive", "jointmode_motion", "jointmode_torque",
                  "jointmode_force", "jointmode_ik", "jointmode_dependent"]:
    if hasattr(sim, constname):
        print("  const %-22s = %s" % (constname, getattr(sim, constname)))

# 动力学 / 脚本检查
print("\n-- 父对象与脚本 --")
p = sim.getObjectParent(gripper)
while p >= 0:
    try:
        sc = sim.getScript(getattr(sim, "scripttype_childscript", 1), p)
    except Exception:
        sc = "?"
    print("  parent %-6d %s   childscript=%s" % (p, alias_of(p), sc))
    p = sim.getObjectParent(p)

print("\n" + "=" * 70)
print("C. 仿真未运行：setJointPosition 直接写")
print("=" * 70)
pos0 = safe(lambda: sim.getJointPosition(gripper))
print("写入前 position =", pos0)
for v in [0.05, -0.025, 0.05]:
    try:
        sim.setJointPosition(gripper, v)
    except Exception as e:
        print("setJointPosition(%s) ERR %s" % (v, e))
    time.sleep(0.3)
    print("写 %-8s -> 读回 position = %s" % (v, safe(lambda: sim.getJointPosition(gripper))))

print("\n" + "=" * 70)
print("D. 启动仿真，持续下发 target，检测覆盖")
print("=" * 70)
try:
    sim.setJointPosition(gripper, 0.0)
except Exception:
    pass
try:
    sim.startSimulation()
    print("仿真已启动")
except Exception as e:
    print("startSimulation ERR:", e)

t_end = time.time() + 6
step = 0
while time.time() < t_end:
    target = 0.05 if (step // 3) % 2 == 0 else -0.025
    try:
        sim.setJointTargetPosition(gripper, target)
        sim.setJointPosition(gripper, target)
    except Exception as e:
        print("下发 ERR:", e)
    time.sleep(0.25)
    cur = safe(lambda: sim.getJointPosition(gripper))
    trg = safe(lambda: sim.getJointTargetPosition(gripper))
    print("t=%4.1fs  目标=%8.4f  position=%8.4f  target读回=%s" %
          (time.time() - (t_end - 6), target, cur, trg))
    step += 1

try:
    sim.stopSimulation()
    print("仿真已停止")
except Exception as e:
    print("stopSimulation ERR:", e)

print("\n" + "=" * 70)
print("E. 结束状态")
print("=" * 70)
print("最终 position =", safe(lambda: sim.getJointPosition(gripper)))
print("完成。")
