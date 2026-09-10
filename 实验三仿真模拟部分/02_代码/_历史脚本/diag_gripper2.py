# -*- coding: utf-8 -*-
"""
diag_gripper2.py  —— 第二层诊断
1) 列出 /RoboMaster 树内的子脚本（对象 -> script handle）
2) 读取夹爪链路相关脚本源码，查找谁在写 Prismatic_joint
3) 打印 Prismatic_joint 的动力学参数
4) 仿真运行: 脚本开启 vs 全部关闭 -> 目标开/关，采样 关节位置 + 左右指尖距离
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

def chain(h):
    out = []
    c = h
    while c >= 0 and len(out) < 60:
        out.append(alias_of(c))
        c = sim.getObjectParent(c)
    return "/".join(reversed(out))

print("=" * 72)
print("1. /RoboMaster 树内全部 child script")
print("=" * 72)
allobjs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
robot = None
for h in allobjs:
    if alias_of(h) == "RoboMaster":
        robot = h
        break
print("RoboMaster handle:", robot)

scripts = []  # (obj, scriptHandle)
for h in allobjs:
    try:
        if h == robot or robot is not None and False:
            pass
    except Exception:
        pass
    try:
        # 判断是否在 robot 树下
        c = h
        under = False
        while c >= 0:
            if c == robot:
                under = True
                break
            c = sim.getObjectParent(c)
        if not under:
            continue
    except Exception:
        continue
    try:
        sh = sim.getScript(sim.scripttype_childscript, h)
        if sh is not None and sh >= 0:
            scripts.append((h, sh))
            nm = ""
            try:
                nm = sim.getScriptStringParam(sh, sim.scriptstringparam_nameext) or ""
            except Exception:
                pass
            print("  obj=%-5d %-45s script=%d name=%s" % (h, alias_of(h), sh, nm))
    except Exception:
        pass

print("\n" + "=" * 72)
print("2. 关键脚本源码（找写 Prismatic_joint 的地方）")
print("=" * 72)
KEY = ["Prismatic", "gripper", "Gripper", "servo", "setJointPosition",
       "setJointTargetPosition", "jointintparam", "Cuboid0", "finger"]
for obj, sh in scripts:
    try:
        txt = sim.getScriptStringParam(sh, sim.scriptstringparam_text) or ""
    except Exception as e:
        print("  (read text err %s)" % e)
        continue
    if not txt.strip():
        continue
    hits = [k for k in KEY if k in txt]
    al = alias_of(obj)
    if any(k in al.lower() for k in ("gripper", "script", "endpoint", "arm_2", "rod", "triangle", "robo", "wheel")):
        print("\n----- script on object [%s] handle %d  (关键词: %s)" % (al, obj, hits))
        # 打印含关键行的前后少量上下文
        lines = txt.splitlines()
        for i, ln in enumerate(lines):
            low = ln.lower()
            if any(k.lower() in low for k in ("prismatic", "setjoint", "gripper", "servo_motor",
                                               "jointtarget", "syscall", "jointintparam", "getgripper", "setgripper")):
                lo = max(0, i - 2)
                hi = min(len(lines), i + 3)
                for j in range(lo, hi):
                    print("    %4d | %s" % (j + 1, lines[j]))
                print("    ----")

print("\n" + "=" * 72)
print("3. Prismatic_joint 参数")
print("=" * 72)
pj = sim.getObject("/Prismatic_joint")
print("handle:", pj)
for pname in ["jointintparam_dynctrlmode", "jointintparam_motor_enabled",
              "jointintparam_ctrl_enabled", "jointintparam_velocity_lock"]:
    try:
        print("  %-32s = %s" % (pname, sim.getObjectInt32Param(pj, getattr(sim, pname))))
    except Exception as e:
        print("  %-32s ERR %s" % (pname, e))
try:
    print("  getJointMode      =", sim.getJointMode(pj))
except Exception as e:
    print("  getJointMode ERR", e)
for pname in ["jointfloatparam_upper_limit", "jointfloatparam_velocity",
              "jointfloatparam_maxvel", "jointfloatparam_maxaccel", "jointfloatparam_pid_p",
              "jointfloatparam_pid_i", "jointfloatparam_pid_d", "jointfloatparam_kc_k",
              "jointfloatparam_error_pos"]:
    try:
        print("  %-32s = %s" % (pname, sim.getObjectFloatParam(pj, getattr(sim, pname))))
    except Exception as e:
        print("  %-32s ERR %s" % (pname, e))

# 指尖可视形状句柄（取 left/right_gripper_7_visual 下的 last）
def find_shape(bare):
    for h in allobjs:
        if alias_of(h) == bare:
            return h
    return None

lf = find_shape("left_gripper_7_visual") or find_shape("left_gripper_5_visual")
rf = find_shape("right_gripper_7_visual") or find_shape("right_gripper_5_visual")
print("\n指尖对象: left=%s right=%s" % (lf, rf))

def finger_gap():
    if lf is None or rf is None:
        return None
    lp = sim.getObjectPosition(lf, sim.handle_world)
    rp = sim.getObjectPosition(rf, sim.handle_world)
    return ((lp[0]-rp[0])**2 + (lp[1]-rp[1])**2 + (lp[2]-rp[2])**2) ** 0.5

def sample(tag):
    try:
        p = sim.getJointPosition(pj)
        pstr = "%.5f" % p
    except Exception:
        pstr = "?"
    g = finger_gap()
    print("  %-22s position=%s  finger_gap=%s" % (tag, pstr, ("%.5f" % g if g is not None else "?")))

print("\n" + "=" * 72)
print("4A. 脚本开启：运行仿真，下发开/关")
print("=" * 72)
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.3)
sim.startSimulation()
time.sleep(0.6)
sample("初始(运行中)")
for label, v in [("OPEN 0.024", 0.024), ("CLOSE -0.023", -0.023), ("OPEN 0.024", 0.024)]:
    sim.setJointTargetPosition(pj, v)
    sim.setJointPosition(pj, v)
    time.sleep(2.0)
    sample(label)
sim.stopSimulation()
time.sleep(0.5)

print("\n" + "=" * 72)
print("4B. 关闭 /RoboMaster 树内全部 child script")
print("=" * 72)
for obj, sh in scripts:
    try:
        sim.setScriptInt32Param(sh, sim.scriptintparam_enabled, 0)
        print("  disabled script %d on %s" % (sh, alias_of(obj)))
    except Exception as e:
        print("  disable %d ERR %s" % (sh, e))

print("\n-- 脚本关闭后：运行仿真，下发开/关 --")
sim.startSimulation()
time.sleep(0.6)
sample("初始(运行中,无脚本)")
for label, v in [("OPEN 0.024", 0.024), ("CLOSE -0.023", -0.023), ("OPEN 0.024", 0.024), ("CLOSE -0.023", -0.023)]:
    sim.setJointTargetPosition(pj, v)
    sim.setJointPosition(pj, v)
    time.sleep(2.0)
    sample(label)
sim.stopSimulation()
time.sleep(0.5)

print("\n-- 恢复脚本 --")
for obj, sh in scripts:
    try:
        sim.setScriptInt32Param(sh, sim.scriptintparam_enabled, 1)
    except Exception:
        pass
print("done")
