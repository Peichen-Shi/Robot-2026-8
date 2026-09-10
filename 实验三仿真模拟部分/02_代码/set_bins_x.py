# -*- coding: utf-8 -*-
"""
set_bins_x.py —— 把左右两个料盒整体移到 x = 0.50，并保存场景
- 料盒由 5 个零件组成：*_bottom / _front / _back / _left / _right
- 整体平移，保持形状；料盒中心 x 变为 0.50
- 保存前会删除临时相机 demo_cam，并确认仿真已停止
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

NEW_CENTER_X = 0.50
SCENE = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\second_imitation.ttt"

client = RemoteAPIClient(host="localhost", port=23000)
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

PARTS = ["bottom", "front", "back", "left", "right"]

try:
    sim.stopSimulation()
    time.sleep(0.4)
except Exception:
    pass

# 删除临时相机（避免被存进场景）
cam = find("demo_cam")
if cam is not None:
    try:
        sim.removeObjects([cam])
        print("已删除临时相机 demo_cam")
    except Exception as e:
        print("删除 demo_cam 失败:", e)

for prefix in ["LeftBin", "RightBin"]:
    hs = {}
    for p in PARTS:
        h = find("%s_%s" % (prefix, p))
        if h is None:
            print("!! 缺少 %s_%s" % (prefix, p))
        hs[p] = h
    if hs["bottom"] is None:
        continue
    cx = pos(hs["bottom"])[0]
    delta = NEW_CENTER_X - cx
    print("\n%s 原中心 x=%.3f -> 新中心 x=%.3f (平移 %+.3f)" % (prefix, cx, NEW_CENTER_X, delta))
    for p, h in hs.items():
        if h is None:
            continue
        old = pos(h)
        sim.setObjectPosition(h, sim.handle_world, [old[0] + delta, old[1], old[2]])
        new = pos(h)
        print("   %-16s (%+.3f, %+.3f, %+.3f) -> (%+.3f, %+.3f, %+.3f)" %
              ("%s_%s" % (prefix, p), old[0], old[1], old[2], new[0], new[1], new[2]))

time.sleep(0.3)
print("\n保存场景 ...")
sim.saveScene(SCENE)
print("已保存:", SCENE)

# 复核
print("\n复核（保存后读取）：")
for prefix in ["LeftBin", "RightBin"]:
    for p in PARTS:
        h = find("%s_%s" % (prefix, p))
        if h is not None:
            v = pos(h)
            print("   %-16s (%+.3f, %+.3f, %+.3f)" % ("%s_%s" % (prefix, p), v[0], v[1], v[2]))
