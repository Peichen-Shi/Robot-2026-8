# -*- coding: utf-8 -*-
"""
recolor_classes.py —— 按类别规则给物体上色（改前自动备份场景并保存）
类别规则：位置 1/3/5 = A 类（蓝色）→ 左料盒；位置 2/4/6 = B 类（红色）→ 右料盒
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import shutil, time, os

SCENE = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\second_imitation.ttt"
A_COLOR = [0.10, 0.35, 0.90]   # A 类：蓝
B_COLOR = [0.90, 0.18, 0.18]   # B 类：红

CLASS_OF_INDEX = {1: "A", 2: "B", 3: "A", 4: "B", 5: "A", 6: "B"}

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

# 备份
bak = SCENE.replace(".ttt", "_before_recolor.ttt")
shutil.copyfile(SCENE, bak)
print("场景备份:", os.path.basename(bak))

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

print("\n%-8s %-6s %-10s %s" % ("物体", "类别", "颜色", "handle"))
for i in range(1, 7):
    h = find("Grid%d" % i)
    if h is None:
        print("!! 缺少 Grid%d" % i); continue
    cls = CLASS_OF_INDEX[i]
    col = A_COLOR if cls == "A" else B_COLOR
    sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, col)
    ret = sim.getShapeColor(h, None, sim.colorcomponent_ambient_diffuse)
    got = ret[1] if isinstance(ret, (list, tuple)) and len(ret) > 1 else ret
    print("%-8s %-6s %-10s h=%d  回读=(%.2f, %.2f, %.2f)" %
          ("Grid%d" % i, cls, "蓝(A)" if cls == "A" else "红(B)", h,
           got[0], got[1], got[2]))

sim.saveScene(SCENE)
print("\n已保存场景:", SCENE)
