# -*- coding: utf-8 -*-
"""
delete_grid4.py —— 删除 Grid4 物体（保留其地面网格格），用于演示“视觉检测不到物体则跳过”
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import shutil, time, os

SCENE = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\second_imitation.ttt"
sim = RemoteAPIClient(host="localhost", port=23000).require("sim")

def find(b):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == b:
                return h
        except Exception:
            pass
    return None

print("备份场景:", os.path.basename(SCENE).replace(".ttt", "_before_del_grid4.ttt"))
shutil.copyfile(SCENE, SCENE.replace(".ttt", "_before_del_grid4.ttt"))
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

g4 = find("Grid4")
if g4 is None:
    print("Grid4 不存在（可能已删除）")
else:
    p = sim.getObjectPosition(g4, sim.handle_world)
    sim.removeObjects([g4])
    print("已删除 Grid4（原位置 (%.3f, %.3f, %.3f)）" % (p[0], p[1], p[2]))

sim.saveScene(SCENE)
print("已保存场景")

print("\n当前物体清单：")
for i in range(1, 7):
    h = find("Grid%d" % i)
    if h is None:
        print("  位置%d：无物体（视觉将判定为“未检测到”并跳过）" % i)
    else:
        v = sim.getObjectPosition(h, sim.handle_world)
        col = sim.getShapeColor(h, None, sim.colorcomponent_ambient_diffuse)[1]
        print("  位置%d：%s色  (%.3f, %+.3f)  %s" %
              (i, "蓝" if col[2] > col[0] else "红", v[0], v[1],
               "A类→左料盒" if i in (1, 3, 5) else "B类→右料盒"))
gl = [h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
      if (sim.getObjectAlias(h, 0) or "").startswith("gridline_")]
print("地面网格线仍有 %d 条（网格保留）" % len(gl))
