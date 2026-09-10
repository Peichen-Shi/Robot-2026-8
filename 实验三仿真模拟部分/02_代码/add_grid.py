# -*- coding: utf-8 -*-
"""
add_grid.py —— 在物体 1~6 下方绘制黑色正方形网格（地板贴线），并保存场景
设计：
  · 6 个正方形格子，每格 0.20 × 0.20 m，格心 = 物体位置 (0.380, -0.5 … +0.5)
  · x 范围 0.280 ~ 0.480，y 范围 -0.600 ~ +0.600
  · 最外侧边缘 y=±0.600，与料盒内壁（±0.675）相距 7.5cm → 不重合、不接触
  · 线宽 6mm、厚 4mm，黑se、static=1、respondable=0（纯视觉，不参与碰撞）
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import shutil, time, os

SCENE = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\second_imitation.ttt"
CELL = 0.20                  # 格子边长
X0, X1 = 0.380 - CELL / 2, 0.380 + CELL / 2      # 0.280 ~ 0.480
YS = [-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6]      # 7 条横线（y 方向）
LINE_W, LINE_T, LINE_Z = 0.006, 0.004, 0.003
BLACK = [0.02, 0.02, 0.02]

sim = RemoteAPIClient(host="localhost", port=23000).require("sim")

def find(b):
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        try:
            if sim.getObjectAlias(h, 0) == b:
                return h
        except Exception:
            pass
    return None

print("备份场景:", os.path.basename(SCENE).replace(".ttt", "_before_grid.ttt"))
shutil.copyfile(SCENE, SCENE.replace(".ttt", "_before_grid.ttt"))
try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

# 先删掉上次生成的网格（可重复运行）
old = [h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
       if (sim.getObjectAlias(h, 0) or "").startswith("gridline_")]
for h in old:
    try:
        sim.removeObjects([h])
    except Exception:
        pass
if old:
    print("已清理旧网格线 %d 条" % len(old))

made = []
# 横线（沿 x 方向）
for i, y in enumerate(YS):
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [CELL + LINE_W, LINE_W, LINE_T], 0)
    sim.setObjectAlias(h, "gridline_h_%02d" % i)
    sim.setObjectPosition(h, sim.handle_world, [(X0 + X1) / 2, y, LINE_Z])
    sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, BLACK)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
    made.append(h)
# 竖线（沿 y 方向，左右两条边）
for i, x in enumerate([X0, X1]):
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [LINE_W, YS[-1] - YS[0] + LINE_W, LINE_T], 0)
    sim.setObjectAlias(h, "gridline_v_%02d" % i)
    sim.setObjectPosition(h, sim.handle_world, [x, (YS[0] + YS[-1]) / 2, LINE_Z])
    sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, BLACK)
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
    made.append(h)

sim.saveScene(SCENE)
print("已创建网格线 %d 条并保存场景" % len(made))
print("\n网格范围：x %.3f~%.3f, y %.3f~%.3f" % (X0, X1, YS[0], YS[-1]))
print("料盒内壁位置：LeftBin 后壁内面 y=%.3f, RightBin 前壁内面 y=+%.3f" % (-0.685 + 0.01, 0.685 - 0.01))
print("外侧间隔：%.3f m（>0 即不重合）" % ((-0.685 + 0.01) - YS[0]))
print("\n网格线与物体位置对应：")
for i, y in enumerate([-0.5, -0.3, -0.1, 0.1, 0.3, 0.5]):
    print("  位置%d: 格心 (%.3f, %+.3f)，格边 y=%.3f~%.3f" % (i + 1, 0.380, y, y - 0.1, y + 0.1))
