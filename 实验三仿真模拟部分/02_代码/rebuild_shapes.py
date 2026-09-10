# -*- coding: utf-8 -*-
"""
rebuild_shapes.py —— 按类别规则重建物体几何（并保存场景）
规则：位置 1/3/5 = A 类（蓝色四棱柱）；位置 2/4/6 = B 类（红色圆柱）
尺寸沿用原场景约定：棱柱 0.06×0.06×0.22；圆柱 半径0.06（直径0.12）×高0.22
只重建需要换形状的：Grid2(棱柱→圆柱)、Grid3(圆柱→棱柱)、Grid5(圆柱→棱柱)
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import shutil, time, os

SCENE = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\second_imitation.ttt"
PRISM = (0.06, 0.06, 0.22)     # 棱柱尺寸（x, y, z）
CYL = (0.06, 0.06, 0.22)       # 圆柱（半径x, 半径y, 高）
A_COLOR, B_COLOR = [0.10, 0.35, 0.90], [0.90, 0.18, 0.18]
PLAN = {2: ("cylinder", "B"), 3: ("cuboid", "A"), 5: ("cuboid", "A")}

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

print("场景备份:", os.path.basename(SCENE).replace(".ttt", "_before_shape.ttt"))
shutil.copyfile(SCENE, SCENE.replace(".ttt", "_before_shape.ttt"))

try:
    sim.stopSimulation()
except Exception:
    pass
time.sleep(0.4)

for idx, (kind, cls) in PLAN.items():
    name = "Grid%d" % idx
    old = find(name)
    if old is None:
        print("!! 找不到", name); continue
    p = sim.getObjectPosition(old, sim.handle_world)
    ret = sim.getShapeColor(old, None, sim.colorcomponent_ambient_diffuse)
    oldcol = ret[1] if isinstance(ret, (list, tuple)) and len(ret) > 1 else ret
    sim.removeObjects([old])
    prim = sim.primitiveshape_cuboid if kind == "cuboid" else sim.primitiveshape_cylinder
    size = PRISM if kind == "cuboid" else CYL
    new = sim.createPrimitiveShape(prim, list(size), 0)
    sim.setObjectAlias(new, name, 1)
    sim.setObjectPosition(new, sim.handle_world, p)
    col = A_COLOR if cls == "A" else B_COLOR
    sim.setShapeColor(new, None, sim.colorcomponent_ambient_diffuse, col)
    sim.setObjectInt32Param(new, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(new, sim.shapeintparam_respondable, 0)
    print("  %-6s 重建为 %-8s (%s类)  位置=(%.3f, %.3f, %.3f)  旧色=(%.2f,%.2f,%.2f) 新h=%d" %
          (name, "棱柱" if kind == "cuboid" else "圆柱", cls,
           p[0], p[1], p[2], oldcol[0], oldcol[1], oldcol[2], new))

sim.saveScene(SCENE)
print("\n已保存场景")

print("\n最终布局核对：")
for i in range(1, 7):
    h = find("Grid%d" % i)
    if h is None:
        print("  Grid%d 缺失" % i); continue
    p = sim.getObjectPosition(h, sim.handle_world)
    ret = sim.getShapeColor(h, None, sim.colorcomponent_ambient_diffuse)
    col = ret[1] if isinstance(ret, (list, tuple)) and len(ret) > 1 else ret
    st = sim.getObjectInt32Param(h, sim.shapeintparam_static)
    cls = "A(蓝棱柱)" if (i in (1, 3, 5)) else "B(红圆柱)"
    print("  Grid%d  应为 %-10s  位置=(%.3f, %.3f, %.3f)  颜色=(%.2f,%.2f,%.2f)  static=%d" %
          (i, cls, p[0], p[1], p[2], col[0], col[1], col[2], st))
