from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.getObject("sim")

box = sim.getObject("/Box")

print("连接成功")

# 获取 Box 当前局部包围盒
min_z = sim.getObjectFloatParam(
    box, sim.objfloatparam_objbbox_min_z
)

max_z = sim.getObjectFloatParam(
    box, sim.objfloatparam_objbbox_max_z
)

old_height = max_z - min_z

print("\n========== 原始 Box ==========")
print(f"高度 = {old_height:.4f} m")
print(f"局部 Z 最低点 = {min_z:.4f}")
print(f"局部 Z 最高点 = {max_z:.4f}")

# ==================================================
# 只把 Z 方向尺寸扩大 5 倍
# X、Y 不变
# ==================================================

sim.scaleObject(box, 1.0, 1.0, 5.0, 0)

time.sleep(0.5)

# ==================================================
# 重新获取缩放后的包围盒
# ==================================================

min_z = sim.getObjectFloatParam(
    box, sim.objfloatparam_objbbox_min_z
)

max_z = sim.getObjectFloatParam(
    box, sim.objfloatparam_objbbox_max_z
)

new_height = max_z - min_z

# ==================================================
# 自动调整 Box 中心位置
# 让 Box 的最低点 = 地面 Z=0
# ==================================================

pos = sim.getObjectPosition(box, -1)

pos[2] = -min_z

sim.setObjectPosition(box, -1, pos)

time.sleep(0.5)

# ==================================================
# 输出结果
# ==================================================

final_pos = sim.getObjectPosition(box, -1)

print("\n========== 修改后的 Box ==========")
print(f"高度 = {new_height:.4f} m")

print("\n位置：")
print(f"X = {final_pos[0]:.4f}")
print(f"Y = {final_pos[1]:.4f}")
print(f"Z = {final_pos[2]:.4f}")

print("\n完成！")
print("X/Y 尺寸不变")
print("Z 高度 ×5")
print("Box 底面保持贴地")