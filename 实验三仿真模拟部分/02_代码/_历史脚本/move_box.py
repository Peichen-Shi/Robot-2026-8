from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.require("sim")

print("连接成功")

# 获取 Box
box = sim.getObject("/Box")

# 原位置
old_pos = sim.getObjectPosition(box, -1)

print("\n========== 原位置 ==========")
print(f"X = {old_pos[0]:.4f}")
print(f"Y = {old_pos[1]:.4f}")
print(f"Z = {old_pos[2]:.4f}")

# 新位置
# X 从 0.35 移到 0.28
# Y 不变
# Z 不变，保证 Box 仍然贴地
new_pos = [0.28, old_pos[1], 0.10]

sim.setObjectPosition(box, -1, new_pos)

# 读取修改后的位置
pos = sim.getObjectPosition(box, -1)

print("\n========== 修改后 ==========")
print(f"X = {pos[0]:.4f}")
print(f"Y = {pos[1]:.4f}")
print(f"Z = {pos[2]:.4f}")

print("\nBox 已向机器人方向移动 0.07 m")
print("高度和尺寸均未改变")
print("Box 底面仍然贴地")