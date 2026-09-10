from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.getObject("sim")

print("连接成功")

# 获取 Box
box = sim.getObject("/Box")

# 读取当前位置
pos = sim.getObjectPosition(box, -1)

print("\n当前 Box 位置：")
print(f"X = {pos[0]:.3f}")
print(f"Y = {pos[1]:.3f}")
print(f"Z = {pos[2]:.3f}")

# ==================================================
# 修改 Box 位置
# X、Y 不变
# Z 改成 0.10 m
# ==================================================

pos[2] = 0.10

sim.setObjectPosition(box, -1, pos)

time.sleep(0.5)

# 再读取一次确认
new_pos = sim.getObjectPosition(box, -1)

print("\n修改后的 Box 位置：")
print(f"X = {new_pos[0]:.3f}")
print(f"Y = {new_pos[1]:.3f}")
print(f"Z = {new_pos[2]:.3f}")

print("\nBox 高度位置已经修改完成！")