from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.getObject("sim")

servo0 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_0"
)

servo1 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_1"
)

gripper = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/gripper_link_respondable"
)

print("连接成功")

def get_pos():
    return sim.getObjectPosition(gripper, -1)

def show(name):
    p = get_pos()
    print(f"{name}")
    print(f"  X = {p[0]:.6f}")
    print(f"  Y = {p[1]:.6f}")
    print(f"  Z = {p[2]:.6f}")
    return p

# ==================================================
# 先回到初始位置
# ==================================================

print("\n========== 回到初始位置 ==========")

sim.setJointTargetPosition(servo0, 0)
sim.setJointTargetPosition(servo1, 0)

time.sleep(3)

p0 = show("初始位置")

# ==================================================
# 测试 servo0
# ==================================================

print("\n========== 测试 servo0 ==========")
print("servo0 = +0.5")
print("servo1 = 0")

sim.setJointTargetPosition(servo0, 0.5)
sim.setJointTargetPosition(servo1, 0)

time.sleep(3)

p1 = show("servo0 +0.5 后")

# ==================================================
# 回零
# ==================================================

print("\n回到初始位置...")

sim.setJointTargetPosition(servo0, 0)
sim.setJointTargetPosition(servo1, 0)

time.sleep(3)

# ==================================================
# 测试 servo1
# ==================================================

print("\n========== 测试 servo1 ==========")
print("servo0 = 0")
print("servo1 = +0.5")

sim.setJointTargetPosition(servo0, 0)
sim.setJointTargetPosition(servo1, 0.5)

time.sleep(3)

p2 = show("servo1 +0.5 后")

# ==================================================
# 回零
# ==================================================

print("\n回到初始位置...")

sim.setJointTargetPosition(servo0, 0)
sim.setJointTargetPosition(servo1, 0)

time.sleep(3)

# ==================================================
# 计算变化量
# ==================================================

print("\n========================================")
print("           关节运动影响分析")
print("========================================")

print("\nservo0 +0.5 的末端变化：")
print(f"ΔX = {p1[0] - p0[0]:+.6f} m")
print(f"ΔY = {p1[1] - p0[1]:+.6f} m")
print(f"ΔZ = {p1[2] - p0[2]:+.6f} m")

print("\nservo1 +0.5 的末端变化：")
print(f"ΔX = {p2[0] - p0[0]:+.6f} m")
print(f"ΔY = {p2[1] - p0[1]:+.6f} m")
print(f"ΔZ = {p2[2] - p0[2]:+.6f} m")

print("\n测试完成")