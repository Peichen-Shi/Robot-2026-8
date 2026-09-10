from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("连接 CoppeliaSim...")

client = RemoteAPIClient()

sim = client.require("sim")
simRobomaster = client.require("simRobomaster")

print("CoppeliaSim 连接成功")
print("simRobomaster 插件连接成功")

# 获取 EP
robot = sim.getObject("/RoboMaster")

# 创建 EP 控制器
ep = simRobomaster.create_ep(robot)

print("EP controller handle:", ep)

print("\n========== 机械臂测试 ==========")

# ------------------------------------------------
# 获取两个机械臂舵机
# ------------------------------------------------

servo0 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_0"
)

servo1 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_1"
)

print("servo0:", servo0)
print("servo1:", servo1)

print("\n初始角度")

print(
    "servo0 =",
    sim.getJointPosition(servo0),
    "rad"
)

print(
    "servo1 =",
    sim.getJointPosition(servo1),
    "rad"
)

# ------------------------------------------------
# 测试 servo0
# 你之前已经确认：
# servo0 = 机械臂上下
# ------------------------------------------------

print("\n========== 测试 servo0 ==========")

print("servo0：向下")

sim.setJointTargetPosition(
    servo0,
    0.5
)

time.sleep(3)

print(
    "servo0 当前角度:",
    sim.getJointPosition(servo0)
)

print("servo0：回到初始位置")

sim.setJointTargetPosition(
    servo0,
    0
)

time.sleep(3)

# ------------------------------------------------
# 测试 servo1
# 你之前已经确认：
# servo1 = 机械臂前后
# ------------------------------------------------

print("\n========== 测试 servo1 ==========")

print("servo1：向前")

sim.setJointTargetPosition(
    servo1,
    0.5
)

time.sleep(3)

print(
    "servo1 当前角度:",
    sim.getJointPosition(servo1)
)

print("servo1：回到初始位置")

sim.setJointTargetPosition(
    servo1,
    0
)

time.sleep(3)

print("\n========== 测试完成 ==========")