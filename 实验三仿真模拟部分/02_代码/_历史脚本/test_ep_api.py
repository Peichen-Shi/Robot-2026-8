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


# =========================
# 读取夹爪状态
# =========================

state = simRobomaster.get_gripper(ep)

print("当前夹爪状态:", state)


# =========================
# 张开夹爪
# =========================

print("夹爪打开")

simRobomaster.set_gripper_target(
    ep,
    "open",
    1.0
)

time.sleep(3)

print(
    "当前状态:",
    simRobomaster.get_gripper(ep)
)


# =========================
# 闭合夹爪
# =========================

print("夹爪关闭")

simRobomaster.set_gripper_target(
    ep,
    "close",
    1.0
)

time.sleep(3)

print(
    "当前状态:",
    simRobomaster.get_gripper(ep)
)


print("测试完成")