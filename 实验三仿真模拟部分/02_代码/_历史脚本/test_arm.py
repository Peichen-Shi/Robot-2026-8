from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.require("sim")

gripper = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/"
    "gripper_link_respondable/Prismatic_joint"
)

print("========== 夹爪诊断 ==========")

# 关节类型
joint_type = sim.getJointType(gripper)
print("Joint type:", joint_type)

# 当前位移
position = sim.getJointPosition(gripper)
print("当前位移:", position, "m")

# 关节限位
try:
    interval = sim.getJointInterval(gripper)
    print("关节范围:", interval)
except Exception as e:
    print("无法读取关节范围:", e)

# 当前目标位置
try:
    target = sim.getJointTargetPosition(gripper)
    print("当前目标位置:", target, "m")
except Exception as e:
    print("无法读取目标位置:", e)

# 当前目标速度
try:
    velocity = sim.getJointTargetVelocity(gripper)
    print("当前目标速度:", velocity, "m/s")
except Exception as e:
    print("无法读取目标速度:", e)

print("================================")

time.sleep(1)

# 测试一个很小的位移
print("\n测试：目标位置 +0.005 m")

sim.setJointTargetPosition(gripper, 0.005)

time.sleep(2)

print("实际位置:",
      sim.getJointPosition(gripper), "m")

print("\n测试完成")