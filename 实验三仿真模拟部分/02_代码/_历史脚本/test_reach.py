from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.getObject("sim")

# 获取机械臂关节
servo0 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_0"
)

servo1 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_1"
)

print("连接成功")
print("开始测试机械臂伸出...")

# ------------------------------------------------
# servo1：向前
# ------------------------------------------------
print("servo1 → 向前")

sim.setJointTargetPosition(servo1, 0.8)

time.sleep(3)

print(
    "servo1 当前角度:",
    sim.getJointPosition(servo1)
)

# ------------------------------------------------
# servo0：向下
# ------------------------------------------------
print("servo0 → 向下")

sim.setJointTargetPosition(servo0, 0.8)

time.sleep(3)

print(
    "servo0 当前角度:",
    sim.getJointPosition(servo0)
)

print()
print("机械臂现在应该已经伸向 Box")
print("暂时不要关闭夹爪！")
print("观察 EP 是否靠近 Box。")

time.sleep(3)

# ------------------------------------------------
# 回到初始位置
# ------------------------------------------------
print("机械臂回到初始位置")

sim.setJointTargetPosition(servo0, 0.0)
time.sleep(2)

sim.setJointTargetPosition(servo1, 0.0)
time.sleep(2)

print("测试完成")