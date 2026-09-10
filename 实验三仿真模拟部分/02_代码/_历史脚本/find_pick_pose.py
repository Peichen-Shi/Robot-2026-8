from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

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

box = sim.getObject("/Box")

box_pos = sim.getObjectPosition(box, -1)

print("Box:", box_pos)

best = None

# 搜索关节角度
# 范围先不要太大，避免机械臂折进车体
servo0_values = [
    i / 20
    for i in range(0, 21)
]

servo1_values = [
    i / 20
    for i in range(0, 21)
]

for a0 in servo0_values:

    for a1 in servo1_values:

        sim.setJointTargetPosition(servo0, a0)
        sim.setJointTargetPosition(servo1, a1)

        # 给仿真一点时间运动
        time.sleep(0.03)

        p = sim.getObjectPosition(gripper, -1)

        dx = p[0] - box_pos[0]
        dy = p[1] - box_pos[1]

        # 抓取时希望夹爪中心略高于 Box
        target_z = box_pos[2] + 0.035
        dz = p[2] - target_z

        distance = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        if best is None or distance < best[0]:

            best = (
                distance,
                a0,
                a1,
                p[0],
                p[1],
                p[2]
            )

# 输出最佳结果
print()
print("================================")
print("        最佳抓取姿态")
print("================================")

print("距离:", best[0])

print("servo0 =", best[1])
print("servo1 =", best[2])

print()
print("Gripper:")
print("X =", best[3])
print("Y =", best[4])
print("Z =", best[5])

print()
print("Box:")
print("X =", box_pos[0])
print("Y =", box_pos[1])
print("Z =", box_pos[2])

# 保持最佳姿态
sim.setJointTargetPosition(servo0, best[1])
sim.setJointTargetPosition(servo1, best[2])

time.sleep(3)

print()
print("机械臂已停在最佳测试姿态")
print("请观察夹爪是否接近 Box")