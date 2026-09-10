from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("连接 CoppeliaSim...")

# =========================
# 连接 CoppeliaSim
# =========================

client = RemoteAPIClient()

sim = client.require("sim")
simRobomaster = client.require("simRobomaster")

print("CoppeliaSim 连接成功")
print("simRobomaster 插件连接成功")


# =========================
# 获取 EP
# =========================

robot = sim.getObject("/RoboMaster")
box = sim.getObject("/Box")

ep = simRobomaster.create_ep(robot)

print("EP controller handle:", ep)


# =========================
# 获取机械臂关节
# =========================

servo0 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_0"
)

servo1 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_1"
)


# =========================
# 工具函数
# =========================

def show_position(name, handle):
    p = sim.getObjectPosition(handle, -1)

    print(f"{name}:")
    print(f"  X = {p[0]:.4f}")
    print(f"  Y = {p[1]:.4f}")
    print(f"  Z = {p[2]:.4f}")


def move_arm(s0, s1, wait=3):

    print()
    print(
        f"机械臂运动：servo0 = {s0:.2f}, "
        f"servo1 = {s1:.2f}"
    )

    sim.setJointTargetPosition(
        servo0,
        s0
    )

    sim.setJointTargetPosition(
        servo1,
        s1
    )

    time.sleep(wait)


# =========================
# 夹爪控制
# =========================

def open_gripper():

    print()
    print("夹爪：打开")

    simRobomaster.set_gripper_target(
        ep,
        "open",
        1.0
    )

    time.sleep(3)

    print(
        "当前夹爪状态:",
        simRobomaster.get_gripper(ep)
    )


def close_gripper():

    print()
    print("夹爪：关闭")

    simRobomaster.set_gripper_target(
        ep,
        "close",
        1.0
    )

    time.sleep(3)

    print(
        "当前夹爪状态:",
        simRobomaster.get_gripper(ep)
    )


# =========================
# 开始测试
# =========================

print()
print("=" * 55)
print("          RoboMaster EP 抓取测试")
print("=" * 55)


# =========================
# 1. 打开夹爪
# =========================

print()
print("========== 1. 打开夹爪 ==========")

open_gripper()


# =========================
# 2. 移动到测试位置
# =========================

print()
print("========== 2. 移动到 Box 附近 ==========")

# 根据之前测试得到的结果：
#
# servo0 = 0
# servo1 = 1
#
# 这是目前距离 Box 较近的位置

move_arm(
    0.0,
    1.0,
    4
)


# =========================
# 3. 输出位置
# =========================

print()
print("========== 3. 当前位置 ==========")

show_position("Box", box)

# 找到机械臂末端
gripper = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/"
    "gripper_link_respondable"
)

show_position("夹爪", gripper)


# =========================
# 4. 暂停确认
# =========================

print()
print("=" * 55)
print("        请观察 CoppeliaSim")
print("=" * 55)

print()
print("此时：")
print("① 夹爪应该已经打开")
print("② 机械臂应该已经移动到 Box 附近")
print("③ 暂时不会闭合夹爪")
print("④ 暂时不会抬起机械臂")

input(
    "\n如果夹爪位置合适，可以按 Enter 继续抓取..."
)


# =========================
# 5. 闭合夹爪
# =========================

print()
print("========== 4. 闭合夹爪 ==========")

close_gripper()


# =========================
# 6. 等待观察
# =========================

print()
print("========== 5. 检查是否抓住 Box ==========")

print("等待 2 秒...")
time.sleep(2)

show_position("Box", box)


# =========================
# 7. 抬起机械臂
# =========================

print()
print("========== 6. 抬起机械臂 ==========")

# 从 servo1 = 1.0
# 回到 0.7

move_arm(
    0.0,
    0.7,
    3
)


# =========================
# 8. 最终结果
# =========================

print()
print("=" * 55)
print("              最终结果")
print("=" * 55)

show_position("Box", box)

box_pos = sim.getObjectPosition(box, -1)

print()
print(f"Box 最终 Z = {box_pos[2]:.4f} m")

if box_pos[2] > 0.12:

    print()
    print(">>> 抓取成功！")
    print(">>> Box 已经被机械臂抬起来。")

else:

    print()
    print(">>> Box 没有被抬起。")
    print(">>> 说明夹爪没有真正抓住 Box。")


print()
print("=" * 55)
print("              测试完成")
print("=" * 55)