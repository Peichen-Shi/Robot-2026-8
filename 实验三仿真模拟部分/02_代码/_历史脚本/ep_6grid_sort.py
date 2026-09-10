from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math


# ============================================================
# RoboMaster EP
# 实验三：Grid1 抓取测试
#
# 当前场景参数严格按照 setup_6grid.py
#
# RoboMaster 初始：
#     (0, 0, 0)
#
# Grid1：
#     (0.380, -0.500, 0.110)
#
# LeftBin：
#     (0.380, -0.900, 0.000)
#
# 实际物体：
#     蓝色四棱柱
#     0.06 × 0.06 × 0.22 m
#
#     红色圆柱
#     直径 0.12 m
#     高度 0.22 m
#
# 本程序：
#
# 1. 小车移动到 Grid1 的 Y
# 2. 机械臂向前伸
# 3. 夹爪靠近 Grid1
# 4. 闭合夹爪
# 5. 抬升
# 6. 机械臂收回
# 7. 小车移动到 LeftBin
# 8. 机械臂伸出
# 9. 下降
# 10. 松开夹爪
# 11. 机械臂收回
# 12. 机械臂复位
# 13. 小车回原点
#
# ============================================================


print()
print("==============================================")
print("       RoboMaster EP Grid1 抓取测试")
print("==============================================")
print()


# ============================================================
# 1. 连接 CoppeliaSim
# ============================================================

print("连接 CoppeliaSim...")

client = RemoteAPIClient()

sim = client.getObject("sim")

print("连接成功")
print()


# ============================================================
# 2. 获取 RoboMaster
# ============================================================

print("获取 RoboMaster...")

robot = sim.getObject("/RoboMaster")

print("RoboMaster 获取成功")
print()


# ============================================================
# 3. 获取机械臂关节
# ============================================================

print("获取机械臂关节...")

servo0 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_0"
)

servo1 = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/servo_motor_1"
)

print("机械臂关节获取成功")
print()


# ============================================================
# 4. 获取夹爪
# ============================================================

print("获取夹爪...")

try:

    gripper_joint = sim.getObject(
        "/RoboMaster/arm_base_link_respondable/Prismatic_joint"
    )

    print("Prismatic_joint 获取成功")

except Exception as e:

    gripper_joint = None

    print("Prismatic_joint 获取失败")
    print(e)


# ============================================================
# 5. 获取夹爪中心
# ============================================================

print()
print("获取夹爪中心...")

try:

    gripper = sim.getObject(
        "/RoboMaster/arm_base_link_respondable/gripper_link_respondable"
    )

    print("夹爪中心获取成功")

except Exception as e:

    gripper = None

    print("夹爪中心获取失败")
    print(e)


print()


# ============================================================
# 6. 获取 Grid1
# ============================================================

print("获取 Grid1...")

try:

    grid1 = sim.getObject("/Grid1")

    print("Grid1 获取成功")

except Exception as e:

    print("Grid1 获取失败")
    print(e)

    input("按 Enter 退出...")
    raise SystemExit


# ============================================================
# 7. 获取 LeftBin
# ============================================================

print("获取 LeftBin...")

try:

    left_bin = sim.getObject("/LeftBin_bottom")

    print("LeftBin 获取成功")

except Exception as e:

    print("LeftBin 获取失败")
    print(e)

    input("按 Enter 退出...")
    raise SystemExit


# ============================================================
# 8. 场景坐标
# ============================================================

GRID1_X = 0.380
GRID1_Y = -0.500

LEFT_BIN_X = 0.380
LEFT_BIN_Y = -0.900


print()
print("==============================================")
print("                 场景坐标")
print("==============================================")

print()
print("Grid1:")
print("X =", GRID1_X)
print("Y =", GRID1_Y)
print("Z = 0.110")

print()
print("LeftBin:")
print("X =", LEFT_BIN_X)
print("Y =", LEFT_BIN_Y)
print("Z = 0.000")

print()


# ============================================================
# 9. 获取当前夹爪位置
# ============================================================

if gripper is not None:

    p = sim.getObjectPosition(
        gripper,
        -1
    )

    print("当前夹爪位置:")
    print(
        "X = %.3f  Y = %.3f  Z = %.3f"
        % (
            p[0],
            p[1],
            p[2]
        )
    )

print()


# ============================================================
# 10. 机械臂初始角度
#
# 注意：
# 这里先读取当前角度。
#
# 这样无论你的模型当前初始姿态是什么，
# 程序结束后都可以恢复。
# ============================================================

initial_servo0 = sim.getJointPosition(
    servo0
)

initial_servo1 = sim.getJointPosition(
    servo1
)


print("机械臂初始角度:")

print(
    "servo0 = %.4f rad"
    % initial_servo0
)

print(
    "servo1 = %.4f rad"
    % initial_servo1
)

print()


# ============================================================
# 11. 夹爪初始位置
# ============================================================

initial_gripper_position = None

if gripper_joint is not None:

    try:

        initial_gripper_position = sim.getJointPosition(
            gripper_joint
        )

        print(
            "夹爪初始位置 = %.4f"
            % initial_gripper_position
        )

    except Exception:

        pass


print()


# ============================================================
# 12. 小车移动函数
#
# 你的要求：
#
# 小车只做左右平移。
#
# 不改变 X
# 不改变 Z
# 不改变姿态
#
# ============================================================

def move_robot_y(target_y):

    print()
    print("----------------------------------------------")
    print("小车平移")
    print("----------------------------------------------")

    current = sim.getObjectPosition(
        robot,
        -1
    )

    print(
        "当前：X=%.3f Y=%.3f Z=%.3f"
        % (
            current[0],
            current[1],
            current[2]
        )
    )

    print(
        "目标：X=%.3f Y=%.3f Z=%.3f"
        % (
            current[0],
            target_y,
            current[2]
        )
    )

    sim.setObjectPosition(
        robot,
        -1,
        [
            current[0],
            target_y,
            current[2]
        ]
    )

    time.sleep(1.5)

    current = sim.getObjectPosition(
        robot,
        -1
    )

    print(
        "到达：X=%.3f Y=%.3f Z=%.3f"
        % (
            current[0],
            current[1],
            current[2]
        )
    )


# ============================================================
# 13. 机械臂运动函数
# ============================================================

def move_arm(a0, a1, wait=2.0):

    print()
    print(
        "机械臂运动：servo0=%.3f  servo1=%.3f"
        % (
            a0,
            a1
        )
    )

    sim.setJointTargetPosition(
        servo0,
        a0
    )

    sim.setJointTargetPosition(
        servo1,
        a1
    )

    time.sleep(wait)


# ============================================================
# 14. 夹爪打开
#
# 如果你的 Prismatic_joint 数值方向相反，
# 后面只需要修改 OPEN_POSITION / CLOSE_POSITION。
# ============================================================

OPEN_POSITION = 0.015
CLOSE_POSITION = 0.0


def gripper_open():

    if gripper_joint is None:

        print("警告：没有找到夹爪关节")

        return

    print()
    print("夹爪打开")

    sim.setJointTargetPosition(
        gripper_joint,
        OPEN_POSITION
    )

    time.sleep(1.0)


# ============================================================
# 15. 夹爪闭合
# ============================================================

def gripper_close():

    if gripper_joint is None:

        print("警告：没有找到夹爪关节")

        return

    print()
    print("夹爪闭合")

    sim.setJointTargetPosition(
        gripper_joint,
        CLOSE_POSITION
    )

    time.sleep(1.0)


# ============================================================
# 16. 根据当前夹爪位置寻找
#     servo0 / servo1
#
# 这里采用和你之前成功测试代码相同的思路：
#
# 搜索两个机械臂关节角度，
# 找到距离目标 XY 最近的位置。
#
# 这样不需要直接写死 servo0 / servo1。
# ============================================================

def find_arm_pose(
    target_x,
    target_y,
    target_z,
    a0_min=0.0,
    a0_max=1.0,
    a1_min=0.0,
    a1_max=1.0
):

    print()
    print("----------------------------------------------")
    print("搜索机械臂目标姿态")
    print("----------------------------------------------")

    best = None

    servo0_values = [
        i / 20
        for i in range(
            int(a0_min * 20),
            int(a0_max * 20) + 1
        )
    ]

    servo1_values = [
        i / 20
        for i in range(
            int(a1_min * 20),
            int(a1_max * 20) + 1
        )
    ]

    for a0 in servo0_values:

        for a1 in servo1_values:

            sim.setJointTargetPosition(
                servo0,
                a0
            )

            sim.setJointTargetPosition(
                servo1,
                a1
            )

            time.sleep(0.025)

            if gripper is None:
                continue

            p = sim.getObjectPosition(
                gripper,
                -1
            )

            dx = p[0] - target_x
            dy = p[1] - target_y
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

    if best is None:

        print("没有找到机械臂姿态")

        return None

    print()
    print("最佳姿态:")

    print(
        "距离 = %.4f m"
        % best[0]
    )

    print(
        "servo0 = %.4f rad"
        % best[1]
    )

    print(
        "servo1 = %.4f rad"
        % best[2]
    )

    print()
    print("夹爪位置:")

    print(
        "X = %.4f"
        % best[3]
    )

    print(
        "Y = %.4f"
        % best[4]
    )

    print(
        "Z = %.4f"
        % best[5]
    )

    return best


# ============================================================
# 17. 开始测试
# ============================================================

print()
print("==============================================")
print("           开始 Grid1 抓取测试")
print("==============================================")
print()


# ============================================================
# STEP 1
#
# 小车平移到 Grid1
#
# Grid1 Y = -0.5
# ============================================================

print()
print("STEP 1：小车移动到 Grid1")

move_robot_y(
    GRID1_Y
)


# ============================================================
# STEP 2
#
# 打开夹爪
# ============================================================

print()
print("STEP 2：打开夹爪")

gripper_open()


# ============================================================
# STEP 3
#
# 获取 Grid1 实际坐标
# ============================================================

grid_pos = sim.getObjectPosition(
    grid1,
    -1
)

print()
print("Grid1 实际坐标:")

print(
    "X = %.3f"
    % grid_pos[0]
)

print(
    "Y = %.3f"
    % grid_pos[1]
)

print(
    "Z = %.3f"
    % grid_pos[2]
)


# ============================================================
# STEP 4
#
# 搜索 Grid1 抓取姿态
#
# 因为物体高度 0.22m，
# 当前先以物体中心附近作为目标。
# ============================================================

target_z = 0.11

best_pose = find_arm_pose(
    GRID1_X,
    GRID1_Y,
    target_z
)


if best_pose is None:

    print("无法找到抓取姿态")

    input("按 Enter 退出...")

    raise SystemExit


# ============================================================
# STEP 5
#
# 机械臂伸向 Grid1
# ============================================================

print()
print("STEP 3：机械臂伸向 Grid1")

move_arm(
    best_pose[1],
    best_pose[2],
    3.0
)


# ============================================================
# STEP 6
#
# 打印当前夹爪位置
# ============================================================

if gripper is not None:

    p = sim.getObjectPosition(
        gripper,
        -1
    )

    print()
    print("当前夹爪中心:")

    print(
        "X = %.3f"
        % p[0]
    )

    print(
        "Y = %.3f"
        % p[1]
    )

    print(
        "Z = %.3f"
        % p[2]
    )


# ============================================================
# STEP 7
#
# 闭合夹爪
# ============================================================

print()
print("STEP 4：闭合夹爪")

gripper_close()

time.sleep(1)


# ============================================================
# STEP 8
#
# 抬升
#
# 这里通过 servo1 改变机械臂高度。
#
# 先在当前姿态基础上稍微抬升。
# ============================================================

print()
print("STEP 5：抬升物体")

lift_servo1 = best_pose[2] - 0.25

move_arm(
    best_pose[1],
    lift_servo1,
    2.0
)


# ============================================================
# STEP 9
#
# 机械臂收回
#
# servo0 回到初始位置。
# ============================================================

print()
print("STEP 6：机械臂收回")

move_arm(
    initial_servo0,
    lift_servo1,
    2.0
)


# ============================================================
# STEP 10
#
# 小车移动到 LeftBin
#
# LeftBin Y = -0.9
# ============================================================

print()
print("STEP 7：小车移动到 LeftBin")

move_robot_y(
    LEFT_BIN_Y
)


# ============================================================
# STEP 11
#
# 机械臂伸出到 LeftBin
#
# 使用之前找到的前伸姿态。
# ============================================================

print()
print("STEP 8：机械臂伸向 LeftBin")

move_arm(
    best_pose[1],
    lift_servo1,
    2.0
)


# ============================================================
# STEP 12
#
# 下降
# ============================================================

print()
print("STEP 9：下降")

move_arm(
    best_pose[1],
    best_pose[2],
    2.0
)


# ============================================================
# STEP 13
#
# 松开夹爪
# ============================================================

print()
print("STEP 10：松开夹爪")

gripper_open()

time.sleep(1.5)


# ============================================================
# STEP 14
#
# 机械臂收回
# ============================================================

print()
print("STEP 11：机械臂收回")

move_arm(
    initial_servo0,
    lift_servo1,
    2.0
)


# ============================================================
# STEP 15
#
# 机械臂恢复初始姿态
# ============================================================

print()
print("STEP 12：机械臂恢复初始姿态")

move_arm(
    initial_servo0,
    initial_servo1,
    2.0
)


# ============================================================
# STEP 16
#
# 小车回原点
# ============================================================

print()
print("STEP 13：小车回到原点")

move_robot_y(
    0.0
)


# ============================================================
# STEP 17
#
# 最终状态
# ============================================================

print()
print("==============================================")
print("             Grid1 抓取测试完成")
print("==============================================")
print()

final_pos = sim.getObjectPosition(
    robot,
    -1
)

print("小车最终位置:")

print(
    "X = %.3f"
    % final_pos[0]
)

print(
    "Y = %.3f"
    % final_pos[1]
)

print(
    "Z = %.3f"
    % final_pos[2]
)

print()

print("理论目标：")

print("X = 0.000")
print("Y = 0.000")
print("Z = 0.000")

print()

print("Grid1 → LeftBin 测试结束")

print()

input("按 Enter 退出...")