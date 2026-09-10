from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math


# ============================================================
# RoboMaster EP Grid1 单次抓取测试
#
# 基于已经成功的 CoppeliaSim Lua 抓取逻辑
#
# 核心原则：
# 1. ZMQ Remote API
# 2. 不使用 simRobomaster.create_ep
# 3. 自动按对象名称寻找关节
# 4. 直接控制 servo_motor_0
# 5. 直接控制 servo_motor_1
# 6. 直接控制 Prismatic_joint
# 7. 抓取后 Grid1 跟随夹爪
# ============================================================


HOST = "localhost"
PORT = 23000


# ============================================================
# 参数
# ============================================================

GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = -0.025

ARM_HOME = 0.0
ARM_LOWER = 2.0
ARM_LIFT = -10.0

ARM_TIME = 3.0
GRIPPER_TIME = 2.0
HOLD_TIME = 1.5


# ============================================================
# 全局抓取状态
# ============================================================

hold_offset = None
held = False


# ============================================================
# 工具函数
# ============================================================

def move_linear(start, end, duration, callback=None):
    """
    线性插值
    """

    steps = max(1, int(duration / 0.03))

    for i in range(steps + 1):

        t = i / steps

        value = start + (end - start) * t

        if callback:
            callback(value)

        time.sleep(duration / steps)


# ------------------------------------------------------------

def get_position(sim, handle):

    return sim.getObjectPosition(
        handle,
        sim.handle_world
    )


# ------------------------------------------------------------

def set_position(sim, handle, pos):

    sim.setObjectPosition(
        handle,
        sim.handle_world,
        pos
    )


# ------------------------------------------------------------

def get_joint_position(sim, handle):

    return sim.getJointPosition(handle)


# ------------------------------------------------------------

def set_joint_position(sim, handle, value):

    # 与之前成功的 Lua 代码保持一致

    try:
        sim.setJointPosition(
            handle,
            value
        )
    except Exception:
        pass

    try:
        sim.setJointTargetPosition(
            handle,
            value
        )
    except Exception:
        pass


# ------------------------------------------------------------

def set_joint_deg(sim, handle, deg):

    rad = math.radians(deg)

    set_joint_position(
        sim,
        handle,
        rad
    )


# ------------------------------------------------------------

def get_joint_deg(sim, handle):

    rad = get_joint_position(
        sim,
        handle
    )

    return math.degrees(rad)


# ------------------------------------------------------------

def set_gripper(sim, handle, value):

    try:
        sim.setJointPosition(
            handle,
            value
        )
    except Exception:
        pass

    try:
        sim.setJointTargetPosition(
            handle,
            value
        )
    except Exception:
        pass


# ------------------------------------------------------------

def get_gripper_position(sim, handle):

    return sim.getJointPosition(handle)


# ------------------------------------------------------------

def finger_center(sim, left_finger, right_finger):

    left = get_position(
        sim,
        left_finger
    )

    right = get_position(
        sim,
        right_finger
    )

    return [
        (left[0] + right[0]) / 2,
        (left[1] + right[1]) / 2,
        (left[2] + right[2]) / 2
    ]


# ============================================================
# 自动寻找对象
# ============================================================

def find_object_by_name(sim, name):

    """
    不依赖对象层级。

    例如：
        servo_motor_1

    不管它在：

        /RoboMaster/xxx/servo_motor_1

    还是：

        /RoboMaster/xxx/xxx/servo_motor_1

    都尝试寻找。
    """

    # --------------------------------------------------------
    # 方法 1：精确别名
    # --------------------------------------------------------

    try:

        handle = sim.getObject(
            "/" + name
        )

        if handle is not None and handle >= 0:

            return handle

    except Exception:

        pass


    # --------------------------------------------------------
    # 方法 2：通配符
    # --------------------------------------------------------

    possible_paths = [

        "/**/" + name,

        "/RoboMaster/**/" + name,

        "/RoboMaster/" + name,

    ]


    for path in possible_paths:

        try:

            handle = sim.getObject(
                path
            )

            if handle is not None and handle >= 0:

                return handle

        except Exception:

            pass


    # --------------------------------------------------------
    # 方法 3：直接查询全部对象
    # --------------------------------------------------------

    try:

        objects = sim.getObjectsInTree(
            sim.handle_scene,
            sim.handle_all,
            0
        )

        for handle in objects:

            try:

                alias = sim.getObjectAlias(
                    handle,
                    0
                )

                # CoppeliaSim alias 有时可能带路径
                if alias == name:

                    return handle

                if alias.endswith("/" + name):

                    return handle

            except Exception:

                pass

    except Exception:

        pass


    return None


# ============================================================
# 抓取
# ============================================================

def attach_grid1(
    sim,
    grid1,
    left_finger,
    right_finger
):

    global hold_offset
    global held


    grid_pos = get_position(
        sim,
        grid1
    )


    center = finger_center(
        sim,
        left_finger,
        right_finger
    )


    hold_offset = [

        grid_pos[0] - center[0],

        grid_pos[1] - center[1],

        grid_pos[2] - center[2]

    ]


    held = True


    print()
    print("============================================================")
    print("                    Grid1 已抓取")
    print("============================================================")


    print(
        f"Grid1："
        f"({grid_pos[0]:.3f}, "
        f"{grid_pos[1]:.3f}, "
        f"{grid_pos[2]:.3f})"
    )


    print(
        f"夹爪中心："
        f"({center[0]:.3f}, "
        f"{center[1]:.3f}, "
        f"{center[2]:.3f})"
    )


    print(
        f"相对偏移："
        f"({hold_offset[0]:.3f}, "
        f"{hold_offset[1]:.3f}, "
        f"{hold_offset[2]:.3f})"
    )


# ============================================================
# 让 Grid1 跟随夹爪
# ============================================================

def update_grid1(
    sim,
    grid1,
    left_finger,
    right_finger
):

    global hold_offset
    global held


    if not held:

        return


    center = finger_center(
        sim,
        left_finger,
        right_finger
    )


    new_pos = [

        center[0] + hold_offset[0],

        center[1] + hold_offset[1],

        center[2] + hold_offset[2]

    ]


    set_position(
        sim,
        grid1,
        new_pos
    )


# ============================================================
# 释放 Grid1
# ============================================================

def release_grid1():

    global held
    global hold_offset


    held = False

    hold_offset = None


    print()
    print("Grid1 已释放")


# ============================================================
# 程序开始
# ============================================================

print()
print("============================================================")
print("          RoboMaster EP Grid1 单次抓取测试")
print("============================================================")


# ============================================================
# 1. 连接 CoppeliaSim
# ============================================================

print()
print("连接 CoppeliaSim...")
print(f"使用 ZMQ Remote API：{PORT}")


client = RemoteAPIClient(
    host=HOST,
    port=PORT
)


sim = client.require("sim")


print("连接成功")


# ============================================================
# 2. 获取 RoboMaster
# ============================================================

print()
print("获取 RoboMaster...")


robot = find_object_by_name(
    sim,
    "RoboMaster"
)


if robot is None:

    raise RuntimeError(
        "找不到 RoboMaster"
    )


print("RoboMaster 获取成功")


# ============================================================
# 3. 获取 Grid1
# ============================================================

print()
print("获取 Grid1...")


grid1 = find_object_by_name(
    sim,
    "Grid1"
)


if grid1 is None:

    raise RuntimeError(
        "找不到 Grid1"
    )


print("Grid1 获取成功")


# ============================================================
# 4. 获取 servo_motor_0
# ============================================================

print()
print("获取机械臂关节...")


servo0 = find_object_by_name(
    sim,
    "servo_motor_0"
)


if servo0 is None:

    raise RuntimeError(
        "找不到 servo_motor_0"
    )


print("servo_motor_0 获取成功")


# ============================================================
# 5. 获取 servo_motor_1
# ============================================================

servo1 = find_object_by_name(
    sim,
    "servo_motor_1"
)


if servo1 is None:

    raise RuntimeError(
        "找不到 servo_motor_1"
    )


print("servo_motor_1 获取成功")


# ============================================================
# 6. 获取夹爪关节
# ============================================================

print()
print("获取夹爪关节...")


gripper = find_object_by_name(
    sim,
    "Prismatic_joint"
)


if gripper is None:

    raise RuntimeError(
        "找不到 Prismatic_joint"
    )


print("Prismatic_joint 获取成功")


# ============================================================
# 7. 获取左夹爪
# ============================================================

print()
print("获取夹爪左右指...")


left_finger = find_object_by_name(
    sim,
    "left_gripper_5_visual"
)


if left_finger is None:

    raise RuntimeError(
        "找不到 left_gripper_5_visual"
    )


print("左夹爪获取成功")


# ============================================================
# 8. 获取右夹爪
# ============================================================

right_finger = find_object_by_name(
    sim,
    "right_gripper_5_visual"
)


if right_finger is None:

    raise RuntimeError(
        "找不到 right_gripper_5_visual"
    )


print("右夹爪获取成功")


# ============================================================
# 9. 初始位置
# ============================================================

print()
print("============================================================")
print("                 初始化场景")
print("============================================================")


robot_start = get_position(
    sim,
    robot
)


grid1_start = get_position(
    sim,
    grid1
)


print()
print("小车初始位置：")


print(
    f"X = {robot_start[0]:.3f}"
)

print(
    f"Y = {robot_start[1]:.3f}"
)

print(
    f"Z = {robot_start[2]:.3f}"
)


print()
print("Grid1 初始位置：")


print(
    f"X = {grid1_start[0]:.3f}"
)

print(
    f"Y = {grid1_start[1]:.3f}"
)

print(
    f"Z = {grid1_start[2]:.3f}"
)


# ============================================================
# 10. 启动仿真
# ============================================================

print()
print("启动仿真...")


try:

    sim.startSimulation()

except Exception:

    pass


time.sleep(1)


# ============================================================
# STEP 1
# 打开夹爪
# ============================================================

print()
print("============================================================")
print("       STEP 1：打开夹爪")
print("============================================================")


current_gripper = get_gripper_position(
    sim,
    gripper
)


print(
    f"夹爪："
    f"{current_gripper:.4f}"
    f" -> "
    f"{GRIPPER_OPEN:.4f}"
)


move_linear(

    current_gripper,

    GRIPPER_OPEN,

    GRIPPER_TIME,

    lambda v:
        set_gripper(
            sim,
            gripper,
            v
        )
)


time.sleep(0.5)


print(
    f"夹爪最终位置："
    f"{get_gripper_position(sim, gripper):.4f}"
)


# ============================================================
# STEP 2
# 机械臂下降
# ============================================================

print()
print("============================================================")
print("       STEP 2：机械臂下降")
print("============================================================")


servo0_start = get_joint_deg(
    sim,
    servo0
)


servo1_start = get_joint_deg(
    sim,
    servo1
)


print(
    f"servo0："
    f"{servo0_start:.2f}"
    f" -> "
    f"{ARM_HOME:.2f} deg"
)


print(
    f"servo1："
    f"{servo1_start:.2f}"
    f" -> "
    f"{ARM_LOWER:.2f} deg"
)


steps = max(
    1,
    int(ARM_TIME / 0.03)
)


for i in range(steps + 1):

    t = i / steps


    a0 = (

        servo0_start
        +
        (ARM_HOME - servo0_start) * t

    )


    a1 = (

        servo1_start
        +
        (ARM_LOWER - servo1_start) * t

    )


    set_joint_deg(
        sim,
        servo0,
        a0
    )


    set_joint_deg(
        sim,
        servo1,
        a1
    )


    time.sleep(
        ARM_TIME / steps
    )


# ============================================================
# STEP 3
# 检查抓取位置
# ============================================================

print()
print("============================================================")
print("                 抓取位置检查")
print("============================================================")


center = finger_center(
    sim,
    left_finger,
    right_finger
)


grid_pos = get_position(
    sim,
    grid1
)


print(
    f"夹爪中心："
    f"({center[0]:.3f}, "
    f"{center[1]:.3f}, "
    f"{center[2]:.3f})"
)


print(
    f"Grid1："
    f"({grid_pos[0]:.3f}, "
    f"{grid_pos[1]:.3f}, "
    f"{grid_pos[2]:.3f})"
)


# ============================================================
# STEP 4
# 闭合夹爪
# ============================================================

print()
print("============================================================")
print("       STEP 4：闭合夹爪")
print("============================================================")


current_gripper = get_gripper_position(
    sim,
    gripper
)


print(
    f"夹爪："
    f"{current_gripper:.4f}"
    f" -> "
    f"{GRIPPER_CLOSE:.4f}"
)


move_linear(

    current_gripper,

    GRIPPER_CLOSE,

    GRIPPER_TIME,

    lambda v:
        set_gripper(
            sim,
            gripper,
            v
        )
)


time.sleep(0.5)


print(
    f"夹爪最终位置："
    f"{get_gripper_position(sim, gripper):.4f}"
)


# ============================================================
# STEP 5
# 抓取
# ============================================================

print()
print("============================================================")
print("                    抓取 Grid1")
print("============================================================")


attach_grid1(

    sim,

    grid1,

    left_finger,

    right_finger
)


# ============================================================
# STEP 6
# 抬升机械臂
# ============================================================

print()
print("============================================================")
print("       STEP 6：抬升 Grid1")
print("============================================================")


servo0_start = get_joint_deg(
    sim,
    servo0
)


servo1_start = get_joint_deg(
    sim,
    servo1
)


print(
    f"servo0："
    f"{servo0_start:.2f}"
    f" -> "
    f"{ARM_HOME:.2f} deg"
)


print(
    f"servo1："
    f"{servo1_start:.2f}"
    f" -> "
    f"{ARM_LIFT:.2f} deg"
)


steps = max(
    1,
    int(ARM_TIME / 0.03)
)


for i in range(steps + 1):

    t = i / steps


    a0 = (

        servo0_start
        +
        (ARM_HOME - servo0_start) * t

    )


    a1 = (

        servo1_start
        +
        (ARM_LIFT - servo1_start) * t

    )


    set_joint_deg(
        sim,
        servo0,
        a0
    )


    set_joint_deg(
        sim,
        servo1,
        a1
    )


    # --------------------------------------------------------
    # 关键
    # 每一帧更新 Grid1
    # --------------------------------------------------------

    update_grid1(

        sim,

        grid1,

        left_finger,

        right_finger

    )


    time.sleep(
        ARM_TIME / steps
    )


# ============================================================
# STEP 7
# 保持抓取
# ============================================================

print()
print("============================================================")
print("       STEP 7：保持抓取")
print("============================================================")


steps = max(
    1,
    int(HOLD_TIME / 0.03)
)


for _ in range(steps):

    update_grid1(

        sim,

        grid1,

        left_finger,

        right_finger

    )

    time.sleep(0.03)


grid_pos = get_position(
    sim,
    grid1
)


center = finger_center(
    sim,
    left_finger,
    right_finger
)


print()
print("当前状态：")


print(
    f"夹爪中心："
    f"({center[0]:.3f}, "
    f"{center[1]:.3f}, "
    f"{center[2]:.3f})"
)


print(
    f"Grid1："
    f"({grid_pos[0]:.3f}, "
    f"{grid_pos[1]:.3f}, "
    f"{grid_pos[2]:.3f})"
)


# ============================================================
# STEP 8
# 松开夹爪
# ============================================================

print()
print("============================================================")
print("       STEP 8：松开夹爪")
print("============================================================")


release_grid1()


current_gripper = get_gripper_position(
    sim,
    gripper
)


print(
    f"夹爪："
    f"{current_gripper:.4f}"
    f" -> "
    f"{GRIPPER_OPEN:.4f}"
)


move_linear(

    current_gripper,

    GRIPPER_OPEN,

    GRIPPER_TIME,

    lambda v:
        set_gripper(
            sim,
            gripper,
            v
        )
)


time.sleep(0.5)


print(
    f"夹爪最终位置："
    f"{get_gripper_position(sim, gripper):.4f}"
)


# ============================================================
# STEP 9
# 最终状态
# ============================================================

print()
print("============================================================")
print("                 测试完成")
print("============================================================")


grid_final = get_position(
    sim,
    grid1
)


print()
print("Grid1 最终位置：")


print(
    f"X = {grid_final[0]:.3f}"
)


print(
    f"Y = {grid_final[1]:.3f}"
)


print(
    f"Z = {grid_final[2]:.3f}"
)


print()
print("夹爪最终状态：")


print(
    f"Prismatic_joint = "
    f"{get_gripper_position(sim, gripper):.4f}"
)


print()
print("============================================================")
print("          Grid1 抓取测试结束")
print("============================================================")


print()
print("CoppeliaSim 仿真保持运行。")
print("你可以直接观察机械臂和夹爪。")