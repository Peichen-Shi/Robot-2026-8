from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


# ============================================================
# CoppeliaSim RoboMaster EP
# 夹爪单独测试
#
# 流程：
# 1. 连接 CoppeliaSim
# 2. 获取 Prismatic_joint
# 3. 夹爪打开
# 4. 等待
# 5. 夹爪关闭
# 6. 等待
# 7. 夹爪再次打开
#
# 不控制：
# - 小车
# - 机械臂
# - Grid1
# ============================================================


HOST = "localhost"
PORT = 23000

# 根据你之前成功的 Lua 代码
GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = -0.025


# ============================================================
# 连接
# ============================================================

print()
print("=" * 60)
print("              RoboMaster EP 夹爪测试")
print("=" * 60)

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
# 获取夹爪关节
# ============================================================

print()
print("获取 Prismatic_joint...")

try:
    gripper = sim.getObject("/Prismatic_joint")
    print("Prismatic_joint 获取成功")
except Exception as e:
    print()
    print("获取夹爪失败！")
    print(e)
    print()
    print("请检查 CoppeliaSim 中夹爪关节的名称是不是：")
    print("Prismatic_joint")
    raise


# ============================================================
# 工具函数
# ============================================================

def get_gripper_position():
    try:
        return sim.getJointPosition(gripper)
    except Exception:
        return None


def set_gripper(value):

    # 和之前成功的 Lua 代码保持一致
    try:
        sim.setJointPosition(
            gripper,
            value
        )
    except Exception as e:
        print("setJointPosition 错误：", e)

    try:
        sim.setJointTargetPosition(
            gripper,
            value
        )
    except Exception as e:
        print("setJointTargetPosition 错误：", e)


def move_gripper(start, target, duration):

    steps = max(
        1,
        int(duration / 0.03)
    )

    for i in range(steps + 1):

        t = i / steps

        value = start + (
            target - start
        ) * t

        set_gripper(value)

        time.sleep(
            duration / steps
        )


# ============================================================
# 当前状态
# ============================================================

current = get_gripper_position()

print()
print("=" * 60)
print("当前夹爪状态")
print("=" * 60)

print(
    f"Prismatic_joint = {current:.4f}"
)


# ============================================================
# STEP 1
# 打开夹爪
# ============================================================

print()
print("=" * 60)
print("STEP 1：打开夹爪")
print("=" * 60)

current = get_gripper_position()

print(
    f"夹爪：{current:.4f}"
    f" -> "
    f"{GRIPPER_OPEN:.4f}"
)

move_gripper(
    current,
    GRIPPER_OPEN,
    2.0
)

time.sleep(1)

current = get_gripper_position()

print(
    f"打开完成：{current:.4f}"
)


# ============================================================
# STEP 2
# 关闭夹爪
# ============================================================

print()
print("=" * 60)
print("STEP 2：关闭夹爪")
print("=" * 60)

current = get_gripper_position()

print(
    f"夹爪：{current:.4f}"
    f" -> "
    f"{GRIPPER_CLOSE:.4f}"
)

move_gripper(
    current,
    GRIPPER_CLOSE,
    2.0
)

time.sleep(1)

current = get_gripper_position()

print(
    f"关闭完成：{current:.4f}"
)


# ============================================================
# STEP 3
# 再次打开夹爪
# ============================================================

print()
print("=" * 60)
print("STEP 3：再次打开夹爪")
print("=" * 60)

current = get_gripper_position()

print(
    f"夹爪：{current:.4f}"
    f" -> "
    f"{GRIPPER_OPEN:.4f}"
)

move_gripper(
    current,
    GRIPPER_OPEN,
    2.0
)

time.sleep(1)

current = get_gripper_position()

print(
    f"再次打开完成：{current:.4f}"
)


# ============================================================
# 最终结果
# ============================================================

print()
print("=" * 60)
print("                 测试结束")
print("=" * 60)

current = get_gripper_position()

print()
print(
    f"最终 Prismatic_joint = {current:.4f}"
)

print()
print("预期：")
print(
    f"打开 ≈ {GRIPPER_OPEN:.3f}"
)

print(
    f"关闭 ≈ {GRIPPER_CLOSE:.3f}"
)

print()
print("=" * 60)