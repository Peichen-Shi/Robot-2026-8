from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


print()
print("==============================================")
print("        RoboMaster EP 夹爪开关测试")
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
# 2. 获取夹爪 Prismatic_joint
# ============================================================

print("获取 Prismatic_joint...")

try:
    gripper = sim.getObject("/RoboMaster/**/Prismatic_joint")
except Exception:
    gripper = -1


if gripper < 0:
    print()
    print("❌ 找不到 Prismatic_joint")
    print()
    print("请检查场景中是否存在：")
    print("Prismatic_joint")
    print()
    input("按 Enter 退出...")
    exit()


print("夹爪关节获取成功")
print("Handle =", gripper)
print()


# ============================================================
# 3. 参数
# ============================================================

GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = -0.025


# ============================================================
# 4. 控制函数
# ============================================================

def set_gripper(position):

    print()
    print("----------------------------------------------")

    if position > 0:
        print("执行：打开夹爪")
    else:
        print("执行：闭合夹爪")

    print("目标位置 =", position)

    # 和原来成功 Lua 完全一致
    sim.setJointPosition(
        gripper,
        position
    )

    sim.setJointTargetPosition(
        gripper,
        position
    )

    time.sleep(2)

    # 读取实际位置
    try:
        actual = sim.getJointPosition(gripper)

        print("实际位置 =", actual)

    except Exception as e:

        print("读取夹爪位置失败：", e)

    print("----------------------------------------------")


# ============================================================
# 5. 先打开
# ============================================================

print("初始：打开夹爪")

set_gripper(GRIPPER_OPEN)


# ============================================================
# 6. 闭合
# ============================================================

print()
print("等待 2 秒...")

time.sleep(2)

print()
print("测试：闭合夹爪")

set_gripper(GRIPPER_CLOSE)


# ============================================================
# 7. 再打开
# ============================================================

print()
print("等待 2 秒...")

time.sleep(2)

print()
print("测试：重新打开夹爪")

set_gripper(GRIPPER_OPEN)


# ============================================================
# 8. 循环测试
# ============================================================

print()
print("==============================================")
print("       开始连续测试夹爪")
print("==============================================")
print()

for i in range(3):

    print("第 %d 次" % (i + 1))

    print("→ 打开")
    set_gripper(GRIPPER_OPEN)

    time.sleep(1)

    print("→ 闭合")
    set_gripper(GRIPPER_CLOSE)

    time.sleep(1)


# ============================================================
# 9. 最后保持打开
# ============================================================

set_gripper(GRIPPER_OPEN)


print()
print("==============================================")
print("             夹爪测试完成")
print("==============================================")
print()

print("如果夹爪正常：")
print("✔ 0.05  = 打开")
print("✔ -0.025 = 闭合")
print()

print("如果控制台数值发生变化，但是夹爪外观不动：")
print("说明 Prismatic_joint 被其他控制脚本/动力学约束覆盖。")
print()

input("按 Enter 退出...")